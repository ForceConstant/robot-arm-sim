#!/usr/bin/env python3
"""Grasp with a recorded, annotated, slow-motion close (2026-11-23).

Brian's notes on the previous gif:
  1. more frames at the moment of grasping -> the jaws close in small steps with
     a pause on each, so the 4.5 fps wrist stream yields ~20 frames of the close
     instead of three;
  2. show the piece-centre target and the jaw-tip centre -> the console overlay
     draws the commanded aim (magenta) and the stored tip pixel (cyan); this
     tool posts /aim before it moves so both are baked into the recorded frames;
  3. the piece was not centred in the jaw before the close -> the tool measures
     the piece centroid against the FK tip at grasp height and prints the
     residual, and refuses to call a close "centred" if it is out of tolerance.

Recording is the console's annotated /stream (portrait 480x640), so the gif
carries the same overlay the dashboard shows. All computation on the BC-250.

    PYTHONPATH=. .venv/bin/python tools/p4_grasp_capture.py \
        --piece pawn --square e5 --execute --video /tmp/grasp_capture.mp4
"""
from __future__ import annotations
import argparse, base64, json, sys, threading, time, urllib.error, urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / 'tools'))
from infer_base_xy import load_model, predict                  # noqa: E402
from p4_irl_servo_detect import grab_frame                     # noqa: E402

C = 'http://192.168.1.171:8080'
B = 'http://192.168.1.167:8100'
CALIBRATION = ROOT / '.private/phase11d/calibration.json'
REFERENCE = ROOT / 'chess_sim/assets/scene/generated/board_charuco.png'


def get(url, t=10):
    with urllib.request.urlopen(url, timeout=t) as r:
        return json.load(r)


def post(path, body, t=90):
    req = urllib.request.Request(C + path, data=json.dumps(body).encode(),
                                 method='POST', headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=t) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return {'http_error': e.code, 'detail': e.read().decode()[:250]}


def state():
    return get(B + '/state')


def tip():
    return np.array(get(C + '/keep-in')['tip_mm'], float)


def raw_frame(t=12):
    d = get(B + '/capture', t)
    arr = np.frombuffer(base64.b64decode(d['jpeg_b64']), np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


class Recorder:
    """Drain the console's annotated MJPEG stream into an mp4.

    Uses OpenCV's own MJPEG reader: a hand-rolled urllib parser only ever got
    partial JPEGs off this stream (measured 2026-11-23: 0 frames in 6 s, while
    cv2.VideoCapture got 27 in the same window).
    """

    def __init__(self, path, fps=8.0):
        self.path = path; self.fps = fps; self.stop = False; self.frames = 0
        self.cap = None; self.writer = None; self.size = None
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            self.cap = cv2.VideoCapture(C + '/stream')
            if not self.cap.isOpened():
                self.error = 'stream not opened'
                return
            while not self.stop:
                ok, img = self.cap.read()
                if not ok or img is None:
                    self.error = 'stream read failed'
                    break
                if self.size is None:
                    h, w = img.shape[:2]
                    self.size = (w, h)
                    self.writer = cv2.VideoWriter(
                        str(self.path), cv2.VideoWriter_fourcc(*'mp4v'),
                        self.fps, (w, h))
                self.writer.write(img)
                self.frames += 1
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'

    def stop_rec(self):
        self.stop = True
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        time.sleep(0.4)
        if self.writer:
            self.writer.release()
        if self.error:
            print(f'  recorder: {self.error}', flush=True)
        return self.frames


class Guard:
    def __init__(self):
        st = state()
        self.roll0 = st['joints']['wrist_roll']

    def check(self, what):
        st = state()
        if st['fault'] or st['mode'] != 'REMOTE' or abs(st['joints']['wrist_roll'] - self.roll0) > 3:
            print(f'  ABORT ({what}): mode={st["mode"]} fault={st["fault"]} '
                  f'roll={st["joints"]["wrist_roll"]:.2f}', flush=True)
            return False
        return True


def move_leg(body, guard, what, dry_limit=15.0):
    """Dry-run, print, execute, then guard. True on success."""
    dr = post('/move', {**body, 'dry_run': True}, t=25)
    if dr.get('http_error'):
        print(f'LEG {what}: dry-run REFUSED {dr.get("detail")}', flush=True)
        return False
    delta = dr.get('max_joint_delta_deg')
    print(f"LEG {what}: {json.dumps(body)}  perr={dr.get('requested', {}).get('position_error_mm')}"
          f"mm maxdelta={delta}", flush=True)
    if delta is not None and delta > dry_limit:
        print('  dry-run delta too large - skipped', flush=True)
        return False
    r = post('/move', body, t=90)
    if r.get('http_error'):
        print('  exec REFUSED', r.get('detail'), flush=True)
        return False
    time.sleep(1.0)
    return guard.check(what)


def move_big(body, guard, what, limit=30.0, splits=4):
    """Execute an absolute xyz leg, splitting it when the joint change is large.

    A refused dry-run used to just skip the leg, and the run then carried on
    from the wrong place (2026-11-23). Splitting keeps the target while holding
    each leg inside the limit, and a leg that still cannot run aborts the run.
    """
    dr = post('/move', {**body, 'dry_run': True}, t=25)
    if dr.get('http_error'):
        print(f'LEG {what}: dry REFUSED {dr.get("detail")}', flush=True)
        return False
    delta = dr.get('max_joint_delta_deg')
    if delta is not None and delta > limit and splits > 0:
        cur = tip()
        mid = {'mode': 'xyz',
               'x_mm': round(float(cur[0] + (body['x_mm'] - cur[0]) * 0.5), 1),
               'y_mm': round(float(cur[1] + (body['y_mm'] - cur[1]) * 0.5), 1),
               'z_mm': round(float(cur[2] + (body['z_mm'] - cur[2]) * 0.5), 1)}
        print(f'LEG {what}: delta {delta} > {limit}, splitting at {mid}', flush=True)
        if not move_big(mid, guard, f'{what} (a)', limit, splits - 1):
            return False
        return move_big(body, guard, f'{what} (b)', limit, splits - 1)
    return move_leg(body, guard, what, dry_limit=limit)


def measure_centring(detector, xy, z_grasp):
    """Piece centroid vs FK tip, in board mm, from one raw frame."""
    try:
        frame = raw_frame()
    except Exception as exc:
        return {'error': f'frame: {exc}'}
    res = detector.detect(frame, aim_xy=(xy[0], xy[1], z_grasp, 'aim'))
    piece = res.get('piece') or {}
    t = tip()
    out = {'status': res.get('status'), 'markers': len(res.get('markers', [])),
           'tip_mm': [round(float(v), 1) for v in t]}
    if piece.get('x_mm') is None:
        out['piece'] = None
        out['reason'] = res.get('reason')
        return out
    dx = float(piece['x_mm']) - float(t[0])
    dy = float(piece['y_mm']) - float(t[1])
    out.update({'piece': {'square': piece.get('square'),
                          'x_mm': piece.get('x_mm'), 'y_mm': piece.get('y_mm'),
                          'uv': piece.get('uv')},
                'delta_mm': [round(dx, 2), round(dy, 2)],
                'abs_mm': round(float(np.hypot(dx, dy)), 2),
                'tip_uv': (get(C + '/tip-center') or {}).get('pixel')})
    return out


def image_gate(target_xy, tol_mm=6.0):
    """Are the jaws actually over the target square, in the image?

    FK says the tip is on target; the camera says where the jaws are. Unproject
    the stored tip-centre pixel at the tip's current height and compare that
    board point with the target square centre. This is the check that was
    missing when a run descended and closed on air with FK reporting 0.5 mm
    (2026-11-23).
    """
    try:
        from chess_sim.board_target import Board
        import base64
        board = Board(json.loads(CALIBRATION.read_text()))
        d = get(B + '/capture', 15)
        arr = np.frombuffer(base64.b64decode(d['jpeg_b64']), np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        sol, dets = board.solve(board.rectify(img))
        if sol is None:
            return False, {'error': f'no PnP ({len(dets)} markers)'}
        tip_px = (get(C + '/tip-center') or {}).get('pixel')
        if not tip_px:
            return False, {'error': 'no stored tip centre'}
        tip_z = float(get(C + '/keep-in')['tip_mm'][2])
        tip_xy = board.unproject(sol, tip_px, tip_z)
        dx = float(target_xy[0] - tip_xy[0])
        dy = float(target_xy[1] - tip_xy[1])
        sep = float(np.hypot(dx, dy))
        info = {'tip_px': tip_px, 'tip_z_mm': round(tip_z, 1),
                'tip_board_mm': [round(float(tip_xy[0]), 1), round(float(tip_xy[1]), 1)],
                'target_mm': [float(target_xy[0]), float(target_xy[1])],
                'delta_mm': [round(dx, 2), round(dy, 2)], 'sep_mm': round(sep, 2),
                'markers': len(dets), 'rms_px': round(float(sol['rms_px']), 2)}
        return sep <= tol_mm, info
    except Exception as exc:
        return False, {'error': f'{type(exc).__name__}: {exc}'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--piece', required=True)
    ap.add_argument('--square', required=True)
    ap.add_argument('--z-safe', type=float, default=70.0)
    ap.add_argument('--z-grasp', type=float, default=15.0)
    ap.add_argument('--lift-z', type=float, default=60.0)
    ap.add_argument('--jaw-close', type=float, default=13.0,
                    help='jaw value the close stops at; 13 is ~5 mm gap, too narrow '
                         'when a piece sits a few mm off centre')
    ap.add_argument('--place-square', default=None,
                    help='after the pick, carry to this square, lower, release and retreat')
    ap.add_argument('--video', default='/tmp/grasp_capture.mp4')
    ap.add_argument('--aim', choices=('square', 'model', 'measured'), default='square',
                    help='square = nominal square centre via cartesian control '
                         '(all 64 squares are reachable - the model\'s six-square '
                         'table is only its training set); model = type-known '
                         'base-XY model; measured = detector centroid')
    ap.add_argument('--z-obs', type=float, default=40.0,
                    help='height for the centring measurement (markers still visible)')
    ap.add_argument('--correct', action='store_true',
                    help='apply one small board-axis correction if the measured offset exceeds 2.5 mm')
    ap.add_argument('--execute', action='store_true')
    a = ap.parse_args()

    st = state()
    print(f'START mode={st["mode"]} armed={st["armed"]} fault={st["fault"]} '
          f'jaw={st["joints"]["gripper"]:.1f} roll={st["joints"]["wrist_roll"]:.1f}', flush=True)
    if st['fault']:
        raise SystemExit('bridge fault')

    from chess_sim.board_target import TargetDetector
    detector = None
    try:
        detector = TargetDetector(str(CALIBRATION), str(REFERENCE))
    except Exception as exc:
        print(f'detector unavailable: {exc}', flush=True)

    frame = grab_frame(timeout=8)
    if a.aim == 'square':
        col = ord(a.square[0].lower()) - 97
        rank = int(a.square[1])
        xy = ((col + 0.5) * 28.0 - 112.0, (rank - 0.5) * 28.0 - 112.0)
        info = {'source': 'cartesian square centre'}
    elif a.aim == 'measured':
        if detector is None:
            raise SystemExit('measured aim needs the detector')
        res = detector.detect(frame)
        piece = res.get('piece') or {}
        xy = None
        if piece.get('x_mm') is not None:
            xy = (piece['x_mm'], piece['y_mm'])
        info = {'source': 'detector', 'status': res.get('status'),
                'markers': len(res.get('markers', [])), 'reason': res.get('reason')}
    else:
        model = load_model()
        xy, info = predict(frame, a.piece.lower(), a.square.lower(), model)
    print(f'PREDICT {a.piece}@{a.square} via {a.aim}: {xy} ({info})', flush=True)
    if xy is None:
        raise SystemExit('no prediction')

    target = (round(float(xy[0]), 1), round(float(xy[1]), 1))
    print(f'AIM {target} -- posting to the console overlay', flush=True)
    post('/aim', {'x_mm': target[0], 'y_mm': target[1], 'z_mm': a.z_grasp,
                  'label': f'{a.piece}@{a.square}'})
    if not a.execute:
        dr = post('/move', {'mode': 'xyz', 'x_mm': target[0], 'y_mm': target[1],
                            'z_mm': a.z_safe, 'dry_run': True}, t=25)
        print('DRY-RUN ONLY. plan dry-run:', dr.get('detail') or 'OK', flush=True)
        return

    post('/mode', {'mode': 'REMOTE'}); time.sleep(1.0)
    guard = Guard()

    rec = Recorder(a.video); rec.start(); time.sleep(0.8)

    # 1) servo the tip over the predicted base at safe height
    # Establish an observation height first: a previous abort can leave the tip
    # low, where only one marker is visible and the servo starts blind.
    if float(tip()[2]) < a.z_safe - 5.0:
        print(f'LIFT to observation height {a.z_safe}', flush=True)
        if not move_big({'mode': 'xyz', 'x_mm': target[0], 'y_mm': target[1],
                         'z_mm': a.z_safe}, guard, 'observe', limit=30.0):
            print('ABORT: could not reach the observation height', flush=True)
            n = rec.stop_rec(); post('/mode', {'mode': 'HOLD'})
            print(f'VIDEO {a.video} frames={n}; bridge HOLD', flush=True)
            return

    # SERVO ON THE IMAGE, not on FK. FK and the camera disagreed by 26 mm at the
    # parked pose (2026-11-23) - driving FK onto the target put the jaws a square
    # away and the run closed on air. Each step measures where the jaws really
    # are (stored tip pixel unprojected at the tip height), adds that error to
    # the current FK position, and moves.
    # Measured 2026-11-23: the arm ignores commands under ~2-3 mm and achieves
    # only 0.6-0.75 of what it is asked, so a proportional servo creeps and
    # stalls. Compensate the gain and stop when movement itself stops.
    # Measured 2026-11-23 at (10,10,70): commanding +8 mm in x achieves only
    # +2.7 mm in x, and the axes cross-talk. Command through the inverse of that
    # matrix (damped) instead of a scalar gain.
    # A fixed inverse measured at one pose oscillated at another (2026-11-23),
    # so command the residual directly and let the image feedback iterate.
    J_INV = np.array([[1.0, 0.0], [0.0, 1.0]], float)
    sag = a.z_safe - float(tip()[2])      # one measurement, reused as the
                                          # standoff compensation
    last_sep = None
    last_move = None
    takeups = 0
    reached = False
    for step in range(12):
        ok_img, info = image_gate(target, tol_mm=3.0)
        print(f"  image servo {step}: {json.dumps(info)}", flush=True)
        if ok_img:
            reached = True
            break
        if 'delta_mm' not in info:
            print('servo aborted: no image measurement', flush=True)
            break
        if last_move is not None and last_move < 1.0:
            # The mechanism ignores small steps (measured: 2 mm -> 0 mm, and only
            # 0.6-0.75 of anything larger). Take up the slack by deliberately
            # overshooting along the error, then approach the target from that
            # side so the backlash is pre-loaded the same way every time.
            if takeups < 2:
                takeups += 1
                dx0, dy0 = info['delta_mm']
                n = float(np.hypot(dx0, dy0)) or 1.0
                cur0 = tip()
                ox = round(float(cur0[0] + dx0 / n * 8.0), 1)
                oy = round(float(cur0[1] + dy0 / n * 8.0), 1)
                print(f'  backlash take-up {takeups}: overshoot to ({ox},{oy})', flush=True)
                if not move_big({'mode': 'xyz', 'x_mm': ox, 'y_mm': oy,
                                 'z_mm': round(float(a.z_safe + sag), 1)},
                                guard, f'takeup {takeups}', limit=30.0):
                    break
                last_move = None
                continue
            print(f'  deadband persists after {takeups} take-ups '
                  f'({last_move:.2f}mm on the last leg)', flush=True)
            break
        last_sep = info['sep_mm']
        dx, dy = info['delta_mm']
        cur = tip()
        before = cur.copy()
        dvec = J_INV @ np.array([dx, dy], float)
        dx, dy = float(dvec[0]), float(dvec[1])
        # hold the current z: asking for z_safe + sag made the arm fight its own
        # gravity each step and the whole command stalled (2026-11-23)
        # hold the STANDOFF, not the readback: holding readback let the tip
        # ratchet down 60.8 -> 45.7 mm over four steps and the pose lost markers
        cmd = (round(float(cur[0] + dx), 1), round(float(cur[1] + dy), 1),
               round(float(a.z_safe + sag), 1))
        if not move_big({'mode': 'xyz', 'x_mm': cmd[0], 'y_mm': cmd[1],
                         'z_mm': cmd[2]}, guard, f'servo {step}', limit=30.0):
            print('servo aborted', flush=True)
            break
        last_move = float(np.hypot(*(tip()[:2] - before[:2])))
        sag = a.z_safe - float(tip()[2])
    if not reached:
        ok_img, info = image_gate(target, tol_mm=4.5)
        print('FINAL IMAGE CHECK', json.dumps(info), flush=True)
        if not ok_img:
            print('ABORT: the jaws are not over the target square in the image; '
                  'nothing closed', flush=True)
            n = rec.stop_rec()
            post('/mode', {'mode': 'HOLD'})
            print(f'VIDEO {a.video} frames={n}; bridge HOLD', flush=True)
            return

    # 1b) centring check at an observation height where markers still solve
    if detector:
        obs = measure_centring(detector, target, a.z_obs)
        obs['at_mm'] = round(float(tip()[2]), 1)
        print('CENTRING @obs', json.dumps(obs), flush=True)
        # correct here, at a height where the detector still has a pose; the
        # grasp-height view is marker-poor and cannot be measured reliably
        if (a.correct and obs.get('delta_mm')
                and 2.5 < obs.get('abs_mm', 99) < 12):
            dx, dy = obs['delta_mm']
            aim_x = round(float(target[0]) + dx, 1)
            aim_y = round(float(target[1]) + dy, 1)
            print(f'CORRECT to ({aim_x},{aim_y}) mm (delta {dx:.2f},{dy:.2f})', flush=True)
            # absolute xyz, not rel: rel dx/dy is in the tool frame and moved
            # the tip the wrong way on 2026-11-23
            if move_leg({'mode': 'xyz', 'x_mm': aim_x, 'y_mm': aim_y,
                         'z_mm': round(float(tip()[2]), 1)}, guard, 'centre',
                        dry_limit=30.0):
                target = [aim_x, aim_y]
                post('/aim', {'x_mm': aim_x, 'y_mm': aim_y, 'z_mm': a.z_grasp,
                              'label': f'{a.piece}@{a.square}'})
                obs2 = measure_centring(detector, target, a.z_obs)
                print('CENTRING @obs after correction', json.dumps(obs2), flush=True)

    # 2) descend to grasp height in 5 mm steps
    for k in range(16):
        cur = tip()
        if cur[2] <= a.z_grasp + 1:
            break
        z = max(a.z_grasp, float(cur[2]) - 5.0)
        # absolute xyz: a rel dz is in the TOOL frame, which walked the tip
        # ~50 mm sideways at this wrist pitch on 2026-11-23
        if not move_leg({'mode': 'xyz', 'x_mm': target[0], 'y_mm': target[1],
                         'z_mm': round(z, 1)}, guard, f'descend {k}', dry_limit=30.0):
            print('descend stopped', flush=True)
            break

    # 3) centring measurement before the close
    centring = measure_centring(detector, target, a.z_grasp) if detector else {'error': 'no detector'}
    print('CENTRING', json.dumps(centring), flush=True)

    # 4) slow close: many frames of the grasp moment
    print('SLOW CLOSE', flush=True)
    for jaw in np.linspace(70.0, a.jaw_close, 11):
        r = post('/move', {'mode': 'joint', 'jaw': float(jaw)}, t=40)
        if r.get('http_error'):
            print('  jaw step refused', r.get('detail'), flush=True)
            break
        print(f'  jaw -> {jaw}', flush=True)
        time.sleep(0.45)
    time.sleep(0.6)

    # 5) slow lift
    print('SLOW LIFT', flush=True)
    # absolute levels, not readback+6: the tip hangs ~8 mm below the command,
    # so stepping from the readback never actually rises (2026-11-23)
    for z in (30.0, 45.0, a.lift_z):
        if float(tip()[2]) >= z - 1:
            continue
        if not move_leg({'mode': 'xyz', 'x_mm': target[0], 'y_mm': target[1],
                         'z_mm': round(z, 1)}, guard, 'lift', dry_limit=30.0):
            break
        time.sleep(0.3)

    # 6) place, if a destination was given
    if a.place_square:
        col = ord(a.place_square[0].lower()) - 97
        rank = int(a.place_square[1])
        px = round((col + 0.5) * 28.0 - 112.0, 1)
        py = round((rank - 0.5) * 28.0 - 112.0, 1)
        print(f'PLACE -> {a.place_square} ({px},{py})', flush=True)
        post('/aim', {'x_mm': px, 'y_mm': py, 'z_mm': a.z_grasp,
                      'label': f'place {a.place_square}'})
        # carry across at safe height, splitting large moves
        if move_big({'mode': 'xyz', 'x_mm': px, 'y_mm': py, 'z_mm': a.z_safe},
                    guard, 'carry', limit=30.0):
            # descend in 5 mm steps
            for k in range(16):
                cur = tip()
                if cur[2] <= a.z_grasp + 1:
                    break
                z = max(a.z_grasp, float(cur[2]) - 5.0)
                if not move_leg({'mode': 'xyz', 'x_mm': px, 'y_mm': py,
                                 'z_mm': round(z, 1)}, guard, f'place descend {k}',
                                dry_limit=30.0):
                    print('place descend stopped', flush=True)
                    break
            # open slowly so the release is visible
            for jaw in (18, 25, 35, 50, 70):
                post('/move', {'mode': 'joint', 'jaw': float(jaw)}, t=40)
                time.sleep(0.35)
            time.sleep(0.5)
            # retreat
            for z in (30.0, 45.0, a.lift_z):
                if float(tip()[2]) >= z - 1:
                    continue
                if not move_leg({'mode': 'xyz', 'x_mm': px, 'y_mm': py,
                                 'z_mm': round(z, 1)}, guard, 'place lift',
                                dry_limit=30.0):
                    break
                time.sleep(0.3)

    fin = state()
    print(f'FINAL tip {tip().round(1)} jaw {fin["joints"]["gripper"]:.1f} '
          f'fault {fin["fault"]}', flush=True)
    n = rec.stop_rec()
    post('/mode', {'mode': 'HOLD'})
    print(f'VIDEO {a.video} frames={n}; bridge HOLD', flush=True)


if __name__ == '__main__':
    main()
