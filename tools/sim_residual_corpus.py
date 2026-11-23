#!/usr/bin/env python3
"""Residual corpus: fixed camera per square, empty-render background, exact labels.

Why this shape:
  * the camera is fixed relative to the SQUARE, not the piece, so the piece's
    offset inside the square is the only thing that moves in the crop;
  * the background is an empty render of the same scene from the same camera,
    so |frame - empty| has zero registration error (the printed-face reference
    is a different image from the board the sim renders - corr -0.676);
  * labels are the placed contact point, converted into the marker-layout frame
    the rectified crop lives in.

    PYTHONPATH=. MUJOCO_GL=egl .venv/bin/python tools/sim_residual_corpus.py \
        --out .private/sim_corpus/resid_grid --squares 64 --offsets 4
"""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import numpy as np, cv2

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT))
import mujoco                                              # noqa: E402
from chess_sim import scene as sim_scene                    # noqa: E402
from chess_sim.config import AppearanceConfig, BoardConfig  # noqa: E402
from chess_sim.board_target import Board                    # noqa: E402

SQ_MM = 28.0
LAYOUT_T = (4.9, -1.9)          # board-body -> marker-layout, measured
BOARD_Z = 0.4444
PPMM, WINDOW_MM, OUT_PX = 8.0, 40.0, 96
CAM_STAND, CAM_HEIGHT = 0.075, 0.150
FOVY = 47.0
PIECES = {'pawn': 'white_pawn1', 'knight': 'white_knight1', 'bishop': 'white_bishop1',
          'rook': 'white_rook1', 'queen': 'white_queen1', 'king': 'white_king1'}


def square_xy(name):
    return ((ord(name[0]) - 97 + 0.5) * SQ_MM - 112.0,
            (int(name[1]) - 0.5) * SQ_MM - 112.0)


def lookat_xyaxes(pos, target, up=(0.0, 0.0, 1.0)):
    z = np.asarray(pos, float) - np.asarray(target, float)
    z /= np.linalg.norm(z)
    x = np.cross(np.asarray(up, float), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return x, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='.private/sim_corpus/resid_grid')
    ap.add_argument('--squares', type=int, default=64)
    ap.add_argument('--offsets', type=int, default=4)
    ap.add_argument('--pieces', default='pawn,king,queen,rook')
    ap.add_argument('--offset-mm', type=float, default=11.0)
    ap.add_argument('--seed', type=int, default=5)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    board = Board(json.loads((ROOT / '.private/phase11d/calibration.json').read_text()))

    spec = sim_scene.build_scene(BoardConfig(), AppearanceConfig(piece_set='br_v4'))
    spec.worldbody.add_camera(name='probe', pos=[0.0, 0.0, 0.15],
                              xyaxes=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0], fovy=FOVY)
    model = spec.compile()
    data = mujoco.MjData(model)
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'probe')
    joints = {}
    for i in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if nm and nm.endswith('_joint') and not nm.startswith('so101'):
            joints[nm] = model.jnt_qposadr[i]
    renderer = mujoco.Renderer(model, 480, 640)

    def park():
        for idx, (nm, adr) in enumerate(joints.items()):
            data.qpos[adr:adr + 3] = [2.0 + 0.4 * (idx % 12), 2.0 + 0.4 * (idx // 12), 0.6]
            data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]

    def place(joint, xy_mm):
        adr = joints[joint + '_joint']
        data.qpos[adr:adr + 3] = [xy_mm[0] / 1000.0, xy_mm[1] / 1000.0, BOARD_Z]
        data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]

    def shoot():
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera='probe')
        return cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

    squares = [f'{f}{r}' for f in 'abcdefgh' for r in '12345678'][:args.squares]
    pieces = [p.strip() for p in args.pieces.split(',') if p.strip()]
    X, Y, SQ, PC = [], [], [], []

    for square in squares:
        sx_b, sy_b = square_xy(square)
        cam = [sx_b / 1000.0 + CAM_STAND, sy_b / 1000.0, BOARD_Z + CAM_HEIGHT]
        tgt = [sx_b / 1000.0, sy_b / 1000.0, BOARD_Z]
        x_, y_ = lookat_xyaxes(cam, tgt)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.column_stack([x_, y_, np.cross(x_, y_)]).reshape(-1))
        model.cam_pos[cid] = cam
        model.cam_quat[cid] = q

        park()
        empty = shoot()
        rect = board.rectify(empty)
        sol, _ = board.solve(rect)
        if sol is None:
            print(f'  {square}: empty render did not solve - skipped'); continue
        grey = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
        top_empty, valid, n, pxm = board.rectified(grey, sol, PPMM)
        sx, sy = -sx_b + LAYOUT_T[0], -sy_b + LAYOUT_T[1]
        cx, cy = int(n / 2 + sx * pxm), int(n / 2 - sy * pxm)
        half = int(round(WINDOW_MM * pxm / 2))

        for piece in pieces:
            for _ in range(args.offsets):
                ox = float(rng.uniform(-args.offset_mm, args.offset_mm))
                oy = float(rng.uniform(-args.offset_mm, args.offset_mm))
                park()
                place(PIECES[piece], (sx_b + ox, sy_b + oy))
                img = shoot()
                r2 = board.rectify(img)
                g2 = cv2.cvtColor(r2, cv2.COLOR_BGR2GRAY)
                top2, _, _, _ = board.rectified(g2, sol, PPMM)
                res = cv2.absdiff(top2, top_empty).astype(np.float32)
                res[valid == 0] = 0
                win = res[max(0, cy - half):cy + half, max(0, cx - half):cx + half]
                if win.shape[0] < 8 or win.shape[1] < 8:
                    continue
                patch = cv2.resize(win, (OUT_PX, OUT_PX), interpolation=cv2.INTER_AREA) / 255.0
                X.append(patch); Y.append([-ox, -oy])     # layout-frame offset
                SQ.append(square); PC.append(piece)
        print(f'  {square}: {len(X)} patches so far', flush=True)

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / 'patches.npz', x=np.stack(X), y=np.array(Y, np.float32),
                        square=np.array(SQ), piece=np.array(PC))
    print(f'wrote {out}/patches.npz  {len(X)} patches, {len(squares)} squares, {len(pieces)} pieces')


if __name__ == '__main__':
    sys.exit(main())
