#!/usr/bin/env python3
"""Keypoint corpus: exact base-contact pixel labels + randomised lighting.

Follows the design brief's recipe:
  * label = the piece's base-contact point projected through the *same* camera
    used to render it (sim ground truth, never an image heuristic);
  * lighting direction/intensity randomised per frame, so a square's shadow
    signature cannot become a learned shortcut;
  * camera height/standoff/bearing varied inside the real operating envelope;
  * a square-centred crop is stored with the keypoint in crop pixels, so the
    model predicts a pixel (translation-equivariant) and geometry does the rest.

    PYTHONPATH=. MUJOCO_GL=egl .venv/bin/python tools/sim_keypoint_corpus.py \
        --out .private/sim_corpus/kp_v1 --squares 64 --offsets 6
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
import numpy as np, cv2

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT))
import mujoco                                              # noqa: E402
from chess_sim import scene as sim_scene                    # noqa: E402
from chess_sim.config import AppearanceConfig, BoardConfig  # noqa: E402

SQ_MM, BOARD_Z = 28.0, 0.4444
LAYOUT_T = (4.9, -1.9)
CROP = 192
W, H, FOVY = 640, 480, 47.0
PIECES = {'pawn': 'white_pawn1', 'knight': 'white_knight1', 'bishop': 'white_bishop1',
          'rook': 'white_rook1', 'queen': 'white_queen1', 'king': 'white_king1'}
PIECE_ORDER = list(PIECES)


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


def project(cam_pos, cam_R, world_pt):
    """World point -> pixel, for a camera looking down its own -z."""
    d = np.asarray(world_pt, float) - np.asarray(cam_pos, float)
    c = cam_R.T @ d
    depth = -c[2]
    if depth <= 1e-9:
        return None
    fy = H / (2.0 * np.tan(np.radians(FOVY) / 2.0))
    return np.array([W / 2.0 + fy * c[0] / depth, H / 2.0 - fy * c[1] / depth])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='.private/sim_corpus/kp_v1')
    ap.add_argument('--squares', type=int, default=64)
    ap.add_argument('--offsets', type=int, default=6)
    ap.add_argument('--offset-mm', type=float, default=11.0)
    ap.add_argument('--seed', type=int, default=17)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    spec = sim_scene.build_scene(BoardConfig(), AppearanceConfig(piece_set='br_v4'))
    spec.worldbody.add_camera(name='probe', pos=[0.0, 0.0, 0.15],
                              xyaxes=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0], fovy=FOVY)
    model = spec.compile()
    data = mujoco.MjData(model)
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'probe')
    renderer = mujoco.Renderer(model, H, W)

    joints = {}
    for i in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if nm and nm.endswith('_joint') and not nm.startswith('so101'):
            joints[nm] = model.jnt_qposadr[i]

    has_light = model.nlight > 0

    X, KP, SQ, PC, CAMPOS, CROP0 = [], [], [], [], [], []
    squares = [f'{f}{r}' for f in 'abcdefgh' for r in '12345678'][:args.squares]

    for square in squares:
        sx_b, sy_b = square_xy(square)
        for k in range(len(PIECE_ORDER) * args.offsets):
            piece = PIECE_ORDER[k % len(PIECE_ORDER)]
            ox = float(rng.uniform(-args.offset_mm, args.offset_mm))
            oy = float(rng.uniform(-args.offset_mm, args.offset_mm))

            # camera inside the operating envelope
            height = rng.uniform(0.12, 0.18)
            stand = rng.uniform(0.055, 0.09)
            bearing = rng.uniform(-0.5, 0.5)
            cam = [sx_b / 1000.0 + stand * math.cos(bearing),
                   sy_b / 1000.0 + stand * math.sin(bearing),
                   BOARD_Z + height]
            tgt = [sx_b / 1000.0, sy_b / 1000.0, BOARD_Z]
            x_, y_ = lookat_xyaxes(cam, tgt)
            cam_R = np.column_stack([x_, y_, np.cross(x_, y_)])
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, cam_R.reshape(-1))
            model.cam_pos[cid] = cam
            model.cam_quat[cid] = q

            if has_light:
                az = rng.uniform(0, 2 * math.pi)
                el = rng.uniform(0.5, 1.2)
                d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el)])
                model.light_dir[0] = d / np.linalg.norm(d)
                inten = rng.uniform(0.35, 1.0)
                model.light_diffuse[0] = [inten, inten, inten]
                model.light_specular[0] = [inten * 0.5] * 3

            for idx, (nm, adr) in enumerate(joints.items()):
                data.qpos[adr:adr + 3] = [2.0 + 0.4 * (idx % 12), 2.0 + 0.4 * (idx // 12), 0.6]
                data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
            adr = joints[PIECES[piece] + '_joint']
            base_world = ((sx_b + ox) / 1000.0, (sy_b + oy) / 1000.0, BOARD_Z)
            data.qpos[adr:adr + 3] = list(base_world)
            data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
            mujoco.mj_forward(model, data)

            kp_full = project(cam, cam_R, base_world)
            sq_full = project(cam, cam_R, [sx_b / 1000.0, sy_b / 1000.0, BOARD_Z])
            if kp_full is None or sq_full is None:
                continue
            x0 = int(round(sq_full[0])) - CROP // 2
            y0 = int(round(sq_full[1])) - CROP // 2
            kp_crop = kp_full - np.array([x0, y0])
            if not (2 < kp_crop[0] < CROP - 3 and 2 < kp_crop[1] < CROP - 3):
                continue

            renderer.update_scene(data, camera='probe')
            img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            pad = cv2.copyMakeBorder(grey, CROP, CROP, CROP, CROP, cv2.BORDER_REPLICATE)
            crop = pad[y0 + CROP:y0 + CROP + CROP, x0 + CROP:x0 + CROP + CROP]

            X.append(crop)
            KP.append(kp_crop.astype(np.float32))
            SQ.append(square)
            PC.append(piece)
            CAMPOS.append(np.array(cam, np.float32))
            CROP0.append(np.array([x0, y0], np.float32))
        print(f'  {square}: {len(X)} samples', flush=True)

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / 'keypoints.npz', x=np.stack(X), kp=np.stack(KP),
                        square=np.array(SQ), piece=np.array(PC),
                        cam=np.stack(CAMPOS), crop0=np.stack(CROP0),
                        board_z=BOARD_Z, fovy=FOVY, size=(W, H), crop=CROP)
    print(f'wrote {out}/keypoints.npz  {len(X)} samples')


if __name__ == '__main__':
    sys.exit(main())
