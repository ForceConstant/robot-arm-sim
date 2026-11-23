#!/usr/bin/env python3
"""Sim data generator with a FREE camera (no arm in the loop).

Brian's call 2026-11-23: don't drive the sim arm — place the camera wherever we
want and label every frame exactly, because the sim knows the piece base.

Writes to .private/sim_corpus/<set>/:
    images/*.jpg            rendered wrist-like views
    manifest.json           piece, square, exact base xy (mm), camera pose
Labels are the placed contact point, so there is no clicking error and full
freedom over within-square offset and viewing angle.

    PYTHONPATH=. MUJOCO_GL=egl .venv/bin/python tools/sim_freecam_corpus.py \
        --out .private/sim_corpus/pilot1 --frames 12
"""
from __future__ import annotations
import argparse, json, math, random, sys
from pathlib import Path

import numpy as np

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT))

import mujoco                                              # noqa: E402
from chess_sim import scene as sim_scene                    # noqa: E402
from chess_sim.config import AppearanceConfig, BoardConfig    # noqa: E402

SQUARE_MM = 28.0
BOARD_Z_M = 0.4444          # piece contact height measured in the built model
CAM_FOVY = 47.0             # measured C270 with the CAD mount
W, H = 640, 480
CAM_NAME = 'freecam'

PIECES = ('pawn', 'knight', 'bishop', 'rook', 'queen', 'king')
BODY_FOR = {  # first slot of each type
    'pawn': 'white_pawn1', 'knight': 'white_knight1', 'bishop': 'white_bishop1',
    'rook': 'white_rook1', 'queen': 'white_queen1', 'king': 'white_king1',
}


def square_xy_mm(name):
    return ((ord(name[0]) - 97 + 0.5) * SQUARE_MM - 112.0,
            (int(name[1]) - 0.5) * SQUARE_MM - 112.0)


def add_free_camera(spec):
    """Camera on the world body; we move it by writing model.cam_pos/cam_quat."""
    spec.worldbody.add_camera(name=CAM_NAME, pos=[0.0, 0.0, 0.15],
                              xyaxes=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                              fovy=CAM_FOVY)
    return CAM_NAME


def look_at(pos, target, up=(0.0, 0.0, 1.0)):
    """MuJoCo xyaxes for a camera at pos looking at target."""
    z = np.asarray(pos, float) - np.asarray(target, float)
    z /= np.linalg.norm(z)                       # camera looks along -z
    x = np.cross(np.asarray(up, float), z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.concatenate([x, y])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='.private/sim_corpus/pilot1')
    ap.add_argument('--frames', type=int, default=12)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--offset-mm', type=float, default=10.0)
    ap.add_argument('--grid', action='store_true',
                    help='iterate square grid x piece types x offsets deterministically')
    ap.add_argument('--per-cell', type=int, default=4,
                    help='frames per (piece, square) in --grid mode')
    ap.add_argument('--pieces', default='pawn,king,queen,rook,bishop,knight',
                    help='comma list for --grid mode')
    ap.add_argument('--fixed-piece', default=None,
                    help='hold this piece type (offset still varies)')
    ap.add_argument('--fixed-square', default=None,
                    help='hold this square (offset still varies)')
    ap.add_argument('--fixed-camera', action='store_true',
                    help='hold the viewpoint constant so offset is the only variation')
    ap.add_argument('--piece-set', default='default',
                    help="piece set to render (e.g. br_v4 for the new queen)")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    out = ROOT / args.out
    (out / 'images').mkdir(parents=True, exist_ok=True)

    spec = sim_scene.build_scene(BoardConfig(),
                                 AppearanceConfig(piece_set=args.piece_set))
    add_free_camera(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAM_NAME)
    print('camera id', cam_id, 'fovy', round(float(model.cam_fovy[cam_id]), 1))

    joints = {}
    for i in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if nm and nm.endswith('_joint') and not nm.startswith('so101'):
            joints[nm] = model.jnt_qposadr[i]

    renderer = mujoco.Renderer(model, H, W)
    records = []
    for k in range(args.frames):
        if args.grid:
            plist = [p.strip() for p in args.pieces.split(',') if p.strip()]
            cells = [(p, f'{f}{r}') for p in plist
                     for f in 'abcdefgh' for r in '12345678']
            cell_i, rep = divmod(k, max(1, args.per_cell))
            piece, square = cells[cell_i % len(cells)]
        else:
            piece = args.fixed_piece or PIECES[k % len(PIECES)]
            square = args.fixed_square or rng.choice(
                [f'{f}{r}' for f in 'abcdefgh' for r in '12345678'])
        sx, sy = square_xy_mm(square)
        ox = rng.uniform(-args.offset_mm, args.offset_mm)
        oy = rng.uniform(-args.offset_mm, args.offset_mm)
        bx, by = sx + ox, sy + oy

        # park every piece far off the board, SPREAD OUT: stacking all 32 on one
        # point produced 11k contacts and a MuJoCo stack overflow (2026-11-23)
        for idx, (name, adr) in enumerate(joints.items()):
            data.qpos[adr:adr + 3] = [2.0 + 0.4 * (idx % 12), 2.0 + 0.4 * (idx // 12), 0.6]
            data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
        target = BODY_FOR[piece]
        adr = joints[target + '_joint']
        data.qpos[adr:adr + 3] = [bx / 1000.0, by / 1000.0, BOARD_Z_M]
        data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]

        # camera: over the square, tilted, like the wrist view but free
        if args.fixed_camera:
            height, stand, bearing = 0.15, 0.075, 0.0
        else:
            height = rng.uniform(0.10, 0.19)
            stand = rng.uniform(0.05, 0.10)
            bearing = rng.uniform(0, 2 * math.pi)
        cam = [bx / 1000.0 + stand * math.cos(bearing),
               by / 1000.0 + stand * math.sin(bearing),
               BOARD_Z_M + height]
        model.cam_pos[cam_id] = cam
        model.cam_quat[cam_id] = _quat_from_xyaxes(look_at(cam, [bx / 1000.0, by / 1000.0, BOARD_Z_M]))
        mujoco.mj_forward(model, data)

        renderer.update_scene(data, camera=CAM_NAME)
        img = renderer.render()
        name = f'{piece}_{square}_{k:03d}.jpg'
        import cv2
        cv2.imwrite(str(out / 'images' / name), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        records.append({'image': name, 'piece': piece, 'square': square,
                        'base_mm': [round(bx, 2), round(by, 2)],
                        'square_centre_mm': [round(sx, 2), round(sy, 2)],
                        'offset_mm': [round(ox, 2), round(oy, 2)],
                        'camera_m': [round(v, 4) for v in cam],
                        'height_m': round(height, 4)})
        print(f'  {name}: base {bx:.1f},{by:.1f} (offset {ox:+.1f},{oy:+.1f}) '
              f'cam h={height:.3f}')

    (out / 'manifest.json').write_text(json.dumps({
        'schema': 'sim-freecam-corpus/1',
        'board_z_m': BOARD_Z_M, 'fovy_deg': CAM_FOVY, 'size': [W, H],
        'frames': records}, indent=2) + '\n')
    print('wrote', out / 'manifest.json', len(records), 'frames')
    return 0


def _quat_from_xyaxes(xyaxes):
    x = np.asarray(xyaxes[:3], float)
    y = np.asarray(xyaxes[3:], float)
    z = np.cross(x, y)
    m = np.column_stack([x, y, z]).reshape(-1)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, m)
    return q


if __name__ == '__main__':
    sys.exit(main())
