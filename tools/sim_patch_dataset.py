#!/usr/bin/env python3
"""Build patch datasets for the offset-CNN: sim corpus + real labelled frames.

Writes .npz with patches (N,64,64) float32 in [0,1] and targets (N,2) = base
offset from the square centre, in millimetres, plus piece index and source.

    PYTHONPATH=. .venv/bin/python tools/sim_patch_dataset.py
"""
import json, sys
from pathlib import Path
import numpy as np, cv2

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT))
from chess_sim.board_target import Board, SQUARE_MM   # noqa: E402

PPMM = 8.0            # rectified pixels per mm
WINDOW_MM = 40.0      # crop window across (piece is ~45 mm, offsets to +/-12)
OUT_PX = 96           # fed to the CNN
SIM = ROOT / '.private/sim_corpus/v4_300'
REAL = ROOT / '.private/real_piece_views/merged1'
OUT = ROOT / '.private/sim_corpus/offset_dataset.npz'
PIECES = ['pawn', 'knight', 'bishop', 'rook', 'queen', 'king']
LAYOUT_T = (4.9, -1.9)   # translation of the 180 deg frame change, mm


def square_xy(name):
    return ((ord(name[0]) - 97 + 0.5) * SQUARE_MM - 112.0,
            (int(name[1]) - 0.5) * SQUARE_MM - 112.0)


def patch(frame, board, centre_xy):
    if frame.shape[0] > frame.shape[1]:
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    rect = board.rectify(frame)
    sol, _ = board.solve(rect)
    if sol is None:
        return None, None
    top, _, n, _ = board.rectified(cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY), sol, PPMM)
    cx = int(n / 2 + centre_xy[0] * PPMM)
    cy = int(n / 2 - centre_xy[1] * PPMM)
    half = int(round(WINDOW_MM * PPMM / 2))
    x0, y0 = cx - half, cy - half
    span = 2 * half
    rs = cv2.copyMakeBorder(top, max(0, -y0), max(0, y0 + span - n),
                            max(0, -x0), max(0, x0 + span - n), cv2.BORDER_REFLECT)
    p = rs[y0 + max(0, -y0):y0 + max(0, -y0) + span,
           x0 + max(0, -x0):x0 + max(0, -x0) + span].astype(np.float32)
    p = cv2.resize(p, (OUT_PX, OUT_PX), interpolation=cv2.INTER_AREA)
    if p.ndim == 3:
        p = cv2.cvtColor(p.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)
    return p / 255.0, sol


def main():
    board = Board(json.loads((ROOT / '.private/phase11d/calibration.json').read_text()))
    xs, ys, ps, src = [], [], [], []

    man = json.loads((SIM / 'manifest.json').read_text())
    for rec in man['frames']:
        img = cv2.imread(str(SIM / 'images' / rec['image']))
        if img is None:
            continue
        # The crop must be centred in the MARKER-LAYOUT frame, because that is
        # the frame the rectified image lives in. The manifest holds board-body
        # millimetres, which are 180 deg rotated (measured 2026-11-23) - cropping
        # on the body centre put the square centre a board-width away.
        sx_b, sy_b = rec['square_centre_mm']
        sx = -sx_b + LAYOUT_T[0]
        sy = -sy_b + LAYOUT_T[1]
        p, sol = patch(img, board, (sx, sy))
        if p is None:
            continue
        # Sim board-body -> marker-layout frame: 180 deg rotation plus a small
        # translation, measured 2026-11-23 by projecting the known base through
        # the generator's own camera and reading it back with PnP (residual
        # after the rotation was (+4.9, -1.9) mm, sd ~2 mm).
        bx_l = -rec['base_mm'][0] + LAYOUT_T[0]
        by_l = -rec['base_mm'][1] + LAYOUT_T[1]
        xs.append(p)
        ys.append([bx_l - sx, by_l - sy])
        ps.append(PIECES.index(rec['piece']))
        src.append(0)
    n_sim = len(xs)
    print('sim patches', n_sim)

    labels = json.loads((REAL / 'labels_human.json').read_text())
    manifest = json.loads((REAL / 'manifest.json').read_text())
    lookup = {}
    for pl in manifest['placements']:
        for v in pl['views']:
            lookup[v['image']] = (pl['piece'], pl['square'])
    n_real = 0
    for name, (px, py) in sorted(labels.items()):
        piece, square = lookup.get(name, (None, None))
        path = REAL / name
        if piece is None or not path.exists():
            continue
        img = cv2.imread(str(path))
        sx, sy = square_xy(square)
        p, sol = patch(img, board, (sx, sy))
        if p is None:
            continue
        base = board.unproject(sol, (px, py), 0.0)
        if not np.all(np.isfinite(base)):
            continue
        xs.append(p); ys.append([base[0] - sx, base[1] - sy])
        ps.append(PIECES.index(piece)); src.append(1)
        n_real += 1
    print('real patches', n_real)

    np.savez_compressed(OUT, x=np.stack(xs), y=np.array(ys, np.float32),
                        piece=np.array(ps), source=np.array(src))
    print('wrote', OUT, np.stack(xs).shape)


if __name__ == '__main__':
    main()
