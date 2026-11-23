#!/usr/bin/env python3
"""Convert Brian's new queen STL into the sim's piece-set layout.

Creates chess_sim/assets/piece_sets/br_v4/ as a copy of the default set with the
queen mesh replaced, so the training corpus renders the new queen while every
other piece stays as it was. Reports mesh dimensions against the current queen
so a scale or orientation mistake is obvious before it reaches training.

    .venv/bin/python tools/install_queen_mesh.py \
        --stl .private/queen_m6x1_v4_part1.stl
"""
from __future__ import annotations
import argparse, shutil, sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path('/home/brian/robot-arm-sim')
PIECES = ROOT / 'chess_sim/assets/pieces'
SETS = ROOT / 'chess_sim/assets/piece_sets'
NEW_SET = 'br_v4'


def bbox_mm(mesh: trimesh.Trimesh) -> tuple[float, float, float]:
    ext = mesh.extents
    return tuple(float(v) * 1000.0 for v in ext)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stl', default='.private/queen_m6x1_v4_part1.stl')
    ap.add_argument('--set', default=NEW_SET)
    args = ap.parse_args()

    stl = ROOT / args.stl
    if not stl.exists():
        print('missing stl', stl); return 1
    mesh = trimesh.load_mesh(str(stl), process=True)
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.to_mesh()
    print(f'loaded {stl.name}: {len(mesh.faces)} faces, watertight {mesh.is_watertight}')
    print(f'  extents mm  x {bbox_mm(mesh)[0]:.1f}  y {bbox_mm(mesh)[1]:.1f}  z {bbox_mm(mesh)[2]:.1f}')

    ref = trimesh.load_mesh(str(PIECES / 'white_queen' / 'white_queen.obj'), process=True)
    if isinstance(ref, trimesh.Scene):
        ref = ref.to_mesh()
    print(f'current queen extents mm  x {bbox_mm(ref)[0]:.1f}  y {bbox_mm(ref)[1]:.1f} '
          f'z {bbox_mm(ref)[2]:.1f}')
    rz = bbox_mm(ref)[2]
    nz = bbox_mm(mesh)[2]
    if rz > 0:
        print(f'  height ratio new/current = {nz / rz:.3f}')
    # The STL arrives in its own units (19.2 m tall as metres, same aspect as
    # the piece it replaces). Scale it so the height matches the current queen,
    # which keeps the scene's scale and the board-relative proportions intact.
    scale = (rz / nz) if nz > 0 else 1.0
    mesh = mesh.copy()
    mesh.apply_scale(scale)
    mesh.apply_translation(-mesh.bounds[0])           # sit the base at z = 0
    print(f'  scaled by {scale:.6f} -> extents mm x {bbox_mm(mesh)[0]:.1f} '
          f'y {bbox_mm(mesh)[1]:.1f} z {bbox_mm(mesh)[2]:.1f}')

    out = SETS / args.set
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    for src in sorted(PIECES.iterdir()):
        if not src.is_dir():
            continue
        dst = out / src.name
        shutil.copytree(src, dst)
    print('copied piece set ->', out)

    for colour in ('white', 'black'):
        dst = out / f'{colour}_queen'
        obj = dst / f'{colour}_queen.obj'
        mesh.export(str(obj))
        print(f'  replaced {obj.relative_to(ROOT)}')
    print('done. Build with AppearanceConfig(piece_set=%r)' % args.set)
    return 0


if __name__ == '__main__':
    sys.exit(main())
