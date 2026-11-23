#!/usr/bin/env python3
"""Scale-invariant focus analysis: measure sharpness on BOARD-RECTIFIED
top-down images (fixed px/mm), removing the magnification confound of raw
Laplacian variance. Uses the clean frames already captured by p4_focus_sweep.
"""
import json,sys,glob,re
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
rep=json.load(open('/home/brian/robot-arm-sim/reports/focus_sweep_20261123.json'))
print('z_cmd | tip_z | raw_lap | rectified_lap(@6px/mm) | markers')
for r in rep:
    z=r['z_cmd'];tip=r['tip']['z']
    # find the capture-view frame saved for this z: sweep used name 'focus'
    # but only the last few persist by timestamp; use stored focus score
    raw=r['focus']['score']
    # We need the image; list recent captures and match by order is unreliable,
    # so report raw and note rectified recompute requires re-capture.
    print(f'{z:5d} | {tip:5.1f} | {raw:7.1f} | (need frame) | -')
print('\nRaw focus score rises with distance: z=35->31, z=60->66, z=74->149.')
print('Laplacian variance is scale-dependent (magnification lowers it), so this')
print('alone cannot distinguish defocus from larger pixels. A scale-invariant')
print('test must rectify each frame to a fixed px/mm board view first.')
