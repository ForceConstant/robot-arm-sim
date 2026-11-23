#!/usr/bin/env python3
"""Unproject a set of image pixels through the live PnP pose to board mm.
Prints a table so the piece's board position can be read from a known pixel."""
import json,sys
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

best=None
for k in range(10):
    f=grab_frame(timeout=6); rect=b.rectify(f); sol,d=b.solve(rect)
    if sol is None or len(d)<3: continue
    best=(f,sol,d);break
if not best:
    print('no clean frame');sys.exit(1)
f,sol,d=best
print(f'markers {len(d)} ids {sol["ids"]} rms {sol["rms_px"]:.2f}')
print('camera_in_board_mm', np.array(sol['camera_in_board_mm']).round(1).tolist())
# e5 nominal projection
p5=b.project(sol,np.array([14.,14.,0.]),0.0)
print(f'e5 nominal (14,14) projects to pixel {p5.round(1)}')
print('\npixel -> board mm at z=0:')
for px,py in [(320,240),(185,210),(155,235),(145,265),(150,270),(160,255),
              (int(p5[0]),int(p5[1])),(200,230),(170,260)]:
    mm=b.unproject(sol,(px,py),0.0)
    print(f'  ({px:3d},{py:3d}) -> {mm.round(1)}')
import cv2
cv2.imwrite('/tmp/unproj_frame.jpg',f)
