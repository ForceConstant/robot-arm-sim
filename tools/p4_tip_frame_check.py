#!/usr/bin/env python3
"""Decisive diagnostic: at the current pose, does the FK jaw tip project through
PnP to the user-set tip pixel? If yes, FK and camera board frames are consistent
and absolute xyz can drive the tip onto the piece. READ-ONLY, no motion."""
import sys, json, urllib.request
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw, saved_tip_pixel
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
with urllib.request.urlopen('http://192.168.1.171:8080/keep-in',timeout=5) as r:
    k=json.load(r)
tip=np.array(k['tip_mm'])   # board xyz from FK
px=saved_tip_pixel()
print('FK tip_mm',tip.round(2),'saved tip pixel',px)

f=grab_frame(timeout=6);raw=to_raw(f)
rect=b.rectify(raw);sol,d=b.solve(rect)
print('markers',len(d),'rms',round(sol['rms_px'],2),'cam_board',np.array(sol['camera_in_board_mm']).round(1))

# project FK tip at its TRUE z and at z=0
for z in (tip[2],0.0):
    p=b.project(sol,np.array([tip[0],tip[1],z]),z)
    print(f'FK tip projected (z={z:.1f}) -> pixel {p.round(1)}')
# unproject the SAVED tip pixel at the tip depth -> board xy
pu=b.unproject(sol,px,tip[2])
print('saved pixel unprojected at tip z -> board xy',pu.round(2))
# piece pixel from last measure ~ (252,187); unproject at z=0
pp=b.unproject(sol,(252,187),0.0)
print('piece pixel (252,187) unprojected z=0 -> board xy',pp.round(2))
delta=np.linalg.norm(pu[:2]-tip[:2])
print(f'|saved-pixel board - FK tip board| = {delta:.1f} mm')
