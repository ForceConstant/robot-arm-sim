#!/usr/bin/env python3
"""Cross-check console piece uv -> PnP board xy vs console's own x_mm/y_mm and
vs the bright-blob detector. READ-ONLY."""
import sys, json, urllib.request
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
st=json.load(urllib.request.urlopen('http://192.168.1.171:8080/state',timeout=8))
p=st['target']['piece']
print('console piece uv',p['uv'],'board',p['x_mm'],p['y_mm'],'area',p['area_mm2'])
f=grab_frame(timeout=6);raw=to_raw(f)
rect=b.rectify(raw);sol,d=b.solve(rect)
print('markers',len(d),'rms',round(sol['rms_px'],2))
# console uv is in DISPLAY (portrait) coords; convert to raw landscape like to_raw does
h,w=f.shape[:2]
u,v=p['uv']
# portrait->landscape CCW: (u,v) in portrait WxH=480x640 -> raw 640x480
# raw = rotate portrait CCW: raw_x = v? derive: CCW rot maps (x,y)_portrait -> (y, W-1-x)_raw
raw_u = v
raw_v = (w-1)-u
print('uv portrait',u,v,'-> raw landscape',raw_u,raw_v)
xy=b.unproject(sol,(raw_u,raw_v),0.0)
print('PnP unproject of console uv (z=0) -> board',xy[:2].round(1))
# try the other mapping too
xy2=b.unproject(sol,(u,v),0.0)
print('PnP unproject raw (no conversion) -> board',xy2[:2].round(1))
