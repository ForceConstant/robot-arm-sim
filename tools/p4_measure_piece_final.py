#!/usr/bin/env python3
"""Measure piece board position (centroid + base) over frames from current pose."""
import sys, json, time
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw, detect
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
rows=[]
for k in range(10):
    f=grab_frame(timeout=6); raw=to_raw(f)
    ann,info=detect(raw)
    if info.get('centroid_mm'):
        rows.append({'k':k,'markers':info['markers'],'rms':info['rms'],
                     'centroid_mm':info['centroid_mm'],'base_mm':info['base_mm'],
                     'offset_mm':info['offset_mm'],'piece_px':None})
        print('k',k,'ids',info['markers'],'rms',info['rms'],'centroid',info['centroid_mm'],
              'base',info['base_mm'],'off',info['offset_mm'],flush=True)
    else:
        print('k',k,'ids',info['markers'],'no piece blob',flush=True)
    time.sleep(0.12)
if rows:
    c=np.array([r['centroid_mm'] for r in rows]); bs=np.array([r['base_mm'] for r in rows])
    print('\ncnt n',len(rows))
    print('centroid median',np.median(c,axis=0).round(1),'std',c.std(axis=0).round(2))
    print('base median',np.median(bs,axis=0).round(1),'std',bs.std(axis=0).round(2))
    json.dump({'rows':rows,'centroid_median':np.median(c,axis=0).round(2).tolist(),
               'base_median':np.median(bs,axis=0).round(2).tolist()},
              open('/home/brian/robot-arm-sim/reports/piece_measure_final_20261122.json','w'),indent=2)
