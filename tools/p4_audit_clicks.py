#!/usr/bin/env python3
"""Audit: for each labeled view, does the click land near the piece's target
square? Computes PnP target-square pixel and the pixel distance to Brian's
click. Large distances mean the click was on the wrong piece (view suspect).
"""
import json,sys
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from pathlib import Path
R=Path('/home/brian/robot-arm-sim/.private/real_piece_views')
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
def sq_xy(sq):
    return ((ord(sq[0].lower())-97)+0.5)*28-112,(int(sq[1])-0.5)*28-112
for ds in ('adapt1','adapt2'):
    d=R/ds
    if not (d/'manifest.json').exists():continue
    mf=json.load(open(d/'manifest.json'));lab=json.load(open(d/'labels_human.json'))
    print(f'\n=== {ds} ===')
    for pl in mf['placements']:
        piece,sq=pl['piece'],pl['square'];tx,ty=sq_xy(sq)
        for v in pl['views']:
            if v['image'] not in lab:continue
            img=cv2.imread(str(d/v['image']))
            h,w=img.shape[:2]
            if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
            sol,_=b.solve(b.rectify(img))
            if sol is None:continue
            p=b.project(sol,np.array([tx,ty,0.]),0.)
            click=np.array(lab[v['image']],float)
            dist=float(np.linalg.norm(click-p))
            flag='  <-- FAR' if dist>90 else ''
            print(f'  {piece:7s} {sq} {v["view"]:18s} target_px=({p[0]:5.0f},{p[1]:5.0f}) click=({click[0]:.0f},{click[1]:.0f}) dist={dist:5.0f}px{flag}')
