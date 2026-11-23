#!/usr/bin/env python3
"""Measure (piece board pos, tip board pos, tip pixel, piece pixel) over frames.
Confirms a stable closed-loop target before attempting the grasp."""
import sys, json, time, urllib.request
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw, saved_tip_pixel
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

def fk_tip():
    with urllib.request.urlopen('http://192.168.1.171:8080/keep-in',timeout=5) as r:
        return json.load(r)['tip_mm']

rows=[]
for k in range(10):
    f=grab_frame(timeout=6); raw=to_raw(f)
    rect=b.rectify(raw); sol,d=b.solve(rect)
    if sol is None or len(d)<3:
        print('k',k,'markers',len(d),'skip');time.sleep(0.15);continue
    grey=cv2.cvtColor(raw,cv2.COLOR_BGR2GRAY)
    p5=b.project(sol,np.array([14.,14.,0.]),0.)
    bright=(grey>170).astype(np.uint8)
    bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((7,7),np.uint8))
    n,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
    cands=[]
    for i in range(1,n):
        x,y,w,h,area=stats[i]; cx,cy=cents[i]
        if area>200 and np.hypot(cx-p5[0],cy-p5[1])<90 and h<180:
            cands.append((area,cx,cy,x,y,h))
    cands.sort(key=lambda c:-c[0])
    tip=fk_tip()
    row={'k':k,'markers':len(d),'rms':round(sol['rms_px'],2),'tip_mm':tip[:2],
         'tip_px':saved_tip_pixel()}
    if cands:
        area,cx,cy,x,y,h=cands[0]
        cm=b.unproject(sol,(cx,cy),0.)
        row['piece_px']=[round(float(cx),1),round(float(cy),1)]
        row['piece_mm']=cm.round(1).tolist()
        row['err_mm']=round(float(np.linalg.norm(cm[:2]-np.array(tip[:2]))),1)
    rows.append(row)
    print('k',k,'ids',len(d),'rms',row['rms'],'tip',row['tip_mm'],
          'piece',row.get('piece_mm'),'err',row.get('err_mm'),'piecepx',row.get('piece_px'))
    time.sleep(0.15)

if rows:
    print('\nSUMMARY n',len(rows))
    pm=np.array([r['piece_mm'] for r in rows if r.get('piece_mm')])
    tm=np.array([r['tip_mm'] for r in rows])
    if len(pm):
        print('piece median',np.median(pm,axis=0).round(1),'spread',pm.std(axis=0).round(1))
    print('tip median',np.median(tm,axis=0).round(1),'spread',tm.std(axis=0).round(1))
    errs=[r['err_mm'] for r in rows if r.get('err_mm')]
    if errs: print('tip-piece error median',round(float(np.median(errs)),1),'min',min(errs),'max',max(errs))
    open('/home/brian/robot-arm-sim/reports/pair_measure_20261122.json','w').write(json.dumps(rows,indent=2))
