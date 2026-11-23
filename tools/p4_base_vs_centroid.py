#!/usr/bin/env python3
"""Read-only: measure pawn centroid vs BASE contact point across frames at the
current stationary pose. The centroid is view-dependent (raised bright top);
the base contact point should be view-invariant. Uses a darker-piece silhouette
plus the bright-blob bottom edge, both unprojected at board z=0.
"""
import sys, json, time
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

rows=[]
for k in range(15):
    f=grab_frame(timeout=6); raw=to_raw(f)
    rect=b.rectify(raw); sol,d=b.solve(rect)
    if sol is None or len(d)<3 or sol['rms_px']>6:
        print('k',k,'ids',len(d),'skip');time.sleep(0.12);continue
    grey=cv2.cvtColor(raw,cv2.COLOR_BGR2GRAY)
    p5=b.project(sol,np.array([14.,14.,0.]),0.)
    # bright top blob (loosened threshold)
    bright=(grey>150).astype(np.uint8)
    bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    n,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
    cands=[]
    for i in range(1,n):
        x,y,w,h,area=stats[i];cx,cy=cents[i]
        if area>100 and np.hypot(cx-p5[0],cy-p5[1])<100 and h<180:
            cands.append((area,cx,cy,x,y,w,h))
    cands.sort(key=lambda c:-c[0])
    row={'k':k,'ids':len(d),'rms':round(sol['rms_px'],2)}
    if cands:
        area,cx,cy,x,y,w,h=cands[0]
        cen=b.unproject(sol,(cx,cy),0.)[:2]
        base=b.unproject(sol,(cx,y+h-1),0.)[:2]   # bottom edge of bright blob
        row['centroid']=cen.round(1).tolist()
        row['base_bright']=base.round(1).tolist()
        row['blob']=[int(x),int(y),int(w),int(h)]
        print('k',k,'ids',len(d),'rms',row['rms'],'centroid',row['centroid'],
              'base_bright',row['base_bright'],'blob',row['blob'],flush=True)
    else:
        print('k',k,'ids',len(d),'no blob',flush=True)
    rows.append(row)
    time.sleep(0.12)

valid=[r for r in rows if 'centroid' in r]
if valid:
    cen=np.array([r['centroid'] for r in valid]); bs=np.array([r['base_bright'] for r in valid])
    print('\nn',len(valid))
    print('centroid median',np.median(cen,axis=0).round(1),'std',cen.std(axis=0).round(2))
    print('base_bright median',np.median(bs,axis=0).round(1),'std',bs.std(axis=0).round(2))
    json.dump({'rows':rows,
               'centroid_median':np.median(cen,axis=0).round(2).tolist(),
               'base_median':np.median(bs,axis=0).round(2).tolist()},
              open('/home/brian/robot-arm-sim/reports/base_vs_centroid_20261122.json','w'),indent=2)
