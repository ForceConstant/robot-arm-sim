#!/usr/bin/env python3
"""Test whether unprojecting the pawn's bright blob at its TRUE HEIGHT (23mm)
gives a view-invariant board point, vs the z=0 projection which shifts.

At each of several camera poses, detect the blob, unproject its centroid at
z=0 and z=23mm, and log. If z=23 is stable across poses while z=0 shifts,
that's the correct aim primitive for general point servoing.

Uses safe rel moves to vary the view slightly; jaws open; no grasp.
"""
import json, sys, time, urllib.request, urllib.error
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
PAWN_H=23.0
def post(p,body,t=60):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:120]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)

def measure(tag):
    best=None
    for k in range(8):
        f=grab_frame(timeout=6);raw=to_raw(f)
        rect=b.rectify(raw);sol,d=b.solve(rect)
        if sol is None or len(d)<3 or sol['rms_px']>6:
            time.sleep(0.12);continue
        grey=cv2.cvtColor(raw,cv2.COLOR_BGR2GRAY)
        p5=b.project(sol,np.array([14.,14.,0.]),0.)
        bright=(grey>150).astype(np.uint8)
        bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        n,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
        cands=[]
        for i in range(1,n):
            x,y,w,h,area=stats[i];cx,cy=cents[i]
            if area>100 and np.hypot(cx-p5[0],cy-p5[1])<100 and h<180:
                cands.append((area,float(cx),float(cy)))
        cands.sort(key=lambda c:-c[0])
        if not cands: time.sleep(0.12);continue
        area,cx,cy=cands[0]
        xy0=b.unproject(sol,(cx,cy),0.0)
        xyh=b.unproject(sol,(cx,cy),PAWN_H)
        best={'tag':tag,'ids':len(d),'rms':round(sol['rms_px'],2),
              'centroid_px':[round(cx,1),round(cy,1)],
              'cam_board':np.array(sol['camera_in_board_mm']).round(1).tolist(),
              'xy_z0':xy0.round(1).tolist(),'xy_z23':xyh.round(1).tolist()}
        break
    return best

def main():
    st=get(BRIDGE+'/state');print('start',st['mode'],st['fault'],flush=True)
    if st['mode']!='REMOTE':post('/mode',{'mode':'REMOTE'});time.sleep(0.6)
    roll0=st['joints']['wrist_roll']
    rows=[]
    # sample at current pose
    m=measure('pose0');rows.append(m);print('pose0',m,flush=True)
    # vary view with small rel moves (jaws open), re-measure each
    for i,(dx,dy) in enumerate([(6,0),(0,6),(-6,0),(0,-6)]):
        body={'mode':'rel','dx_mm':dx,'dy_mm':dy,'dz_mm':0}
        dr=post('/move',{**body,'dry_run':True})
        if dr.get('detail') or (dr.get('max_joint_delta_deg') or 99)>15:
            print('skip move',i,dr.get('detail'),flush=True);continue
        post('/move',body,t=60);time.sleep(1.0)
        st=get(BRIDGE+'/state')
        if st['fault'] or abs(st['joints']['wrist_roll']-roll0)>5:
            print('guard abort',flush=True);break
        m=measure(f'pose{i+1}')
        if m: rows.append(m);print(f'pose{i+1}',m,flush=True)
    print('\n=== VARIABILITY ===',flush=True)
    z0=np.array([r['xy_z0'] for r in rows if r]);zh=np.array([r['xy_z23'] for r in rows if r])
    print('z=0 projection  std',z0.std(axis=0).round(2),'range',(z0.max(axis=0)-z0.min(axis=0)).round(1),flush=True)
    print('z=23 projection std',zh.std(axis=0).round(2),'range',(zh.max(axis=0)-zh.min(axis=0)).round(1),flush=True)
    json.dump(rows,open('/home/brian/robot-arm-sim/reports/view_invariance_20261122.json','w'),indent=2)
    post('/mode',{'mode':'HOLD'})
    print('HOLD',flush=True)

if __name__=='__main__':main()
