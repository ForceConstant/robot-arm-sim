#!/usr/bin/env python3
"""Capture REAL multi-view wrist images of a piece (2026-11-22).

Uses the 17 JOINT poses from marker_scan_plan_charuco.json - all measured on
this real arm and known reachable (look_at ring was NOT reachable from the
folded posture; these poses are). For each pose: move to joints, grab raw frame
+ PnP/pose metadata, save image. Jaws open, no grasp, guards on fault/mode.

    --square e5 --piece pawn --views 6 --label pc1
"""
import argparse, hashlib, json, sys, time, urllib.request, urllib.error
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw

CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
ROOT=Path('/home/brian/robot-arm-sim')
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
JOINTS=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
def post(p,body,t=90):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:150]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)

def grab_meta():
    for k in range(8):
        f=grab_frame(timeout=6); raw=to_raw(f)
        rect=b.rectify(raw); sol,d=b.solve(rect)
        if sol is None or len(d)<3 or sol['rms_px']>8:
            time.sleep(0.12);continue
        return {'frame':raw,'markers':len(d),'rms':round(sol['rms_px'],2),
                'ids':[int(x.marker_id) for x in d],
                'camera_in_board_mm':np.array(sol['camera_in_board_mm']).round(2).tolist(),
                'R':np.array(sol['R']).round(6).tolist(),
                'tvec':np.array(sol['tvec']).round(6).tolist()}
    return None

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--square',default='e5');ap.add_argument('--piece',default='pawn')
    ap.add_argument('--views',type=int,default=8);ap.add_argument('--label',default='pc1')
    ap.add_argument('--out',default='.private/real_piece_views')
    ap.add_argument('--stride',type=int,default=2)
    a=ap.parse_args()
    outdir=ROOT/a.out/(f'{a.piece}_{a.square}_{a.label}')
    outdir.mkdir(parents=True,exist_ok=True)
    plan=json.load(open(str(ROOT/'chess_sim/assets/marker_scan_plan_charuco.json')))
    poses=plan['poses'][::a.stride][:a.views]
    st=get(BRIDGE+'/state');assert st['fault'] is None,st['fault']
    print('start',st['mode'],'fault',st['fault'],'roll',round(st['joints']['wrist_roll'],2),flush=True)
    if st['mode']!='REMOTE':print('->REMOTE',post('/mode',{'mode':'REMOTE'}));time.sleep(0.6)
    st=get(BRIDGE+'/state');assert st['mode']=='REMOTE','not REMOTE'
    ctrls={c['name']:c['value'] for c in get(CONSOLE+'/camera/controls')['controls']}
    manifest={'schema':'real-piece-views/1','square':a.square,'piece':a.piece,'label':a.label,
              'camera_controls':ctrls,'intrinsics':{'fx':b.fx,'fy':b.fy,'cx':b.cx,'cy':b.cy},
              'calibration':'.private/phase11d/calibration.json',
              'source_plan':'chess_sim/assets/marker_scan_plan_charuco.json','samples':[]}
    n=0
    for i,p in enumerate(poses):
        jrad=p['joints']
        jdeg={JOINTS[k]:float(np.rad2deg(jrad[k])) for k in range(5)}
        jdeg['gripper']=100.0
        body={'mode':'joint','joints':jdeg,'jaw':100}
        print(f'view {i} {p["name"]}: roll {p.get("measured_roll_deg")} -> {jdeg}',flush=True)
        r=post('/move',body,t=90)
        if r.get('http_error') or r.get('detail'):
            print('  move fail',r,flush=True);continue
        time.sleep(1.0)
        st=get(BRIDGE+'/state')
        if st['fault'] or st['mode']!='REMOTE':print('  guard abort',flush=True);break
        m=grab_meta()
        if not m:
            print('  no PnP frame, skip',flush=True);continue
        img=m.pop('frame')
        name=f'{a.piece}_{a.square}_{a.label}_v{i:02d}_{p["name"]}.jpg'
        cv2.imwrite(str(outdir/name),img)
        sha=hashlib.sha256((outdir/name).read_bytes()).hexdigest()
        pose=get(CONSOLE+'/pose');keep=get(CONSOLE+'/keep-in')
        manifest['samples'].append({'view_index':i,'pose_name':p['name'],
            'planned_roll_deg':p.get('measured_roll_deg'),'image':name,'sha256':sha,
            'joints_deg':pose.get('joints_deg'),'tool_mm':pose.get('tool_mm'),
            'tip_mm':keep.get('tip_mm'),**m})
        n+=1
        print(f'  saved ids {m["ids"]} rms {m["rms"]}',flush=True)
    post('/mode',{'mode':'HOLD'})
    (outdir/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'DONE {n} views -> {outdir}; HOLD',flush=True)

if __name__=='__main__':main()
