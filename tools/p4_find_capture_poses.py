#!/usr/bin/env python3
"""Empirically find capture viewpoints that (a) keep >=3 good markers and
(b) keep the target square inside the raw frame (2026-11-22).

Starts from the current known-good e5 pose; probes bounded joint perturbations
(roll branch preserved ~68), and for each candidate logs markers/rms and the
target's projected pixel. Dry-run each leg; returns to the base pose at the end.
Motion standing-authorized (Pi powered); every leg logged.
"""
import json, sys, time, urllib.request, urllib.error
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw

CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
ROOT='/home/brian/robot-arm-sim'
b=Board(json.load(open(ROOT+'/.private/phase11d/calibration.json')))
JOINTS=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
def post(p,body,t=60):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:120]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)

def evaluate(tag):
    for k in range(6):
        f=grab_frame(timeout=6);raw=to_raw(f)
        rect=b.rectify(raw);sol,d=b.solve(rect)
        if sol is None or len(d)<2:
            time.sleep(0.12);continue
        ids=[int(x.marker_id) for x in d]
        # project e5 centre (14,14,0) into raw frame
        p=b.project(sol,np.array([14.,14.,0.]),0.)
        inside = 0<=p[0]<640 and 0<=p[1]<480
        return {'tag':tag,'markers':len(d),'ids':ids,'rms':round(sol['rms_px'],2),
                'e5_px':[round(float(p[0]),1),round(float(p[1]),1)],'e5_inside':bool(inside),
                'cam':np.array(sol['camera_in_board_mm']).round(1).tolist()}
    return {'tag':tag,'markers':0,'e5_inside':False}

def move_joints(jd):
    body={'mode':'joint','joints':jd,'jaw':100}
    dr=post('/move',{**body,'dry_run':True},t=25)
    if dr.get('detail') or (dr.get('max_joint_delta_deg') or 99)>20:
        return False,dr
    r=post('/move',body,t=60)
    time.sleep(0.9)
    return True,r

def main():
    st=get(BRIDGE+'/state');assert st['fault'] is None,st['fault']
    if st['mode']!='REMOTE':post('/mode',{'mode':'REMOTE'});time.sleep(0.6)
    base={n:st['joints'][n] for n in JOINTS}
    print('base joints',{k:round(v,2) for k,v in base.items()},flush=True)
    roll0=base['wrist_roll']
    base_eval=evaluate('base');print('BASE',base_eval,flush=True)
    # candidate perturbations: (joint, delta). keep roll fixed to preserve branch
    deltas=[('shoulder_pan',-4),('shoulder_pan',4),('shoulder_lift',-4),('shoulder_lift',4),
            ('elbow_flex',-4),('elbow_flex',4),('wrist_flex',-4),('wrist_flex',4),
            ('shoulder_pan',-8),('shoulder_pan',8),('shoulder_lift',-8),('shoulder_lift',8)]
    results=[base_eval]
    for jname,dj in deltas:
        jd=dict(base);jd[jname]=base[jname]+dj
        ok,r=move_joints(jd)
        if not ok:
            print(f'{jname}{dj:+d}: dry FAIL',r.get('detail'),flush=True);continue
        ev=evaluate(f'{jname}{dj:+d}')
        good=ev['markers']>=3 and ev['rms']<8 and ev['e5_inside']
        ev['good']=good
        results.append(ev)
        print(f'{jname}{dj:+d}: markers {ev["markers"]} rms {ev.get("rms")} e5px {ev.get("e5_px")} inside {ev.get("e5_inside")} GOOD={good}',flush=True)
    # return to base
    move_joints(base);post('/mode',{'mode':'HOLD'})
    good=[r for r in results if r.get('good')]
    print(f'\nGOOD POSES {len(good)}:',flush=True)
    for g in good:print(' ',g['tag'],g['e5_px'],'rms',g['rms'],flush=True)
    json.dump({'base':{k:round(v,3) for k,v in base.items()},'results':results},
              open(ROOT+'/reports/capture_pose_search_20261122.json','w'),indent=2)
    print('HOLD; report written',flush=True)

if __name__=='__main__':main()
