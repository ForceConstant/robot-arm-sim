#!/usr/bin/env python3
"""Find a joint-offset pose that frames g5 (board 70,14) with good PnP.
Probes bounded offsets from BASE; logs target pixel + markers + rms."""
import json,sys,time,urllib.request,urllib.error
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
def post(p,body,t=50,retry=1):
    req=urllib.request.Request(C+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:
        if retry and e.code==409:
            time.sleep(0.8)
            try:
                rr=urllib.request.Request(C+'/mode',data=b'{"mode":"REMOTE"}',method='POST',headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(rr,timeout=10) as r:r.read()
            except Exception:pass
            time.sleep(0.8);return post(p,body,t=t,retry=0)
        return {'http_error':e.code}
def get(u):
    with urllib.request.urlopen(u,timeout=8) as r:return json.load(r)
BASE=dict(shoulder_pan=-19.78,shoulder_lift=-84.09,elbow_flex=91.56,wrist_flex=61.05,wrist_roll=67.34)
def go(jd):
    if get(B+'/state')['mode']!='REMOTE':post('/mode',{'mode':'REMOTE'});time.sleep(1)
    dr=post('/move',{**{'mode':'joint','joints':jd,'jaw':100},'dry_run':True},t=25)
    md=dr.get('max_joint_delta_deg')
    if dr.get('detail') or md is None or md>20:return False
    post('/move',{'mode':'joint','joints':jd,'jaw':100});time.sleep(1.5);return True
def check():
    for k in range(8):
        try:f=grab_frame(timeout=6)
        except Exception:time.sleep(0.3);continue
        raw=to_raw(f);rect=b.rectify(raw);sol,d=b.solve(rect)
        if sol is None or len(d)<2:time.sleep(0.15);continue
        p=b.project(sol,np.array([70.,14.,0.]),0.)  # g5
        inside=0<=p[0]<640 and 0<=p[1]<480
        return len(d),round(sol['rms_px'],2),[round(float(p[0]),0),round(float(p[1]),0)],inside
    return None
post('/mode',{'mode':'REMOTE'});time.sleep(1)
print('BASE g5',check(),flush=True)
# offsets: pan toward +x (shoulder_pan negative moves gaze right in past tests)
cands=[('pan_m4',{'shoulder_pan':-4}),('pan_m8',{'shoulder_pan':-8}),
       ('pan_p4',{'shoulder_pan':4}),('lift_p4',{'shoulder_lift':4}),('lift_p8',{'shoulder_lift':8}),
       ('pan_m4_lift_p4',{'shoulder_pan':-4,'shoulder_lift':4}),
       ('wflex_p6',{'wrist_flex':6}),('wflex_m6',{'wrist_flex':-6})]
good=[]
for name,off in cands:
    jd=dict(BASE)
    for k,v in off.items():jd[k]=BASE[k]+v
    if not go(jd):
        print(name,'move fail',flush=True);continue
    r=check()
    ok=r and r[3] and r[0]>=2 and r[1]<6
    print(name,off,'->',r,'GOOD' if ok else '',flush=True)
    if ok:good.append((name,off,r))
    go(BASE)
post('/mode',{'mode':'HOLD'})
print('GOOD',[g[0] for g in good],flush=True)
json.dump([{'name':n,'offset':o,'check':c} for n,o,c in good],
          open('/home/brian/robot-arm-sim/reports/g5_pose_search.json','w'),indent=2)
print('HOLD',flush=True)
