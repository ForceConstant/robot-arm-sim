#!/usr/bin/env python3
"""From clean base, move to each of the 5 candidate view poses individually,
check PnP quality + e5-in-frame, return to base between each. Reports which
poses actually work as first-move capture views."""
import json,sys,time,urllib.request
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
import urllib.error
def post(p,body,t=50,retry=1):
    req=urllib.request.Request(C+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:
        if retry and e.code==409:
            # mode race: re-assert REMOTE and retry once
            time.sleep(0.8)
            try:
                with urllib.request.urlopen(urllib.request.Request(C+'/mode',data=b'{"mode":"REMOTE"}',method='POST',headers={'Content-Type':'application/json'}),timeout=10) as r: r.read()
            except Exception: pass
            time.sleep(0.8)
            return post(p,body,t=t,retry=0)
        return {'http_error':e.code}
def get(u):
    with urllib.request.urlopen(u,timeout=8) as r:return json.load(r)
BASE=dict(shoulder_pan=-19.78,shoulder_lift=-84.09,elbow_flex=91.56,wrist_flex=61.05,wrist_roll=67.34)
VIEWS={'pan_m4':{'shoulder_pan':-4},'lift_p4':{'shoulder_lift':4},
       'wflex_m4':{'wrist_flex':-4},'wflex_p4':{'wrist_flex':4},'lift_m8':{'shoulder_lift':-8}}
def go(jd):
    # ensure REMOTE right before each move (avoids stale-mode 409)
    if get(B+'/state')['mode']!='REMOTE':
        post('/mode',{'mode':'REMOTE'});time.sleep(1.0)
    r=post('/move',{'mode':'joint','joints':jd,'jaw':100})
    time.sleep(2.0)
    return r
def check():
    for k in range(8):
        try:f=grab_frame(timeout=6)
        except Exception:time.sleep(0.3);continue
        raw=to_raw(f);rect=b.rectify(raw);sol,d=b.solve(rect)
        if sol is None or len(d)<2:time.sleep(0.2);continue
        p=b.project(sol,np.array([14.,14.,0.]),0.)
        return len(d),round(sol['rms_px'],2),[round(float(p[0]),0),round(float(p[1]),0)],0<=p[0]<640 and 0<=p[1]<480
    return None
post('/mode',{'mode':'REMOTE'});time.sleep(1.5)
print('MODE',get(B+'/state')['mode'],'BASE',check(),flush=True)
for name,off in VIEWS.items():
    jd=dict(BASE);jd.update(off);go(jd)
    r=check()
    print(name,off,'->',r,flush=True)
    go(BASE)
post('/mode',{'mode':'HOLD'})
print('HOLD',flush=True)
