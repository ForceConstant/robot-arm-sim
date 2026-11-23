#!/usr/bin/env python3
"""Find joint-offset poses that FRAME skipped pieces well (2026-11-23).

For each target square, probe candidate joint offsets from BASE; score by the
target's projected pixel and its margin from frame edges (a whole piece needs
~60px margin since tall pieces extend beyond their base pixel). Record poses
that place the base pixel with >=80px margin and have valid PnP (rms<6).

    PYTHONPATH=. .venv/bin/python -u tools/p4_find_framing_poses.py
"""
import json,sys,time,urllib.request,urllib.error
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
def post(p,body,t=50,retry=2):
    req=urllib.request.Request(C+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:
        if retry and e.code==409:
            time.sleep(0.7)
            try:
                rr=urllib.request.Request(C+'/mode',data=b'{"mode":"REMOTE"}',method='POST',headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(rr,timeout=10) as r:r.read()
            except Exception:pass
            time.sleep(0.7);return post(p,body,t=t,retry=retry-1)
        return {'http_error':e.code}
def get(u):
    with urllib.request.urlopen(u,timeout=8) as r:return json.load(r)
BASE=dict(shoulder_pan=-19.78,shoulder_lift=-84.09,elbow_flex=91.56,wrist_flex=61.05,wrist_roll=67.34)
CANDIDATES=[('base',{}),('lift_m8',{'shoulder_lift':-8}),('lift_p4',{'shoulder_lift':4}),
            ('lift_m12',{'shoulder_lift':-12}),('lift_m16',{'shoulder_lift':-16}),
            ('pan_m4',{'shoulder_pan':-4}),('pan_p4',{'shoulder_pan':4}),
            ('wflex_m6',{'wrist_flex':-6}),('wflex_p6',{'wrist_flex':6}),
            ('elbow_m6',{'elbow_flex':-6}),('lift_m8_pan_m4',{'shoulder_lift':-8,'shoulder_pan':-4}),
            ('lift_m8_pan_p4',{'shoulder_lift':-8,'shoulder_pan':4})]
TARGETS={'bishop':('c3',-42,-42),'king':('e3',14,-42),'rook':('d4',-14,-14)}
def go(jd):
    if get(B+'/state')['mode']!='REMOTE':post('/mode',{'mode':'REMOTE'});time.sleep(1)
    dr=post('/move',{**{'mode':'joint','joints':jd,'jaw':100},'dry_run':True},t=25)
    md=dr.get('max_joint_delta_deg')
    if dr.get('detail') or md is None or md>35:return False
    post('/move',{'mode':'joint','joints':jd,'jaw':100});time.sleep(2.0);return True
def probe(x,y):
    for k in range(6):
        try:f=grab_frame(timeout=6)
        except Exception:time.sleep(0.3);continue
        raw=to_raw(f);rect=b.rectify(raw);sol,d=b.solve(rect)
        if sol is None or len(d)<2 or sol['rms_px']>6:continue
        p=b.project(sol,np.array([x,y,0.]),0.)
        px,py=float(p[0]),float(p[1])
        margin=min(px,640-px,py,480-py)
        return round(px),round(py),round(margin),len(d),round(sol['rms_px'],2)
    return None
def main():
    post('/mode',{'mode':'REMOTE'});time.sleep(1)
    results={}
    for piece,(sq,x,y) in TARGETS.items():
        print(f'\n=== {piece} @ {sq} ({x},{y}) ===',flush=True)
        good=[]
        for name,off in CANDIDATES:
            jd=dict(BASE)
            for k,v in off.items():jd[k]=BASE[k]+v
            if not go(jd):print(f'  {name}: move fail',flush=True);continue
            r=probe(x,y)
            if not r:print(f'  {name}: no PnP',flush=True);continue
            px,py,margin,nd,rms=r
            ok=margin>=70
            print(f'  {name:16s} px({px:4d},{py:4d}) margin={margin:4d} rms={rms} {"GOOD" if ok else ""}',flush=True)
            if ok:good.append({'name':name,'offset':off,'px':[px,py],'margin':margin,'rms':rms})
        results[piece]=good
        go(BASE)
    post('/mode',{'mode':'HOLD'})
    print('\nGOOD FRAMING POSES:',flush=True)
    for piece,good in results.items():
        print(f'  {piece}: {[g["name"] for g in good]}',flush=True)
    json.dump(results,open('/home/brian/robot-arm-sim/reports/framing_poses_20261123.json','w'),indent=2)
    print('HOLD',flush=True)
if __name__=='__main__':main()
