#!/usr/bin/env python3
"""Probe camera POSITIONS: move jaw tip to board points via absolute xyz and
measure (via PnP) which board point lands at image centre. Finds positions whose
natural view covers far squares c3/e3 (which wrist tilt couldn't frame).
Motion standing-authorized (Pi powered); dry-run each leg; jaws open.
"""
import json,sys,time,urllib.request,urllib.error
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
def post(p,body,t=60,retry=1):
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
            time.sleep(0.8);return post(p,body,t=t,retry=retry-1)
        return {'http_error':e.code,'detail':e.read().decode()[:100]}
def get(u):
    with urllib.request.urlopen(u,timeout=8) as r:return json.load(r)
def measure():
    for k in range(6):
        try:f=grab_frame(timeout=6)
        except Exception:time.sleep(0.3);continue
        raw=to_raw(f);sol,d=b.solve(b.rectify(raw))
        if sol is None or len(d)<2 or sol['rms_px']>8:continue
        # board point at image centre and at target squares
        cen=b.unproject(sol,(320,240),0.)
        c3=b.project(sol,np.array([-42.,-42.,0.]),0.)
        e3=b.project(sol,np.array([14.,-42.,0.]),0.)
        e5=b.project(sol,np.array([14.,14.,0.]),0.)
        return {'ids':len(d),'rms':round(sol['rms_px'],2),
                'centre_board':cen.round(0).tolist(),
                'c3_px':c3.round(0).tolist(),'e3_px':e3.round(0).tolist(),
                'e5_px':e5.round(0).tolist(),
                'cam':np.array(sol['camera_in_board_mm']).round(0).tolist()}
    return None
def main():
    st=get(B+'/state');assert st['fault'] is None
    roll0=st['joints']['wrist_roll']
    post('/mode',{'mode':'REMOTE'});time.sleep(1)
    # target tip positions: move camera to stand over different board areas
    # tip x/y at z=60 (safe height), jaw open
    targets=[('e5',14,14),('d4',-14,-14),('c4',-14,0),('c3',-42,-42),
             ('e3',14,-42),('d3',-14,-42),('c5',-42,14),('centre',0,0)]
    results=[]
    for name,x,y in targets:
        body={'mode':'xyz','x_mm':x,'y_mm':y,'z_mm':60,'jaw':100}
        dr=post('/move',{**body,'dry_run':True},t=25)
        pe=dr.get('requested',{}).get('position_error_mm')
        if dr.get('detail') or (pe is not None and pe>10):
            print(f'{name}: dry fail {dr.get("detail")} pe={pe}',flush=True);continue
        print(f'{name}: cmd tip ({x},{y},60) ->',flush=True)
        r=post('/move',body,t=60)
        if r.get('http_error'):print('  exec fail',flush=True);continue
        time.sleep(1.5)
        j=get(B+'/state')
        if j['fault'] or abs(j['joints']['wrist_roll']-roll0)>5:
            print('  guard abort',flush=True);break
        m=measure()
        if not m:print('  no PnP',flush=True);continue
        print(f'  centre_board={m["centre_board"]} c3_px={m["c3_px"]} e3_px={m["e3_px"]} rms={m["rms"]}',flush=True)
        results.append({'name':name,'tip_target':[x,y,60],**m})
    post('/mode',{'mode':'HOLD'})
    json.dump(results,open('/home/brian/robot-arm-sim/reports/camera_positions_20261123.json','w'),indent=2)
    print('HOLD; report written',flush=True)
if __name__=='__main__':main()
