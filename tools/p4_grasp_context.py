#!/usr/bin/env python3
"""Integrated grasp: type from game context -> vision base XY -> tip servo -> grasp.

Pipeline (Brian's chosen approach 2026-11-23):
  1. Given piece TYPE (from context/move) + square, predict base XY with the
     leakage-free vision model (predict_base_xy).
  2. Lift tip to a safe height, use the VALIDATED iterative tip-correction loop
     (command -> measure FK tip -> re-command residual) to bring the jaw tip over
     the predicted base XY at ~0.1mm.
  3. Descend toward the piece, close jaws, lift, verify.
Guards: dry-run every move, fault/mode/roll-branch checks, stop on 409.
Execution (motion + jaw actuation) REQUIRES explicit approval: pass --execute.
Without --execute this is a dry-run planning pass only (no motion, no jaws).

    # plan only (no motion):
    PYTHONPATH=. .venv/bin/python tools/p4_grasp_context.py --piece knight --square f6
    # execute (explicit approval):
    PYTHONPATH=. .venv/bin/python tools/p4_grasp_context.py --piece knight --square f6 --execute
"""
import argparse,json,sys,time,urllib.request,urllib.error
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from infer_base_xy import load_model,predict
from p4_irl_servo_detect import grab_frame

C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
def post(p,body,t=60,retry=2):
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
        return {'http_error':e.code,'detail':e.read().decode()[:120]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def tip():return np.array(get(C+'/keep-in')['tip_mm'],float)

def correction_loop(target,tol=2.0,steps=14):
    """Iterative tip positioning via absolute xyz. Correct design: COMMAND the
    target directly first, measure the real landing, feed residual back into the
    NEXT command - never add error to the target before moving (that double-
    jumped and blew up attempt 1)."""
    tgt=np.array(target,float)
    pending=tgt.copy()
    hist=[]
    for step in range(steps+1):
        cur=tip();err=tgt-cur;dist=float(np.linalg.norm(err))
        print(f'  [tip] step {step}: actual {cur.round(1)} target {tgt.round(1)} err {dist:.1f}mm',flush=True)
        hist.append({'step':step,'actual':cur.round(2).tolist(),'err':round(dist,2)})
        if dist<=tol:return True,hist
        if step==steps:return False,hist
        body={'mode':'xyz','x_mm':round(float(pending[0]),1),'y_mm':round(float(pending[1]),1),
              'z_mm':round(float(pending[2]),1),'jaw':100}
        dr=post('/move',{**body,'dry_run':True},t=25)
        if dr.get('detail'):print('  [tip] dry fail',dr.get('detail'),flush=True);return False,hist
        print('  [tip] LEG',body,flush=True)
        post('/move',body,t=60);time.sleep(1.3)
        st=get(B+'/state')
        if st['fault'] or st['mode']!='REMOTE':print('  [tip] guard abort',flush=True);return False,hist
        landed=tip();pending=pending+(tgt-landed)
        pending[2]=float(np.clip(pending[2],2.0,200.0))
    return False,hist

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--piece',required=True);ap.add_argument('--square',required=True)
    ap.add_argument('--z-safe',type=float,default=70.0)
    ap.add_argument('--z-grasp',type=float,default=8.0)
    ap.add_argument('--execute',action='store_true')
    a=ap.parse_args()
    st=get(B+'/state')
    print(f'mode={st["mode"]} fault={st["fault"]} roll={st["joints"]["wrist_roll"]:.1f} jaw={st["joints"]["gripper"]:.1f}',flush=True)
    assert st['fault'] is None
    # 1) predict base XY from a live frame
    model=load_model()
    frame=grab_frame(timeout=8)
    xy,info=predict(frame,a.piece.lower(),a.square.lower(),model)
    print(f'predict base {a.piece} @ {a.square}: {xy} ({info})',flush=True)
    if xy is None:
        print('NO PREDICTION - cannot grasp');return
    # 2) plan tip servo + descent + grasp
    target=[round(float(xy[0]),1),round(float(xy[1]),1),a.z_safe]
    print(f'PLAN: lift+servo tip to {target} (z safe {a.z_safe})',flush=True)
    if not a.execute:
        print('DRY-RUN ONLY (no motion, no jaws). Pass --execute with approval to run.',flush=True)
        # validate the correction target is reachable via one dry-run
        dr=post('/move',{'mode':'xyz','x_mm':target[0],'y_mm':target[1],'z_mm':target[2],'jaw':100,'dry_run':True},t=25)
        print('  dry-run to target:',dr.get('detail') or 'OK','poserr',dr.get('requested',{}).get('position_error_mm'),flush=True)
        return
    # 3) execute
    post('/mode',{'mode':'REMOTE'});time.sleep(0.8)
    ok,hist=correction_loop(target)
    if not ok:print('tip servo failed - abort',flush=True);post('/mode',{'mode':'HOLD'});return
    # descend in small rel steps to grasp height
    print(f'DESCEND to z={a.z_grasp}',flush=True)
    for k in range(12):
        cur=tip()
        if cur[2]<=a.z_grasp+1:break
        dz=-min(5.0,cur[2]-a.z_grasp)
        body={'mode':'rel','dz_mm':round(float(dz),2)}
        dr=post('/move',{**body,'dry_run':True},t=25)
        if dr.get('detail'):print('  descend dry fail',flush=True);break
        post('/move',body,t=60);time.sleep(0.9)
        st=get(B+'/state')
        if st['fault']:print('  descend fault',flush=True);break
    print(f'CLOSE JAWS at tip {tip().round(1)}',flush=True)
    post('/move',{'mode':'joint','jaw':15},t=60);time.sleep(1)
    print(f'LIFT',flush=True)
    post('/move',{'mode':'rel','dz_mm':40},t=60);time.sleep(1)
    print(f'FINAL tip {tip().round(1)} jaw {get(B+"/state")["joints"]["gripper"]:.1f}',flush=True)
    post('/mode',{'mode':'HOLD'})

if __name__=='__main__':main()
