#!/usr/bin/env python3
"""Option A: iterative FK-tip correction loop (Brian 2026-11-22).

The absolute xyz solver's IK is good but the bridge lands the joints ~1-2deg
off, giving a stable ~13mm tip error. Correct it by FEEDBACK: command the tip
to a target, read the ACTUAL FK tip from /keep-in, and re-command the residual
(target - actual) until the tip converges. Uses only FK tip (reliable) - no
camera PnP, so marker visibility at height is irrelevant.

Jaws stay OPEN. No descent, no grasp. Strict guards, every leg logged.

    --target-x/y/z : desired tip board position (mm)
    --steps        : max correction iterations
    --tol-mm       : convergence threshold
"""
import argparse, json, sys, time, urllib.request, urllib.error
import numpy as np
CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
def post(p,body,t=90):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:150]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def tip():
    return np.array(get(CONSOLE+'/keep-in')['tip_mm'],float)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--target-x',type=float,default=0.0)
    ap.add_argument('--target-y',type=float,default=40.0)
    ap.add_argument('--target-z',type=float,default=60.0)
    ap.add_argument('--steps',type=int,default=8)
    ap.add_argument('--tol-mm',type=float,default=2.0)
    ap.add_argument('--settle',type=float,default=1.2)
    a=ap.parse_args()
    target=np.array([a.target_x,a.target_y,a.target_z])

    st=get(BRIDGE+'/state'); assert st['fault'] is None, st['fault']
    roll0=st['joints']['wrist_roll']
    print(f'START mode={st["mode"]} fault={st["fault"]} roll={roll0:.2f} jaw={st["joints"]["gripper"]:.1f}',flush=True)
    print(f'TARGET {target.tolist()} tol={a.tol_mm} steps={a.steps}',flush=True)
    if st['mode']!='REMOTE': print('->REMOTE',post('/mode',{'mode':'REMOTE'}));time.sleep(0.6)
    st=get(BRIDGE+'/state'); assert st['mode']=='REMOTE','not REMOTE'
    print('start tip',tip().round(1).tolist(),flush=True)

    history=[]
    cmd=target.copy()   # accumulate the residual into the commanded point
    for step in range(a.steps+1):
        cur=tip()
        err=target-cur
        dist=float(np.linalg.norm(err))
        print(f'step {step}: actual_tip={cur.round(1).tolist()} cmd={cmd.round(1).tolist()} err={err.round(1).tolist()} |err|={dist:.1f}mm',flush=True)
        history.append({'step':step,'actual':cur.round(2).tolist(),'cmd':cmd.round(2).tolist(),'err':err.round(2).tolist(),'dist':round(dist,2)})
        if dist<=a.tol_mm:
            print(f'CONVERGED step {step}: |err|={dist:.1f}mm',flush=True);break
        if step==a.steps:
            print(f'MAX STEPS reached: |err|={dist:.1f}mm',flush=True);break
        # feedforward correction: shift the command by the measured residual so
        # a repeatable joint-space bias is cancelled on the next move.
        cmd=cmd+err
        body={'mode':'xyz','x_mm':round(float(cmd[0]),1),'y_mm':round(float(cmd[1]),1),
              'z_mm':round(float(cmd[2]),1),'jaw':100}
        dr=post('/move',{**body,'dry_run':True},t=30)
        pe=dr.get('requested',{}).get('position_error_mm')
        print('   dry detail',dr.get('detail'),'poserr',pe,'delta',dr.get('max_joint_delta_deg'),flush=True)
        if dr.get('detail') or (pe is not None and pe>8):
            print('   ABORT dry',flush=True);break
        print('   LEG ->',body,flush=True)
        r=post('/move',body,t=90)
        if r.get('http_error') or r.get('detail'):
            print('   ABORT exec',r,flush=True);break
        time.sleep(a.settle)
        st=get(BRIDGE+'/state')
        if st['fault'] or st['mode']!='REMOTE' or abs(st['joints']['wrist_roll']-roll0)>5:
            print('   ABORT guard',st['fault'],st['mode'],flush=True);break
    final=tip()
    print('FINAL tip',final.round(1).tolist(),'|err|',round(float(np.linalg.norm(target-final)),1),flush=True)
    with open('/home/brian/robot-arm-sim/reports/tip_correct_loop_20261122.json','w') as fh:
        json.dump({'target':target.tolist(),'history':history,'final':final.round(2).tolist()},fh,indent=2)
    post('/mode',{'mode':'HOLD'})
    print('HOLD; report written',flush=True)

if __name__=='__main__':main()
