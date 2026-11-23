#!/usr/bin/env python3
"""Closed-loop: servo the CLOSED-loop jaw tip (FK board) onto the piece (PnP board).

Alignment phase only - JAWS STAY OPEN. No descent, no grasp.

Error = piece_board_mm - tip_board_mm, applied as `/move rel` in board axes
(the rel primitive translates the tool/camera together; see p4_irl_servo.py).
Re-measures each step, so parallax/mount error is absorbed by the loop.

Verification: at the pose where the tip pixel was set, a correct alignment
should drive the PIECE pixel to the saved TIP pixel (320,152). If it does not,
FK and the by-eye tip calibration disagree and we stop before grasping.

Guards: markers>=3, rms<6, fault, mode, roll branch preserved, per-step max
joint delta, dry-run each leg. Prints every leg before executing.
"""
import json, sys, time, urllib.request, urllib.error
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw, saved_tip_pixel

CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

def post(p,body,t=60):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:200]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)

def sense(retries=6):
    last=None
    for k in range(retries):
        f=grab_frame(timeout=6); raw=to_raw(f)
        rect=b.rectify(raw); sol,d=b.solve(rect)
        if sol is None or len(d)<3 or sol['rms_px']>6:
            last={'markers':len(d),'rms':round(sol['rms_px'],2) if sol else None};time.sleep(0.15);continue
        grey=cv2.cvtColor(raw,cv2.COLOR_BGR2GRAY)
        p5=b.project(sol,np.array([14.,14.,0.]),0.)
        bright=(grey>170).astype(np.uint8)
        bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((7,7),np.uint8))
        n,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
        cands=[]
        for i in range(1,n):
            x,y,w,h,area=stats[i];cx,cy=cents[i]
            if area>200 and np.hypot(cx-p5[0],cy-p5[1])<90 and h<180:
                cands.append((area,cx,cy))
        cands.sort(key=lambda c:-c[0])
        if not cands:
            last={'markers':len(d),'rms':round(sol['rms_px'],2),'reason':'no piece blob'};time.sleep(0.15);continue
        area,cx,cy=cands[0]
        piece_mm=b.unproject(sol,(cx,cy),0.)
        tip=np.array(get(CONSOLE+'/keep-in')['tip_mm'][:2])
        return {'ok':True,'markers':len(d),'rms':round(sol['rms_px'],2),
                'piece_px':[round(float(cx),1),round(float(cy),1)],
                'piece_mm':piece_mm.round(2).tolist(),'tip_mm':tip.round(2).tolist(),
                'err_mm':(piece_mm[:2]-tip).round(2).tolist()}
    return {'ok':False,**(last or {})}

def main():
    st=get(BRIDGE+'/state')
    assert st['fault'] is None, st['fault']
    roll0=st['joints']['wrist_roll']
    jaw=st['joints']['gripper']
    print(f'START mode={st["mode"]} fault={st["fault"]} roll={roll0:.2f} jaw={jaw:.1f}',flush=True)
    if st['mode']!='REMOTE':
        print('arm -> REMOTE',post('/mode',{'mode':'REMOTE'}));time.sleep(0.6)
    tip_px=saved_tip_pixel()
    print('saved tip pixel',tip_px,flush=True)
    GAIN=0.75; MAXSTEP=10.0; TOL=3.0; MAXSTEPS=10
    history=[]
    for step in range(MAXSTEPS+1):
        s=sense()
        if not s.get('ok'):
            print(f'step {step}: NO SENSE {s}',flush=True);break
        err=np.array(s['err_mm']);dist=float(np.linalg.norm(err))
        pc=np.array(s['piece_px']);tp=np.array(tip_px)
        pixdist=float(np.linalg.norm(pc-tp))
        print(f'step {step}: m={s["markers"]} rms={s["rms"]} tip={s["tip_mm"]} '
              f'piece={s["piece_mm"]} err={err.round(1)} |err|={dist:.1f}mm '
              f'piece_px={s["piece_px"]} px_to_tip={pixdist:.0f}px',flush=True)
        history.append({'step':step,**s,'pixdist':round(pixdist,1)})
        if dist<=TOL:
            print(f'ALIGNED step {step}: |err|={dist:.1f}mm, piece px->tip {pixdist:.0f}px',flush=True)
            break
        if step==MAXSTEPS: break
        step_xy=np.clip(GAIN*err,-MAXSTEP,MAXSTEP)
        body={'mode':'rel','dx_mm':round(float(step_xy[0]),2),'dy_mm':round(float(step_xy[1]),2),'dz_mm':0.0}
        dr=post('/move',{**body,'dry_run':True},t=25)
        md=dr.get('max_joint_delta_deg')
        print('   dry',body,'max_delta',md,flush=True)
        if dr.get('http_error') or md is None or md>15:
            print('   ABORT dry-run',dr,flush=True);break
        print('   LEG -> EXEC',flush=True)
        r=post('/move',body,t=60)
        if r.get('http_error'):print('   ABORT exec',r,flush=True);break
        time.sleep(0.7)
        st=get(BRIDGE+'/state')
        if st['fault'] or st['mode']!='REMOTE' or abs(st['joints']['wrist_roll']-roll0)>3:
            print('   ABORT guard',st['fault'],st['mode'],flush=True);break
    else:
        print('no sense',flush=True)
    final=sense()
    print('FINAL',final,flush=True)
    get(CONSOLE+'/mode')  # keep REMOTE for next phase
    out='/home/brian/robot-arm-sim/reports/tip_align_20261122.json'
    with open(out,'w') as fh: json.dump({'tip_pixel':tip_px,'history':history,'final':final},fh,indent=2)
    print('wrote',out,flush=True)

if __name__=='__main__':main()
