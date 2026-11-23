#!/usr/bin/env python3
"""Closed-loop alignment of the FK jaw tip onto the piece using ABSOLUTE xyz.

Brian 2026-11-22: absolute xyz reaches all board points (verified 2-5mm
pos_err); the earlier 'unreachable' was a rel-move/boundary artifact.

Two trustworthy signals:
  - piece board xy from PnP (unproject the bright-piece pixel at z=0)
  - jaw tip board xy from FK keep-in (tip-frame constant validated by the
    closed-jaw floor test reaching z=2.8mm)

error = piece_xy - tip_xy. Command absolute xyz to (tip + gain*error) at a safe
height with JAWS OPEN; re-measure each step. Then stop for image verification.
NO descent, NO jaw close, NO grasp in this phase.

Guards: markers>=3, rms<6, dry-run each leg (pos_err<8, detail not 'no
reachable'), fault/mode/roll-branch preserved. Prints every leg before exec.
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

def sense(retries=8):
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
                cands.append((area,float(cx),float(cy)))
        cands.sort(key=lambda c:-c[0])
        if not cands:
            last={'markers':len(d),'rms':round(sol['rms_px'],2),'reason':'no piece blob'};time.sleep(0.15);continue
        area,cx,cy=cands[0]
        piece=b.unproject(sol,(cx,cy),0.0)[:2]
        tip=np.array(get(CONSOLE+'/keep-in')['tip_mm'][:2])
        tip_px_proj=b.project(sol,np.array([tip[0],tip[1],get(CONSOLE+'/keep-in')['tip_mm'][2]]),
                              get(CONSOLE+'/keep-in')['tip_mm'][2])
        return {'ok':True,'markers':len(d),'rms':round(sol['rms_px'],2),
                'piece_px':[round(cx,1),round(cy,1)],'piece_xy':piece.round(2).tolist(),
                'tip_xy':tip.round(2).tolist(),
                'tip_px_proj':[round(float(tip_px_proj[0]),1),round(float(tip_px_proj[1]),1)],
                'err':(piece-tip).round(2).tolist()}
    return {'ok':False,**(last or {})}

def main():
    st=get(BRIDGE+'/state')
    assert st['fault'] is None, st['fault']
    roll0=st['joints']['wrist_roll']; jaw=st['joints']['gripper']
    print(f'START mode={st["mode"]} fault={st["fault"]} roll={roll0:.2f} jaw={jaw:.1f}',flush=True)
    if st['mode']!='REMOTE':
        print('arm -> REMOTE',post('/mode',{'mode':'REMOTE'}));time.sleep(0.6)
    st=get(BRIDGE+'/state'); assert st['mode']=='REMOTE','failed REMOTE'
    tip0=get(CONSOLE+'/keep-in')['tip_mm']
    print('start tip_mm',tip0,'saved tip px',saved_tip_pixel(),flush=True)

    # Phase A: lift straight up at current xy to a safe transit height
    zsafe=70.0
    lift={'mode':'xyz','x_mm':round(tip0[0],1),'y_mm':round(tip0[1],1),'z_mm':zsafe,'jaw':100}
    dr=post('/move',{**lift,'dry_run':True},t=25)
    print('LIFT dry',dr.get('detail'),'poserr',dr.get('requested',{}).get('position_error_mm'),
          'delta',dr.get('max_joint_delta_deg'),flush=True)
    if dr.get('detail') or (dr.get('requested',{}).get('position_error_mm') or 99)>8:
        print('LIFT ABORT',flush=True);return
    print('LIFT LEG -> EXEC',flush=True)
    r=post('/move',lift,t=60); print('  lift resp',json.dumps(r)[:150],flush=True)
    time.sleep(0.9)
    st=get(BRIDGE+'/state'); print('  after lift fault',st['fault'],'mode',st['mode'],flush=True)
    if st['fault'] or st['mode']!='REMOTE': print('ABORT after lift',flush=True);return

    # Phase B: closed-loop horizontal alignment at safe height
    GAIN=0.75; MAXSTEP=8.0; TOL=2.5; MAXSTEPS=8
    history=[]
    for step in range(MAXSTEPS+1):
        s=sense()
        if not s.get('ok'):
            print(f'step {step}: NO SENSE {s}',flush=True);break
        err=np.array(s['err']); dist=float(np.linalg.norm(err))
        print(f'step {step}: m={s["markers"]} rms={s["rms"]} tip={s["tip_xy"]} '
              f'piece={s["piece_xy"]} err={err.round(1)} |err|={dist:.1f}mm '
              f'piece_px={s["piece_px"]} tip_px_proj={s["tip_px_proj"]}',flush=True)
        history.append({'step':step,**s})
        if dist<=TOL:
            print(f'ALIGNED step {step}: |err|={dist:.1f}mm',flush=True);break
        if step==MAXSTEPS: break
        cur=np.array(s['tip_xy'])
        tgt=cur+np.clip(GAIN*err,-MAXSTEP,MAXSTEP)
        body={'mode':'xyz','x_mm':round(float(tgt[0]),1),'y_mm':round(float(tgt[1]),1),
              'z_mm':zsafe,'jaw':100}
        dr=post('/move',{**body,'dry_run':True},t=25)
        pe=dr.get('requested',{}).get('position_error_mm')
        print('   dry',body,'detail',dr.get('detail'),'poserr',pe,'delta',dr.get('max_joint_delta_deg'),flush=True)
        if dr.get('detail') or (pe is not None and pe>8):
            print('   ABORT dry',flush=True);break
        print('   LEG -> EXEC',flush=True)
        r=post('/move',body,t=60)
        if r.get('http_error'): print('   ABORT exec',r,flush=True);break
        time.sleep(0.8)
        st=get(BRIDGE+'/state')
        if st['fault'] or st['mode']!='REMOTE' or abs(st['joints']['wrist_roll']-roll0)>5:
            print('   ABORT guard',st['fault'],st['mode'],st['joints']['wrist_roll'],flush=True);break

    final=sense()
    print('FINAL',final,flush=True)
    with open('/home/brian/robot-arm-sim/reports/align_abs_20261122.json','w') as fh:
        json.dump({'start_tip':tip0,'zsafe':zsafe,'history':history,'final':final},fh,indent=2)
    # leave in HOLD, jaws open
    post('/mode',{'mode':'HOLD'})
    print('wrote report; HOLD',flush=True)

if __name__=='__main__':main()
