#!/usr/bin/env python3
"""IRL mount-free gaze servo onto a target square (2026-11-22).

Everything in the perception + error path is mount-free:
  - Board.solve gives camera pose from marker PnP (no FK, no camera_in_tool)
  - grid() projects square centres to pixels via that PnP
  - unproject() maps image centre and the target square's pixel to BOARD mm
  - error = target_board_mm - gaze_board_mm

The motion primitive is `/move rel` with dx_mm,dy_mm in BOARD axes. rel uses
chain FK + board axes only (no camera_in_tool), translating the tool and hence
the wrist camera by the same board-frame delta. Closed loop absorbs mount/kinematic
error by re-measuring each step. NO GRIPPER ACTUATION.

    # dry run (no motion):
    PYTHONPATH=. .venv/bin/python tools/p4_irl_servo.py --square e5 --dry-run
    # live:
    PYTHONPATH=. .venv/bin/python tools/p4_irl_servo.py --square e5 --steps 8
"""
from __future__ import annotations
import argparse, json, math, sys, time
from pathlib import Path
import cv2, numpy as np, urllib.request, urllib.error

ROOT=Path('/home/brian/robot-arm-sim')
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tools'))
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame

CONSOLE='http://192.168.1.171:8080'
BRIDGE='http://192.168.1.167:8100'
CAL=ROOT/'.private/phase11d/calibration.json'

def post(path,body,timeout=20):
    data=json.dumps(body).encode()
    req=urllib.request.Request(CONSOLE+path,data=data,method='POST',
                               headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=timeout) as r:
            return json.loads(r.read().decode() or '{}')
    except urllib.error.HTTPError as e:
        return {'http_error':e.code,'detail':e.read().decode()[:200]}

def get(path,timeout=10):
    with urllib.request.urlopen(CONSOLE+path,timeout=timeout) as r:
        return json.loads(r.read().decode())

def bridge_state():
    with urllib.request.urlopen(BRIDGE+'/state',timeout=8) as r:
        return json.loads(r.read().decode())

def square_centre_mm(sq):
    return np.array([((ord(sq[0])-97)+0.5)*28.0-112.0,(int(sq[1])-1+0.5)*28.0-112.0])

def sense(board,square,retries=6):
    """Retry the MJPEG stream until a valid PnP pose with the target visible."""
    last={'markers':0,'reason':'no frame'}
    for k in range(retries):
        try:
            frame=grab_frame(timeout=6.0)
        except Exception as e:
            last={'markers':0,'reason':f'grab: {e}'};continue
        rect=board.rectify(frame)
        sol,dets=board.solve(rect)
        if sol is None:
            last={'markers':len(dets),'reason':'no pose'};continue
        grid=board.grid(sol)
        centre=next((np.array(c['uv'],float) for c in grid['centres'] if c['name']==square),None)
        if centre is None:
            last={'markers':len(dets),'reason':'target square not projected','rms':sol['rms_px']};continue
        last=None
        break
    if last is not None:
        return None,last
    # unproject image centre (gaze) and the target square pixel to board plane z=0
    gaze=board.unproject(sol,(320.0,240.0),0.0)
    tgt=board.unproject(sol,centre,0.0)
    err=tgt-gaze
    info={'markers':len(dets),'rms':round(sol['rms_px'],2),
          'gaze_board_mm':np.round(gaze,1).tolist(),
          'target_board_mm':np.round(tgt,1).tolist(),
          'target_px':np.round(centre,1).tolist()}
    return err,info

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--square',default='e5')
    ap.add_argument('--steps',type=int,default=8)
    ap.add_argument('--gain',type=float,default=0.6)
    ap.add_argument('--max-step-mm',type=float,default=20.0)
    ap.add_argument('--tol-mm',type=float,default=4.0)
    ap.add_argument('--dry-run',action='store_true')
    ap.add_argument('--go-remote',action='store_true',
                    help='switch bridge to REMOTE before motion (else require already REMOTE)')
    args=ap.parse_args()

    board=Board(json.loads(CAL.read_text()))
    st=bridge_state()
    print(f"bridge mode {st['mode']} armed {st['armed']} fault {st['fault']} "
          f"roll {round(st['joints']['wrist_roll'],2)} seq {st['last_seq']}")
    if st['mode']!='HOLD' and st['mode']!='REMOTE':
        raise SystemExit('bridge must be HOLD or REMOTE')
    if not st['armed'] or st['fault']:
        raise SystemExit('bridge not armed or has fault')

    if not args.dry_run:
        if st['mode']!='REMOTE':
            if not args.go_remote:
                raise SystemExit('bridge not REMOTE; pass --go-remote to switch')
            print('arm -> REMOTE:',post('/mode',{'mode':'REMOTE'}))
            time.sleep(0.5)
        st=bridge_state()
        if st['mode']!='REMOTE':
            raise SystemExit('failed to enter REMOTE')

    history=[]
    for step in range(args.steps+1 if not args.dry_run else 3):
        err,info=sense(board,args.square)
        if err is None:
            print(f"step {step}: no gaze ({info})");history.append({'step':step,'error_mm':None,**info});break
        dist=float(np.linalg.norm(err))
        print(f"step {step}: markers {info['markers']} rms {info['rms']} "
              f"gaze {info['gaze_board_mm']} target {info['target_board_mm']} "
              f"err {np.round(err,1)} |err| {dist:.2f}mm")
        history.append({'step':step,'error_mm':round(dist,2),**info})
        if dist<=args.tol_mm:
            print(f"ON TARGET step {step}: {dist:.2f}mm");break
        if args.dry_run or step==args.steps:
            break
        step_xy=np.clip(args.gain*err,-args.max_step_mm,args.max_step_mm)
        body={'mode':'rel','dx_mm':round(float(step_xy[0]),2),'dy_mm':round(float(step_xy[1]),2),'dz_mm':0.0}
        print('   rel',body)
        resp=post('/move',body,timeout=60)
        print('   ->',json.dumps(resp)[:400])
        if 'http_error' in resp or (isinstance(resp,dict) and resp.get('status')=='error'):
            print('   MOVE FAILED - stopping');break
        # guard: branch flip / fault / mode change
        time.sleep(0.4)
        st=bridge_state()
        if st['fault'] or st['mode']!='REMOTE':
            print('   bridge fault/mode change - stopping');break

    out=ROOT/'reports'/f'irl_servo_{args.square}.json'
    out.write_text(json.dumps({'schema':'irl-gaze-servo/1','square':args.square,
                               'dry_run':args.dry_run,'history':history},indent=2)+'\n')
    print('wrote',out)

if __name__=='__main__':
    main()
