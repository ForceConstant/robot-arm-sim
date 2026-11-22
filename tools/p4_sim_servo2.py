#!/usr/bin/env python3
"""Sim close-then-servo, faithful to the IRL e5 scenario (2026-11-22).

Piece sits on the known target square. Servo the wrist camera so its board-plane
projection lands on the piece centroid. BOTH quantities come from marker PnP
(mount-free): camera_in_board_mm from Board.solve, piece centroid from unproject.
Close the loop by moving the camera by the board-frame residual each step.

Even if the motion primitive (CameraIK in sim / `rel` IRL) has mount error,
closed feedback converges because we re-measure the actual residual each step.
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
import cv2, mujoco, numpy as np

ROOT=Path('/home/brian/robot-arm-sim')
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tools'))
from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.product import layout_env_config
from chess_sim.camera_ik import CameraIK
from p1_piece_detect import Board, square_table
from p1_sim_pickup import Poser, sim_intrinsics, piece_at_square

REFERENCE=ROOT/'chess_sim/assets/scene/generated/board_charuco.png'

def square_centre_mm(square):
    return np.array([((ord(square[0])-97)+0.5)*28.0-112.0,
                     (int(square[1])-1+0.5)*28.0-112.0])

def board_axes(env):
    c=math.cos(float(env.board.yaw)); s=math.sin(float(env.board.yaw))
    return np.array([c,s,0.0]), np.array([-s,c,0.0])

def local_piece_xy(frame, rect, valid, n, pxm, expected, board, sol, square):
    """Deviation blob on the known target square; tolerant area floor for pawns."""
    import cv2, numpy as np, math
    half=int(round(14*pxm)); trim=int(round(3*pxm))
    col=ord(square[0])-97; rank=int(square[1])-1
    centre_mm=((col+0.5)*28.0-112.0,(rank+0.5)*28.0-112.0)
    u=int(round((centre_mm[0]+128.0)*pxm)); v=int(round((128.0-centre_mm[1])*pxm))
    diff=cv2.absdiff(rect,expected)
    mask=((diff>60)&(valid>0)).astype(np.uint8)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    count,labels,stats,cents=cv2.connectedComponentsWithStats(mask,8)
    best=None
    for i in range(1,count):
        area=stats[i,cv2.CC_STAT_AREA]/pxm**2
        if not 60.0<=area<=2500.0: continue
        cx_mm=cents[i][0]/pxm-128.0; cy_mm=128.0-cents[i][1]/pxm
        d=math.hypot(cx_mm-centre_mm[0],cy_mm-centre_mm[1])
        if d>20.0: continue
        if best is None or area>best[0]: best=(area,np.array([cx_mm,cy_mm]))
    return (None if best is None else best[1])

def sense(env, board, reference, square, want_piece=True):
    frame=cv2.cvtColor(env.render(Camera.WRIST)[:,:,:3],cv2.COLOR_RGB2BGR)
    sol,dets=board.solve(frame)
    if sol is None:
        return None,{"markers":len(dets),"reason":"no pose"}
    cam_board=np.asarray(sol['camera_in_board_mm'],float)[:2]
    grey=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    rect,valid,n,pxm=board.rectified(grey,sol,4.0)
    expected=cv2.resize(reference,(n,n),interpolation=cv2.INTER_AREA)
    rows=square_table(frame,expected,rect,valid,n,pxm,board.layout,board,sol)
    info={"markers":len(dets),"rms":round(sol['rms_px'],2),
          "cam_board_mm":cam_board.round(1).tolist(),"why":None}
    if want_piece:
        target=local_piece_xy(frame,rect,valid,n,pxm,expected,board,sol,square)
        if target is None:
            info['why']='no deviating blob on square'
            return None,info
    else:
        target=square_centre_mm(square)
    info['target_board_mm']=target.round(1).tolist()
    return target-cam_board,info   # board-mm error: move camera onto target

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--square',default='e5')
    ap.add_argument('--steps',type=int,default=10)
    ap.add_argument('--offset-mm',type=float,default=25.0,
                    help='start camera this far (board mm) from the piece')
    ap.add_argument('--tol-mm',type=float,default=3.0)
    ap.add_argument('--max-step-mm',type=float,default=20.0)
    ap.add_argument('--gain',type=float,default=0.5)
    args=ap.parse_args()

    config=layout_env_config()
    env=ChessSimEnv(config)
    # One white pawn on the target square.
    col=ord(args.square[0])-97; rank=int(args.square[1])-1
    bd=['']*64; bd[(7-rank)*8+col]='P'
    fen=''
    for r in range(8):
        line=''; empty=0
        for c in range(8):
            ch=bd[r*8+c]
            if ch=='': empty+=1
            else:
                if empty: line+=str(empty);empty=0
                line+=ch
        if empty: line+=str(empty)
        fen+=line+('/' if r<7 else ' ')
    fen+='w - - 0 1'
    env.reset(fen)
    slot=env.slot_at(args.square)
    truth=env.piece_position(slot)
    centre=square_centre_mm(args.square)
    print(f"target {args.square} centre {centre.round(1)} truth_board~{centre.round(1)}")

    board=Board(sim_intrinsics(env,'wrist'))
    reference=cv2.imread(str(REFERENCE),cv2.IMREAD_GRAYSCALE)
    poser=Poser(env)
    cik=CameraIK(env,'wrist')

    # Coarse observation pose: point at the piece, then offset the camera
    # horizontally by --offset-mm so the servo has something to correct.
    joints,_=poser.solve(truth)
    env.data.qpos[poser.qadr]=joints; mujoco.mj_forward(env.model,env.data)
    cam_pos,cam_rot=cik.camera_pose(env.data.qpos[cik.qpos_index])
    bx,by=board_axes(env)
    off=bx*(args.offset_mm/1000)+by*(args.offset_mm/1000)
    r0=cik.solve(cam_pos+off,cam_rot,seed=env.data.qpos[cik.qpos_index])
    env.data.qpos[poser.qadr]=r0['q']; mujoco.mj_forward(env.model,env.data)
    print(f"coarse offset applied: pos_err {r0['position_error_mm']:.1f}mm "
          f"aim_err {r0['aim_error_deg']:.1f}deg")

    history=[]
    for step in range(args.steps):
        err,info=sense(env,board,reference,args.square,want_piece=True)
        if err is None:
            print(f"step {step}: no piece ({info.get('why')}) markers {info['markers']}")
            history.append({'step':step,'error_mm':None,'markers':info['markers'],
                            'why':info.get('why')})
            break
        dist=float(np.linalg.norm(err))
        print(f"step {step}: markers {info['markers']} rms {info['rms']} "
              f"cam {info['cam_board_mm']} piece {info['target_board_mm']} "
              f"err {err.round(1)} |err| {dist:.2f}mm")
        history.append({'step':step,'error_mm':round(dist,2),
                        'cam_board_mm':info['cam_board_mm'],
                        'piece_board_mm':info['target_board_mm'],
                        'markers':info['markers']})
        if dist<=args.tol_mm:
            print(f"CONVERGED step {step}: {dist:.2f}mm")
            break
        step=np.clip(args.gain*err,-args.max_step_mm,args.max_step_mm)
        cam_pos,cam_rot=cik.camera_pose(env.data.qpos[cik.qpos_index])
        delta=bx*(step[0]/1000)+by*(step[1]/1000)
        res=cik.solve(cam_pos+delta,cam_rot,seed=env.data.qpos[cik.qpos_index])
        env.data.qpos[poser.qadr]=res['q']; mujoco.mj_forward(env.model,env.data)

    (ROOT/'reports'/f'visual_servo_sim_{args.square}.json').write_text(
        json.dumps({'schema':'visual-servo-sim/2','square':args.square,
                    'offset_mm':args.offset_mm,'history':history},indent=2)+'\n')
    errs=[h['error_mm'] for h in history if h['error_mm'] is not None]
    print(f"FINAL {errs[-1] if errs else 'n/a'}mm over {len(errs)} steps")

if __name__=='__main__':
    main()
