#!/usr/bin/env python3
"""IRL detect-only check for the close-then-servo loop (2026-11-22).

Grabs one wrist-camera frame from the live console stream, runs the SAME
mount-free detector as the dashboard (Board.solve -> PnP, no FK/mount), and
reports the camera board position and the piece centroid on the target square.

NO MOTION. NO GRIPPER ACTUATION. This verifies the perception signal only.
"""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import cv2, numpy as np, urllib.request

ROOT=Path('/home/brian/robot-arm-sim')
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tools'))
from chess_sim.board_target import Board
from p4_sim_servo2 import local_piece_xy

REFERENCE=ROOT/'chess_sim/assets/scene/generated/board_charuco.png'
STREAM='http://192.168.1.171:8080/stream'

def grab_frame(timeout=6.0):
    req=urllib.request.Request(STREAM)
    data=b''
    t0=time.time()
    with urllib.request.urlopen(req,timeout=timeout) as r:
        while time.time()-t0<timeout and len(data)<200000:
            chunk=r.read(8192)
            if not chunk: break
            data+=chunk
    s=data.find(b'\xff\xd8'); e=data.find(b'\xff\xd9',s)
    if s<0 or e<=s: raise RuntimeError('no jpeg in stream')
    arr=np.frombuffer(data[s:e+2],np.uint8)
    return cv2.imdecode(arr,cv2.IMREAD_COLOR)

def square_centre_mm(sq):
    return np.array([((ord(sq[0])-97)+0.5)*28.0-112.0,(int(sq[1])-1+0.5)*28.0-112.0])

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--square',default='e5')
    ap.add_argument('--frames',type=int,default=3)
    args=ap.parse_args()
    cal=json.loads((ROOT/'.private/phase11d/calibration.json').read_text())
    board=Board(cal)
    ref=cv2.imread(str(REFERENCE),cv2.IMREAD_GRAYSCALE)
    for i in range(args.frames):
        frame=grab_frame()
        rect=board.rectify(frame)
        sol,dets=board.solve(rect)
        if sol is None:
            print(f"frame {i}: {len(dets)} markers, no pose"); continue
        cam=np.asarray(sol['camera_in_board_mm'],float)[:2]
        grey=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
        rr,valid,n,pxm=board.rectified(grey,sol,4.0)
        expected=cv2.resize(ref,(n,n),interpolation=cv2.INTER_AREA)
        piece=local_piece_xy(frame,rr,valid,n,pxm,expected,board,sol,args.square)
        print(f"frame {i}: {len(dets)} markers rms {sol['rms_px']:.2f}px "
              f"cam_board {cam.round(1)} piece {piece and piece.round(1)} "
              f"square_centre {square_centre_mm(args.square)}")
        cv2.imwrite(f'/tmp/irl_servo_frame_{i}.jpg',frame)

if __name__=='__main__':
    main()
