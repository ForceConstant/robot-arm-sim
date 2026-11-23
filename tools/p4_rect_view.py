#!/usr/bin/env python3
"""Render the rectified board with a labelled metric grid so the piece's true
board-mm position can be read unambiguously. Saves an annotated image."""
import json,sys
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

frames=[]
for k in range(8):
    f=grab_frame(timeout=6);rect=b.rectify(f);sol,d=b.solve(rect)
    if sol is None or len(d)<3: continue
    grey=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
    rr,valid,n,pxm=b.rectified(grey,sol,4.0)
    # draw grid lines every 28mm and square labels directly on rectified image
    out=cv2.cvtColor(rr,cv2.COLOR_GRAY2BGR)
    for i in range(9):
        mm=-112+i*28
        u=int((mm+128)*pxm); v=int((128-mm)*pxm)
        cv2.line(out,(u,0),(u,n),(90,90,90),1)
        cv2.line(out,(0,v),(n,v),(90,90,90),1)
    # label each square centre with coords
    for col in range(8):
        for rank in range(8):
            cx_mm=(col+0.5)*28-112; cy_mm=(rank+0.5)*28-112
            u=int((cx_mm+128)*pxm); v=int((128-cy_mm)*pxm)
            cv2.putText(out,f"{chr(97+col)}{rank+1}",(u-10,v+4),
                cv2.FONT_HERSHEY_SIMPLEX,0.32,(0,200,255),1,cv2.LINE_AA)
    # mark nominal e5 and image centre in board mm
    def mm2px(x,y): return (int((x+128)*pxm),int((128-y)*pxm))
    cv2.drawMarker(out,mm2px(14,14),(0,0,255),cv2.MARKER_CROSS,30,2,cv2.LINE_AA)
    g=b.grid(sol); gaze=b.unproject(sol,(320.,240.),0.)
    cv2.drawMarker(out,mm2px(*gaze),(60,255,90),cv2.MARKER_CROSS,24,2,cv2.LINE_AA)
    cv2.imwrite('/tmp/rect_grid.jpg',out)
    print('saved, markers',len(d),'rms',round(sol['rms_px'],2),
          'gaze_board',gaze.round(1).tolist())
    break
else:
    print('no clean frame')
