#!/usr/bin/env python3
"""Measure where the piece ACTUALLY is vs the nominal e5 aim point (2026-11-22).

On a clean frame (jaws clear, >=3 markers), report three things in board mm:
  - NOMINAL AIM (14,14) - what the red crosshair marks and what the tip was commanded to
  - BLOB CENTROID unprojected to z=0 - projection of the standing piece
  - BASE CONTACT - the bottom of the deviation blob where it meets the board
Also save an annotated image with all three so the offset is visible.
"""
import json, sys, time
import cv2, numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame

SQ=28.0
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
ref=cv2.imread('/home/brian/robot-arm-sim/chess_sim/assets/scene/generated/board_charuco.png',0)

def measure(frame):
    rect=b.rectify(frame)
    sol,dets=b.solve(rect)
    if sol is None: return None,{'markers':len(dets),'reason':'no pose'}
    grey=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
    rr,valid,n,pxm=b.rectified(grey,sol,4.0)
    exp=cv2.resize(ref,(n,n),interpolation=cv2.INTER_AREA)
    diff=cv2.absdiff(rr,exp)
    mask=((diff>60)&(valid>0)).astype(np.uint8)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    cnt,lab,stats,cents=cv2.connectedComponentsWithStats(mask,8)
    # piece blob near e5
    best=None
    for i in range(1,cnt):
        area=stats[i,cv2.CC_STAT_AREA]/pxm**2
        cx,cy=cents[i]
        cx_mm=cx/pxm-128.0; cy_mm=128.0-cy/pxm
        dist=np.hypot(cx_mm-14,cy_mm-14)
        if 80<=area<=3000 and dist<25:
            if best is None or area>best[0]:
                best=(area,i,cx_mm,cy_mm)
    out={'markers':len(dets),'rms':round(sol['rms_px'],2)}
    if best is None:
        return frame,out
    area,i,cx_mm,cy_mm=best
    # blob pixel bbox in rectified image -> bottom edge (max v)
    x,y,w,h,apx=stats[i]
    bottom_v=y+h-1
    bottom_u=int(cents[i][0])
    # convert rectified (u,v) -> board mm
    def rpx_to_board(u,v): return np.array([u/pxm-128.0, 128.0-v/pxm])
    centroid_mm=np.array([cx_mm,cy_mm])
    base_mm=rpx_to_board(bottom_u,bottom_v)
    nominal=np.array([14.0,14.0])
    out.update({
        'nominal_aim_mm':nominal.round(2).tolist(),
        'blob_centroid_mm':centroid_mm.round(2).tolist(),
        'base_contact_mm':base_mm.round(2).tolist(),
        'nominal_to_centroid_mm':round(float(np.linalg.norm(centroid_mm-nominal)),2),
        'nominal_to_base_mm':round(float(np.linalg.norm(base_mm-nominal)),2),
        'area_mm2':round(float(area),0),
    })
    # annotate the RAW frame using PnP projection of the board points
    def proj(mm):
        p=b.project(sol,np.array([mm[0],mm[1],0.0]),0.0)
        return (int(p[0]),int(p[1]))
    cv2.drawMarker(frame,proj(nominal),(0,0,255),cv2.MARKER_CROSS,20,2,cv2.LINE_AA)
    cv2.drawMarker(frame,proj(centroid_mm),(60,255,90),cv2.MARKER_CROSS,18,2,cv2.LINE_AA)
    cv2.drawMarker(frame,proj(base_mm),(0,220,255),cv2.MARKER_DIAMOND,14,2,cv2.LINE_AA)
    def lbl(t,p,c):
        cv2.putText(frame,t,(p[0]+8,p[1]-8),cv2.FONT_HERSHEY_SIMPLEX,0.45,c,1,cv2.LINE_AA)
    lbl('RED nominal(14,14)',proj(nominal),(0,0,255))
    lbl('GRN blob centre',proj(centroid_mm),(60,255,90))
    lbl('YLW base contact',proj(base_mm),(0,220,255))
    cv2.putText(frame,f"markers {len(dets)} rms {sol['rms_px']:.2f} "
                f"nom->centroid {out['nominal_to_centroid_mm']}mm nom->base {out['nominal_to_base_mm']}mm",
                (10,frame.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,255,255),1,cv2.LINE_AA)
    return frame,out

if __name__=='__main__':
    best=None
    for k in range(10):
        f=grab_frame(timeout=6)
        ann,out=measure(f.copy())
        print('try',k,out)
        if out.get('base_contact_mm'):
            best=(ann,out);break
        time.sleep(0.2)
    if best:
        ann,out=best
        cv2.imwrite('/tmp/aim_measure.jpg',ann)
        open('/tmp/aim_measure.json','w').write(json.dumps(out,indent=2))
        print('SAVED',json.dumps(out,indent=2))
    else:
        print('no measurement')
