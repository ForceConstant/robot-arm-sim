#!/usr/bin/env python3
"""Robust piece locator: the printed pawn is near-WHITE and much brighter than
the grey board squares. Find the largest very-bright component near e5, then
report its centroid and its base contact (bottom edge in rectified board mm).
"""
import json, sys, time
import cv2, numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame

b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

def locate(frame):
    rect=b.rectify(frame)
    sol,dets=b.solve(rect)
    if sol is None: return None,{'markers':len(dets),'reason':'no pose'}
    grey=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
    rr,valid,n,pxm=b.rectified(grey,sol,4.0)
    # very bright pixels: the white piece stands out from the ~grey board
    bright=((rr>200)&(valid>0)).astype(np.uint8)
    bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    cnt,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
    cands=[]
    for i in range(1,cnt):
        area=stats[i,cv2.CC_STAT_AREA]/pxm**2
        cx,cy=cents[i]
        cx_mm=cx/pxm-128.0; cy_mm=128.0-cy/pxm
        d=np.hypot(cx_mm-14,cy_mm-14)
        if area>60 and d<40:
            x,y,w,h,ap=stats[i]
            # base contact = bottom-centre of the component (max v)
            bu=int(cents[i][0]); bv=y+h-1
            base_mm=np.array([bu/pxm-128.0, 128.0-bv/pxm])
            cands.append((area,np.array([cx_mm,cy_mm]),base_mm,(x,y,w,h)))
    cands.sort(key=lambda c:-c[0])
    out={'markers':len(dets),'rms':round(sol['rms_px'],2),'n_cands':len(cands)}
    if not cands: return frame,out
    area,cent,base,bb=cands[0]
    nominal=np.array([14.0,14.0])
    out.update({'nominal_mm':nominal.round(1).tolist(),
                'bright_centroid_mm':cent.round(1).tolist(),
                'base_contact_mm':base.round(1).tolist(),
                'area_mm2':round(float(area),0),
                'centroid_offset_mm':round(float(np.linalg.norm(cent-nominal)),1),
                'base_offset_mm':round(float(np.linalg.norm(base-nominal)),1)})
    # annotate raw frame
    def proj(mm):
        p=b.project(sol,np.array([mm[0],mm[1],0.0]),0.0)
        return (int(p[0]),int(p[1]))
    cv2.drawMarker(frame,proj(nominal),(0,0,255),cv2.MARKER_CROSS,20,2,cv2.LINE_AA)
    cv2.drawMarker(frame,proj(cent),(60,255,90),cv2.MARKER_CROSS,18,2,cv2.LINE_AA)
    cv2.drawMarker(frame,proj(base),(0,220,255),cv2.MARKER_DIAMOND,14,2,cv2.LINE_AA)
    cv2.putText(frame,f"RED nominal GRN bright-centroid YLW base",(10,frame.shape[0]-12),
                cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(frame,f"markers {len(dets)} cent_off {out['centroid_offset_mm']}mm base_off {out['base_offset_mm']}mm area {out['area_mm2']}",
                (10,frame.shape[0]-28),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,255,255),1,cv2.LINE_AA)
    return frame,out

if __name__=='__main__':
    best=None
    for k in range(10):
        f=grab_frame(timeout=6)
        ann,out=locate(f.copy())
        print('try',k,out)
        if out.get('base_contact_mm') and out['area_mm2']>150:
            best=(ann,out);break
        time.sleep(0.2)
    if best:
        cv2.imwrite('/tmp/piece_bright.jpg',best[0])
        open('/tmp/piece_bright.json','w').write(json.dumps(best[1],indent=2))
        print('SAVED',json.dumps(best[1],indent=2))
    else: print('no measurement')
