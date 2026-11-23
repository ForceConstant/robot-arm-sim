#!/usr/bin/env python3
"""Locate the white piece from the RAW image (brightest blob near e5), then
unproject its centroid and base through the live PnP pose to board mm.

The piece is near-white (~200+) on a grey board (~100-150); white marker squares
are smaller and we exclude them by requiring the blob to be near e5's pixel and
by size. Multiple frames are sampled; the median is reported for stability.
"""
import json,sys,time
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

def one(frame):
    rect=b.rectify(frame); sol,d=b.solve(rect)
    if sol is None or len(d)<3: return None,{'markers':len(d),'reason':'no pose'}
    p5=b.project(sol,np.array([14.,14.,0.]),0.0)  # e5 nominal pixel
    grey=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # bright mask on RAW frame
    bright=((grey>175)).astype(np.uint8)
    bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((7,7),np.uint8))
    cnt,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
    cands=[]
    for i in range(1,cnt):
        x,y,w,h,area=stats[i]
        cx,cy=cents[i]
        dp=np.hypot(cx-p5[0],cy-p5[1])
        # the piece is a large bright blob close to e5
        if area>250 and dp<70 and w<h*1.6 and h<200:
            # base = bottom of blob
            base_px=np.array([cx, y+h-1])
            cent_mm=b.unproject(sol,(cx,cy),0.0)
            base_mm=b.unproject(sol,tuple(base_px),0.0)
            cands.append((area,float(cx),float(cy),cent_mm,base_mm))
    out={'markers':len(d),'rms':round(sol['rms_px'],2),
         'e5_pixel':[round(float(p5[0]),1),round(float(p5[1]),1)],
         'cam_board':np.array(sol['camera_in_board_mm'])[:2].round(1).tolist(),
         'n_cands':len(cands)}
    if not cands: return frame,out
    cands.sort(key=lambda c:-c[0])
    area,cx,cy,cent_mm,base_mm=cands[0]
    nom=np.array([14.,14.])
    out.update({'blob_area_px':area,'blob_px':[round(cx,1),round(cy,1)],
                'centroid_mm':cent_mm.round(1).tolist(),
                'base_mm':base_mm.round(1).tolist(),
                'centroid_off_mm':round(float(np.linalg.norm(cent_mm-nom)),1),
                'base_off_mm':round(float(np.linalg.norm(base_mm-nom)),1)})
    # annotate
    def pp(mm):
        q=b.project(sol,np.array([mm[0],mm[1],0.]),0.0);return(int(q[0]),int(q[1]))
    cv2.drawMarker(frame,pp(nom),(0,0,255),cv2.MARKER_CROSS,20,2,cv2.LINE_AA)
    cv2.drawMarker(frame,pp(cent_mm),(60,255,90),cv2.MARKER_CROSS,20,3,cv2.LINE_AA)
    cv2.drawMarker(frame,pp(base_mm),(0,220,255),cv2.MARKER_DIAMOND,14,2,cv2.LINE_AA)
    cv2.putText(frame,f"RED nom GRN centroid YLW base | off {out['base_off_mm']}mm",
                (10,frame.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,0.42,(255,255,255),1,cv2.LINE_AA)
    return frame,out

if __name__=='__main__':
    rows=[]; annotated=None
    for k in range(12):
        f=grab_frame(timeout=6)
        ann,out=one(f.copy())
        if out.get('base_mm'):
            rows.append(out); annotated=ann
            print('frame',k,'ids',out['markers'],'base_mm',out['base_mm'],'cent_mm',out['centroid_mm'],
                  'e5px',out['e5_pixel'],'blobpx',out['blob_px'])
        time.sleep(0.15)
    if rows:
        base=np.array([r['base_mm'] for r in rows],float)
        print('\nMEDIAN base_mm',np.median(base,axis=0).round(1),
              'spread',base.std(axis=0).round(1),'n',len(rows))
        cv2.imwrite('/tmp/piece_raw.jpg',annotated)
        open('/tmp/piece_raw.json','w').write(json.dumps({'median_base_mm':np.median(base,axis=0).round(1).tolist(),
            'spread_mm':base.std(axis=0).round(1).tolist(),'n':len(rows),'rows':rows},indent=2))
    else: print('no measurement')
