#!/usr/bin/env python3
"""Locate the piece correctly: rotate the portrait stream frame to the
calibration's 640x480 landscape, then PnP + unproject. Report base board mm.
"""
import json,sys,time
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

def orient(f):
    h,w=f.shape[:2]
    if h>w:  # portrait -> rotate to landscape
        return cv2.rotate(f,cv2.ROTATE_90_COUNTERCLOCKWISE)
    return f

def one(frame):
    f=orient(frame)
    h,w=f.shape[:2]
    if (w,h)!=(640,480):
        f=cv2.resize(f,(640,480))
    rect=b.rectify(f); sol,d=b.solve(rect)
    if sol is None or len(d)<3: return None,{'markers':len(d),'reason':'no pose'}
    p5=b.project(sol,np.array([14.,14.,0.]),0.0)
    grey=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
    # white pawn: brightest blob near e5 pixel
    bright=((grey>170)).astype(np.uint8)
    bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((7,7),np.uint8))
    cnt,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
    cands=[]
    for i in range(1,cnt):
        x,y,bw,bh,area=stats[i]
        cx,cy=cents[i]
        dp=np.hypot(cx-p5[0],cy-p5[1])
        if area>200 and dp<80 and bh<180:
            base_mm=b.unproject(sol,(cx,y+bh-1),0.0)
            cent_mm=b.unproject(sol,(cx,cy),0.0)
            cands.append((area,float(cx),float(cy),cent_mm,base_mm))
    out={'markers':len(d),'rms':round(sol['rms_px'],2),
         'e5_pixel':p5.round(1).tolist(),'frame_wh':[w,h],
         'cam_board':np.array(sol['camera_in_board_mm'])[:2].round(1).tolist(),
         'n_cands':len(cands)}
    cands.sort(key=lambda c:-c[0])
    if not cands: return f,out
    area,cx,cy,cent_mm,base_mm=cands[0]
    nom=np.array([14.,14.])
    out.update({'blob_area_px':area,'blob_px':[round(cx,1),round(cy,1)],
                'centroid_mm':cent_mm.round(1).tolist(),
                'base_mm':base_mm.round(1).tolist(),
                'base_off_mm':round(float(np.linalg.norm(base_mm-nom)),1)})
    def pp(mm):
        q=b.project(sol,np.array([mm[0],mm[1],0.]),0.0);return(int(q[0]),int(q[1]))
    cv2.drawMarker(f,pp(nom),(0,0,255),cv2.MARKER_CROSS,20,2,cv2.LINE_AA)
    cv2.drawMarker(f,pp(cent_mm),(60,255,90),cv2.MARKER_CROSS,20,3,cv2.LINE_AA)
    cv2.drawMarker(f,pp(base_mm),(0,220,255),cv2.MARKER_DIAMOND,14,2,cv2.LINE_AA)
    cv2.putText(f,f"RED nom GRN cent YLW base | base_off {out['base_off_mm']}mm",
                (10,465),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255),1,cv2.LINE_AA)
    return f,out

if __name__=='__main__':
    rows=[];ann=None
    for k in range(12):
        f=grab_frame(timeout=6)
        a,out=one(f.copy())
        if out.get('base_mm'):
            rows.append(out);ann=a
            print('f',k,'ids',out['markers'],'rms',out['rms'],'e5px',out['e5_pixel'],
                  'blobpx',out['blob_px'],'base',out['base_mm'],'off',out['base_off_mm'])
        time.sleep(0.15)
    if rows:
        base=np.array([r['base_mm'] for r in rows],float)
        cm=np.array([r['centroid_mm'] for r in rows],float)
        print('\nMEDIAN base_mm',np.median(base,axis=0).round(1),'spread',base.std(axis=0).round(1),'n',len(rows))
        print('MEDIAN centroid_mm',np.median(cm,axis=0).round(1),'spread',cm.std(axis=0).round(1))
        cv2.imwrite('/tmp/piece_fixed.jpg',ann)
        open('/tmp/piece_fixed.json','w').write(json.dumps({'median_base_mm':np.median(base,axis=0).round(1).tolist(),
            'median_centroid_mm':np.median(cm,axis=0).round(1).tolist(),
            'spread_mm':base.std(axis=0).round(1).tolist(),'n':len(rows),'rows':rows},indent=2))
    else: print('no measurement')
