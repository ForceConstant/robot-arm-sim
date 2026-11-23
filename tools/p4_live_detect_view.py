#!/usr/bin/env python3
"""Live piece-detection viewer on the CachyOS desktop (2026-11-22).

Shows, in real time:
  - green cross: detected piece centre (bright-white blob, unprojected to board mm)
  - yellow diamond: estimated piece base contact
  - blue grid: the 28mm ChArUco square lattice from mount-free PnP
  - red cross: nominal e5 centre (14,14) for reference
  - HUD: board position of the detected piece, its offset from square centre,
    marker count and PnP RMS

Displays via mpv from a continuously written video file. NO arm motion, NO grasp.
"""
import json,sys,time,threading,urllib.request
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board

CONSOLE='http://192.168.1.171:8080'
STREAM=CONSOLE+'/stream'
VIDEO='/tmp/live_detect.mp4'
OUT='/tmp/live_detect.jpg'
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))

orient=lambda f: cv2.rotate(f,cv2.ROTATE_90_CLOCKWISE) if f.shape[0]>f.shape[1] else f

def detect(frame):
    """Mount-free PnP + bright-piece localization. Returns annotated copy + info."""
    f=orient(frame).copy()
    if f.shape[:2]!=(480,640): f=cv2.resize(f,(640,480))
    rect=b.rectify(f); sol,d=b.solve(rect)
    info={'markers':len(d),'rms':round(sol['rms_px'],2) if sol else None}
    if sol is None or len(d)<2:
        cv2.putText(f,f"ids {len(d)} - no pose",(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
        return f,info
    # draw board grid (mount-free)
    g=b.grid(sol)
    for row in g['points']:
        for p in row:
            cv2.circle(f,(int(p[0]),int(p[1])),2,(255,120,60),-1)
    def pp(mm):
        q=b.project(sol,np.array([mm[0],mm[1],0.]),0.);return(int(q[0]),int(q[1]))
    nom=np.array([14.,14.])
    cv2.drawMarker(f,pp(nom),(0,0,255),cv2.MARKER_CROSS,18,2,cv2.LINE_AA)
    cv2.putText(f,'e5 nominal',(pp(nom)[0]+8,pp(nom)[1]-8),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,0,255),1)
    # bright piece blob near e5
    grey=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
    p5=b.project(sol,np.array([14.,14.,0.]),0.)
    bright=(grey>170).astype(np.uint8)
    bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((7,7),np.uint8))
    cnt,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
    cands=[]
    for i in range(1,cnt):
        x,y,w,h,area=stats[i]; cx,cy=cents[i]
        dp=np.hypot(cx-p5[0],cy-p5[1])
        if area>200 and dp<90 and h<180:
            cands.append((area,cx,cy,x,y,w,h))
    cands.sort(key=lambda c:-c[0])
    if cands:
        area,cx,cy,x,y,w,h=cands[0]
        cent_mm=b.unproject(sol,(cx,cy),0.)
        base_mm=b.unproject(sol,(cx,y+h-1),0.)
        info.update({'centroid_mm':cent_mm.round(1).tolist(),
                     'base_mm':base_mm.round(1).tolist(),
                     'offset_mm':round(float(np.linalg.norm(cent_mm-nom)),1),
                     'area_px':int(area)})
        cv2.drawMarker(f,(int(cx),int(cy)),(60,255,90),cv2.MARKER_CROSS,22,3,cv2.LINE_AA)
        cv2.drawMarker(f,(int(cx),int(y+h-1)),(0,220,255),cv2.MARKER_DIAMOND,14,2,cv2.LINE_AA)
        cv2.putText(f,f'piece {cent_mm.round(1).tolist()} mm',(int(cx)+10,int(cy)-10),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(60,255,90),2)
    hud=f"ids {len(d)} rms {info['rms']} | " + (
        f"piece {info.get('centroid_mm')} off {info.get('offset_mm')}mm" if cands else 'no piece blob')
    cv2.rectangle(f,(0,450),(640,480),(0,0,0),-1)
    cv2.putText(f,hud,(8,472),cv2.FONT_HERSHEY_SIMPLEX,0.48,(255,255,255),1,cv2.LINE_AA)
    return f,info

def main():
    # persistent stream reader
    buf=b''; frame=None; last=0
    wr=cv2.VideoWriter(VIDEO,cv2.VideoWriter_fourcc(*'mp4v'),8.0,(640,480))
    resp=urllib.request.urlopen(STREAM,timeout=10)
    end=time.time()+45  # 45s viewer
    print('viewing for 45s ->',VIDEO,flush=True)
    sample=None
    while time.time()<end:
        try:
            c=resp.read(8192)
            if not c: break
            buf+=c
            while True:
                s=buf.find(b'\xff\xd8')
                if s<0: break
                e=buf.find(b'\xff\xd9',s+2)
                if e<0: break
                img=cv2.imdecode(np.frombuffer(buf[s:e+2],np.uint8),cv2.IMREAD_COLOR)
                if img is not None and time.time()-last>1/8:
                    last=time.time()
                    ann,info=detect(img)
                    wr.write(ann); sample=info
                    cv2.imwrite(OUT,ann)
                buf=buf[e+2:]
        except Exception as ex:
            print('stream err',ex,flush=True);break
    wr.release()
    print('final sample',sample,flush=True)
    open('/tmp/live_detect.json','w').write(json.dumps(sample or {},indent=2))

if __name__=='__main__':main()
