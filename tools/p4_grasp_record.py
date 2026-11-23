#!/usr/bin/env python3
"""Re-record the grasp attempt with a fast persistent-stream recorder and a
mount-free detected-piece aim marker overlaid on every frame.

The marker shows the piece location from marker-PnP detection (green crosshair)
and the square-centre aim point actually commanded (red crosshair). If they are
offset, that is why the grasp missed.

    PYTHONPATH=. .venv/bin/python tools/p4_grasp_record.py
"""
from __future__ import annotations
import argparse, json, sys, time, threading, urllib.request, urllib.error
from pathlib import Path
import cv2, numpy as np

ROOT=Path('/home/brian/robot-arm-sim')
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'tools'))
from chess_sim.board_target import Board
from p4_sim_servo2 import local_piece_xy

CONSOLE='http://192.168.1.171:8080'
BRIDGE='http://192.168.1.167:8100'
STREAM=CONSOLE+'/stream'
VIDEO=Path('/tmp/grasp_marked.mp4')
CAL=ROOT/'.private/phase11d/calibration.json'
REF=ROOT/'chess_sim/assets/scene/generated/board_charuco.png'

def get(url,timeout=10):
    with urllib.request.urlopen(url,timeout=timeout) as r: return json.loads(r.read())
def post(path,body,timeout=90):
    req=urllib.request.Request(CONSOLE+path,data=json.dumps(body).encode(),
                               method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=timeout) as r: return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {'http_error':e.code,'detail':e.read().decode()[:300]}

class MJPEGReader:
    """One persistent connection; parses JPEG frames at the stream's native rate."""
    def __init__(self,url):
        self.url=url; self.buf=b''; self.resp=None
        self.frame=None; self.ts=0.0; self.stop=False
        self.thread=threading.Thread(target=self._run,daemon=True)
    def start(self): self.thread.start()
    def _connect(self):
        return urllib.request.urlopen(self.url,timeout=15)
    def _run(self):
        while not self.stop:
            try:
                if self.resp is None: self.resp=self._connect()
                chunk=self.resp.read(8192)
                if not chunk:
                    try: self.resp.close()
                    except Exception: pass
                    self.resp=None; continue
                self.buf+=chunk
                while True:
                    s=self.buf.find(b'\xff\xd8')
                    if s<0:
                        self.buf=self.buf[-4096:]; break
                    e=self.buf.find(b'\xff\xd9',s+2)
                    if e<0: break
                    arr=np.frombuffer(self.buf[s:e+2],np.uint8)
                    img=cv2.imdecode(arr,cv2.IMREAD_COLOR)
                    if img is not None:
                        self.frame=img; self.ts=time.time()
                    self.buf=self.buf[e+2:]
            except Exception:
                try:
                    if self.resp: self.resp.close()
                except Exception: pass
                self.resp=None; time.sleep(0.2)
    def latest(self):
        return self.frame,self.ts

class OverlayRecorder:
    def __init__(self,board,ref,square,aim_xy,path,fps=8.0):
        self.board=board; self.ref=ref; self.square=square; self.aim_xy=aim_xy
        self.path=path; self.fps=fps; self.reader=MJPEGReader(STREAM)
        self.writer=None; self.size=None; self.stop=False; self.count=0
        self.detections=[]
        self.thread=threading.Thread(target=self._run,daemon=True)
    def start(self): self.reader.start(); self.thread.start()
    def _run(self):
        last=0.0
        while not self.stop:
            frame,ts=self.reader.latest()
            if frame is None or ts<=last: time.sleep(0.03); continue
            last=ts
            out=self.overlay(frame.copy())
            h,w=out.shape[:2]
            if self.size is None:
                self.size=(w,h)
                self.writer=cv2.VideoWriter(str(self.path),
                    cv2.VideoWriter_fourcc(*'mp4v'),self.fps,(w,h))
            self.writer.write(out); self.count+=1
            time.sleep(0.02)
    def overlay(self,frame):
        try:
            sol,dets=self.board.solve(frame)   # mount-free PnP
            if sol is None:
                cv2.putText(frame,f"markers {len(dets)} - no pose",(10,26),
                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2,cv2.LINE_AA)
                return frame
            # red crosshair: commanded aim point (square centre, board mm)
            aim_px=self.board.project(sol,np.asarray(self.aim_xy,float),0.0)
            cv2.drawMarker(frame,(int(aim_px[0]),int(aim_px[1])),(0,0,255),
                cv2.MARKER_CROSS,18,2,cv2.LINE_AA)
            # green crosshair: DETECTED piece (deviation blob on the square)
            grey=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            rr,valid,n,pxm=self.board.rectified(grey,sol,4.0)
            expected=cv2.resize(self.ref,(n,n),interpolation=cv2.INTER_AREA)
            piece=local_piece_xy(frame,rr,valid,n,pxm,expected,self.board,sol,self.square)
            if piece is not None:
                p_px=self.board.project(sol,np.asarray(piece,float),0.0)
                cv2.drawMarker(frame,(int(p_px[0]),int(p_px[1])),(60,255,90),
                    cv2.MARKER_CROSS,22,3,cv2.LINE_AA)
                d=float(np.linalg.norm(np.asarray(piece,float)-np.asarray(self.aim_xy,float)))
                self.detections.append({'piece_board_mm':piece.round(1).tolist(),
                                        'aim_board_mm':list(map(float,self.aim_xy)),
                                        'offset_mm':round(d,2)})
                label=f"detected piece {piece.round(1).tolist()} | aim {list(map(float,self.aim_xy))} | off {d:.1f}mm"
            else:
                label=f"markers {len(dets)} rms {sol['rms_px']:.2f} - no piece blob"
            cv2.putText(frame,label,(10,frame.shape[0]-12),
                cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255),1,cv2.LINE_AA)
        except Exception as e:
            cv2.putText(frame,f"overlay err {str(e)[:30]}",(10,26),
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1,cv2.LINE_AA)
        return frame
    def stop_and_save(self):
        self.stop=True
        self.thread.join(timeout=10); self.reader.stop=True
        try: self.reader.resp.close()
        except Exception: pass
        if self.writer: self.writer.release()
        sz=self.path.stat().st_size if self.path.exists() else 0
        return sz,self.count

def dryrun(body):
    r=post('/move',{**body,'dry_run':True},timeout=25)
    if r.get('http_error') or 'no reachable' in str(r.get('detail','')):
        return False,json.dumps(r)[:180]
    req=r.get('requested',{})
    return True,f"perr {req.get('position_error_mm')}mm roll {r.get('target_joints',{}).get('wrist_roll')}"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--square',default='e5')
    ap.add_argument('--x',type=float,default=14.0)
    ap.add_argument('--y',type=float,default=14.0)
    ap.add_argument('--hover-z',type=float,default=45.0)
    ap.add_argument('--grasp-z',type=float,default=16.0)
    ap.add_argument('--lift-z',type=float,default=70.0)
    ap.add_argument('--jaw-open',type=float,default=100.0)
    ap.add_argument('--jaw-close',type=float,default=30.0)
    args=ap.parse_args()

    st=get(BRIDGE+'/state'); roll0=st['joints']['wrist_roll']
    print('start',st['mode'],'roll',round(roll0,2),'jaw',st['joints']['gripper'])
    if st['mode']!='REMOTE': print('mode->',post('/mode',{'mode':'REMOTE'}));time.sleep(0.6)
    board=Board(json.loads(CAL.read_text()))
    ref=cv2.imread(str(REF),cv2.IMREAD_GRAYSCALE)
    rec=OverlayRecorder(board,ref,args.square,
                        (args.x,args.y),VIDEO,fps=8.0)
    rec.start(); print('recording started',VIDEO); time.sleep(1.0)

    steps=[
        ('open jaws',{'mode':'joint','jaw':args.jaw_open}),
        ('hover',{'mode':'xyz','x_mm':args.x,'y_mm':args.y,'z_mm':args.hover_z}),
        ('descend',{'mode':'xyz','x_mm':args.x,'y_mm':args.y,'z_mm':args.grasp_z}),
        ('close jaws',{'mode':'joint','jaw':args.jaw_close}),
        ('lift',{'mode':'xyz','x_mm':args.x,'y_mm':args.y,'z_mm':args.lift_z}),
    ]
    for name,body in steps:
        ok,info=dryrun(body); print(f"[{name}] dry {'OK' if ok else 'FAIL'} {info}")
        if not ok: print('ABORT');break
        if body['mode']=='joint':
            s=get(BRIDGE+'/state')['joints']
            body={'mode':'joint','joints':{k:float(s[k]) for k in
                 ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')},
                 'jaw':body['jaw']}
        r=post('/move',body,timeout=90)
        print(f"[{name}] exec {json.dumps(r)[:150]}")
        s=get(BRIDGE+'/state')
        if s['fault'] or s['mode']!='REMOTE' or abs(s['joints']['wrist_roll']-roll0)>3:
            print('ABORT');break
        time.sleep(0.7)

    time.sleep(1.0)
    sz,count=rec.stop_and_save()
    print(f"video {VIDEO} {sz} bytes, {count} frames")
    print('restore HOLD',post('/mode',{'mode':'HOLD'}))
    dets=rec.detections
    if dets:
        ds=[d['offset_mm'] for d in dets]
        print(f"detections {len(dets)}: aim-vs-piece offset mean {np.mean(ds):.1f}mm "
              f"min {min(ds):.1f} max {max(ds):.1f}")
        print('sample detection',dets[len(dets)//2])
    out=ROOT/'reports'/'grasp_marked.json'
    out.write_text(json.dumps({'schema':'grasp-marked/1','aim_board_mm':[args.x,args.y],
        'frames':count,'detections':dets[-20:]},indent=2)+'\n')
    print('wrote',out)

if __name__=='__main__':
    main()
