#!/usr/bin/env python3
"""Run the integrated grasp while recording the wrist stream to MP4.

Starts a background persistent-stream recorder, executes p4_grasp_context's
pipeline, then stops recording. All computation on BC-250. Convert to GIF with
ffmpeg afterwards (separate step).

    PYTHONPATH=. .venv/bin/python tools/p4_grasp_with_video.py --piece pawn --square e5 --video /tmp/grasp_final.mp4
"""
import argparse,sys,time,threading,urllib.request
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from p4_grasp_context import post,get,tip,correction_loop,C,B
from infer_base_xy import load_model,predict
from p4_irl_servo_detect import grab_frame

STREAM=C+'/stream'
class Recorder:
    def __init__(self,path,fps=12):
        self.path=path;self.fps=fps;self.stop=False;self.frames=0
        self.buf=b'';self.resp=None;self.writer=None
        self.thread=threading.Thread(target=self._run,daemon=True)
    def start(self):self.thread.start()
    def _run(self):
        while not self.stop:
            try:
                if self.resp is None:self.resp=urllib.request.urlopen(STREAM,timeout=15)
                chunk=self.resp.read(8192)
                if not chunk:
                    try:self.resp.close()
                    except Exception:pass
                    self.resp=None;continue
                self.buf+=chunk
                while True:
                    s=self.buf.find(b'\xff\xd8');e=self.buf.find(b'\xff\xd9',s)
                    if s<0 or e<s:
                        self.buf=self.buf[max(0,s-1):] if s>0 else self.buf[-4096:];break
                    jpg=self.buf[s:e+2];self.buf=self.buf[e+2:]
                    img=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
                    if img is None:continue
                    h,w=img.shape[:2]
                    if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
                    if img.shape[:2]!=(480,640):img=cv2.resize(img,(640,480))
                    if self.writer is None:
                        self.writer=cv2.VideoWriter(self.path,cv2.VideoWriter_fourcc(*'mp4v'),self.fps,(640,480))
                    self.writer.write(img);self.frames+=1
            except Exception as ex:
                self.resp=None;time.sleep(0.2)
    def stop_rec(self):
        self.stop=True
        try:
            if self.resp:self.resp.close()
        except Exception:pass
        time.sleep(0.5)
        if self.writer:self.writer.release()
        return self.frames

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--piece',default='pawn');ap.add_argument('--square',default='e5')
    ap.add_argument('--video',default='/tmp/grasp_final.mp4')
    ap.add_argument('--z-safe',type=float,default=70);ap.add_argument('--z-grasp',type=float,default=8)
    a=ap.parse_args()
    st=get(B+'/state');print('START',st['mode'],st['fault'],'jaw',round(st['joints']['gripper'],1),flush=True)
    assert st['fault'] is None
    rec=Recorder(a.video);rec.start();time.sleep(1)
    model=load_model()
    frame=grab_frame(timeout=8)
    xy,info=predict(frame,a.piece.lower(),a.square.lower(),model)
    print(f'PREDICT {a.piece}@{a.square}: {xy} ({info})',flush=True)
    if xy is None:
        print('NO PREDICTION - abort');rec.stop_rec();return
    target=[round(float(xy[0]),1),round(float(xy[1]),1),a.z_safe]
    print('LIFT+SERVO to',target,flush=True)
    post('/mode',{'mode':'REMOTE'});time.sleep(0.7)
    ok,hist=correction_loop(target)
    if not ok:
        print('SERVO FAILED - abort');rec.stop_rec();post('/mode',{'mode':'HOLD'});return
    # descend
    print('DESCEND toward piece',flush=True)
    for k in range(14):
        cur=tip()
        if cur[2]<=a.z_grasp+1:break
        dz=-min(5.0,cur[2]-a.z_grasp)
        dr=post('/move',{'mode':'rel','dz_mm':round(float(dz),2),'dry_run':True},t=25)
        if dr.get('detail'):print('  descend dry fail',flush=True);break
        post('/move',{'mode':'rel','dz_mm':round(float(dz),2)},t=60);time.sleep(0.9)
        if get(B+'/state')['fault']:print('  descend fault',flush=True);break
    print('AT',tip().round(1),'z - CLOSE JAWS',flush=True)
    post('/move',{'mode':'joint','jaw':12},t=60);time.sleep(1.2)
    print('LIFT',flush=True)
    post('/move',{'mode':'rel','dz_mm':45},t=60);time.sleep(1.2)
    fin=get(B+'/state')
    print(f'FINAL tip {tip().round(1)} jaw {fin["joints"]["gripper"]:.1f} fault {fin["fault"]}',flush=True)
    n=rec.stop_rec()
    try:post('/mode',{'mode':'HOLD'})
    except Exception:pass
    print(f'VIDEO {a.video} frames={n}; HOLD',flush=True)

if __name__=='__main__':main()
