#!/usr/bin/env python3
"""Grasp attempt 3: aim the jaw tip at the MEASURED piece position, not the
nominal square centre, then descend and close with video.

Aim point measured mount-free at the e5 observation pose:
  piece centroid board (8.7, 20.1) mm vs nominal (14,14).
"""
import json,sys,time,threading,urllib.request,urllib.error
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from p4_irl_servo_detect import grab_frame

CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
def get(u,t=10):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def post(p,b,t=90):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(b).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:250]}

class Rec:
    def __init__(self,path,fps=8):
        self.path=path;self.fps=fps;self.stop=False;self.count=0
        self.buf=b'';self.frame=None;self.last=0
        self.thread=threading.Thread(target=self._run,daemon=True)
    def start(self):self.thread.start()
    def _run(self):
        try:resp=urllib.request.urlopen(CONSOLE+'/stream',timeout=10)
        except Exception:return
        wr=None
        while not self.stop:
            try:
                c=resp.read(8192)
                if not c:break
                self.buf+=c
                while True:
                    s=self.buf.find(b'\xff\xd8')
                    if s<0:break
                    e=self.buf.find(b'\xff\xd9',s+2)
                    if e<0:break
                    img=cv2.imdecode(np.frombuffer(self.buf[s:e+2],np.uint8),cv2.IMREAD_COLOR)
                    if img is not None and time.time()-self.last>1.0/self.fps:
                        self.frame=img;self.last=time.time()
                        # rotate to landscape for viewing
                        if img.shape[0]>img.shape[1]:img=cv2.rotate(img,cv2.ROTATE_90_CLOCKWISE)
                        if wr is None:
                            wr=cv2.VideoWriter(str(self.path),cv2.VideoWriter_fourcc(*'mp4v'),self.fps,(img.shape[1],img.shape[0]))
                        cv2.putText(img,'GRASP 3 - aim at measured piece',(10,26),cv2.FONT_HERSHEY_SIMPLEX,0.6,(90,220,220),2)
                        wr.write(img);self.count+=1
                    self.buf=self.buf[e+2:]
            except Exception:break
        if wr:wr.release()
    def stopit(self):
        self.stop=True;self.thread.join(timeout=8)
        return self.path.stat().st_size if self.path.exists() else 0

def main():
    X,Y=8.7,20.1   # measured piece centroid
    st=get(BRIDGE+'/state');roll0=st['joints']['wrist_roll']
    print('start',st['mode'],'roll',round(roll0,2),'jaw',round(st['joints']['gripper'],1))
    if st['mode']!='REMOTE':post('/mode',{'mode':'REMOTE'});time.sleep(0.6)
    rec=Rec('/tmp/grasp3.mp4',fps=8);rec.start();time.sleep(1)
    steps=[
      ('open',{'mode':'joint','jaw':100}),
      ('hover',{'mode':'xyz','x_mm':X,'y_mm':Y,'z_mm':45}),
      ('descend',{'mode':'xyz','x_mm':X,'y_mm':Y,'z_mm':14}),
      ('close',{'mode':'joint','jaw':18}),
      ('lift',{'mode':'xyz','x_mm':X,'y_mm':Y,'z_mm':70}),
    ]
    log=[]
    for name,body in steps:
        dr=post('/move',{**body,'dry_run':True},t=25)
        ok=not(dr.get('http_error') or 'no reachable' in str(dr.get('detail','')))
        print(f'[{name}] dry {ok} {dr.get("requested",{}).get("position_error_mm")}mm')
        if not ok:print('ABORT');break
        if body['mode']=='joint':
            s=get(BRIDGE+'/state')['joints']
            body={'mode':'joint','joints':{k:float(s[k]) for k in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')},'jaw':body['jaw']}
        r=post('/move',body,t=90);print(f'[{name}] exec',json.dumps(r)[:120])
        log.append({'step':name,'resp':r})
        s=get(BRIDGE+'/state')
        if s['fault'] or s['mode']!='REMOTE' or abs(s['joints']['wrist_roll']-roll0)>3:
            print('ABORT');break
        time.sleep(0.7)
    time.sleep(1)
    sz=rec.stopit();print('video',sz,'bytes',rec.count,'frames')
    post('/mode',{'mode':'HOLD'})
    t=get(CONSOLE+'/state').get('target',{})
    print('final target status',t.get('status'),'piece',t.get('piece'))
    open('/home/brian/robot-arm-sim/reports/grasp3.json','w').write(json.dumps({'aim_board_mm':[X,Y],'log':log,'video_frames':rec.count},indent=2))
    print('wrote grasp3.json')

if __name__=='__main__':main()
