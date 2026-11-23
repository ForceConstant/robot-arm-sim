#!/usr/bin/env python3
"""First IRL grasp attempt on e5, with full video recording (2026-11-22).

Gripper actuation explicitly authorized by Brian 2026-11-22. The approach uses
FK-based tip targeting (/move xyz), which does NOT use the suspect camera mount:
only marker PnP (mount-free) was used to confirm the target square.

Sequence (every leg dry-run first, abort on 409 / fault / mode change / branch flip):
  record video -> jaws open -> hover above e5 -> descend to grasp height ->
  close jaws -> lift -> report -> stop video

No arm travel beyond the e5 column. Tip target = e5 centre (14,14) mm board.
"""
from __future__ import annotations
import argparse, json, sys, time, threading
import cv2, numpy as np
from pathlib import Path
import urllib.request, urllib.error

ROOT=Path('/home/brian/robot-arm-sim')
CONSOLE='http://192.168.1.171:8080'
BRIDGE='http://192.168.1.167:8100'
VIDEO=Path('/tmp/grasp_attempt.mp4')

def get(url,timeout=10):
    with urllib.request.urlopen(url,timeout=timeout) as r: return json.loads(r.read())
def post(path,body,timeout=60):
    req=urllib.request.Request(CONSOLE+path,data=json.dumps(body).encode(),
                               method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=timeout) as r: return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {'http_error':e.code,'detail':e.read().decode()[:300]}
def bridge():
    return get(BRIDGE+'/state',8)

def dryrun(body):
    r=post('/move',{**body,'dry_run':True},timeout=25)
    if r.get('http_error') or r.get('detail','').startswith('no reachable'):
        return False,json.dumps(r)[:200]
    req=r.get('requested',{})
    return True,f"perr {req.get('position_error_mm')}mm appr {req.get('approach_error_deg')}deg roll {r.get('target_joints',{}).get('wrist_roll')}"

class Recorder:
    """Background MJPEG-frame recorder -> mp4, covers the whole attempt."""
    def __init__(self, path, fps=5.0):
        self.path=path; self.fps=fps; self.frames=[]; self.stop=False
        self.thread=None; self.writer=None; self.size=None
        from p4_irl_servo_detect import grab_frame
        self.grab=grab_frame
    def start(self):
        self.thread=threading.Thread(target=self._run,daemon=True); self.thread.start()
    def _run(self):
        while not self.stop:
            try:
                f=self.grab(timeout=6.0)
                if f is None: continue
                h,w=f.shape[:2]
                if self.size is None:
                    self.size=(w,h)
                    self.writer=cv2.VideoWriter(str(self.path),
                        cv2.VideoWriter_fourcc(*'mp4v'),self.fps,(w,h))
                self.writer.write(f)
            except Exception:
                pass
            time.sleep(1.0/self.fps)
    def stop_and_save(self):
        self.stop=True
        if self.thread: self.thread.join(timeout=8)
        if self.writer: self.writer.release()
        n=len(self.frames)
        return self.path.stat().st_size if self.path.exists() else 0

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--x',type=float,default=14.0)
    ap.add_argument('--y',type=float,default=14.0)
    ap.add_argument('--hover-z',type=float,default=45.0)
    ap.add_argument('--grasp-z',type=float,default=16.0)
    ap.add_argument('--lift-z',type=float,default=70.0)
    ap.add_argument('--jaw-open',type=float,default=100.0)
    ap.add_argument('--jaw-close',type=float,default=25.0)
    ap.add_argument('--no-video',action='store_true')
    args=ap.parse_args()

    st=bridge(); roll0=st['joints']['wrist_roll']
    print(f"start mode {st['mode']} armed {st['armed']} fault {st['fault']} roll {roll0:.2f} jaw {st['joints']['gripper']:.1f}")
    if st['mode']!='REMOTE':
        print('mode ->',post('/mode',{'mode':'REMOTE'}));time.sleep(0.6)
    st=bridge()
    if st['mode']!='REMOTE' or not st['armed'] or st['fault']:
        raise SystemExit('bridge not ready')

    # video: capture the MJPEG stream to mp4 for the whole attempt
    recorder=None
    if not args.no_video:
        recorder=Recorder(VIDEO,fps=5.0); recorder.start()
        print('video recording started ->',VIDEO)

    steps=[
        ('open jaws',{'mode':'joint','joints':{},'jaw':args.jaw_open}),
        ('hover',{'mode':'xyz','x_mm':args.x,'y_mm':args.y,'z_mm':args.hover_z}),
        ('descend',{'mode':'xyz','x_mm':args.x,'y_mm':args.y,'z_mm':args.grasp_z}),
        ('close jaws',{'mode':'joint','joints':{},'jaw':args.jaw_close}),
        ('lift',{'mode':'xyz','x_mm':args.x,'y_mm':args.y,'z_mm':args.lift_z}),
    ]
    report=[]
    for name,body in steps:
        ok,info=dryrun(body)
        print(f"[{name}] dry-run {'OK' if ok else 'FAIL'}: {info}")
        if not ok:
            print('  ABORT (dry-run failed)');break
        # for joint moves, the body must carry the current joints; use current state
        if body['mode']=='joint':
            s=bridge()['joints']
            body={'mode':'joint','joints':{k:float(s[k]) for k in
                  ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')},
                  'jaw':body['jaw']}
        r=post('/move',body,timeout=90)
        print(f"[{name}] exec -> {json.dumps(r)[:220]}")
        report.append({'step':name,'dry':info,'resp':r})
        if isinstance(r,dict) and (r.get('http_error') or 'error' in str(r.get('detail',''))):
            print('  ABORT (move error)');break
        s=bridge()
        if s['fault'] or s['mode']!='REMOTE':
            print('  ABORT (fault/mode)');break
        if abs(s['joints']['wrist_roll']-roll0)>3.0:
            print(f"  ABORT (roll branch flip {roll0:.1f}->{s['joints']['wrist_roll']:.1f})");break
        time.sleep(0.5)

    final=bridge()
    print(f"final mode {final['mode']} fault {final['fault']} roll {final['joints']['wrist_roll']:.2f} "
          f"jaw {final['joints']['gripper']:.1f}")
    # restore: HOLD, jaws back to 40 (as found)
    print('restore HOLD:',post('/mode',{'mode':'HOLD'}))
    time.sleep(0.5)
    if recorder:
        sz=recorder.stop_and_save()
        print(f"video saved {VIDEO} ({sz} bytes)")

    out=ROOT/'reports'/f'grasp_attempt_{args.y}_{args.x}.json'
    out.write_text(json.dumps({'schema':'grasp-attempt/1','target_board_mm':[args.x,args.y],
                               'heights':[args.hover_z,args.grasp_z,args.lift_z],
                               'jaw_open':args.jaw_open,'jaw_close':args.jaw_close,
                               'roll_start':roll0,'report':report,
                               'final':{k:final[k] for k in ('mode','fault')}},indent=2)+'\n')
    print('wrote',out)

if __name__=='__main__':
    main()
