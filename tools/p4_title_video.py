#!/usr/bin/env python3
"""Build a titled, steady-rate grasp video from a recorded mp4."""
import cv2, numpy as np, sys

SRC=sys.argv[1] if len(sys.argv)>1 else '/tmp/grasp3.mp4'
OUT=sys.argv[2] if len(sys.argv)>2 else '/tmp/grasp3_titled.mp4'
TITLE_LINES=sys.argv[3] if len(sys.argv)>3 else 'GRASP ATTEMPT 3'
W,H=480,640
FPS=6

cap=cv2.VideoCapture(SRC); frames=[]
while True:
    ok,f=cap.read()
    if not ok:break
    frames.append(cv2.resize(f,(W,H)))
cap.release()
print('source frames',len(frames))

title=np.full((H,W,3),(16,22,30),np.uint8)
def t(s,y,sz=0.5,c=(235,240,245),th=1):
    cv2.putText(title,s,(18,y),cv2.FONT_HERSHEY_SIMPLEX,sz,c,th,cv2.LINE_AA)
cv2.putText(title,'ROBOT GRASP - ATTEMPT 3',(18,55),cv2.FONT_HERSHEY_SIMPLEX,0.6,(90,220,220),2,cv2.LINE_AA)
t('Fix from attempts 1-2:',100,0.52,(255,180,90))
t('red cross was the NOMINAL square centre,',135,0.46)
t('not the piece. Aim at MEASURED piece.',163,0.46)
t('Measured piece centre (mount-free PnP):',215,0.52,(90,220,220))
t('  board (8.7, 20.1) mm',245,0.5)
t('  vs nominal (14, 14) mm',275,0.5)
t('  -> ~15 mm offset, mostly in +y',305,0.5)
t('Sequence: open 100 / hover 45 /',360,0.48)
t('descend 14 / close 18 / lift 70',388,0.48)
t('Note: stream frames are portrait;',440,0.45,(180,200,215))
t('camera calibration is 640x480.',468,0.45,(180,200,215))
t('RESULT: FAILED - piece on board',545,0.54,(90,90,255),2)
t('Jaws closed ~15mm off; pawn stayed put.',580,0.44)

allf=[title]*(FPS*3)+frames+[frames[-1]]*(FPS*1)
wr=cv2.VideoWriter(OUT,cv2.VideoWriter_fourcc(*'mp4v'),FPS,(W,H))
for f in allf:wr.write(f)
wr.release()
print('wrote',OUT,len(allf),'frames',round(len(allf)/FPS,1),'s')
