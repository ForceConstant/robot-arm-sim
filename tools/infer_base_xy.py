#!/usr/bin/env python3
"""Inference: given a live frame + piece type + square, predict base XY (board mm).

Uses the leakage-free model trained by base_xy_model.py. Type comes from game
context (Brian's decision); vision supplies only the base centre for targeting.
"""
import sys,json,urllib.request
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from base_xy_model import image_features,ROOT,b
from p4_irl_servo_detect import grab_frame

def load_model():
    m=json.load(open(ROOT/'live/base_xy_model.json'))
    by={}
    for s in m['samples']:
        by.setdefault(s['piece'],[]).append((np.array(s['feat']),np.array(s['base_xy'])))
    return by

def predict(frame,piece,square,by=None):
    by=by or load_model()
    f,sol=image_features(frame,square)
    if f is None:return None,'no PnP'
    samples=by.get(piece)
    if not samples:return None,f'unknown type {piece}'
    d=[np.linalg.norm(f-x) for x,_ in samples]
    j=int(np.argmin(d))
    return samples[j][1].copy(),{'nn_distance':round(float(d[j]),3),'n':len(samples)}

if __name__=='__main__':
    piece=sys.argv[1] if len(sys.argv)>1 else 'knight'
    square=sys.argv[2] if len(sys.argv)>2 else 'f6'
    by=load_model()
    frame=grab_frame(timeout=8)
    xy,info=predict(frame,piece,square,by)
    print(f'predict base for {piece} @ {square}: {xy} ({info})')
