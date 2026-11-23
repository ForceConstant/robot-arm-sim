#!/usr/bin/env python3
"""Probe: set a joint pose, wait, read back what the arm ACTUALLY reached.
Tests whether the joint move lands where commanded (read-lag vs real error)."""
import json,time,urllib.request
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
def post(p,body,t=40):
    req=urllib.request.Request(C+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
def get(u):
    with urllib.request.urlopen(u,timeout=8) as r:return json.load(r)
TARGET=dict(shoulder_pan=-19.78,shoulder_lift=-84.09,elbow_flex=91.56,wrist_flex=61.05,wrist_roll=67.34)
post('/mode',{'mode':'REMOTE'});time.sleep(1)
print('cmd',TARGET)
r=post('/move',{'mode':'joint','joints':TARGET,'jaw':100})
print('resp',r.get('error_deg'),r.get('detail'))
for k in range(8):
    time.sleep(1)
    j=get(B+'/state')['joints']
    print(f'  t+{k+1}s',{n:round(j[n],2) for n in TARGET})
post('/mode',{'mode':'HOLD'})
