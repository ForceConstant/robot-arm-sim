#!/usr/bin/env python3
"""Sweep camera standoff (tip z) and measure clean-frame focus score.

Finds whether the captures are soft because the camera is too far/near. Uses
absolute xyz tip positions over a fixed board point at several heights, then
measures a Laplacian focus score on a clean capture-view frame (no HUD).
Motion standing-authorized (Pi powered); dry-run each leg; jaws open.
"""
import json,sys,time,urllib.request,urllib.error
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
CAP='/home/brian/robot-arm-sim/live/captures'
def post(p,body,t=60,retry=1):
    req=urllib.request.Request(C+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:
        if retry and e.code==409:
            time.sleep(0.8)
            try:
                rr=urllib.request.Request(C+'/mode',data=b'{"mode":"REMOTE"}',method='POST',headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(rr,timeout=10) as r:r.read()
            except Exception:pass
            time.sleep(0.8);return post(p,body,t=t,retry=retry-1)
        return {'http_error':e.code}
def get(u):
    with urllib.request.urlopen(u,timeout=8) as r:return json.load(r)
def focus_clean():
    info=post('/capture-view',{'name':'focus'},t=30)
    if not info.get('name'):return None,info
    img=cv2.imread(CAP+'/'+info['name']+'.jpg',cv2.IMREAD_GRAYSCALE)
    if img is None:return None,'no img'
    lap=cv2.Laplacian(img,cv2.CV_64F)
    score=float(lap.var())
    # also central-region score (where pieces are)
    h,w=img.shape;c=img[h//4:3*h//4,w//4:3*w//4]
    cscore=float(cv2.Laplacian(c,cv2.CV_64F).var())
    return {'score':round(score,1),'centre':round(cscore,1)},info
def main():
    st=get(B+'/state');assert st['fault'] is None
    roll0=st['joints']['wrist_roll']
    post('/mode',{'mode':'REMOTE'});time.sleep(1)
    # sweep tip z at board centre x,y
    heights=[60,50,45,40,35,30,70,85]  # start near current pose, step down, then up
    results=[]
    for z in heights:
        body={'mode':'xyz','x_mm':0,'y_mm':0,'z_mm':z,'jaw':100,'roll_lock':True}
        dr=post('/move',{**body,'dry_run':True},t=25)
        pe=dr.get('requested',{}).get('position_error_mm')
        if dr.get('detail') or (pe is not None and pe>10):
            print(f'z={z}: dry fail {dr.get("detail")} pe={pe}',flush=True);continue
        post('/move',body,t=60);time.sleep(2.0)
        j=get(B+'/state')
        if j['fault'] or abs(j['joints']['wrist_roll']-roll0)>5:
            print('guard abort',flush=True);break
        tip=get(C+'/keep-in')['tip_mm']
        fs,info=focus_clean()
        if fs is None:print(f'z={z}: focus fail {info}',flush=True);continue
        print(f'z_cmd={z} tip_z={tip[2]:.1f} focus={fs}',flush=True)
        results.append({'z_cmd':z,'tip':tip,'focus':fs})
    post('/mode',{'mode':'HOLD'})
    json.dump(results,open('/home/brian/robot-arm-sim/reports/focus_sweep_20261123.json','w'),indent=2)
    print('HOLD; report',flush=True)
if __name__=='__main__':main()
