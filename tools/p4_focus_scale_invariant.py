#!/usr/bin/env python3
"""Scale-invariant focus sweep: at each tip height, capture a CLEAN frame and
compute Laplacian sharpness both raw and on a board-rectified top-down view at
fixed px/mm. The rectified score removes magnification, isolating true defocus.
"""
import json,sys,time,urllib.request,urllib.error
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
CAP='/home/brian/robot-arm-sim/live/captures'
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
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
def clean_frame():
    info=post('/capture-view',{'name':'fsel'},t=30)
    if not info.get('name'):return None,info
    return cv2.imread(CAP+'/'+info['name']+'.jpg'),info
def sharp(g):
    return float(cv2.Laplacian(g,cv2.CV_64F).var())
def main():
    st=get(B+'/state');assert st['fault'] is None
    roll0=st['joints']['wrist_roll']
    post('/mode',{'mode':'REMOTE'});time.sleep(1)
    heights=[60,50,45,40,70,85]  # step down from safe, then up
    results=[]
    for z in heights:
        body={'mode':'xyz','x_mm':0,'y_mm':0,'z_mm':z,'jaw':100,'roll_lock':True}
        dr=post('/move',{**body,'dry_run':True},t=25)
        if dr.get('detail'):print(f'z={z} dry fail',flush=True);continue
        post('/move',body,t=60);time.sleep(2.5)
        j=get(B+'/state')
        if j['fault'] or abs(j['joints']['wrist_roll']-roll0)>5:print('guard abort',flush=True);break
        tip=get(C+'/keep-in')['tip_mm']
        img,info=clean_frame()
        if img is None:
            time.sleep(1);img,info=clean_frame()
        if img is None:print(f'z={z} no clean frame',flush=True);continue
        h,w=img.shape[:2]
        if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
        grey=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        sol,d=b.solve(b.rectify(img))
        raw=sharp(grey)
        rect=[None]
        if sol is not None and len(d)>=2:
            top,valid,n,scale=b.rectified(img,sol,6.0)  # fixed 6px/mm
            g2=cv2.cvtColor(top,cv2.COLOR_BGR2GRAY)
            m=valid[...,0]>0 if valid.ndim==3 else valid>0
            rect=[round(sharp(g2[m]),1)]
        print(f'z_cmd={z} tip_z={tip[2]:.1f} raw={raw:.1f} rectified={rect[0]} markers={len(d) if sol else 0}',flush=True)
        results.append({'z':z,'tip_z':tip[2],'raw':round(raw,1),'rectified':rect[0],'markers':len(d) if sol else 0})
    post('/mode',{'mode':'HOLD'})
    json.dump(results,open('/home/brian/robot-arm-sim/reports/focus_scale_invariant_20261123.json','w'),indent=2)
    print('HOLD; report',flush=True)
if __name__=='__main__':main()
