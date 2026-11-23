#!/usr/bin/env python3
"""Find the fixed finger-tip midpoint PIXEL, attempt 2.

Improvement over attempt 1: lock manual exposure (auto_exposure=1) so the
open/close diff is not contaminated by global brightness oscillation, and take
a per-pixel MEDIAN over 8 frames per jaw state instead of a 3-frame mean.

Jaw actuation authorized while Pi bridge powered. Restores all settings after.
"""
import json,sys,time,urllib.request
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from p4_irl_servo_detect import grab_frame

CONSOLE='http://192.168.1.171:8080';BRIDGE='http://192.168.1.167:8100'
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def post(p,b,t=60):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(b).encode(),method='POST',headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
def ctrl(**kw):
    return post('/camera/controls',{'controls':kw},t=10)

def set_jaw(v):
    st=get(BRIDGE+'/state')['joints']
    body={'mode':'joint','joints':{k:float(st[k]) for k in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')},'jaw':v}
    post('/move',body,t=60);time.sleep(1.0)

def median_frames(n=8):
    fs=[]
    for _ in range(n):
        fs.append(grab_frame(timeout=6))
        time.sleep(0.12)
    return np.median(np.stack(fs),axis=0).astype(np.uint8)

def main():
    st=get(BRIDGE+'/state')
    assert st['fault'] is None
    print('start',st['mode'],'jaw',round(st['joints']['gripper'],1))
    if st['mode']!='REMOTE': post('/mode',{'mode':'REMOTE'});time.sleep(0.6)
    orig=ctrl_d=None
    try:
        ctrls=get(CONSOLE+'/camera/controls')['controls']
        orig={c['name']:c['value'] for c in ctrls}
        print('orig auto_exposure',orig.get('auto_exposure'),'exposure',orig.get('exposure_time_absolute'))
        r=ctrl(auto_exposure=1)
        print('set manual ->',{x['name']:x['value'] for x in r['controls'] if x['name']=='auto_exposure'})
        time.sleep(0.5)
        set_jaw(100); f_open=median_frames(8)
        set_jaw(15);  f_closed=median_frames(8)
        cv2.imwrite('/tmp/f2_open.jpg',f_open);cv2.imwrite('/tmp/f2_closed.jpg',f_closed)
        diff=cv2.absdiff(f_open,f_closed)
        grey=cv2.cvtColor(diff,cv2.COLOR_BGR2GRAY)
        print('diff mean',round(float(grey.mean()),2),'p95',int(np.percentile(grey,95)))
        mask=(grey>30).astype(np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        n,lab,stats,cents=cv2.connectedComponentsWithStats(mask,8)
        comps=[]
        for i in range(1,n):
            area=int(stats[i,cv2.CC_STAT_AREA])
            x,y,w,h=stats[i,:4]
            if area>150 and w<160 and h<160:
                comps.append((area,cents[i].copy(),(int(x),int(y),int(w),int(h))))
        comps.sort(key=lambda c:-c[0])
        print('components',[(a,c.round(1).tolist(),bb) for a,c,bb in comps[:8]])
        cv2.imwrite('/tmp/f2_diff.png',diff);cv2.imwrite('/tmp/f2_mask.png',mask*255)
        if len(comps)>=2:
            pts=[c for a,c,bb in comps[:2]]
            mid=(pts[0]+pts[1])/2
            print('FINGER MIDPOINT PIXEL',mid.round(1).tolist())
            open('/tmp/finger_pixel2.json','w').write(json.dumps({'finger_mid_px':mid.round(1).tolist(),
                 'components':[{'area':a,'centroid':c.round(1).tolist(),'bbox':bb} for a,c,bb in comps[:6]]}))
            vis=f_open.copy()
            for a,c,bb in comps[:4]:
                cv2.rectangle(vis,(bb[0],bb[1]),(bb[0]+bb[2],bb[1]+bb[3]),(0,0,255),2)
            cv2.drawMarker(vis,(int(mid[0]),int(mid[1])),(255,255,0),cv2.MARKER_CROSS,26,3)
            cv2.imwrite('/tmp/f2_vis.jpg',vis)
        else:
            print('NOT ENOUGH COMPONENTS')
    finally:
        # restore everything
        try:
            if orig: ctrl(auto_exposure=int(orig['auto_exposure']))
            set_jaw(100)
            post('/mode',{'mode':'HOLD'})
            print('restored')
        except Exception as e: print('restore err',e)

if __name__=='__main__':main()
