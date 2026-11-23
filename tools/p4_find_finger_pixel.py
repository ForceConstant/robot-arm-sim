#!/usr/bin/env python3
"""Find the finger-tip midpoint PIXEL by differencing jaw-open vs jaw-closed
frames at a fixed arm pose. The fingers are rigid to the wrist camera, so the
only things that move when the jaw actuates are the finger tips themselves.

Jaw actuation is standing-authorized while the Pi bridge is powered.
No grip on the piece: the arm stays at its pose, only the jaw value changes.

Result: the fixed pixel of the finger-tip midpoint in the RAW camera frame,
written to /tmp/finger_pixel.json and used by the overlay's cyan marker.
"""
import json,sys,time,urllib.request
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from p4_irl_servo_detect import grab_frame

CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def post(p,b,t=60):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(b).encode(),method='POST',headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)

def set_jaw(v):
    st=get(BRIDGE+'/state')['joints']
    body={'mode':'joint','joints':{k:float(st[k]) for k in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')},'jaw':v}
    post('/move',body,t=60);time.sleep(0.8)

def avg_frames(n=3):
    acc=None
    for _ in range(n):
        f=grab_frame(timeout=6).astype(np.float32)
        acc=f if acc is None else acc+f
        time.sleep(0.12)
    return acc/n

def main():
    st=get(BRIDGE+'/state')
    assert st['fault'] is None, st['fault']
    print('start',st['mode'],'jaw',round(st['joints']['gripper'],1))
    if st['mode']!='REMOTE':
        post('/mode',{'mode':'REMOTE'}); time.sleep(0.6)
    # capture many frame pairs across a few open/close cycles; median the diff
    dips=[]
    for cycle in range(2):
        set_jaw(100); f_open=avg_frames(3)
        set_jaw(15);  f_closed=avg_frames(3)
        diff=cv2.absdiff(f_open.astype(np.uint8),f_closed.astype(np.uint8))
        grey=cv2.cvtColor(diff,cv2.COLOR_BGR2GRAY)
        mask=(grey>25).astype(np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        n,lab,stats,cents=cv2.connectedComponentsWithStats(mask,8)
        comps=[]
        for i in range(1,n):
            area=stats[i,cv2.CC_STAT_AREA]
            if area>60:
                comps.append((area,cents[i]))
        comps.sort(key=lambda c:-c[0])
        print('cycle',cycle,'components',[(a,c.round(1).tolist()) for a,c in comps[:5]])
        cv2.imwrite(f'/tmp/jaw_diff_{cycle}.png',diff)
        cv2.imwrite(f'/tmp/jaw_open_{cycle}.jpg',f_open.astype(np.uint8))
        cv2.imwrite(f'/tmp/jaw_closed_{cycle}.jpg',f_closed.astype(np.uint8))
        if comps:
            # the two largest moving blobs are the finger tips
            pts=[c for a,c in comps[:2]]
            if len(pts)==2:
                mid=(pts[0]+pts[1])/2
                dips.append(mid)
    finally_jaw=100
    set_jaw(finally_jaw)
    post('/mode',{'mode':'HOLD'})
    if dips:
        mid=np.median(np.array(dips),axis=0)
        print('FINGER MIDPOINT RAW PIXEL',mid.round(1).tolist(),'n',len(dips))
        open('/tmp/finger_pixel.json','w').write(json.dumps({'finger_mid_px':mid.round(1).tolist(),'n':len(dips)}))
        # draw it on an open-jaw frame
        img=cv2.imread('/tmp/jaw_open_0.jpg')
        cv2.drawMarker(img,(int(mid[0]),int(mid[1])),(255,255,0),cv2.MARKER_CROSS,26,3)
        cv2.imwrite('/tmp/finger_pixel_vis.jpg',img)
    else:
        print('NO FINGER PIXELS FOUND')

if __name__=='__main__':main()
