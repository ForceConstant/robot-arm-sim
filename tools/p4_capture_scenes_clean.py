#!/usr/bin/env python3
"""Capture CLEAN (overlay-free) multi-view wrist images for the piece dataset.

Unlike p4_capture_scenes.py (which pulled from /stream and baked in the
detection HUD), this uses POST /capture-view which saves the RAW frame before
overlay. PnP pose is computed locally from the clean image. Metadata (joints,
tip, controls) is read from the console at capture time.

Per placement, per view:
  - move to viewpoint (joint, dry-run checked, REMOTE guaranteed)
  - POST /capture-view -> clean JPEG in live/captures/ + .json metadata
  - read the clean JPEG, compute PnP locally, save into dataset dir (+sha256)
  - build rectified top-down mosaic from clean frames

Jaws open, no grasp, arm ends HOLD.

    PYTHONPATH=. .venv/bin/python tools/p4_capture_scenes_clean.py \
       --placements-file tools/piece_placements.json --label clean1
"""
import argparse, hashlib, json, sys, time, urllib.request, urllib.error
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board

CONSOLE='http://192.168.1.171:8080';BRIDGE='http://192.168.1.167:8100'
ROOT=Path('/home/brian/robot-arm-sim')
CAPDIR=ROOT/'live/captures'
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
BASE=dict(shoulder_pan=-19.78,shoulder_lift=-84.09,elbow_flex=91.56,
          wrist_flex=61.05,wrist_roll=67.34,gripper=100.0)
VIEW_OFFSETS=[('base',{}),('lift_m8',{'shoulder_lift':-8})]
def post(p,body,t=60,retry=1):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:
        if retry and e.code==409:
            time.sleep(0.8)
            try:
                rr=urllib.request.Request(CONSOLE+'/mode',data=b'{"mode":"REMOTE"}',method='POST',headers={'Content-Type':'application/json'})
                with urllib.request.urlopen(rr,timeout=10) as r:r.read()
            except Exception:pass
            time.sleep(0.8)
            return post(p,body,t=t,retry=0)
        return {'http_error':e.code,'detail':e.read().decode()[:120]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def sq_xy(sq):
    return ((ord(sq[0].lower())-97)+0.5)*28.0-112.0,(int(sq[1])-0.5)*28.0-112.0

def to_landscape(img):
    h,w=img.shape[:2]
    if h>w: img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
    if (img.shape[1],img.shape[0])!=(640,480): img=cv2.resize(img,(640,480))
    return img

def move_to(jd):
    body={'mode':'joint','joints':jd,'jaw':100}
    dr=post('/move',{**body,'dry_run':True},t=25)
    md=dr.get('max_joint_delta_deg')
    if dr.get('http_error') or dr.get('detail') or md is None or md>30:
        return False,dr
    r=post('/move',body,t=60)
    # capture-view rejects frames still moving (moved_px>~2). Longer settle.
    time.sleep(2.5)
    return True,r

def clean_capture(retries=4):
    """POST capture-view, retry if it reports the arm still moving."""
    for k in range(retries):
        info=post('/capture-view',{'name':'dataset'},t=30)
        if info.get('name'):
            break
        time.sleep(1.0)
    if 'http_error' in info or not info.get('name'):
        return None,info
    name=info['name']
    jp=CAPDIR/f'{name}.jpg'
    if not jp.exists(): return None,{'error':'no jpg'}
    img=cv2.imread(str(jp))
    return (img,info)

def pnp(img):
    rect=b.rectify(img);sol,d=b.solve(rect)
    if sol is None or len(d)<2 or sol['rms_px']>8:return None
    return {'rect':rect,'sol':sol,'ids':[int(x.marker_id) for x in d],
            'rms':round(sol['rms_px'],2),'cam':np.array(sol['camera_in_board_mm']).round(2).tolist()}

def mosaic(views,px_per_mm=6.0):
    acc=None;wsum=None
    for v in views:
        img,w,_,_=b.rectified(v['img'],v['pnp']['sol'],px_per_mm)
        f=img.astype(np.float32)
        m=(w[...,0]>0).astype(np.float32) if w.ndim==3 else (w>0).astype(np.float32)
        m3=m[:,:,None]
        acc=f*m3 if acc is None else acc+f*m3
        wsum=m if wsum is None else wsum+m
    if acc is None:return None
    return np.clip(acc/np.maximum(wsum[:,:,None],1),0,255).astype(np.uint8)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--placements-file',default='tools/piece_placements.json')
    ap.add_argument('--label',default='clean1')
    ap.add_argument('--out',default='.private/real_piece_views')
    a=ap.parse_args()
    placements=json.load(open(ROOT/a.placements_file))['placements']
    outdir=ROOT/a.out/a.label;outdir.mkdir(parents=True,exist_ok=True)
    post('/mode',{'mode':'REMOTE'});time.sleep(1.2)
    assert get(BRIDGE+'/state')['mode']=='REMOTE','not REMOTE'
    ctrls={c['name']:c['value'] for c in get(CONSOLE+'/camera/controls')['controls']}
    manifest={'schema':'real-piece-views-clean/1','label':a.label,
              'camera_controls':ctrls,'intrinsics':{'fx':b.fx,'fy':b.fy,'cx':b.cx,'cy':b.cy},
              'calibration':'.private/phase11d/calibration.json','placements':[]}
    print('placements',[(p['piece'],p['square']) for p in placements],flush=True)
    total=0
    for pl in placements:
        piece,sq=pl['piece'],pl['square']
        x,y=sq_xy(sq)
        entry={'piece':piece,'square':sq,'square_xy_mm':[round(x,1),round(y,1)],'views':[]}
        print(f'\n=== {piece} @ {sq} ===',flush=True)
        captured=[]
        for vname,off in VIEW_OFFSETS:
            jd=dict(BASE)
            for kj,dj in off.items():jd[kj]=BASE[kj]+dj
            ok,r=move_to(jd)
            if not ok:
                print(f'  {vname}: move fail {r.get("detail")}',flush=True);continue
            img,info=clean_capture()
            if img is None:
                print(f'  {vname}: capture fail {info}',flush=True);continue
            img=to_landscape(img)
            pp=pnp(img)
            if pp is None:
                print(f'  {vname}: no PnP',flush=True);continue
            pv=b.project(pp['sol'],np.array([x,y,0.]),0.)
            inside=0<=pv[0]<640 and 0<=pv[1]<480
            name=f'{piece}_{sq}_{a.label}_{vname}.jpg'
            cv2.imwrite(str(outdir/name),img)
            sha=hashlib.sha256((outdir/name).read_bytes()).hexdigest()
            rec={'view':vname,'image':name,'sha256':sha,'ids':pp['ids'],'rms':pp['rms'],
                 'cam':pp['cam'],'target_px':[round(float(pv[0]),1),round(float(pv[1]),1)],
                 'target_inside':bool(inside),'capture_meta':info}
            entry['views'].append(rec)
            captured.append({'img':img,'pnp':pp})
            total+=1
            print(f'  {vname}: ids {pp["ids"]} rms {pp["rms"]} targetpx {rec["target_px"]} inside {inside}',flush=True)
        mos=mosaic(captured)
        if mos is not None:
            mn=f'{piece}_{sq}_{a.label}_mosaic.png'
            cv2.imwrite(str(outdir/mn),mos);entry['mosaic']=mn
            print(f'  mosaic -> {mn}',flush=True)
        manifest['placements'].append(entry)
    post('/mode',{'mode':'HOLD'})
    (outdir/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'\nDONE {total} clean views -> {outdir}; HOLD',flush=True)

if __name__=='__main__':main()
