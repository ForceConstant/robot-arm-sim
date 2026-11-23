#!/usr/bin/env python3
"""Adaptive clean multi-view capture with VERIFIED camera separation (2026-11-22).

Problem: joint commands land with ~2deg error (two IK basins), so the same
"base" pose produced camera z=105 or 117, and lift-8 often didn't move at all —
 most "2 views" were identical. This tool fixes that by MEASURING the actual
camera pose (PnP) after every move and only accepting a capture when it is
genuinely distinct (>= MIN_SEP mm) from views already kept for that piece.

Per placement: try a list of joint offsets from base; after each move compute
camera_in_board via PnP; if target in frame, rms ok, and camera separation from
all kept views >= MIN_SEP, save a CLEAN frame via /capture-view. Stop when
TARGET_VIEWS distinct views are collected (or offsets exhausted).

Jaws open, no grasp, arm ends HOLD.

    PYTHONPATH=. .venv/bin/python tools/p4_capture_adaptive.py \
      --placements-file tools/piece_placements.json --label adapt1 \
      --target-views 4 --min-sep 8
"""
import argparse, hashlib, json, sys, time, urllib.request, urllib.error
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board

C='http://192.168.1.171:8080'; B='http://192.168.1.167:8100'
ROOT=Path('/home/brian/robot-arm-sim'); CAPDIR=ROOT/'live/captures'
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
BASE=dict(shoulder_pan=-19.78,shoulder_lift=-84.09,elbow_flex=91.56,
          wrist_flex=61.05,wrist_roll=67.34,gripper=100.0)
# Broad set of reachable offsets; tool filters by measured separation
CANDIDATES=[
    ('base',      {}),
    ('lift_m8',   {'shoulder_lift':-8}),
    ('lift_p4',   {'shoulder_lift':4}),
    ('pan_m4',    {'shoulder_pan':-4}),
    ('pan_p4',    {'shoulder_pan':4}),
    ('wflex_m6',  {'wrist_flex':-6}),
    ('wflex_p6',  {'wrist_flex':6}),
    ('lift_m8_pan_m4', {'shoulder_lift':-8,'shoulder_pan':-4}),
    ('lift_p4_wflex_m4', {'shoulder_lift':4,'wrist_flex':-4}),
    ('elbow_m6',  {'elbow_flex':-6}),
    ('lift_m12',  {'shoulder_lift':-12}),
]
def post(p,body,t=60,retry=2):
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
        return {'http_error':e.code,'detail':e.read().decode()[:120]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def sq_xy(sq):
    return ((ord(sq[0].lower())-97)+0.5)*28.0-112.0,(int(sq[1])-0.5)*28.0-112.0

def move_to(jd):
    body={'mode':'joint','joints':jd,'jaw':100}
    dr=post('/move',{**body,'dry_run':True},t=25)
    md=dr.get('max_joint_delta_deg')
    if dr.get('http_error') or dr.get('detail') or md is None or md>35:
        return False
    post('/move',body,t=60);time.sleep(2.2);return True

def frame_pnp():
    """Grab ONE frame via the chunk-reading helper (plain urlopen().read()
    blocks forever on the infinite MJPEG stream) and return PnP."""
    from p4_irl_servo_detect import grab_frame
    for k in range(8):
        try:
            img=grab_frame(timeout=6)
        except Exception:
            time.sleep(0.3);continue
        h,w=img.shape[:2]
        if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
        if (img.shape[1],img.shape[0])!=(640,480):
            img=cv2.resize(img,(640,480))
        rect=b.rectify(img);sol,d=b.solve(rect)
        if sol is None or len(d)<2 or sol['rms_px']>8:continue
        return np.array(sol['camera_in_board_mm'],float),[int(x.marker_id) for x in d],round(sol['rms_px'],2)
    return None

def clean_capture(retries=5):
    for k in range(retries):
        info=post('/capture-view',{'name':'adapt'},t=30)
        if info.get('name'):return info
        time.sleep(1.0)
    return None

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--placements-file',default='tools/piece_placements.json')
    ap.add_argument('--label',default='adapt1')
    ap.add_argument('--out',default='.private/real_piece_views')
    ap.add_argument('--target-views',type=int,default=4)
    ap.add_argument('--min-sep',type=float,default=8.0)
    ap.add_argument('--poses',default='',help='comma list of candidate names to restrict to')
    a=ap.parse_args()
    if a.poses:
        want=set(a.poses.split(','))
        CANDIDATES[:]=[c for c in CANDIDATES if c[0] in want]
    plist=json.load(open(ROOT/a.placements_file))['placements']
    outdir=ROOT/a.out/a.label;outdir.mkdir(parents=True,exist_ok=True)
    post('/mode',{'mode':'REMOTE'});time.sleep(1.2)
    assert get(B+'/state')['mode']=='REMOTE','not REMOTE'
    ctrls={c['name']:c['value'] for c in get(C+'/camera/controls')['controls']}
    manifest={'schema':'real-piece-views-adaptive/1','label':a.label,'base_pose':BASE,
              'target_views':a.target_views,'min_sep_mm':a.min_sep,
              'camera_controls':ctrls,'intrinsics':{'fx':b.fx,'fy':b.fy,'cx':b.cx,'cy':b.cy},
              'placements':[]}
    total=0
    print(f'target {a.target_views} distinct views, min_sep {a.min_sep}mm, poses {[c[0] for c in CANDIDATES]}',flush=True)
    for pl in plist:
        piece,sq=pl['piece'],pl['square']; x,y=sq_xy(sq)
        entry={'piece':piece,'square':sq,'square_xy_mm':[round(x,1),round(y,1)],'views':[]}
        kept=[]  # list of (cam, view record)
        print(f'\n=== {piece} @ {sq} ===',flush=True)
        for name,off in CANDIDATES:
            if len(kept)>=a.target_views:break
            jd=dict(BASE)
            for kj,dj in off.items():jd[kj]=BASE[kj]+dj
            if not move_to(jd):
                print(f'  {name}: move fail',flush=True);continue
            pc=frame_pnp()
            if pc is None:
                print(f'  {name}: no PnP',flush=True);continue
            cam,ids,rms=pc
            # separation from kept views
            seps=[float(np.linalg.norm(cam-k[0])) for k in kept]
            min_sep=min(seps) if seps else 999
            # a genuinely distinct camera pose is the acceptance criterion
            if min_sep < a.min_sep:
                print(f'  {name}: cam {cam.round(0)} sep {min_sep:.1f}mm < {a.min_sep} skip',flush=True);continue
            # capture clean frame
            ci=clean_capture()
            if ci is None:
                print(f'  {name}: capture-view fail',flush=True);continue
            jp=CAPDIR/f'{ci["name"]}.jpg'
            img=cv2.imread(str(jp))
            if img is None:continue
            h,w=img.shape[:2]
            if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
            if (img.shape[1],img.shape[0])!=(640,480):img=cv2.resize(img,(640,480))
            name_img=f'{piece}_{sq}_{a.label}_{name}.jpg'
            cv2.imwrite(str(outdir/name_img),img)
            sha=hashlib.sha256((outdir/name_img).read_bytes()).hexdigest()
            rec={'view':name,'image':name_img,'sha256':sha,'cam':cam.round(2).tolist(),
                 'ids':ids,'rms':rms,'min_sep_mm':round(min_sep,1),'offset':off,
                 'capture_meta':ci}
            entry['views'].append(rec);kept.append((cam,rec));total+=1
            print(f'  {name}: KEPT cam {cam.round(0)} sep {min_sep:.1f}mm rms {rms}',flush=True)
        entry['n_distinct']=len(kept)
        print(f'  -> {len(kept)} distinct views',flush=True)
        manifest['placements'].append(entry)
    post('/mode',{'mode':'HOLD'})
    (outdir/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'\nDONE {total} distinct clean views -> {outdir}; HOLD',flush=True)

if __name__=='__main__':main()
