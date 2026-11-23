#!/usr/bin/env python3
"""Capture REAL multi-view scenes for the piece dataset (2026-11-22).

Uses the 5 empirically-good joint viewpoints (markers>=3, target in frame) from
reports/capture_pose_search_20261122.json, expressed as offsets from the known
good base pose. For each placement in a placements config:
  - move to each viewpoint (joint move, dry-run checked, roll branch preserved)
  - grab raw frame + PnP + pose + tip + camera-controls metadata
  - save image (sha256) + per-view metadata
  - build a rectified top-down mosaic from all views for base-centre labeling

Jaws open, no grasp. Motion standing-authorized (Pi powered), all legs logged.

Usage:
  # single placement (POC, pawn already at e5)
  PYTHONPATH=. .venv/bin/python tools/p4_capture_scenes.py --placements 'pawn:e5' --label poc
  # full set once pieces are placed
  PYTHONPATH=. .venv/bin/python tools/p4_capture_scenes.py --placements-file tools/piece_placements.json --label full1
"""
import argparse, hashlib, json, sys, time, urllib.request, urllib.error
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw

CONSOLE='http://192.168.1.171:8080';BRIDGE='http://192.168.1.167:8100'
ROOT=Path('/home/brian/robot-arm-sim')
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
JOINTS=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
BASE=dict(shoulder_pan=-19.78,shoulder_lift=-84.09,elbow_flex=91.56,
          wrist_flex=61.05,wrist_roll=67.34,gripper=100.0)
# Verified 2026-11-22 (p4_verify_view_poses): only these reliably give >=3
# markers with e5 in frame. pan_m4/wflex_* failed; lift_m8 was marginal (2).
VIEW_OFFSETS=[('base',{}),('lift_p4',{'shoulder_lift':4}),
              ('lift_m8',{'shoulder_lift':-8})]
def post(p,body,t=60,retry=1):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:
        if retry and e.code==409:
            # mode race: re-assert REMOTE and retry once
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

def move_to(jd):
    body={'mode':'joint','joints':jd,'jaw':100}
    dr=post('/move',{**body,'dry_run':True},t=25)
    md=dr.get('max_joint_delta_deg')
    # proper None check: a legitimate 0.0 delta is falsy but valid
    if dr.get('http_error') or dr.get('detail') or md is None or md>30:
        mode_now=get(BRIDGE+'/state').get('mode')
        print(f'    [move_to DEBUG] mode={mode_now} dry={json.dumps(dr)[:200]}',flush=True)
        return False,dr
    r=post('/move',body,t=60)
    time.sleep(1.0)
    return True,r

def grab_meta():
    for k in range(10):
        try:
            f=grab_frame(timeout=6)
        except Exception as ex:
            # transient stream timeout (other viewers share the MJPEG); retry
            time.sleep(0.3);continue
        raw=to_raw(f)
        rect=b.rectify(raw);sol,d=b.solve(rect)
        # With pieces on the board only ~2 markers stay visible; PnP is still
        # valid at 2 markers (rms ~4). Record rms in metadata for later filtering.
        if sol is None or len(d)<2 or sol['rms_px']>8:
            time.sleep(0.15);continue
        ids=[int(x.marker_id) for x in d]
        return {'frame':raw,'rect':rect,'sol':sol,'markers':len(d),'ids':ids,
                'rms':round(sol['rms_px'],2),
                'cam':np.array(sol['camera_in_board_mm']).round(2).tolist()}
    return None

def mosaic(views,px_per_mm=6.0):
    """Average rectified top-down views into one board-plane mosaic."""
    n=int(round(256*px_per_mm))  # 256mm board field
    acc=None; wsum=None  # wsum is 2D (H,W)
    for v in views:
        img,w,_,_=b.rectified(v['frame'],v['sol'],px_per_mm)
        f=img.astype(np.float32)
        # rectified()'s valid mask is 3-channel (warps a BGR full frame);
        # collapse to 2D by taking one channel.
        m=(w[...,0]>0).astype(np.float32)
        m3=m[:,:,None]
        acc=f*m3 if acc is None else acc+f*m3
        wsum=m if wsum is None else wsum+m
    if acc is None:return None
    return np.clip(acc/np.maximum(wsum[:,:,None],1),0,255).astype(np.uint8)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--placements',default='')  # comma list piece:square
    ap.add_argument('--placements-file',default='')
    ap.add_argument('--label',default='scene1')
    ap.add_argument('--out',default='.private/real_piece_views')
    a=ap.parse_args()
    if a.placements_file:
        placements=json.load(open(a.placements_file))
        plist=[(p['piece'],p['square']) for p in placements['placements']]
    else:
        plist=[tuple(x.split(':')) for x in a.placements.split(',') if x]
    outdir=ROOT/a.out/a.label;outdir.mkdir(parents=True,exist_ok=True)
    st=get(BRIDGE+'/state');assert st['fault'] is None,st['fault']
    # Force REMOTE and VERIFY - a stale HOLD makes every /move 409 with
    # md=None, which previously looked like a silent capture failure.
    post('/mode',{'mode':'REMOTE'});time.sleep(0.8)
    st=get(BRIDGE+'/state')
    assert st['mode']=='REMOTE', f'failed to enter REMOTE: {st["mode"]}'
    # CRITICAL: return to the EXACT base pose before applying offsets, otherwise
    # offsets compound on whatever pose the previous run left behind.
    ok,r=move_to(dict(BASE))
    print('-> base pose',ok,r.get('detail') if isinstance(r,dict) else '',flush=True)
    time.sleep(0.8)
    ctrls={c['name']:c['value'] for c in get(CONSOLE+'/camera/controls')['controls']}
    manifest={'schema':'real-scene-capture/1','label':a.label,'base_pose':BASE,
              'view_offsets':VIEW_OFFSETS,'camera_controls':ctrls,
              'intrinsics':{'fx':b.fx,'fy':b.fy,'cx':b.cx,'cy':b.cy},
              'calibration':'.private/phase11d/calibration.json','placements':[]}
    print('base',BASE,'placements',plist,flush=True)
    for piece,sq in plist:
        x,y=sq_xy(sq)
        entry={'piece':piece,'square':sq,'square_xy_mm':[round(x,1),round(y,1)],'views':[]}
        print(f'\n=== {piece} @ {sq} ({x:.1f},{y:.1f}) ===',flush=True)
        captured=[]
        for vname,off in VIEW_OFFSETS:
            jd=dict(BASE)
            for kj,dj in off.items():
                jd[kj]=BASE[kj]+dj   # ADD offset to base, don't replace
            ok,r=move_to(jd)
            if not ok:
                print(f'  {vname}: dry FAIL {r.get("detail")}',flush=True);continue
            m=grab_meta()
            if not m:
                print(f'  {vname}: no PnP frame',flush=True);continue
            img=m['frame'];sol=m['sol']
            # is target square in frame?
            p=b.project(sol,np.array([x,y,0.]),0.)
            inside=0<=p[0]<640 and 0<=p[1]<480
            name=f'{piece}_{sq}_{a.label}_{vname}.jpg'
            cv2.imwrite(str(outdir/name),img)
            sha=hashlib.sha256((outdir/name).read_bytes()).hexdigest()
            pose=get(CONSOLE+'/pose');keep=get(CONSOLE+'/keep-in')
            rec={'view':vname,'image':name,'sha256':sha,'ids':m['ids'],'rms':m['rms'],
                 'cam':m['cam'],'target_px':[round(float(p[0]),1),round(float(p[1]),1)],
                 'target_inside':bool(inside),'joints_deg':pose.get('joints_deg'),
                 'tool_mm':pose.get('tool_mm'),'tip_mm':keep.get('tip_mm')}
            entry['views'].append(rec);captured.append(m)
            print(f'  {vname}: ids {m["ids"]} rms {m["rms"]} targetpx {rec["target_px"]} inside {inside}',flush=True)
        mos=mosaic(captured)
        if mos is not None:
            mname=f'{piece}_{sq}_{a.label}_mosaic.png'
            cv2.imwrite(str(outdir/mname),mos)
            entry['mosaic']=mname
            print(f'  mosaic -> {mname}',flush=True)
        manifest['placements'].append(entry)
    post('/mode',{'mode':'HOLD'})
    (outdir/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    total=sum(len(p['views']) for p in manifest['placements'])
    print(f'\nDONE {total} views across {len(plist)} placements -> {outdir}; HOLD',flush=True)

if __name__=='__main__':main()
