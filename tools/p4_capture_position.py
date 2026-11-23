#!/usr/bin/env python3
"""Position-based clean capture for far squares (2026-11-23).

Places the jaw tip over a target square (absolute xyz) so the camera's natural
view centers that square, then captures several clean views with small joint
offsets for angle diversity (verified by camera separation via PnP).

    PYTHONPATH=. .venv/bin/python -u tools/p4_capture_position.py \
      --placements-file tools/pos_placements.json --label pos1 --target-views 4
"""
import argparse,hashlib,json,sys,time,urllib.request,urllib.error
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import to_raw
C='http://192.168.1.171:8080';B='http://192.168.1.167:8100'
ROOT=Path('/home/brian/robot-arm-sim');CAPDIR=ROOT/'live/captures'
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
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
    return ((ord(sq[0].lower())-97)+0.5)*28-112,(int(sq[1])-0.5)*28-112
def cur_joints():
    j=get(B+'/state')['joints']
    return {k:j[k] for k in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')}
def measure():
    for k in range(6):
        try:f=grab_frame(timeout=6)
        except Exception:time.sleep(0.3);continue
        raw=to_raw(f);sol,d=b.solve(b.rectify(raw))
        if sol is None or len(d)<2 or sol['rms_px']>8:continue
        return np.array(sol['camera_in_board_mm'],float),[int(x.marker_id) for x in d],round(sol['rms_px'],2)
    return None
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--placements-file',default='tools/pos_placements.json')
    ap.add_argument('--label',default='pos1');ap.add_argument('--target-views',type=int,default=4)
    ap.add_argument('--min-sep',type=float,default=6.0);ap.add_argument('--z',type=float,default=60)
    a=ap.parse_args()
    plist=json.load(open(ROOT/a.placements_file))['placements']
    outdir=ROOT/'.private/real_piece_views'/a.label;outdir.mkdir(parents=True,exist_ok=True)
    post('/mode',{'mode':'REMOTE'});time.sleep(1.2)
    assert get(B+'/state')['mode']=='REMOTE'
    mf={'schema':'real-piece-views-position/1','label':a.label,'z_mm':a.z,
        'camera_controls':{c['name']:c['value'] for c in get(C+'/camera/controls')['controls']},
        'placements':[]}
    total=0
    for pl in plist:
        piece,sq=pl['piece'],pl['square'];x,y=sq_xy(sq)
        print(f'\n=== {piece} @ {sq} target ({x},{y}) ===',flush=True)
        # position tip over square
        body={'mode':'xyz','x_mm':x,'y_mm':y,'z_mm':a.z,'jaw':100}
        dr=post('/move',{**body,'dry_run':True},t=25)
        print('  position dry',dr.get('detail'),dr.get('requested',{}).get('position_error_mm'),flush=True)
        if dr.get('detail'):continue
        post('/move',body,t=60);time.sleep(2.0)
        base_j=cur_joints()
        kept=[]
        # small joint offsets for angle diversity (roll fixed)
        offsets=[('p0',{}),('lift_m3',{'shoulder_lift':-3}),('lift_p3',{'shoulder_lift':3}),
                 ('pan_m3',{'shoulder_pan':-3}),('pan_p3',{'shoulder_pan':3}),
                 ('wflex_m4',{'wrist_flex':-4}),('wflex_p4',{'wrist_flex':4}),
                 ('elbow_m3',{'elbow_flex':-3}),
                 ('roll_m8',{'wrist_roll':-8}),('roll_p8',{'wrist_roll':8}),
                 ('lift_m5_pan_p4',{'shoulder_lift':-5,'shoulder_pan':4}),
                 ('lift_p5_pan_m4',{'shoulder_lift':5,'shoulder_pan':-4})]
        entry={'piece':piece,'square':sq,'square_xy_mm':[x,y],'views':[]}
        for vn,off in offsets:
            if len(kept)>=a.target_views:break
            jd=dict(base_j)
            for k,v in off.items():jd[k]=base_j[k]+v
            dr=post('/move',{**{'mode':'joint','joints':jd,'jaw':100},'dry_run':True},t=25)
            md=dr.get('max_joint_delta_deg')
            if dr.get('detail') or md is None or md>20:continue
            post('/move',{'mode':'joint','joints':jd,'jaw':100});time.sleep(2.0)
            m=measure()
            if not m:print(f'  {vn}: no PnP',flush=True);continue
            cam,ids,rms=m
            seps=[float(np.linalg.norm(cam-k[0])) for k in kept]
            sep=min(seps) if seps else 999
            if sep<a.min_sep:print(f'  {vn}: sep {sep:.1f} skip',flush=True);continue
            ci=post('/capture-view',{'name':'pos'},t=30)
            if not ci.get('name'):print(f'  {vn}: capture fail',flush=True);continue
            img=cv2.imread(str(CAPDIR/f'{ci["name"]}.jpg'))
            h,w=img.shape[:2]
            if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
            if (img.shape[1],img.shape[0])!=(640,480):img=cv2.resize(img,(640,480))
            name=f'{piece}_{sq}_{a.label}_{vn}.jpg'
            cv2.imwrite(str(outdir/name),img)
            sha=hashlib.sha256((outdir/name).read_bytes()).hexdigest()
            p=b.project(b.solve(b.rectify(img))[0],np.array([x,y,0.]),0.)
            rec={'view':vn,'image':name,'sha256':sha,'cam':cam.round(2).tolist(),
                 'ids':ids,'rms':rms,'min_sep':round(sep,1),'target_px':[round(float(p[0]),0),round(float(p[1]),0)],
                 'capture_meta':ci}
            entry['views'].append(rec);kept.append((cam,rec));total+=1
            print(f'  {vn}: KEPT sep {sep:.1f} target_px {rec["target_px"]} rms {rms}',flush=True)
        mf['placements'].append(entry)
        print(f'  -> {len(kept)} views',flush=True)
    post('/mode',{'mode':'HOLD'})
    (outdir/'manifest.json').write_text(json.dumps(mf,indent=2)+'\n')
    (outdir/'labels_human.json').write_text('{}\n')
    print(f'\nDONE {total} views -> {outdir}; HOLD',flush=True)
if __name__=='__main__':main()
