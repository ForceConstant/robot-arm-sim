#!/usr/bin/env python3
"""Square-relative real-piece classifier (2026-11-23, Brian's correction).

Fix: pieces never moved; my earlier 'placement confound' was an artifact of
unprojecting base clicks at piece HEIGHT (parallax). A base click is a z=0 point.
Here:
  - base board xy is unprojected at z=0 -> consistent across sessions
  - features are SQUARE-RELATIVE: in a board-rectified top-down image, crop a
    patch centred on the piece's actual square centre (not absolute image pos),
    so every piece/square is seen in the same local frame.
Evaluates merged 37 with a placement-aware split and leave-one-view-out.
"""
import sys,json
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from train_real_piece_model import TYPES

ROOT=Path('/home/brian/robot-arm-sim/.private/real_piece_views')
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
SQXY={'e5':(14,14),'d4':(-14,-14),'f6':(42,42),'c3':(-42,-42),'c5':(-42,14),'e3':(14,-42)}
def load_rows(ds):
    ddir=ROOT/ds
    mf=json.load(open(ddir/'manifest.json'));lab=json.load(open(ddir/'labels_human.json'))
    rows=[]
    for pl in mf['placements']:
        piece=pl['piece'];sq=pl['square'];sx,sy=SQXY[sq]
        for v in pl['views']:
            k=v['image']
            if k not in lab:continue
            img=cv2.imread(str(ddir/k));h,w=img.shape[:2]
            if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
            rect=b.rectify(img);sol,d=b.solve(rect)
            if sol is None:continue
            base0=b.unproject(sol,lab[k],0.0)  # z=0 base click
            # board-rectified top-down patch around the SQUARE centre
            ppmm=8.0;n=int(round(256*ppmm))
            top,_,_,_=b.rectified(rect,sol,ppmm)
            npx=64
            sx_px=int(n/2+sx*ppmm);sy_px=int(n/2+sy*ppmm)
            x0,y0=sx_px-npx//2,sy_px-npx//2
            rs=cv2.copyMakeBorder(top,max(0,-y0),max(0,y0+npx-n),max(0,-x0),max(0,x0+npx-n),cv2.BORDER_REFLECT)
            ox=x0+max(0,-x0);oy=y0+max(0,-y0)
            patch=rs[oy:oy+npx,ox:ox+npx].astype(np.float32)
            if patch.shape[:2]!=(npx,npx):patch=cv2.resize(patch,(npx,npx))
            grey=cv2.cvtColor(patch.astype(np.uint8),cv2.COLOR_BGR2GRAY).astype(np.float32)
            ring=np.concatenate([patch[:6].reshape(-1,3),patch[-6:].reshape(-1,3),
                                 patch[:,:6].reshape(-1,3),patch[:,-6:].reshape(-1,3)])
            # ILLUMINATION NORMALIZATION: gray-world on the outer board ring
            board_mean=ring.mean(0)+1e-6
            patch=patch*(board_mean.mean()/board_mean)  # per-channel gain
            bg=np.median(np.concatenate([patch[:6].reshape(-1,3),patch[-6:].reshape(-1,3),
                                         patch[:,:6].reshape(-1,3),patch[:,-6:].reshape(-1,3)]),axis=0)
            resid=patch-bg
            g=np.linalg.norm(resid,axis=2)
            # square-relative click offset (where on the square the piece base is)
            off=[(base0[0]-sx)/28.0,(base0[1]-sy)/28.0]
            thumb=cv2.resize(np.clip(g,0,255),(8,8),interpolation=cv2.INTER_AREA).ravel()/255
            mn=resid.reshape(-1,3).mean(0)
            f=list(thumb)+[g.mean()/255,g.std()/255]+off+[float(v)/(np.linalg.norm(mn)+1e-6) for v in mn]
            rows.append({'key':k,'piece':piece,'square':sq,'view':v['view'],
                         'feat':np.array(f,np.float32),'base0':base0})
    return rows

def acc(rows,label):
    X=np.array([r['feat'] for r in rows]);y=[r['piece'] for r in rows]
    mean=X.mean(0);std=np.where(X.std(0)<1e-6,1.0,X.std(0));Xn=(X-mean)/std
    ok=0
    for i in range(len(rows)):
        tr=np.array([j for j in range(len(rows)) if j!=i])
        cents={t:Xn[tr[[k for k,j in enumerate(tr) if y[j]==t]]].mean(0) for t in TYPES if any(y[j]==t for j in tr)}
        p=min(cents,key=lambda t:np.linalg.norm(Xn[i]-cents[t]))
        ok+=(p==rows[i]['piece'])
    print(f'{label}: {ok}/{len(rows)} = {ok/len(rows):.3f}',flush=True)
    return ok/len(rows)

def main():
    rows=load_rows('merged1');print(f'square-relative rows {len(rows)} dim {rows[0]["feat"].shape[0]}',flush=True)
    r1=[r for r in rows if 'round2' not in r['key']];r2=[r for r in rows if 'round2' in r['key']]
    res={'merged':acc(rows,'merged'),'round1':acc(r1,'round1'),'round2':acc(r2,'round2')}
    json.dump(res,open('/home/brian/robot-arm-sim/reports/colornorm_eval.json','w'),indent=2)
    print('report written',flush=True)
if __name__=='__main__':main()
