#!/usr/bin/env python3
"""Position-invariant real-piece type classifier (2026-11-23).

Fix for the placement confound: previous patch features encoded the board square
background (parity/colour) and absolute position, so a model trained on pieces at
one board position failed when the same piece sat elsewhere. Here each patch is
normalised against its LOCAL board background (border median), leaving only the
piece's residual signal; features are shape/brightness descriptors of that
residual, independent of where on the board the piece sits.

Evaluates merged (37), round1-only, round2-only with leave-one-view-out.
"""
import sys,json
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from train_real_piece_model import TYPES

ROOT=Path('/home/brian/robot-arm-sim/.private/real_piece_views')
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
H={'pawn':23.0,'rook':26.0,'knight':32.0,'bishop':36.5,'queen':40.5,'king':46.5}

def inv_features(rect,px,py,patch_px=56):
    """Piece-vs-local-background residual features (position invariant)."""
    pad=patch_px//2
    rs=cv2.copyMakeBorder(rect,pad,pad,pad,pad,cv2.BORDER_REFLECT)
    x=int(px)+pad;y=int(py)+pad
    patch=rs[y-pad:y+pad,x-pad:x+pad].astype(np.float32)
    # local board background = median of an outer ring (avoid the piece centre)
    ring=np.concatenate([patch[:8].reshape(-1,3),patch[-8:].reshape(-1,3),
                         patch[:,:8].reshape(-1,3),patch[:,-8:].reshape(-1,3)])
    bg=np.median(ring,axis=0)
    resid=patch-bg
    gres=np.linalg.norm(resid,axis=2)           # piece signal magnitude
    # threshold residual to a soft mask, then shape descriptors
    mask=(gres>np.percentile(gres,60)).astype(np.uint8)
    n,lab,stats,cents=cv2.connectedComponentsWithStats(mask,8)
    # largest central component
    best=0;area=0
    for i in range(1,n):
        if stats[i,cv2.CC_STAT_AREA]>area:
            area=stats[i,cv2.CC_STAT_AREA];best=i
    m=(lab==best).astype(np.uint8)
    feats=[]
    # residual magnitude stats
    feats += [gres.mean()/255, gres.std()/255, float(gres.max())/255]
    # silhouette descriptors (scale-ish invariant)
    feats += [area/(patch_px*patch_px)]
    if area>0:
        ys,xs=np.where(m)
        hgt=ys.max()-ys.min()+1;wid=xs.max()-xs.min()+1
        feats += [hgt/max(wid,1), area/max(hgt*wid,1)]   # aspect, fill
        # radial profile: fraction of mask in inner vs outer rings
        cy,cx=patch_px/2,patch_px/2
        r=np.hypot(ys-cy,xs-cx)
        feats += [(r<patch_px*0.2).mean(),(r>patch_px*0.35).mean()]
    else:
        feats += [1.0,0.0,0.0,0.0]
    # colour direction of residual (wood tint) - mean RGB residual normalised
    mn=resid.reshape(-1,3).mean(0)
    feats += [float(v)/(np.linalg.norm(mn)+1e-6) for v in mn]
    return np.array(feats,np.float32)

def load_rows(ds):
    ddir=ROOT/ds
    mf=json.load(open(ddir/'manifest.json'));lab=json.load(open(ddir/'labels_human.json'))
    rows=[]
    for pl in mf['placements']:
        piece,sq=pl['piece'],pl['square']
        for v in pl['views']:
            k=v['image']
            if k not in lab:continue
            img=cv2.imread(str(ddir/k))
            h,w=img.shape[:2]
            if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
            rect=b.rectify(img);sol,d=b.solve(rect)
            if sol is None:continue
            f=inv_features(rect,lab[k][0],lab[k][1])
            base=b.unproject(sol,lab[k],H[piece])
            rows.append({'key':k,'piece':piece,'view':v['view'],'feat':f,'base':base})
    return rows

def predict(train,test):
    X=np.array([r['feat'] for r in train]);y=[r['piece'] for r in train]
    mean=X.mean(0);std=np.where(X.std(0)<1e-6,1.0,X.std(0))
    Xn=(X-mean)/std
    cents={t:Xn[[i for i,yy in enumerate(y) if yy==t]].mean(0) for t in TYPES if t in y}
    ok=0
    for r in test:
        x=(r['feat']-mean)/std
        p=min(cents,key=lambda t:np.linalg.norm(x-cents[t]))
        ok+= (p==r['piece'])
    return ok,len(test)

def acc(rows,label):
    ok=0
    for i in range(len(rows)):
        tr=[r for j,r in enumerate(rows) if j!=i]
        o,_=predict(tr,[rows[i]]);ok+=o
    print(f'{label}: {ok}/{len(rows)} = {ok/len(rows):.3f}',flush=True)
    return ok/len(rows)

def main():
    rows=load_rows('merged1')
    print(f'invariant features loaded: {len(rows)} rows, dim {rows[0]["feat"].shape[0]}',flush=True)
    r1=[r for r in rows if 'round2' not in r['key']]
    r2=[r for r in rows if 'round2' in r['key']]
    results={'merged':acc(rows,'merged'),'round1':acc(r1,'round1'),'round2':acc(r2,'round2')}
    json.dump(results,open('/home/brian/robot-arm-sim/reports/invariant_eval.json','w'),indent=2)
    print('report written',flush=True)
if __name__=='__main__':main()
