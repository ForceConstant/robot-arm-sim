#!/usr/bin/env python3
"""Leakage-free base-XY model for type-known grasping (2026-11-23).

The earlier 0.6mm result was INVALID: features included the click-derived
offset (the target). Here features come ONLY from the image:
  - rectify frame to top-down board view (fixed px/mm)
  - crop a patch centred on the KNOWN SQUARE centre (from context/move)
  - residual vs local board background -> a spatial "piece signal" map
  - features = spatial thumbnail + shape/stats. NO click/target input.
Target (for training/eval only) = human-clicked base xy at z=0.

Eval: leave-one-view-out, type-known (nearest neighbour restricted to true
type). This should be HONEST: no target leakage. Also saves a model artifact
and an inference function predict_base_xy(frame, piece, square) -> board xy.
"""
import sys,json,hashlib
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board

ROOT=Path('/home/brian/robot-arm-sim')
DATA=ROOT/'.private/real_piece_views'
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
# The six placements the dataset was captured on. f4 is added 2026-11-23 for a
# live run Brian asked for: the crop is square-relative, so the machinery works,
# but no training placements sit on f4 - treat that prediction as
# out-of-distribution, not a validated result.
SQXY={'e5':(14,14),'d4':(-14,-14),'f6':(42,42),'c3':(-42,-42),'c5':(-42,14),'e3':(14,-42),
      'f4':(42,-14)}
PPMM=8.0; NPX=64

def image_features(frame_bgr, square):
    """Leakage-free feature vector from a raw frame + known square. No clicks."""
    h,w=frame_bgr.shape[:2]
    if h>w:frame_bgr=cv2.rotate(frame_bgr,cv2.ROTATE_90_COUNTERCLOCKWISE)
    rect=b.rectify(frame_bgr);sol,d=b.solve(rect)
    if sol is None:return None,None
    sx,sy=SQXY[square]
    top,_,n,_=b.rectified(rect,sol,PPMM)
    cx=int(n/2+sx*PPMM);cy=int(n/2+sy*PPMM)
    x0,y0=cx-NPX//2,cy-NPX//2
    rs=cv2.copyMakeBorder(top,max(0,-y0),max(0,y0+NPX-n),max(0,-x0),max(0,x0+NPX-n),cv2.BORDER_REFLECT)
    patch=rs[y0+max(0,-y0):y0+max(0,-y0)+NPX, x0+max(0,-x0):x0+max(0,-x0)+NPX].astype(np.float32)
    if patch.shape[:2]!=(NPX,NPX):patch=cv2.resize(patch,(NPX,NPX))
    ring=np.concatenate([patch[:6].reshape(-1,3),patch[-6:].reshape(-1,3),
                         patch[:,:6].reshape(-1,3),patch[:,-6:].reshape(-1,3)])
    bg=np.median(ring,axis=0)
    resid=patch-bg
    g=np.clip(np.linalg.norm(resid,axis=2),0,255)
    # spatial thumbnail (12x12) of piece signal - encodes WHERE the piece is
    thumb=cv2.resize(g,(12,12),interpolation=cv2.INTER_AREA).ravel()/255.0
    # also a centroid-like descriptor: weighted centre of mass of the residual
    ys,xs=np.mgrid[0:NPX,0:NPX]
    wgt=np.clip(g-g.mean(),0,None)
    tot=wgt.sum()+1e-6
    comx=(wgt*xs).sum()/tot/NPX; comy=(wgt*ys).sum()/tot/NPX
    feats=np.concatenate([thumb,[comx,comy,g.mean()/255,g.std()/255]]).astype(np.float32)
    return feats,sol

def load_rows(ds):
    mf=json.load(open(DATA/ds/'manifest.json'));lab=json.load(open(DATA/ds/'labels_human.json'))
    rows=[]
    for pl in mf['placements']:
        piece,sq=pl['piece'],pl['square']
        for v in pl['views']:
            k=v['image']
            if k not in lab:continue
            img=cv2.imread(str(DATA/ds/k))
            if img is None:continue
            f,sol=image_features(img,sq)
            if f is None:continue
            base0=b.unproject(sol,lab[k],0.0)
            rows.append({'key':k,'piece':piece,'square':sq,'feat':f,'base0':np.array(base0)})
    return rows

def main():
    rows=load_rows('merged1')
    X=np.array([r['feat'] for r in rows]);Y=np.array([r['base0'] for r in rows])
    types=[r['piece'] for r in rows]
    print(f'rows {len(rows)} featdim {X.shape[1]} (NO click target in features)',flush=True)
    errs=[]
    for i in range(len(rows)):
        same=[j for j in range(len(rows)) if j!=i and types[j]==types[i]]
        if not same:continue
        d=np.linalg.norm(X[same]-X[i],axis=1);j=same[d.argmin()]
        errs.append(float(np.linalg.norm(Y[i]-Y[j])))
    e=np.array(errs)
    print(f'LEAKAGE-FREE type-known base-XY (LOOV): median {np.median(e):.2f} mean {e.mean():.2f} '
          f'p90 {np.percentile(e,90):.2f} max {e.max():.2f}',flush=True)
    print(f'  within 3mm {(e<=3).sum()}/{len(e)}  within 5mm {(e<=5).sum()}/{len(e)}',flush=True)
    # save model (store features+labels per type; inference = NN)
    model={'schema':'base-xy-model/1','features':'topdown_square_centred_residual_v1',
           'PPMM':PPMM,'NPX':NPX,'note':'NO target leakage; features from image only',
           'samples':[{'piece':r['piece'],'square':r['square'],'feat':r['feat'].tolist(),
                       'base_xy':r['base0'].tolist()} for r in rows]}
    p=ROOT/'live/base_xy_model.json';p.write_text(json.dumps(model,indent=1)+'\n')
    print(f'model saved {p} sha256 {hashlib.sha256(p.read_bytes()).hexdigest()[:12]}',flush=True)
    json.dump({'n':len(e),'median':round(float(np.median(e)),2),'mean':round(float(e.mean()),2),
               'within3':int((e<=3).sum()),'within5':int((e<=5).sum())},
              open(str(ROOT/'reports/base_xy_leakagefree_eval.json'),'w'),indent=2)
if __name__=='__main__':main()
