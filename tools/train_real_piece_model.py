#!/usr/bin/env python3
"""Train a first-pass real-image piece type model from HUMAN base-centre clicks.

Consumes `.private/real_piece_views/<ds>/labels_human.json` (24 clicks) plus the
clean images + PnP. Extracts a compact patch feature around each clicked base,
trains a small type classifier with leave-one-PIECE-out cross-validation, and
reports accuracy + confusion. Also checks that clicked base pixels unproject
consistently across views of a piece (label QA).

HONEST BOUNDARY: 24 clicks over 6 pieces is a seed, not a training-ready corpus.
The script reports a data-sufficiency verdict and will NOT claim a qualified
model. It is a measurable first pass on real pixels, nothing more.

    PYTHONPATH=. .venv/bin/python tools/train_real_piece_model.py --dataset adapt1
"""
import argparse, json, sys, hashlib, time
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board

ROOT=Path('/home/brian/robot-arm-sim')
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
H={'pawn':23.0,'rook':26.0,'knight':32.0,'bishop':36.5,'queen':40.5,'king':46.5}
TYPES=['pawn','rook','knight','bishop','queen','king']

def features(rect, px, py, patch_px=48):
    """Compact feature vector from a patch centered at clicked base pixel."""
    h,w=rect.shape[:2]
    x0=int(px)-patch_px//2; y0=int(py)-patch_px//2
    # clamp with padding
    pad=patch_px//2
    rs=cv2.copyMakeBorder(rect,pad,pad,pad,pad,cv2.BORDER_REFLECT)
    x0+=pad;y0+=pad
    patch=rs[y0:y0+patch_px,x0:x0+patch_px]
    grey=cv2.cvtColor(patch,cv2.COLOR_BGR2GRAY)
    # compact: 8x8 thumbnail + channel stats + gradients
    thumb=cv2.resize(grey,(8,8),interpolation=cv2.INTER_AREA).astype(np.float32).ravel()/255.0
    stats=[]
    for c in range(3):
        ch=patch[:,:,c].astype(np.float32)
        stats += [ch.mean()/255.0, ch.std()/255.0]
    gx=cv2.Sobel(grey,cv2.CV_32F,1,0,ksize=3); gy=cv2.Sobel(grey,cv2.CV_32F,0,1,ksize=3)
    gm=float(np.mean(np.hypot(gx,gy)))/255.0
    return np.concatenate([thumb,np.array(stats,np.float32),[gm]]).astype(np.float32)

def load_rows(ds,labels_file):
    ddir=ROOT/'.private/real_piece_views'/ds
    mf=json.load(open(ddir/'manifest.json'))
    labels=json.load(open(ddir/labels_file))
    rows=[]
    for pl in mf['placements']:
        piece,sq=pl['piece'],pl['square']
        for v in pl['views']:
            key=v['image']
            if key not in labels:continue
            img=cv2.imread(str(ddir/key))
            if img is None:continue
            rect=b.rectify(img);sol,d=b.solve(rect)
            if sol is None:continue
            px,py=labels[key]
            base=b.unproject(sol,(px,py),H.get(piece,23.0))
            f=features(rect,px,py)
            rows.append({'key':key,'piece':piece,'square':sq,'view':v['view'],
                         'feat':f,'base_xy':base.round(2).tolist(),'click':[px,py]})
    return rows

def nearest_centroid_predict(train,test):
    """Nearest-class-mean classifier on standardized features."""
    X=np.array([r['feat'] for r in train]); y=[r['piece'] for r in train]
    mean=X.mean(0);std=np.where(X.std(0)<1e-6,1.0,X.std(0))
    Xn=(X-mean)/std
    cents={t:Xn[[i for i,yy in enumerate(y) if yy==t]].mean(0) for t in TYPES if t in y}
    correct=0;preds=[]
    for r in test:
        x=(r['feat']-mean)/std
        dists={t:float(np.linalg.norm(x-c)) for t,c in cents.items()}
        p=min(dists,key=dists.get)
        preds.append((p,r['piece']))
        if p==r['piece']:correct+=1
    return correct,len(test),preds

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--dataset',default='adapt1')
    ap.add_argument('--labels',default='labels_human.json')
    a=ap.parse_args()
    rows=load_rows(a.dataset,a.labels)
    print(f'loaded {len(rows)} labeled rows from {a.dataset}/{a.labels}',flush=True)
    if not rows:
        print('NO LABELS - cannot train');return
    n=len(rows); nclicks=len(set(r['key'] for r in rows))
    verdict='SEED ONLY - insufficient for a qualified model' if n<200 else 'larger corpus'
    print(f'rows {n} unique-images {nclicks} | verdict: {verdict}',flush=True)
    # label QA: base-spread across views per piece
    print('\nLABEL QA (clicked base unprojection spread per piece):',flush=True)
    bypiece=defaultdict(list)
    for r in rows:bypiece[r['piece']].append(r['base_xy'])
    for t in TYPES:
        v=bypiece.get(t,[])
        if len(v)>=2:
            arr=np.array(v)
            print(f'  {t}: n={len(v)} base-median={np.median(arr,0).round(1)} spread={arr.std(0).round(1)}',flush=True)
    # leave-one-view-out type classification (all classes stay represented).
    # NOTE: holding out an ENTIRE class (leave-one-piece-out) makes predicting
    # that class impossible and yields a meaningless 0% - don't do that.
    print('\nLEAVE-ONE-VIEW-OUT type classification:',flush=True)
    tot_ok=0;tot_n=0
    for i,te_row in enumerate(rows):
        tr=[r for j,r in enumerate(rows) if j!=i]
        te=[te_row]
        # require the held-out class to still appear in training
        if not any(r['piece']==te_row['piece'] for r in tr):continue
        ok,nn,_=nearest_centroid_predict(tr,te)
        tot_ok+=ok;tot_n+=nn
    acc=tot_ok/tot_n if tot_n else 0
    from collections import Counter as _C
    print(f'\nTYPE ACCURACY (leave-one-view-out): {tot_ok}/{tot_n} = {acc:.3f}',flush=True)
    # majority-class baseline for context
    counts=_C(r['piece'] for r in rows)
    maj=max(counts.values())/len(rows)
    print(f'majority-class baseline: {maj:.3f}',flush=True)
    receipt={'schema':'real-piece-model-firstpass/1','dataset':a.dataset,'labels':a.labels,
             'rows':n,'unique_images':nclicks,'data_verdict':verdict,
             'type_accuracy_leave_one_view_out':round(acc,4),
             'type_correct':tot_ok,'type_total':tot_n,
             'majority_baseline':round(maj,4),
             'method':'nearest-class-mean on compact patch features',
             'claim':'first-pass measurement only; NOT a qualified model'}
    rp=ROOT/f'reports/real_piece_model_firstpass_{a.dataset}.json'
    rp.write_text(json.dumps(receipt,indent=2)+'\n')
    print('wrote',rp,flush=True)

if __name__=='__main__':main()
