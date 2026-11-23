#!/usr/bin/env python3
"""Base-XY regression + abstention eval on the merged human labels (2026-11-23).

Reuses features/loading from train_real_piece_model.py. For each labeled view:
  - target = board XY obtained from the human click unprojected at piece height
  - leave-one-view-out nearest-neighbour regression in feature space; report
    XY error (Euclidean) vs a ~3mm bar
  - type classifier confidence from nearest-class-mean; abstain below threshold;
    report coverage and precision on the covered set (safety-oriented)
All honest - seed data (19 views), NOT a qualified model.
"""
import sys,json
from collections import Counter
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from train_real_piece_model import load_rows,nearest_centroid_predict,TYPES

rows=load_rows('merged1','labels_human.json')
print(f'rows {len(rows)}',flush=True)
X=np.array([r['feat'] for r in rows])
Y=np.array([r['base_xy'] for r in rows])
mean=X.mean(0);std=np.where(X.std(0)<1e-6,1.0,X.std(0));Xn=(X-mean)/std

# ---- base-XY regression (leave-one-view-out, nearest neighbour in features)
errs=[]
for i in range(len(rows)):
    d=np.linalg.norm(Xn-Xn[i],axis=1);d[i]=1e9
    j=int(d.argmin())
    err=float(np.linalg.norm(Y[i]-Y[j]))
    errs.append(err)
errs=np.array(errs)
print('\nBASE-XY leave-one-view-out (nearest neighbour):')
print(f'  mean {errs.mean():.1f}mm median {np.median(errs):.1f}mm '
      f'p90 {np.percentile(errs,90):.1f}mm max {errs.max():.1f}mm')
print(f'  within 3mm: {int((errs<=3).sum())}/{len(errs)}  within 5mm: {int((errs<=5).sum())}/{len(errs)}')

# ---- type classifier + abstention
# confidence = margin between top-2 class-mean distances normalised
confs=[];corr=[]
for i in range(len(rows)):
    tr=[r for j,r in enumerate(rows) if j!=i];te=[rows[i]]
    # distances to class means built from tr
    Xtr=np.array([r['feat'] for r in tr]); m2=Xtr.mean(0);s2=np.where(Xtr.std(0)<1e-6,1.0,Xtr.std(0))
    trn=(Xtr-m2)/s2; xn=(rows[i]['feat']-m2)/s2
    cents={t:trn[[k for k,r in enumerate(tr) if r['piece']==t]].mean(0) for t in TYPES if any(r['piece']==t for r in tr)}
    ds=sorted((float(np.linalg.norm(xn-c)),t) for t,c in cents.items())
    pred=ds[0][1]; margin=ds[1][0]-ds[0][0] if len(ds)>1 else 0.0
    confs.append(margin);corr.append(pred==rows[i]['piece'])
confs=np.array(confs);corr=np.array(corr)
print('\nTYPE abstention by confidence margin:')
for thr in (0.0,0.2,0.5,1.0,2.0):
    m=confs>=thr
    cov=m.mean()
    prec=corr[m].mean() if m.any() else float('nan')
    print(f'  margin>={thr:4.1f}: coverage {cov:.2f}  precision {prec:.2f}')

receipt={'schema':'xy-abstain-eval/1','n':len(rows),
         'xy_err_mean_mm':round(float(errs.mean()),2),
         'xy_err_median_mm':round(float(np.median(errs)),2),
         'xy_within_3mm':int((errs<=3).sum()),'xy_within_5mm':int((errs<=5).sum()),
         'abstain':[{'margin':t,'coverage':round(float((confs>=t).mean()),3),
                     'precision':round(float(corr[confs>=t].mean()),3) if (confs>=t).any() else None}
                    for t in (0.0,0.2,0.5,1.0,2.0)]}
Path('/home/brian/robot-arm-sim/reports/xy_abstain_eval_merged1.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('\nwrote receipt',flush=True)
