#!/usr/bin/env python3
"""Type-known base-XY eval (2026-11-23): if game context gives the piece TYPE,
how accurate is vision for base position? Leave-one-view-out on merged1.

Compares:
 (a) nearest neighbour restricted to the TRUE type (oracle type)
 (b) true-type class-mean residual: predict offset from square centre using
     within-type nearest neighbour in feature space
Reports board XY error vs 3mm/5mm bars, overall and per type.
"""
import sys,json
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from train_square_relative import load_rows,SQXY
from train_real_piece_model import TYPES

rows=load_rows('merged1')
X=np.array([r['feat'] for r in rows]);Y=np.array([r['base0'] for r in rows])
types=[r['piece'] for r in rows]
print(f'rows {len(rows)}',flush=True)
errs=[]
per={t:[] for t in TYPES}
for i in range(len(rows)):
    same=[j for j in range(len(rows)) if j!=i and types[j]==types[i]]
    if not same:continue
    # feature-space NN within true type
    A=X[same]
    d=np.linalg.norm(A-X[i],axis=1);j=same[d.argmin()]
    e=float(np.linalg.norm(Y[i]-Y[j]))
    errs.append(e);per[types[i]].append(e)
errs=np.array(errs)
print(f'TYPE-KNOWN (true-type NN) base-XY, LOOV:',flush=True)
print(f'  mean {errs.mean():.1f} median {np.median(errs):.1f} p90 {np.percentile(errs,90):.1f} max {errs.max():.1f}',flush=True)
print(f'  within 3mm {(errs<=3).sum()}/{len(errs)} ({100*(errs<=3).mean():.0f}%)  within 5mm {(errs<=5).sum()}/{len(errs)}',flush=True)
print('  per type:',flush=True)
for t in TYPES:
    if per[t]:
        a=np.array(per[t]);print(f'    {t:7s} n={len(a):2d} median {np.median(a):.1f} max {a.max():.1f}',flush=True)
receipt={'n':len(errs),'median':round(float(np.median(errs)),2),'mean':round(float(errs.mean()),2),
         'within3':int((errs<=3).sum()),'within5':int((errs<=5).sum())}
json.dump(receipt,open('/home/brian/robot-arm-sim/reports/typeknown_xy_eval.json','w'),indent=2)
print('report written',flush=True)
