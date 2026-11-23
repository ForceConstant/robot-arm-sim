#!/usr/bin/env python3
"""Base-XY regression on square-relative features (2026-11-23).

Uses train_square_relative.load_rows (z=0 labels, square-centred top-down
patches). Target = board XY (z=0). Two regressors, leave-one-view-out:
  (1) nearest neighbour in feature space (same-type conditioned via MLP pred)
  (2) linear ridge from features to board XY
Reports Euclidean error vs 3mm / 5mm bars, and within-square offset error.
"""
import sys,json
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from train_square_relative import load_rows,SQXY
from train_real_piece_model import TYPES
from train_mlp_real import train_mlp

rows=load_rows('merged1')
X=np.array([r['feat'] for r in rows])
Y=np.array([r['base0'] for r in rows])   # z=0 board xy
print(f'rows {len(rows)} dim {X.shape[1]}',flush=True)

def ridge(train_idx,test_x,lam=1.0):
    Xt=X[train_idx];Yt=Y[train_idx]
    m=Xt.mean(0);s=np.where(Xt.std(0)<1e-6,1,Xt.std(0))
    A=(Xt-m)/s;at=(test_x-m)/s
    A1=np.hstack([A,np.ones((len(A),1))])
    W=np.linalg.solve(A1.T@A1+lam*np.eye(A1.shape[1]),A1.T@Yt)
    return np.r_[at,1.0]@W

errs_nn=[];errs_ridge=[];errs_off=[]
for i in range(len(rows)):
    tr=np.array([j for j in range(len(rows)) if j!=i])
    # nearest neighbour
    Xt=X[tr];m=Xt.mean(0);s=np.where(Xt.std(0)<1e-6,1,Xt.std(0))
    d=np.linalg.norm((X[i]-m)/s-(Xt-m)/s,axis=1);j=tr[d.argmin()]
    errs_nn.append(float(np.linalg.norm(Y[i]-Y[j])))
    # ridge
    yp=ridge(tr,X[i]);errs_ridge.append(float(np.linalg.norm(Y[i]-yp)))
    # within-square offset error (target minus square centre)
    sq=rows[i]['square'];sx,sy=SQXY[sq]
    off_true=Y[i]-np.array([sx,sy]);off_pred=yp-np.array([sx,sy])
    errs_off.append(float(np.linalg.norm(off_true-off_pred)))

for name,e in [('nearest-neighbour',errs_nn),('ridge',errs_ridge)]:
    e=np.array(e)
    print(f'{name}: mean {e.mean():.1f} median {np.median(e):.1f} p90 {np.percentile(e,90):.1f} max {e.max():.1f}',
          flush=True)
    print(f'  within3mm {(e<=3).sum()}/{len(e)}  within5mm {(e<=5).sum()}/{len(e)}',flush=True)
e=np.array(errs_off)
print(f'ridge within-square offset: mean {e.mean():.1f} median {np.median(e):.1f}',flush=True)
receipt={'n':len(rows),
 'nn_median':round(float(np.median(errs_nn)),2),'ridge_median':round(float(np.median(errs_ridge)),2),
 'ridge_mean':round(float(np.mean(errs_ridge)),2),
 'ridge_within3':int((np.array(errs_ridge)<=3).sum()),'ridge_within5':int((np.array(errs_ridge)<=5).sum()),
 'ridge_offset_median':round(float(np.median(errs_off)),2)}
json.dump(receipt,open('/home/brian/robot-arm-sim/reports/xy_regression_sqrel.json','w'),indent=2)
print('report written',flush=True)
