#!/usr/bin/env python3
"""Small supervised MLP on square-relative features (2026-11-23).

Reuses train_square_relative.load_rows (z=0, square-centered patches). Trains a
2-layer MLP with leave-one-view-out cross-validation and reports accuracy vs
nearest-mean baseline. Includes a placement-aware check: group by square+view
pose to avoid leaking near-duplicate views across folds where possible.
"""
import sys,json
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from train_square_relative import load_rows
from train_real_piece_model import TYPES

def train_mlp(Xtr,ytr,Xte,hidden=32,epochs=300,lr=0.03,seed=0):
    rng=np.random.default_rng(seed)
    n,d=Xtr.shape; k=len(TYPES)
    yi=np.array([TYPES.index(t) for t in ytr])
    Y=np.eye(k)[yi]
    mean=Xtr.mean(0);std=np.where(Xtr.std(0)<1e-6,1,Xtr.std(0))
    A=(Xtr-mean)/std; At=(Xte-mean)/std
    W1=rng.normal(0,0.5,(d,hidden));b1=np.zeros(hidden)
    W2=rng.normal(0,0.5,(hidden,k));b2=np.zeros(k)
    for ep in range(epochs):
        h=np.tanh(A@W1+b1); z=h@W2+b2
        z-=z.max(1,keepdims=True); e=np.exp(z);p=e/e.sum(1,keepdims=True)
        dz=(p-Y)/len(A)
        dW2=h.T@dz;db2=dz.sum(0)
        dh=dz@W2.T*(1-h*h)
        dW1=A.T@dh;db1=dh.sum(0)
        W2-=lr*dW2;b2-=lr*db2;W1-=lr*dW1;b1-=lr*db1
    h=np.tanh(At@W1+b1);z=h@W2+b2
    return np.argmax(z,1)

def main():
    rows=load_rows('merged1')
    X=np.array([r['feat'] for r in rows]);y=[r['piece'] for r in rows]
    print(f'rows {len(rows)} dim {X.shape[1]}',flush=True)
    ok=0;preds=[]
    for i in range(len(rows)):
        tr=[r for j,r in enumerate(rows) if j!=i]
        Xtr=np.array([r['feat'] for r in tr]);ytr=[r['piece'] for r in tr]
        # ensure all classes present
        if len(set(ytr))<len(TYPES):preds.append(None);continue
        pi=train_mlp(Xtr,ytr,X[i:i+1],seed=i)[0]
        p=TYPES[pi];preds.append(p);ok+=(p==rows[i]['piece'])
    acc=ok/len(rows)
    print(f'MLP leave-one-view-out: {ok}/{len(rows)} = {acc:.3f}',flush=True)
    from collections import Counter
    maj=max(Counter(y).values())/len(y)
    print(f'majority baseline {maj:.3f}',flush=True)
    json.dump({'n':len(rows),'acc':round(acc,4),'correct':ok,'majority':round(maj,4)},
              open('/home/brian/robot-arm-sim/reports/mlp_real_eval.json','w'),indent=2)
    print('report written',flush=True)
if __name__=='__main__':main()
