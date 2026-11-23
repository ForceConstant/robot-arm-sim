#!/usr/bin/env python3
"""Honest generalization eval: group cross-validation on the real-piece data.

Random leave-one-view-out leaks near-duplicate views (same pose, tiny offsets)
across train/test, inflating accuracy. Here we hold out GROUPS:
  - by capture session (round1 vs round2) - different tone/appearance
  - by (piece, square) placement - though each piece is in one square, this
    tests type generalization across appearance groups instead.
Reports accuracy under grouped splits vs random LOOV, for the MLP on
square-relative features. Honest estimate of generalization.
"""
import sys,json
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from train_square_relative import load_rows
from train_real_piece_model import TYPES
from train_mlp_real import train_mlp

rows=load_rows('merged1')
X=np.array([r['feat'] for r in rows]);y=[r['piece'] for r in rows]
def group_of(r):
    return 'round2' if 'round2' in r['key'] else 'round1'
groups=sorted(set(group_of(r) for r in rows))
print(f'{len(rows)} rows, groups: {groups}',flush=True)

# 1) leave-one-GROUP-out
print('\nLeave-one-SESSION-out (train on one session, test on the other):',flush=True)
tot_ok=tot=0
for g in groups:
    tr=[r for r in rows if group_of(r)!=g];te=[r for r in rows if group_of(r)==g]
    # may miss classes in train; check
    missing=set(r['piece'] for r in te)-set(r['piece'] for r in tr)
    if missing:
        print(f'  hold {g}: SKIP - train missing classes {missing}',flush=True);continue
    Xtr=np.array([r['feat'] for r in tr]);ytr=[r['piece'] for r in tr]
    pred=train_mlp(Xtr,ytr,np.array([r['feat'] for r in te]),seed=1)
    ok=sum(TYPES[p]==r['piece'] for p,r in zip(pred,te))
    print(f'  train {[x for x in groups if x!=g]} -> hold {g}: {ok}/{len(te)} = {ok/len(te):.3f}',flush=True)
    tot_ok+=ok;tot+=len(te)
print(f'  overall grouped: {tot_ok}/{tot} = {tot_ok/tot:.3f}' if tot else '  n/a',flush=True)

# 2) random LOOV for comparison
ok=0
for i in range(len(rows)):
    tr=[r for j,r in enumerate(rows) if j!=i]
    if len(set(r['piece'] for r in tr))<len(TYPES):continue
    p=TYPES[train_mlp(np.array([r['feat'] for r in tr]),[r['piece'] for r in tr],X[i:i+1],seed=i)[0]]
    ok+=(p==rows[i]['piece'])
print(f'\nRandom leave-one-view-out (for comparison): {ok}/{len(rows)} = {ok/len(rows):.3f}',flush=True)
json.dump({'n':len(rows),'random_loov':round(ok/len(rows),4),'grouped_sessions':'see log'},
          open('/home/brian/robot-arm-sim/reports/grouped_eval.json','w'),indent=2)
print('report written',flush=True)
