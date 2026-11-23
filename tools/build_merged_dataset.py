#!/usr/bin/env python3
"""Merge all human-labeled views into one clean dataset (2026-11-23).

Sources:
  adapt1  - pawn/rook/knight/queen/king clicks (drop the bad rook pan_m4)
  bishop_pos_label - 4 bishop clicks (0.5mm spread)
  king_pos_label   - 3 king clicks (0.9mm spread)

Copies images, writes a merged manifest + flat labels_human.json.
"""
import json,shutil
from pathlib import Path
R=Path('/home/brian/robot-arm-sim/.private/real_piece_views')
OUT=R/'merged1';OUT.mkdir(parents=True,exist_ok=True)

def sq_xy(sq):
    return ((ord(sq[0].lower())-97)+0.5)*28-112,(int(sq[1])-0.5)*28-112

# source datasets: (dir, piece)
sources=[('adapt1',None),('bishop_pos_label',None),('king_pos_label',None),
         ('round2',None),('round3',None)]  # all human-labeled rounds
# bad clicks -> drop (rook pan_m4 outlier, old king adapt1 click)
DROP={'rook_d4_adapt1_pan_m4.jpg','king_e3_adapt1_king_e3_adapt1_lift_m12.jpg'}

outmf={'schema':'real-piece-views-merged/1','label':'merged1','placements':[]}
outlabels={}
seen=set()
for ds,_ in sources:
    d=R/ds
    if not (d/'manifest.json').exists():continue
    mf=json.load(open(d/'manifest.json'))
    lab=json.load(open(d/'labels_human.json'))
    for pl in mf['placements']:
        piece,sq=pl['piece'],pl['square']
        # find or create placement in output
        plout=next((p for p in outmf['placements'] if p['piece']==piece),None)
        if plout is None:
            plout={'piece':piece,'square':sq,'square_xy_mm':list(sq_xy(sq)),'views':[]}
            outmf['placements'].append(plout)
        for v in pl['views']:
            oldname=v['image']
            if oldname not in lab or oldname in DROP:continue
            newname=f'{piece}_{sq}_{ds}_{oldname}'
            if newname in seen:continue
            seen.add(newname)
            shutil.copy(d/oldname,OUT/newname)
            rec={k:vv for k,vv in v.items() if k!='image'}
            rec['image']=newname;rec['source']=ds
            plout['views'].append(rec)
            outlabels[newname]=lab[oldname]

(OUT/'manifest.json').write_text(json.dumps(outmf,indent=2)+'\n')
(OUT/'labels_human.json').write_text(json.dumps(outlabels,indent=2)+'\n')
print('merged:',sum(len(p['views']) for p in outmf['placements']),'views,',len(outlabels),'labels')
for p in outmf['placements']:
    print(f"  {p['piece']:7s} {p['square']}: {len(p['views'])} views")
