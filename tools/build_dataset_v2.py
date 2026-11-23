#!/usr/bin/env python3
"""Build adapt2: remove skipped cases, add the new well-framed bishop views,
and pre-fill the valid human labels. Brian then labels only the new views.

Keeps only views that are in labels_human.json (valid clicks), plus the
newly captured well-framed bishop views. Writes a fresh manifest, copies the
images, and a labels_human.json with the old clicks pre-filled.
"""
import json, shutil
from pathlib import Path
ROOT=Path('/home/brian/robot-arm-sim')
BASE=ROOT/'.private/real_piece_views'
OUT=BASE/'adapt2'
OUT.mkdir(parents=True,exist_ok=True)

ad=BASE/'adapt1'
mf=json.load(open(ad/'manifest.json'))
labels=json.load(open(ad/'labels_human.json'))

# valid labeled views from adapt1, grouped by piece
keep={}
for pl in mf['placements']:
    piece=pl['piece']
    for v in pl['views']:
        if v['image'] in labels:
            keep.setdefault(piece,[]).append((ad/v['image'],v,piece,pl['square']))

# new well-framed bishop views (dedupe by pose/camera; pick distinct ones)
for ds in ('bishop_deep','bishop_extra','bishop_wb'):
    d=BASE/ds
    if not (d/'manifest.json').exists():continue
    m=json.load(open(d/'manifest.json'))
    for pl in m['placements']:
        for v in pl['views']:
            # only poses the framing scan scored as well-framed
            if v['view'] not in ('lift_m8','lift_m12','lift_m16','wflex_m6',
                                 'lift_m8_pan_m4','lift_m8_pan_p4'):continue
            cam=v.get('cam') or []
            # dedupe against existing bishop cameras
            existing=[x[1].get('cam') or [] for x in keep.get('bishop',[])]
            if any(cam and ec and sum((a-b)**2 for a,b in zip(cam,ec))<25 for ec in existing):
                continue
            keep.setdefault('bishop',[]).append((d/v['image'],v,'bishop',pl['square']))

outmf={'schema':'real-piece-views-v2','label':'adapt2',
       'note':'skipped cases removed; human-labeled views + new framed bishop views',
       'camera_controls':mf.get('camera_controls'),'intrinsics':mf.get('intrinsics'),
       'placements':[]}
outlabels={}
for piece,entries in keep.items():
    sq=entries[0][3]
    plout={'piece':piece,'square':sq,'square_xy_mm':entries[0][1].get('square_xy_mm'),'views':[]}
    for src,v,p,sq2 in entries:
        name=f'{piece}_{sq}_adapt2_{v["view"]}.jpg'
        shutil.copy(src,OUT/name)
        rec={k:vv for k,vv in v.items() if k!='image'}
        rec['image']=name
        plout['views'].append(rec)
        # prefill old label if this was a labeled view
        if src.parent==ad and v['image'] in labels:
            outlabels[name]=labels[v['image']]
    outmf['placements'].append(plout)

(OUT/'manifest.json').write_text(json.dumps(outmf,indent=2)+'\n')
(OUT/'labels_human.json').write_text(json.dumps(outlabels,indent=2)+'\n')
print('adapt2 built:',sum(len(p['views']) for p in outmf['placements']),'views,',
      len(outlabels),'prefilled labels')
for p in outmf['placements']:
    n=sum(1 for v in p['views'] if v['image'] in outlabels)
    print(f"  {p['piece']:7s} {p['square']}: {len(p['views'])} views ({n} labeled)")
