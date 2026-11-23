#!/usr/bin/env python3
"""Label piece base centres from the CLEAN captured views (2026-11-22).

For each placement/view in a dataset manifest:
  - load the clean raw frame, compute PnP pose
  - detect the piece silhouette near its target square
  - unproject the detected top centroid at the piece's KNOWN HEIGHT (so the
    result is the base xy, view-invariant, not the parallax-shifted z=0 point)
  - record base xy per view; check agreement across the views of a placement

Piece heights from .private/grasp-analysis.json (mm). Labels go into a
labels.json next to the manifest, usable by a training script.

This is a LABELING/measurement tool; it does not train anything.
"""
import argparse, json, sys
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board

ROOT=Path('/home/brian/robot-arm-sim')
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
# Piece heights (mm) from .private/grasp-analysis.json (recorded 2026-10-28).
# Embedded so this tool doesn't depend on syncing a private asset to BC-250.
H={'pawn':23.0,'rook':26.0,'knight':32.0,'bishop':36.5,'queen':40.5,'king':46.5}

def detect_top(img,sol,sq_xy):
    """Detect piece top blob near a target square; return pixel centroid."""
    rect=b.rectify(img)
    grey=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
    p=b.project(sol,np.array([sq_xy[0],sq_xy[1],0.]),0.)
    # try multiple thresholds; piece is darker than bright markers in these images
    best=None
    for thr in (110,130,150,170,190):
        m=(grey>thr).astype(np.uint8)
        m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        n,lab,stats,cents=cv2.connectedComponentsWithStats(m,8)
        cands=[]
        for i in range(1,n):
            x,y,w,h,area=stats[i];cx,cy=cents[i]
            if area>80 and np.hypot(cx-p[0],cy-p[1])<90 and 5<h<180:
                cands.append((area,float(cx),float(cy),int(x),int(y),int(w),int(h)))
        cands.sort(key=lambda c:-c[0])
        if cands:
            best=(thr,cands[0]);break
    return best

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('dataset',help='dataset dir under .private/real_piece_views')
    a=ap.parse_args()
    ddir=ROOT/'.private/real_piece_views'/a.dataset
    manifests=[ddir/'manifest.json']
    if not manifests[0].exists():
        print('no manifest',manifests[0]);return
    mf=json.load(open(manifests[0]))
    out={'schema':'base-centre-labels/1','dataset':a.dataset,'labels':[]}
    for pl in mf['placements']:
        piece,sq=pl['piece'],pl['square']
        x,y=pl['square_xy_mm']
        h=H.get(piece,23.0)
        entry={'piece':piece,'square':sq,'square_xy_mm':[x,y],'height_mm':h,'views':[]}
        print(f'\n=== {piece} @ {sq} (h={h}) ===',flush=True)
        for v in pl['views']:
            img=cv2.imread(str(ddir/v['image']))
            if img is None:
                print('  missing image',v['image'],flush=True);continue
            # recompute PnP from clean image (R/tvec were not stored)
            rect=b.rectify(img);sol,d=b.solve(rect)
            if sol is None:
                print('  no PnP',v['view'],flush=True);continue
            det=detect_top(img,sol,(x,y))
            rec={'view':v['view']}
            if det:
                thr,(area,cx,cy,bx,by,bw,bh)=det
                base_xy=b.unproject(sol,(cx,cy),h)  # unproject at piece height
                z0_xy=b.unproject(sol,(cx,cy),0.0)
                rec.update({'threshold':thr,'blob_px':[round(cx,1),round(cy,1)],
                            'base_xy':base_xy.round(2).tolist(),
                            'z0_xy':z0_xy.round(2).tolist(),
                            'blob':[bx,by,bw,bh]})
                print(f'  {v["view"]}: thr{thr} blob({cx:.0f},{cy:.0f}) base={base_xy.round(1)} z0={z0_xy.round(1)}',flush=True)
            else:
                print(f'  {v["view"]}: no blob',flush=True)
            entry['views'].append(rec)
        # agreement across views
        bases=np.array([v['base_xy'] for v in entry['views'] if 'base_xy' in v])
        if len(bases)>=2:
            entry['base_spread_mm']=np.round(bases.std(axis=0),2).tolist()
            entry['base_median']=np.round(np.median(bases,axis=0),2).tolist()
            print('  spread',entry['base_spread_mm'],'median',entry['base_median'],flush=True)
        out['labels'].append(entry)
    (ddir/'labels.json').write_text(json.dumps(out,indent=2)+'\n')
    print('\nwrote',ddir/'labels.json',flush=True)

if __name__=='__main__':main()
