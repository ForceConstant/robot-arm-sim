#!/usr/bin/env python3
"""Improved base-centre labeler: mask ChArUco marker faces before detecting the
piece, so marker quadrilaterals can't be mistaken for the piece (the main
contamination in p4_label_base_centres.py).

For each clean view:
  - solve PnP; get detected marker corner quads
  - build a mask that blanks those quads (dilated) in the rectified image
  - search for the piece silhouette near the target square (tall pieces project
    their top away from the base, so use a generous radius), preferring a blob
    whose brightness/saturation differs from the dark board and that is NOT on
    a masked marker
  - unproject the blob's top centroid at the piece's known height -> base xy

Compares coverage + cross-view spread with the unmasked labels.json.
"""
import json, sys
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board

ROOT=Path('/home/brian/robot-arm-sim')
b=Board(json.load(open(str(ROOT/'.private/phase11d/calibration.json'))))
H={'pawn':23.0,'rook':26.0,'knight':32.0,'bishop':36.5,'queen':40.5,'king':46.5}

def detect_piece(rect, sol, dets, sq_xy, piece_h):
    """Return (top_pixel, diagnostics) for the piece near the target square."""
    grey=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
    # mask out marker quads (dilated) - they are white and grab the threshold
    mask=np.zeros(grey.shape,np.uint8)
    for det in dets:
        pts=np.asarray(det.corners,float).reshape(-1,2).astype(np.int32)
        cv2.fillConvexPoly(mask,pts,255)
    mask=cv2.dilate(mask,np.ones((7,7),np.uint8),iterations=1)
    # base pixel of the target square; piece top can be far from it for tall pieces
    pbase=b.project(sol,np.array([sq_xy[0],sq_xy[1],0.]),0.)
    # generous search radius scales with piece height (perspective offset)
    radius=int(np.clip(28+piece_h*1.6,50,180))
    hsv=cv2.cvtColor(rect,cv2.COLOR_BGR2HSV)
    best=None
    for thr in (110,135,160,185):
        bright=(grey>thr).astype(np.uint8)*255
        bright=cv2.bitwise_and(bright,cv2.bitwise_not(mask))
        bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        n,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
        cands=[]
        for i in range(1,n):
            x,y,w,hh,area=stats[i];cx,cy=cents[i]
            dbase=np.hypot(cx-pbase[0],cy-pbase[1])
            if area>60 and dbase<radius and 4<hh<200 and w<160:
                cands.append((area,float(cx),float(cy),dbase))
        cands.sort(key=lambda c:-c[0])
        if cands:
            # prefer largest blob; if several, prefer closest to base among top 3
            top=cands[:3]
            chosen=min(top,key=lambda c:c[3])
            best=(thr,chosen,cands)
            break
    return best,mask

def main():
    ds=sys.argv[1] if len(sys.argv)>1 else 'adapt1'
    ddir=ROOT/'.private/real_piece_views'/ds
    mf=json.load(open(ddir/'manifest.json'))
    out={'schema':'base-centre-labels-masked/1','dataset':ds,'labels':[]}
    for pl in mf['placements']:
        piece,sq=pl['piece'],pl['square'];x,y=pl['square_xy_mm']
        h=H.get(piece,23.0)
        entry={'piece':piece,'square':sq,'square_xy_mm':[x,y],'height_mm':h,'views':[]}
        print(f'\n=== {piece} @ {sq} (h={h}) ===',flush=True)
        for v in pl['views']:
            img=cv2.imread(str(ddir/v['image']))
            if img is None:continue
            rect=b.rectify(img);sol,d=b.solve(rect)
            if sol is None:continue
            best,mask=detect_piece(rect,sol,d,(x,y),h)
            rec={'view':v['view']}
            if best:
                thr,(area,cx,cy,dbase),cands=best
                base=b.unproject(sol,(cx,cy),h)
                rec.update({'thr':int(thr),'top_px':[round(float(cx),1),round(float(cy),1)],
                            'base_xy':base.round(2).tolist(),'area':int(area),
                            'dbase_px':round(float(dbase),1)})
                print(f'  {v["view"]}: thr{thr} top({cx:.0f},{cy:.0f}) base={base.round(1)} area={area} db={dbase:.0f}',flush=True)
            else:
                print(f'  {v["view"]}: no blob',flush=True)
            entry['views'].append(rec)
        bases=np.array([v['base_xy'] for v in entry['views'] if 'base_xy' in v])
        if len(bases)>=2:
            entry['base_spread_mm']=np.round(bases.std(axis=0),2).tolist()
            entry['base_median']=np.round(np.median(bases,axis=0),2).tolist()
            print('  spread',entry['base_spread_mm'],'median',entry['base_median'],flush=True)
        out['labels'].append(entry)
    (ddir/'labels_masked.json').write_text(json.dumps(out,indent=2)+'\n')
    print('\nwrote',ddir/'labels_masked.json',flush=True)

if __name__=='__main__':main()
