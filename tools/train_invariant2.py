#!/usr/bin/env python3
"""Position-invariant v2: features from BOARD-RECTIFIED top-down patches.

The camera pose differs per view, so raw-image patches of the same piece look
different. Here we warp each clean frame to a canonical top-down board image
(Board.rectified), crop a patch centred on the piece's BOARD position (from the
human click), and compute features there. Every piece is then seen in the same
board frame at the same scale, independent of camera position/angle.

Evaluates merged/round1/round2 with leave-one-view-out.
"""
import sys,json
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from chess_sim.board_target import Board
from train_real_piece_model import TYPES

ROOT=Path('/home/brian/robot-arm-sim/.private/real_piece_views')
b=Board(json.load(open('/home/brian/robot-arm-sim/.private/phase11d/calibration.json')))
H={'pawn':23.0,'rook':26.0,'knight':32.0,'bishop':36.5,'queen':40.5,'king':46.5}

def topdown_features(rect,sol,click_uv,board_xy,piece,ppmm=8.0,patch_mm=64):
    """Warp to canonical board top-down; crop around the piece's board xy."""
    img,valid,n,scale=b.rectified(rect,sol,ppmm)
    # rectified: n x n covers 256mm board centered; pixel = (board_mm*ppmm + n/2)
    npx=int(round(patch_mm*ppmm))
    cx=int(n/2+board_xy[0]*ppmm);cy=int(n/2+board_xy[1]*ppmm)
    x0,y0=cx-npx//2,cy-npx//2
    rs=cv2.copyMakeBorder(img,max(0,-y0),max(0,y0+npx-n),max(0,-x0),max(0,x0+npx-n),
                          cv2.BORDER_REFLECT)
    ox=x0+max(0,-x0);oy=y0+max(0,-y0)
    patch=rs[oy:oy+npx,ox:ox+npx].astype(np.float32)
    if patch.shape[:2]!=(npx,npx):
        patch=cv2.resize(patch,(npx,npx))
    grey=cv2.cvtColor(patch.astype(np.uint8),cv2.COLOR_BGR2GRAY).astype(np.float32)
    # background = outer ring, residual = piece
    ring=np.concatenate([patch[:6].reshape(-1,3),patch[-6:].reshape(-1,3),
                         patch[:,:6].reshape(-1,3),patch[:,-6:].reshape(-1,3)])
    bg=np.median(ring,axis=0);resid=patch-bg
    g=np.linalg.norm(resid,axis=2)
    mask=(g>np.percentile(g,60)).astype(np.uint8)
    ncc,lab,stats,cents=cv2.connectedComponentsWithStats(mask,8)
    area=max([stats[i,cv2.CC_STAT_AREA] for i in range(1,ncc)] or [0])
    f=[g.mean()/255,g.std()/255,float(g.max())/255,area/(npx*npx)]
    # thumbnail of residual (shape signature)
    thumb=cv2.resize(np.clip(g,0,255),(8,8),interpolation=cv2.INTER_AREA).ravel()/255
    f=list(f)+list(thumb)
    mn=resid.reshape(-1,3).mean(0);f+=[float(v)/(np.linalg.norm(mn)+1e-6) for v in mn]
    return np.array(f,np.float32)

def load_rows(ds):
    ddir=ROOT/ds
    mf=json.load(open(ddir/'manifest.json'));lab=json.load(open(ddir/'labels_human.json'))
    rows=[]
    for pl in mf['placements']:
        piece=pl['piece']
        for v in pl['views']:
            k=v['image']
            if k not in lab:continue
            img=cv2.imread(str(ddir/k));h,w=img.shape[:2]
            if h>w:img=cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)
            rect=b.rectify(img);sol,d=b.solve(rect)
            if sol is None:continue
            board_xy=b.unproject(sol,lab[k],H[piece])
            f=topdown_features(rect,sol,lab[k],board_xy,piece)
            rows.append({'key':k,'piece':piece,'view':v['view'],'feat':f,'base':board_xy})
    return rows

def acc(rows,label):
    X=np.array([r['feat'] for r in rows]);y=[r['piece'] for r in rows]
    mean=X.mean(0);std=np.where(X.std(0)<1e-6,1.0,X.std(0));Xn=(X-mean)/std
    ok=0
    for i in range(len(rows)):
        tr=np.array([j for j in range(len(rows)) if j!=i])
        cents={t:Xn[tr[[k for k,j in enumerate(tr) if y[j]==t]]].mean(0) for t in TYPES if any(y[j]==t for j in tr)}
        p=min(cents,key=lambda t:np.linalg.norm(Xn[i]-cents[t]))
        ok+=(p==rows[i]['piece'])
    print(f'{label}: {ok}/{len(rows)} = {ok/len(rows):.3f}',flush=True)
    return ok/len(rows)

def main():
    rows=load_rows('merged1');print(f'topdown rows {len(rows)} dim {rows[0]["feat"].shape[0]}',flush=True)
    r1=[r for r in rows if 'round2' not in r['key']];r2=[r for r in rows if 'round2' in r['key']]
    res={'merged':acc(rows,'merged'),'round1':acc(r1,'round1'),'round2':acc(r2,'round2')}
    json.dump(res,open('/home/brian/robot-arm-sim/reports/invariant2_eval.json','w'),indent=2)
    print('report written',flush=True)
if __name__=='__main__':main()
