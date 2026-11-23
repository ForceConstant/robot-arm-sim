#!/usr/bin/env python3
"""Shared detection overlay for the live viewers (2026-11-22).

Key fix: the console /stream serves a frame already rotated 90 CW for display
(rotate_deg=90). The calibration expects the RAW landscape 640x480 frame, so we
rotate 90 COUNTER-CLOCKWISE to recover it. Previously rotating CW double-rotated
the image, causing sideways display and bad PnP.

Markers:
  RED cross      = nominal e5 centre (14,14) board mm  (the assumption)
  GREEN cross    = detected piece centroid (bright blob)  (the measurement)
  YELLOW diamond = estimated piece base contact
  CYAN square    = jaw tip centre from FK (/keep-in tip_mm), projected via PnP
  ORANGE dots    = PnP square lattice
"""
import json, urllib.request
import cv2, numpy as np
from chess_sim.board_target import Board

CAL='/home/brian/robot-arm-sim/.private/phase11d/calibration.json'
b=Board(json.load(open(CAL)))

def to_raw(frame):
    """Console stream is portrait (rotated CW for display); recover landscape."""
    h,w=frame.shape[:2]
    if h>w:
        frame=cv2.rotate(frame,cv2.ROTATE_90_COUNTERCLOCKWISE)
    if (frame.shape[1],frame.shape[0])!=(640,480):
        frame=cv2.resize(frame,(640,480))
    return frame

def jaw_tip_board():
    try:
        with urllib.request.urlopen('http://192.168.1.171:8080/keep-in',timeout=2) as r:
            d=json.load(r)
        return np.array(d['tip_mm'][:2],float)
    except Exception:
        return None


def saved_tip_pixel():
    """User-set finger-tip midpoint pixel (raw 640x480) from the dashboard.

    The jaws are rigid to the wrist camera so this is a fixed pixel needing no
    hand-eye calibration.  Returns (u, v) or None when not set.
    """
    try:
        with urllib.request.urlopen('http://192.168.1.171:8080/tip-center',timeout=2) as r:
            d=json.load(r)
        if d.get('set') and d.get('pixel'):
            return float(d['pixel'][0]), float(d['pixel'][1])
    except Exception:
        pass
    return None


def draw_tip_marker(f, sol=None):
    """Draw the cyan jaw-tip-centre marker from the saved pixel.

    Falls back to the legacy FK-tip-through-PnP projection only if the user has
    not set a pixel (that fallback is known-invalid; it is kept for the unset
    state so a marker is still shown).  Returns the info dict contribution.
    """
    px=saved_tip_pixel()
    if px is not None:
        u,v=int(px[0]),int(px[1])
        cv2.drawMarker(f,(u,v),(255,255,0),cv2.MARKER_CROSS,20,2,cv2.LINE_AA)
        cv2.putText(f,'tip centre',(u+8,v+16),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,0),1)
        return {'tip_pixel':[u,v],'tip_source':'user'}
    if sol is not None:
        tip=jaw_tip_board()
        if tip is not None:
            q=b.project(sol,np.array([tip[0],tip[1],0.]),0.)
            u,v=int(q[0]),int(q[1])
            cv2.drawMarker(f,(u,v),(255,255,0),cv2.MARKER_SQUARE,16,2,cv2.LINE_AA)
            cv2.putText(f,'jaw tip (FK)',(u+8,v+16),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,0),1)
            return {'tip_pixel':[u,v],'tip_source':'fk','jaw_tip_mm':tip.round(1).tolist()}
    return {}


def detect(frame, draw_jaw=True):
    f=to_raw(frame)
    rect=b.rectify(f); sol,d=b.solve(rect)
    info={'markers':len(d),'rms':round(sol['rms_px'],2) if sol else None}
    if sol is None or len(d)<2:
        cv2.putText(f,f"ids {len(d)} - no pose",(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
        if draw_jaw:
            info.update(draw_tip_marker(f, None))
        return f,info
    # square lattice
    g=b.grid(sol)
    for row in g['points']:
        for p in row:
            cv2.circle(f,(int(p[0]),int(p[1])),2,(255,120,60),-1)
    def pp(mm):
        q=b.project(sol,np.array([mm[0],mm[1],0.]),0.);return(int(q[0]),int(q[1]))
    nom=np.array([14.,14.])
    cv2.drawMarker(f,pp(nom),(0,0,255),cv2.MARKER_CROSS,18,2,cv2.LINE_AA)
    cv2.putText(f,'e5 nominal',(pp(nom)[0]+8,pp(nom)[1]-8),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,0,255),1)
    # jaw tip centre marker (user-set pixel; legacy FK fallback)
    if draw_jaw:
        info.update(draw_tip_marker(f, sol))
    # bright piece blob near e5
    grey=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
    p5=b.project(sol,np.array([14.,14.,0.]),0.)
    bright=(grey>170).astype(np.uint8)
    bright=cv2.morphologyEx(bright,cv2.MORPH_OPEN,np.ones((7,7),np.uint8))
    cnt,lab,stats,cents=cv2.connectedComponentsWithStats(bright,8)
    cands=[]
    for i in range(1,cnt):
        x,y,w,h,area=stats[i]; cx,cy=cents[i]
        dp=np.hypot(cx-p5[0],cy-p5[1])
        if area>200 and dp<90 and h<180:
            cands.append((area,cx,cy,x,y,w,h))
    cands.sort(key=lambda c:-c[0])
    if cands:
        area,cx,cy,x,y,w,h=cands[0]
        cm=b.unproject(sol,(cx,cy),0.); base=b.unproject(sol,(cx,y+h-1),0.)
        info.update({'centroid_mm':cm.round(1).tolist(),'base_mm':base.round(1).tolist(),
                     'offset_mm':round(float(np.linalg.norm(cm-nom)),1)})
        cv2.drawMarker(f,(int(cx),int(cy)),(60,255,90),cv2.MARKER_CROSS,22,3,cv2.LINE_AA)
        cv2.drawMarker(f,(int(cx),int(y+h-1)),(0,220,255),cv2.MARKER_DIAMOND,14,2,cv2.LINE_AA)
        cv2.putText(f,f'piece {cm.round(1).tolist()}',(int(cx)+10,int(cy)-10),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(60,255,90),2)
    # HUD
    bits=[f"ids {len(d)} rms {info['rms']}"]
    if info.get('centroid_mm'): bits.append(f"piece {info['centroid_mm']} off {info['offset_mm']}mm")
    else: bits.append('no piece blob')
    if info.get('jaw_tip_mm'): bits.append(f"jaw {info['jaw_tip_mm']}")
    if info.get('tip_source')=='user':
        bits.append(f"tip px {info['tip_pixel']}")
    cv2.rectangle(f,(0,452),(640,480),(0,0,0),-1)
    cv2.putText(f,' | '.join(bits),(8,472),cv2.FONT_HERSHEY_SIMPLEX,0.42,(255,255,255),1,cv2.LINE_AA)
    return f,info
