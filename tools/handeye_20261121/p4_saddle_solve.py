#!/usr/bin/env python3
"""Offline diagnostic: densify hand-eye constraints with grid-vertex corners.

OpenCV's CharucoDetector cannot read our physical print (markers sit on dark
squares, CharUCo needs white), so we derive saddle-corner observations ourselves:

  * detect the ArUco markers (as before),
  * fit a board-plane -> image homography from their corners,
  * project the 8x8 grid's inner vertices into the image,
  * keep vertices whose projection is close to detected marker corners
    (no wild extrapolation from a single marker),
  * refine each projected vertex on the real image with cornerSubPix.

The refined vertices are genuine image observations of the printed grid and add
constraints at locations the marker corners do not cover. We then run the same
staged solve as tools/handeye_bundle.py with (a) ArUco corners only and
(b) ArUco corners + refined saddle corners, and compare reprojection RMS.
"""
from __future__ import annotations

import json, math, sys
from pathlib import Path
import cv2, numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tools"))
from chess_sim import fiducials  # noqa
from chess_sim.wrist_calibration import WristCalibration  # noqa
from handeye_bundle import (JOINTS, FIRST_ID, board_corners, chain_fk,  # noqa
                            pose_from_params, _average_pose)

CAP = ROOT / "live" / (sys.argv[1] if len(sys.argv) > 1 else "captures_handeye_all")
CAL_PATH = ROOT / ".private" / "phase11d" / "calibration.json"
BOARD = json.loads((ROOT / "live" / "board.json").read_text())
SQ = 28.0  # mm

cal = WristCalibration.load(CAL_PATH)
fx, fy, cx, cy = cal.intrinsics
K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1.0]])
DIST = np.asarray(cal.distortion, float).reshape(-1,1) if any(cal.distortion) else None

board0 = np.eye(4)
board0[:3,:3] = Rotation.from_euler("z", math.radians(BOARD["yaw_deg"])).as_matrix()
board0[:3,3] = [BOARD["x_mm"]/1000, BOARD["y_mm"]/1000, BOARD["z_mm"]/1000]
x0 = np.asarray(cal.camera_in_tool, float)

def load_records():
    out=[]
    for p in sorted(CAP.glob("*.json")):
        r=json.loads(p.read_text())
        out.append({"name": r["name"],
                    "joints_deg": {k: float(r["joints_deg"][k]) for k in JOINTS},
                    "roll": float(r["joints_deg"]["wrist_roll"])})
    return out

# Board-frame grid vertices (metres, z=0). 8x8 squares -> 9x9 vertices at
# square boundaries; use inner 7x7 plus boundary, then filter by projection.
verts=[]
for i in range(9):
    for j in range(9):
        x=(i-4)*SQ/1000.0; y=(j-4)*SQ/1000.0
        verts.append((x,y))
verts=np.array(verts,float)

def detect_with_saddles(rec, use_saddle):
    img=cv2.imread(str(CAP/f"{rec['name']}.jpg"))
    if img is None: return None
    frame=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    dets=fiducials.detect_markers(frame)
    objs=[]; imgs=[]
    for d in dets:
        mid=int(d.marker_id)
        if not (FIRST_ID<=mid<=FIRST_ID+31): continue
        objs.append(board_corners(mid))
        imgs.append(np.asarray(d.corners,float))
    n_aruco=len(objs)
    n_saddle=0
    if use_saddle and objs:
        allobj=np.vstack(objs).astype(np.float32)
        allimg=np.vstack(imgs).astype(np.float32)
        # homography board(mm? no, metres)->image; scale to millimetres for conditioning
        H,_=cv2.findHomography(allobj*1000.0, allimg, cv2.RANSAC, 3.0)
        if H is not None:
            pts3=verts*1000.0
            proj=cv2.perspectiveTransform(pts3.reshape(-1,1,2).astype(np.float32),H).reshape(-1,2)
            h,w=img.shape[:2]
            # keep vertices inside image and within 1.5 squares of any detected corner
            corners_img=np.vstack(imgs).astype(np.float32)
            keep=[]
            for k,p in enumerate(proj):
                if not (0<p[0]<w and 0<p[1]<h): continue
                dmin=np.min(np.linalg.norm(corners_img-p,axis=1))
                if dmin < 1.5*45:  # ~1.5 squares at ~45px/square (generous)
                    keep.append((k,p))
            gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            win=(7,7); crit=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.01)
            for k,p in keep:
                seed=np.array([[p]],np.float32)
                ref=cv2.cornerSubPix(gray,seed,win,(-1,-1),crit)[0][0]
                # reject if refinement jumped far (locked onto wrong feature)
                if np.linalg.norm(ref-p) < 12:
                    objs.append(np.array([[verts[k,0], verts[k,1], 0.0]]))
                    imgs.append(ref.reshape(1,2))
                    n_saddle+=1
    if not objs: return None
    return {"fk":chain_fk(cal,[rec["joints_deg"][k] for k in JOINTS]),
            "objects":np.vstack(objs).astype(np.float64),
            "images":np.vstack(imgs).astype(np.float64),
            "aruco":n_aruco,"saddle":n_saddle}

def solve(views, x_init, board_init, iters=12, bootstrap=True):
    X=x_init.copy(); B=board_init.copy()
    if bootstrap:
        # same trick as solve_staged: closed-form mount per view from the nominal board,
        # then a robust average rejects the wild single-marker solutions.
        cands=[]
        for v in views:
            ok,rvec,tvec=cv2.solvePnP(v["objects"],v["images"],K,None,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok: continue
            bc=np.eye(4); bc[:3,:3]=cv2.Rodrigues(rvec)[0]; bc[:3,3]=np.ravel(tvec)
            cands.append(np.linalg.inv(v["fk"])@board_init@np.linalg.inv(bc))
        if cands:
            X=_average_pose(cands)
    for it in range(iters+4):
        # robust per-view board-pose pruning (MAD), as in solve_staged
        dev=[]; bcs=[]
        for v in views:
            ok,rvec,tvec=cv2.solvePnP(v["objects"],v["images"],K,None,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                dev.append(1e9); bcs.append(None); continue
            bc=np.eye(4); bc[:3,:3]=cv2.Rodrigues(rvec)[0]; bc[:3,3]=np.ravel(tvec)
            bcs.append(bc)
            dev.append(np.linalg.norm((v["fk"]@X@bc)[:3,3]-B[:3,3]))
        dev=np.array(dev); med=np.median(dev); mad=np.median(np.abs(dev-med))+1e-6
        keep=[i for i,d in enumerate(dev) if d<=med+3*mad]
        if len(keep)>=max(3,len(views)//2):
            vv=[views[i] for i in keep]; bb=[bcs[i] for i in keep]
        else:
            vv=views; bb=[b for b in bcs if b is not None]
        if bb: B=_average_pose([v["fk"]@X@bc for v,bc in zip(vv,bb)])
        xs=[]
        for v,bc in zip(vv,bb):
            xs.append(np.linalg.inv(v["fk"])@B@np.linalg.inv(bc))
        if xs: X=_average_pose(xs)
    resid=[]
    for v in views:
        base_cam=v["fk"]@X
        w2c=np.linalg.inv(base_cam)
        bp=(B[:3,:3]@v["objects"].T).T+B[:3,3]
        cp=(w2c[:3,:3]@bp.T).T+w2c[:3,3]
        z=np.where(np.abs(cp[:,2])<1e-6,1e-6,cp[:,2])
        proj=np.column_stack([fx*cp[:,0]/z+cx,fy*cp[:,1]/z+cy])
        resid.extend((proj-v["images"]).ravel())
    resid=np.array(resid)
    return {"rms":float(np.sqrt(np.mean(resid**2))),"views":len(views),
            "npts":sum(len(v["objects"]) for v in views),"X":X,"B":B}

records=load_records()
for use in (False,True):
    views=[]
    for r in records:
        v=detect_with_saddles(r,use)
        if v: views.append(v)
    if len(views)<3:
        print("too few views",len(views)); continue
    res=solve(views,x0,board0)
    tag="ArUco-only" if not use else "ArUco+saddle"
    print(f"{tag}: views={res['views']} points={res['npts']} "
          f"aruco={sum(v['aruco'] for v in views)} saddle={sum(v['saddle'] for v in views)} "
          f"RMS={res['rms']:.3f} px  board z={res['B'][2,3]*1000:.1f}mm")
    if use:
        out={"method":"saddle_augmented","rms_reproj_px":round(res['rms'],3),
             "views":res['views'],"points":res['npts'],
             "sightings_aruco":sum(v['aruco'] for v in views),
             "sightings_saddle":sum(v['saddle'] for v in views),
             "camera_in_tool":[[round(float(x),6) for x in row] for row in res['X']],
             "board":{"x_mm":round(float(res['B'][0,3])*1000,2),
                      "y_mm":round(float(res['B'][1,3])*1000,2),
                      "z_mm":round(float(res['B'][2,3])*1000,2)}}
        (ROOT/"reports"/"handeye_saddle_20261122.json").write_text(json.dumps(out,indent=2))
