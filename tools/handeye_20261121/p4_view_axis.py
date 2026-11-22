#!/usr/bin/env python3
"""Mount orientation error per view: FK camera vs PnP camera (stored mount)."""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tools")); sys.path.insert(0, str(ROOT / "live"))
from chess_sim import fiducials
from chess_sim.wrist_calibration import WristCalibration
from handeye_bundle import FIRST_ID, JOINTS, board_corners, chain_fk

CAP = ROOT / "live" / "captures_handeye_20261121"


def rot_deg(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1.0) / 2.0))))


def main():
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    K = np.array([[cal.intrinsics[0], 0, cal.intrinsics[2]],
                  [0, cal.intrinsics[1], cal.intrinsics[3]], [0, 0, 1]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    X = np.asarray(cal.camera_in_tool, float)
    errs = []
    for path in sorted(CAP.glob("*.json")):
        rec = json.loads(path.read_text())
        joints = [float(rec["joints_deg"][n]) for n in JOINTS]
        image = cv2.imread(str(CAP / f"{rec['name']}.jpg"))
        dets = fiducials.detect_markers(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        obj, img = [], []
        for d in dets:
            if FIRST_ID <= d.marker_id <= FIRST_ID + 31:
                obj.append(board_corners(d.marker_id)); img.append(np.asarray(d.corners, float))
        if len(obj) < 2:
            continue
        ok, rvec, tvec = cv2.solvePnP(np.vstack(obj), np.vstack(img), K, dist,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        R_pnp = cv2.Rodrigues(rvec)[0]          # camera<-board
        R_fk = (chain_fk(cal, joints) @ X)[:3, :3]  # camera<-? no: base<-camera
        R_fk = R_fk.T                            # camera<-base
        # both are camera<-board / camera<-base; compare view axes in base
        view_pnp = -(R_pnp.T @ np.array([0, 0, 1.0]))
        view_fk = -np.array([0, 0, 1.0]) @ (chain_fk(cal, joints) @ X)[:3, :3]
        ang = math.degrees(math.acos(max(-1, min(1, float(
            np.dot(view_pnp / np.linalg.norm(view_pnp), view_fk / np.linalg.norm(view_fk)))))))
        errs.append(ang)
        print(f"{rec['name'][-16:]:16s} markers {len(obj)} view-axis error {ang:5.2f} deg")
    if errs:
        print(f"view-axis error: mean {np.mean(errs):.2f} deg max {np.max(errs):.2f} deg")
    return 0


raise SystemExit(main())
