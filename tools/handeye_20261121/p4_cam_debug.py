#!/usr/bin/env python3
"""Debug: FK camera vs PnP camera for one capture, in base and board frames."""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tools")); sys.path.insert(0, str(ROOT / "live"))
from chess_sim import fiducials
from chess_sim.wrist_calibration import WristCalibration
from handeye_bundle import FIRST_ID, JOINTS, board_corners, chain_fk
from live.kinematics import ArmKinematics, frame_at

CAP = ROOT / "live" / "captures_handeye_20261121"


def main():
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    b = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])
    kin = ArmKinematics(cal, {}, board)
    K = np.array([[cal.intrinsics[0], 0, cal.intrinsics[2]],
                  [0, cal.intrinsics[1], cal.intrinsics[3]], [0, 0, 1]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    X = np.asarray(cal.camera_in_tool, float)
    for path in sorted(CAP.glob("*.json")):
        rec = json.loads(path.read_text())
        joints = [float(rec["joints_deg"][n]) for n in JOINTS]
        image = cv2.imread(str(CAP / f"{rec['name']}.jpg"))
        dets = fiducials.detect_markers(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        obj, img = [], []
        for d in dets:
            if FIRST_ID <= d.marker_id <= FIRST_ID + 31:
                obj.append(board_corners(d.marker_id)); img.append(np.asarray(d.corners, float))
        if not obj:
            continue
        ok, rvec, tvec = cv2.solvePnP(np.vstack(obj), np.vstack(img), K, dist,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        R = cv2.Rodrigues(rvec)[0]
        pnp_cam_base = (board[:3, :3] @ (-R.T @ np.ravel(tvec))) + board[:3, 3]
        fk_cam_base = kin.camera_pose(joints)[:3, 3]
        tip = (kin.fk(joints) @ np.eye(4))[:3, 3]
        print(f"{rec['name'][-16:]:16s} ids {sorted(d.marker_id for d in dets)}")
        print(f"   pnp_cam_base {np.round(pnp_cam_base*1000,1).tolist()}  "
              f"fk_cam_base {np.round(fk_cam_base*1000,1).tolist()}  "
              f"diff {np.linalg.norm(pnp_cam_base-fk_cam_base)*1000:.1f} mm")
        print(f"   tool_origin_base {np.round(tip*1000,1).tolist()}")
    return 0


raise SystemExit(main())
