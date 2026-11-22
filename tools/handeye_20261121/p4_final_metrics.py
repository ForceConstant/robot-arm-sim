#!/usr/bin/env python3
"""Final metrics: before/after PnP-vs-FK disagreement and branch grouping."""
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
from handeye_bundle import (FIRST_ID, JOINTS, board_corners, chain_fk,
                            solve_staged)
from live.kinematics import ArmKinematics, frame_at

CAP = ROOT / "live" / "captures_handeye_20261121"


def main():
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    b = json.loads((ROOT / "live/board.json").read_text())
    board_f = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])
    kin = ArmKinematics(cal, {}, board_f)
    K = np.array([[cal.intrinsics[0], 0, cal.intrinsics[2]],
                  [0, cal.intrinsics[1], cal.intrinsics[3]], [0, 0, 1]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    records = [json.loads(p.read_text()) for p in sorted(CAP.glob("*.json"))]
    board0 = board_f.copy()
    res = solve_staged(CAP, records, cal, tuple(float(v) for v in cal.intrinsics),
                       np.asarray(cal.camera_in_tool, float), board0,
                       distortion=cal.distortion, bootstrap=True)
    print("solve:", {k: res[k] for k in ("views", "sightings", "rms_reproj_px",
                                         "camera_translation_mm", "board")})
    x_old = np.asarray(cal.camera_in_tool, float)
    x_new = np.asarray(res["camera_in_tool"], float)
    rd = x_old[:3, :3].T @ x_new[:3, :3]
    ang = math.degrees(math.acos(max(-1, min(1, (np.trace(rd) - 1) / 2))))
    axis = cv2.Rodrigues(rd)[0].ravel()
    print(f"stored mount mm {np.round(x_old[:3,3]*1000,1).tolist()}")
    print(f"new mount    mm {np.round(x_new[:3,3]*1000,1).tolist()}")
    print(f"rotation delta {ang:.1f} deg about {np.round(axis/np.linalg.norm(axis),2).tolist()}")

    rows = []
    for rec in records:
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
        pnp_cam = board_f[:3, :3] @ (-R.T @ np.ravel(tvec)) + board_f[:3, 3]
        tip = kin.fk(joints)[:3, 3]
        out = {"name": rec["name"][-16:], "roll": float(rec["joints_deg"]["wrist_roll"]),
               "markers": sorted(int(d.marker_id) for d in dets)}
        for tag, X in (("before", x_old), ("after", x_new)):
            cam = (kin.fk(joints) @ X)[:3, 3]
            out[tag] = float(np.linalg.norm(cam - pnp_cam) * 1000)
            az_p = math.degrees(math.atan2(*(pnp_cam[:2] - tip[:2])[::-1]))
            az_f = math.degrees(math.atan2(*(cam[:2] - tip[:2])[::-1]))
            out[tag + "_az"] = (az_p - az_f + 180) % 360 - 180
        rows.append(out)
    print("\nper view:")
    for r in rows:
        print(f"  {r['name']:16s} roll {r['roll']:6.2f} markers {str(r['markers']):12s} "
              f"before {r['before']:6.2f} mm ({r['before_az']:+6.2f} deg)  "
              f"after {r['after']:6.2f} mm ({r['after_az']:+6.2f} deg)")
    for tag in ("before", "after"):
        d = np.array([r[tag] for r in rows]); a = np.array([r[tag + "_az"] for r in rows])
        print(f"{tag}: mean {d.mean():.2f} mm max {d.max():.2f} mm | "
              f"azimuth mean {a.mean():+.2f} std {a.std():.2f}")
    for branch in sorted({round(r["roll"], 1) for r in rows}):
        sub = [r for r in rows if round(r["roll"], 1) == branch]
        for tag in ("before", "after"):
            d = np.array([r[tag] for r in sub])
            print(f"  branch {branch:6.2f} n {len(sub)} {tag}: mean {d.mean():.2f} "
                  f"max {d.max():.2f}")
    print(f"\nmarker sightings {sum(len(r['markers']) for r in rows)} over {len(rows)} views")
    print("views with >=3 markers:",
          sum(1 for r in rows if len(r["markers"]) >= 3))
    return 0


raise SystemExit(main())
