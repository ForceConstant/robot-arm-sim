#!/usr/bin/env python3
"""Read-only hand-eye evaluation: before/after residuals per roll branch.

Uses tools/handeye_bundle.solve_staged (the documented routine) and reports:
  * per-view board-pose deviation with the stored mount and with the new one
  * PnP-vs-FK camera disagreement per view (the Step 2 metric)
  * leave-one-out held-out prediction error
  * grouping by the measured wrist roll
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from handeye_bundle import (FIRST_ID, JOINTS, board_corners,  # noqa: E402
                            chain_fk, solve_staged)

import sys as _s
CAP = ROOT / "live" / (_s.argv[1] if len(_s.argv) > 1 else "captures_handeye_20261121")


def load_records():
    out = []
    for path in sorted(CAP.glob("*.json")):
        rec = json.loads(path.read_text())
        out.append({"name": rec["name"],
                    "joints_deg": {k: float(rec["joints_deg"][k]) for k in JOINTS},
                    "roll": float(rec["joints_deg"]["wrist_roll"]),
                    "markers": [m["id"] for m in rec.get("markers", [])]})
    return out


def detect(rec, cal):
    image = cv2.imread(str(CAP / f"{rec['name']}.jpg"))
    if image is None:
        return None
    frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    dets = fiducials.detect_markers(frame)
    objects, images = [], []
    for d in dets:
        if not (FIRST_ID <= d.marker_id <= FIRST_ID + 31):
            continue
        objects.append(board_corners(d.marker_id))
        images.append(np.asarray(d.corners, float))
    if not objects:
        return None
    obj = np.vstack(objects).astype(np.float64)
    img = np.vstack(images).astype(np.float64)
    K = np.array([[cal.intrinsics[0], 0, cal.intrinsics[2]],
                  [0, cal.intrinsics[1], cal.intrinsics[3]], [0, 0, 1]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    board_camera = np.eye(4)
    board_camera[:3, :3] = cv2.Rodrigues(rvec)[0]
    board_camera[:3, 3] = np.ravel(tvec)
    return {"obj": obj, "img": img, "board_camera": board_camera,
            "fk": chain_fk(cal, [rec["joints_deg"][n] for n in JOINTS]),
            "markers": len(objects)}


def board_pose_from_result(result):
    x = np.asarray(result["camera_in_tool"], float)
    b = result["board"]
    board = np.eye(4)
    board[:3, :3] = Rotation.from_euler("z", math.radians(b["yaw_deg"])).as_matrix()
    board[:3, 3] = [b["x_mm"] / 1000.0, b["y_mm"] / 1000.0, b["z_mm"] / 1000.0]
    return x, board


def rot_deg(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1.0) / 2.0))))


def spread(rows):
    t = np.array([r["t"] for r in rows])
    med = np.median(t, axis=0)
    d = np.linalg.norm(t - med, axis=1)
    return med, d


def main() -> int:
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    board_meas = json.loads((ROOT / "live/board.json").read_text())
    records = load_records()
    print(f"{len(records)} capture records in {CAP.name}")
    if len(records) < 4:
        print("too few views")
        return 2

    views = {}
    for rec in records:
        v = detect(rec, cal)
        if v is not None:
            views[rec["name"]] = v
    print(f"{len(views)} views with a PnP solve; rolls "
          f"{sorted({round(r['roll'], 2) for r in records})}")

    board0 = np.eye(4)
    board0[:3, :3] = Rotation.from_euler("z", math.radians(board_meas["yaw_deg"])).as_matrix()
    board0[:3, 3] = [board_meas["x_mm"] / 1000.0, board_meas["y_mm"] / 1000.0,
                     board_meas["z_mm"] / 1000.0]

    intrinsics = tuple(float(v) for v in cal.intrinsics)
    result = solve_staged(CAP, records, cal, intrinsics,
                          np.asarray(cal.camera_in_tool, float), board0,
                          distortion=cal.distortion, bootstrap=True)
    if "error" in result:
        print("staged solve failed:", result["error"])
        return 3
    print(f"staged solve: views {result['views']} sightings {result['sightings']} "
          f"rms {result['rms_reproj_px']} px")
    print(f"camera_in_tool new mm {result['camera_translation_mm']}")
    print(f"board new {result['board']}")

    x_old = np.asarray(cal.camera_in_tool, float)
    x_new, board_new = board_pose_from_result(result)
    dt = np.linalg.norm(x_new[:3, 3] - x_old[:3, 3]) * 1000
    dr = rot_deg(x_old[:3, :3], x_new[:3, :3])
    print(f"mount change: {dt:.2f} mm, {dr:.2f} deg")

    def per_view(rows_x):
        out = []
        for rec in records:
            v = views.get(rec["name"])
            if v is None:
                continue
            board = v["fk"] @ rows_x @ v["board_camera"]
            out.append({"name": rec["name"], "roll": rec["roll"], "t": board[:3, 3],
                        "R": board[:3, :3], "markers": v["markers"]})
        return out

    for label, X in (("BEFORE (stored mount)", x_old), ("AFTER (new mount)", x_new)):
        rows = per_view(X)
        med, d = spread(rows)
        print(f"\n{label}: board-pose deviation from the view median")
        print(f"  mean {d.mean()*1000:.2f} mm  max {d.max()*1000:.2f} mm  "
              f"std {d.std()*1000:.2f} mm  (n={len(rows)})")
        for r in sorted(rows, key=lambda r: -np.linalg.norm(r["t"] - med)):
            dev = np.linalg.norm(r["t"] - med) * 1000
            print(f"    {r['name'][-18:]:18s} roll {r['roll']:7.2f} "
                  f"markers {r['markers']} dev {dev:6.2f} mm")
        for branch in sorted({round(r["roll"], 1) for r in rows}):
            sub = [r for r in rows if round(r["roll"], 1) == branch]
            dd = np.array([np.linalg.norm(r["t"] - med) for r in sub]) * 1000
            print(f"  branch {branch:7.2f}: n {len(sub)} mean {dd.mean():.2f} mm "
                  f"max {dd.max():.2f} mm")

    # PnP-vs-FK camera disagreement, the Step 2 metric
    print("\nPnP-vs-FK camera disagreement (mm) / azimuth diff (deg):")
    for label, X in (("before", x_old), ("after", x_new)):
        rows = []
        for rec in records:
            v = views.get(rec["name"])
            if v is None:
                continue
            board = np.linalg.inv(board0) @ v["fk"] @ X
            fk_cam = -board[:3, :3].T @ board[:3, 3]
            bc = v["board_camera"]
            pnp_cam = -bc[:3, :3].T @ bc[:3, 3]
            tip = (np.linalg.inv(board0) @ v["fk"])[:3, 3]
            az_pnp = math.degrees(math.atan2(pnp_cam[1] - tip[1], pnp_cam[0] - tip[0]))
            az_fk = math.degrees(math.atan2(fk_cam[1] - tip[1], fk_cam[0] - tip[0]))
            rows.append((rec["name"], rec["roll"],
                         np.linalg.norm(pnp_cam - fk_cam) * 1000,
                         (az_pnp - az_fk + 180) % 360 - 180))
        d = np.array([r[2] for r in rows])
        a = np.array([r[3] for r in rows])
        print(f"  {label}: mean {d.mean():.2f} mm  max {d.max():.2f} mm  "
              f"azimuth mean {a.mean():+.2f} deg  std {a.std():.2f} deg")
        for name, roll, mm, az in rows:
            print(f"    {name[-18:]:18s} roll {roll:7.2f}  {mm:6.2f} mm  {az:+7.2f} deg")

    # Leave-one-out: fit on 12 views, predict the held-out board pose
    print("\nleave-one-out held-out prediction (new mount):")
    errs = []
    for rec in records:
        train = [r for r in records if r["name"] != rec["name"]]
        res = solve_staged(CAP, train, cal, intrinsics,
                           np.asarray(cal.camera_in_tool, float), board0,
                           distortion=cal.distortion, bootstrap=True)
        if "error" in res:
            print(f"  {rec['name'][-18:]}: train failed")
            continue
        xt, bt = board_pose_from_result(res)
        v = views.get(rec["name"])
        if v is None:
            continue
        pred = v["fk"] @ xt @ v["board_camera"]
        e = float(np.linalg.norm(pred[:3, 3] - bt[:3, 3]) * 1000)
        errs.append(e)
        print(f"  {rec['name'][-18:]:18s} roll {rec['roll']:7.2f}  err {e:6.2f} mm  "
              f"train_rms {res['rms_reproj_px']} px")
    if errs:
        print(f"  held-out mean {np.mean(errs):.2f} mm  max {np.max(errs):.2f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
