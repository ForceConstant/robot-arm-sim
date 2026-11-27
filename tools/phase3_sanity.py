#!/usr/bin/env python3
"""Phase 3 sanity: has the board (or the camera mount) moved since 2026-11-21?

Read-only.  Grabs frames at the current pose (no motion) and runs the *same*
estimator over the 2026-11-21 hand-eye captures.  Each marker PnP gives the
board pose in the arm base frame through FK; the median over markers is one
board-pose estimate per capture set.  Because both sides use the same pipeline
and the same (biased) mount, a board that has not moved gives the same estimate
on both days; a board that moved shows up as a difference between them.

The stored mount is a MuJoCo-convention camera frame (local -Z is the view
axis, local +Y is up -- see ``CameraCalibration.project``); OpenCV PnP uses +Z
forward and +Y down, hence the 180 deg flip about local X before composing.

    .venv/bin/python tools/phase3_sanity.py [--frames 3]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from chess_sim.projection import FLIP, pnp_tool_frame  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.kinematics import ArmKinematics, frame_at  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")

# OpenCV camera axes (+Z view, +Y down) expressed in the MuJoCo camera frame
# (+Z up, -Z view, +Y up): a 180 deg rotation about local X.
CV_IN_MJ = np.diag([1.0, -1.0, -1.0, 1.0])


def median_pose(poses):
    t = np.median(np.array([p[:3, 3] for p in poses]), axis=0)
    yaws = np.array([math.atan2(p[1, 0], p[0, 0]) for p in poses])
    ref = yaws[0]
    yaws = ref + np.angle(np.exp(1j * (yaws - ref)))
    return t, float(np.median(yaws))


def estimate_board(records, cal, kin, layout, K, dist, flip=True):
    """One board pose per capture from every marker in it."""
    lens = {"intrinsics": (K[0, 0], K[1, 1], K[0, 2], K[1, 2]),
            "distortion": np.asarray(dist, float).ravel()}
    poses = []
    per_view = []
    for name, joints, image in records:
        base_camera = kin.camera_pose(np.asarray(joints, float))
        if flip:
            base_camera = base_camera @ CV_IN_MJ
        dets = fiducials.detect_markers(image)
        ids = []
        for d in dets:
            if not (5 <= d.marker_id <= 36):
                continue
            obj = np.asarray(layout.corners_mm(d.marker_id), float) / 1000.0
            obj3 = np.column_stack([obj, np.zeros(4)]).reshape(-1, 1, 3)
            img = np.asarray(d.corners, float).reshape(-1, 1, 2)
            solved = pnp_tool_frame(obj3.reshape(-1, 3), img.reshape(-1, 2), lens)
            if solved is None:
                continue
            # Back to the OpenCV camera frame this function composes in; the
            # helper returns the project's flipped frame.
            transform = solved[0]
            T = np.eye(4)
            T[:3, :3] = FLIP @ transform[:3, :3]
            T[:3, 3] = FLIP @ transform[:3, 3]
            poses.append(base_camera @ T)
            ids.append(int(d.marker_id))
        per_view.append({"ids": ids, "roll": round(float(joints[4]), 2)})
    return poses, per_view


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--baseline-dir", default=str(ROOT / "live/captures_handeye_20261121c"))
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_sanity.json"))
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    fx, fy, cx, cy = cal.intrinsics
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    stored = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(stored["x_mm"], stored["y_mm"], stored["z_mm"], stored["yaw_deg"])
    kin = ArmKinematics(cal, {}, board)
    layout = CharucoLayout(square_mm=28.0, marker_mm=20.0)

    token = (ROOT / cfg["token_file"]).read_text().strip()
    from bridge_client import HttpFrames, HttpBridge  # noqa: E402
    frames = HttpFrames(cfg["bridge_url"], token=token, timeout=12.0)
    bridge = HttpBridge(cfg["bridge_url"], token=token, timeout=6.0)

    # ---- today, current pose, no motion
    today = []
    view_rows = []
    for index in range(args.frames):
        frame = frames.capture()
        joints = tuple(float(v) for v in frame.joints_deg)
        state = bridge.state()
        cur = tuple(float(state["joints"][n]) for n in NAMES)
        drift = max(abs(a - b) for a, b in zip(cur, joints))
        today.append((f"live{index}", joints, frame.image))
        view_rows.append({"frame": index, "drift_deg": round(drift, 3),
                          "roll": joints[4], "joints": dict(zip(NAMES, joints))})
        print(f"frame {index}: drift {drift:.2f} deg  roll {joints[4]:.2f}", flush=True)
        time.sleep(0.2)
    poses_today, _ = estimate_board(today, cal, kin, layout, K, dist)
    if not poses_today:
        print("FAIL: no markers today")
        return 2
    t_today, yaw_today = median_pose(poses_today)

    # ---- baseline, the 2026-11-21 roll-68 pass, same estimator
    base_dir = Path(args.baseline_dir)
    records = []
    if base_dir.exists():
        for path in sorted(base_dir.glob("*.json")):
            rec = json.loads(path.read_text())
            img = cv2.imread(str(path.with_suffix(".jpg")))
            if img is None or "joints_deg" not in rec:
                continue
            joints = [float(rec["joints_deg"][n]) for n in NAMES]
            records.append((path.stem, joints, cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
    poses_base, base_views = estimate_board(records, cal, kin, layout, K, dist) if records else ([], [])
    out = {"schema": "phase3-sanity/1",
           "stored_board": stored,
           "today": {"x_mm": round(float(t_today[0]) * 1000, 2),
                     "y_mm": round(float(t_today[1]) * 1000, 2),
                     "z_mm": round(float(t_today[2]) * 1000, 2),
                     "yaw_deg": round(math.degrees(yaw_today), 2),
                     "markers": len(poses_today), "views": view_rows},
           "baseline": None}
    print(f"\ntoday ({len(poses_today)} marker poses, roll "
          f"{view_rows[0]['roll']:.1f}): "
          f"x {t_today[0]*1000:8.2f} y {t_today[1]*1000:8.2f} "
          f"z {t_today[2]*1000:8.2f} yaw {math.degrees(yaw_today):8.2f}")

    if poses_base:
        t_base, yaw_base = median_pose(poses_base)
        d = (t_today - t_base) * 1000.0
        dyaw = math.degrees(math.atan2(math.sin(yaw_today - yaw_base),
                                       math.cos(yaw_today - yaw_base)))
        out["baseline"] = {"dir": str(base_dir),
                           "x_mm": round(float(t_base[0]) * 1000, 2),
                           "y_mm": round(float(t_base[1]) * 1000, 2),
                           "z_mm": round(float(t_base[2]) * 1000, 2),
                           "yaw_deg": round(math.degrees(yaw_base), 2),
                           "markers": len(poses_base), "views": base_views}
        out["today_vs_baseline"] = {"dx_mm": round(float(d[0]), 2),
                                    "dy_mm": round(float(d[1]), 2),
                                    "dz_mm": round(float(d[2]), 2),
                                    "dist_mm": round(float(np.linalg.norm(d)), 2),
                                    "dyaw_deg": round(float(dyaw), 2)}
        print(f"baseline ({len(poses_base)} marker poses, "
              f"{base_views[0]['roll']:.1f}): "
              f"x {t_base[0]*1000:8.2f} y {t_base[1]*1000:8.2f} "
              f"z {t_base[2]*1000:8.2f} yaw {math.degrees(yaw_base):8.2f}")
        print(f"today - baseline: dx {d[0]:.2f} dy {d[1]:.2f} dz {d[2]:.2f} "
              f"|d| {np.linalg.norm(d):.2f} mm  dyaw {dyaw:.2f} deg")
    else:
        print("no baseline captures found")

    # offsets against the stored physical placement (bias, not movement)
    off = (t_today - np.array([stored["x_mm"], stored["y_mm"], stored["z_mm"]]) / 1000.0) * 1000
    out["today_vs_board_json"] = {"dx_mm": round(float(off[0]), 2),
                                  "dy_mm": round(float(off[1]), 2),
                                  "dz_mm": round(float(off[2]), 2),
                                  "dist_mm": round(float(np.linalg.norm(off)), 2)}
    print(f"today - board.json: dx {off[0]:.2f} dy {off[1]:.2f} dz {off[2]:.2f} "
          f"|d| {np.linalg.norm(off):.2f} mm")
    Path(args.out).write_text(json.dumps(out, indent=2) + "\n")
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
