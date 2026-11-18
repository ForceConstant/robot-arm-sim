#!/usr/bin/env python3
"""Block 5r: solve camera_in_tool over the wide volume, validated on held-out views.

Uses the wide-volume captures (tools/block5q_wide_cal.py --execute), the fixed
4c intrinsics and the known board pose.  For each view:

    T_cam_board   <- planar PnP of the detected ChArUco tags (board frame)
    T_base_flange <- FK at the measured joints (no camera mount involved)
    T_flange_cam  = inv(T_base_flange) @ T_base_board @ inv(T_cam_board)

The mount is the rotation mean / translation median over the *solve* split; the
*held-out* split then reports reprojection RMS and the per-view board-pose
spread, which is the honest residual (the board does not move).

    robot-arm-sim python tools/block5r_wide_solve.py \
        --captures live/captures/wide_5q --out reports/block5r_mount.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))
sys.path.insert(0, str(ROOT / "tools"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from chess_sim.marker_scan import marker_corners_world  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from solve_wrist_live import flange_pose  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
BOARD = None
LAYOUT = None


def load(captures: Path):
    views = []
    for meta in sorted(captures.glob("*.json")):
        rec = json.loads(meta.read_text())
        img_path = meta.with_suffix(".jpg")
        if not img_path.exists():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        dets = fiducials.detect_markers(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        if len(dets) < 3:
            continue
        views.append({
            "name": meta.stem, "image": img, "detections": dets,
            "joints_deg": [float(rec["measured_deg"][j]) for j in JOINTS],
            "move_error": rec.get("move_error_deg"),
        })
    return views


def board_matrix():
    yaw = math.radians(BOARD["yaw_deg"])
    M = np.eye(4)
    M[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    M[:3, 3] = [BOARD["x_mm"] / 1000, BOARD["y_mm"] / 1000, BOARD["z_mm"] / 1000]
    return M


def view_mount(view, calibration, board_from_base):
    """T_flange_cam.  Returns None if PnP fails."""
    K = np.array([[calibration.intrinsics[0], 0, calibration.intrinsics[2]],
                  [0, calibration.intrinsics[1], calibration.intrinsics[3]],
                  [0, 0, 1]], float)
    dist = np.asarray(calibration.distortion, float)
    obj, img = [], []
    for det in view["detections"]:
        try:
            corners = marker_corners_world(LAYOUT, det.marker_id, origin_xy=(0.0, 0.0),
                                           yaw=0.0, marker_z=0.0)
        except (ValueError, KeyError):
            continue
        obj.extend(np.asarray(corners, float))
        img.extend(np.asarray(det.corners, float))
    if len(obj) < 12:
        return None
    obj = np.asarray(obj, np.float32); img = np.asarray(img, np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    rot, _ = cv2.Rodrigues(rvec)
    flip = np.diag([1.0, -1.0, -1.0])
    cam_from_board = np.eye(4)
    cam_from_board[:3, :3] = flip @ rot
    cam_from_board[:3, 3] = flip @ np.ravel(tvec)
    joints = np.radians(view["joints_deg"])
    base_from_flange = flange_pose(calibration, joints)
    base_from_cam = board_from_base @ np.linalg.inv(cam_from_board)
    return np.linalg.inv(base_from_flange) @ base_from_cam


def average(mounts):
    rots = Rotation.from_matrix([m[:3, :3] for m in mounts])
    mean = rots.mean().as_matrix()
    trans = np.median(np.array([m[:3, 3] for m in mounts]), axis=0)
    out = np.eye(4)
    out[:3, :3] = mean
    out[:3, 3] = trans
    return out


def validate(views, calibration, board_from_base, mount):
    """Reprojection RMS of the detected corners under a given mount."""
    K = np.array([[calibration.intrinsics[0], 0, calibration.intrinsics[2]],
                  [0, calibration.intrinsics[1], calibration.intrinsics[3]],
                  [0, 0, 1]], float)
    out = []
    for view in views:
        obj, img = [], []
        for det in view["detections"]:
            try:
                corners = marker_corners_world(LAYOUT, det.marker_id, origin_xy=(0.0, 0.0),
                                               yaw=0.0, marker_z=0.0)
            except (ValueError, KeyError):
                continue
            obj.extend(np.asarray(corners, float))
            img.extend(np.asarray(det.corners, float))
        if len(obj) < 12:
            continue
        base_from_flange = flange_pose(calibration, np.radians(view["joints_deg"]))
        cam_from_base = np.linalg.inv(base_from_flange @ mount)
        pts = (cam_from_base @ board_from_base @ np.c_[np.asarray(obj), np.ones(len(obj))].T).T[:, :3]
        proj = []
        for p in pts:
            depth = -p[2]
            if depth <= 1e-6:
                proj.append([np.nan, np.nan]); continue
            proj.append([K[0, 0] * p[0] / depth + K[0, 2],
                         K[1, 1] * (-p[1]) / depth + K[1, 2]])
        err = np.asarray(proj) - np.asarray(img, float)
        out.append(float(np.sqrt(np.nanmean(err ** 2))))
    return np.asarray(out)


def main() -> int:
    global BOARD, LAYOUT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--captures", default="live/captures/wide_5q")
    ap.add_argument("--calibration", default=".private/phase11d/calibration.json")
    ap.add_argument("--out", default="reports/block5r_mount.json")
    args = ap.parse_args()

    cal = WristCalibration.load(str(ROOT / args.calibration))
    BOARD = json.loads((ROOT / "live/board.json").read_text())
    LAYOUT = CharucoLayout(square_mm=28.0)
    board_from_base = board_matrix()

    views = load(ROOT / args.captures)
    print(f"views with >=3 tags: {len(views)}")
    if len(views) < 8:
        print("not enough views to solve; stopping")
        return 1

    solve_views = [v for i, v in enumerate(views) if i % 2 == 0]
    held_views = [v for i, v in enumerate(views) if i % 2 == 1]
    mounts = [m for m in (view_mount(v, cal, board_from_base) for v in solve_views) if m is not None]
    print(f"solve split {len(mounts)}/{len(solve_views)} usable, held-out {len(held_views)}")
    if len(mounts) < 4:
        print("too few usable solve views; stopping")
        return 1
    mount = average(mounts)

    cur = np.asarray(cal.camera_in_tool, float)
    print("candidate translation mm:", np.round(mount[:3, 3] * 1000, 2))
    print("current   translation mm:", np.round(cur[:3, 3] * 1000, 2))
    d = mount[:3, 3] - cur[:3, 3]
    print("delta mm:", np.round(d * 1000, 2), "|d| =", round(float(np.linalg.norm(d)) * 1000, 1))
    angle = Rotation.from_matrix(mount[:3, :3] @ cur[:3, :3].T).magnitude()
    print("rotation delta:", round(math.degrees(float(angle)), 2), "deg")

    rms = validate(held_views, cal, board_from_base, mount)
    print("held-out reprojection rms: median", round(float(np.median(rms)), 2),
          "max", round(float(np.max(rms)), 2), "n", len(rms))
    rms_cur = validate(held_views, cal, board_from_base, cur)
    print("held-out rms with the CURRENT mount: median",
          round(float(np.median(rms_cur)), 2), "max", round(float(np.max(rms_cur)), 2))

    out = {"schema": "block5r/mount/1",
           "candidate": mount.tolist(), "current": cur.tolist(),
           "delta_mm": (d * 1000).tolist(),
           "rotation_delta_deg": round(math.degrees(float(angle)), 3),
           "held_out_candidate": {"median_px": float(np.median(rms)),
                                  "max_px": float(np.max(rms)), "n": int(len(rms))},
           "held_out_current": {"median_px": float(np.median(rms_cur)),
                                "max_px": float(np.max(rms_cur))}}
    (ROOT / args.out).write_text(json.dumps(out, indent=2) + "\n")
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
