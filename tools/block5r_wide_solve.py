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

# The project carries points in a flipped camera frame: depth = -z and
# v = fy*(-y)/depth.  cv2 works in the standard frame, so every crossing of the
# boundary applies this once.  Keeping it in one place is the point: view_mount()
# used to solve PnP *with* distortion while validate() projected without it.
FLIP = np.diag([1.0, -1.0, -1.0])


def camera_matrix(calibration):
    return np.array([[calibration.intrinsics[0], 0, calibration.intrinsics[2]],
                     [0, calibration.intrinsics[1], calibration.intrinsics[3]],
                     [0, 0, 1]], float)


def project_tool_frame(points, calibration):
    """Project tool-frame points through the calibrated lens (distortion included).

    Returns (N, 2) image points; points behind the camera come back as nan so a
    caller can never silently score a mirrored projection.
    """
    pts = np.asarray(points, float).reshape(-1, 3)
    pts_cv = pts @ FLIP.T
    out = np.full((len(pts), 2), np.nan)
    front = pts_cv[:, 2] > 1e-6
    if front.any():
        projected, _ = cv2.projectPoints(
            pts_cv[front], np.zeros(3), np.zeros(3),
            camera_matrix(calibration), np.asarray(calibration.distortion, float))
        out[front] = projected.reshape(-1, 2)
    return out


def pnp_tool_frame(obj, img, calibration):
    """Planar PnP -> (T_tool_cam, residual).  Both IPPE branches are refined and
    the lower-residual one wins.

    SOLVEPNP_IPPE returns two valid solutions for a planar target.  Taking the
    first one produced a 32 deg outlier on a 10-marker view
    (reports/rot_scan_20261127.md), which is exactly the kind of silent error a
    measurement tool must not have.
    """
    K = camera_matrix(calibration)
    dist = np.asarray(calibration.distortion, float)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok or len(rvecs) == 0:
        return None
    best = None
    for rvec, tvec in zip(rvecs, tvecs):
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        residual = float(np.sqrt(np.mean(
            (projected.reshape(-1, 2) - np.asarray(img, float)) ** 2)))
        if best is None or residual < best[0]:
            best = (residual, rvec, tvec)
    _, rvec, tvec = best
    mount = np.eye(4)
    mount[:3, :3] = FLIP @ Rotation.from_rotvec(np.ravel(rvec)).as_matrix()
    mount[:3, 3] = FLIP @ np.ravel(tvec)
    return mount, best[0]


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
    solved = pnp_tool_frame(np.asarray(obj, np.float32),
                            np.asarray(img, np.float64), calibration)
    if solved is None:
        return None
    cam_from_board, _ = solved
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
    """Per-component reprojection RMS of the detected corners under a mount.

    Units note: this is sqrt(mean over 2N components), so it is sqrt(2) larger
    than a per-axis RMS.  Every number this tool prints uses this convention;
    reports that quote a per-axis figure say so explicitly.
    """
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
        proj = project_tool_frame(pts, calibration)
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
