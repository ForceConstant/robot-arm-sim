#!/usr/bin/env python3
"""Estimate the board's pose in the arm base frame from marker sightings.

Works on the same capture pairs the live app writes (``live/captures/*.json``
plus ``.jpg``), and on the ChArUco face as well as the older mat tags:

    robot-arm-sim python tools/calibrate_board_charuco.py \
        --layout charuco --captures live/captures --out reports/board_charuco.json

Method, per capture:

1. PnP against each marker's *board-frame* corners gives the camera pose in a
   board whose origin is the marker layout origin.
2. Forward kinematics plus the calibrated hand-eye mount gives the camera pose
   in the arm base frame.
3. Composing the two yields one estimate of the board's pose in the base frame.

Estimates are collected across captures and reported as a median with the
spread that actually matters: per-axis millimetres, and the YAML/yaw degrees.
The spread is the honest error -- nothing here is ground truth, and a tight
spread is what licenses using the number for picking.

A marker whose reprojection error is poor is down-weighted rather than dropped
silently, so a bad view cannot masquerade as a confident result.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from chess_sim import fiducials
from chess_sim.marker_scan import MarkerLayout, marker_corners_world
from chess_sim.wrist_calibration import WristCalibration

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")

# Marker ids per layout.  ChArUco face uses 5..36; the mat/centre set uses 0..4.
LAYOUTS = {
    "charuco": dict(offset_mm=148.0, size_mm=20.0, ids=range(5, 37)),
    "mat": dict(offset_mm=152.0, size_mm=40.0, ids=range(0, 5)),
}


def flange_pose(calibration, joints_rad):
    from scipy.spatial.transform import Rotation
    from chess_sim.wrist_calibration import _transform

    joints = np.asarray(joints_rad, dtype=float)
    world = np.eye(4)
    for link in calibration.chain:
        world = world @ _transform(link["position"], link["rotation"])
        if "joint_index" not in link:
            continue
        angle = joints[int(link["joint_index"])]
        anchor = np.asarray(link["anchor"], dtype=float)
        rotation = Rotation.from_rotvec(
            np.asarray(link["axis"], dtype=float)
            * (angle - float(link["reference"]))).as_matrix()
        world = world @ _transform(anchor - rotation @ anchor, rotation)
    return world


def layout_for(name: str) -> MarkerLayout:
    spec = LAYOUTS[name]
    return MarkerLayout(offset_mm=spec["offset_mm"], size_mm=spec["size_mm"])


def estimate(captures: Path, calibration: WristCalibration, layout_name: str,
             max_reproj_px: float = 2.5) -> dict:
    layout = layout_for(layout_name)
    allowed = set(LAYOUTS[layout_name]["ids"])
    fx, fy, cx, cy = calibration.intrinsics
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    estimates = []
    rejected = []
    for record_path in sorted(captures.glob("*.json")):
        record = json.loads(record_path.read_text())
        joints = record.get("state_joints_deg") or record.get("joints_deg") or {}
        if len(joints) != 6:
            continue
        image_path = record_path.with_suffix(".jpg")
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        detections = fiducials.detect_markers(frame)
        values = [float(joints[name]) for name in JOINTS]
        base_camera = flange_pose(calibration, np.radians(values)) @ \
            np.asarray(calibration.camera_in_tool)
        for detection in detections:
            if detection.marker_id not in allowed:
                continue
            try:
                corners = np.asarray(marker_corners_world(
                    layout, detection.marker_id, origin_xy=(0, 0),
                    yaw=0, marker_z=0), dtype=float)
            except (ValueError, KeyError):
                continue
            ok, rvec, tvec = cv2.solvePnP(
                corners, np.asarray(detection.corners, dtype=float),
                camera_matrix, None, flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                continue
            projected, _ = cv2.projectPoints(corners, rvec, tvec, camera_matrix, None)
            error = float(np.mean(np.linalg.norm(
                projected.reshape(-1, 2) - np.asarray(detection.corners, float), axis=1)))
            if error > max_reproj_px:
                rejected.append((record_path.stem, detection.marker_id, round(error, 2)))
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            board_camera = np.eye(4)
            board_camera[:3, :3] = rotation
            board_camera[:3, 3] = np.ravel(tvec)
            board = base_camera @ np.linalg.inv(board_camera)
            estimates.append({
                "capture": record_path.stem,
                "marker_id": detection.marker_id,
                "reproj_px": round(error, 2),
                "x_mm": float(board[0, 3]) * 1000.0,
                "y_mm": float(board[1, 3]) * 1000.0,
                "z_mm": float(board[2, 3]) * 1000.0,
                "yaw_deg": math.degrees(math.atan2(board[1, 0], board[0, 0])),
            })
    if not estimates:
        return {"error": "no usable marker sightings",
                "rejected": rejected[:10]}
    keys = ("x_mm", "y_mm", "z_mm", "yaw_deg")
    summary = {}
    for key in keys:
        column = np.array([e[key] for e in estimates])
        # yaw lives on a circle: unwrap about the median before spreading
        if key == "yaw_deg":
            reference = np.median(column)
            column = reference + ((column - reference + 180.0) % 360.0) - 180.0
        summary[key] = {
            "median": round(float(np.median(column)), 2),
            "mean": round(float(np.mean(column)), 2),
            "spread_mm": round(float(np.percentile(column, 84) - np.percentile(column, 16)), 2),
            "min": round(float(column.min()), 2),
            "max": round(float(column.max()), 2),
        }
    worst = max(summary[key]["spread_mm"] for key in keys[:3])
    return {"layout": layout_name, "sightings": len(estimates),
            "captures": len({e["capture"] for e in estimates}),
            "board": {key: summary[key]["median"] for key in keys},
            "spread": summary,
            "worst_axis_spread_mm": round(worst, 2),
            "rejected": rejected[:10],
            "per_marker": estimates}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--calibration", default=".private/phase11d/calibration.json")
    parser.add_argument("--layout", choices=sorted(LAYOUTS), default="charuco")
    parser.add_argument("--out", default="reports/board_charuco.json")
    parser.add_argument("--max-reproj-px", type=float, default=2.5)
    args = parser.parse_args()

    calibration = WristCalibration.load(args.calibration)
    result = estimate(Path(args.captures), calibration, args.layout, args.max_reproj_px)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n")
    if "error" in result:
        print("FAILED:", result["error"])
        print("rejected:", result["rejected"])
        return
    print(f"{result['sightings']} sightings over {result['captures']} captures")
    for key, value in result["board"].items():
        spread = result["spread"][key]["spread_mm"]
        print(f"  {key:9s} {value:9.2f}   spread {spread:7.2f}")
    print(f"worst axis spread {result['worst_axis_spread_mm']} mm -> {args.out}")


if __name__ == "__main__":
    main()
