#!/usr/bin/env python3
"""Block 4c: re-solve the live camera mount and board pose with the measured lens.

Model, stated up front: the camera sits at a fixed unknown pose in the tool
frame (``camera_in_tool``) and the board lies flat on the table at an unknown
(x, y, z, yaw).  Every corner of every detected ChArUco marker in the 37
fixed-board ``_cal`` captures is a residual, so nothing is estimated from
relative motions (they are small on this arm and ill-conditioned).

Solving that as one 10-parameter nonlinear fit from the *stored* mount fails --
that mount was derived with a 2.66x-wrong focal length and is 150 deg out, and
the fit lands in a basin at 100+ px RMS.  So this stages it, as
``handeye_bundle.solve_staged`` does:

  1. per-view board PnP in the camera frame (needs >=2 markers: four coplanar
     points from one tag are two-fold ambiguous),
  2. bootstrap the mount from the *nominal* board placement -- the one thing a
     bench setup knows approximately -- and average the per-view mounts with a
     MAD rejection,
  3. alternate board <- mean(FK(q) X T_cam_board) and X <- mean(FK(q)^-1 board
     T_cam_board^-1) to convergence,
  4. polish X (6 DoF) and board (x, y, z, yaw) with the measured distortion in
     the projection.

Camera conventions: the project stores ``camera_in_tool`` in OpenGL camera axes
(+X right, +Y up, -Z view).  Projection here is OpenCV (+Z view), so the stored
mount is converted with ``diag(1,-1,-1)`` on the way in and the solve is
converted back on the way out.  The flip is its own inverse.

    PYTHONPATH=. robot-arm-sim python tools/block4c_mount.py \
        --intrinsics reports/block4c_intrinsics.json \
        --model all/shared_centre_pp_fixed_focal --out reports/block4c_mount.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from chess_sim import fiducials
from chess_sim.wrist_calibration import WristCalibration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handeye_bundle import _average_pose, board_corners, chain_fk  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
FIRST_ID, LAST_ID = 5, 36
FLIP = np.diag([1.0, -1.0, -1.0])          # OpenGL <-> OpenCV camera axes


def gl_to_cv(transform: np.ndarray) -> np.ndarray:
    out = np.asarray(transform, dtype=float).copy()
    out[:3, :3] = out[:3, :3] @ FLIP
    return out


cv_to_gl = gl_to_cv


def pose_from_params(params) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = Rotation.from_rotvec(params[:3]).as_matrix()
    out[:3, 3] = params[3:6]
    return out


def board_from_params(params) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("z", params[3]).as_matrix()
    out[:3, 3] = params[:3]
    return out


def board_summary(board: np.ndarray) -> dict:
    return {"x_mm": round(float(board[0, 3]) * 1000, 2),
            "y_mm": round(float(board[1, 3]) * 1000, 2),
            "z_mm": round(float(board[2, 3]) * 1000, 2),
            "yaw_deg": round(math.degrees(math.atan2(board[1, 0], board[0, 0])), 2)}


def load_views(captures: Path, calibration) -> list[dict]:
    """Fixed-board ``_cal`` frames only: the rest are other board placements."""
    views = []
    for item in sorted(captures.glob("*.json")):
        record = json.loads(item.read_text())
        if not (record.get("label") == "cal"
                or record.get("name", item.stem).endswith("_cal")):
            continue
        joints = record.get("state_joints_deg") or record.get("joints_deg")
        if not joints or len(joints) != 6:
            continue
        # Recorded corners, not a fresh detection: these are the pixels the live
        # app actually measured, and re-detecting invents a tag on one blurry
        # frame (120954128_cal recorded none, re-detection "found" id 17).
        objects, pixels = [], []
        for marker in record.get("markers", []):
            marker_id = int(marker["id"])
            if FIRST_ID <= marker_id <= LAST_ID:
                objects.append(board_corners(marker_id))
                pixels.append(np.asarray(marker["corners"], float))
        if not objects:
            continue
        views.append({"name": record.get("name", item.stem),
                      "base_tool": chain_fk(calibration, [float(joints[n]) for n in JOINTS]),
                      "objects": np.vstack(objects), "pixels": np.vstack(pixels),
                      "markers": len(objects)})
    return views


def per_view_pnp(views, camera_matrix, distortion, min_markers=2):
    for view in views:
        if view["markers"] < min_markers:
            continue
        ok, rvec, tvec = cv2.solvePnP(view["objects"].astype(np.float64),
                                      view["pixels"].astype(np.float64),
                                      camera_matrix, distortion,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        pose = np.eye(4)
        pose[:3, :3] = cv2.Rodrigues(rvec)[0]
        pose[:3, 3] = np.ravel(tvec)
        view["cam_board"] = pose
    return [v for v in views if "cam_board" in v]


def robust_average(poses, rejection=3.0):
    if not poses:
        return None, []
    reference = _average_pose(poses)
    deviations = np.array([np.linalg.norm(p[:3, 3] - reference[:3, 3]) for p in poses])
    median = float(np.median(deviations))
    mad = float(np.median(np.abs(deviations - median))) + 1e-6
    keep = [i for i, d in enumerate(deviations) if d <= median + rejection * mad]
    if len(keep) < max(3, len(poses) // 2):
        keep = list(range(len(poses)))
    return _average_pose([poses[i] for i in keep]), [i for i in range(len(poses)) if i not in keep]


def staged(views, board0, iterations=20):
    candidates = [np.linalg.inv(v["base_tool"]) @ board0 @ np.linalg.inv(v["cam_board"])
                  for v in views]
    x_pose, dropped = robust_average(candidates)
    board = board0.copy()
    for _ in range(iterations):
        board, _ = robust_average([v["base_tool"] @ x_pose @ v["cam_board"] for v in views])
        x_pose, dropped = robust_average(
            [np.linalg.inv(v["base_tool"]) @ board @ np.linalg.inv(v["cam_board"]) for v in views])
    return x_pose, board, len(dropped)


def residuals(params, views, camera_matrix, distortion):
    x_pose = pose_from_params(params[:6])
    board = board_from_params(params[6:10])
    out = []
    for view in views:
        target = np.linalg.inv(view["base_tool"] @ x_pose) @ board
        rvec = cv2.Rodrigues(target[:3, :3])[0]
        tvec = target[:3, 3].reshape(3, 1)
        projected, _ = cv2.projectPoints(view["objects"].reshape(-1, 1, 3), rvec, tvec,
                                         camera_matrix, distortion)
        out.extend((projected.reshape(-1, 2) - view["pixels"]).ravel())
    return np.asarray(out)


def evaluate(views, x_pose, board, camera_matrix, distortion):
    per_view, errors = [], []
    for view in views:
        target = np.linalg.inv(view["base_tool"] @ x_pose) @ board
        rvec = cv2.Rodrigues(target[:3, :3])[0]
        tvec = target[:3, 3].reshape(3, 1)
        projected, _ = cv2.projectPoints(view["objects"].reshape(-1, 1, 3), rvec, tvec,
                                         camera_matrix, distortion)
        distance = np.linalg.norm(projected.reshape(-1, 2) - view["pixels"], axis=1)
        errors.extend(distance.tolist())
        per_view.append({"name": view["name"], "markers": view["markers"],
                         "rms_px": round(float(np.sqrt(np.mean(distance ** 2))), 3),
                         "max_px": round(float(distance.max()), 3)})
    errors = np.asarray(errors)
    return per_view, errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--calibration", default=".private/phase11d/calibration.json")
    parser.add_argument("--intrinsics", default="reports/block4c_intrinsics.json")
    parser.add_argument("--model", default="all/shared_centre_pp_fixed_focal")
    parser.add_argument("--board", default="live/board.json")
    parser.add_argument("--out", default="reports/block4c_mount.json")
    parser.add_argument("--min-markers", type=int, default=2)
    args = parser.parse_args()

    data = json.loads(Path(args.intrinsics).read_text())
    scope, model = args.model.split("/")
    lens = data["models"][scope][model]
    fx, fy, cx, cy = lens["fx"], lens["fy"], lens["cx"], lens["cy"]
    distortion = np.asarray(lens["distortion"], dtype=float).reshape(-1, 1)
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    print(f"lens {scope}/{model}: fx {fx} fy {fy} cx {cx} cy {cy} dist {lens['distortion']}")

    calibration = WristCalibration.load(args.calibration)
    all_views = load_views(Path(args.captures), calibration)
    print(f"cal views with a detection: {len(all_views)} "
          f"({sum(v['markers'] for v in all_views)} markers)")
    fit_views = per_view_pnp(all_views, camera_matrix, distortion, args.min_markers)
    print(f"views with >={args.min_markers} markers for the mount fit: {len(fit_views)}")
    if len(fit_views) < 4:
        print("not enough views")
        return 1

    prior = json.loads(Path(args.board).read_text())
    board0 = np.eye(4)
    board0[:3, :3] = Rotation.from_euler("z", math.radians(prior["yaw_deg"])).as_matrix()
    board0[:3, 3] = [prior["x_mm"] / 1000.0, prior["y_mm"] / 1000.0, prior["z_mm"] / 1000.0]

    x_staged, board_staged, dropped = staged(fit_views, board0)
    print(f"staged: dropped {dropped} views, board {board_summary(board_staged)}")
    print("staged camera_in_tool translation mm:",
          np.round(x_staged[:3, 3] * 1000, 2))

    params0 = np.concatenate([
        Rotation.from_matrix(x_staged[:3, :3]).as_rotvec(), x_staged[:3, 3],
        board_staged[:3, 3],
        [math.atan2(board_staged[1, 0], board_staged[0, 0])]])
    result = least_squares(residuals, params0, args=(fit_views, camera_matrix, distortion),
                           loss="soft_l1", f_scale=1.5, x_scale="jac",
                           max_nfev=5000, xtol=1e-14, ftol=1e-14)
    x_pose = pose_from_params(result.x[:6])
    board = board_from_params(result.x[6:10])

    per_view, errors = evaluate(all_views, x_pose, board, camera_matrix, distortion)
    stored = np.asarray(calibration.camera_in_tool, dtype=float)
    delta = np.linalg.inv(stored) @ cv_to_gl(x_pose)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(delta[:3, :3]) - 1) / 2))))

    summary = {
        "lens": {"scope": scope, "model": model, "fx": fx, "fy": fy, "cx": cx, "cy": cy,
                 "distortion": lens["distortion"]},
        "views": len(all_views), "fit_views": len(fit_views),
        "sightings": int(sum(v["markers"] for v in all_views)),
        "camera_in_tool_cv": [[round(float(v), 6) for v in row] for row in x_pose],
        "camera_in_tool_gl": [[round(float(v), 6) for v in row] for row in cv_to_gl(x_pose)],
        "camera_translation_mm": [round(float(v) * 1000, 2) for v in x_pose[:3, 3]],
        "board": board_summary(board),
        "delta_vs_stored": {
            "translation_delta_mm": round(float(np.linalg.norm(delta[:3, 3]) * 1000), 2),
            "translation_vector_mm": [round(float(v) * 1000, 2) for v in delta[:3, 3]],
            "rotation_delta_deg": round(angle, 2),
            "camera_displacement_mm": round(float(np.linalg.norm(
                cv_to_gl(x_pose)[:3, 3] - stored[:3, 3]) * 1000), 2)},
        "board_delta_vs_live": {k: round(summary_board - float(prior[k]), 2)
                                for k, summary_board in board_summary(board).items()},
        "corner_error_px": {"median": round(float(np.median(errors)), 3),
                            "p90": round(float(np.percentile(errors, 90)), 3),
                            "max": round(float(errors.max()), 3),
                            "count": int(errors.size)},
        "rms_px": round(float(np.sqrt(np.mean(errors ** 2))), 3),
        "per_view": sorted(per_view, key=lambda r: -r["rms_px"]),
        "stored_camera_in_tool": [[round(float(v), 6) for v in row] for row in stored],
        "prior_board": prior,
        "optimizer": {"cost": round(float(result.cost), 3), "nfev": int(result.nfev),
                      "status": int(result.status)},
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(summary, indent=2) + "\n")

    print("camera_in_tool (OpenGL, for calibration.json):")
    print(np.round(np.asarray(summary["camera_in_tool_gl"]), 6))
    print("board:", summary["board"], " delta vs live:", summary["board_delta_vs_live"])
    print("mount delta vs stored:", summary["delta_vs_stored"])
    print("reprojection:", summary["corner_error_px"], "rms", summary["rms_px"])
    print("worst views:", summary["per_view"][:4])
    print("->", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
