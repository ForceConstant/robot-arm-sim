"""Fit wrist-camera intrinsics and the hand-eye transform from live captures.

Reads the JSON+JPEG pairs written by the focus page's *Save current* button
(``live/captures``), where every record carries the measured joints and every
image is the same frame the detector saw.

    PYTHONPATH=. robot-arm-sim python tools/solve_wrist_live.py \
        --calibration .private/phase11d/calibration.json \
        --captures live/captures --out reports/wrist_live.json

Two stages:

1. ``cv2.calibrateCamera`` over all views gives fx, fy, cx, cy and Brown-Conrady
   distortion.  A single 40 mm marker is four points, so many views are needed
   and image-position variety matters more than view count.
2. Per-view planar PnP gives ``T_cam_mat``; forward kinematics gives
   ``T_base_flange``; ``cv2.calibrateHandEye`` returns ``T_flange_cam``.

The check that matters: the board does not move during the capture set, so
``T_base_mat = T_base_cam @ T_cam_mat`` must come out the same for every view.
Its spread across views is the honest residual -- no ground truth required.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from chess_sim import fiducials
from chess_sim.marker_scan import marker_corners_world
from chess_sim.wrist_calibration import WristCalibration, _transform

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
LAYOUT = fiducials.MarkerLayout.for_board(224.0, 15.0, plane="mat", margin_mm=40.0)


def flange_pose(calibration: WristCalibration, joints_rad) -> np.ndarray:
    """``T_base_flange`` from the calibrated chain, without the camera mount."""
    joints = np.asarray(joints_rad, dtype=float)
    world = np.eye(4)
    for link in calibration.chain:
        world = world @ _transform(link["position"], link["rotation"])
        if "joint_index" not in link:
            continue
        angle = joints[link["joint_index"]]
        anchor = np.asarray(link["anchor"])
        rotation = Rotation.from_rotvec(
            np.asarray(link["axis"]) * (angle - link["reference"])).as_matrix()
        world = world @ _transform(anchor - rotation @ anchor, rotation)
    return world


def load_views(captures: Path):
    views = []
    for meta_path in sorted(captures.glob("*.json")):
        record = json.loads(meta_path.read_text())
        if record.get("duplicate_of") or not record.get("markers"):
            continue
        image_path = meta_path.with_suffix(".jpg")
        if not image_path.exists():
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        views.append({
            "name": record.get("name", meta_path.stem),
            "image": image,
            "image_points": [np.asarray(m["corners"], np.float32)
                             for m in record["markers"]],
            "ids": [m["id"] for m in record["markers"]],
            "joints_deg": np.array([record["state_joints_deg"][j] for j in JOINTS]),
        })
    return views


def mat_points(marker_id: int) -> np.ndarray:
    corners = marker_corners_world(LAYOUT, marker_id, origin_xy=(0.0, 0.0),
                                   yaw=0.0, marker_z=0.0)
    return np.asarray(corners, dtype=np.float32)


def fit_intrinsics(views, size):
    obj_points, img_points = [], []
    for view in views:
        for marker_id, corners in zip(view["ids"], view["image_points"]):
            obj_points.append(np.ascontiguousarray(mat_points(marker_id), np.float32))
            img_points.append(np.ascontiguousarray(corners, np.float32))
    flags = cv2.CALIB_FIX_K3
    rms, matrix, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, size, None, None, flags=flags)
    return rms, matrix, dist.ravel()


def pose_from_pnp(view, matrix, dist):
    obj = np.vstack([mat_points(i) for i in view["ids"]])
    img = np.vstack(view["image_points"])
    count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj, img, matrix, dist, flags=cv2.SOLVEPNP_IPPE)
    if count == 0:
        return None
    best = None
    for index in range(count):
        rotation, _ = cv2.Rodrigues(rvecs[index])
        translation = np.asarray(tvecs[index]).ravel()
        facing = float(np.dot(rotation[:, 2], -translation))
        if best is None or facing > best[0]:
            best = (facing, rotation, translation)
    _, rotation, translation = best
    if float(np.dot(rotation[:, 2], -translation)) <= 0:
        return None
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", default=".private/phase11d/calibration.json")
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--out", default="reports/wrist_live.json")
    args = parser.parse_args()

    calibration = WristCalibration.load(args.calibration)
    views = load_views(Path(args.captures))
    if len(views) < 6:
        raise SystemExit(f"need at least 6 usable views, got {len(views)}")
    size = (views[0]["image"].shape[1], views[0]["image"].shape[0])

    rms, matrix, dist = fit_intrinsics(views, size)
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    print(f"views {len(views)}  rms {rms:.3f} px  fx {fx:.1f} fy {fy:.1f} "
          f"cx {matrix[0,2]:.1f} cy {matrix[1,2]:.1f}")
    print("distortion", np.round(dist, 5).tolist())

    rotation_g, translation_g = [], []
    rotation_t, translation_t = [], []
    used = []
    for view in views:
        board = pose_from_pnp(view, matrix, dist)
        if board is None:
            continue
        tool = flange_pose(calibration, np.radians(view["joints_deg"]))
        rotation_g.append(tool[:3, :3])
        translation_g.append(tool[:3, 3].reshape(3, 1))
        rotation_t.append(board[:3, :3])
        translation_t.append(board[:3, 3].reshape(3, 1))
        used.append((view, board, tool))
    if len(used) < 4:
        raise SystemExit(f"need at least 4 posed views, got {len(used)}")

    rotation_x, translation_x = cv2.calibrateHandEye(
        rotation_g, translation_g, rotation_t, translation_t,
        method=cv2.CALIB_HAND_EYE_PARK)
    transform = np.eye(4)
    transform[:3, :3] = rotation_x
    transform[:3, 3] = translation_x.ravel()

    prior = np.asarray(calibration.camera_in_tool, np.float64)
    prior_angle = math.degrees(math.acos(max(-1.0, min(1.0,
        (np.trace(transform[:3, :3].T @ prior[:3, :3]) - 1.0) / 2.0))))
    prior_mm = float(np.linalg.norm(transform[:3, 3] - prior[:3, 3]) * 1000)

    # The board is static: rebuilt board poses must agree across views.
    boards = []
    for _, board, tool in used:
        camera = tool @ transform
        boards.append(camera @ board)
    positions = np.array([b[:3, 3] for b in boards])
    rotations = Rotation.from_matrix(np.array([b[:3, :3] for b in boards]))
    centre = positions.mean(axis=0)
    reference = rotations.mean()
    spread_mm = float(np.linalg.norm(positions - centre, axis=1).max() * 1000)
    spread_deg = float(np.degrees((reference.inv() * rotations).magnitude()).max())
    print(f"hand-eye solved: prior offset {prior_angle:.2f} deg / {prior_mm:.1f} mm")
    print(f"board spread across {len(boards)} views: {spread_mm:.2f} mm / {spread_deg:.2f} deg")

    payload = {
        "schema": "robot-arm-wrist-live/1",
        "views": len(used),
        "intrinsics": {"fx": fx, "fy": fy,
                       "cx": float(matrix[0, 2]), "cy": float(matrix[1, 2]),
                       "width": size[0], "height": size[1],
                       "distortion": dist.tolist(), "rms_px": round(float(rms), 4)},
        "hand_eye": {"rotation": transform[:3, :3].tolist(),
                     "translation_m": transform[:3, 3].tolist()},
        "prior_offset": {"rotation_deg": round(prior_angle, 3),
                         "translation_mm": round(prior_mm, 3)},
        "board_spread": {"translation_mm": round(spread_mm, 3),
                         "rotation_deg": round(spread_deg, 3)},
        "views_detail": [{"name": v["name"], "markers": v["ids"]} for v, _, _ in used],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=2) + "\n")
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
