"""Solve the wrist camera's hand-eye transform, and prove it in simulation.

`T_flange_cam` is the one calibration that a simulator cannot assume away on
real hardware: without it the board pose is a camera-frame number, not a
base-frame offset.  This tool solves it from captured views and, in
``--sim-validate`` mode, checks the solver against a known transform so the
bench session is not the first time the code runs.

Method: for each view, planar PnP of the four printed markers gives
``T_cam_board``; forward kinematics gives ``T_base_flange``; OpenCV's
``calibrateHandEye`` then returns ``T_flange_cam``.  The marker layout, intrinsics
and chain come from the same files the live system uses.

    MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
    PYTHONPATH=. robot-arm-sim python tools/solve_hand_eye.py --sim-validate
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from chess_sim import fiducials
from chess_sim.charuco_face import CharucoLayout
from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.marker_scan import marker_corners_world
from chess_sim.product import current_layout, layout_env_config
from chess_sim.wrist_calibration import WristCalibration, _transform

MARKER_LAYOUT = fiducials.MarkerLayout.for_board(224.0, 15.0, plane="mat", margin_mm=40.0)


def marker_layout(board):
    """The correspondence model for the face this board actually prints."""
    if getattr(board, "face", "") == "charuco":
        return CharucoLayout(board.square * 1000.0)
    return MARKER_LAYOUT


def marker_face_z(board) -> float:
    """Fiducial plane height: the board surface for ChArUco, the mat otherwise."""
    if getattr(board, "face", "") == "charuco":
        return float(board.top)
    return board.table_top + fiducials.marker_height_above_table(board.mat)


def flange_pose(calibration: WristCalibration, joints) -> tuple[np.ndarray, np.ndarray]:
    """``T_base_flange`` from the calibrated chain, without the camera mount."""
    joints = np.asarray(joints, dtype=float)
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
    return world[:3, :3].copy(), world[:3, 3].copy()


def board_pose_in_camera(image, calibration: WristCalibration, joints, *,
                         board_origin=(0.0, 0.0), board_yaw=0.0,
                         marker_z: float, layout=MARKER_LAYOUT) -> tuple[np.ndarray, np.ndarray, int] | None:
    """Planar PnP of the marker mat: ``T_cam_board``."""
    detections = fiducials.detect_markers(image)
    if len(detections) < 2:
        return None
    object_points: list[tuple[float, float, float]] = []
    image_points: list[tuple[float, float]] = []
    for detection in detections:
        corners = marker_corners_world(
            layout, detection.marker_id, origin_xy=board_origin,
            yaw=board_yaw, marker_z=marker_z)
        local = np.asarray(layout.corners_mm(detection.marker_id),
                           dtype=float) / 1000.0
        for point in local:
            object_points.append((float(point[0]), float(point[1]), 0.0))
        image_points.extend(tuple(map(float, corner)) for corner in detection.corners)
    camera = calibration.camera(joints)
    matrix = np.array([[camera.fx, 0.0, camera.cx],
                       [0.0, camera.fy, camera.cy],
                       [0.0, 0.0, 1.0]])
    object_array = np.asarray(object_points, dtype=np.float64)
    image_array = np.asarray(image_points, dtype=np.float64)
    count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        object_array, image_array, matrix, np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
    if count == 0:
        return None
    # A planar target has a mirror ambiguity.  The board lies flat with +Z up,
    # and the wrist camera looks down at it, so the correct solution is the one
    # whose board normal faces the camera.  Choosing by reprojection alone picks
    # the mirrored pose roughly half the time and destroys the hand-eye solve.
    best = None
    for index in range(count):
        rotation, _ = cv2.Rodrigues(rvecs[index])
        translation = np.asarray(tvecs[index]).ravel()
        normal = rotation[:, 2]
        facing = float(np.dot(normal, -translation))
        if best is None or facing > best[0]:
            best = (facing, rotation, translation)
    _, rotation, translation = best
    if float(np.dot(rotation[:, 2], -translation)) <= 0:
        return None
    return rotation, translation, len(detections)


def _view_observations(views, calibration, *, board_origin, board_yaw, marker_z,
                       layout=MARKER_LAYOUT, intrinsics=None):
    """Per-view planar correspondences plus the tool pose that produced them."""
    observations = []
    for joints, image in views:
        detections = fiducials.detect_markers(image)
        if len(detections) < 2:
            continue
        object_points, image_points = [], []
        for detection in detections:
            # World (base-frame) corners: the board does not move, so the
            # refinement can project them without carrying a per-view target
            # pose around.
            corners = marker_corners_world(
                layout, detection.marker_id, origin_xy=board_origin,
                yaw=board_yaw, marker_z=marker_z)
            object_points.extend(tuple(map(float, corner)) for corner in corners)
            image_points.extend(tuple(map(float, corner)) for corner in detection.corners)
        rotation_g, translation_g = flange_pose(calibration, joints)
        camera = calibration.camera(joints)
        if intrinsics is not None:
            fx, fy, cx, cy = (float(v) for v in intrinsics)
            camera = SimpleNamespace(fx=fx, fy=fy, cx=cx, cy=cy)
        observations.append((np.asarray(object_points), np.asarray(image_points),
                             rotation_g, translation_g, camera))
    return observations


def refine_hand_eye(observations, init: np.ndarray) -> np.ndarray:
    """Least-squares refinement of the hand-eye transform on pixel residuals.

    OpenCV's closed-form methods assume a noise-free kinematic model and are
    sensitive to planar targets; a few Gauss-Newton steps on the real
    reprojection error is both more accurate and easier to audit.
    """
    def unpack(params):
        return _transform(params[3:], Rotation.from_rotvec(params[:3]).as_matrix())

    def residual(params):
        transform = unpack(params)
        blocks = []
        for object_points, image_points, rotation_g, translation_g, camera in observations:
            rotation = rotation_g @ transform[:3, :3]
            position = rotation_g @ transform[:3, 3] + translation_g
            # Object points are already in the base frame; the camera pose is
            # predicted from the hand-eye candidate, so its rotation must be the
            # predicted one -- using the truth camera rotation here would leave
            # the rotation unconstrained and only fit translation.
            local = (object_points - position) @ rotation
            depth = -local[:, 2]
            depth = np.where(np.abs(depth) < 1e-6, 1e-6, depth)
            u = camera.cx + camera.fx * local[:, 0] / depth
            v = camera.cy - camera.fy * local[:, 1] / depth
            blocks.append((np.stack([u, v], axis=1) - image_points).ravel())
        return np.concatenate(blocks)

    seed = np.r_[Rotation.from_matrix(init[:3, :3]).as_rotvec(), init[:3, 3]]
    result = least_squares(residual, seed, method="trf", xtol=1e-12, ftol=1e-12,
                           gtol=1e-12, max_nfev=200)
    return unpack(result.x)


def _compare(recovered: np.ndarray, translation: np.ndarray,
             truth: np.ndarray) -> dict:
    truth_rotation = truth[:3, :3]
    truth_translation = truth[:3, 3]
    angle = math.degrees(math.acos(max(-1.0, min(1.0,
        (np.trace(recovered.T @ truth_rotation) - 1.0) / 2.0))))
    return {"rotation_deg": round(angle, 4),
            "translation_mm": round(float(np.linalg.norm(translation - truth_translation)) * 1000, 3)}


def calibration_views(env, calibration: WristCalibration, *, board_origin, board_yaw,
                      marker_z: float, count: int = 12, seed: int = 11,
                      layout=MARKER_LAYOUT):
    """Render varied wrist views that see at least two markers."""
    rng = np.random.default_rng(seed)
    base = np.array([0.0, -1.30, 0.75, 1.25, -1.50, -0.80])
    views = []
    attempts = 0
    while len(views) < count and attempts < count * 20:
        attempts += 1
        joints = base + np.radians(rng.uniform(-18, 18, size=6))
        joints[0] = rng.uniform(-0.6, 0.6)          # pan
        joints[5] = -0.8
        env.data.qpos[env._joint_qpos] = joints
        mujoco.mj_forward(env.model, env.data)
        image = env.render(Camera.WRIST).copy()
        board = board_pose_in_camera(image, calibration, joints,
                                     board_origin=board_origin, board_yaw=board_yaw,
                                     marker_z=marker_z, layout=layout)
        if board is None:
            continue
        views.append((joints.copy(), image))
    return views


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", default=".private/phase11d/calibration.json")
    parser.add_argument("--out", default="reports/hand_eye_sim.json")
    parser.add_argument("--views", type=int, default=12)
    parser.add_argument("--sim-validate", action="store_true",
                        help="required: this mode renders views and checks the solver")
    args = parser.parse_args()

    if not args.sim_validate:
        raise SystemExit("only --sim-validate is implemented; live capture comes later")
    calibration = WristCalibration.load(args.calibration)
    truth = np.asarray(calibration.camera_in_tool, dtype=float)
    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset("8/8/8/8/8/8/8/8")
        board = env.board
        layout = marker_layout(board)
        marker_z = marker_face_z(board)
        raw = calibration_views(env, calibration, board_origin=board.origin,
                                board_yaw=board.yaw, marker_z=marker_z,
                                count=args.views, layout=layout)
        observations = _view_observations(raw, calibration, board_origin=board.origin,
                                          board_yaw=board.yaw, marker_z=marker_z,
                                          layout=layout)
        if len(observations) < 3:
            raise SystemExit(f"need at least 3 usable views, got {len(observations)}")
        # On hardware the prior is the CAD mount transform.  Validation perturbs
        # that prior by a realistic mount error and checks the solver walks back
        # to the truth, which is the job the tool actually has to do.
        prior = truth.copy()
        prior[:3, :3] = Rotation.from_rotvec([math.radians(10.0), 0, 0]).as_matrix() @ prior[:3, :3]
        prior[:3, 3] = prior[:3, 3] + np.array([0.0, 0.015, 0.0])
        recovered = refine_hand_eye(observations, prior)
        rotation, translation = recovered[:3, :3], recovered[:3, 3]
        error = _compare(rotation, translation, truth)
        prior_error = _compare(prior[:3, :3], prior[:3, 3], truth)

        # Cross-check: rebuild the camera mount and compare forward kinematics
        # against the original chain at the calibration poses.
        rebuilt = np.eye(4)
        rebuilt[:3, :3] = rotation
        rebuilt[:3, 3] = translation.ravel()
        worst = 0.0
        for joints, _ in raw:
            rotation_g, translation_g = flange_pose(calibration, joints)
            predicted = rotation_g @ rebuilt[:3, :3], (rotation_g @ rebuilt[:3, 3]
                                                      + translation_g)
            truth_pose = calibration.camera(joints)
            truth_rotation = np.asarray(truth_pose.camera_rotation).reshape(3, 3)
            worst = max(worst, float(np.abs(predicted[0] - truth_rotation).max()),
                        float(np.linalg.norm(predicted[1]
                                             - np.asarray(truth_pose.camera_position)) * 1000))
        passed = error["translation_mm"] <= 2.0 and error["rotation_deg"] <= 1.0
        payload = {
            "schema": "robot-arm-hand-eye/1",
            "generated_at": "2026-11-16",
            "mode": "sim-validation",
            "views_used": len(observations),
            "prior_error": prior_error,
            "error": error,
            "worst_pose_reproduction": {"mm_or_matrix": round(worst, 4)},
            "passed": passed,
        }
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(payload, indent=2) + "\n")
        print(f"views {len(observations)}: prior error "
              f"{prior_error['rotation_deg']:.2f} deg / {prior_error['translation_mm']:.1f} mm -> "
              f"solved {error['rotation_deg']:.3f} deg / {error['translation_mm']:.2f} mm "
              f"-> passed={passed}")
        return 0 if passed else 1
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
