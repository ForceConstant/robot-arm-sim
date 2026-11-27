#!/usr/bin/env python3
"""Block 3 — recover known board placements through the three estimators.

Runs the production marker plan and the printed ChArUco face in the product
simulator, with the board *physically* moved (mocap body, x/y/z and yaw), and
compares:

  a. the multi-pose ``ScanController`` fusion used by ``/scan``
  b. the single-marker PnP maths used by ``/board/derive``
  c. the offline per-marker median in ``tools/calibrate_board_charuco.py``

Truth is the transform applied to the board body, never a re-detected value.
Run on the BC-250:  robot-arm-sim python tools/block3_placement_matrix.py
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import mujoco
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))
sys.path.insert(0, str(ROOT / "tools"))

import calibrate_board_charuco as offline_tool  # noqa: E402
from chess_sim import fiducials  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from chess_sim.config import Camera  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.marker_scan import (MarkerLayout, MarkerScanPlan,  # noqa: E402
                                   PoseCoverage, board_offset, load_plan,
                                   marker_corners_world)
from chess_sim.projection import pnp_tool_frame, project_tool_frame  # noqa: E402
from chess_sim.product import current_layout, layout_env_config  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from scan_controller import Frame, ScanController, plan_in_degrees  # noqa: E402

# The 2026-11-17 live file now holds the *real* hand-eye mount (and the old
# 312.8 px intrinsics), while the scene's wrist camera is the 36 deg / 738.6 px
# lens.  For a sim proof use the calibration exported from the model itself:
#   robot-arm-sim python tools/export_wrist_calibration.py \
#       --out reports/block3_sim_calibration.json
CALIBRATION = ROOT / "reports/block3_sim_calibration.json"
PLAN = ROOT / "chess_sim/assets/marker_scan_plan.json"
NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
SQUARE_MM = 28.0
MARKER_MM = 20.0
# The live config's marker height is the mat plane (0.5 mm above the table).
# The printed ChArUco face is the playing surface (3.7 mm above the table).
STALE_MARKER_Z_M = 0.43056


@dataclass(frozen=True)
class Placement:
    name: str
    dx_mm: float = 0.0
    dy_mm: float = 0.0
    yaw_deg: float = 0.0
    dz_mm: float = 0.0


# ±8 mm x/y, ±2.5 deg yaw, one +3.0 mm z lift of the whole board body
# (playing surface and printed marker face move together), and two corners.
MATRIX = (
    Placement("centre"),
    Placement("x-8", dx_mm=-8.0),
    Placement("x+8", dx_mm=8.0),
    Placement("y-8", dy_mm=-8.0),
    Placement("y+8", dy_mm=8.0),
    Placement("yaw-2.5", yaw_deg=-2.5),
    Placement("yaw+2.5", yaw_deg=2.5),
    Placement("z+3.0", dz_mm=3.0),
    Placement("x+8_y-8_yaw+2.5", dx_mm=8.0, dy_mm=-8.0, yaw_deg=2.5),
    Placement("x-8_y+8_yaw-2.5", dx_mm=-8.0, dy_mm=8.0, yaw_deg=-2.5),
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeBridge:
    """Same simulator-backed bridge the scan-controller tests use."""

    def __init__(self, env, clock, *, rate_deg_s: float = 240.0) -> None:
        self.env = env
        self.clock = clock
        self.rate = rate_deg_s

    def joints(self) -> tuple[float, ...]:
        return tuple(float(np.degrees(v))
                     for v in self.env.data.qpos[self.env._joint_qpos])

    def state(self) -> dict:
        return {"mode": "REMOTE", "armed": True,
                "joints": dict(zip(NAMES, self.joints()))}

    def command(self, joints, *, seq: int, ttl_ms: int, speed_scale: float) -> dict:
        target = np.radians(np.asarray(joints, dtype=float))
        current = self.env.data.qpos[self.env._joint_qpos].copy()
        step = math.radians(self.rate) * 0.1
        self.env.data.qpos[self.env._joint_qpos] = current + np.clip(
            target - current, -step, step)
        mujoco.mj_forward(self.env.model, self.env.data)
        return {"accepted": True, "seq": seq}


class RecordingFrames:
    """Renders the wrist view and remembers every frame with its joints."""

    def __init__(self, env, clock, calibration: WristCalibration) -> None:
        self.env = env
        self.clock = clock
        self.calibration = calibration
        self.records: list[tuple[np.ndarray, tuple[float, ...]]] = []

    def joints(self) -> tuple[float, ...]:
        return tuple(float(np.degrees(v))
                     for v in self.env.data.qpos[self.env._joint_qpos])

    def capture(self) -> Frame:
        joints = self.joints()
        image = self.env.render(Camera.WRIST).copy()
        self.records.append((image, joints))
        return Frame(image=image, joints_deg=joints, timestamp=self.clock())


def build_env(placement: Placement):
    config = layout_env_config("printed", cameras=(Camera.WRIST,))
    board = config.board.model_copy(update={
        "face": "charuco",
        "origin": (placement.dx_mm / 1000.0, placement.dy_mm / 1000.0),
        "yaw": math.radians(placement.yaw_deg),
    })
    env = ChessSimEnv(config.model_copy(update={"board": board}))
    env.reset("8/8/8/8/8/8/8/8")
    nominal_z = float(env.data.mocap_pos[env._board_mocap][2])
    env._move_board(board.origin, board.yaw)
    env.data.mocap_pos[env._board_mocap][2] = nominal_z + placement.dz_mm / 1000.0
    mujoco.mj_forward(env.model, env.data)
    # The mocap body's z is the slab centre (half a thickness below the
    # playing surface).  The printed ChArUco face is on the surface, so the
    # marker plane is board.top, not the body centre.
    return env, env.board.top + placement.dz_mm / 1000.0


def base_xy() -> tuple[float, float]:
    mount = current_layout()["mount"]
    return (mount["white_arm_base_frame_mm"][0] / 1000.0,
            mount["white_arm_base_frame_mm"][1] / 1000.0)


def run_scan(env, calibration, marker_z_m: float):
    clock = FakeClock()
    frames = RecordingFrames(env, clock, calibration)
    plan = plan_in_degrees(load_plan(PLAN))
    controller = ScanController(
        FakeBridge(env, clock), frames,
        lambda joints_deg: calibration.camera(
            tuple(math.radians(v) for v in joints_deg)),
        plan, layout=CharucoLayout(square_mm=SQUARE_MM, marker_mm=MARKER_MM),
        square_m=SQUARE_MM / 1000.0,
        board_z_m=env.board.top,
        marker_z_m=marker_z_m,
        base_xy_m=base_xy(),
        board_half_mm=env.board.width * 500.0,
        clock=clock, sleep=clock.sleep)
    return controller.run(), frames.records


def derive_candidates(frame: np.ndarray, joints_deg, calibration,
                      layout: CharucoLayout) -> list[dict]:
    """/board/derive maths: per-marker PnP, camera pose composed with FK."""
    detections = fiducials.detect_markers(frame)
    base_camera = (offline_tool.flange_pose(
        calibration, np.radians(np.asarray(joints_deg, dtype=float)))
        @ np.asarray(calibration.camera_in_tool, dtype=float))
    out = []
    for detection in detections:
        if not 5 <= detection.marker_id <= 36:
            continue
        try:
            corners = np.asarray(marker_corners_world(
                layout, detection.marker_id, origin_xy=(0, 0), yaw=0,
                marker_z=0), dtype=float)
        except (ValueError, KeyError):
            continue
        solved = pnp_tool_frame(corners,
                                np.asarray(detection.corners, dtype=float),
                                calibration)
        if solved is None:
            continue
        # pnp_tool_frame returns board -> calibrated camera (-z forward,
        # +y up), the frame base_camera lives in.
        camera_from_board = solved[0]
        board = base_camera @ camera_from_board
        camera_pts = (camera_from_board @ np.c_[corners, np.ones(len(corners))].T).T[:, :3]
        projected = project_tool_frame(camera_pts, calibration)
        error_px = float(np.mean(np.linalg.norm(
            projected.reshape(-1, 2) - np.asarray(detection.corners, dtype=float),
            axis=1)))
        out.append({"marker_id": detection.marker_id,
                    "reproj_px": error_px,
                    "x_mm": float(board[0, 3]) * 1000.0,
                    "y_mm": float(board[1, 3]) * 1000.0,
                    "z_mm": float(board[2, 3]) * 1000.0,
                    "yaw_deg": math.degrees(math.atan2(board[1, 0], board[0, 0]))})
    return out


def plane_fit_view(frame: np.ndarray, joints_deg, calibration,
                   layout: CharucoLayout, seed_pose=None) -> dict | None:
    """One full planar pose per view from every marker it can see.

    This is the marker-plane fit whose z stability the block has to report.
    A single 20 mm marker is depth-poor; the 224 mm face is not.
    """
    detections = [d for d in fiducials.detect_markers(frame)
                  if 5 <= d.marker_id <= 36]
    if len(detections) < 2:
        return None
    object_list, image_list, ids = [], [], []
    for detection in detections:
        try:
            corners = np.asarray(marker_corners_world(
                layout, detection.marker_id, origin_xy=(0, 0), yaw=0,
                marker_z=0), dtype=float)
        except (ValueError, KeyError):
            continue
        object_list.extend(corners)
        image_list.extend(np.asarray(detection.corners, dtype=float))
        ids.append(detection.marker_id)
    if not ids:
        return None
    points = np.asarray(object_list, dtype=float)
    pixels = np.asarray(image_list, dtype=float)
    base_camera = (offline_tool.flange_pose(
        calibration, np.radians(np.asarray(joints_deg, dtype=float)))
        @ np.asarray(calibration.camera_in_tool, dtype=float))
    view_calibration = calibration.camera(
        np.radians(np.asarray(joints_deg, dtype=float)))
    fx, fy, cx, cy = (view_calibration.fx, view_calibration.fy,
                      view_calibration.cx, view_calibration.cy)
    solved = pnp_tool_frame(points, pixels, view_calibration)
    if solved is None:
        return None
    camera_from_board = solved[0]
    board = base_camera @ camera_from_board

    # Refine this view's full planar pose in the calibrated (MuJoCo) camera
    # convention with z free.  The single-PnP seed is decent in x/y but its
    # depth branch can be a few millimetres -- or, on a two-marker view, the
    # wrong plane entirely.  A bounded least-squares fit against every visible
    # corner removes that branch ambiguity and is the marker-plane fit whose
    # z stability the block has to report.
    cam_position = np.asarray(view_calibration.camera_position, dtype=float)
    cam_rotation = np.asarray(view_calibration.camera_rotation,
                              dtype=float).reshape(3, 3)

    def project(params):
        x, y, z, yaw = params
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.stack([x + c * points[:, 0] - s * points[:, 1],
                          y + s * points[:, 0] + c * points[:, 1],
                          np.full(len(points), z)], axis=1)
        camera = (world - cam_position) @ cam_rotation
        depth = np.where(camera[:, 2] < -1e-6, -camera[:, 2], 1e-6)
        return np.column_stack([cx + fx * camera[:, 0] / depth,
                                cy - fy * camera[:, 1] / depth])

    coarse_seed = np.array([board[0, 3], board[1, 3], board[2, 3],
                            math.atan2(board[1, 0], board[0, 0])])
    if seed_pose is not None:
        # A single tag (or two) can pick the mirrored IPPE plane, 100s of mm
        # away.  Seeding every view from the robust median seed turns that
        # branch ambiguity into a fit that either agrees or is visibly wrong,
        # instead of an outlier that masquerades as z noise.
        seed = np.asarray(seed_pose, dtype=float)
    else:
        seed = coarse_seed
    lower = seed - np.array([0.05, 0.05, 0.05, math.radians(10.0)])
    upper = seed + np.array([0.05, 0.05, 0.05, math.radians(10.0)])
    fitted = least_squares(lambda p: (project(p) - pixels).ravel(), seed,
                           bounds=(lower, upper), method="trf",
                           loss="soft_l1", f_scale=0.5,
                           xtol=1e-12, ftol=1e-12, gtol=1e-12)
    board = board.copy()
    board[:3, 3] = fitted.x[:3]
    board[:3, :3] = np.array([[math.cos(fitted.x[3]), -math.sin(fitted.x[3]), 0.0],
                              [math.sin(fitted.x[3]), math.cos(fitted.x[3]), 0.0],
                              [0.0, 0.0, 1.0]])
    rms = float(np.sqrt(np.mean((project(fitted.x) - pixels) ** 2)))
    return {"markers": len(ids), "rms_px": rms, "coarse_seed": coarse_seed.tolist(),
            "x_mm": float(board[0, 3]) * 1000.0,
            "y_mm": float(board[1, 3]) * 1000.0,
            "z_mm": float(board[2, 3]) * 1000.0,
            "yaw_deg": math.degrees(math.atan2(board[1, 0], board[0, 0]))}


def yaw_error_deg(got: float, want: float) -> float:
    return abs((got - want + 180.0) % 360.0 - 180.0)


def plane_fit_all(records, calibration, layout, seed_pose) -> dict | None:
    """One marker-plane fit through every pose of a placement.

    Each view keeps its own calibrated camera; the four plane parameters are
    shared.  This is the stable quantity: a single view's z is dominated by its
    own corner noise, while the shared fit uses every sighting on the board.
    """
    observations = []
    for image, joints in records:
        detections = [d for d in fiducials.detect_markers(image)
                      if 5 <= d.marker_id <= 36]
        if not detections:
            continue
        camera = calibration.camera(np.radians(np.asarray(joints, dtype=float)))
        object_list, image_list = [], []
        for detection in detections:
            try:
                object_list.extend(marker_corners_world(
                    layout, detection.marker_id, marker_z=0))
            except (ValueError, KeyError):
                continue
            image_list.extend(np.asarray(detection.corners, dtype=float))
        if object_list:
            observations.append((camera, np.asarray(object_list, dtype=float),
                                 np.asarray(image_list, dtype=float)))
    if not observations:
        return None

    def project(params, camera, points):
        x, y, z, yaw = params
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.stack([x + c * points[:, 0] - s * points[:, 1],
                          y + s * points[:, 0] + c * points[:, 1],
                          np.full(len(points), z)], axis=1)
        cam = (world - np.asarray(camera.camera_position, dtype=float)) \
            @ np.asarray(camera.camera_rotation, dtype=float).reshape(3, 3)
        depth = np.where(cam[:, 2] < -1e-6, -cam[:, 2], 1e-6)
        return np.column_stack([camera.cx + camera.fx * cam[:, 0] / depth,
                                camera.cy - camera.fy * cam[:, 1] / depth])

    def residual(params):
        return np.concatenate([
            (project(params, camera, points) - pixels).ravel()
            for camera, points, pixels in observations])

    seed = np.asarray(seed_pose, dtype=float)
    lower = seed - np.array([0.05, 0.05, 0.05, math.radians(10.0)])
    upper = seed + np.array([0.05, 0.05, 0.05, math.radians(10.0)])
    result = least_squares(residual, seed, bounds=(lower, upper), method="trf",
                           loss="soft_l1", f_scale=0.5,
                           xtol=1e-12, ftol=1e-12, gtol=1e-12)
    for _ in range(2):
        view_rms = [float(np.sqrt(np.mean(
            (project(result.x, camera, points) - pixels) ** 2)))
            for camera, points, pixels in observations]
        median = float(np.median(view_rms))
        mad = float(np.median(np.abs(np.asarray(view_rms) - median))) + 1e-6
        keep = [i for i, value in enumerate(view_rms)
                if value <= max(1.2, median + 3.0 * mad)]
        if len(keep) == len(observations) or len(keep) < max(2, len(observations) // 2):
            break
        observations = [observations[i] for i in keep]
        result = least_squares(residual, np.clip(result.x, lower, upper),
                               bounds=(lower, upper), method="trf",
                               loss="soft_l1", f_scale=0.5,
                               xtol=1e-12, ftol=1e-12, gtol=1e-12)
    pose = result.x
    return {"views": len(observations),
            "x_mm": float(pose[0]) * 1000.0,
            "y_mm": float(pose[1]) * 1000.0,
            "z_mm": float(pose[2]) * 1000.0,
            "yaw_deg": math.degrees(float(pose[3])),
            "rms_px": float(np.sqrt(np.mean(residual(pose) ** 2)))}


def z_fit_view(frame, joints, calibration, layout, plane_xy_yaw) -> float | None:
    """Depth of one view with the plane's x/y/yaw held at the shared fit.

    This isolates the z estimate from in-plane pose error: the board is one
    rigid plane, so once its in-plane pose is established a per-view refit has
    only one free parameter.  The spread of this number is the plane's z
    stability; a free four-parameter refit adds in-plane noise to depth.
    """
    detections = [d for d in fiducials.detect_markers(frame)
                  if 5 <= d.marker_id <= 36]
    if len(detections) < 2:
        return None
    camera = calibration.camera(np.radians(np.asarray(joints, dtype=float)))
    object_list, image_list = [], []
    for detection in detections:
        try:
            object_list.extend(marker_corners_world(
                layout, detection.marker_id, marker_z=0))
        except (ValueError, KeyError):
            continue
        image_list.extend(np.asarray(detection.corners, dtype=float))
    if not object_list:
        return None
    points = np.asarray(object_list, dtype=float)
    pixels = np.asarray(image_list, dtype=float)
    x, y, yaw = (float(v) for v in plane_xy_yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    world_xy = np.stack([x + c * points[:, 0] - s * points[:, 1],
                         y + s * points[:, 0] + c * points[:, 1]], axis=1)
    cam_position = np.asarray(camera.camera_position, dtype=float)
    cam_rotation = np.asarray(camera.camera_rotation, dtype=float).reshape(3, 3)

    def residual(depth):
        world = np.column_stack([world_xy, np.full(len(points), depth[0])])
        cam = (world - cam_position) @ cam_rotation
        z = np.where(cam[:, 2] < -1e-6, -cam[:, 2], 1e-6)
        projected = np.column_stack([camera.cx + camera.fx * cam[:, 0] / z,
                                     camera.cy - camera.fy * cam[:, 1] / z])
        return (projected - pixels).ravel()

    result = least_squares(residual, np.array([0.4337]),
                           bounds=([0.30], [0.60]), method="trf",
                           xtol=1e-14, ftol=1e-14, gtol=1e-14)
    return float(result.x[0]) * 1000.0


def dump_captures(records, directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for index, (image, joints) in enumerate(records):
        stem = directory / f"capture_{index:02d}"
        stem.with_suffix(".jpg").write_bytes(
            cv2.imencode(".jpg", image[..., ::-1])[1].tobytes())
        stem.with_suffix(".json").write_text(json.dumps({
            "state_joints_deg": dict(zip(NAMES, joints))}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="reports/block3_matrix.json")
    parser.add_argument("--captures", default="reports/block3_captures")
    parser.add_argument("--calibration", default=str(CALIBRATION))
    parser.add_argument("--only", default="",
                        help="comma-separated placement names for a quick check")
    parser.add_argument("--no-stale", action="store_true",
                        help="skip the stale live marker-plane sensitivity run")
    args = parser.parse_args()

    calibration = WristCalibration.load(args.calibration)
    layout = CharucoLayout(square_mm=SQUARE_MM, marker_mm=MARKER_MM)
    capture_root = ROOT / args.captures
    if capture_root.exists():
        shutil.rmtree(capture_root)

    rows = []
    for placement in MATRIX:
        if args.only and placement.name not in args.only.split(","):
            continue
        env, board_z_m = build_env(placement)
        try:
            truth_z = board_z_m
            report, records = run_scan(env, calibration, marker_z_m=truth_z)
            stale_report = None
            if not args.no_stale:
                # Live config still passes the mat plane (0.5 mm above table).
                stale_report = run_scan(env, calibration,
                                        marker_z_m=STALE_MARKER_Z_M)[0]
            truth = board_offset(env.board.origin, env.board.yaw,
                                 base_xy=base_xy(),
                                 board_half_mm=env.board.width * 500.0)
            truth_deg = {"x_mm": truth.x_mm, "y_mm": truth.y_mm,
                         "yaw_deg": truth.yaw_deg, "z_mm": truth_z * 1000.0}

            # b. the /board/derive maths, on the first scan frame that has a tag
            # (a live derive happens at whatever posture the arm is in).
            derive_row = None
            derive_pose = None
            for index, (image, joints) in enumerate(records):
                candidates = derive_candidates(image, joints, calibration, layout)
                if candidates:
                    best = min(candidates, key=lambda item: item["reproj_px"])
                    derive_row = best
                    derive_pose = f"frame{index:02d}"
                    break

            # c. the offline median over the same captures.
            dump_captures(records, capture_root / placement.name)
            offline = offline_tool.estimate(capture_root / placement.name,
                                            calibration, "charuco")
            # What the live config's marker plane does to the same views.  The
            # shipped gate rejects every one, so report the fit a relaxed gate
            # would return and the residual that raised the rejection.
            stale_views = [(image, calibration.camera(
                np.radians(np.asarray(joints, dtype=float))))
                for image, joints in records]
            stale_relaxed = fiducials.estimate_board_pose(
                stale_views, SQUARE_MM / 1000.0, board_z_m,
                marker_z=STALE_MARKER_Z_M, layout=layout,
                min_markers=2, max_rms_px=99.0)

            # stability: one full planar fit per view
            coarse = []
            for image, joints in records:
                if (fit := plane_fit_view(image, joints, calibration, layout)):
                    coarse.append((image, joints, fit))
            fits = []
            if coarse:
                seeds = np.asarray([fit["coarse_seed"] for _, _, fit in coarse])
                reference = float(np.median(seeds[:, 3]))
                yaw = reference + ((seeds[:, 3] - reference + math.pi)
                                   % (2.0 * math.pi)) - math.pi
                median_seed = np.array([np.median(seeds[:, 0]), np.median(seeds[:, 1]),
                                        np.median(seeds[:, 2]), np.median(yaw)])
                fits = [fit for image, joints, _ in coarse
                        if (fit := plane_fit_view(image, joints, calibration,
                                                  layout, seed_pose=median_seed))]
                shared = plane_fit_all(records, calibration, layout, median_seed)
            else:
                shared = None
            kept = [fit for fit in fits if fit["rms_px"] <= 1.2] or fits
            zs = [fit["z_mm"] for fit in kept]
            z_only = []
            if shared is not None:
                for image, joints in records:
                    value = z_fit_view(image, joints, calibration, layout,
                                       (shared["x_mm"] / 1000.0,
                                        shared["y_mm"] / 1000.0,
                                        math.radians(shared["yaw_deg"])))
                    if value is not None:
                        z_only.append(value)

            round3 = lambda value: None if value is None else round(float(value), 3)
            rows.append({
                "placement": placement.__dict__,
                "truth": {k: round(v, 3) for k, v in truth_deg.items()},
                "truth_origin_world_mm": [round(placement.dx_mm, 3),
                                          round(placement.dy_mm, 3),
                                          round(truth_z * 1000.0, 3)],
                "scan": None if not report.ok else {
                    "x_mm": round(report.offset.x_mm, 3),
                    "y_mm": round(report.offset.y_mm, 3),
                    "yaw_deg": round(report.offset.yaw_deg, 4),
                    "ex": round(report.offset.x_mm - truth.x_mm, 3),
                    "ey": round(report.offset.y_mm - truth.y_mm, 3),
                    "eyaw": round(yaw_error_deg(report.offset.yaw_deg,
                                                truth.yaw_deg), 4),
                    "views": report.views_used,
                    "markers": list(report.markers_seen),
                    "note": report.note,
                },
                "scan_note": report.note,
                "scan_stale_plane": None if stale_report is None or not stale_report.ok else {
                    "x_mm": round(stale_report.offset.x_mm, 3),
                    "y_mm": round(stale_report.offset.y_mm, 3),
                    "yaw_deg": round(stale_report.offset.yaw_deg, 4),
                    "ex": round(stale_report.offset.x_mm - truth.x_mm, 3),
                    "ey": round(stale_report.offset.y_mm - truth.y_mm, 3),
                    "eyaw": round(yaw_error_deg(stale_report.offset.yaw_deg,
                                                truth.yaw_deg), 4),
                },
                "scan_stale_note": None if stale_report is None else stale_report.note,
                "stale_plane_relaxed": None if stale_relaxed is None else {
                    "rms_px": round(99.0 * (1.0 - stale_relaxed.score), 3),
                    "ex": round(stale_relaxed.origin[0] * 1000.0
                                - placement.dx_mm, 3),
                    "ey": round(stale_relaxed.origin[1] * 1000.0
                                - placement.dy_mm, 3),
                    "eyaw": round(yaw_error_deg(
                        math.degrees(stale_relaxed.yaw), truth.yaw_deg), 3),
                },
                "derive": None if derive_row is None else {
                    "pose": derive_pose,
                    "marker_id": derive_row["marker_id"],
                    "reproj_px": round(derive_row["reproj_px"], 3),
                    "x_mm": round(derive_row["x_mm"], 3),
                    "y_mm": round(derive_row["y_mm"], 3),
                    "z_mm": round(derive_row["z_mm"], 3),
                    "yaw_deg": round(derive_row["yaw_deg"], 4),
                    "ex": round(derive_row["x_mm"] - placement.dx_mm, 3),
                    "ey": round(derive_row["y_mm"] - placement.dy_mm, 3),
                    "ez": round(derive_row["z_mm"] - truth_z * 1000.0, 3),
                    "eyaw": round(yaw_error_deg(derive_row["yaw_deg"],
                                                truth.yaw_deg), 4),
                },
                "offline": None if "error" in offline else {
                    **{k: round(float(offline["board"][k]), 3)
                       for k in ("x_mm", "y_mm", "z_mm", "yaw_deg")},
                    "ex": round(float(offline["board"]["x_mm"]) - placement.dx_mm, 3),
                    "ey": round(float(offline["board"]["y_mm"]) - placement.dy_mm, 3),
                    "ez": round(float(offline["board"]["z_mm"]) - truth_z * 1000.0, 3),
                    "eyaw": round(yaw_error_deg(float(offline["board"]["yaw_deg"]),
                                                truth.yaw_deg), 4),
                    "sightings": offline["sightings"],
                    "worst_axis_spread_mm": offline["worst_axis_spread_mm"],
                },
                "offline_note": offline.get("error"),
                "plane_fit": {
                    "views": len(kept),
                    "views_rejected": len(fits) - len(kept),
                    "z_values_mm": [round(z, 3) for z in zs],
                    "views_detail": [{"z_mm": round(f["z_mm"], 3),
                                      "markers": f["markers"],
                                      "rms_px": round(f["rms_px"], 3)} for f in kept],
                    "z_median_mm": round(float(np.median(zs)), 3) if zs else None,
                    "z_min_mm": round(float(np.min(zs)), 3) if zs else None,
                    "z_max_mm": round(float(np.max(zs)), 3) if zs else None,
                    "z_spread_mm": round(float(np.max(zs) - np.min(zs)), 3) if zs else None,
                    "z_bias_mm": round(float(np.median(zs)) - truth_z * 1000.0, 3) if zs else None,
                    "rms_px_median": round(float(np.median([f["rms_px"] for f in fits])), 3) if fits else None,
                    "z_only_median_mm": round(float(np.median(z_only)), 3) if z_only else None,
                    "z_only_spread_mm": round(float(np.max(z_only) - np.min(z_only)), 3) if z_only else None,
                    "z_only_bias_mm": round(float(np.median(z_only)) - truth_z * 1000.0, 3) if z_only else None,
                    "shared": None if shared is None else {
                        "views": shared["views"],
                        "x_mm": round(shared["x_mm"], 3),
                        "y_mm": round(shared["y_mm"], 3),
                        "z_mm": round(shared["z_mm"], 3),
                        "yaw_deg": round(shared["yaw_deg"], 4),
                        "rms_px": round(shared["rms_px"], 3),
                        "ex": round(shared["x_mm"] - placement.dx_mm, 3),
                        "ey": round(shared["y_mm"] - placement.dy_mm, 3),
                        "ez": round(shared["z_mm"] - truth_z * 1000.0, 3),
                        "eyaw": round(yaw_error_deg(shared["yaw_deg"], truth.yaw_deg), 4),
                    },
                },
                "frames": len(records),
            })
        finally:
            env.close()

    # summaries
    def summarize(est: str, keys=("ex", "ey", "eyaw")):
        values = {key: [] for key in keys}
        for row in rows:
            entry = row.get(est)
            if not entry:
                continue
            for key in keys:
                if key in entry and entry[key] is not None:
                    values[key].append(abs(entry[key]))
        return {key: {"median": round(float(np.median(v)), 3),
                      "max": round(float(np.max(v)), 3)} for key, v in values.items() if v}

    summary = {
        "scan": summarize("scan", ("ex", "ey", "eyaw")),
        "scan_stale_plane": summarize("scan_stale_plane", ("ex", "ey", "eyaw")),
        "derive": summarize("derive", ("ex", "ey", "ez", "eyaw")),
        "offline": summarize("offline", ("ex", "ey", "ez", "eyaw")),
    }
    spreads = [row["plane_fit"]["z_spread_mm"] for row in rows
               if row["plane_fit"]["z_spread_mm"] is not None]
    biases = [row["plane_fit"]["z_bias_mm"] for row in rows
              if row["plane_fit"]["z_bias_mm"] is not None]
    shared_biases = [row["plane_fit"]["shared"]["ez"] for row in rows
                     if row["plane_fit"].get("shared")]
    z_only_biases = [row["plane_fit"]["z_only_bias_mm"] for row in rows
                     if row["plane_fit"].get("z_only_bias_mm") is not None]
    z_only_spreads = [row["plane_fit"]["z_only_spread_mm"] for row in rows
                      if row["plane_fit"].get("z_only_spread_mm") is not None]
    summary["plane_fit"] = {
        "within_placement_spread_median_mm": round(float(np.median(spreads)), 3) if spreads else None,
        "within_placement_spread_max_mm": round(float(np.max(spreads)), 3) if spreads else None,
        "per_view_bias_spread_mm": round(float(np.max(biases) - np.min(biases)), 3) if biases else None,
        "per_view_bias_median_mm": round(float(np.median(biases)), 3) if biases else None,
        "shared_bias_spread_mm": round(float(np.max(shared_biases) - np.min(shared_biases)), 3) if shared_biases else None,
        "shared_bias_median_mm": round(float(np.median(shared_biases)), 3) if shared_biases else None,
        "z_only_spread_median_mm": round(float(np.median(z_only_spreads)), 3) if z_only_spreads else None,
        "z_only_spread_max_mm": round(float(np.max(z_only_spreads)), 3) if z_only_spreads else None,
        "z_only_bias_spread_mm": round(float(np.max(z_only_biases) - np.min(z_only_biases)), 3) if z_only_biases else None,
        "z_only_bias_median_mm": round(float(np.median(z_only_biases)), 3) if z_only_biases else None,
    }

    payload = {"schema": "robot-arm-block3-placement-matrix/1",
               "solver": "production plan + ChArUco face, board moved physically",
               "calibration": {"path": args.calibration,
                               "id": getattr(calibration, "calibration_id", None),
                               "intrinsics": list(calibration.intrinsics)},
               "marker_plane_m": 0.4337,
               "stale_live_marker_z_m": STALE_MARKER_Z_M,
               "summary": summary, "rows": rows}
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n")

    print(f"{'placement':22s} {'scan ex/ey/eyaw':26s} {'derive ex/ey/ez/eyaw':34s} "
          f"{'offline ex/ey/ez/eyaw':34s} {'zspread':>8s}")
    for row in rows:
        def fmt(entry, keys):
            if not entry:
                return "-"
            return "/".join(f"{entry[k]:+.2f}" if entry.get(k) is not None else "-"
                            for k in keys)
        scan = fmt(row["scan"], ("ex", "ey", "eyaw"))
        derive = fmt(row["derive"], ("ex", "ey", "ez", "eyaw"))
        offline = fmt(row["offline"], ("ex", "ey", "ez", "eyaw"))
        spread = row["plane_fit"]["z_spread_mm"]
        print(f"{row['placement']['name']:22s} {scan:26s} {derive:34s} "
              f"{offline:34s} {spread if spread is not None else float('nan'):8.3f}")
    print("\nsummary:", json.dumps(summary, indent=2))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
