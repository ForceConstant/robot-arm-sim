"""Cartesian control reaches every square, verified by what the camera sees.

Two layers, deliberately:

* ``test_recorded_sweep_*`` assert on ``reports/camera_sweep.json`` — the artefact
  produced by ``tools/sweep_camera_poses.py``.  They are fast and they encode the
  full 64-square result, including the per-square aim offset and marker count.
* ``test_live_*`` re-run the solver on a few squares and render real frames, so a
  regression in the solver cannot hide behind a stale artefact.

The lesson behind the aim check: a pose can report 2-4 mm of position error while
the camera points at the sky.  Position residuals alone cannot confirm a pose
move, so every verification here also asks whether the target square actually
lands near frame centre with markers in sight.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from chess_sim import fiducials
from chess_sim.camera_ik import CameraIK
from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
from chess_sim.env import ChessSimEnv

REPORT = Path(__file__).resolve().parents[1] / "reports/camera_sweep.json"
AIM_TOLERANCE_PX = 60.0
POSITION_TOLERANCE_MM = 12.0
AIM_TOLERANCE_DEG = 6.0


@pytest.fixture(scope="module")
def report():
    if not REPORT.exists():
        pytest.skip("no camera sweep artefact; run tools/sweep_camera_poses.py")
    return json.loads(REPORT.read_text())


def test_recorded_sweep_covers_every_square(report):
    assert report["total"] == 64
    assert report["solved"] == 64, [
        s["square"] for s in report["squares"] if not s["ok"]]


def test_recorded_sweep_aims_at_the_target(report):
    aimed = report["aimed_within_60px"]
    assert aimed >= 60, f"only {aimed}/64 landed within {AIM_TOLERANCE_PX:.0f} px"


def test_recorded_sweep_sees_markers(report):
    """Every solved square must show at least one tag, or the aim is unverified."""
    missing = [s["square"] for s in report["squares"] if s["ok"] and s["markers"] < 1]
    assert not missing, f"no markers visible at: {missing}"


def test_recorded_sweep_errors_are_small(report):
    for square in report["squares"]:
        if not square["ok"]:
            continue
        assert square["position_error_mm"] < POSITION_TOLERANCE_MM, square
        assert square["aim_error_deg"] < AIM_TOLERANCE_DEG, square


@pytest.fixture(scope="module")
def live_env():
    board = BoardConfig(face="charuco", yaw=math.pi)
    env = ChessSimEnv(EnvConfig(board=board,
                                appearance=AppearanceConfig(board_markers=False)))
    env.reset("8/8/8/8/8/8/8/8")
    try:
        yield env
    finally:
        env.close()


def _stare_at(env, x_mm: float, y_mm: float):
    """Solve, render, and report what the camera actually saw."""
    board = env.board
    cam = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
    height, width = 480, 640
    focal = 0.5 * height / math.tan(math.radians(float(env.model.cam_fovy[cam])) / 2.0)
    ik = CameraIK(env, "wrist")

    yaw = board.yaw
    rot2d = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    offset = rot2d @ np.array([x_mm / 1000.0, y_mm / 1000.0])
    point = np.array([board.origin[0] + offset[0], board.origin[1] + offset[1], board.top])

    seeds = [np.radians(v) for v in ([0, -55, 72, 39, 22], [-45, -60, 70, 25, 0],
                                     [45, -60, 70, 25, 0])]
    best = None
    for azimuth in (0.0, 45.0, -45.0):
        for distance in (230.0, 180.0):
            position, rotation = ik.aim_at(point, distance, pitch_deg=20.0,
                                           azimuth_deg=azimuth)
            for seed in seeds:
                trial = ik.solve(position, rotation, seed=seed)
                score = trial["position_error_mm"] + 20.0 * trial["aim_error_deg"]
                if best is None or score < best[0]:
                    best = (score, trial)
                if trial["position_error_mm"] < 3 and trial["aim_error_deg"] < 4:
                    break
    solution = best[1]

    camera_position, camera_rotation = ik.camera_pose(solution["q"])
    point_camera = camera_rotation.T @ (point - camera_position)
    depth = -point_camera[2]                       # cameras look along -z
    aim_px = None
    if depth > 1e-6:
        u = focal * point_camera[0] / depth + width / 2.0
        v = -focal * point_camera[1] / depth + height / 2.0
        aim_px = float(np.hypot(u - width / 2.0, v - height / 2.0))
    image = env.render("wrist")
    markers = len([d for d in fiducials.detect_markers(image)
                   if 5 <= d.marker_id <= 36])
    return solution, aim_px, markers


@pytest.mark.parametrize("square,xy", [("a1", (-98.0, -98.0)), ("d4", (-14.0, 14.0)),
                                       ("h8", (98.0, 98.0))])
def test_live_solve_aims_and_sees_markers(live_env, square, xy):
    """Solve, point, and confirm with the camera — not with a residual."""
    solution, aim_px, markers = _stare_at(live_env, *xy)
    assert solution["position_error_mm"] < POSITION_TOLERANCE_MM, (square, solution)
    assert solution["aim_error_deg"] < AIM_TOLERANCE_DEG, (square, solution)
    assert aim_px is not None and aim_px < AIM_TOLERANCE_PX, (square, aim_px)
    assert markers >= 1, f"{square}: no markers in view, aim is unverified"
