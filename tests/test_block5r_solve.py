"""tests/test_block5r_solve.py — the measurement path the wide-volume solve uses.

These are pure-geometry tests.  They exist because the tool it guards had two
silent defects that made its numbers untrustworthy:

* ``view_mount()`` solved PnP *with* lens distortion while ``validate()``
  projected without it, so the residual and the fit used different cameras;
* ``SOLVEPNP_IPPE`` returns two valid solutions for a planar target and the code
  took the first one, which produced a 32 deg outlier on a 10-marker view
  (``reports/rot_scan_20261127.md``).

No hardware, no MuJoCo, no captured images: a known camera pose is projected
through a known lens and must come back.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import block5r_wide_solve as solve  # noqa: E402


class _Calibration:
    """Just the two intrinsics fields the projection helpers read."""

    intrinsics = (831.897, 833.014, 320.0, 240.0)
    distortion = (-0.102251, 0.685733, 0.001112, 0.003239, 0.0)


class _Pinhole(_Calibration):
    distortion = (0.0, 0.0, 0.0, 0.0, 0.0)


def _board_points() -> np.ndarray:
    """A few ChArUco-ish square corners on the board plane, metres."""
    points = []
    for marker_id, (cx, cy) in enumerate([(0, 0), (1, 0), (0, 1), (1, 1)]):
        for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            points.append([(cx * 0.2) + dx * 0.014, (cy * 0.2) + dy * 0.014, 0.0])
    return np.asarray(points, float)


def _pose(forward_m: float, tilt_deg: float, yaw_deg: float, roll_deg: float) -> np.ndarray:
    """A board-in-camera pose in the project's flipped convention."""
    rotation = (Rotation.from_euler("z", yaw_deg, degrees=True)
                * Rotation.from_euler("x", tilt_deg, degrees=True)
                * Rotation.from_euler("z", roll_deg, degrees=True)).as_matrix()
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = [0.0, 0.0, -forward_m]     # depth is -z in this convention
    return pose


def _project(obj: np.ndarray, pose: np.ndarray, calibration) -> np.ndarray:
    pts = (pose @ np.c_[obj, np.ones(len(obj))].T).T[:, :3]
    return solve.project_tool_frame(pts, calibration)


def test_projection_matches_the_project_convention():
    """Zero distortion must reduce to depth = -z, v = fy*(-y)/depth."""
    obj = _board_points()
    pose = _pose(0.42, 6.0, 3.0, 1.0)
    pts = (pose @ np.c_[obj, np.ones(len(obj))].T).T[:, :3]
    fx, fy, cx, cy = _Pinhole.intrinsics
    depth = -pts[:, 2]
    expected = np.stack([fx * pts[:, 0] / depth + cx,
                         fy * (-pts[:, 1]) / depth + cy], axis=1)
    got = solve.project_tool_frame(pts, _Pinhole())
    assert np.allclose(got, expected, atol=1e-6)


def test_points_behind_the_camera_are_not_scored():
    obj = _board_points()
    behind = _pose(-0.42, 0.0, 0.0, 0.0)      # positive z -> behind
    projected = _project(obj, behind, _Pinhole())
    assert np.isnan(projected).all()


@pytest.mark.parametrize("forward,tilt,yaw,roll", [
    (0.30, 0.0, 0.0, 0.0),
    (0.43, 12.0, 4.0, 0.0),
    (0.38, 24.0, -9.0, 8.0),
    (0.52, 30.0, 14.0, -6.0),
])
def test_pnp_recovers_a_known_mount(forward, tilt, yaw, roll):
    obj = _board_points()
    truth = _pose(forward, tilt, yaw, roll)
    image = _project(obj, truth, _Calibration())
    assert np.isfinite(image).all()

    solved = solve.pnp_tool_frame(obj, image, _Calibration())
    assert solved is not None
    mount, residual = solved

    assert residual < 0.05, f"residual {residual:.3f} px on a noise-free projection"
    translation_error = np.linalg.norm(mount[:3, 3] - truth[:3, 3]) * 1000.0
    rotation_error = math.degrees(
        Rotation.from_matrix(truth[:3, :3].T @ mount[:3, :3]).magnitude())
    assert translation_error < 0.2, f"{translation_error:.3f} mm"
    assert rotation_error < 0.05, f"{rotation_error:.3f} deg"


def test_pnp_never_worse_than_the_first_ippe_branch():
    """The branch that used to be taken blindly is sometimes the wrong one."""
    obj = _board_points()
    pose = _pose(0.40, 18.0, 7.0, 4.0)
    image = _project(obj, pose, _Calibration())

    calibration = _Calibration()
    K = solve.camera_matrix(calibration)
    dist = np.asarray(calibration.distortion, float)
    first_errors = []
    for view in (pose, _pose(0.35, -22.0, -11.0, 9.0), _pose(0.48, 5.0, 20.0, -15.0)):
        points = _project(obj, view, calibration)
        ok, rvec, tvec = cv2.solvePnP(obj, points, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        first_errors.append(float(np.sqrt(np.mean(
            (projected.reshape(-1, 2) - points) ** 2))))

        solved = solve.pnp_tool_frame(obj, points, calibration)
        assert solved is not None
        assert solved[1] <= first_errors[-1] + 1e-6

    assert first_errors, "no IPPE solution to compare against"


def test_distortion_changes_the_projection():
    """Guard against the model silently becoming a pinhole again."""
    obj = _board_points()
    pose = _pose(0.35, 28.0, 12.0, 0.0)
    with_lens = _project(obj, pose, _Calibration())
    without = _project(obj, pose, _Pinhole())
    assert np.abs(with_lens - without).max() > 1.0
