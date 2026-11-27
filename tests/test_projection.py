"""tests/test_projection.py — the shared planar-PnP/projection module.

``chess_sim/projection.py`` is now the one implementation of the project's
flipped camera frame, planar PnP and the residual conventions.  These tests pin
the four things a caller is allowed to rely on:

* the projection is the documented formula (depth = -z, v = fy*(-y)/depth);
* a point behind the camera is nan, never a mirrored projection;
* a synthetic round trip recovers a known pose to better than 0.2 mm / 0.05 deg;
* the chosen IPPE branch is never worse than the first raw branch -- the defect
  that produced a 32 deg outlier on a 10-marker view (reports/rot_scan_20261127.md).

No hardware, no MuJoCo, no captured images.
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from chess_sim import projection
from chess_sim.projection import (FLIP, camera_matrix, per_axis_rms,
                                  per_component_rms, pnp_tool_frame,
                                  project_tool_frame)


class _Lens:
    intrinsics = (831.897, 833.014, 320.0, 240.0)
    distortion = (-0.102251, 0.685733, 0.001112, 0.003239, 0.0)


class _Pinhole(_Lens):
    distortion = (0.0, 0.0, 0.0, 0.0, 0.0)


def _points() -> np.ndarray:
    """Four ChArUco-ish square patches on a plane, metres, board frame."""
    out = []
    for cx, cy in [(0.0, 0.0), (0.20, 0.0), (0.0, 0.20), (0.20, 0.20)]:
        for dx, dy in [(-0.014, -0.014), (0.014, -0.014), (0.014, 0.014), (-0.014, 0.014)]:
            out.append([cx + dx, cy + dy, 0.0])
    return np.asarray(out, float)


def _pose(forward_m: float, tilt_deg: float, yaw_deg: float, roll_deg: float) -> np.ndarray:
    rotation = (Rotation.from_euler("z", yaw_deg, degrees=True)
                * Rotation.from_euler("x", tilt_deg, degrees=True)
                * Rotation.from_euler("z", roll_deg, degrees=True)).as_matrix()
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = [0.0, 0.0, -forward_m]      # depth is -z in this convention
    return pose


def _project(obj: np.ndarray, pose: np.ndarray, lens) -> np.ndarray:
    pts = (pose @ np.c_[obj, np.ones(len(obj))].T).T[:, :3]
    return project_tool_frame(pts, lens)


def test_projection_is_the_documented_formula():
    obj = _points()
    pose = _pose(0.42, 6.0, 3.0, 1.0)
    pts = (pose @ np.c_[obj, np.ones(len(obj))].T).T[:, :3]
    assert np.allclose(FLIP, np.diag([1.0, -1.0, -1.0]))
    fx, fy, cx, cy = _Pinhole.intrinsics
    depth = -pts[:, 2]
    expected = np.stack([fx * pts[:, 0] / depth + cx,
                         fy * (-pts[:, 1]) / depth + cy], axis=1)
    assert np.allclose(project_tool_frame(pts, _Pinhole()), expected, atol=1e-9)


def test_behind_the_camera_is_nan():
    obj = _points()
    projected = _project(obj, _pose(-0.42, 0.0, 0.0, 0.0), _Pinhole())
    assert np.isnan(projected).all()


def test_camera_matrix_and_lens_shapes():
    K = camera_matrix(_Lens())
    assert K.shape == (3, 3)
    assert K[0, 0] == pytest.approx(831.897)
    assert K[1, 1] == pytest.approx(833.014)
    # A mapping without distortion is a pinhole; a mapping with the key uses it.
    assert camera_matrix({"intrinsics": (1.0, 2.0, 3.0, 4.0)})[1, 1] == pytest.approx(2.0)
    assert projection.distortion_vector(
        {"intrinsics": (1.0, 2.0, 3.0, 4.0), "distortion": (0.1,) * 5})[0] == pytest.approx(0.1)


@pytest.mark.parametrize("forward,tilt,yaw,roll", [
    (0.30, 0.0, 0.0, 0.0),
    (0.43, 12.0, 4.0, 0.0),
    (0.38, 24.0, -9.0, 8.0),
    (0.52, 30.0, 14.0, -6.0),
])
def test_round_trip_recovers_a_known_pose(forward, tilt, yaw, roll):
    obj = _points()
    truth = _pose(forward, tilt, yaw, roll)
    image = _project(obj, truth, _Lens())
    assert np.isfinite(image).all()

    solved = pnp_tool_frame(obj, image, _Lens())
    assert solved is not None
    pose, residual = solved
    assert residual < 0.05, f"{residual:.4f} px on a noise-free projection"

    translation_mm = np.linalg.norm(pose[:3, 3] - truth[:3, 3]) * 1000.0
    rotation_deg = math.degrees(
        Rotation.from_matrix(truth[:3, :3].T @ pose[:3, :3]).magnitude())
    assert translation_mm < 0.2, f"{translation_mm:.3f} mm"
    assert rotation_deg < 0.05, f"{rotation_deg:.3f} deg"


def test_branch_is_never_worse_than_the_first_ippe_solution():
    obj = _points()
    calibration = _Lens()
    K = camera_matrix(calibration)
    dist = projection.distortion_vector(calibration)
    compared = 0
    for view in (_pose(0.40, 18.0, 7.0, 4.0), _pose(0.35, -22.0, -11.0, 9.0),
                 _pose(0.48, 5.0, 20.0, -15.0)):
        image = _project(obj, view, calibration)
        ok, rvec, tvec = cv2.solvePnP(obj, image, K, dist, flags=cv2.SOLVEPNP_IPPE)
        assert ok
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        first = per_axis_rms(projected.reshape(-1, 2), image)
        solved = pnp_tool_frame(obj, image, calibration)
        assert solved is not None
        assert solved[1] <= first + 1e-9, f"{solved[1]:.4f} > first branch {first:.4f}"
        compared += 1
    assert compared >= 2


def test_rms_conventions_differ_by_sqrt_two():
    """The two names are not interchangeable; the docstrings must stay true."""
    error = np.array([[1.0, 1.0], [-1.0, 1.0], [1.0, -1.0], [-1.0, -1.0]])
    observed = np.zeros_like(error)
    assert per_axis_rms(error, observed) == pytest.approx(1.0)
    assert per_component_rms(error, observed) == pytest.approx(math.sqrt(2.0))
    assert per_component_rms(error, observed) == pytest.approx(
        math.sqrt(2.0) * per_axis_rms(error, observed))


def test_rms_ignores_non_finite_pairs():
    observed = np.zeros((3, 2))
    projected = np.array([[0.0, 0.0], [np.nan, np.nan], [2.0, 2.0]])
    assert per_axis_rms(projected, observed) == pytest.approx(math.sqrt(2.0))
    assert per_component_rms(projected, observed) == pytest.approx(2.0)
    with pytest.raises(ValueError):
        per_axis_rms(projected[:2], observed)
