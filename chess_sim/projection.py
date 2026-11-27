"""Planar PnP and projection in the project's flipped camera frame.

The project carries camera-frame points with the optical axis along ``-z`` and
``+y`` up (the MuJoCo/OpenGL convention), so a projection is::

    depth = -z
    u = fx * x / depth + cx
    v = fy * (-y) / depth + cy

which is ``p_tool = diag(1, -1, -1) @ p_cv``.  OpenCV works in the standard
frame, so every crossing of that boundary applies ``FLIP`` exactly once.  This
module is the one place that crossing happens: one projective model, one PnP,
one residual convention.

Planar PnP note.  ``SOLVEPNP_IPPE`` returns two valid solutions for a planar
target.  Taking the first one produced a 32 deg outlier on a 10-marker view
(``reports/rot_scan_20261127.md``), so :func:`pnp_tool_frame` refines *both*
branches and keeps the lower reprojection residual.  A four-corner single
marker is nearly ambiguous in residual terms; callers that carry a stronger
physical prior (the board normal faces the camera) keep their own selector and
say so at the call site.
"""
from __future__ import annotations

from collections.abc import Mapping

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

#: OpenGL/MuJoCo camera axes <-> OpenCV camera axes.  Its own inverse.
FLIP = np.diag([1.0, -1.0, -1.0])


def _lens(calibration):
    """Normalise a lens calibration to ``(intrinsics, distortion)``.

    Accepts the saved ``WristCalibration`` (``.intrinsics``/``.distortion``),
    a plain mapping such as ``chess_sim.board_target.Board`` keeps, or any
    object exposing ``fx, fy, cx, cy``.  A mapping without ``distortion`` is a
    pinhole, which is what a caller that already undistorted its image wants.
    """
    if isinstance(calibration, Mapping):
        intrinsics = tuple(float(v) for v in calibration["intrinsics"])
        distortion = tuple(float(v) for v in calibration.get("distortion", (0.0,) * 5))
    elif hasattr(calibration, "intrinsics"):
        intrinsics = tuple(float(v) for v in calibration.intrinsics)
        distortion = tuple(float(v) for v in calibration.distortion)
    else:
        intrinsics = (float(calibration.fx), float(calibration.fy),
                      float(calibration.cx), float(calibration.cy))
        distortion = tuple(float(v) for v in getattr(calibration, "distortion",
                                                     (0.0,) * 5))
    if len(intrinsics) != 4 or len(distortion) != 5:
        raise ValueError("a lens needs four intrinsics and five distortion terms")
    return intrinsics, distortion


def camera_matrix(calibration) -> np.ndarray:
    """The 3x3 pinhole matrix ``K`` for a calibration of any supported shape."""
    fx, fy, cx, cy = _lens(calibration)[0]
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], float)


def distortion_vector(calibration) -> np.ndarray:
    """The Brown–Conrady coefficients as a float64 vector."""
    return np.asarray(_lens(calibration)[1], float)


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
            camera_matrix(calibration), distortion_vector(calibration))
        out[front] = projected.reshape(-1, 2)
    return out


def pnp_tool_frame(obj, img, calibration):
    """Planar PnP -> (T_tool_cam, residual).  Both IPPE branches are refined and
    the lower-residual one wins.

    ``obj`` is (N, 3) in the target frame, ``img`` is (N, 2) in pixels, and the
    returned transform maps target points into the flipped tool frame.  The
    residual is :func:`per_axis_rms` in pixels, computed with the same lens the
    fit used.  Returns None when no solution exists.
    """
    K = camera_matrix(calibration)
    dist = distortion_vector(calibration)
    obj = np.asarray(obj, float).reshape(-1, 3)
    img = np.asarray(img, float).reshape(-1, 2)
    try:
        ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if not ok or len(rvecs) == 0:
        return None
    best = None
    for rvec, tvec in zip(rvecs, tvecs):
        try:
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
        except cv2.error:
            continue
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        residual = per_axis_rms(projected.reshape(-1, 2), img)
        if not np.isfinite(residual):
            continue
        if best is None or residual < best[0]:
            best = (residual, rvec, tvec)
    if best is None:
        return None
    _, rvec, tvec = best
    mount = np.eye(4)
    mount[:3, :3] = FLIP @ Rotation.from_rotvec(np.ravel(rvec)).as_matrix()
    mount[:3, 3] = FLIP @ np.ravel(tvec)
    return mount, best[0]


def _paired_error(projected, observed) -> np.ndarray:
    """Per-point (dx, dy) with non-finite pairs dropped."""
    proj = np.asarray(projected, float).reshape(-1, 2)
    obs = np.asarray(observed, float).reshape(-1, 2)
    if proj.shape != obs.shape:
        raise ValueError(f"shape mismatch {proj.shape} vs {obs.shape}")
    error = proj - obs
    return error[np.isfinite(error).all(axis=1)]


def per_axis_rms(projected, observed) -> float:
    """RMS of one coordinate error: ``sqrt(mean over all 2N components of d^2)``.

    This is the convention ``tools/block5r_wide_solve.py`` has always reported
    (its docstring used to call it per-component).  For isotropic error it is
    ``sqrt(2)`` times smaller than :func:`per_component_rms`, so the two are
    **not interchangeable**.
    """
    error = _paired_error(projected, observed)
    if error.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(error ** 2)))


def per_component_rms(projected, observed) -> float:
    """RMS of the per-point error vector: ``sqrt(mean over N of dx^2 + dy^2)``.

    Vector-magnitude convention, used by tools that report "mean pixel error"
    per corner (``live/app.py`` derive, ``tools/calibrate_board_charuco.py``).
    It is ``sqrt(2)`` larger than :func:`per_axis_rms` for isotropic error.
    """
    error = _paired_error(projected, observed)
    if error.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))))
