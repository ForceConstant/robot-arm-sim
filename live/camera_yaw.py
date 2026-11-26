"""User-local camera yaw offset applied on top of the wrist hand-eye calibration.

The wrist-to-jaw joint can seat with a small residual rotation about the camera's
optical axis.  The pinned hand-eye calibration is a measured installation input
and is never rewritten at runtime, so the correction lives in its own file
(``live/camera_yaw.json``) and is applied wherever the dashboard builds a wrist
camera from measured joints.

Convention: camera axes follow the project's OpenGL/MuJoCo layout (+X right,
+Y up, -Z view).  The offset rotates the camera about its own optical axis
(camera-local +Z), so the world rotation is post-multiplied::

    R_world_camera' = R_world_camera @ Rz(offset)

A positive offset turns the image content clockwise on screen, which increases
the apparent in-plane yaw of the board's +x axis measured in image coordinates
(u right, v down).  Nulling a measured yaw therefore means
``offset_new = offset_old - measured``.
"""
from __future__ import annotations

from dataclasses import replace
import json
import math
import os
from pathlib import Path
import time

import numpy as np

#: File schema; a different value is treated as unreadable rather than migrated.
SCHEMA = "robot-arm-camera-yaw/1"

#: A mount yaw beyond this is a typo, not a calibration.  The file loader falls
#: back to zero instead of applying nonsense to every camera the app builds.
MAX_OFFSET_DEG = 45.0


def default_path() -> Path:
    return Path("live/camera_yaw.json")


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def load(path: str | Path | None = None) -> dict:
    """Read the offset file; a missing, corrupt or out-of-range file is zero."""
    location = Path(path) if path is not None else default_path()
    data: dict = {}
    if location.exists():
        try:
            data = json.loads(location.read_text())
        except (OSError, ValueError):
            data = {}
    if not isinstance(data, dict) or data.get("schema") != SCHEMA:
        data = {}
    try:
        offset = float(data.get("offset_deg", 0.0))
    except (TypeError, ValueError):
        offset = 0.0
    if not math.isfinite(offset) or abs(offset) > MAX_OFFSET_DEG:
        offset = 0.0
    return {"schema": SCHEMA,
            "offset_deg": round(offset, 3),
            "updated": data.get("updated"),
            "source": data.get("source")}


def save(path: str | Path, offset_deg: float, source: str = "manual",
         updated: str | None = None) -> dict:
    """Persist one offset; atomic so a crash cannot leave half a JSON file."""
    offset = float(offset_deg)
    if not math.isfinite(offset) or abs(offset) > MAX_OFFSET_DEG:
        raise ValueError(
            f"camera yaw offset must be finite and within "
            f"+/-{MAX_OFFSET_DEG:.0f} deg")
    location = Path(path)
    data = {"schema": SCHEMA,
            "offset_deg": round(offset, 3),
            "updated": updated or _now(),
            "source": str(source or "manual")}
    location.parent.mkdir(parents=True, exist_ok=True)
    temporary = location.with_name(location.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    os.replace(temporary, location)
    return dict(data)


def rotation_matrix(offset_deg: float) -> np.ndarray:
    """Rz in *camera* coordinates: the optical-axis rotation itself."""
    angle = math.radians(float(offset_deg))
    if not math.isfinite(angle):
        raise ValueError("camera yaw offset must be finite")
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[cos, -sin, 0.0],
                     [sin, cos, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


def apply(camera, offset_deg: float):
    """Return a copy of ``CameraCalibration`` rotated about its optical axis.

    The calibration object itself is never mutated: callers keep the pinned
    hand-eye result, and the yaw correction is layered on for this camera only.
    """
    offset = float(offset_deg)
    if not math.isfinite(offset):
        raise ValueError("camera yaw offset must be finite")
    if offset == 0.0:
        return camera
    axes = (np.asarray(camera.camera_rotation, dtype=float).reshape(3, 3)
            @ rotation_matrix(offset))
    return replace(camera, camera_rotation=tuple(float(v) for v in axes.ravel()))


def image_yaw_deg(project, half_span_mm: float) -> float | None:
    """Apparent in-plane rotation of the board x-axis, in degrees.

    ``project(xy_mm, z_mm)`` maps board millimetres to a pixel pair in image
    coordinates (u right, v down); it returns ``None`` when the point does not
    land in front of the lens.  The span is symmetric about the board centre so
    the angle comes from a real baseline, not from one point and the principal
    point.
    """
    span = float(half_span_mm)
    if not math.isfinite(span) or span <= 0.0:
        raise ValueError("half_span_mm must be finite and positive")
    left = project((-span, 0.0), 0.0)
    right = project((span, 0.0), 0.0)
    if left is None or right is None:
        return None
    dx = float(right[0]) - float(left[0])
    dy = float(right[1]) - float(left[1])
    if math.hypot(dx, dy) < 1e-6:
        return None
    return math.degrees(math.atan2(dy, dx))

def yaw_error_deg(measured_rotation, predicted_rotation) -> float:
    """Residual camera roll about the optical axis, in degrees.

    Both rotations are camera-from-board.  Their difference in camera
    coordinates is ``R_pred^T @ R_meas``; its z-rotation is the in-plane yaw.
    Positive means the measured camera is rolled one way about its view axis,
    and nulling it is ``offset -= error`` (see the module convention).
    """
    measured = np.asarray(measured_rotation, dtype=float).reshape(3, 3)
    predicted = np.asarray(predicted_rotation, dtype=float).reshape(3, 3)
    delta = predicted.T @ measured
    return math.degrees(math.atan2(float(delta[1, 0]), float(delta[0, 0])))

