"""Printed fiducial mat for wrist-only board registration.

A thin printed mat sits under the board and carries four 40 mm ArUco markers
(DICT_4X4_50, ids 0-3) in a square around the playing field.  The mat is rigidly
attached to the board in the simulator and is intended to be taped to the real
board, so a marker corner is a directly measurable board-frame point.

Registration here is a geometry problem with no learned component and no
simulator lookup at inference: detect marker corners in one or more wrist
frames, solve the mat pose in each frame from the saved hand-eye calibration,
then refine one board pose (x, y, yaw) against every detected corner.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from .board_registration import BoardRegistration
from .vision_geometry import CameraCalibration

DICTIONARY_ID = cv2.aruco.DICT_4X4_50
MARKER_IDS: tuple[int, ...] = (0, 1, 2, 3)
MARKER_MM = 40.0
MARKER_OFFSET_MM = 152.0      # marker centre from the board centre, per axis
MAT_HALF_MM = 184.0           # mat plate half-width (board half is 127 mm)
MAT_THICKNESS_MM = 0.8
CELLS = 6                     # 1 black border + 4x4 payload... plus quiet ring
QUIET_MM = 6.0
MIN_MARKERS = 2
MAX_RMS_PX = 1.2


def _dictionary():
    return cv2.aruco.getPredefinedDictionary(DICTIONARY_ID)


@dataclass(frozen=True)
class MarkerSpec:
    marker_id: int
    centre_mm: tuple[float, float]

    @property
    def corners_mm(self) -> tuple[tuple[float, float], ...]:
        """Top-left, top-right, bottom-right, bottom-left in board axes."""
        h = MARKER_MM / 2.0
        cx, cy = self.centre_mm
        return ((cx - h, cy + h), (cx + h, cy + h),
                (cx + h, cy - h), (cx - h, cy - h))


MARKERS: tuple[MarkerSpec, ...] = (
    MarkerSpec(0, (-MARKER_OFFSET_MM, -MARKER_OFFSET_MM)),
    MarkerSpec(1, (MARKER_OFFSET_MM, -MARKER_OFFSET_MM)),
    MarkerSpec(2, (MARKER_OFFSET_MM, MARKER_OFFSET_MM)),
    MarkerSpec(3, (-MARKER_OFFSET_MM, MARKER_OFFSET_MM)),
)

# Marker plane sits this far below the playing surface: the mat is under the slab.
DEFAULT_BOARD_THICKNESS = 0.0032
MAT_THICKNESS_M = 0.0005


PRINT_RAISE_M = 0.00006      # marker ink sits this far above the mat face


def marker_height_above_table(mat_thickness: float = MAT_THICKNESS_M) -> float:
    """Printed marker face height above the table.

    The table is the datum for every board-relative measurement: the mat sits
    on it, and the markers print on the mat's top face.  No negative offsets
    relative to the playing surface are used anywhere.
    """
    if not np.isfinite(mat_thickness) or mat_thickness < 0:
        raise ValueError("mat thickness must be finite and non-negative")
    return mat_thickness + PRINT_RAISE_M


def marker_bits(marker_id: int) -> np.ndarray:
    """``CELLS x CELLS`` 0/1 bitmap of one marker (1 = white)."""
    image = cv2.aruco.generateImageMarker(_dictionary(), int(marker_id), CELLS)
    return (np.asarray(image) > 127).astype(np.int8)


def object_points() -> tuple[np.ndarray, np.ndarray]:
    """Flattened marker ids and their board-frame corner points (metres, z=0)."""
    ids: list[int] = []
    points: list[tuple[float, float, float]] = []
    for spec in MARKERS:
        for x_mm, y_mm in spec.corners_mm:
            ids.append(spec.marker_id)
            points.append((x_mm / 1000.0, y_mm / 1000.0, 0.0))
    return np.asarray(ids, dtype=np.int32), np.asarray(points, dtype=np.float64)


def mat_geoms(board_thickness: float = DEFAULT_BOARD_THICKNESS,
              mat_thickness: float = MAT_THICKNESS_M) -> list[dict]:
    """Scene primitives for the mat, expressed in the board body frame.

    Geometry, not a texture: a bitmap texture's orientation on a box face is
    not guaranteed, and a wrongly flipped mat would silently move every marker.
    The plate carries the board; markers print on the plate face around it.
    """
    if mat_thickness <= 0:
        raise ValueError("the fiducial mat needs a positive thickness")
    bottom = -board_thickness / 2.0
    plate_top = bottom
    geoms: list[dict] = [{
        "name": "marker_mat",
        "type": "box",
        "size": (MAT_HALF_MM / 1000.0, MAT_HALF_MM / 1000.0, mat_thickness / 2.0),
        "pos": (0.0, 0.0, plate_top - mat_thickness / 2.0),
        "rgba": (0.96, 0.96, 0.94, 1.0),
    }]
    cell = MARKER_MM / CELLS / 1000.0
    for spec in MARKERS:
        bits = marker_bits(spec.marker_id)
        cx, cy = spec.centre_mm[0] / 1000.0, spec.centre_mm[1] / 1000.0
        geoms.append({
            "name": f"marker_bg_{spec.marker_id}",
            "type": "box",
            "size": (MARKER_MM / 2000.0 + QUIET_MM / 2000.0,
                     MARKER_MM / 2000.0 + QUIET_MM / 2000.0, 0.00002),
            "pos": (cx, cy, plate_top + 0.00002),
            "rgba": (0.98, 0.98, 0.97, 1.0),
        })
        for row in range(CELLS):
            for col in range(CELLS):
                if bits[row, col]:
                    continue
                x = cx + (col - (CELLS - 1) / 2.0) * cell
                y = cy - (row - (CELLS - 1) / 2.0) * cell
                geoms.append({
                    "name": f"marker_{spec.marker_id}_{row}_{col}",
                    "type": "box",
                    "size": (cell / 2.0, cell / 2.0, 0.00002),
                    "pos": (x, y, plate_top + 0.00006),
                    "rgba": (0.04, 0.04, 0.04, 1.0),
                })
    return geoms


@dataclass
class MarkerDetection:
    marker_id: int
    corners: np.ndarray          # (4, 2) pixel corners, same order as the spec


@dataclass
class MarkerFit:
    """One frame's mat pose in world coordinates and its reprojection error."""

    registration: BoardRegistration
    rms_px: float
    marker_ids: tuple[int, ...]
    camera_position: tuple[float, float, float]


def _detector() -> "cv2.aruco.ArucoDetector":
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 35
    params.adaptiveThreshWinSizeStep = 6
    params.minMarkerPerimeterRate = 0.02
    params.maxMarkerPerimeterRate = 0.6
    params.errorCorrectionRate = 0.6
    return cv2.aruco.ArucoDetector(_dictionary(), params)


_DETECTOR = None


def detect_markers(frame: np.ndarray) -> list[MarkerDetection]:
    """Detect mat markers in one undistorted RGB frame."""
    global _DETECTOR
    if _DETECTOR is None:
        _DETECTOR = _detector()
    image = np.asarray(frame)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("frame must be H x W x 3 RGB")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = _DETECTOR.detectMarkers(gray)
    found: list[MarkerDetection] = []
    if ids is None:
        return found
    for marker_id, quad in zip(ids.ravel(), corners):
        if int(marker_id) not in MARKER_IDS:
            continue
        found.append(MarkerDetection(int(marker_id),
                                     np.asarray(quad, dtype=np.float64).reshape(4, 2)))
    return found


def _corner_map(detections: list[MarkerDetection]) -> dict[int, list[np.ndarray]]:
    """Marker corners sorted by id order of the mat spec, not detection order."""
    table: dict[int, list[np.ndarray]] = {}
    for detection in detections:
        table.setdefault(detection.marker_id, []).append(detection.corners)
    order = {spec.marker_id: index for index, spec in enumerate(MARKERS)}
    return dict(sorted(table.items(), key=lambda item: order.get(item[0], 99)))


def _project(pose, calibration: CameraCalibration, points: np.ndarray,
             marker_z: float) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x = pose[0] + c * points[:, 0] - s * points[:, 1]
    y = pose[1] + s * points[:, 0] + c * points[:, 1]
    world = np.stack([x, y, np.full_like(x, marker_z)], axis=1)
    rotation = np.asarray(calibration.camera_rotation).reshape(3, 3)
    cam = (world - np.asarray(calibration.camera_position)) @ rotation
    depth = -cam[:, 2]
    depth = np.where(depth > 1e-6, depth, 1e-6)
    return np.stack([calibration.cx + calibration.fx * cam[:, 0] / depth,
                     calibration.cy - calibration.fy * cam[:, 1] / depth], axis=1)


def estimate_board_pose(views: list[tuple[np.ndarray, CameraCalibration]],
                        square: float, board_z: float, *,
                        marker_z: float,
                        min_markers: int = MIN_MARKERS,
                        max_rms_px: float = MAX_RMS_PX,
                        search_mm: float = 40.0,
                        search_deg: float = 8.0) -> BoardRegistration | None:
    """Fuse marker corners from one or more frames into one board pose.

    ``views`` are ``(frame, calibration)`` pairs; the calibration carries the
    arm's measured joints, so no simulator state is used at inference.  A
    bounded coarse search followed by bounded refinement finds the pose; there
    is no simulator ground-truth seed.  Returns ``None`` when fewer than
    ``min_markers`` markers are detected or the joint reprojection error
    exceeds ``max_rms_px``.
    """
    if not np.isfinite(marker_z):
        raise ValueError("marker_z must be a finite world height")
    _, obj_points = object_points()
    # One marker contributes four consecutive corners; build the index from the
    # mat spec because the flattened id array repeats each id four times.
    corner_index = {spec.marker_id: 4 * index for index, spec in enumerate(MARKERS)}
    observations: list[tuple[CameraCalibration, np.ndarray, np.ndarray]] = []
    seen: set[int] = set()
    for frame, calibration in views:
        detections = detect_markers(frame)
        if not detections:
            continue
        object_list: list[np.ndarray] = []
        image_list: list[np.ndarray] = []
        for detection in detections:
            base = corner_index[detection.marker_id]
            object_list.extend(obj_points[base:base + 4])
            image_list.extend(detection.corners)
            seen.add(detection.marker_id)
        observations.append((calibration, np.asarray(object_list, dtype=np.float64),
                             np.asarray(image_list, dtype=np.float64)))
    if len(seen) < min_markers or not observations:
        return None

    def error(params: np.ndarray) -> np.ndarray:
        return np.concatenate([
            (_project(params, calibration, points, marker_z) - pixels).ravel()
            for calibration, points, pixels in observations])

    # Initial guess from per-view planar PnP; the coarse grid is only a fallback
    # when PnP fails, because a blind (x, y, yaw) search can settle on a
    # translated branch that still reprojects well with few markers.
    guesses: list[tuple[float, np.ndarray]] = []
    for calibration, points, pixels in observations:
        matrix = np.array([[calibration.fx, 0.0, calibration.cx],
                           [0.0, calibration.fy, calibration.cy],
                           [0.0, 0.0, 1.0]])
        try:
            ok, rvec, tvec = cv2.solvePnP(
                np.ascontiguousarray(points, dtype=np.float64),
                np.ascontiguousarray(pixels, dtype=np.float64),
                matrix, np.zeros(5), flags=cv2.SOLVEPNP_SQPNP)
        except cv2.error:
            continue
        if not ok:
            continue
        rotation, _ = cv2.Rodrigues(rvec)
        world_rotation = np.asarray(calibration.camera_rotation).reshape(3, 3) @ rotation
        world_position = (np.asarray(calibration.camera_position)
                          + np.asarray(calibration.camera_rotation).reshape(3, 3)
                          @ np.asarray(tvec).ravel())
        guess = np.array([world_position[0], world_position[1],
                          math.atan2(world_rotation[1, 0], world_rotation[0, 0])])
        rms = float(np.sqrt(np.mean(
            (_project(guess, calibration, points, marker_z) - pixels) ** 2)))
        guesses.append((rms, guess))
    span = search_mm / 1000.0
    yaw_span = math.radians(search_deg)
    if guesses:
        guesses.sort(key=lambda item: item[0])
        best = np.median(np.asarray([g for _, g in guesses[:4]]), axis=0)
    else:
        best = None
        best_cost = float("inf")
        for dx in np.linspace(-span, span, 9):
            for dy in np.linspace(-span, span, 9):
                for dyaw in np.linspace(-yaw_span, yaw_span, 9):
                    candidate = np.array([dx, dy, dyaw])
                    cost = float(np.mean(error(candidate) ** 2))
                    if cost < best_cost:
                        best_cost, best = cost, candidate
        if best is None:
            return None
    from scipy.optimize import least_squares
    lower = np.array([-span * 2, -span * 2, -yaw_span * 2])
    upper = np.array([span * 2, span * 2, yaw_span * 2])
    best = np.clip(np.asarray(best, dtype=float), lower + 1e-9, upper - 1e-9)
    result = least_squares(
        error, best, method="trf", xtol=1e-12, ftol=1e-12, gtol=1e-12,
        bounds=([-span * 2, -span * 2, -yaw_span * 2],
                [span * 2, span * 2, yaw_span * 2]))
    pose = result.x
    rms = float(np.sqrt(np.mean(result.fun ** 2)))
    if not np.isfinite(rms) or rms > max_rms_px:
        return None
    coverage = float(len(seen)) / len(MARKERS)
    return BoardRegistration(
        origin=(float(pose[0]), float(pose[1])),
        yaw=float(pose[2]),
        square=float(square),
        board_z=float(board_z),
        score=float(max(0.0, 1.0 - rms / max_rms_px)),
        coverage=coverage,
        edge_score=float(len(observations)) / max(1.0, len(views)),
        corner_pixels=(),
    )
