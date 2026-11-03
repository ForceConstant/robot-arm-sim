"""Printed ArUco markers that give the board pose.

Four 40 mm ``DICT_4X4_50`` markers (ids 0-3) sit in the board border, one per
corner, centred in the band.  A 40 mm marker needs a 6.7 mm quiet zone, so the
border must be at least ~54 mm; a 55 mm border gives 7.5 mm.

Registration is pure geometry: detect marker corners, solve the mat/board pose
in each frame from the saved hand-eye calibration, then refine one board pose
(x, y, yaw) against every detected corner.  No simulator state is consulted.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np
from scipy.optimize import least_squares

from .board_registration import BoardRegistration
from .vision_geometry import CameraCalibration

DICTIONARY_ID = cv2.aruco.DICT_4X4_50
MARKER_IDS: tuple[int, ...] = (0, 1, 2, 3)
MARKER_MM = 40.0
QUIET_MM = 6.0                     # minimum white margin around a marker
CELLS = 6                          # 4 data modules + 1 border each side
MIN_MARKERS = 2
MAX_RMS_PX = 1.2
PRINT_RAISE_M = 0.00006            # ink sits this far above the printed face
DEFAULT_BOARD_THICKNESS = 0.0032
MAT_THICKNESS_M = 0.0005      # default mat when markers are mat-mounted

# Unit offsets per marker id: (-x,-y), (+x,-y), (+x,+y), (-x,+y).
_CENTRE_SIGNS = {0: (-1.0, -1.0), 1: (1.0, -1.0), 2: (1.0, 1.0), 3: (-1.0, 1.0)}


@dataclass(frozen=True)
class MarkerLayout:
    """Marker centres in board-frame millimetres.

    ``offset_mm`` is the distance from the board centre to each marker centre,
    on both axes: half the playing field plus half the border.
    """

    offset_mm: float
    size_mm: float = MARKER_MM
    quiet_mm: float = QUIET_MM

    @classmethod
    def in_border(cls, field_mm: float, border_mm: float,
                  size_mm: float = MARKER_MM) -> "MarkerLayout":
        """Markers centred in the border band, validated against the quiet zone."""
        if field_mm <= 0 or border_mm <= 0 or size_mm <= 0:
            raise ValueError("field, border and marker size must be positive")
        quiet = (border_mm - size_mm) / 2.0
        if quiet + 1e-9 < QUIET_MM:
            raise ValueError(
                f"a {size_mm:.0f} mm marker needs at least {QUIET_MM:.0f} mm of quiet "
                f"zone each side: border {border_mm:.0f} mm leaves {quiet:.1f} mm, "
                f"needs {size_mm + 2 * QUIET_MM:.0f} mm or more")
        return cls(field_mm / 2.0 + border_mm / 2.0, size_mm, quiet)

    @classmethod
    def for_board(cls, field_mm: float, border_mm: float, plane: str = "mat",
                  size_mm: float = MARKER_MM, margin_mm: float = 40.0) -> "MarkerLayout":
        """Layout for a board: in its border, or on a mat around it.

        ``plane="board"`` needs a border wide enough for the marker plus its
        quiet zones (>= 54 mm for a 40 mm marker).  ``plane="mat"`` puts the
        markers on a thin sheet outside the board, so the border width is
        irrelevant and edge squares keep their jaw clearance.
        """
        if plane == "board":
            return cls.in_border(field_mm, border_mm, size_mm)
        if plane != "mat":
            raise ValueError('marker plane must be "mat" or "board"')
        return cls(field_mm / 2.0 + margin_mm, size_mm, QUIET_MM)

    def centre_mm(self, marker_id: int) -> tuple[float, float]:
        if int(marker_id) not in _CENTRE_SIGNS:
            raise ValueError(f"unknown marker id {marker_id!r}")
        sx, sy = _CENTRE_SIGNS[int(marker_id)]
        return (sx * self.offset_mm, sy * self.offset_mm)

    def corners_mm(self, marker_id: int) -> tuple[tuple[float, float], ...]:
        """Top-left, top-right, bottom-right, bottom-left in board axes."""
        h = self.size_mm / 2.0
        cx, cy = self.centre_mm(marker_id)
        return ((cx - h, cy + h), (cx + h, cy + h),
                (cx + h, cy - h), (cx - h, cy - h))

    @property
    def outer_mm(self) -> float:
        """Half-extent of the printed marker footprint, quiet zone included."""
        return self.offset_mm + self.size_mm / 2.0 + self.quiet_mm


def marker_height_above_table(mat_thickness: float = 0.0) -> float:
    """Marker face height above the table when the markers print on a mat."""
    if not np.isfinite(mat_thickness) or mat_thickness < 0:
        raise ValueError("mat thickness must be finite and non-negative")
    return mat_thickness + PRINT_RAISE_M


def marker_bits(marker_id: int) -> np.ndarray:
    """``CELLS x CELLS`` 0/1 bitmap of one marker (1 = white)."""
    image = cv2.aruco.generateImageMarker(_dictionary(), int(marker_id), CELLS)
    return (np.asarray(image) > 127).astype(np.int8)


def _dictionary():
    return cv2.aruco.getPredefinedDictionary(DICTIONARY_ID)


def object_points(layout: MarkerLayout) -> tuple[np.ndarray, np.ndarray]:
    """Flattened marker ids and their board-frame corner points (metres, z=0)."""
    ids: list[int] = []
    points: list[tuple[float, float, float]] = []
    for marker_id in MARKER_IDS:
        for x_mm, y_mm in layout.corners_mm(marker_id):
            ids.append(marker_id)
            points.append((x_mm / 1000.0, y_mm / 1000.0, 0.0))
    return np.asarray(ids, dtype=np.int32), np.asarray(points, dtype=np.float64)


def marker_geoms(board_thickness: float, layout: MarkerLayout, *,
                 on_surface: bool = False, mat_thickness: float = 0.0) -> list[dict]:
    """Scene primitives for the markers, in the board body frame.

    ``on_surface`` prints into the board border, coplanar with the playing
    surface; otherwise the markers sit on a mat under the slab.  Geometry, not
    a texture: a texture's face orientation is not guaranteed, and a flipped
    mat would silently move every marker.
    """
    top = board_thickness / 2.0
    if on_surface:
        plane = top + 0.00002
        geoms: list[dict] = []
    else:
        if mat_thickness <= 0:
            raise ValueError("a mat-mounted layout needs a positive mat thickness")
        plane = -top + 0.00002
        geoms = [{
            "name": "marker_mat",
            "type": "box",
            "size": (layout.outer_mm / 1000.0, layout.outer_mm / 1000.0,
                     mat_thickness / 2.0),
            "pos": (0.0, 0.0, -top - mat_thickness / 2.0),
            "rgba": (0.96, 0.96, 0.94, 1.0),
        }]
    half = (layout.size_mm + 2 * layout.quiet_mm) / 2000.0
    cell = layout.size_mm / CELLS / 1000.0
    for marker_id in MARKER_IDS:
        bits = marker_bits(marker_id)
        cx, cy = (v / 1000.0 for v in layout.centre_mm(marker_id))
        geoms.append({
            "name": f"marker_bg_{marker_id}", "type": "box",
            "size": (half, half, 0.00002),
            "pos": (cx, cy, plane), "rgba": (0.98, 0.98, 0.97, 1.0),
        })
        for row in range(CELLS):
            for col in range(CELLS):
                if bits[row, col]:
                    continue
                x = cx + (col - (CELLS - 1) / 2.0) * cell
                y = cy - (row - (CELLS - 1) / 2.0) * cell
                geoms.append({
                    "name": f"marker_{marker_id}_{row}_{col}", "type": "box",
                    "size": (cell / 2.0, cell / 2.0, 0.00002),
                    "pos": (x, y, plane + 0.00004), "rgba": (0.04, 0.04, 0.04, 1.0),
                })
    return geoms


@dataclass
class MarkerDetection:
    marker_id: int
    corners: np.ndarray          # (4, 2) pixel corners, spec order


def _detector():
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
    """Detect markers in one undistorted RGB frame."""
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
                        marker_z: float, layout: MarkerLayout,
                        min_markers: int = MIN_MARKERS,
                        max_rms_px: float = MAX_RMS_PX,
                        search_mm: float = 40.0,
                        search_deg: float = 8.0) -> BoardRegistration | None:
    """Fuse marker corners from one or more frames into one board pose.

    ``views`` are ``(frame, calibration)`` pairs; the calibration carries the
    arm's measured joints, so no simulator state is used at inference.
    ``marker_z`` is the world height of the printed marker face, derived from
    the table datum.  Returns ``None`` below ``min_markers`` detected markers or
    above ``max_rms_px`` joint reprojection error.
    """
    if not np.isfinite(marker_z):
        raise ValueError("marker_z must be a finite world height")
    _, obj_points = object_points(layout)
    base_index = {marker_id: 4 * index for index, marker_id in enumerate(MARKER_IDS)}
    observations: list[tuple[CameraCalibration, np.ndarray, np.ndarray]] = []
    seen: set[int] = set()
    for frame, calibration in views:
        detections = detect_markers(frame)
        if not detections:
            continue
        object_list: list[np.ndarray] = []
        image_list: list[np.ndarray] = []
        for detection in detections:
            base = base_index[detection.marker_id]
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

    # Seed from per-view planar PnP; fall back to a bounded coarse search.
    guesses: list[tuple[float, np.ndarray]] = []
    for calibration, points, pixels in observations:
        matrix = np.array([[calibration.fx, 0.0, calibration.cx],
                           [0.0, calibration.fy, calibration.cy],
                           [0.0, 0.0, 1.0]])
        try:
            ok, rvec, tvec = cv2.solvePnP(points, pixels, matrix, np.zeros(5),
                                          flags=cv2.SOLVEPNP_SQPNP)
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

    lower = np.array([-span * 2, -span * 2, -yaw_span * 2])
    upper = np.array([span * 2, span * 2, yaw_span * 2])
    best = np.clip(np.asarray(best, dtype=float), lower + 1e-9, upper - 1e-9)
    result = least_squares(error, best, method="trf", xtol=1e-12, ftol=1e-12,
                           gtol=1e-12, bounds=(lower, upper))
    pose = result.x
    rms = float(np.sqrt(np.mean(result.fun ** 2)))
    if not np.isfinite(rms) or rms > max_rms_px:
        return None
    return BoardRegistration(
        origin=(float(pose[0]), float(pose[1])),
        yaw=float(pose[2]),
        square=float(square),
        board_z=float(board_z),
        score=float(max(0.0, 1.0 - rms / max_rms_px)),
        coverage=float(len(seen)) / len(MARKER_IDS),
        edge_score=float(len(observations)) / max(1.0, len(views)),
        corner_pixels=(),
    )
