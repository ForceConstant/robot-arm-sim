"""Marker-coverage scan planning and board-offset reporting.

Phase 9's committed scan plan was selected to see every *square* of the playing
surface.  It was never scored against the four printed board markers, which sit
28 mm outside the corner squares at +/-152 mm.  On the current layout that plan
sees only the two far markers (ids 2 and 3), so the live board solve needs its
own pose set.

This module is pure geometry: a calibrated wrist camera, a marker layout and
measured joints, exactly what the BC-250 has at scan time.  It never accepts a
simulator camera, a board truth transform or a rendered frame, so the planner
and the live runtime share one definition of "this marker is usable".

The planner is deliberately conservative:

* the whole marker quad must be inside the frame with a pixel margin;
* the projected edge must be long enough to decode (a 40 mm DICT_4X4_50 marker
  is six modules across, so the Phase 9 pipeline's ~7 px/module is ~42 px);
* the arm's own silhouette must not overlap the quad (an authored-mesh
  occlusion mask, not a rendered shadow);
* a marker counts as covered only after it is seen in ``min_views`` poses, so a
  single bad frame cannot produce a confident board pose.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .fiducials import MARKER_IDS, MarkerLayout
from .vision_geometry import CameraCalibration

SCHEMA = "robot-arm-marker-scan-plan/1"
MIN_EDGE_PX = 36.0
PREFERRED_EDGE_PX = 44.0
EDGE_MARGIN_PX = 10.0
OCCLUSION_FRACTION = 0.02


@dataclass(frozen=True)
class MarkerObservation:
    """One marker as it would appear from one calibrated pose."""

    marker_id: int
    corners_px: tuple[tuple[float, float], ...]
    edge_px: float

    @property
    def centre_px(self) -> tuple[float, float]:
        return (float(np.mean([c[0] for c in self.corners_px])),
                float(np.mean([c[1] for c in self.corners_px])))


@dataclass(frozen=True)
class PoseCoverage:
    """A candidate pose and the markers it resolves well enough to decode."""

    name: str
    joints: tuple[float, ...]
    visible: tuple[int, ...]
    edge_px: tuple[float, ...]
    camera_position: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        if len(self.joints) != 6:
            raise ValueError("a scan pose needs six joint angles")
        if len(self.visible) != len(self.edge_px):
            raise ValueError("every visible marker needs one edge measurement")

    def edges(self) -> dict[int, float]:
        return dict(zip(self.visible, self.edge_px))


@dataclass(frozen=True)
class MarkerScanPlan:
    poses: tuple[PoseCoverage, ...]
    min_views: int = 2
    layout_offset_mm: float = 152.0
    note: str = ""
    # Per-marker fallback pose, keyed by marker id: {"name", "joints", "edge_px"}.
    recovery: dict[int, dict] = field(default_factory=dict)
    # The markers this plan is responsible for.  ``None`` keeps the old
    # behaviour (every id in the dictionary), which is only right for a plan
    # that covers a whole face and is why a ChArUco plan has to say so: the
    # mat tags 0..4 are not on the printed face and are not "missing".
    marker_ids: tuple[int, ...] | None = None

    def expected(self) -> tuple[int, ...]:
        return tuple(self.marker_ids) if self.marker_ids else MARKER_IDS

    def views_per_marker(self) -> dict[int, int]:
        counts = {marker_id: 0 for marker_id in self.expected()}
        for pose in self.poses:
            for marker_id in pose.visible:
                counts[marker_id] = counts.get(marker_id, 0) + 1
        return counts

    def complete(self) -> bool:
        return all(count >= self.min_views for count in self.views_per_marker().values())

    def missing(self) -> tuple[int, ...]:
        counts = self.views_per_marker()
        return tuple(sorted(m for m, count in counts.items() if count < self.min_views))


@dataclass(frozen=True)
class BoardOffset:
    """Board pose expressed in the arm-base frame, in millimetres and degrees."""

    x_mm: float
    y_mm: float
    yaw_deg: float
    edge_gap_mm: float
    board_half_mm: float

    def as_dict(self) -> dict[str, float]:
        return {"x_mm": round(self.x_mm, 3), "y_mm": round(self.y_mm, 3),
                "yaw_deg": round(self.yaw_deg, 4),
                "edge_gap_mm": round(self.edge_gap_mm, 3),
                "board_half_mm": round(self.board_half_mm, 3)}


def marker_corners_world(layout: MarkerLayout, marker_id: int, *,
                         origin_xy: tuple[float, float] = (0.0, 0.0),
                         yaw: float = 0.0,
                         marker_z: float = 0.0) -> np.ndarray:
    """Four marker corners in world metres, board-frame layout applied."""
    if not np.isfinite(marker_z):
        raise ValueError("marker height must be finite")
    half = layout.size_mm / 2000.0
    corners = np.asarray(layout.corners_mm(marker_id), dtype=float) / 1000.0
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    xy = corners @ rotation.T + np.asarray(origin_xy, dtype=float)
    return np.column_stack([xy, np.full(4, float(marker_z))])


def _overlap_fraction(mask: np.ndarray, quad: np.ndarray) -> float:
    """Fraction of the marker quad covered by an exclusion mask."""
    image = Image.new("1", (mask.shape[1], mask.shape[0]), 0)
    ImageDraw.Draw(image).polygon([tuple(p) for p in quad], fill=1)
    drawn = np.asarray(image, dtype=bool)
    area = int(drawn.sum())
    if area == 0:
        return 1.0
    return float((drawn & np.asarray(mask, dtype=bool)).sum()) / area


def observe_markers(calibration: CameraCalibration, joints, layout: MarkerLayout, *,
                    board_origin: tuple[float, float] = (0.0, 0.0),
                    board_yaw: float = 0.0,
                    marker_z: float,
                    margin_px: float = EDGE_MARGIN_PX,
                    min_edge_px: float = MIN_EDGE_PX,
                    occlusion: np.ndarray | None = None,
                    occlusion_fraction: float = OCCLUSION_FRACTION,
                    ) -> dict[int, MarkerObservation]:
    """Markers this pose can decode, keyed by marker id.

    ``occlusion`` is an HxW boolean silhouette for the same joints (for example
    ``SelfOcclusionModel.exclusion_mask``).  Passing ``None`` skips the arm
    check, which is only appropriate for a fixture that has no arm in view.
    """
    joints = tuple(float(v) for v in joints)
    if len(joints) != 6 or not np.isfinite(joints).all():
        raise ValueError("six finite joint angles required")
    width, height = calibration.width, calibration.height
    found: dict[int, MarkerObservation] = {}
    for marker_id in MARKER_IDS:
        try:
            world = marker_corners_world(layout, marker_id, origin_xy=board_origin,
                                         yaw=board_yaw, marker_z=marker_z)
        except (ValueError, KeyError):
            # MARKER_IDS spans both faces (mat tags 0..4 and printed ChArUco
            # 5..36) while a layout describes only one of them.  Asking a mat
            # layout for marker 5 used to raise and take the whole scan down,
            # which is the same crash ``object_points`` already guards against.
            continue
        pixels = [calibration.project(point) for point in world]
        if any(pixel is None for pixel in pixels):
            continue
        quad = np.asarray(pixels, dtype=float)
        if (quad[:, 0].min() < margin_px or quad[:, 1].min() < margin_px
                or quad[:, 0].max() > width - margin_px
                or quad[:, 1].max() > height - margin_px):
            continue
        edges = [float(np.linalg.norm(quad[i] - quad[(i + 1) % 4])) for i in range(4)]
        edge = float(np.mean(edges))
        if edge < min_edge_px:
            continue
        if occlusion is not None and _overlap_fraction(occlusion, quad) > occlusion_fraction:
            continue
        found[marker_id] = MarkerObservation(
            marker_id=marker_id,
            corners_px=tuple((float(x), float(y)) for x, y in quad),
            edge_px=edge)
    return found


def _too_close(a: PoseCoverage, b: PoseCoverage, separation_m: float) -> bool:
    if a.camera_position is None or b.camera_position is None:
        return False
    return (float(np.linalg.norm(np.asarray(a.camera_position, dtype=float)
                                 - np.asarray(b.camera_position, dtype=float)))
            < separation_m)


def select_poses(candidates: list[PoseCoverage], *,
                 min_views: int = 2,
                 marker_ids: tuple[int, ...] = tuple(MARKER_IDS),
                 min_separation_m: float = 0.025,
                 ) -> list[PoseCoverage]:
    """Greedy cover: every marker in ``min_views`` poses, then maximise margin.

    Ties on new coverage go to the pose whose weakest marker projects largest.
    A candidate that would only repeat a marker from essentially the same
    camera position is skipped, because two frames from one viewpoint share
    every failure mode; if no diverse candidate exists the cover falls back to
    the plain greedy result rather than failing.
    """
    if min_views < 1:
        raise ValueError("min_views must be positive")
    if min_separation_m < 0:
        raise ValueError("min_separation_m must be nonnegative")

    def cover(enforce_diversity: bool) -> list[PoseCoverage]:
        chosen: list[PoseCoverage] = []
        counts = {marker_id: 0 for marker_id in marker_ids}
        remaining = list(candidates)
        while any(count < min_views for count in counts.values()) and remaining:
            def score(pose: PoseCoverage) -> tuple[int, float]:
                gain = sum(1 for marker in pose.visible
                           if marker in counts and counts[marker] < min_views)
                weakest = min(pose.edges().get(marker, 0.0) for marker in pose.visible) \
                    if pose.visible else 0.0
                return gain, weakest
            eligible = [
                pose for pose in remaining
                if not enforce_diversity or not any(
                    _too_close(pose, picked, min_separation_m)
                    and set(pose.visible) & set(picked.visible)
                    for picked in chosen)]
            if not eligible:
                break
            best = max(eligible, key=score)
            if score(best)[0] <= 0:
                break
            chosen.append(best)
            remaining.remove(best)
            for marker in best.visible:
                if marker in counts:
                    counts[marker] += 1
        return chosen

    chosen = cover(True)
    counts = _counts(chosen)
    if any(counts.get(marker, 0) < min_views for marker in marker_ids):
        chosen = cover(False)
    return chosen


def plan_from_observations(entries: list[tuple[str, tuple[float, ...],
                                               dict[int, MarkerObservation]]], *,
                           min_views: int = 2,
                           layout_offset_mm: float = 152.0,
                           positions: dict[str, tuple[float, float, float]] | None = None,
                           min_separation_m: float = 0.025,
                           marker_ids: tuple[int, ...] | None = None) -> MarkerScanPlan:
    """Build a plan from already-scored poses (the offline tool's output)."""
    positions = positions or {}
    expected = tuple(marker_ids) if marker_ids else MARKER_IDS
    coverages = [
        PoseCoverage(name=name, joints=tuple(float(v) for v in joints),
                     visible=tuple(sorted(found)),
                     edge_px=tuple(found[m].edge_px for m in sorted(found)),
                     camera_position=positions.get(name))
        for name, joints, found in entries
    ]
    chosen = select_poses(coverages, min_views=min_views,
                          marker_ids=expected,
                          min_separation_m=min_separation_m)
    missing = [m for m, count in _counts(chosen, expected).items() if count < min_views]
    note = "" if not missing else f"uncovered markers: {missing}"
    return MarkerScanPlan(poses=tuple(chosen), min_views=min_views,
                          layout_offset_mm=float(layout_offset_mm), note=note,
                          marker_ids=tuple(marker_ids) if marker_ids else None)


def _counts(poses, marker_ids=MARKER_IDS) -> dict[int, int]:
    counts = {marker_id: 0 for marker_id in marker_ids}
    for pose in poses:
        for marker_id in pose.visible:
            counts[marker_id] = counts.get(marker_id, 0) + 1
    return counts


def refine_target(layout: MarkerLayout, marker_id: int, *,
                  board_origin: tuple[float, float] = (0.0, 0.0),
                  board_yaw: float = 0.0,
                  height_m: float = 0.24,
                  marker_z: float = 0.0) -> tuple[float, float, float]:
    """Camera target directly above a missing marker, for a recovery pose.

    The orchestrator still has to solve and safety-check the pose; this only
    says where to look.  Staying above the marker (rather than tilting across
    the board) keeps the recovery inside the same clearance envelope.
    """
    if not np.isfinite(height_m) or height_m <= 0:
        raise ValueError("height must be positive")
    corners = marker_corners_world(layout, marker_id, origin_xy=board_origin,
                                   yaw=board_yaw, marker_z=marker_z)
    centre = corners[:, :2].mean(axis=0)
    return (float(centre[0]), float(centre[1]), float(marker_z + height_m))


def board_offset(origin_xy: tuple[float, float], yaw: float, *,
                 base_xy: tuple[float, float],
                 board_half_mm: float,
                 base_yaw: float = 0.0) -> BoardOffset:
    """Board origin relative to the arm base, plus the base-to-edge gap.

    ``edge_gap_mm`` is positive when the base lies outside the board rectangle
    and negative when it lies inside.  It uses axis distance (the nearest edge
    line), not corner Euclidean distance, and is therefore an axis gap — the
    same quantity the layout note calls the "board-edge-to-base-frame" gap.
    """
    dx = (float(origin_xy[0]) - float(base_xy[0])) * 1000.0
    dy = (float(origin_xy[1]) - float(base_xy[1])) * 1000.0
    relative = float(yaw) - float(base_yaw)
    c, s = math.cos(-relative), math.sin(-relative)
    local_x = c * dx - s * dy
    local_y = s * dx + c * dy
    half = float(board_half_mm)
    signed = max(abs(local_x) - half, abs(local_y) - half)
    return BoardOffset(x_mm=dx, y_mm=dy,
                       yaw_deg=math.degrees(relative),
                       edge_gap_mm=signed, board_half_mm=half)


def save_plan(plan: MarkerScanPlan, path: str | Path, **extra) -> Path:
    payload = {
        "schema": SCHEMA,
        "min_views": plan.min_views,
        "layout_offset_mm": plan.layout_offset_mm,
        "note": plan.note,
        **({"marker_ids": list(plan.marker_ids)} if plan.marker_ids else {}),
        "poses": [{"name": pose.name, "joints": list(pose.joints),
                   "visible": list(pose.visible),
                   "edge_px": [round(v, 2) for v in pose.edge_px],
                   **({"camera_position": list(pose.camera_position)}
                      if pose.camera_position is not None else {})}
                  for pose in plan.poses],
        "views_per_marker": {str(k): v for k, v in plan.views_per_marker().items()},
        "recovery": {str(k): v for k, v in plan.recovery.items()},
        **extra,
    }
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, indent=2) + "\n")
    return target


def load_plan(path: str | Path) -> MarkerScanPlan:
    data = json.loads(Path(path).read_text())
    if data.get("schema") != SCHEMA:
        raise ValueError("unsupported marker scan plan")
    poses = []
    for item in data["poses"]:
        joints = tuple(float(v) for v in item["joints"])
        visible = tuple(int(v) for v in item["visible"])
        edges = tuple(float(v) for v in item["edge_px"])
        position = item.get("camera_position")
        poses.append(PoseCoverage(
            item.get("name", "?"), joints, visible, edges,
            tuple(float(v) for v in position) if position is not None else None))
    if not poses:
        raise ValueError("a marker scan plan needs at least one pose")
    recovery = {int(key): value
                for key, value in (data.get("recovery") or {}).items()}
    return MarkerScanPlan(poses=tuple(poses), min_views=int(data.get("min_views", 2)),
                          layout_offset_mm=float(data.get("layout_offset_mm", 152.0)),
                          note=str(data.get("note", "")), recovery=recovery,
                          marker_ids=(tuple(int(v) for v in data["marker_ids"])
                                      if data.get("marker_ids") else None))
