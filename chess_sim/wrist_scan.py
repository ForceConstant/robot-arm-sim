"""Wrist-only board scan: multi-pose capture, registration and perception.

The scanner never sees an ``Env``, a FEN, a piece pose or a simulator camera.
It receives undistorted RGB frames, the measured joint angles that produced
each frame, a saved :class:`~chess_sim.wrist_calibration.WristCalibration`, and
an optional per-pose empty-board reference captured in the same epoch.

Registration uses the printed marker mat (:mod:`chess_sim.fiducials`).  The
checker-lattice fit in :mod:`chess_sim.board_registration` remains available as
an ablation and for marker-free rigs.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path

import numpy as np

from .board_registration import BoardRegistration, register_board
from .fiducials import estimate_board_pose, marker_plane_offset
from .perception import (CLASSES, PerceptionModel, SquarePrediction,
                         predict_board, square_features, board_patches)
from .vision_geometry import CameraCalibration
from .wrist_calibration import ReferenceEpoch, WristCalibration

PLAN_PATH = Path(__file__).resolve().parent / "assets" / "wrist_scan_plan.json"
WRIST_MODEL_PATH = Path(__file__).resolve().parent / "assets" / "perception" / "model_wrist.json"
SCHEMA = "robot-arm-wrist-scan-plan/1"


@dataclass(frozen=True)
class ScanPose:
    name: str
    joints: tuple[float, ...]

    def __post_init__(self) -> None:
        values = tuple(float(v) for v in self.joints)
        if len(values) != 6 or not np.isfinite(values).all():
            raise ValueError("a scan pose needs six finite joint angles")
        object.__setattr__(self, "joints", values)


def load_plan(path: str | Path = PLAN_PATH) -> list[ScanPose]:
    payload = json.loads(Path(path).read_text())
    if payload.get("schema") != SCHEMA:
        raise ValueError("unsupported wrist scan plan")
    poses = [ScanPose(item["name"], tuple(item["joints"])) for item in payload["poses"]]
    if len(poses) < 2:
        raise ValueError("a scan plan needs at least two poses")
    return poses


@dataclass(frozen=True)
class ScanFrame:
    frame: np.ndarray
    joints: tuple[float, ...]
    timestamp: float = 0.0
    pose: str = ""


@dataclass
class ScanResult:
    registration: BoardRegistration | None
    predictions: list[SquarePrediction] = field(default_factory=list)
    frames_used: int = 0
    markers: tuple[int, ...] = ()
    note: str = ""


def fuse_predictions(views: list[list[SquarePrediction]], *,
                     threshold: float = 0.75) -> list[SquarePrediction]:
    """Element-wise median of per-view class probabilities.

    A median (not a mean) rejects one view's confident mistake, which is what a
    single occluding arm produces.  Squares no view observed stay ``unknown``.
    """
    if not views:
        return []
    rows = np.full((len(views), 64, len(CLASSES)), np.nan)
    valid = np.zeros((len(views), 64), dtype=bool)
    for index, view in enumerate(views):
        for prediction in view:
            valid[index, prediction.square] = prediction.valid_fraction >= 0.45
            if valid[index, prediction.square]:
                rows[index, prediction.square] = prediction.probabilities
    result: list[SquarePrediction] = []
    for square in range(64):
        block = rows[valid[:, square], square]
        if len(block) == 0:
            result.append(SquarePrediction(square, "unknown", 0.0,
                                           (0.25, 0.25, 0.25, 0.25), 0.0))
            continue
        probabilities = np.median(block, axis=0)
        label = int(np.argmax(probabilities))
        confidence = float(probabilities[label])
        if confidence < threshold or CLASSES[label] == "unknown":
            label = CLASSES.index("unknown")
        result.append(SquarePrediction(
            square=square, label=CLASSES[label], confidence=confidence,
            probabilities=tuple(float(v) for v in probabilities), valid_fraction=1.0))
    return result


class WristScanner:
    """Calibrated wrist-only perception; constructed from files, not a scene."""

    def __init__(self, calibration: WristCalibration, poses: list[ScanPose],
                 model: PerceptionModel, *, square: float, board_z: float,
                 board_thickness: float = 0.0032,
                 method: str = "markers", min_markers: int = 2):
        if method not in ("markers", "checker"):
            raise ValueError("method must be 'markers' or 'checker'")
        self.calibration = calibration
        self.poses = list(poses)
        self.model = model
        self.square = float(square)
        self.board_z = float(board_z)
        self.board_thickness = float(board_thickness)
        self.method = method
        self.min_markers = int(min_markers)
        self.epoch = ReferenceEpoch()
        self._reference_cache: dict = {}
        self._reference_cache_key = None

    def cameras(self, joints) -> CameraCalibration:
        return self.calibration.camera(joints)

    def register(self, frames: list[ScanFrame]) -> BoardRegistration | None:
        views = [(self.calibration.rectify(item.frame), self.cameras(item.joints))
                 for item in frames]
        if self.method == "markers":
            return estimate_board_pose(
                views, self.square, self.board_z,
                marker_z=self.board_z + marker_plane_offset(self.board_thickness),
                min_markers=self.min_markers)
        return register_board(views[0][0], views[0][1], self.board_z,
                              prior=BoardRegistration((0.0, 0.0), 0.0, self.square,
                                                      self.board_z, 0.0, 1.0),
                              origin_search=0.02, yaw_search_deg=5.0,
                              coarse_step=0.003,
                              square_range=(self.square, self.square))

    def _reference_patches(self, references, registration, item):
        """Empty-reference patches per pose, cached for the current epoch."""
        if not references or item.pose not in references:
            return None
        key = (item.pose, self.epoch.epoch)
        if self._reference_cache_key != key:
            camera = self.cameras(item.joints)
            patches, _ = board_patches(references[item.pose], camera, registration)
            self._reference_cache[key] = patches
            self._reference_cache_key = key
        return self._reference_cache.get(key)

    def perceive(self, frames: list[ScanFrame], registration: BoardRegistration,
                 references: dict[str, np.ndarray] | None = None) -> ScanResult:
        views: list[list[SquarePrediction]] = []
        for item in frames:
            frame = self.calibration.rectify(item.frame)
            camera = self.cameras(item.joints)
            reference = self._reference_patches(references, registration, item)
            views.append(predict_board(
                frame, camera, registration, self.model,
                view="wrist", min_confidence=0.0,
                reference_patches=reference, recover_empty=False))
        fused = fuse_predictions(views, threshold=self.model.confidence_threshold)
        return ScanResult(registration=registration, predictions=fused,
                          frames_used=len(frames))

    def scan(self, frames: list[ScanFrame],
             references: dict[str, np.ndarray] | None = None,
             timestamp: float = 0.0) -> ScanResult:
        registration = self.register(frames)
        if registration is None:
            return ScanResult(None, [], 0, (), "registration failed")
        self.epoch.update(registration, self.calibration.calibration_id, timestamp)
        if self.epoch.references is not None and references is not None:
            self.epoch.references = {k: v for k, v in references.items()}
        return self.perceive(frames, registration, references)
