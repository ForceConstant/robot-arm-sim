"""Pure adapter: captured wrist frames -> :class:`chess_sim.wrist_scan.WristScanner`.

This module is the seam between the live console and the simulator-qualified
wrist perception stack.  It takes frames that were already captured, the
measured joints that produced them, and the same installation files the live
app loads, and returns the board registration plus 64 square predictions.  It
performs **no** HTTP, no motion, no bridge call and no hardware access, and it
is deliberately not wired into ``live/app.py`` yet: the caller records the
captures, this module only reasons about them.

References are always ``None``: the empty-board reference epoch needs a
capture of the same board with the same lighting, which the console does not
have until it owns a re-registration step.  ``benchmark_phase9 --no-references``
is the qualification for that path.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from chess_sim.charuco_face import CharucoLayout
from chess_sim.config import WRIST_ROLL_ZERO_SIM_DEG
from chess_sim.fiducials import MarkerLayout
from chess_sim.perception import CLASSES, PerceptionModel
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.wrist_scan import (PLAN_PATH, WRIST_MODEL_PATH, ScanFrame,
                                  ScanPose, ScanResult, WristScanner,
                                  load_plan)

CONFIG_PATH = Path(__file__).resolve().parent / "live_config.json"
SQUARE_CLASSES = CLASSES
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


@dataclass(frozen=True)
class ScanInputs:
    """Everything inference needs, resolved from files rather than a scene."""

    calibration: WristCalibration
    poses: tuple[ScanPose, ...]
    model: PerceptionModel
    square: float
    board_z: float
    marker_z: float
    layout: MarkerLayout | CharucoLayout
    config_path: Path


@dataclass(frozen=True)
class Capture:
    """One wrist frame and the measured joints that produced it."""

    pose: str
    frame: np.ndarray
    joints: tuple[float, ...]

    def __post_init__(self) -> None:
        joints = tuple(float(v) for v in self.joints)
        if len(joints) != 6 or not np.isfinite(joints).all():
            raise ValueError("a capture needs six finite measured joint angles")
        frame = np.asarray(self.frame)
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("a capture frame must be an RGB image")
        object.__setattr__(self, "joints", joints)
        object.__setattr__(self, "frame", frame)


def _as_capture(item: "Capture | Mapping[str, Any]") -> Capture:
    if isinstance(item, Capture):
        return item
    return Capture(pose=str(item["pose"]), frame=item["frame"],
                   joints=tuple(item["joints"]))


def load_config(config_path: str | Path = CONFIG_PATH) -> dict:
    """The live config, exactly as the console reads it (no defaults merged)."""
    return json.loads(Path(config_path).read_text())


def layout_for(config: Mapping[str, Any]) -> MarkerLayout | CharucoLayout:
    """The board's fiducial layout, chosen the way the live app chooses it."""
    square_mm = float(config.get("square_m", 0.028)) * 1000.0
    if str(config.get("layout_kind", "mat")) == "charuco":
        return CharucoLayout(square_mm=square_mm)
    return MarkerLayout(offset_mm=float(config["layout_offset_mm"]))


def load_inputs(config_path: str | Path = CONFIG_PATH,
                plan_path: str | Path = PLAN_PATH,
                model_path: str | Path = WRIST_MODEL_PATH) -> ScanInputs:
    """Resolve calibration, plan, model and board geometry from files.

    ``config_path`` is the live console's config; its ``calibration`` entry is
    the same file the app loads.  The plan and model default to the simulator
    assets the wrist path was qualified with.
    """
    config = load_config(config_path)
    calibration = WristCalibration.load(config["calibration"])
    return ScanInputs(
        calibration=calibration,
        poses=tuple(load_plan(plan_path)),
        model=PerceptionModel.load(model_path),
        square=float(config.get("square_m", 0.028)),
        board_z=float(config["board_z_m"]),
        marker_z=float(config.get("marker_z_m", config["board_z_m"])),
        layout=layout_for(config),
        config_path=Path(config_path),
    )


def scanner_for(inputs: ScanInputs, *, method: str = "markers",
                self_occlusion: bool = True) -> WristScanner:
    """Construct the scanner; construction is pure, no capture is required."""
    return WristScanner(
        inputs.calibration, list(inputs.poses), inputs.model,
        square=inputs.square, board_z=inputs.board_z, marker_z=inputs.marker_z,
        marker_layout=inputs.layout, method=method,
        self_occlusion=self_occlusion)


def scan_frames(captures: Iterable["Capture | Mapping[str, Any]"]) -> list[ScanFrame]:
    """Convert captures to the scanner's frame type, keeping pose and joints."""
    return [ScanFrame(frame=item.frame, joints=item.joints, timestamp=0.0,
                      pose=item.pose)
            for item in (_as_capture(value) for value in captures)]


def plan_pose_degrees(pose: ScanPose) -> tuple[float, ...]:
    """Simulator plan radians -> Pi-bridge degrees.

    The five arm joints are a plain radians conversion, except ``wrist_roll``:
    the sim joint is offset by -87.21 deg from the live zero, so the bridge
    value is ``sim - WRIST_ROLL_ZERO_SIM_DEG``.  The jaw is lerobot's 0..100.
    """
    from chess_sim import gripper

    values = [math.degrees(float(v)) for v in pose.joints]
    values[4] = math.degrees(float(pose.joints[4])) - WRIST_ROLL_ZERO_SIM_DEG
    span = float(gripper.JOINT_MAX - gripper.JOINT_MIN)
    values[5] = (float(np.clip((float(pose.joints[5]) - gripper.JOINT_MIN)
                               / span * 100.0, 0.0, 100.0)) if span > 0 else 0.0)
    return tuple(values)


def frames_from_captured(captured: Iterable[tuple[str, Any]]) -> list[ScanFrame]:
    """``ScanController.captured`` (degrees) -> scanner frames (radians).

    The live calibration's ``camera(joints)`` takes radians; the bridge and the
    capture record are in degrees, so this is the one place that converts.
    """
    frames: list[ScanFrame] = []
    for name, frame in captured:
        joints = tuple(math.radians(float(v)) for v in frame.joints_deg)
        frames.append(ScanFrame(frame=frame.image, joints=joints,
                                timestamp=float(frame.timestamp), pose=str(name)))
    return frames


def run_scan(captures: Sequence["Capture | Mapping[str, Any]"],
             inputs: ScanInputs | None = None, *,
             timestamp: float = 0.0) -> ScanResult:
    """Register the board and classify all 64 squares from the captures.

    Returns the scanner's :class:`ScanResult`: the registration (``None`` when
    the marker solve fails) and, when it succeeds, one prediction per square.
    A failed registration is a safe abstention, not an error.
    """
    if not captures:
        raise ValueError("a wrist scan needs at least one capture")
    resolved = inputs or load_inputs()
    frames = scan_frames(captures)
    return scanner_for(resolved).scan(frames, references=None,
                                      timestamp=float(timestamp))


def summarize(result: ScanResult) -> dict[str, Any]:
    """JSON-safe view of a scan result for receipts and console state."""
    registration = result.registration
    return {
        "registered": registration is not None,
        "registration": None if registration is None else {
            "origin_m": [float(registration.origin[0]),
                         float(registration.origin[1])],
            "yaw_rad": float(registration.yaw),
            "square_m": float(registration.square),
            "board_z_m": float(registration.board_z),
            "score": float(registration.score),
            "coverage": float(registration.coverage),
        },
        "frames_used": int(result.frames_used),
        "markers": [int(marker) for marker in result.markers],
        "note": result.note,
        "predictions": [
            {"square": int(p.square), "label": p.label,
             "confidence": float(p.confidence),
             "occupied": bool(p.occupied),
             "valid_fraction": float(p.valid_fraction)}
            for p in result.predictions
        ],
        "unknown_squares": [int(p.square) for p in result.predictions
                            if p.label == "unknown"],
    }
