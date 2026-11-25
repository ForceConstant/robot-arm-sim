"""Scan orchestration: poses in, board pose and base offset out.

This is the piece that was missing between the plan and the fusion math.  It
drives a real (or simulated) arm through the marker plan, waits for each pose to
actually arrive, captures a frame with the joints that produced it, detects
markers, fuses one board pose, and reports the offset in the arm-base frame.

Two rules are enforced here rather than left to callers:

* **One unit at the boundary.**  Joints on the wire, in the plan and in the
  frame metadata are degrees, matching the Pi bridge.  Forward kinematics may
  want another unit; that conversion is an injected callable, not an assumption
  buried in the loop.
* **FK at capture.**  A frame is only accepted when the joint state returned with
  it matches the arm's current state within ``freshness_deg``.  A frame paired
  with "whatever pose the arm is in now" silently biases every corner.
* **No implicit motion authority.**  The controller never switches modes, never
  arms the bridge and never raises the speed cap.  If the bridge is not already
  in REMOTE and armed, it refuses and reports why.

The transport and the camera are protocols, so the same controller runs against
the Pi bridge and against a simulator-backed fake in tests.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Callable, Protocol, Sequence

import numpy as np

from chess_sim import fiducials
from chess_sim.board_registration import BoardRegistration
from chess_sim.fiducials import MARKER_IDS, MarkerLayout
from chess_sim.marker_scan import BoardOffset, MarkerScanPlan, board_offset
from chess_sim.vision_geometry import CameraCalibration
from chess_sim.wrist_calibration import WristCalibration

DEFAULT_POSE_TOLERANCE_DEG = 1.5
DEFAULT_FRESHNESS_DEG = 2.0
DEFAULT_SETTLE_S = 0.6
# A 90 deg reorientation at speed_scale 0.3 (10.5 deg/s * 0.3 = 3.15 deg/s)
# takes ~29 s; 12 s guaranteed a timeout on the first pose of a plan that
# starts far from the parked posture (block 5g: 16.3 deg short after 12 s).
DEFAULT_POSE_TIMEOUT_S = 45.0
DEFAULT_HOLD_S = 2.0
DEFAULT_CORRECTION_ROUNDS = 2
DEFAULT_CAPTURE_RETRIES = 2
DEFAULT_SPEED_SCALE = 0.3


class BridgeClient(Protocol):
    """The subset of the bridge API this controller is allowed to use."""

    def state(self) -> dict: ...

    def command(self, joints: Sequence[float], *, seq: int, ttl_ms: int,
                speed_scale: float) -> dict: ...


class FrameSource(Protocol):
    """One frame with the joints that produced it, and its capture time."""

    def capture(self) -> "Frame": ...


@dataclass(frozen=True)
class Frame:
    image: np.ndarray
    joints_deg: tuple[float, ...]
    timestamp: float


@dataclass
class PoseVisit:
    name: str
    target_deg: tuple[float, ...]
    reached: bool
    captured: bool
    markers: tuple[int, ...] = ()
    error_deg: float = float("inf")
    settle_s: float = 0.0
    note: str = ""


@dataclass
class ScanReport:
    ok: bool
    registration: BoardRegistration | None
    offset: BoardOffset | None
    visits: list[PoseVisit] = field(default_factory=list)
    views_used: int = 0
    markers_seen: tuple[int, ...] = ()
    recovery_used: tuple[int, ...] = ()
    duration_s: float = 0.0
    note: str = ""

    def as_dict(self) -> dict:
        return {
            "ok": self.ok,
            "board": None if self.registration is None else {
                "x_mm": round(self.registration.origin[0] * 1000, 3),
                "y_mm": round(self.registration.origin[1] * 1000, 3),
                "yaw_deg": round(math.degrees(self.registration.yaw), 4),
                "score": round(self.registration.score, 3),
            },
            "offset": None if self.offset is None else self.offset.as_dict(),
            "views_used": self.views_used,
            "markers_seen": list(self.markers_seen),
            "recovery_used": list(self.recovery_used),
            "duration_s": round(self.duration_s, 2),
            "note": self.note,
            "visits": [{"name": v.name, "reached": v.reached, "captured": v.captured,
                        "markers": list(v.markers), "error_deg": round(v.error_deg, 3),
                        "settle_s": round(v.settle_s, 2), "note": v.note}
                       for v in self.visits],
        }


def _sim_jaw_to_command(value: float) -> float:
    """Map the simulated jaw angle onto lerobot's 0..100 gripper command.

    The five arm joints are plain radians->degrees; the jaw is not.  This linear
    map is a *convention*: which end is open has not been verified on hardware,
    so treat a hardware plan's jaw values as provisional until checked.
    """
    from chess_sim import gripper

    span = float(gripper.JOINT_MAX - gripper.JOINT_MIN)
    if span <= 0:
        return float(np.clip(value, 0.0, 100.0))
    return float(np.clip((value - gripper.JOINT_MIN) / span * 100.0, 0.0, 100.0))


def plan_in_degrees(plan: MarkerScanPlan) -> MarkerScanPlan:
    """Convert a simulator-radian plan into bridge units (degrees, jaw 0..100).

    The plan files are authored against the MuJoCo model; the Pi bridge speaks
    degrees.  Doing this explicitly at the boundary is what keeps the controller
    from quietly commanding 0.36 deg when the pose meant 0.36 rad.
    """
    from chess_sim.marker_scan import PoseCoverage

    def convert(joints) -> tuple[float, ...]:
        values = [float(np.degrees(v)) for v in joints]
        values[5] = _sim_jaw_to_command(float(joints[5]))
        return tuple(values)

    return MarkerScanPlan(
        poses=tuple(PoseCoverage(pose.name, convert(pose.joints),
                                 pose.visible, pose.edge_px, pose.camera_position)
                    for pose in plan.poses),
        min_views=plan.min_views, layout_offset_mm=plan.layout_offset_mm,
        note=plan.note,
        recovery={marker: {**entry,
                           "joints": list(convert(entry["joints"]))}
                  for marker, entry in plan.recovery.items()})


class ScanController:
    """Execute a marker plan against the bridge and fuse the result."""

    def __init__(self, bridge: BridgeClient, frames: FrameSource,
                 camera_from_joints: Callable[[Sequence[float]], CameraCalibration],
                 plan: MarkerScanPlan, *,
                 layout: MarkerLayout, square_m: float, board_z_m: float,
                 marker_z_m: float, base_xy_m: tuple[float, float],
                 board_half_mm: float,
                 pose_tolerance_deg: float = DEFAULT_POSE_TOLERANCE_DEG,
                 freshness_deg: float = DEFAULT_FRESHNESS_DEG,
                 settle_s: float = DEFAULT_SETTLE_S,
                 pose_timeout_s: float = DEFAULT_POSE_TIMEOUT_S,
                 hold_s: float = DEFAULT_HOLD_S,
                 correction_rounds: int = DEFAULT_CORRECTION_ROUNDS,
                 capture_retries: int = DEFAULT_CAPTURE_RETRIES,
                 speed_scale: float = DEFAULT_SPEED_SCALE,
                 ttl_ms: int = 200,
                 command_period_s: float = 0.1,
                 min_markers: int = 2,
                 clock=time.monotonic,
                 sleep=time.sleep,
                 ) -> None:
        if not 0 < speed_scale <= 0.3:
            raise ValueError("speed_scale must be within (0, 0.3]")
        if not 40 <= ttl_ms <= 400:
            raise ValueError("ttl_ms must be within 40..400")
        if settle_s < 0 or pose_timeout_s <= settle_s:
            raise ValueError("invalid timing configuration")
        for pose in plan.poses:
            if any(abs(v) > 360.0 or not math.isfinite(v) for v in pose.joints):
                raise ValueError(
                    f"plan pose {pose.name!r} has implausible joint values for "
                    "degrees; convert the plan to bridge units before constructing "
                    "the controller")
        self.bridge = bridge
        self.frames = frames
        # The controller speaks bridge units (degrees).  The installation
        # forward-kinematics model may want radians, so the conversion lives in
        # this callable instead of being assumed here.
        self.camera_from_joints = camera_from_joints
        self.plan = plan
        self.layout = layout
        self.square_m = float(square_m)
        self.board_z_m = float(board_z_m)
        self.marker_z_m = float(marker_z_m)
        self.base_xy_m = (float(base_xy_m[0]), float(base_xy_m[1]))
        self.board_half_mm = float(board_half_mm)
        self.pose_tolerance_deg = float(pose_tolerance_deg)
        self.freshness_deg = float(freshness_deg)
        self.settle_s = float(settle_s)
        self.pose_timeout_s = float(pose_timeout_s)
        self.hold_s = float(hold_s)
        self.correction_rounds = int(correction_rounds)
        self.capture_retries = int(capture_retries)
        self.speed_scale = float(speed_scale)
        self.ttl_ms = int(ttl_ms)
        self.command_period_s = float(command_period_s)
        self.min_markers = int(min_markers)
        self.clock = clock
        self.sleep = sleep
        self._seq = 0
        # Accepted frames with the pose that produced them.  Consumers that
        # need the pixels (wrist perception) read them here; the marker scan
        # itself only keeps the calibration.
        self.captured: list[tuple[str, Frame]] = []

    # ------------------------------------------------------------------ public
    def run(self) -> ScanReport:
        started = self.clock()
        state = self.bridge.state()
        if state.get("mode") != "REMOTE":
            return ScanReport(False, None, None,
                              note=f"bridge is in {state.get('mode')}, not REMOTE")
        if not state.get("armed"):
            return ScanReport(False, None, None, note="bridge is not armed")
        # The bridge rejects any sequence that is not newer than the last one
        # it accepted, and that counter is session-global (it survives /move).
        # A controller built with ``_seq = 0`` could therefore never command a
        # second scan in the same session (block 5c probe hit exactly this:
        # "seq 1 is not newer than 1795020098210").
        try:
            self._seq = max(self._seq, int(state.get("last_seq") or 0))
        except (TypeError, ValueError):
            pass
        limits = state.get("limits") or {}
        if limits:
            checks = [(pose.name, pose.joints) for pose in self.plan.poses]
            checks += [(entry["name"], tuple(float(v) for v in entry["joints"]))
                       for entry in self.plan.recovery.values()]
            for name, joints in checks:
                for index, joint in enumerate(("shoulder_pan", "shoulder_lift",
                                               "elbow_flex", "wrist_flex",
                                               "wrist_roll", "gripper")):
                    low, high = limits.get(joint, (-1e9, 1e9))
                    value = joints[index]
                    if not low - 0.5 <= value <= high + 0.5:
                        return ScanReport(
                            False, None, None,
                            note=(f"pose {name!r} is outside {joint} limits "
                                  f"({value:.1f} vs {low:.1f}..{high:.1f}); "
                                  "plan units probably are not degrees"))

        visits: list[PoseVisit] = []
        views: list[tuple[np.ndarray, CameraCalibration]] = []
        seen: set[int] = set()

        for pose in self.plan.poses:
            visit, view = self._visit(pose.name, pose.joints)
            visits.append(visit)
            if view is not None:
                views.append(view)
                markers = fiducials.detect_markers(view[0])
                seen |= {d.marker_id for d in markers}
                visit.markers = tuple(sorted({d.marker_id for d in markers}))

        recovery_used = []
        if len(seen) < self.min_markers:
            for marker in MARKER_IDS:
                if marker in seen:
                    continue
                entry = self.plan.recovery.get(marker) if self.plan.recovery else None
                if entry is None:
                    continue
                visit, view = self._visit(f"recovery-{entry['name']}", entry["joints"])
                visits.append(visit)
                recovery_used.append(marker)
                if view is not None:
                    views.append(view)
                    markers = fiducials.detect_markers(view[0])
                    seen |= {d.marker_id for d in markers}
                    visit.markers = tuple(sorted({d.marker_id for d in markers}))

        registration = None
        if views:
            registration = fiducials.estimate_board_pose(
                views, self.square_m, self.board_z_m,
                marker_z=self.marker_z_m, layout=self.layout,
                min_markers=self.min_markers)

        duration = self.clock() - started
        if registration is None:
            return ScanReport(False, None, None, visits=visits,
                              views_used=len(views), markers_seen=tuple(sorted(seen)),
                              recovery_used=tuple(recovery_used), duration_s=duration,
                              note=f"board solve failed with {len(seen)} markers")
        offset = board_offset(registration.origin, registration.yaw,
                              base_xy=self.base_xy_m,
                              board_half_mm=self.board_half_mm)
        return ScanReport(True, registration, offset, visits=visits,
                          views_used=len(views), markers_seen=tuple(sorted(seen)),
                          recovery_used=tuple(recovery_used), duration_s=duration)

    # ---------------------------------------------------------------- internals
    def _joints_from_state(self, state: dict) -> tuple[float, ...]:
        joints = state.get("joints") or {}
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll", "gripper")
        try:
            return tuple(float(joints[name]) for name in names)
        except (KeyError, TypeError) as exc:
            raise RuntimeError(f"bridge state has no usable joints: {exc}") from exc

    def _error_deg(self, current: tuple[float, ...],
                   target: Sequence[float]) -> float:
        return max(abs(a - b) for a, b in zip(current, target))

    def _goto(self, name: str, target: Sequence[float]) -> tuple[bool, float, float]:
        deadline = self.clock() + self.pose_timeout_s
        reached_at = None
        error = float("inf")
        while True:
            self._seq += 1
            self.bridge.command(target, seq=self._seq, ttl_ms=self.ttl_ms,
                                speed_scale=self.speed_scale)
            self.sleep(self.command_period_s)
            state = self.bridge.state()
            current = self._joints_from_state(state)
            error = self._error_deg(current, target)
            if error <= self.pose_tolerance_deg:
                if reached_at is None:
                    reached_at = self.clock()
                elif self.clock() - reached_at >= self.settle_s:
                    return self._hold_and_correct(target, reached_at)
            else:
                reached_at = None
            if self.clock() >= deadline:
                # A joint that settles short (gravity/stiffness) never reaches
                # the band, so the correction must run on the timeout path too
                # or the pose is abandoned before it is ever nudged.
                _, final_error, _ = self._hold_and_correct(
                    target, reached_at if reached_at is not None else self.clock())
                return final_error <= self.pose_tolerance_deg, final_error, 0.0

    def _hold_and_correct(self, target: Sequence[float], reached_at: float):
        """Keep the target on the wire, then nudge it by the standing residual.

        A servo under load settles short of its setpoint and does not integrate
        the error away: repeating the same target for seconds changes nothing
        (block 5c, 2026-11-18 — the elbow held 1.57 deg short for a full 6 s).
        Commanding ``target + residual`` does move it, because the stiffness
        error is roughly proportional.  Alternating a hold with a correction
        took the same joint from 1.57 to 0.34 deg.  Neither the tolerance nor
        the acceptance gate is touched; this only makes the arm actually arrive.
        """
        error = float("inf")
        current = tuple(float(v) for v in target)
        command = [float(v) for v in target]
        limits = None
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll", "gripper")
        for round_index in range(self.correction_rounds + 1):
            hold_until = self.clock() + self.hold_s
            while self.clock() < hold_until:
                self._seq += 1
                self.bridge.command(command, seq=self._seq, ttl_ms=self.ttl_ms,
                                    speed_scale=self.speed_scale)
                self.sleep(self.command_period_s)
                state = self.bridge.state()
                current = self._joints_from_state(state)
                limits = state.get("limits") or limits
                error = self._error_deg(current, target)
            if error <= self.pose_tolerance_deg or round_index >= self.correction_rounds:
                break
            command = [c + (t - m) for c, t, m in zip(command, target, current)]
            if limits:
                command = [min(max(c, float(limits[n][0])), float(limits[n][1]))
                           for c, n in zip(command, names)]
        return True, error, self.clock() - reached_at

    def _visit(self, name: str, target: Sequence[float]):
        visit = PoseVisit(name=name, target_deg=tuple(float(v) for v in target),
                          reached=False, captured=False)
        started = self.clock()
        reached, error, settled = self._goto(name, target)
        visit.reached, visit.error_deg = reached, error
        visit.settle_s = settled
        if not reached:
            visit.note = "pose timeout"
            return visit, None
        for attempt in range(self.capture_retries + 1):
            try:
                frame = self.frames.capture()
            except Exception as exc:              # noqa: BLE001 - camera blips
                visit.note = f"capture failed: {exc}"
                self.sleep(self.command_period_s)
                continue
            state = self.bridge.state()
            current = self._joints_from_state(state)
            drift = self._error_deg(current, frame.joints_deg)
            if drift <= self.freshness_deg:
                visit.captured = True
                camera = self.camera_from_joints(frame.joints_deg)
                self.captured.append((name, frame))
                return visit, (frame.image, camera)
            visit.note = f"stale frame ({drift:.1f} deg from current pose)"
            self.sleep(self.command_period_s)
        visit.note = visit.note or "no fresh frame"
        visit.note += f" after {self.clock() - started:.1f}s"
        return visit, None
