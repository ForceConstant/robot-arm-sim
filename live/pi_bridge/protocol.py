"""Versioned joint-command protocol shared by the Pi bridge and the BC-250.

Pure Python and dependency-free so both sides import the same validation and
the same unit conversion.  Everything on the wire is degrees at the joint
interface; raw servo counts never leave the Pi.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

PROTOCOL_VERSION = 1
MAX_TTL_MS = 400
MIN_TTL_MS = 40
MAX_SPEED_SCALE = 0.3

JOINTS: tuple[str, ...] = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                           "wrist_flex", "wrist_roll", "gripper")
GRIPPER_RANGE = (0.0, 100.0)
COUNTS_PER_DEGREE = 4095.0 / 360.0


class ProtocolError(ValueError):
    """Rejected message; the caller maps it to HTTP 400/409."""


@dataclass(frozen=True)
class Command:
    seq: int
    ttl_ms: int
    joints: tuple[float, ...]
    speed_scale: float


@dataclass(frozen=True)
class Acceptance:
    """Result of applying deltas and limits to one command."""

    joints: dict[str, float]
    clamped: tuple[str, ...]
    max_step_deg: float


def parse_command(payload: dict) -> Command:
    if not isinstance(payload, dict):
        raise ProtocolError("command body must be an object")
    version = payload.get("v", PROTOCOL_VERSION)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version!r}")
    seq = payload.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise ProtocolError("seq must be a nonnegative integer")
    ttl = payload.get("ttl_ms", 150)
    if not isinstance(ttl, int) or isinstance(ttl, bool):
        raise ProtocolError("ttl_ms must be an integer")
    if not MIN_TTL_MS <= ttl <= MAX_TTL_MS:
        raise ProtocolError(f"ttl_ms must be within {MIN_TTL_MS}..{MAX_TTL_MS}")
    values = payload.get("joints")
    if not isinstance(values, (list, tuple)) or len(values) != len(JOINTS):
        raise ProtocolError(f"joints must be a list of {len(JOINTS)} angles")
    joints = []
    for name, value in zip(JOINTS, values):
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ProtocolError(f"{name} must be a number")
        value = float(value)
        if not math.isfinite(value):
            raise ProtocolError(f"{name} must be finite")
        joints.append(value)
    speed = payload.get("speed_scale", 0.2)
    if not isinstance(speed, (int, float)) or isinstance(speed, bool):
        raise ProtocolError("speed_scale must be a number")
    speed = float(speed)
    if not 0.0 < speed <= MAX_SPEED_SCALE:
        raise ProtocolError(f"speed_scale must be within 0..{MAX_SPEED_SCALE}")
    return Command(seq=seq, ttl_ms=ttl, joints=tuple(joints), speed_scale=speed)


def limits_from_calibration(data: dict, *, margin_deg: float = 2.0,
                            margin_by_joint: dict[str, float] | None = None
                            ) -> dict[str, tuple[float, float]]:
    """Per-joint degree limits from a lerobot calibration file.

    Counts are 12-bit across a full turn, so ``counts / 4095 * 360 - 180`` is
    the joint angle at that mechanical stop.  The margin keeps a command inside
    the stop even after a small calibration drift; ``margin_by_joint`` overrides
    it per joint (wrist_roll runs at the full ±90° window).  The gripper is
    commanded in lerobot's 0..100 units, so its counts are not converted.
    """
    limits: dict[str, tuple[float, float]] = {}
    for name in JOINTS:
        if name == "gripper":
            limits[name] = GRIPPER_RANGE
            continue
        entry = data.get(name)
        if not isinstance(entry, dict):
            raise ProtocolError(f"calibration is missing {name}")
        low = float(entry["range_min"]) / COUNTS_PER_DEGREE - 180.0
        high = float(entry["range_max"]) / COUNTS_PER_DEGREE - 180.0
        if not (math.isfinite(low) and math.isfinite(high)) or high <= low:
            raise ProtocolError(f"invalid calibrated range for {name}")
        margin = float((margin_by_joint or {}).get(name, margin_deg))
        low, high = low + margin, high - margin
        if high <= low:
            raise ProtocolError(f"calibrated range for {name} is too narrow")
        limits[name] = (low, high)
    return limits


def apply_target(command: Command, current: dict[str, float],
                 limits: dict[str, tuple[float, float]], *,
                 dt_s: float, max_joint_speed_deg_s: float) -> Acceptance:
    """Clamp one command to the joint envelope and the per-cycle step budget.

    Clamping is per joint and per cycle: a command far outside the envelope is
    walked in at the configured speed instead of being executed as a jump.  The
    caller keeps sending updated targets to continue the approach.
    """
    if dt_s <= 0:
        raise ProtocolError("dt must be positive")
    step = max_joint_speed_deg_s * float(command.speed_scale) * dt_s
    result: dict[str, float] = {}
    clamped: list[str] = []
    worst = 0.0
    for name, target in zip(JOINTS, command.joints):
        if name not in current:
            raise ProtocolError(f"current state is missing {name}")
        low, high = limits[name]
        here = float(current[name])
        # A measured pose can sit a degree outside the calibrated envelope
        # (calibration drift, a stop relaxed by hand).  Never pull such a joint
        # inward on the first command: widen the bounds to include where it is,
        # which still forbids moving further outward.
        low, high = min(low, here), max(high, here)
        bounded = min(max(target, low), high)
        if abs(bounded - target) > 0.01:
            clamped.append(name)
        delta = bounded - here
        if abs(delta) > step:
            bounded = here + math.copysign(step, delta)
            if name not in clamped:
                clamped.append(name)
        worst = max(worst, abs(bounded - here))
        result[name] = bounded
    return Acceptance(joints=result, clamped=tuple(clamped), max_step_deg=worst)
