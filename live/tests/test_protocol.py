"""Bridge protocol: units, envelope and per-cycle step budget."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pi_bridge"))

from protocol import (JOINTS, MAX_SPEED_SCALE, Command, ProtocolError,  # noqa: E402
                      apply_target, limits_from_calibration, parse_command)

CALIBRATION = {
    "shoulder_pan": {"range_min": 785, "range_max": 3451},
    "shoulder_lift": {"range_min": 807, "range_max": 3306},
    "elbow_flex": {"range_min": 925, "range_max": 3148},
    "wrist_flex": {"range_min": 835, "range_max": 3194},
    "wrist_roll": {"range_min": 1081, "range_max": 2759},
    "gripper": {"range_min": 1533, "range_max": 3272},
}

CURRENT = {"shoulder_pan": 0.0, "shoulder_lift": -100.0, "elbow_flex": 90.0,
           "wrist_flex": 70.0, "wrist_roll": 50.0, "gripper": 30.0}


def body(**overrides):
    payload = {"v": 1, "seq": 1, "ttl_ms": 150,
               "joints": [0.0, -100.0, 90.0, 70.0, 50.0, 30.0],
               "speed_scale": 0.2}
    payload.update(overrides)
    return payload


def test_parse_accepts_a_well_formed_command():
    command = parse_command(body())
    assert command.seq == 1
    assert command.joints == (0.0, -100.0, 90.0, 70.0, 50.0, 30.0)


@pytest.mark.parametrize("overrides", [
    {"v": 2},
    {"seq": -1},
    {"seq": True},
    {"ttl_ms": 0},
    {"ttl_ms": 1000},
    {"joints": [0.0] * 5},
    {"joints": [0.0, 0.0, 0.0, 0.0, 0.0, float("nan")]},
    {"joints": ["1", 0.0, 0.0, 0.0, 0.0, 0.0]},
    {"speed_scale": 0.0},
    {"speed_scale": MAX_SPEED_SCALE + 0.01},
])
def test_parse_rejects_bad_commands(overrides):
    with pytest.raises(ProtocolError):
        parse_command(body(**overrides))


def test_limits_come_from_the_calibration_counts():
    limits = limits_from_calibration(CALIBRATION)
    low, high = limits["shoulder_pan"]
    assert low == pytest.approx(785 / (4095 / 360) - 180 + 2, abs=0.01)
    assert high == pytest.approx(3451 / (4095 / 360) - 180 - 2, abs=0.01)
    assert limits["gripper"] == (0.0, 100.0)


def test_missing_calibration_joint_is_rejected():
    broken = dict(CALIBRATION)
    del broken["elbow_flex"]
    with pytest.raises(ProtocolError):
        limits_from_calibration(broken)


def test_target_inside_envelope_and_step_passes_through():
    limits = limits_from_calibration(CALIBRATION)
    command = parse_command(body(joints=[0.5, -100.0, 90.0, 70.0, 50.0, 30.0]))
    result = apply_target(command, CURRENT, limits, dt_s=1 / 30, max_joint_speed_deg_s=35)
    # 35 deg/s * 0.2 * 1/30 s = 0.233 deg per cycle, so 0.5 deg is walked in.
    assert result.joints["shoulder_pan"] == pytest.approx(35 * 0.2 / 30, abs=1e-9)
    assert "shoulder_pan" in result.clamped
    assert result.max_step_deg <= 35 * 0.2 / 30 + 1e-9


def test_small_command_is_exact():
    limits = limits_from_calibration(CALIBRATION)
    command = parse_command(body(joints=[0.1, -100.0, 90.0, 70.0, 50.0, 30.0]))
    result = apply_target(command, CURRENT, limits, dt_s=1 / 30, max_joint_speed_deg_s=35)
    assert result.joints["shoulder_pan"] == pytest.approx(0.1)
    assert result.clamped == ()


def test_target_outside_envelope_is_clamped_to_the_stop():
    limits = limits_from_calibration(CALIBRATION)
    command = parse_command(body(joints=[500.0, -100.0, 90.0, 70.0, 50.0, 30.0],
                                 speed_scale=0.3))
    # Walk 40 cycles toward the stop and confirm it never goes past it.
    state = dict(CURRENT)
    for _ in range(40):
        result = apply_target(command, state, limits, dt_s=1 / 30,
                              max_joint_speed_deg_s=35)
        state = result.joints
    assert state["shoulder_pan"] <= limits["shoulder_pan"][1] + 1e-9


def test_pose_outside_envelope_is_not_pulled_inward_on_hold():
    limits = limits_from_calibration(CALIBRATION)
    current = dict(CURRENT)
    current["elbow_flex"] = limits["elbow_flex"][1] + 1.6   # measured drift
    command = parse_command(body(joints=[current[name] for name in JOINTS]))
    result = apply_target(command, current, limits, dt_s=1 / 30,
                          max_joint_speed_deg_s=35)
    assert result.joints["elbow_flex"] == pytest.approx(current["elbow_flex"])
    assert "elbow_flex" not in result.clamped
    # Moving further outward is still refused.
    outward = dict(current)
    outward["elbow_flex"] += 5.0
    command = parse_command(body(joints=[outward[name] for name in JOINTS]))
    result = apply_target(command, current, limits, dt_s=10.0,
                          max_joint_speed_deg_s=1000.0)
    assert result.joints["elbow_flex"] == pytest.approx(current["elbow_flex"])


def test_gripper_is_kept_in_zero_to_one_hundred():
    limits = limits_from_calibration(CALIBRATION)
    command = parse_command(body(joints=[0.0, -100.0, 90.0, 70.0, 50.0, 250.0]))
    result = apply_target(command, CURRENT, limits, dt_s=10.0,
                          max_joint_speed_deg_s=1000.0)
    assert result.joints["gripper"] == 100.0


def test_command_reports_its_own_fields():
    command = Command(seq=7, ttl_ms=120, joints=(0.0,) * 6, speed_scale=0.1)
    assert command.seq == 7 and len(command.joints) == len(JOINTS)
