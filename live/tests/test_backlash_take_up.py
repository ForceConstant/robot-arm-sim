"""Behaviour test for the backlash take-up primitive (no MuJoCo, no arm).

The primitive is two things: a *plan* (drive past the target by a bounded
margin, then back) and a *gate* (disabled by config, refused whenever the
overshoot would leave the joint envelope, the wrist window or the keep-in box).
The numeric proof that it improves the settle is the hardware A/B in
``tools/backlash_take_up_test.py``; this test pins the geometry, the guards and
the command order, which is what a future edit would break silently.

NOTE: importing ``app`` builds the mink arm model at import time from the rig's
calibration and the custom tool profile, so this module sets ROBOT_GRIPPER,
ROBOT_GRIPPER_ASSETS and LIVE_CONFIG (setdefault, never overriding).  Because
``chess_sim.gripper`` reads those at import, the choice sticks for the pytest
session: run this module on its own, or run the session with the same
environment the ``robot-live.service`` unit uses.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

LIVE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LIVE))
# ``app`` builds the mink model at import time from the rig's calibration and
# the custom tool profile.  The service unit supplies these three (see
# robot-live.service); a bare pytest run has to supply them too or the import
# raises KeyError: moving_jaw_so101_v1.  setdefault, so a caller can override.
os.environ.setdefault("ROBOT_GRIPPER", "cad_parallel")
os.environ.setdefault("ROBOT_GRIPPER_ASSETS",
                      str(LIVE.parent / ".private" / "gripper-assets-5y"))
os.environ.setdefault("LIVE_CONFIG", str(LIVE / "live_config.json"))

from app import JOINTS, Live  # noqa: E402


class FakeBridge:
    """A servo that lands exactly on its goal, and remembers every command."""

    def __init__(self, pose: list[float]) -> None:
        self.pose = list(pose)
        self.commands: list[list[float]] = []

    def state(self) -> dict:
        return {"mode": "REMOTE", "armed": True,
                "joints": {name: self.pose[i] for i, name in enumerate(JOINTS)}}

    def command(self, joints, **kwargs) -> dict:
        self.pose = [float(v) for v in joints]
        self.commands.append(list(self.pose))
        return {"ok": True}


class Harness:
    """``Live``'s take-up methods over a fake bridge and a plain config."""

    _stream_to = Live._stream_to
    _take_up_request = Live._take_up_request
    _stream_with_take_up = Live._stream_with_take_up
    _take_up_receipt = staticmethod(Live._take_up_receipt)

    def __init__(self, pose, config=None, limits=None, keep_in=None,
                 window=None) -> None:
        self.config = {"speed_scale": 0.25, **(config or {})}
        self.command_seq = 0
        self.bridge = FakeBridge(pose)
        self.frame_offset = SimpleNamespace(offset_deg=0.0)
        self._limits = limits or {}
        self._keep_in = keep_in
        self._window = window
        self.kinematics = SimpleNamespace(
            violations=lambda joints: [
                (name, round(float(joints[i]), 2), self._limits[name])
                for i, name in enumerate(JOINTS[:5])
                if name in self._limits and not
                self._limits[name][0] - 1e-6 <= float(joints[i]) <= self._limits[name][1] + 1e-6])

    def _wrist_window(self):
        return self._window

    def _keep_in_problem(self, joints):
        return self._keep_in(joints) if self._keep_in else None

    def _excess(self, joints):
        total = 0.0
        for index, name in enumerate(JOINTS[:5]):
            limits = self._limits.get(name)
            if not limits:
                continue
            value = float(joints[index])
            total += max(0.0, limits[0] - value, value - limits[1])
        return total


ANCHOR = [0.0, -30.0, 20.0, 60.0, 0.0, 30.0]


def test_plans_a_bounded_overshoot_past_the_target():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[2] += 10.0                      # elbow +10
    target[1] -= 2.0                       # shoulder_lift -2
    plan = Harness(start)._take_up_request(target, start)
    assert plan["applied"] is True
    assert "shoulder_lift" in [JOINTS[i] for i in plan["moving"]]
    assert plan["target"][2] == pytest.approx(target[2] + 3.5)
    assert plan["target"][1] == pytest.approx(target[1] - 3.5)
    # nothing that did not move gets an overshoot
    assert plan["target"][0] == pytest.approx(target[0])
    # and the gripper is never part of it
    assert plan["target"][5] == pytest.approx(target[5])


def test_disabled_means_disabled():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[2] += 10.0
    plan = Harness(start, {"backlash_take_up_deg": 0})._take_up_request(target, start)
    assert plan["applied"] is False
    assert "disabled" in plan["reason"]


def test_a_tiny_move_has_nothing_to_take_up():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[2] += 0.1
    plan = Harness(start)._take_up_request(target, start)
    assert plan["applied"] is False
    assert "no non-gripper travel" in plan["reason"]


def test_gripper_only_move_never_takes_up():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[5] += 10.0
    plan = Harness(start)._take_up_request(target, start)
    assert plan["applied"] is False


def test_overshoot_past_a_joint_limit_is_refused():
    limits = {"elbow_flex": (-10.0, 82.0)}
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[2] = 81.0                       # inside, overshoot to 84.5 is outside
    plan = Harness(start, limits=limits)._take_up_request(target, start)
    assert plan["applied"] is False
    assert "joint envelope" in plan["reason"]


def test_overshoot_outside_the_wrist_window_is_refused():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[4] = 88.0                       # window ends at 90, overshoot hits 91.5
    plan = Harness(start, window=(0.0, 90.0))._take_up_request(target, start)
    assert plan["applied"] is False
    assert "wrist roll" in plan["reason"]


def test_overshoot_outside_keep_in_is_refused():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[3] += 5.0
    harness = Harness(start, keep_in=lambda joints: "keep-in zone: x 130 outside ±118")
    plan = harness._take_up_request(target, start)
    assert plan["applied"] is False
    assert "keep-in" in plan["reason"]


def test_the_command_sequence_overshoots_then_lands_on_the_target():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[2] += 10.0
    harness = Harness(start)
    result = harness._stream_with_take_up(target, start)
    assert result["take_up"]["applied"] is True
    assert result["take_up"]["moving"] == ["elbow_flex"]
    assert result["joints"]["elbow_flex"] == pytest.approx(target[2], abs=0.3)
    # Some command went past the target by (nearly) the margin, and the last
    # command is the target itself -- not an accumulated lead beyond it.
    passed = max(c[2] for c in harness.bridge.commands)
    assert passed == pytest.approx(target[2] + 3.5, abs=0.75)
    assert harness.bridge.commands[-1][2] == pytest.approx(target[2], abs=1e-9)
    # And the whole sequence is longer than a direct approach: two legs.
    assert result["take_up"]["overshoot_commands"] >= 4
    assert result["take_up"]["final_commands"] >= 4


def test_refused_plan_still_moves_directly_and_says_why():
    start = list(ANCHOR)
    target = list(ANCHOR)
    target[2] += 10.0
    harness = Harness(start, {"backlash_take_up_deg": 0})
    result = harness._stream_with_take_up(target, start)
    assert result["take_up"]["applied"] is False
    assert result["joints"]["elbow_flex"] == pytest.approx(target[2], abs=0.3)
    assert max(c[2] for c in harness.bridge.commands) <= target[2] + 1e-9
