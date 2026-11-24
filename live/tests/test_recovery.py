"""Recovery primitive: a stuck arm resets to the stored home pose, once.

Owner's rule: "if the robot gets stuck, it should always just reset to home
position high above middle square and start again."  The pose is stored in
config, so recovery never needs the solver — the solver is one of the things
that gets stuck.

These tests cover the trigger, the single retry, the cap and the no-loop rule
without hardware, by driving the HTTP endpoints against the sim bridge.
"""
from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request
from pathlib import Path

import pytest

LIVE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LIVE))
sys.path.insert(0, str(LIVE / "tests"))

from test_console_responsive import console  # noqa: E402,F401  (fixture reuse)
from test_app import sim_bridge  # noqa: E402,F401  (console's dependency)

from kinematics import JOINTS  # noqa: E402

MOVE_BODY = {"mode": "xyz", "x_mm": 0.0, "y_mm": 0.0, "z_mm": 80.0}


def _call(url: str, body: dict, timeout: float = 60.0) -> tuple[int, dict]:
    data = json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": "application/json",
                                              "x-bridge-token": "test-token"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return reply.status, json.loads(reply.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        text = exc.read().decode(errors="replace")
        try:
            return exc.code, json.loads(text or "{}")
        except ValueError:
            return exc.code, {"detail": text}


@pytest.fixture()
def live_and_url(console):
    """The live object plus a restore point for every attribute a test patches."""
    live = console["module"].live
    saved = {name: getattr(live, name) for name in
             ("move", "recover_to_home", "_keep_in_problem")}
    saved_config = dict(live.config)
    try:
        yield live, console["url"], console["module"]
    finally:
        for name, value in saved.items():
            setattr(live, name, value)
        live.config.clear()
        live.config.update(saved_config)


def test_recovery_pose_is_a_stored_joint_pose(live_and_url):
    live, _url, _module = live_and_url
    pose = live.recovery_pose()
    assert len(pose) == 6
    assert all(isinstance(value, float) for value in pose)
    # High above the centre, on the play roll branch, gripper untouched.
    tip = live.kinematics.fk(pose) @ live.jaw_frame()
    board = live.kinematics.to_board(tip[:3, 3]) * 1000.0
    assert abs(board[0]) < 5.0 and abs(board[1]) < 5.0
    assert 100.0 < board[2] < 140.0


def test_stuck_move_recovers_once_then_retries(live_and_url):
    live, url, module = live_and_url
    calls = {"move": 0, "recover": 0}

    def flaky_move(body):
        calls["move"] += 1
        if calls["move"] == 1:
            raise module.StuckError("simulated stall: residual 79.8 deg")
        return {"ok": True, "recovered": True}

    def counting_recover(reason="manual"):
        calls["recover"] += 1
        return {"ok": True, "reason": reason}

    live.move = flaky_move
    live.recover_to_home = counting_recover

    status, reply = _call(url + "/move", MOVE_BODY)
    assert status == 200, reply
    assert reply["recovered"] is True
    assert calls == {"move": 2, "recover": 1}, calls


def test_second_failure_stops_without_looping(live_and_url):
    live, url, module = live_and_url
    calls = {"move": 0, "recover": 0}

    def always_stuck(body):
        calls["move"] += 1
        raise module.StuckError("simulated stall")

    def counting_recover(reason="manual"):
        calls["recover"] += 1
        return {"ok": True}

    live.move = always_stuck
    live.recover_to_home = counting_recover

    status, reply = _call(url + "/move", MOVE_BODY)
    assert status == 409, reply
    assert "simulated stall" in json.dumps(reply)
    assert calls["recover"] == 1, calls          # exactly one recovery
    assert calls["move"] == 2, calls             # original + one retry


def test_recovery_cap_bounds_the_loop(live_and_url):
    live, url, module = live_and_url
    live.config["recovery_max_attempts"] = 1
    live.config["move_retries_after_recovery"] = 9      # would loop without the cap
    calls = {"move": 0, "recover": 0}

    def always_stuck(body):
        calls["move"] += 1
        raise module.StuckError("simulated stall")

    def counting_recover(reason="manual"):
        calls["recover"] += 1
        return {"ok": True}

    live.move = always_stuck
    live.recover_to_home = counting_recover

    status, _reply = _call(url + "/move", MOVE_BODY)
    assert status == 409
    assert calls["recover"] <= 1, calls
    assert calls["move"] <= 2, calls


def test_move_landing_outside_keep_in_recovers(live_and_url):
    live, url, _module = live_and_url
    live.move = lambda body: {"ok": True}
    problems = iter(["keep-in zone: y 130 outside ±118", None])
    live._keep_in_problem = lambda joints: next(problems)
    calls = {"recover": 0}

    def counting_recover(reason="manual"):
        calls["recover"] += 1
        return {"ok": True}

    live.recover_to_home = counting_recover

    status, reply = _call(url + "/move", MOVE_BODY)
    assert status == 200, reply
    assert calls["recover"] == 1


def test_dry_run_never_recovers(live_and_url):
    live, url, module = live_and_url
    calls = {"recover": 0}

    def stuck(body):
        raise module.StuckError("no reachable pose: off by 25.05 mm / 9.01 deg")

    def counting_recover(reason="manual"):
        calls["recover"] += 1
        return {"ok": True}

    live.move = stuck
    live.recover_to_home = counting_recover
    status, _reply = _call(url + "/move", dict(MOVE_BODY, dry_run=True))
    assert status == 409
    assert calls["recover"] == 0


def test_recover_endpoint_uses_no_solver_and_no_gripper(live_and_url):
    live, url, module = live_and_url
    live._keep_in_problem = lambda joints: None       # sim geometry differs
    calls = {"solve": 0}

    def boom(*_args, **_kwargs):
        calls["solve"] += 1
        raise RuntimeError("solver must not be used for recovery")

    live.solver.solve_tip = boom
    live.solver.solve_multi = boom

    status, reply = _call(url + "/recover", {"reason": "test jam"})
    assert status == 200, reply
    assert reply["ok"] is True
    assert calls["solve"] == 0, "recovery routed through the solver"
    assert reply["target_joints"]["gripper"] == reply["from_joints"]["gripper"]

    # The arm is at the COMMANDED (limit-clamped) pose in joint space.  Checked
    # against the clamp-aware target, not the raw stored pose: the sim's
    # wrist_roll envelope is narrower than the real arm's, so the stored 87.7
    # legitimately clamps to 65 there and that clamp is the behaviour under test.
    commanded = [float(reply["target_joints"][name]) for name in JOINTS]
    stored = live.recovery_pose()
    assert max(abs(a - b) for a, b in zip(commanded, stored)) > 0.0  # clamp happened
    assert reply["clamped"], "expected the sim's narrower roll limit to be named"
    state = live.bridge.state()
    measured = [float(state["joints"][name]) for name in JOINTS]
    assert max(abs(a - b) for a, b in zip(measured, commanded)) <= 2.0
