"""Loopback tests for the two-state bridge: nothing can wedge, teleop is immediate.

These drive ``Bridge._step`` and the public HTTP-facing methods against fake
serial devices, so they need no hardware and no Pi.  Each test is written as a
claim about the redesign:

(a) no reachable state refuses a recovery command — there is no latch, no
    armed handshake, and no mode a command cannot leave;
(b) TELEOP takes control immediately and follows every aligned joint while one
    joint is far out, then clicks that joint in when the leader comes near;
(c) a client whose ``seq`` is millions behind still gets its next command
    accepted (the 2026-11-24 63M-gap wedge);
(d) a dead client fails safe: the target expires and the arm holds.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pi_bridge"))

import app as bridge_app  # noqa: E402
from protocol import JOINTS, Command, ProtocolError  # noqa: E402

CALIBRATION = {
    "shoulder_pan": {"range_min": 785, "range_max": 3451},
    "shoulder_lift": {"range_min": 807, "range_max": 3306},
    "elbow_flex": {"range_min": 925, "range_max": 3148},
    "wrist_flex": {"range_min": 835, "range_max": 3194},
    "wrist_roll": {"range_min": 1081, "range_max": 2759},
    "gripper": {"range_min": 1533, "range_max": 3272},
}

POSE = {"shoulder_pan": 0.0, "shoulder_lift": -20.0, "elbow_flex": 40.0,
        "wrist_flex": 30.0, "wrist_roll": 0.0, "gripper": 30.0}


class FakeFollower:
    """Records every action; ``get_observation`` reads back what was sent."""

    def __init__(self, pose: dict[str, float]) -> None:
        self.pose = dict(pose)
        self.actions: list[dict[str, float]] = []
        self.fail_reads = False
        self.bus = type("Bus", (), {"connect": lambda self: None,
                                    "write_calibration": lambda self, cal: None,
                                    "enable_torque": lambda self, name: None,
                                    "disable_torque": lambda self, name: None,
                                    "read": lambda *a, **k: 0,
                                    "protocol_version": 0})()

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, float]:
        if self.fail_reads:
            raise RuntimeError("bus timeout")
        return {f"{name}.pos": value for name, value in self.pose.items()}

    def send_action(self, action: dict[str, float]) -> None:
        self.actions.append(dict(action))
        for key, value in action.items():
            self.pose[key[: -len(".pos")]] = value

    def disconnect(self) -> None:
        return None


class FakeLeader:
    def __init__(self, action: dict[str, float]) -> None:
        self.action = dict(action)
        self.bus = type("Bus", (), {"connect": lambda self: None,
                                    "write_calibration": lambda self, cal: None})()

    def configure(self) -> None:
        return None

    def get_action(self) -> dict[str, float]:
        return {f"{name}.pos": value for name, value in self.action.items()}

    def disconnect(self) -> None:
        return None


def _bridge(pose: dict[str, float] | None = None) -> bridge_app.Bridge:
    bridge = bridge_app.Bridge()
    limits = bridge_app.limits_from_calibration(
        CALIBRATION, margin_by_joint={"wrist_roll": 0.0})
    bridge.limits = limits
    bridge.joints = dict(pose or POSE)
    bridge.joints_ts = 1.0
    bridge.joints_source = "follower"
    bridge._entered = True  # skip the boot entry; the test opens devices itself
    return bridge


def _stub_hardware_openers(bridge: bridge_app.Bridge) -> None:
    """Replace the lerobot imports with no-ops; the fakes are already installed.

    ``_open_follower``/``_open_leader`` return early when a device is present,
    but they import lerobot before that check, and lerobot is not installed on
    the simulation host.  Stubbing the two openers is the seam that keeps these
    tests hardware-free while the transition logic itself runs for real.
    """
    bridge._open_follower = lambda: None          # type: ignore[method-assign]
    bridge._open_leader = lambda: bridge.leader   # type: ignore[method-assign]
    bridge._stop_lelab_teleop = lambda: None      # type: ignore[method-assign]


def _open_remote(bridge: bridge_app.Bridge, follower: FakeFollower) -> None:
    bridge.follower = follower
    bridge.mode = "REMOTE"
    bridge.hold = dict(bridge.joints)
    bridge.desired = None
    _stub_hardware_openers(bridge)


def _open_teleop(bridge: bridge_app.Bridge, follower: FakeFollower,
                 leader: FakeLeader) -> None:
    """Enter TELEOP through the real code path with fake devices installed.

    ``_open_follower``/``_open_leader`` return early when a device is already
    present, so installing the fakes first makes entry real without hardware.
    """
    bridge.follower = follower
    bridge.leader = leader
    bridge.joints = dict(follower.pose)
    _stub_hardware_openers(bridge)
    bridge._enter_teleop()


def _joints(pose: dict[str, float]) -> tuple[float, ...]:
    return tuple(pose[name] for name in JOINTS)


# ---------------------------------------------------------------- (a) no wedge
@pytest.mark.parametrize("condition", ["clean", "error", "estop",
                                       "error_and_estop", "stale_seq"])
def test_remote_accepts_a_command_after_any_reported_condition(condition):
    bridge = _bridge()
    _open_remote(bridge, FakeFollower(POSE))
    if condition in ("error", "error_and_estop"):
        bridge.error = "RuntimeError: bus timeout"
        bridge.error_at = 1.0
    if condition in ("estop", "error_and_estop"):
        bridge.estop = True
    seq = 1
    if condition == "stale_seq":
        bridge.last_seq = 63_000_000
        seq = 5
    result = bridge.submit(Command(seq=seq, ttl_ms=150,
                                   joints=_joints(POSE), speed_scale=0.2))
    assert result["accepted"] is True
    # And the condition is cleared, not latched.
    assert bridge.estop is False
    assert bridge.error is None


def test_a_bus_exception_is_reported_and_the_next_command_still_works():
    follower = FakeFollower(POSE)
    bridge = _bridge()
    _open_remote(bridge, follower)
    follower.fail_reads = True
    for i in range(3):                       # the follower is read every 3rd tick
        bridge._step(now=10.0 + i * 0.01)
    assert bridge.mode == "REMOTE"          # mode is untouched
    assert bridge.error and "bus timeout" in bridge.error
    follower.fail_reads = False
    assert bridge.submit(Command(seq=1, ttl_ms=150, joints=_joints(POSE),
                                 speed_scale=0.2))["accepted"]
    bridge._step(now=11.0)
    assert follower.actions, "a command still drives the arm after an error"


def test_teleop_can_be_left_for_remote_without_any_precondition():
    follower = FakeFollower(POSE)
    leader = FakeLeader({name: value + 80.0 for name, value in POSE.items()})
    bridge = _bridge()
    _open_teleop(bridge, follower, leader)      # wildly misaligned leader
    # No armed handshake, no "leader must match" gate: REMOTE is one request.
    bridge._transition("REMOTE")
    assert bridge.mode == "REMOTE"
    assert bridge.leader is None                # leader bus released


def test_estop_is_a_hold_not_a_latch():
    follower = FakeFollower(POSE)
    bridge = _bridge()
    _open_remote(bridge, follower)
    state = bridge.estop_request()
    assert state["estop"] is True and state["latched"] is False
    bridge._step(now=5.0)
    held = follower.actions[-1]
    assert held == {f"{name}.pos": POSE[name] for name in JOINTS}
    # A command resumes immediately; no /arm, no power cycle.
    target = dict(POSE)
    target["shoulder_pan"] += 0.2
    bridge.submit(Command(seq=9, ttl_ms=200, joints=_joints(target),
                          speed_scale=0.3))
    bridge._step(now=5.1)
    assert follower.actions[-1]["shoulder_pan.pos"] > POSE["shoulder_pan"]


def test_switch_mode_waiting_on_the_worker_reports_an_honest_failure():
    import threading
    import time as real_time

    bridge = _bridge()
    _open_remote(bridge, FakeFollower(POSE))

    def boom() -> None:
        raise RuntimeError("no leader port")

    bridge._enter_teleop = boom          # type: ignore[method-assign]
    box: dict = {}

    def waiter() -> None:
        box["result"] = bridge.switch_mode("TELEOP", timeout=5.0)

    thread = threading.Thread(target=waiter, daemon=True)
    thread.start()
    deadline = real_time.time() + 5.0
    while thread.is_alive() and real_time.time() < deadline:
        bridge._step(now=real_time.time())
        real_time.sleep(0.01)
    thread.join(timeout=2.0)
    result = box["result"]
    assert result["error"] and "leader port" in result["error"]
    # The failure is reported, not latched; the bridge is still REMOTE.
    assert bridge.mode == "REMOTE"
    assert bridge.desired is None
    assert bridge.submit(Command(seq=1, ttl_ms=150, joints=_joints(POSE),
                                 speed_scale=0.2))["accepted"]


# ------------------------------------------------------- (b) immediate teleop
def test_teleop_starts_immediately_and_follows_every_aligned_joint():
    follower = FakeFollower(POSE)
    leader_pose = dict(POSE)
    leader_pose["shoulder_pan"] = 3.0        # aligned (< 10 deg)
    leader_pose["elbow_flex"] = 43.0         # aligned
    leader_pose["wrist_roll"] = 60.0         # 60 deg out: must NOT block
    leader = FakeLeader(leader_pose)
    bridge = _bridge()
    _open_teleop(bridge, follower, leader)
    assert "wrist_roll" in bridge.click_in
    assert "shoulder_pan" not in bridge.click_in

    bridge._step(now=1.0)
    sent = follower.actions[-1]
    with bridge.lock:
        engaged = set(bridge.teleop_engaged)
    # Aligned joints follow at once...
    assert sent["shoulder_pan.pos"] > POSE["shoulder_pan"]
    assert sent["elbow_flex.pos"] > POSE["elbow_flex"]
    assert engaged >= {"shoulder_pan", "elbow_flex"}
    # ...and the far-out joint holds its pose instead of jumping or refusing.
    assert "wrist_roll" not in engaged
    assert sent["wrist_roll.pos"] == pytest.approx(POSE["wrist_roll"])


def test_the_out_of_spec_joint_clicks_in_once_the_leader_comes_close():
    follower = FakeFollower(POSE)
    leader_pose = dict(POSE)
    leader_pose["wrist_roll"] = 70.0
    leader = FakeLeader(leader_pose)
    bridge = _bridge()
    _open_teleop(bridge, follower, leader)
    bridge._step(now=1.0)
    assert "wrist_roll" in bridge.click_in
    # The operator brings the leader wrist near the follower's wrist.
    leader.action["wrist_roll"] = POSE["wrist_roll"] + 5.0
    bridge._step(now=1.1)
    with bridge.lock:
        assert "wrist_roll" in bridge.teleop_engaged
        assert "wrist_roll" not in bridge.click_in
    bridge._step(now=1.2)
    assert follower.actions[-1]["wrist_roll.pos"] > POSE["wrist_roll"]


def test_a_dead_leader_does_not_hide_behind_a_healthy_follower_read():
    follower = FakeFollower(POSE)
    leader = FakeLeader(dict(POSE))
    bridge = _bridge()
    _open_teleop(bridge, follower, leader)

    def broken() -> dict:
        raise RuntimeError("leader unplugged")

    leader.get_action = broken          # type: ignore[method-assign]
    for i in range(6):                  # follower reads succeed throughout
        bridge._step(now=3.0 + i * 0.01)
    assert "leader unplugged" in (bridge.error or "")
    assert bridge.state()["error_source"] == "leader"


def test_one_bad_leader_read_holds_and_does_not_end_teleop():
    follower = FakeFollower(POSE)
    leader = FakeLeader(dict(POSE))
    bridge = _bridge()
    _open_teleop(bridge, follower, leader)

    def broken() -> dict:
        raise RuntimeError("leader unplugged")

    leader.get_action = broken          # type: ignore[method-assign]
    bridge._step(now=2.0)
    assert bridge.mode == "TELEOP"      # still TELEOP, just holding
    assert "leader unplugged" in (bridge.error or "")
    leader.get_action = lambda: {f"{n}.pos": v for n, v in leader.action.items()}
    bridge._step(now=2.1)
    assert bridge.error is None


def test_a_lost_leader_is_reopened_in_teleop_without_a_mode_round_trip():
    follower = FakeFollower(POSE)
    leader = FakeLeader(dict(POSE))
    bridge = _bridge()
    _open_teleop(bridge, follower, leader)
    reopened = []
    bridge._open_leader = lambda: reopened.append(1) or leader  # type: ignore[assignment]
    bridge.leader = None                       # a USB drop mid-session
    bridge._step(now=5.0)
    assert reopened, "TELEOP must try to recover the leader by itself"
    assert bridge.mode == "TELEOP"


# ------------------------------------------------------------- (c) stale seq
def test_a_client_millions_behind_recovers_on_its_next_command():
    follower = FakeFollower(POSE)
    bridge = _bridge()
    _open_remote(bridge, follower)
    # A second console had driven the counter to 63M before this one restarted.
    bridge.last_seq = 63_000_000
    result = bridge.submit(Command(seq=3, ttl_ms=150, joints=_joints(POSE),
                                   speed_scale=0.2))
    assert result["accepted"] is True
    assert result["seq_reset"] is True
    assert bridge.seq_resets == 1
    # Its counter now climbs from 3 and every later command is accepted too.
    assert bridge.submit(Command(seq=4, ttl_ms=150, joints=_joints(POSE),
                                 speed_scale=0.2))["accepted"]


def test_duplicate_and_older_sequences_are_not_rejected():
    bridge = _bridge()
    _open_remote(bridge, FakeFollower(POSE))
    for seq in (10, 10, 9, 11):
        assert bridge.submit(Command(seq=seq, ttl_ms=150, joints=_joints(POSE),
                                     speed_scale=0.2))["accepted"]


def test_freshness_comes_from_arrival_time_not_the_counter():
    follower = FakeFollower(POSE)
    bridge = _bridge()
    _open_remote(bridge, follower)
    target = dict(POSE)
    target["shoulder_pan"] = POSE["shoulder_pan"] + 2.0
    bridge.submit(Command(seq=1, ttl_ms=100, joints=_joints(target),
                          speed_scale=0.3))
    bridge.command_ts = 100.0
    bridge._step(now=100.05)            # 50 ms in: fresh, it moves
    moved = follower.actions[-1]["shoulder_pan.pos"]
    assert moved > POSE["shoulder_pan"]
    bridge._step(now=100.3)             # 300 ms in: expired, it holds
    assert follower.actions[-1]["shoulder_pan.pos"] == pytest.approx(moved)


# ---------------------------------------------------------------- (d) dead client
def test_a_dead_client_fails_safe_on_ttl_expiry():
    follower = FakeFollower(POSE)
    bridge = _bridge()
    _open_remote(bridge, follower)
    target = dict(POSE)
    target["shoulder_pan"] = POSE["shoulder_pan"] + 5.0
    bridge.submit(Command(seq=1, ttl_ms=150, joints=_joints(target),
                          speed_scale=0.3))
    bridge.command_ts = 50.0
    for i in range(5):                  # a live client is transmitting
        bridge._step(now=50.02 * i)
    assert follower.actions[-1]["shoulder_pan.pos"] > POSE["shoulder_pan"]
    frozen = follower.actions[-1]["shoulder_pan.pos"]
    # The client dies.  Nothing is sent; after the TTL the bridge holds.
    bridge._step(now=50.5)
    assert follower.actions[-1]["shoulder_pan.pos"] == pytest.approx(frozen)


def test_only_remote_commands_are_refused_and_the_refusal_names_the_fix():
    bridge = _bridge()
    _open_remote(bridge, FakeFollower(POSE))
    bridge.mode = "TELEOP"
    with pytest.raises(ProtocolError) as caught:
        bridge.submit(Command(seq=1, ttl_ms=150, joints=_joints(POSE),
                              speed_scale=0.2))
    assert "REMOTE" in str(caught.value)   # says how to recover


# ------------------------------------------------------ HTTP loopback (real app)
@pytest.fixture()
def http_bridge(monkeypatch, tmp_path):
    """The real FastAPI app with fake devices, so the wiring is exercised too.

    The worker thread is started for real because it is what performs a mode
    transition, and ``/mode`` waits on it; a fake there would hide a timeout bug.
    """
    from fastapi.testclient import TestClient

    token = "loopback-token"
    token_file = tmp_path / "token.txt"
    token_file.write_text(token + "\n")
    monkeypatch.setattr(bridge_app, "TOKEN_FILE", token_file)
    monkeypatch.setenv("SO101_BRIDGE_TOKEN", token)

    bridge = bridge_app.Bridge()
    bridge.limits = bridge_app.limits_from_calibration(
        CALIBRATION, margin_by_joint={"wrist_roll": 0.0})
    bridge.joints = dict(POSE)
    bridge.joints_ts = 1.0
    bridge.joints_source = "follower"
    bridge._entered = True
    bridge._stop_lelab_teleop = lambda: None          # type: ignore[method-assign]
    bridge._open_follower = lambda: None              # type: ignore[method-assign]
    bridge._open_leader = lambda: bridge.leader       # type: ignore[method-assign]
    bridge._read_leader_action = staticmethod(         # type: ignore[assignment]
        lambda leader: dict(leader.action))

    follower = FakeFollower(POSE)
    leader = FakeLeader(dict(POSE))
    bridge.follower = follower
    bridge.leader = leader
    monkeypatch.setattr(bridge_app, "bridge", bridge)
    bridge.start()
    try:
        client = TestClient(bridge_app.app)
        client.headers.update({"x-bridge-token": token})
        yield client, bridge, follower, leader
    finally:
        bridge.stop.set()
        if bridge.thread is not None:
            bridge.thread.join(timeout=2.0)


def test_http_loopback_two_states_no_wedge(http_bridge):
    client, bridge, follower, leader = http_bridge

    # TELEOP takes over on the first request.
    assert client.post("/mode", json={"mode": "TELEOP"}).json()["mode"] == "TELEOP"

    # A command while TELEOP is refused, but the refusal names the recovery.
    refused = client.post("/command", json={
        "v": 1, "seq": 1, "ttl_ms": 150,
        "joints": list(_joints(POSE)), "speed_scale": 0.2})
    assert refused.status_code == 409
    assert "REMOTE" in refused.json()["detail"]

    # HOLD is accepted as an alias for REMOTE: an old console cannot strand it.
    assert client.post("/mode", json={"mode": "HOLD"}).json()["mode"] == "REMOTE"

    # A client 63M behind is accepted on its very next command.
    bridge.last_seq = 63_000_000
    accepted = client.post("/command", json={
        "v": 1, "seq": 2, "ttl_ms": 150,
        "joints": list(_joints(POSE)), "speed_scale": 0.2})
    assert accepted.status_code == 200
    assert accepted.json()["seq_reset"] is True

    # E-stop holds and does not latch; the next command resumes.
    stop = client.post("/estop", json={}).json()
    assert stop["estop"] is True and stop["latched"] is False
    resumed = client.post("/command", json={
        "v": 1, "seq": 3, "ttl_ms": 150,
        "joints": list(_joints(POSE)), "speed_scale": 0.2})
    assert resumed.status_code == 200
    assert client.get("/state").json()["estop"] is False


def test_http_loopback_teleop_click_in_is_visible_in_state(http_bridge):
    client, bridge, follower, leader = http_bridge
    leader.action["wrist_roll"] = POSE["wrist_roll"] + 50.0
    assert client.post("/mode", json={"mode": "TELEOP"}).json()["mode"] == "TELEOP"
    state = client.get("/state").json()
    assert state["mode"] == "TELEOP"            # immediate, no alignment gate
    assert "wrist_roll" in state["click_in"]
    assert "shoulder_pan" in state["teleop_engaged"]

    leader.action["wrist_roll"] = POSE["wrist_roll"] + 4.0
    deadline = time.time() + 3.0
    while time.time() < deadline:
        state = client.get("/state").json()
        if "wrist_roll" in state["teleop_engaged"]:
            break
        time.sleep(0.02)
    assert "wrist_roll" in state["teleop_engaged"]
    assert state["click_in"] == []


def test_http_loopback_bad_token_is_still_refused(http_bridge):
    client, _bridge, _follower, _leader = http_bridge
    bad = client.post("/mode", json={"mode": "REMOTE"},
                      headers={"x-bridge-token": "wrong"})
    assert bad.status_code == 403
    assert client.post("/command", json={"v": 1, "seq": 1, "ttl_ms": 150,
                                        "joints": list(_joints(POSE))},
                       headers={"x-bridge-token": "wrong"}).status_code == 403
