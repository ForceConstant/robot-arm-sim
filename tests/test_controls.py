import importlib

import pytest
from fastapi.testclient import TestClient

from robot_web.lab import Interrupted, Lab


@pytest.fixture
def lab(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOT_DATA_DIR", str(tmp_path))
    return Lab()


def test_pause_resume_preserves_in_flight_move(lab):
    lab.reset_requested = False
    lab.running = True
    lab.dirty = True
    lab.state.update(phase="moving", current_move="e2e4")
    lab.command("pause")
    lab.command("start")
    assert lab.running
    assert not lab.reset_requested
    assert lab.state["current_move"] == "e2e4"


def test_stop_then_start_requires_new_game(lab):
    lab.reset_requested = False
    lab.dirty = True
    lab.command("stop")
    with pytest.raises(Interrupted):
        lab._checkpoint()
    lab.command("start")
    assert lab.reset_requested


def test_running_configuration_is_rejected(lab):
    lab.running = True
    with pytest.raises(ValueError):
        lab.command("configure", mode="opening")


def test_running_suite_exclusively_owns_control(lab):
    lab.suite = {"phase": "running", "index": 0, "results": []}
    lab.running = True
    for action, options in (("start", {}), ("jog", {"dx": 0.002, "droll": 0.07}),
                            ("reset", {}), ("speed", {"speed": 2.0}),
                            ("suite", {"suite": "start"})):
        with pytest.raises(ValueError, match="exclusive control"):
            lab.command(action, **options)
    assert lab.suite["phase"] == "running"
    assert lab.running
    assert lab.jog_request is None
    # A stop is the one permitted command, so a real user can still cancel.
    lab.command("stop")
    assert lab.stop_requested


def test_api_control_bounds_and_sensor_contract(lab, monkeypatch):
    module = importlib.import_module("robot_web.app")
    monkeypatch.setattr(module, "lab", lab)
    lab.ready = True
    # No context manager: this API test must not start another physics worker.
    client = TestClient(module.app)
    assert client.post("/api/control", json={"action": "start"}).status_code == 403
    headers = {"X-Robot-Lab": "1"}
    assert client.post("/api/control", headers={**headers, "Origin": "http://evil.invalid"},
                       json={"action": "start"}).status_code == 403
    assert client.post("/api/control", headers=headers,
                       json={"action": "speed", "speed": 999}).status_code == 422
    # Teleop deltas are clamped: a step larger than the arm can absorb is a bug,
    # not a valid command.
    assert client.post("/api/control", headers=headers,
                       json={"action": "jog", "droll": 9}).status_code == 422
    assert client.post("/api/control", headers=headers,
                       json={"action": "jog", "dgrip": 9}).status_code == 422
    data = client.get("/api/observations").json()
    assert set(data["cameras"]) == {"white_wrist", "black_wrist"}
    assert "fen" not in data and "pieces" not in data
    assert client.post("/api/control", headers=headers,
                       json={"action": "configure", "observations": "all"}).status_code == 200
    assert "top" in client.get("/api/observations").json()["cameras"]
    assert client.get("/api/frame/unknown.jpg").status_code == 404
