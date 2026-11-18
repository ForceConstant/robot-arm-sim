"""Browser app smoke test against a simulator-backed bridge.

Proves the page loads, `/state` reaches the bridge, and the Find-board button's
endpoint drives a full scan through the real controller over HTTP.  The app is a
thin shell, so this is mostly a wiring test — but wiring is exactly what breaks
when nobody looks.
"""
from __future__ import annotations

import importlib
import json
import math
import sys
import threading
import time
from pathlib import Path

import mujoco  # noqa: F401 - imported for the env dependency
import numpy as np
import pytest
import uvicorn
from fastapi.testclient import TestClient

LIVE = Path(__file__).resolve().parents[1]
ROOT = LIVE.parent
sys.path.insert(0, str(LIVE))
sys.path.insert(0, str(LIVE / "tests"))

from test_http_pipeline import build_app as build_sim_app  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.product import layout_env_config  # noqa: E402


@pytest.fixture(scope="module")
def sim_bridge():
    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    env.reset("8/8/8/8/8/8/8/8")
    app = build_sim_app(env)
    config = uvicorn.Config(app, host="127.0.0.1", port=0, log_level="error")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    deadline = time.time() + 15
    while not server.started and time.time() < deadline:
        time.sleep(0.05)
    assert server.started
    port = server.servers[0].sockets[0].getsockname()[1]
    yield f"http://127.0.0.1:{port}"
    server.should_exit = True
    thread.join(timeout=5)
    env.close()


def test_page_loads_and_scan_runs(sim_bridge, tmp_path, monkeypatch):
    token = "test-token"
    token_file = tmp_path / "token.txt"
    token_file.write_text(token)
    config_file = tmp_path / "live.json"
    config_file.write_text(json.dumps({
        "bridge_url": sim_bridge,
        "token_file": str(token_file),
        "calibration": str(ROOT / ".private/phase11d/calibration.json"),
        "plan": str(ROOT / "chess_sim/assets/marker_scan_plan.json"),
        "settle_s": 0.2,
        "pose_timeout_s": 20.0,
    }))
    monkeypatch.setenv("LIVE_CONFIG", str(config_file))

    sys.modules.pop("app", None)
    module = importlib.import_module("app")
    client = TestClient(module.app)

    page = client.get("/")
    assert page.status_code == 200
    assert "Robot arm" in page.text

    state = client.get("/state").json()
    assert state["bridge"]["mode"] == "REMOTE"
    assert state["bridge"]["armed"] is True

    headers = {"x-bridge-token": token}
    assert client.post("/scan", headers=headers).status_code == 200

    deadline = time.time() + 90
    scan = None
    while time.time() < deadline:
        state = client.get("/state").json()
        if not state["scan_running"] and state["scan"] is not None:
            scan = state["scan"]
            break
        time.sleep(0.5)
    assert scan is not None, "scan never finished"
    assert scan["ok"], scan.get("note")
    assert abs(scan["offset"]["y_mm"] - 217.0) <= 2.0
    assert client.post("/scan", headers={"x-bridge-token": "wrong"}).status_code == 403
