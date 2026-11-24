"""A long /move must not freeze the console.

Regression for the 2026-11-24 IRL gate stall: ``POST /move`` was a bare
``async def`` wrapping the blocking ``Live.move``.  A cold h1 solve takes ~169 s
on least_squares, and for that whole time the event loop was parked inside the
handler, so even ``GET /state`` hung — the console looked dead and a client
polling it timed out.  The route now dispatches to a worker thread.

The test holds a /move request in flight (a stub that sleeps) and asserts
/state still answers promptly, then checks the same route still performs a real
jog through the sim bridge so the wrap is not just "returns fast, does nothing".
"""
from __future__ import annotations

import importlib
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest
import uvicorn

LIVE = Path(__file__).resolve().parents[1]
ROOT = LIVE.parent
sys.path.insert(0, str(LIVE))
sys.path.insert(0, str(LIVE / "tests"))

from test_app import sim_bridge  # noqa: E402,F401  (reuse the module fixture)
from chess_sim.env import ChessSimEnv  # noqa: E402  (fixture dependency)

TOKEN = "test-token"


def _post(url: str, body: dict, token: str = TOKEN, timeout: float = 60.0) -> dict:
    data = json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": "application/json",
                                              "x-bridge-token": token})
    with urllib.request.urlopen(request, timeout=timeout) as reply:
        return json.loads(reply.read().decode() or "{}")


def _get(url: str, timeout: float = 5.0) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as reply:
        return json.loads(reply.read().decode() or "{}")


@pytest.fixture(scope="module")
def console(sim_bridge, tmp_path_factory):
    """The real app, on a real port, against the sim bridge."""
    workdir = tmp_path_factory.mktemp("console")
    token_file = workdir / "token.txt"
    token_file.write_text(TOKEN)
    config_file = workdir / "live.json"
    config_file.write_text(json.dumps({
        "bridge_url": sim_bridge,
        "token_file": str(token_file),
        "calibration": str(ROOT / ".private/phase11d/calibration.json"),
        "plan": str(ROOT / "chess_sim/assets/marker_scan_plan.json"),
        "poses_file": str(workdir / "poses.json"),
        "aim_file": str(workdir / "aim.json"),
        "tip_center_file": str(workdir / "tip_center.json"),
        "capture_dir": str(workdir / "captures"),
        "recovery_log": str(workdir / "recovery_log.jsonl"),
        "target_overlay": False,
        "settle_s": 0.2,
        "pose_timeout_s": 20.0,
        "port": 0,
    }))
    import os
    previous = os.environ.get("LIVE_CONFIG")
    os.environ["LIVE_CONFIG"] = str(config_file)
    sys.modules.pop("app", None)
    module = importlib.import_module("app")

    config = uvicorn.Config(module.app, host="127.0.0.1", port=0,
                            log_level="error")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    deadline = time.time() + 20
    while not server.started and time.time() < deadline:
        time.sleep(0.05)
    assert server.started
    port = server.servers[0].sockets[0].getsockname()[1]
    yield {"url": f"http://127.0.0.1:{port}", "module": module}
    server.should_exit = True
    thread.join(timeout=5)
    if previous is None:
        os.environ.pop("LIVE_CONFIG", None)
    else:
        os.environ["LIVE_CONFIG"] = previous


def test_state_answers_while_a_long_move_is_in_flight(console):
    url, module = console["url"], console["module"]
    original = module.live.move
    started_move = threading.Event()

    def slow_move(body):
        started_move.set()
        time.sleep(3.0)
        return {"ok": True, "slow": True, "body": body}

    module.live.move = slow_move
    box: dict = {}

    def call() -> None:
        try:
            box["reply"] = _post(url + "/move",
                                 {"mode": "xyz", "x_mm": 0, "y_mm": 0,
                                  "z_mm": 80})
        except Exception as exc:                     # noqa: BLE001
            box["error"] = f"{type(exc).__name__}: {exc}"

    worker = threading.Thread(target=call, daemon=True)
    try:
        worker.start()
        assert started_move.wait(5.0), "the move never reached the handler"
        # The handler is now parked for 3 s. /state must not be parked with it.
        poll_started = time.time()
        state = _get(url + "/state", timeout=2.0)
        elapsed = time.time() - poll_started
        assert elapsed < 1.0, f"/state took {elapsed:.2f} s while a move was in flight"
        assert state["bridge"]["mode"] == "REMOTE"
        assert state["bridge"]["joints"], "no joints in /state"
        worker.join(timeout=15)
        assert "error" not in box, box.get("error")
        assert box["reply"]["slow"] is True
    finally:
        module.live.move = original


def test_the_same_route_still_moves_the_arm(console):
    """The threadpool wrap must not change motion: a real jog still travels."""
    url = console["url"]
    before = _get(url + "/state")["bridge"]["joints"]["shoulder_pan"]
    reply = _post(url + "/jog", {"joint": "shoulder_pan", "delta": 3.0})
    after = _get(url + "/state")["bridge"]["joints"]["shoulder_pan"]
    assert abs(after - before) >= 1.0, (reply, before, after)
