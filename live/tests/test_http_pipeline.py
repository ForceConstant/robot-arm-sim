"""End-to-end transport test: real HTTP client against a simulator-backed bridge.

The in-process fakes prove the controller logic.  This test proves the bytes:
`HttpBridge` and `HttpFrames` talking JSON and base64 JPEG to a server that
speaks the same API as the Pi bridge, driving a MuJoCo scene.  It is the closest
thing to the live path that can run with the Pi switched off.
"""
from __future__ import annotations

import base64
import math
import sys
import threading
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
import pytest
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from chess_sim import fiducials  # noqa: E402
from chess_sim.config import Camera  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.marker_scan import MarkerLayout, load_plan  # noqa: E402
from chess_sim.product import current_layout, layout_env_config  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from bridge_client import HttpBridge, HttpFrames  # noqa: E402
from scan_controller import ScanController, plan_in_degrees  # noqa: E402

CALIBRATION = Path(__file__).resolve().parents[2] / ".private/phase11d/calibration.json"
PLAN = Path(__file__).resolve().parents[2] / "chess_sim/assets/marker_scan_plan.json"
NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
TOKEN = "test-token"


class Command(BaseModel):
    v: int = 1
    seq: int
    ttl_ms: int = 200
    joints: list[float]
    speed_scale: float = 0.3


class SimBridge:
    """Minimal stand-in for the Pi bridge: same endpoints, same units."""

    def __init__(self, env, rate_deg_s: float = 400.0) -> None:
        self.env = env
        self.rate = rate_deg_s
        self.last_seq = -1
        self.mode = "REMOTE"
        self.armed = True

    def joints(self) -> list[float]:
        return [float(np.degrees(v)) for v in self.env.data.qpos[self.env._joint_qpos]]

    def apply(self, target_deg: list[float]) -> None:
        target = np.radians(np.asarray(target_deg, dtype=float))
        current = self.env.data.qpos[self.env._joint_qpos].copy()
        step = math.radians(self.rate) * 0.1
        self.env.data.qpos[self.env._joint_qpos] = current + np.clip(
            target - current, -step, step)
        mujoco.mj_forward(self.env.model, self.env.data)

    def render_jpeg(self) -> bytes:
        frame = self.env.render(Camera.WRIST).copy()
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError("encode failed")
        return encoded.tobytes()


def build_app(env) -> FastAPI:
    app = FastAPI()
    sim = SimBridge(env)
    lock = threading.Lock()

    def auth(headers) -> None:
        if headers.get("x-bridge-token") != TOKEN:
            raise HTTPException(403, "bad token")

    # All endpoints are async so MuJoCo rendering stays on uvicorn's event-loop
    # thread; sync endpoints would be dispatched to a threadpool and the EGL
    # context would be created on, and then used from, different threads.
    @app.get("/state")
    async def state():
        with lock:
            return {"mode": sim.mode, "armed": sim.armed,
                    "joints": dict(zip(NAMES, sim.joints())),
                    "limits": {"shoulder_pan": [-120.0, 130.0],
                               "shoulder_lift": [-115.0, 115.0],
                               "elbow_flex": [-100.0, 100.0],
                               "wrist_flex": [-110.0, 105.0],
                               "wrist_roll": [-90.0, 65.0],
                               "gripper": [0.0, 100.0]}}

    @app.post("/command")
    async def command(body: Command, request: Request):
        auth(request.headers)
        with lock:
            if body.seq <= sim.last_seq:
                raise HTTPException(409, "stale seq")
            sim.last_seq = body.seq
            sim.apply(body.joints)
            return {"accepted": True, "seq": body.seq}

    @app.get("/capture")
    async def capture():
        with lock:
            jpeg = sim.render_jpeg()
            return {"jpeg_b64": base64.b64encode(jpeg).decode("ascii"),
                    "frame_ts": time.time(),
                    "joints": dict(zip(NAMES, sim.joints()))}

    return app


@pytest.fixture(scope="module")
def server():
    config = layout_env_config("printed", cameras=())
    env = ChessSimEnv(config)
    env.reset("8/8/8/8/8/8/8/8")
    app = build_app(env)
    config_uv = uvicorn.Config(app, host="127.0.0.1", port=0, log_level="error")
    srv = uvicorn.Server(config_uv)
    thread = threading.Thread(target=srv.run, daemon=True)
    thread.start()
    deadline = time.time() + 15
    while not srv.started and time.time() < deadline:
        time.sleep(0.05)
    assert srv.started
    port = srv.servers[0].sockets[0].getsockname()[1]
    yield f"http://127.0.0.1:{port}", env
    srv.should_exit = True
    thread.join(timeout=5)
    env.close()


def test_http_pipeline_returns_board_offset(server):
    base_url, env = server
    calibration = WristCalibration.load(CALIBRATION)
    bridge = HttpBridge(base_url, token=TOKEN)
    frames = HttpFrames(base_url, token=TOKEN)
    plan = plan_in_degrees(load_plan(PLAN))

    layout = MarkerLayout.for_board(224.0, 15.0, plane="mat", margin_mm=40.0)
    marker_z = env.board.table_top + fiducials.marker_height_above_table(env.board.mat)
    mount = current_layout()["mount"]
    base_xy = (mount["white_arm_base_frame_mm"][0] / 1000.0,
               mount["white_arm_base_frame_mm"][1] / 1000.0)

    ctrl = ScanController(
        bridge, frames,
        lambda joints_deg: calibration.camera(
            tuple(math.radians(v) for v in joints_deg)),
        plan, layout=layout, square_m=env.board.square, board_z_m=env.board.top,
        marker_z_m=marker_z, base_xy_m=base_xy,
        board_half_mm=env.board.width * 500.0,
        settle_s=0.2, pose_timeout_s=20.0, command_period_s=0.05)

    report = ctrl.run()
    assert report.ok, report.note
    assert report.views_used >= 6
    assert report.offset is not None
    # The board is nominal, so the offset is the layout invariant: 90 mm gap
    # and the centre 217 mm from the base in y.
    assert abs(report.offset.x_mm) <= 2.0
    assert abs(report.offset.y_mm - 217.0) <= 2.0
    assert abs(report.offset.edge_gap_mm - 90.0) <= 2.0
    assert all(v.reached and v.captured for v in report.visits)
