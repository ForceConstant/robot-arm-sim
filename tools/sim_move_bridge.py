#!/usr/bin/env python3
"""Sim-backed stand-in for the Pi bridge, with the failure modes injected.

The repo already has a rate-limited MuJoCo bridge stand-in
(`live/tests/test_http_pipeline.py::SimBridge`); this is the same idea as a
standalone server so a harness can talk to it over real HTTP, plus three knobs
that reproduce what the real servos do and the ways a move can silently fail:

``--sag shoulder_lift=-2.0``   achieved pose sits N deg off the command
``--deadband 0.3``             a step smaller than this is not taken at all
``--stall-above 25``           a command asking for more than this much travel
                               from the measured pose is accepted and ignored
``--stale-readback 0``         /state keeps returning the pose it first
                               reported (a frozen encoder readback)

The last one is the 2026-11-23 failure in hardware form: the console got a 200,
reported a small error, and nothing moved. A verification test that only reads
the response cannot see it; one that measures the joints can.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import sys
import threading
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import cv2  # noqa: E402
import mujoco  # noqa: E402
import uvicorn  # noqa: E402
from fastapi import FastAPI, HTTPException, Request  # noqa: E402
from pydantic import BaseModel  # noqa: E402

from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.product import layout_env_config  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")


class Command(BaseModel):
    v: int = 1
    seq: int
    ttl_ms: int = 200
    joints: list[float]
    speed_scale: float = 0.3


class ModeBody(BaseModel):
    mode: str


class SimMechanism:
    """Joint-target servo with the knobs a real follower has."""

    def __init__(self, env, *, rate_deg_s: float = 60.0, sag_deg: dict | None = None,
                 deadband_deg: float = 0.0, stall_above_deg: float | None = None,
                 stale_readback: bool = False, tick_s: float = 0.1) -> None:
        self.env = env
        self.rate = float(rate_deg_s)
        self.tick_s = float(tick_s)
        self.sag = [float(sag_deg.get(name, 0.0)) for name in NAMES] if sag_deg else [0.0] * 6
        self.deadband = float(deadband_deg)
        self.stall_above = stall_above_deg
        # A frozen readback: the mechanism keeps moving, /state never says so.
        self.stale_readback = bool(stale_readback)
        self._stale_pose: list[float] | None = None
        self.mode = "REMOTE"
        self.armed = True
        self.last_seq = -1
        self.goal: list[float] | None = None
        self.accepted = 0
        self.stalled = 0
        self.steps = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._tick, name="servo", daemon=True)
        self._thread.start()

    # -- state ------------------------------------------------------------
    def measured(self) -> list[float]:
        return [float(np.degrees(v)) for v in self.env.data.qpos[self.env._joint_qpos]]

    def _write(self, joints: list[float]) -> None:
        self.env.data.qpos[self.env._joint_qpos] = np.radians(np.asarray(joints, float))
        mujoco.mj_forward(self.env.model, self.env.data)

    # -- servo ------------------------------------------------------------
    def _tick(self) -> None:
        step = self.rate * self.tick_s
        while not self._stop.is_set():
            goal = self.goal
            if goal is not None:
                current = self.measured()
                command = []
                for here, want in zip(current, goal):
                    delta = want - here
                    if abs(delta) < self.deadband:
                        delta = 0.0
                    command.append(here + max(-step, min(step, delta)))
                self._write(command)
                self.steps += 1
            time.sleep(self.tick_s)

    def submit(self, body: Command) -> dict:
        if self.mode != "REMOTE" or not self.armed:
            raise HTTPException(409, f"mode {self.mode}, armed {self.armed}")
        if body.seq <= self.last_seq:
            raise HTTPException(409, f"seq {body.seq} not newer than {self.last_seq}")
        self.last_seq = body.seq
        wanted = [float(v) for v in body.joints]
        measured = self.measured()
        worst = max(abs(a - b) for a, b in zip(wanted, measured))
        if self.stall_above is not None and worst > float(self.stall_above):
            # Accept, report success, move nothing: the dangerous case.
            self.stalled += 1
            if self.goal is None:
                self.goal = list(measured)
            return {"accepted": True, "seq": body.seq, "stalled": True}
        # Sag: the mechanism settles N deg away from the command.
        self.goal = [w + s for w, s in zip(wanted, self.sag)]
        self.accepted += 1
        return {"accepted": True, "seq": body.seq}

    def state(self) -> dict:
        measured = self.measured()
        if self.stale_readback:
            if self._stale_pose is None:
                self._stale_pose = list(measured)
            measured = list(self._stale_pose)
        age = 0.0 if not self.stale_readback else round(
            time.time() - self.started, 1)
        return {"protocol": 1, "mode": self.mode, "desired": self.mode,
                "armed": self.armed, "fault": None, "error": None,
                "joints": {n: round(v, 3) for n, v in zip(NAMES, measured)},
                "joints_age_s": age, "joints_source": "sim",
                "last_seq": self.last_seq, "commands_applied": self.accepted,
                "commands_rejected": 0, "last_clamped": [],
                "lelab_active": False, "follower_open": True,
                "limits": {"shoulder_pan": [-120.0, 130.0],
                           "shoulder_lift": [-115.0, 115.0],
                           "elbow_flex": [-100.0, 100.0],
                           "wrist_flex": [-110.0, 105.0],
                           "wrist_roll": [-90.0, 65.0],
                           "gripper": [0.0, 100.0]},
                "uptime_s": round(time.time() - self.started, 1)}


def build(mechanism: SimMechanism) -> FastAPI:
    app = FastAPI()
    lock = threading.Lock()

    @app.get("/health")
    def health():
        return {"ok": True, "mode": mechanism.mode, "armed": mechanism.armed}

    @app.get("/state")
    def state():
        with lock:
            return mechanism.state()

    @app.post("/command")
    def command(body: Command):
        with lock:
            return mechanism.submit(body)

    @app.post("/mode")
    def mode(body: ModeBody):
        with lock:
            target = body.mode.upper()
            if target not in ("HOLD", "TELEOP", "REMOTE"):
                raise HTTPException(400, "mode must be HOLD, TELEOP or REMOTE")
            mechanism.mode = target
            if target != "REMOTE":
                mechanism.goal = None
            return {"mode": mechanism.mode}

    @app.post("/arm")
    def arm():
        mechanism.armed = True
        return {"armed": True, "mode": mechanism.mode}

    @app.get("/capture")
    def capture():
        frame = mechanism.env.render("wrist")
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise HTTPException(500, "encode failed")
        return {"jpeg_b64": base64.b64encode(encoded.tobytes()).decode("ascii"),
                "frame_ts": time.time(),
                "joints": {n: round(v, 3) for n, v in zip(NAMES, mechanism.measured())}}

    return app


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8199)
    ap.add_argument("--rate-deg-s", type=float, default=60.0)
    ap.add_argument("--deadband", type=float, default=0.0)
    ap.add_argument("--stall-above", type=float, default=None)
    ap.add_argument("--stale-readback", action="store_true",
                    help="freeze /state.joints at the start pose (the 2026-11-23 shape)")
    ap.add_argument("--sag", action="append", default=[],
                    help="joint=deg, repeatable (e.g. shoulder_lift=-2.0)")
    ap.add_argument("--home", default="", help="comma-separated joint degrees")
    ap.add_argument("--ready-file", default="")
    args = ap.parse_args()

    sag = {}
    for item in args.sag:
        name, _, value = item.partition("=")
        sag[name.strip()] = float(value)

    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    env.reset("8/8/8/8/8/8/8/8")
    if args.home:
        start = [float(v) for v in args.home.split(",")]
        env.data.qpos[env._joint_qpos] = np.radians(np.asarray(start))
        mujoco.mj_forward(env.model, env.data)

    mechanism = SimMechanism(env, rate_deg_s=args.rate_deg_s, sag_deg=sag,
                             deadband_deg=args.deadband,
                             stall_above_deg=args.stall_above,
                             stale_readback=args.stale_readback)
    mechanism.started = time.time()
    app = build(mechanism)
    print(f"sim bridge on :{args.port} sag={sag} deadband={args.deadband} "
          f"stall_above={args.stall_above} stale={args.stale_readback} "
          f"start={mechanism.measured()}", flush=True)
    if args.ready_file:
        with open(args.ready_file, "w") as handle:
            handle.write(str(os.getpid()))
    uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
