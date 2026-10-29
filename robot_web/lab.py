"""Single-owner physics/render worker; HTTP threads never touch a MuJoCo context."""
from __future__ import annotations

import io
import json
import os
import shutil
import threading
import time
import traceback
import zlib
from collections import deque
from pathlib import Path

import chess
import chess.engine
import chess.pgn
import mujoco
import numpy as np
from PIL import Image

from chess_sim import AppearanceConfig, ControlConfig, EnvConfig, MoveTask, CaptureTask, Camera
from chess_sim.dual import DualArmEnv
from chess_sim import gripper
from chess_sim.assets import set_properties
from chess_sim.controller import PlanOverride
from .incidents import (Store, Recording, restore, bookkeeping, physics_state,
                        reset_diagnostics)
from .torture import CASES, SUITE_VERSION

OPENING = ["e2e4", "e7e5", "g1f3", "b8c6", "f1c4", "f8c5", "c2c3", "g8f6",
           "d2d3", "d7d6", "b1d2", "a7a6", "c4b3", "c5a7", "h2h3", "h7h6",
           "d2f1", "c8e6", "f1g3", "d8d7"]
CAMERAS = {"overview": Camera.EXTERNAL, "top": Camera.TOP,
           "white_wrist": Camera.WRIST, "black_wrist": Camera.WRIST_BLACK}
# Fixed viewpoints instead of a free orbit camera: a draggable camera gimbal-locks
# near the poles and drifts while the arm is moving. Azimuth 0 looks from +X,
# 90 from -Y (White's side), 180 from -X, 270 from +Y (Black's side).
VIEWS = {
    "overview": ("camera", Camera.EXTERNAL),
    # Distances solved so the scene bounding box covers ~68% of the frame at
    # 800x600 / fovy 45 (was 26-41%, i.e. the arm and board filled a quarter of
    # the screen). `lookat` is the scene centre, not the origin: the board sits
    # at z~0.56 m, so aiming at the origin cropped the subject when zoomed.
    "top": ("orbit", {"azimuth": 0.0, "elevation": -88.9, "distance": 0.669, "lookat": [0.0, 0.0, 0.5587]}),
    "right": ("orbit", {"azimuth": 0.0, "elevation": -20.0, "distance": 0.663, "lookat": [0.0, 0.0, 0.5587]}),
    "left": ("orbit", {"azimuth": 180.0, "elevation": -20.0, "distance": 0.663, "lookat": [0.0, 0.0, 0.5587]}),
    "white_side": ("orbit", {"azimuth": 90.0, "elevation": -42.0, "distance": 0.708, "lookat": [0.0, 0.0, 0.5587]}),
    "black_side": ("orbit", {"azimuth": 270.0, "elevation": -42.0, "distance": 0.708, "lookat": [0.0, 0.0, 0.5587]}),
    "iso_ne": ("orbit", {"azimuth": 45.0, "elevation": -35.0, "distance": 0.791, "lookat": [0.0, 0.0, 0.5587]}),
    "iso_nw": ("orbit", {"azimuth": 135.0, "elevation": -35.0, "distance": 0.791, "lookat": [0.0, 0.0, 0.5587]}),
    "iso_se": ("orbit", {"azimuth": 315.0, "elevation": -35.0, "distance": 0.791, "lookat": [0.0, 0.0, 0.5587]}),
    "iso_sw": ("orbit", {"azimuth": 225.0, "elevation": -35.0, "distance": 0.791, "lookat": [0.0, 0.0, 0.5587]}),
    "white_wrist": ("camera", Camera.WRIST),
    "black_wrist": ("camera", Camera.WRIST_BLACK),
}
SCENARIOS = {"standard": chess.STARTING_FEN,
             "capture": "4k3/8/8/3p4/4P3/8/8/4K3 w - - 0 1"}


class Interrupted(Exception):
    pass


class Lab:
    def __init__(self):
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.shutdown = threading.Event()
        self.running = False
        self.stop_requested = False
        self.reset_requested = True
        self.step_requested = False
        self.jog_request = None
        self.ready = False
        self.env = None
        self.engine = None
        self.game = chess.Board()
        self.history = []
        self.events = deque(maxlen=100)
        self.frames = {}
        self.frame_id = 0
        self.last_render = 0.0
        self.last_tick = 0.0
        self.speed = 2.0
        self.mode = "engine"
        self.observations = "wrists"
        self.scenario = "standard"
        self.dirty = False
        self.storage = Path(os.environ.get("ROBOT_DATA_DIR", "/tmp/robot-arm-data"))
        self.storage.mkdir(parents=True, exist_ok=True)
        self.recorder = Store(self.storage)
        self.model_signature = None
        self.runtime_signature = None
        self.recording = None
        self.record_enabled = True
        self.current_case = None
        self.suite = None
        self.replay = None
        self.replay_request = None
        self.rerun_request = None
        self.correct_request = None
        self.sweep_request = None
        self.incident_count = 0
        self.viewers = {}
        self.state = {"phase": "initializing", "message": "Building the two-arm scene",
                      "renderer": "Starting", "engine": "Starting", "sim_time": 0,
                      "active_arm": None, "current_move": None, "last_result": None}
        self.thread = threading.Thread(target=self._run, name="physics-render", daemon=True)

    def start_worker(self):
        self.thread.start()

    def log(self, message, level="info"):
        with self.lock:
            self.events.appendleft({"time": time.strftime("%H:%M:%S"), "level": level,
                                    "message": str(message)})
        print(f"[{level}] {message}", flush=True)

    def snapshot(self):
        with self.lock:
            return {**self.state, "ready": self.ready, "running": self.running,
                    "mode": self.mode, "speed": self.speed, "observations": self.observations,
                    "scenario": self.scenario, "fen": self.game.fen(), "ply": len(self.history),
                    "turn": "white" if self.game.turn else "black", "history": list(self.history),
                    "events": list(self.events), "frame_id": self.frame_id,
                    "control": "oracle-state scripted IK; contact-based grasping",
                    "perception_validated": False, "hardware_connected": False,
                    "gripper": gripper.description(),
                    "gripper_feedback": getattr(self, 'gripper_feedback', {}),
                    "recording": None if self.recording is None else self.recording.id,
                    "record_enabled": self.record_enabled,
                    "incidents": self.incident_count,
                    "case": self.current_case,
                    "sweep": self.state.get("sweep"),
                    "replay": None if self.replay is None else
                        {"id": self.replay["ident"], "frame": self.replay["frame"],
                         "frames": self.replay["frames"],
                         "playing": self.replay.get("playing", False)},
                    "suite": None if self.suite is None else
                        {"version": self.suite["version"], "index": self.suite["index"],
                         "total": len(CASES), "phase": self.suite["phase"],
                         "results": list(self.suite["results"])},
                    "observation_cameras": ["white_wrist", "black_wrist"] +
                        (["top"] if self.observations == "all" else []),
                    "joints": getattr(self, "joint_snapshot", {})}

    def command(self, action, **options):
        with self.lock:
            # A suite owns the single physics worker and its per-action result
            # fields until it finishes. Letting Start/jog/reset through here
            # could replace `self.suite` or queue a second controller request
            # while the active suite loop still owns the same MuJoCo world.
            # Stop remains available as the explicit cancellation path.
            if self.suite is not None and self.suite.get("phase") == "running":
                stopping_suite = action == "stop" or (
                    action == "suite" and options.get("suite", "start") == "stop")
                if not stopping_suite:
                    raise ValueError("The crowded suite has exclusive control until it completes")
            if action == "configure":
                if self.running or self.state["phase"] == "paused":
                    raise ValueError("Stop or reset before changing the game configuration")
                self.mode = options.get("mode", self.mode)
                self.scenario = options.get("scenario", self.scenario)
                self.observations = options.get("observations", self.observations)
                self.reset_requested = True
            elif action in ("start", "step"):
                if self.state["phase"] in ("fault", "complete", "stopped") or (
                        self.dirty and self.state["phase"] != "paused"):
                    self.reset_requested = True
                self.suite = None
                self.stop_requested = False
                self.step_requested = action == "step"
                self.running = True
                if self.state["phase"] == "paused":
                    self.state.update(phase="moving" if self.dirty else "ready", message="Simulation resumed")
            elif action == "pause":
                self.running = False
                self.state.update(phase="paused", message="Simulation paused")
            elif action == "stop":
                self.running = False
                self.stop_requested = True
                self.state.update(phase="stopped", message="Simulation stopped; Start begins a fresh game")
            elif action == "reset":
                self.running = False
                self.stop_requested = True
                self.reset_requested = True
                self.suite = None
                self.rerun_request = None
                if self.replay is not None:
                    self.replay_request = ("live", None, False)
            elif action == "jog":
                # Manual gamepad step. Staged here and consumed by the physics
                # worker thread: HTTP never touches MuJoCo, and one request moves
                # the arm one step, so a dropped controller stops the arm dead.
                self.jog_request = {"dx": options.get("dx", 0.0), "dy": options.get("dy", 0.0),
                                    "dz": options.get("dz", 0.0), "grip": options.get("grip"),
                                    "arm": options.get("arm"),
                                    "joint": options.get("joint"), "dq": options.get("dq", 0.0)}
                self.running = False
                self.stop_requested = False
                self.state.update(phase="manual", message="Manual jog")
            elif action == "speed":
                self.speed = float(options["speed"])
            elif action == "record":
                self.record_enabled = bool(options.get("enable", True))
                self.log("Incident recording " + ("enabled" if self.record_enabled else "disabled"))
            elif action == "suite":
                choice = options.get("suite", "start")
                if choice == "start":
                    self.suite = {"version": SUITE_VERSION, "index": 0, "phase": "running",
                                  "results": []}
                    self.running, self.stop_requested, self.step_requested = True, False, False
                    self.log(f"Torture suite {SUITE_VERSION}: {len(CASES)} independent cases")
                elif choice == "stop":
                    if self.suite:
                        self.suite["phase"] = "stopped"
                    self.running = False
                    self.log("Torture suite stops after the current case")
                else:
                    raise ValueError("Unknown suite action")
            elif action == "replay":
                ident = str(options["id"]) if options.get("id") else (
                    self.replay["ident"] if self.replay else None)
                if not ident:
                    raise ValueError("A recording id is required")
                frame = options.get("frame")
                self.replay_request = (ident, None if frame is None else int(frame),
                                       bool(options.get("play")))
                self.running = False
            elif action == "live":
                self.running = False
                self.replay_request = ("live", None, False)
            elif action == "rerun":
                self.stop_requested = False
                self.rerun_request = str(options["id"])
                self.running = False
            elif action == "correct":
                if not options.get("id"):
                    raise ValueError("A recording id is required")
                self.stop_requested = False
                self.correct_request = {"id": str(options["id"]), "params": options.get("params") or {}}
                self.running = False
            elif action == "sweep":
                if not options.get("id"):
                    raise ValueError("A recording id is required")
                self.stop_requested = False
                self.sweep_request = {"id": str(options["id"])}
                self.running = False
            else:
                raise ValueError("Unknown action")
            self.wake.set()


    def touch_view(self, name):
        """Remember which view a browser is watching, so replay renders it."""
        with self.lock:
            self.viewers[name] = time.monotonic()

    def _active_views(self, include_wrists=False):
        now = time.monotonic()
        with self.lock:
            active = {k for k, t in self.viewers.items() if now - t < 6 and k in VIEWS}
        if not include_wrists:
            active -= {"white_wrist", "black_wrist"}
        return active or {"overview"}

    def _render(self, force=False, subset=None):
        now = time.monotonic()
        if not force and now - self.last_render < 1 / float(os.environ.get("ROBOT_VIEW_FPS", "5")):
            return
        frames = {}
        # All contexts and renders stay on this thread. The debug top view is
        # distinct from the declared observations, even in wrist-only mode.
        if subset is None:
            # Routine passes render only what is on screen; the full set is for
            # resets and explicit snapshots. Twelve views at 5 fps was 60 renders
            # a second for images nobody was looking at.
            subset = (list(VIEWS) if force else
                      tuple(self._active_views(include_wrists=True)))
        keys = [(k, VIEWS[k]) for k in subset]
        lookat = self.env.orbit_default()["lookat"]
        lookat = [lookat[0], lookat[1], lookat[2] + 0.03]
        for key, (kind, value) in keys:
            image = (self.env.render(value) if kind == "camera"
                     else self.env.render(Camera.EXTERNAL, orbit={**value, "lookat": lookat}))
            buf = io.BytesIO()
            Image.fromarray(image).save(buf, "JPEG", quality=82)
            frames[key] = buf.getvalue()
        joints = {side: self.env.data.qpos[c["qpos"]].tolist()
                  for side, c in self.env._contexts.items()}
        feedback = {}
        if gripper.CUSTOM:
            for side, prefix in (('white', 'so101:'), ('black', 'black:')):
                positions = [float(self.env.data.qpos[self.env.model.joint(prefix + n).qposadr[0]])
                             for n in gripper.PROFILE['jaw_joints']]
                feedback[side] = {'jaw_positions_m': positions,
                    'aperture_mm': 1000 * (gripper.GAP_AT_ZERO - sum(positions))}
        with self.lock:
            if subset is None:
                self.frames = frames
            else:
                self.frames.update(frames)
            self.frame_id += 1
            self.joint_snapshot = joints
            self.gripper_feedback = feedback
            self.state["sim_time"] = float(self.env.data.time)
        self.last_render = time.monotonic()

    def _cross_arm_contact(self, env=None):
        env = self.env if env is None else env
        for c in env.data.contact:
            if c.dist >= -0.001:
                continue
            names = [env.model.body(int(env.model.geom_bodyid[g])).name
                     for g in (c.geom1, c.geom2)]
            if ((names[0].startswith("so101:") and names[1].startswith("black:")) or
                    (names[1].startswith("so101:") and names[0].startswith("black:"))):
                return names
        return None

    def _checkpoint(self, _action=None):
        while True:
            if self.shutdown.is_set() or self.stop_requested or self.reset_requested:
                raise Interrupted()
            if self.running:
                break
            self._render()
            self.wake.wait(0.04)
            self.wake.clear()
        contact = self._cross_arm_contact()
        if contact:
            raise RuntimeError("Arm-to-arm collision: " + " / ".join(contact))
        self._render()
        period = 1 / self.env.control_hz / self.speed
        now = time.monotonic()
        if self.last_tick:
            remaining = period - (now - self.last_tick)
            if remaining > 0:
                self.shutdown.wait(min(remaining, 0.1))
        self.last_tick = time.monotonic()

    def _pace(self, _action=None):
        """Checkpoint for actions that run while the sim is stopped.

        Corrections, sweeps and re-runs are requested directly, so they must not
        wait for the Start button the way a game turn does. Stop still aborts
        them, and arm-to-arm contact is still fatal.
        """
        if self.shutdown.is_set() or self.stop_requested:
            raise Interrupted()
        contact = self._cross_arm_contact()
        if contact:
            raise RuntimeError("Arm-to-arm collision: " + " / ".join(contact))
        self._render()
        period = 1 / self.env.control_hz / self.speed
        now = time.monotonic()
        if self.last_tick:
            remaining = period - (now - self.last_tick)
            if remaining > 0:
                self.shutdown.wait(min(remaining, 0.1))
        self.last_tick = time.monotonic()

    def _jog(self):
        """Apply one manual Cartesian step, on the physics thread.

        Reports requested vs achieved motion so the UI's BLOCKED lamp reflects
        what the arm actually did, not merely that a command was accepted. A
        step whose IK residual is too large is refused outright: snapping to the
        nearest reachable pose is what makes a teleoperated arm feel like it is
        fighting you.
        """
        req, self.jog_request = self.jog_request, None
        if req is None:
            return
        env = self.env
        side = req.get("arm") or (self.state.get("active_arm") or "white")
        try:
            env.select_arm(side)
        except Exception:
            pass
        prefix = "black:" if side == "black" else "so101:"
        if req.get("joint") is not None:
            self._jog_joint(env, side, prefix, req)
            return
        zeros = np.zeros(3)
        start, _ = env.ik.tool_pose(env.data, zeros)
        delta = np.array([req.get("dx") or 0.0, req.get("dy") or 0.0, req.get("dz") or 0.0],
                         dtype=float)
        target = start + delta
        requested = float(np.linalg.norm(delta))
        blocked, achieved, solution = None, 0.0, None
        if requested > 0.0:
            sol = env.ik.solve(env.data, target)
            if sol.position_error > 0.002:
                blocked = "unreachable"
            else:
                solution = np.asarray(sol.q, dtype=float)
        if blocked is None:
            joints = np.asarray(env.arm_joint_positions(), dtype=float)
            q = solution if solution is not None else joints[:5]
            grip = req.get("grip")
            gap = (gripper.angle_to_gap(float(joints[5])) if grip is None else float(grip))
            env.apply_action(np.concatenate([q, [gripper.gap_to_angle(gap)]]))
            now, _ = env.ik.tool_pose(env.data, zeros)
            achieved = float(np.linalg.norm(now - start))
            if requested > 0.0 and achieved < 0.35 * requested:
                blocked = "contact"
        # The white arm is built as `so101:` and the black one as `black:`; `side`
        # is the logical name, so map it rather than assuming side + ':'.
        prefix = "black:" if side == "black" else "so101:"
        near = []
        for jn in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
            joint = env.model.joint(prefix + jn)
            if not joint.limited:
                continue
            value = float(env.data.qpos[joint.qposadr[0]])
            lo, hi = float(joint.range[0]), float(joint.range[1])
            if min(value - lo, hi - value) < 0.10:
                near.append(jn)
        with self.lock:
            self.state.update(manual={"arm": side, "blocked": blocked,
                                      "requested_mm": round(requested * 1000, 2),
                                      "achieved_mm": round(achieved * 1000, 2),
                                      "near_limit": near})
        self._render(force=True)

    def _jog_joint(self, env, side, prefix, req):
        """Step one servo by a key-pair press (joint space, not Cartesian)."""
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
        idx = max(0, min(len(names) - 1, int(req["joint"])))
        dq = float(req.get("dq") or 0.0)
        joints = np.asarray(env.arm_joint_positions(), dtype=float).copy()
        joint = env.model.joint(prefix + names[idx])
        lo, hi = (float(joint.range[0]), float(joint.range[1])) if joint.limited else (-1e9, 1e9)
        before = float(joints[idx])
        after = min(max(before + dq, lo), hi)
        joints[idx] = after
        blocked = None
        moved = abs(after - before)
        if dq != 0.0 and moved < 1e-9:
            blocked = "joint limit"
        else:
            env.apply_action(joints)
            realised = abs(float(env.arm_joint_positions()[idx]) - before)
            if dq != 0.0 and realised < 0.25 * abs(dq):
                blocked = "contact"
        near = []
        for jn in names[:5]:
            other = env.model.joint(prefix + jn)
            if not other.limited:
                continue
            value = float(env.data.qpos[other.qposadr[0]])
            if min(value - float(other.range[0]), float(other.range[1]) - value) < 0.10:
                near.append(jn)
        with self.lock:
            self.state.update(manual={"arm": side, "joint": names[idx], "unit": "deg",
                                      "blocked": blocked,
                                      "requested_mm": round(float(np.degrees(dq)), 2),
                                      "achieved_mm": round(float(np.degrees(after - before)), 2),
                                      "near_limit": near})
        self._render(force=True)

    def _reset(self, fen=None):
        with self.lock:
            self.state.update(phase="resetting", message="Resetting the board", active_arm=None,
                              current_move=None, last_result=None)
        self.game = chess.Board(fen or SCENARIOS[self.scenario])
        self.history = []
        self.env.reset(self.game.fen())
        self.env.cameras = ((Camera.WRIST, Camera.WRIST_BLACK, Camera.TOP)
                            if self.observations == "all" else (Camera.WRIST, Camera.WRIST_BLACK))
        self.dirty = False
        self.stop_requested = False
        self.reset_requested = False
        self.last_tick = 0
        self._render(force=True)
        with self.lock:
            self.state.update(phase="ready", message="Ready — two arms, one board")
        self.log("Board reset: " + self.scenario)

    def _choose_move(self):
        if self.mode == "opening":
            if self.scenario == "capture":
                return chess.Move.from_uci("e4d5") if not self.history else None
            if len(self.history) >= len(OPENING):
                return None
            move = chess.Move.from_uci(OPENING[len(self.history)])
            if move not in self.game.legal_moves:
                raise RuntimeError("Opening replay no longer matches game state")
            return move
        if self.engine is None:
            raise RuntimeError("Stockfish unavailable; select Opening replay or check the container logs")
        candidates = self.env.candidate_moves(self.game)
        if not candidates:
            raise RuntimeError("No reachable legal move; layout or promotion handling needs attention")
        return self.engine.play(self.game, chess.engine.Limit(time=0.15),
                                root_moves=candidates).move

    def _physical(self, task, env=None, callback=None, report=True,
                  override=None, extra=None):
        env = self.env if env is None else env
        reset_diagnostics(env)          # this action owns its diagnostics, not the last one
        recording = self._record_start(env, task, extra)
        try:
            result = env.execute(task, on_step=self._record_step(
                recording, callback or self._checkpoint), override=override)
        except BaseException as exc:
            self._record_finish(recording, False, None,
                                "Interrupted" if isinstance(exc, Interrupted)
                                else (str(exc) or type(exc).__name__))
            raise
        outcome = {"success": result.success,
                   "error_mm": round(result.placement_error * 1000, 2),
                   "disturbed": [str(x) for x in result.disturbed],
                   "upright": bool(result.upright),
                   "waypoints_precise": bool(result.waypoints_precise),
                   "picked": None if result.picked is None else str(result.picked),
                   "reason": str(result.reason) if result.reason else None}
        self._record_finish(recording, result.success, outcome, "")
        if report:
            with self.lock:
                self.state["last_result"] = {"success": result.success,
                    "error_mm": outcome["error_mm"],
                    "disturbed": outcome["disturbed"],
                    "reason": outcome["reason"]}
        if not result.success:
            raise RuntimeError(f"Physical action failed: {result.reason or 'placement/clearance'}; "
                               f"error {result.placement_error * 1000:.1f} mm")

    # -- incident recording --------------------------------------------------

    def _frames(self):
        with self.lock:
            return dict(self.frames)

    def _record_start(self, env, task, extra=None):
        if not self.record_enabled or not self.model_signature:
            return None
        payload = ({"kind": "move", "from": str(task.from_square), "to": str(task.to_square)}
                   if isinstance(task, MoveTask)
                   else {"kind": "capture", "square": str(task.square)})
        slot = env._slot_for(task)
        try:
            rec = Recording(self.recorder, env, self.model_signature, self.runtime_signature,
                            {"action": payload["kind"], "task": payload,
                             "arm": env.active_side, "case_id": self.current_case,
                             "move": self.state.get("current_move"), "speed": self.speed,
                             "gripper": gripper.description(), **(extra or {})},
                            self._frames())
        except Exception as exc:      # recording must never block a physical action
            self.log("Recording could not start: " + str(exc), "error")
            return None
        rec.target_body = None if slot is None else slot.body
        with self.lock:
            self.recording = rec
        return rec

    def _record_step(self, recording, checkpoint):
        def step(action=None):
            checkpoint(action)
            if recording is None:
                return
            try:
                recording.contact_sample()
                recording.sample(action)
            except Exception as exc:
                self.log("Recording stopped: " + str(exc), "error")
                with self.lock:
                    self.recording = None
        return step

    def _record_finish(self, recording, success, outcome, message):
        if recording is None:
            return
        with self.lock:
            self.recording = None
        try:
            meta = recording.finish(success, outcome, message, self._frames())
        except Exception as exc:
            self.log("Recording could not be saved: " + str(exc), "error")
            return
        with self.lock:
            self.incident_count += 1
            self.last_recording_id = meta["id"]
            self.state["last_recording"] = meta["id"]
            self.state["last_categories"] = meta["categories"]
        self.log(("Kept" if not success else "Recorded") + f" incident {meta['id'][:8]}: "
                 + ", ".join(meta["categories"]))

    def _turn(self):
        side = "white" if self.game.turn else "black"
        self.env.select_arm(side)
        with self.lock:
            self.state.update(phase="thinking", active_arm=side, message=side.title() + " is choosing a move")
        if self.game.is_game_over(claim_draw=True):
            return self._complete("Game over: " + self.game.result(claim_draw=True))
        move = self._choose_move()
        if move is None:
            return self._complete("Opening replay complete")
        if move.promotion:
            raise RuntimeError("Promotion needs a physical piece exchange; not automated in this version")
        san = self.game.san(move)
        with self.lock:
            self.state.update(phase="moving", current_move=move.uci(),
                              message=f"{side.title()} · {san} ({move.uci()})")
        self.log(f"{side.title()} arm: {san} ({move.uci()})")
        self.dirty = True
        self._execute_on(self.env, move, self._checkpoint, report=True)
        expected = self.game.copy()
        expected.push(move)
        self.game = expected
        self.env.position = self.game.copy()
        self.history.append({"ply": len(self.history) + 1, "side": side,
                             "san": san, "uci": move.uci()})
        self.dirty = False
        self.log(f"Verified {san}; {self.state['last_result']['error_mm']:.1f} mm placement error")
        self._save_game()
        self._render(force=True)
        if self.step_requested:
            self.running = False
            self.step_requested = False
            with self.lock:
                self.state.update(phase="paused", message="Single turn complete")

    def _execute_on(self, env, move, callback, report=False):
        def physical(task):
            self._physical(task, env=env, callback=callback, report=report)
        # Turn-independent capture test. python-chess reports is_capture() True
        # for an illegal move by the side not to move, which silently converted
        # a black arm's move into a capture of its own destination square.
        moving = self.game.piece_at(move.from_square)
        occupant = self.game.piece_at(move.to_square)
        is_capture = bool(
            self.game.is_en_passant(move)
            or (moving is not None and occupant is not None
                and occupant.color != moving.color))
        if is_capture:
            captured = move.to_square
            if self.game.is_en_passant(move):
                captured += -8 if self.game.turn else 8
            physical(CaptureTask(square=chess.square_name(captured)))
        physical(MoveTask(from_square=chess.square_name(move.from_square),
                                to_square=chess.square_name(move.to_square)))
        if self.game.is_castling(move):
            rank = chess.square_rank(move.from_square)
            king_side = self.game.is_kingside_castling(move)
            src, dst = (7, 5) if king_side else (0, 3)
            physical(MoveTask(from_square=chess.square_name(chess.square(src, rank)),
                                    to_square=chess.square_name(chess.square(dst, rank))))
        expected = self.game.copy()
        expected.push(move)
        error = env.full_board_error(expected)
        if error:
            raise RuntimeError(error)
        env.park(on_step=callback)
        error = env.full_board_error(expected)
        if error:
            raise RuntimeError("After parking: " + error)

    def _complete(self, message):
        self.running = False
        with self.lock:
            self.state.update(phase="complete", message=message, current_move=None)
        self.log(message)

    # -- torture suite: independent, predetermined physical cases -------------

    def _suite_run(self):
        """Each case gets a fresh board, so a failed case cannot poison the next.
        These are physical manipulation tasks, not legal chess games."""
        suite = self.suite
        suite["phase"] = "running"
        while suite["index"] < len(CASES):
            if self.shutdown.is_set() or self.stop_requested or not self.running or \
                    suite["phase"] != "running":
                suite["phase"] = "stopped"
                self.running = False
                return
            case = CASES[suite["index"]]
            self.current_case = case.id
            self.log(f"Case {suite['index'] + 1}/{len(CASES)} · {case.id} · {case.move}")
            with self.lock:
                self.state.update(phase="moving", active_arm=case.arm, current_move=case.move,
                                  message=f"{case.id}: {case.description}", last_result=None,
                                  last_recording=None, last_categories=None)
            self.game = chess.Board(case.fen)
            self.history = []
            move = chess.Move.from_uci(case.move)
            # Fixed draw per case: the suite is a regression check, so the same
            # case must see the same servo noise on every build.
            seed = zlib.crc32(case.id.encode()) & 0xffffffff
            self.env._noise_rng = np.random.default_rng(seed)
            self.env.reset(self.game.fen())
            self.env.select_arm(case.arm)
            self.env.position = self.game.copy()
            # Every case must start from a fully placed board. A missing source
            # square is silent in the old path (it surfaced once as a bare
            # "no piece on source square"), so reload and say so instead.
            if self.env._square_slot.get(move.from_square) is None:
                self.log(f"{case.id}: source square missing after reset "
                         f"({len(self.env._square_slot)} slots); reloading the board", "error")
                self.env.reset(chess.Board(case.fen))
                self.env.select_arm(case.arm)
                self.env.position = self.game.copy()
            slots = len(self.env._square_slot)
            if self.env._square_slot.get(move.from_square) is None:
                self.log(f"{case.id}: source square still missing ({slots} slots); skipping", "error")
                suite["results"].append({"case": case.id, "description": case.description,
                    "move": case.move, "arm": case.arm, "success": False, "error_mm": None,
                    "slots": slots, "categories": ["setup_error"],
                    "message": "source square missing after reset", "recording": None})
                suite["index"] += 1
                continue
            self.dirty = False
            self.last_recording_id = None
            self.state["last_result"] = None      # this case owns its verdict, not the last one
            self._render(force=True)
            record = {"case": case.id, "description": case.description, "move": case.move,
                      "arm": case.arm, "success": False, "error_mm": None, "slots": slots,
                      "categories": [], "message": "", "recording": None}
            try:
                self._execute_on(self.env, move, self._checkpoint, report=True)
                record["success"] = True
            except Interrupted:
                suite["results"].append(record)
                suite["phase"] = "stopped"
                self.running = False
                return
            except Exception as exc:
                record["message"] = str(exc)
                last = self.state.get("last_result") or {}
                record["error_mm"] = last.get("error_mm")
            # One source of truth for the verdict. Success used to mean "no
            # exception escaped", while categories came from the recorder: a
            # checkpoint raising after a good placement scored the case FAIL
            # with categories ['success'], and the same physical result passed
            # in one runner and failed in another.
            outcome = self.state.get("last_result") or {}
            if outcome:
                record["success"] = bool(outcome.get("success"))
                record["error_mm"] = outcome.get("error_mm")
            elif record["success"]:
                record["success"] = False
                record.setdefault("categories", []).append("no_outcome_recorded")
            record["recording"] = getattr(self, "last_recording_id", None)
            record["categories"] = list(self.state.get("last_categories") or [])
            suite["results"].append(record)
            suite["index"] += 1
        suite["phase"] = "complete"
        self.current_case = None
        self.running = False
        passed = sum(1 for r in suite["results"] if r["success"])
        with self.lock:
            self.state.update(phase="complete", current_move=None, active_arm=None,
                              message=f"Torture suite {suite['version']}: "
                                      f"{passed}/{len(suite['results'])} cases passed")
        self.log(f"Torture suite {suite['version']} complete: {passed}/{len(suite['results'])}")

    # -- replay / re-run -----------------------------------------------------

    def _replay(self):
        request, self.replay_request = self.replay_request, None
        if request is None:
            return
        ident, frame, play = (list(request) + [None, False])[:3]
        if ident == "live":
            return self._live()
        if self.replay is None or self.replay["ident"] != ident:
            if self.replay is not None:
                self._live()
            meta, states, trace = self.recorder.load(ident, self.model_signature)
            self.replay = {"ident": ident, "frame": 0, "frames": len(states),
                           "playing": False, "states": states, "trace": trace, "meta": meta,
                           "legacy": meta.get("schema", 1) < 2,
                           "live_state": physics_state(self.env),
                           "live_meta": bookkeeping(self.env),
                           "live_phase": self.state["phase"],
                           "live_message": self.state["message"]}
        # An explicit frame is always honoured: scrubbing pauses there, and play
        # starts from the frame the viewer is looking at. Pressing play at the end
        # restarts rather than doing nothing visible.
        idx = self.replay["frame"] if frame is None else int(frame)
        if play and self.replay["frames"] < 2:
            self.replay["playing"] = False
            self._replay_show(0)      # show the frame first: it sets its own message
            with self.lock:
                self.state.update(phase="replay", active_arm=self.replay["meta"].get("arm"),
                                  message=f"Replay {ident[:8]} · single frame, nothing to play")
            return
        if play and idx >= self.replay["frames"] - 1:
            idx = 0
        self.replay["playing"] = bool(play)
        self._replay_show(idx)

    def _replay_show(self, idx):
        replay = self.replay
        idx = max(0, min(int(idx), replay["frames"] - 1))
        states, trace, meta = replay["states"], replay["trace"], replay["meta"]
        mujoco.mj_setState(self.env.model, self.env.data, states[idx],
                           mujoco.mjtState.mjSTATE_INTEGRATION)
        warm = self.env.data.qacc_warmstart.copy()
        mujoco.mj_forward(self.env.model, self.env.data)
        self.env.data.qacc_warmstart[:] = warm
        replay["frame"] = idx
        step = trace[idx] if idx < len(trace) else {}
        with self.lock:
            self.state.update(phase="replay", active_arm=meta.get("arm"),
                              current_move=meta.get("move"),
                              message=f"Replay {replay['ident'][:8]} · frame {idx + 1}/"
                                      f"{replay['frames']} · {step.get('phase', '?')}"
                                      + (" · playing" if replay["playing"] else "")
                                      + (" · legacy recording" if replay.get("legacy") else ""))
        # Playback renders whatever the viewer has selected, not a hard-coded
        # overview: an isometric replay used to sit frozen while the clock ran.
        # Both wrist views always follow the recording too — they are the view
        # that explains a bad grasp, and the page shows them throughout.
        subset = tuple(set(self._active_views()) | {"white_wrist", "black_wrist"})
        self._render(force=True, subset=subset)

    def _replay_tick(self):
        replay = self.replay
        if replay is None or not replay.get("playing"):
            return
        if replay["frames"] < 2:
            replay["playing"] = False
            with self.lock:
                self.state.update(message=f"Replay {replay['ident'][:8]} · single frame, "
                                          f"nothing to play")
            return
        if replay["frame"] >= replay["frames"] - 1:
            replay["playing"] = False
            with self.lock:
                self.state.update(message=f"Replay {replay['ident'][:8]} · end of recording "
                                          f"(press play to restart)")
            return
        self._replay_show(replay["frame"] + 1)
        # Never busy-spin; pause/stop land within one frame interval.
        self.shutdown.wait(1 / float(os.environ.get("ROBOT_REPLAY_FPS", "12")))

    def _live(self):
        if self.replay is None:
            return
        replay, self.replay = self.replay, None
        restore(self.env, replay["live_state"], replay["live_meta"])
        self.game = chess.Board(replay["live_meta"]["position"])
        self.dirty = False
        phase = replay["live_phase"]
        if phase == "moving" and not self.running:
            phase = "ready"       # a stale label from the moment replay began
        with self.lock:
            self.state.update(phase=phase, message=replay["live_message"])
        self._render(force=True)

    def _rerun(self):
        ident, self.rerun_request = self.rerun_request, None
        if ident is None:
            return
        self._live()
        try:
            meta, states, _ = self.recorder.load(ident, self.model_signature)
        except ValueError as exc:
            with self.lock:
                self.state.update(phase="ready", message="Re-run unavailable: " + str(exc))
            self.log("Re-run unavailable: " + str(exc))
            return
        if meta.get("runtime_signature") != self.runtime_signature:
            message = ("Re-run unavailable: this recording was made by different controller code; "
                       "replay still works, re-run needs the build that produced it")
            with self.lock:
                self.state.update(phase="ready", message=message)
            self.log(message)
            return
        restore(self.env, states[0], meta["initial"])
        self.game = chess.Board(meta["initial"]["position"])
        self.history = []
        self.dirty = False
        self.current_case = meta.get("case_id")
        payload = meta.get("task") or {}
        task = (MoveTask(from_square=payload["from"], to_square=payload["to"])
                if payload.get("kind") == "move" else CaptureTask(square=payload["square"]))
        with self.lock:
            self.state.update(phase="moving", current_move=meta.get("move"),
                              message=f"Re-run of {ident[:8]} from its saved starting state",
                              last_result=None, last_recording=None)
        self._render(force=True)
        try:
            self._physical(task, callback=self._pace)
            with self.lock:
                self.state.update(phase="ready", message="Re-run finished")
        except Exception as exc:
            with self.lock:
                self.state.update(phase="fault", message="Re-run: " + str(exc))
            self.log("Re-run failed: " + str(exc), "error")
        finally:
            self.current_case = None

    # -- human correction ----------------------------------------------------

    SWEEP_GRID = (
        {},
        {"grasp_dz": -0.004}, {"grasp_dz": 0.004},
        {"span_deg": -20.0}, {"span_deg": 20.0},
        {"place_dy": 0.002}, {"place_dy": -0.002},
    )

    def _message(self, phase, text):
        with self.lock:
            self.state.update(phase=phase, message=text)

    def _task_from(self, meta):
        payload = meta.get("task") or {}
        return (MoveTask(from_square=payload["from"], to_square=payload["to"])
                if payload.get("kind") == "move" else CaptureTask(square=payload["square"]))

    def _restore_from(self, meta, states):
        restore(self.env, states[0], meta["initial"])
        self.game = chess.Board(meta["initial"]["position"])
        self.history = []
        self.dirty = False
        self.current_case = meta.get("case_id")

    def _correct(self):
        """Re-run a recording from its saved start with human-chosen parameters."""
        request, self.correct_request = self.correct_request, None
        if not request:
            return
        ident, params = request["id"], request.get("params") or {}
        try:
            meta, states, _ = self.recorder.load(ident, self.model_signature)
        except ValueError as exc:
            self._message("ready", "Correction unavailable: " + str(exc))
            return
        self._live()
        self._restore_from(meta, states)
        task = self._task_from(meta)
        self._message("moving", "Correcting " + ident[:8] + " with " + (str(params) or "no change"))
        self._render(force=True)
        try:
            self._physical(task, callback=self._pace, override=PlanOverride(**params),
                           extra={"correction_of": ident, "override": params})
            self._message("ready", "Correction finished")
        except Exception as exc:
            self._message("fault", "Correction: " + str(exc))
        finally:
            self.current_case = None

    def _sweep(self):
        """Search a small parameter grid; record only the best variant.

        This is the cheap answer to 'do we need human demonstrations?': if a
        grasp that failed can be fixed by moving the jaw height 4 mm, the
        dataset can be repaired without a human in the loop.
        """
        request, self.sweep_request = self.sweep_request, None
        if not request:
            return
        ident = request["id"]
        try:
            meta, states, _ = self.recorder.load(ident, self.model_signature)
        except ValueError as exc:
            self._message("ready", "Sweep unavailable: " + str(exc))
            return
        self._live()
        task = self._task_from(meta)
        results = []
        keep_recording, self.record_enabled = self.record_enabled, False
        try:
            for params in self.SWEEP_GRID:
                if self.shutdown.is_set() or self.stop_requested:
                    break
                self._restore_from(meta, states)
                self._message("moving", "Sweep " + ident[:8] + " " + (str(params) or "baseline"))
                try:
                    result = self.env.execute(task, on_step=self._pace,
                                              override=PlanOverride(**params))
                    results.append({"params": params, "success": bool(result.success),
                                    "error_mm": round(result.placement_error * 1000, 1)})
                except Exception as exc:
                    results.append({"params": params, "success": False,
                                    "error_mm": None, "message": str(exc)[:120]})
        finally:
            self.record_enabled = keep_recording
        best = next((r["params"] for r in results if r["success"]), None)
        with self.lock:
            self.state["sweep"] = {"id": ident, "results": list(results), "best": best}
        if best is not None:
            self._restore_from(meta, states)
            try:
                self._physical(task, callback=self._pace, override=PlanOverride(**best),
                               extra={"correction_of": ident, "override": best, "swept": True})
                self._message("ready", "Sweep fixed %s with %s" % (ident[:8], best or "no change"))
            except Exception as exc:
                self._message("fault", "Sweep recording failed: " + str(exc))
        else:
            self._message("ready", "Sweep found no fix for " + ident[:8])
        self.current_case = None

    def _save_game(self):
        g = chess.pgn.Game.from_board(self.game)
        g.headers.update(Event="SO101 simulation", White="White SO101", Black="Black SO101")
        tmp = self.storage / "last-game.pgn.tmp"
        tmp.write_text(str(g) + "\n")
        tmp.replace(self.storage / "last-game.pgn")
        (self.storage / "last-run.json").write_text(json.dumps({"history": self.history,
            "fen": self.game.fen(), "control": "oracle-state contact-based scripted IK"}, indent=2))

    def _run(self):
        try:
            self.env = DualArmEnv(EnvConfig(
                appearance=AppearanceConfig(camera_mast=False,
                    piece_set=os.environ.get("ROBOT_PIECE_SET", "default")),
                control=ControlConfig(cameras=(), image_size=(800, 600))))
            self.env.model.vis.quality.shadowsize = 2048
            self.model_signature = __import__("robot_web.incidents", fromlist=["model_signature"]).model_signature(self.env)
            self.runtime_signature = __import__("robot_web.incidents", fromlist=["runtime_signature"]).runtime_signature(self.model_signature)
            self.incident_count = len(self.recorder.rows())
            self._reset()
            from OpenGL import GL
            renderer = GL.glGetString(GL.GL_RENDERER)
            renderer = renderer.decode() if renderer else "Unknown OpenGL renderer"
            path = os.environ.get("STOCKFISH_PATH") or shutil.which("stockfish")
            if not path and Path("/usr/games/stockfish").is_file():
                path = "/usr/games/stockfish"
            if path:
                self.engine = chess.engine.SimpleEngine.popen_uci(path)
                self.engine.configure({"Threads": 1, "Hash": 32})
            with self.lock:
                self.ready = True
                self.state.update(piece_set={"id": self.env.config.appearance.piece_set,
                    "name": set_properties(self.env.config.appearance.piece_set).get("name", "Original generated set")},
                    renderer=renderer, engine=self.engine.id.get("name", "Stockfish")
                                  if self.engine else "Unavailable — opening replay only")
            self.log("Renderer: " + renderer)
            while not self.shutdown.is_set():
                try:
                    if self.reset_requested:
                        self._reset()
                    if self.replay_request is not None:
                        self._replay()
                    if self.rerun_request is not None:
                        self._rerun()
                    if self.correct_request is not None:
                        self._correct()
                    if self.sweep_request is not None:
                        self._sweep()
                    if self.replay is not None and self.replay.get("playing") \
                            and not self.running:
                        self._replay_tick()
                        continue
                    if self.jog_request is not None:
                        self._jog()
                        continue
                    if self.suite is not None and self.suite["phase"] == "running":
                        self._suite_run()
                    elif self.running:
                        self._turn()
                    else:
                        self.wake.wait(0.15)
                        self.wake.clear()
                except Interrupted:
                    self.running = False
                    if not self.reset_requested:
                        with self.lock:
                            self.state.update(phase="stopped", message="Stopped at a control checkpoint; Start resets the game")
                except Exception as exc:
                    self.running = False
                    self.dirty = True
                    if self.replay is not None:
                        self.replay["playing"] = False    # never tick past a fault
                    with self.lock:
                        self.state.update(phase="fault", message=str(exc))
                    self.log(str(exc), "error")
                    traceback.print_exc()
                    self._render(force=True)
        except Exception as exc:
            with self.lock:
                self.ready = False
                self.state.update(phase="error", message=str(exc))
            self.log("Initialization failed: " + str(exc), "error")
            traceback.print_exc()
        finally:
            if self.engine:
                self.engine.quit()
            if self.env:
                self.env.close()
