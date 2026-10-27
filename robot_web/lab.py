"""Single-owner physics/render worker; HTTP threads never touch a MuJoCo context."""
from __future__ import annotations

import io
import json
import os
import shutil
import threading
import time
import traceback
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
from .incidents import Store, Recording, restore, bookkeeping, physics_state
from .torture import CASES, SUITE_VERSION

OPENING = ["e2e4", "e7e5", "g1f3", "b8c6", "f1c4", "f8c5", "c2c3", "g8f6",
           "d2d3", "d7d6", "b1d2", "a7a6", "c4b3", "c5a7", "h2h3", "h7h6",
           "d2f1", "c8e6", "f1g3", "d8d7"]
CAMERAS = {"overview": Camera.EXTERNAL, "top": Camera.TOP,
           "white_wrist": Camera.WRIST, "black_wrist": Camera.WRIST_BLACK}
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
        self.orbit = None
        self.incident_count = 0
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
                    "orbit": self.orbit,
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
            elif action == "speed":
                self.speed = float(options["speed"])
            elif action == "orbit":
                base = self.env.orbit_default() if self.env else {"lookat": [0, 0, 0.02],
                                                                  "distance": 0.9, "azimuth": 130.0,
                                                                  "elevation": -35.0}
                orbit = dict(self.orbit or base)
                for key, limit in (("distance", (0.22, 1.8)), ("elevation", (-88.0, 88.0))):
                    if options.get(key) is not None:
                        orbit[key] = float(np.clip(float(options[key]), *limit))
                if options.get("azimuth") is not None:
                    orbit["azimuth"] = float(options["azimuth"]) % 360.0
                if options.get("reset"):
                    orbit = base
                self.orbit = orbit
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
                self.rerun_request = str(options["id"])
                self.running = False
            else:
                raise ValueError("Unknown action")
            self.wake.set()

    def _render(self, force=False, subset=None):
        now = time.monotonic()
        if not force and now - self.last_render < 1 / float(os.environ.get("ROBOT_VIEW_FPS", "5")):
            return
        frames = {}
        # All contexts and renders stay on this thread. The debug top view is
        # distinct from the declared observations, even in wrist-only mode.
        keys = list(CAMERAS.items()) if subset is None else [(k, CAMERAS[k]) for k in subset]
        for key, camera in keys:
            orbit = self.orbit if key == "overview" else None
            image = self.env.render(camera, orbit=orbit)
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

    def _physical(self, task, env=None, callback=None, report=True):
        env = self.env if env is None else env
        recording = self._record_start(env, task)
        try:
            result = env.execute(task, on_step=self._record_step(
                recording, callback or self._checkpoint))
        except BaseException as exc:
            self._record_finish(recording, False, None,
                                "Interrupted" if isinstance(exc, Interrupted)
                                else (str(exc) or type(exc).__name__))
            raise
        outcome = {"success": result.success,
                   "error_mm": round(result.placement_error * 1000, 2),
                   "disturbed": [str(x) for x in result.disturbed],
                   "upright": bool(result.upright),
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

    def _record_start(self, env, task):
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
                             "gripper": gripper.description()},
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
            with self.lock:
                self.state.update(phase="replay", active_arm=self.replay["meta"].get("arm"),
                                  message=f"Replay {ident[:8]} · single frame, nothing to play")
            self._replay_show(0)
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
        # Playback only needs the main view; the wrist streams keep their last
        # real frame rather than paying for three extra renders per step.
        self._render(force=True, subset=("overview",))

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
            self._physical(task)
            with self.lock:
                self.state.update(phase="ready", message="Re-run finished")
        except Exception as exc:
            with self.lock:
                self.state.update(phase="fault", message="Re-run: " + str(exc))
            self.log("Re-run failed: " + str(exc), "error")
        finally:
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
                appearance=AppearanceConfig(camera_mast=False),
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
                self.state.update(renderer=renderer, engine=self.engine.id.get("name", "Stockfish")
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
                    if self.replay is not None and self.replay.get("playing") \
                            and not self.running:
                        self._replay_tick()
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
