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
import numpy as np
from PIL import Image

from chess_sim import AppearanceConfig, ControlConfig, EnvConfig, MoveTask, CaptureTask, Camera
from chess_sim.dual import DualArmEnv

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
        self.state = {"phase": "initializing", "message": "Building the two-arm scene",
                      "renderer": "Starting", "engine": "Starting", "sim_time": 0,
                      "active_arm": None, "current_move": None, "last_result": None}
        self.storage = Path(os.environ.get("ROBOT_DATA_DIR", "/tmp/robot-arm-data"))
        self.storage.mkdir(parents=True, exist_ok=True)
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
            elif action == "speed":
                self.speed = float(options["speed"])
            else:
                raise ValueError("Unknown action")
            self.wake.set()

    def _render(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_render < 1 / float(os.environ.get("ROBOT_VIEW_FPS", "5")):
            return
        frames = {}
        # All contexts and renders stay on this thread. The debug top view is
        # distinct from the declared observations, even in wrist-only mode.
        for key, camera in CAMERAS.items():
            image = self.env.render(camera)
            buf = io.BytesIO()
            Image.fromarray(image).save(buf, "JPEG", quality=82)
            frames[key] = buf.getvalue()
        joints = {side: self.env.data.qpos[c["qpos"]].tolist()
                  for side, c in self.env._contexts.items()}
        with self.lock:
            self.frames = frames
            self.frame_id += 1
            self.joint_snapshot = joints
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

    def _reset(self):
        with self.lock:
            self.state.update(phase="resetting", message="Resetting the board", active_arm=None,
                              current_move=None, last_result=None)
        self.game = chess.Board(SCENARIOS[self.scenario])
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
        result = env.execute(task, on_step=callback or self._checkpoint)
        if report:
            with self.lock:
                self.state["last_result"] = {"success": result.success,
                    "error_mm": round(result.placement_error * 1000, 2),
                    "disturbed": [str(x) for x in result.disturbed],
                    "reason": str(result.reason) if result.reason else None}
        if not result.success:
            raise RuntimeError(f"Physical action failed: {result.reason or 'placement/clearance'}; "
                               f"error {result.placement_error * 1000:.1f} mm")

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
        if self.game.is_capture(move):
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
                    if self.running:
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
