#!/usr/bin/env python3
"""robot-live — browser console for the Phase 11d scan.

Serves one page: the wrist camera with detected markers drawn on it, the arm
mode and joints, the last board offset, and the buttons that arm the bridge,
switch teleop/remote, run a scan, or stop.

It is a thin front end. All motion authority stays in the Pi bridge and all
scan logic stays in `scan_controller.py`; this process only orchestrates and
draws.  Reading state is unauthenticated on the LAN; every POST needs the
bridge token.

    python live/app.py   (env: LIVE_CONFIG, LIVE_PORT)
"""
from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import replace
from functools import partial
from pathlib import Path
import threading
import time
from collections import deque

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import (HTMLResponse, JSONResponse, RedirectResponse,
                               Response, StreamingResponse)
from starlette.concurrency import run_in_threadpool

import camera_yaw
from aim import (CameraMount, PlaneTracker, aim_translation,
                 evaluate as aim_evaluate, plane_from_solution)
from bridge_client import BridgeError, HttpBridge, HttpFrames
from kinematics import JOINTS, SQUARE_MM, ArmKinematics, square_xy
from solver import MoveSolver
from scan_controller import ScanController, plan_in_degrees
from target_overlay import TargetOverlay, draw as draw_target
from tip_frame import TIP_IN_CHAIN, describe as describe_tip_frame
from wrist_scan_runner import (ScanInputs, frames_from_captured, load_inputs,
                               plan_pose_degrees, run_scan as run_wrist_scan,
                               summarize as summarize_wrist_scan)
from chess_sim import fiducials
from chess_sim.board_target import Board, SQUARE_MM as BOARD_SQUARE_MM
from chess_sim.projection import per_component_rms, pnp_tool_frame, project_tool_frame
from chess_sim.marker_scan import (MarkerLayout, MarkerScanPlan, PoseCoverage,
                                   board_offset, load_plan, marker_corners_world)
from chess_sim.product import current_layout
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.wrist_scan import load_plan as load_wrist_poses


#: Take-up final-leg behaviour.  A take-up exists to remove the last 0.2-0.7 deg
#: of direction-dependent settle, so the final leg may not use the ordinary
#: 3.5 deg settle allowance (that would swallow the very error it is there to
#: remove) and it may not stream for the full 30 s (it is only a few degrees
#: long by construction).
#:
#: The allowance is 1.6 deg because the return leg lands inside the servo's static
#: band, which measured 0.6-1.3 deg on the loaded elbow at a folded pose.  The
#: settle break and the stall FAULT are therefore separate numbers:
#:
#:   * a leg that has stopped with <= 1.6 deg to go is a settle (break, report it);
#:   * a leg with a > 3.0 deg lead and no motion for 20 ticks is not actuating
#:     (raise, exactly as before).  The leg is only 3.5 deg long, so a 3.0 lead
#:     cannot be a resting offset.
#:
#: Both numbers matter: with the allowance at 0.8 the stall guard fired on a
#: healthy 0.80 settle, and at 1.2 it fired at 1.22 - each time the move path's
#: recovery swung the arm home for a servo that was simply resting inside its
#: friction band (measured 2026-11-25, recovery_log 09:08:57).
TAKE_UP_SETTLE_DEG = 0.3
TAKE_UP_SETTLE_ALLOW_DEG = 1.6
TAKE_UP_STALL_LEAD_DEG = 3.0
TAKE_UP_FINAL_DEADLINE_S = 10.0


class StuckError(BridgeError):
    """A move that cannot continue without the arm being reset.

    Only the conditions the recovery primitive exists for are raised as this
    subclass: keep-in refusal, an unreachable/solver failure, the stream loop's
    stall or non-execution raise, and a move that lands outside keep-in.  A bad
    request (unknown mode, missing field) stays a plain ``BridgeError`` — it is
    not something driving the arm home would fix.
    """


CONFIG_PATH = Path(os.environ.get("LIVE_CONFIG", "live/live_config.json"))
DEFAULTS = {
    "bridge_url": "http://192.168.1.167:8100",
    "token_file": "live/bridge_token.txt",
    "calibration": ".private/phase11d/calibration.json",
    "plan": "chess_sim/assets/marker_scan_plan.json",
    "board_half_mm": 122.0,
    "square_m": 0.028,
    "board_z_m": 0.4337,
    "marker_z_m": 0.43056,
    "layout_offset_mm": 152.0,
    # "mat" = the four border tags, "charuco" = the printed calibration face.
    "layout_kind": "mat",
    # Target overlay on the camera view: the P1 perception path (printed-face
    # registration + piece centroid) drawn where the arm would go.  Detection
    # runs off the stream thread, once per ``target_refresh_s``.
    "target_overlay": True,
    "target_reference": "chess_sim/assets/scene/generated/board_charuco.png",
    "target_px_per_mm": 4.0,
    "target_refresh_s": 0.5,
    "settle_s": 0.6,
    "pose_timeout_s": 12.0,
    "speed_scale": 0.3,
    "port": 8080,
    # Board frame: origin at the mat centre, +x along the files, +y along the
    # ranks, z at the playing surface.  Measured once with a tape; refined later
    # by the tag solve.  Without these, x/y/z commands mean nothing physical.
    "board_x_m": 0.0,
    "board_y_m": 0.0,
    "board_yaw_deg": 0.0,
    "board_file": "live/board.json",
    # Wrist calibration: the roll angle accepted as "wrist 0" and the ±90°
    # window around it that manual roll moves are allowed to use.
    "wrist_file": "live/wrist_calibration.json",
    # Camera yaw: a small user-local rotation about the wrist camera's
    # optical axis, stacked on the pinned hand-eye calibration.  Its own
    # file: the calibration and rig_state are never rewritten for this.
    "camera_yaw_file": "live/camera_yaw.json",
    # Working envelope in board-frame mm.  The arm should stay over the playing
    # surface, both so the camera keeps the board in view and so a Cartesian
    # mistake cannot drive the jaws into the table beside it.
    "keep_in_x_mm": 118.0,
    "keep_in_y_mm": 118.0,
    "keep_in_z_min_mm": -5.0,
    "keep_in_z_max_mm": 320.0,
    "keep_in_enforce": True,
    # Acceptance gate for a solved Cartesian target.  This bounds the SOLVER's
    # residual, not physical landing: the ~19 mm model bias lives outside FK and
    # this gate cannot see it (Phase 1 gate decomposition).
    #
    # PHASE 2 FLIPPED 2026-11-24, mink + 3.0 mm as a pair.  The IRL gate passed
    # all five cases on exactly this configuration (park 94 mm, relocation
    # 158 mm, three small steps; residuals 0.6-2.0 deg) once the servo-deadband
    # loop fix landed.  3.0 mm on the least_squares path alone would refuse
    # 52/64 valid squares, which is why the two always change together.
    "move_tolerance_mm": 3.0,
    # Backlash take-up (2026-11-25).  The servos settle 0.2-0.7 deg away from a
    # commanded goal, and the sign depends on the direction the joint arrived
    # from (reports/deadband_root_cause_20261125.md).  For the final approach to
    # a target, the joint is driven this many degrees PAST it and then back, so
    # the play is always taken up from one known side and the last leg is a
    # short, known-length move instead of a stop inside the stiction band.
    # 0 disables it (the A/B switch); the gripper is never part of a take-up.
    "backlash_take_up_deg": 3.5,
    # Phase 2: mink solves /move.  ``least_squares`` stays supported in
    # live/solver.py for bisecting a solver problem on the arm, but it is no
    # longer the default.
    "ik_solver": "mink",
    # Cartesian targets mean the jaw tip.  The one chain->tip transform lives
    # in ``tip_frame.TIP_IN_CHAIN`` (MJCF-derived, see that module); it is not
    # a config value because three competing offsets in the tree is exactly
    # the bug this replaced.
    # Relaxed middle pose the controller can always return to: joint targets, so
    # homing never depends on IK finding a solution from a bad posture.
    "home_joints": {"shoulder_pan": 0.0, "shoulder_lift": -55.0, "elbow_flex": 55.0,
                    "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 40.0},
    # Recovery pose (owner, 2026-11-24): "if the robot gets stuck, it should
    # always just reset to home position high above middle square and start
    # again".  This is a STORED joint pose, not an IK target, because the thing
    # that is stuck is often the solver — a recovery that needs a solve is not a
    # recovery.  Derived once, offline, with tools/recovery_pose.py:
    #   board tip (0.17, -0.91, 116.53) mm, 1.07 mm from (0, 0, 116),
    #   approach 0.12 deg from straight down, wrist_roll 87.74 (play branch),
    #   all six values inside the measured servo limits.
    # z is 116 rather than 130..140 because a jaws-down hover above the centre
    # needs wrist_flex ~102.5 at z=130, past the calibrated 98.79 limit; this is
    # the highest centre hover that is actually reachable with the jaws down.
    # One-time leader capture, 2026-11-24 13:58 EST (tools/leader_pose.py: a
    # read-only Present_Position read on a free leader bus, no register writes,
    # wrist_flex forced to 0): {pan 29.8, lift -102.68, elbow 96.44, flex 0,
    # roll -29.85}.  FK puts its tip at board (-144.6, -24.7, 188.7) mm with the
    # jaws 96 deg off vertical and elbow_flex 96.44 past its 94.75 limit, i.e.
    # the leader is parked away from the middle square.  REJECTED, so the stored
    # pose below is the offline-solved fallback.  Receipt with every reason:
    # reports/recovery_leader_candidate.json.  The leader is NOT polled at
    # runtime; recovery never reads it.
    # The gripper value is the one to keep, never to actuate.
    "recovery_joints": {"shoulder_pan": -12.37, "shoulder_lift": -13.02,
                        "elbow_flex": 4.8, "wrist_flex": 98.34,
                        "wrist_roll": 0.0, "gripper": 43.88},
    # How many times one operation may drive home before it gives up, and how
    # many times the original move is retried after a recovery.  Both bounded on
    # purpose: a stuck robot must never loop.
    "recovery_max_attempts": 3,
    "move_retries_after_recovery": 1,
    "recovery_log": "live/recovery_log.jsonl",
}


def load_config() -> dict:
    config = dict(DEFAULTS)
    if CONFIG_PATH.exists():
        config.update(json.loads(CONFIG_PATH.read_text()))
    mount = current_layout()["mount"]
    config.setdefault("base_x_m", mount["white_arm_base_frame_mm"][0] / 1000.0)
    config.setdefault("base_y_m", mount["white_arm_base_frame_mm"][1] / 1000.0)
    return config


class _JointFrame:
    """Servo frame -> model frame offset for wrist_roll.

    Re-zeroing wrist_roll in the servo EEPROM changes the physical joint
    coordinate.  The MJCF model and the hand-eye calibration were built in the
    old frame, so the dashboard keeps working there and translates only at the
    bridge boundary.  Zero until a servo zero is actually written.
    """

    def __init__(self) -> None:
        self.offset_deg = 0.0


class _FramedBridge:
    def __init__(self, inner: HttpBridge, frame: _JointFrame) -> None:
        self.inner = inner
        self.frame = frame

    def state(self) -> dict:
        state = dict(self.inner.state())
        offset = self.frame.offset_deg
        joints = dict(state.get("joints") or {})
        if "wrist_roll" in joints:
            joints["wrist_roll"] = float(joints["wrist_roll"]) + offset
        state["joints"] = joints
        limits = dict(state.get("limits") or {})
        if "wrist_roll" in limits:
            low, high = limits["wrist_roll"]
            limits["wrist_roll"] = [float(low) + offset, float(high) + offset]
        state["limits"] = limits
        return state

    def command(self, joints, **kwargs):
        values = [float(v) for v in joints]
        values[4] -= self.frame.offset_deg
        return self.inner.command(values, **kwargs)

    def __getattr__(self, name):
        return getattr(self.inner, name)


class _FramedFrames:
    def __init__(self, inner: HttpFrames, frame: _JointFrame) -> None:
        self.inner = inner
        self.frame = frame

    def capture(self):
        captured = self.inner.capture()
        joints = list(captured.joints_deg)
        joints[4] -= self.frame.offset_deg
        return replace(captured, joints_deg=tuple(joints))

    def __getattr__(self, name):
        return getattr(self.inner, name)


class Live:
    """Shared state: bridge handles, latest annotations, scan progress."""

    def __init__(self) -> None:
        self.config = load_config()
        self.frame_offset = _JointFrame()
        self.bridge = _FramedBridge(
            HttpBridge(self.config["bridge_url"],
                       token_file=self.config["token_file"], timeout=6.0),
            self.frame_offset)
        self.frames = _FramedFrames(
            HttpFrames(self.config["bridge_url"],
                       token_file=self.config["token_file"]),
            self.frame_offset)
        self.calibration = WristCalibration.load(self.config["calibration"])
        self.plan = plan_in_degrees(load_plan(self.config["plan"]))
        # The board's marker layout: the mat tags, or the printed ChArUco face.
        # Without this the derive path silently skips every 5..36 sighting,
        # because the mat layout only knows ids 0..4.
        if self.config.get("layout_kind", "mat") == "charuco":
            from chess_sim.charuco_face import CharucoLayout
            self.layout = CharucoLayout(
                square_mm=float(self.config.get("square_m", 0.028)) * 1000.0)
        else:
            self.layout = MarkerLayout(
                offset_mm=float(self.config["layout_offset_mm"]))
        # Wrist perception scan: a separate plan (sim joint units converted to
        # bridge degrees here) and the reference-free model.  It is additive;
        # /scan keeps the marker plan and the aim overlay keeps TargetDetector.
        self.wrist_poses = load_wrist_poses()
        self.wrist_plan = MarkerScanPlan(
            poses=tuple(PoseCoverage(pose.name, plan_pose_degrees(pose), (), ())
                        for pose in self.wrist_poses),
            min_views=1,
            layout_offset_mm=float(self.config.get("layout_offset_mm", 152.0)),
            note="wrist perception plan (bridge units)")
        self.lock = threading.Lock()
        self.scan_lock = threading.Lock()
        self.last_scan: dict | None = None
        self.scan_running = False
        self.last_board: dict | None = None
        self.last_board_dir: str | None = None
        self.board_scan_running = False
        self.board_scan_error: str | None = None
        self.last_error: str | None = None
        self.annotated: bytes | None = None
        self.annotated_ts = 0.0
        self.clients = 0
        self.stream_thread: threading.Thread | None = None
        self.poses_path = Path(self.config.get("poses_file", "live/poses.json"))
        self.capture_dir = Path(self.config.get("capture_dir", "live/captures"))
        # Must exceed whatever the bridge last accepted, even across restarts:
        # a modulo counter that wrapped behind last_seq froze every command.
        self.command_seq = int(time.time() * 1000)
        self.motion = threading.Lock()
        self.board_path = Path(self.config.get("board_file", "live/board.json"))
        self.tip_center_path = Path(self.config.get(
            "tip_center_file", "live/tip_center.json"))
        self.aim_path = Path(self.config.get("aim_file", "live/aim.json"))
        self._aim: dict | None = None
        self.wrist_path = Path(self.config.get(
            "wrist_file", "live/wrist_calibration.json"))
        self.yaw_path = Path(self.config.get(
            "camera_yaw_file", "live/camera_yaw.json"))
        self.yaw = camera_yaw.load(self.yaw_path)
        self.yaw_measured: float | None = None
        self.yaw_measured_offset: float | None = None
        self.yaw_measured_at: str | None = None
        self.frame_offset.offset_deg = float(
            self.wrist_cal().get("model_offset_deg", 0.0))
        self.kinematics = ArmKinematics(self.calibration, {}, self._board_frame())
        # The aim ray: one mount, one plane estimate, shared by /state, the
        # overlay and the aim_point move mode.  The mount is the calibrated
        # camera-in-tool of the *installed* tool — the B5 reference mount in
        # chess_sim/tool_reference.py is a 90 deg reconstruction and is not it
        # (tools/aim_validate.py: plane tilt 4.5 vs 62 deg).
        self.mount = CameraMount.from_calibration(self.calibration)
        self.plane_tracker = PlaneTracker(
            window=int(self.config.get("aim_plane_window", 9)))
        self.aim_lock = threading.Lock()
        self.aim_square_mm = float(self.config.get("square_m", 0.028)) * 1000.0
        # Phase 1: the solver behind /move. Default ``least_squares`` keeps the
        # incumbent path; ``mink`` swaps only the numerical solve. Board-frame
        # bookkeeping, mode expansion, span/roll-lock, keep-in and every guard
        # stay in ArmKinematics/Live, so the flag cannot change motion safety.
        self.solver = MoveSolver(
            self.kinematics, calibration=self.calibration,
            tip_in_chain=TIP_IN_CHAIN,
            use_mink=str(self.config.get("ik_solver", "least_squares")).lower()
            in ("mink", "qp", "so101ik"))
        if self.solver.use_mink:
            # The live console refreshes limits from the bridge at every move;
            # the mink model must follow, or it plans past the servo's range.
            self.solver.sync_limits({name: list(value) for name, value
                                     in self.kinematics.limits.items()})
        self.frame_times: deque[float] = deque(maxlen=40)
        self.frame_cost_ms: deque[float] = deque(maxlen=40)
        # Target overlay: one detector, its own thread.  It never runs on the
        # stream thread and never touches motion state.
        self.overlay = None
        if self.config.get("target_overlay", True):
            try:
                self.overlay = TargetOverlay(
                    self.config["calibration"],
                    self.config["target_reference"],
                    px_per_mm=float(self.config.get("target_px_per_mm", 4.0)),
                    refresh_s=float(self.config.get("target_refresh_s", 0.5)),
                    state_fn=self.bridge.state)
            except Exception as exc:      # a broken overlay must not stop the view
                self.last_error = (f"target overlay off: "
                                   f"{type(exc).__name__}: {exc}")

    # ------------------------------------------------------------------ board
    def _board_frame(self):
        spec = {"x_mm": float(self.config.get("board_x_m", 0.0)) * 1000.0,
                "y_mm": float(self.config.get("board_y_m", 0.0)) * 1000.0,
                "z_mm": float(self.config.get("board_z_m", 0.0)) * 1000.0,
                "yaw_deg": float(self.config.get("board_yaw_deg", 0.0))}
        if self.board_path.exists():
            try:
                spec.update({k: float(v) for k, v in json.loads(
                    self.board_path.read_text()).items() if k in spec})
            except (OSError, ValueError, TypeError):
                pass
        from kinematics import frame_at
        return frame_at(spec["x_mm"], spec["y_mm"], spec["z_mm"], spec["yaw_deg"])

    def board(self) -> dict:
        return self.kinematics.board_mm()

    # ------------------------------------------------- overlay pose propagation
    def tool_pose_board(self):
        """T_board_tool from the measured joints, for overlay propagation.

        The detector's own pose needs markers; this one needs only the encoders,
        so the overlay can keep projecting while the markers are occluded.
        """
        try:
            state = self.bridge.state()
            joints = state.get("joints") or {}
            if len(joints) != 6:
                return None
            values = [float(joints[name]) for name in JOINTS]
            return np.linalg.inv(self._board_frame()) @ self.kinematics.fk(values)
        except Exception:
            return None

    def tool_and_tip(self):
        """(T_board_tool, tip_mm, T_base_tool) from one joint read.

        The tip marker is the projection of this tip point through the solved
        pose, not a stored pixel: the cartesian move puts the jaw square to the
        board over the target square, so at that pose the tip projects onto the
        middle of that square and follows pitch on its own.
        """
        try:
            state = self.bridge.state()
            joints = state.get("joints") or {}
            if len(joints) != 6:
                return None, None, None
            values = [float(joints[name]) for name in JOINTS]
            board_frame = self.kinematics.board
            flange = self.kinematics.fk(values)
            tip = (flange @ self.jaw_frame())[:3, 3]
            tip_mm = self.kinematics.to_board(tip) * 1000.0
            return (np.linalg.inv(board_frame) @ flange,
                    [float(v) for v in tip_mm], flange)
        except Exception:
            return None, None, None

    def _propagate_snapshot(self, snap: dict, tool_now) -> dict:
        """Swap in a projected grid/aim when the live solve has gone away."""
        # OFF by default: validated 2026-11-23 against a real second solve, the
        # FK propagation disagreed by 6.6 deg / 18 mm, putting a projected point
        # 91-144 px from where the board actually was. A marker that wrong is
        # worse than an absent one. Enable only after the camera-to-tool (or the
        # FK orientation) is trustworthy.
        if not self.config.get("propagate_pose", False):
            return snap
        if tool_now is None:
            return snap
        result = snap.get("result") or {}
        age = snap.get("age_s") or 0.0
        if result.get("board") and age < 1.2:
            return snap
        solution = self.overlay.propagated(tool_now) if self.overlay else None
        if solution is None:
            return snap
        board = self.overlay.detector.board
        res = dict(result)
        try:
            res["grid"] = board.grid(solution)
            aim = self.aim()
            if aim.get("set"):
                uv = board.project(solution, (aim["x_mm"], aim["y_mm"]),
                                   aim.get("z_mm", 0.0))
                res["aim"] = {"x_mm": aim["x_mm"], "y_mm": aim["y_mm"],
                              "z_mm": aim.get("z_mm", 0.0),
                              "uv": [round(float(uv[0]), 2), round(float(uv[1]), 2)],
                              "label": aim.get("label", "aim")}
        except Exception:
            return snap
        out = dict(snap)
        out["result"] = res
        out["propagated"] = True
        return out

    def set_board(self, body: dict) -> dict:
        current = self.kinematics.board_mm()
        spec = {key: float(body.get(key, current[key]))
                for key in ("x_mm", "y_mm", "z_mm", "yaw_deg")}
        self.kinematics.set_board(spec["x_mm"], spec["y_mm"],
                                  spec["z_mm"], spec["yaw_deg"])
        self.board_path.parent.mkdir(parents=True, exist_ok=True)
        self.board_path.write_text(json.dumps(spec, indent=2) + "\n")
        return self.board()

    def assume_board(self, body: dict) -> dict:
        """Seed the board frame from the sim placement and one height measurement.

        Before calibration the board is assumed to sit where the sim puts it,
        so ``(0, 0)`` means the expected centre rather than a tag sighting.
        The surface height comes from a tape measure: the camera's FK height
        minus the measured standoff, which sidesteps every frame-convention
        question about where the chain's origin sits.
        """
        state = self._require_remote()
        self._refresh_limits(state)
        joints = [float(state["joints"][name]) for name in JOINTS]
        x_mm = float(body.get("x_mm", 0.0))
        y_mm = float(body.get("y_mm", 0.0))
        yaw_deg = float(body.get("yaw_deg", 0.0))
        height = body.get("camera_height_mm")
        if height is None:
            raise BridgeError("camera_height_mm is required")
        camera = self.kinematics.camera_pose(joints)
        if body.get("from_camera"):
            # The camera is looking at the centre target, so the board centre is
            # directly below it: anchor x/y to the camera and only z comes from
            # the measured standoff.
            x_mm = float(camera[0, 3]) * 1000.0
            y_mm = float(camera[1, 3]) * 1000.0
        z_mm = float(camera[2, 3]) * 1000.0 - float(height)
        return self.set_board({"x_mm": x_mm, "y_mm": y_mm,
                               "z_mm": z_mm, "yaw_deg": yaw_deg})

    # -------------------------------------------------------- tip centre (manual)
    def tip_center(self) -> dict:
        """User-set finger-tip midpoint pixel in the raw 640x480 wrist frame.

        The jaws are rigid to the wrist camera, so this is a fixed pixel.  The
        detection overlay reads it to place the true tool-centre marker without
        relying on the (unsolved) camera-to-jaw hand-eye transform.
        """
        data: dict = {}
        if self.tip_center_path.exists():
            try:
                data = json.loads(self.tip_center_path.read_text())
            except (OSError, ValueError):
                data = {}
        px = data.get("pixel")
        if not (isinstance(px, list) and len(px) == 2):
            px = None
        return {"set": px is not None,
                "pixel": px,
                "updated": data.get("updated"),
                "set_by": data.get("set_by"),
                "joints_deg": data.get("joints_deg")}

    def set_tip_center(self, u: float, v: float) -> dict:
        if not (0 <= u <= 640 and 0 <= v <= 480):
            raise ValueError("tip centre pixel must be inside 640x480")
        # Record the pose the click was made at. Measured 2026-11-23: u holds
        # under wrist pitch to +-2 px, while v tracks the camera's tilt to the
        # board, so a pixel without its pose is only half a fact.
        pose: dict = {}
        try:
            state = self.bridge.state()
            pose = {k: round(float(x), 2)
                    for k, x in (state.get("joints") or {}).items()}
        except Exception:                      # display metadata, never fatal
            pose = {}
        data = {"pixel": [round(float(u), 1), round(float(v), 1)],
                "updated": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "set_by": "user",
                "joints_deg": pose}
        self.tip_center_path.parent.mkdir(parents=True, exist_ok=True)
        self.tip_center_path.write_text(json.dumps(data, indent=2) + "\n")
        return self.tip_center()

    # ------------------------------------------------------- aim point (display)
    def aim(self) -> dict:
        """Board-mm point the grasp run is aiming at, drawn on the stream.

        Display only: the tool that moves the arm posts here so the recorded
        frames carry the commanded target beside the tip pixel. Nothing in the
        console acts on it.
        """
        if self._aim is None:
            data: dict = {}
            if self.aim_path.exists():
                try:
                    data = json.loads(self.aim_path.read_text())
                except (OSError, ValueError):
                    data = {}
            self._aim = data if data.get("set") else {"set": False}
        return dict(self._aim)

    def set_aim(self, x_mm: float, y_mm: float, z_mm: float,
                label: str = "aim") -> dict:
        data = {"set": True,
                "x_mm": round(float(x_mm), 2), "y_mm": round(float(y_mm), 2),
                "z_mm": round(float(z_mm), 2), "label": str(label)[:40],
                "updated": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        self.aim_path.parent.mkdir(parents=True, exist_ok=True)
        self.aim_path.write_text(json.dumps(data, indent=2) + "\n")
        self._aim = data
        return dict(data)

    def clear_aim(self) -> dict:
        self._aim = {"set": False}
        try:
            if self.aim_path.exists():
                self.aim_path.unlink()
        except OSError:
            pass
        return dict(self._aim)

    def aim_tuple(self):
        data = self.aim()
        if not data.get("set"):
            return None
        return (data["x_mm"], data["y_mm"], data.get("z_mm", 0.0),
                data.get("label", "aim"))

    # -------------------------------------------------------- wrist calibration
    def wrist_cal(self) -> dict:
        data: dict = {}
        if self.wrist_path.exists():
            try:
                data = json.loads(self.wrist_path.read_text())
            except (OSError, ValueError):
                data = {}
        zero = float(data.get("zero_deg", 0.0))
        # The window is stored relative to the zero.  Older files stored the
        # absolute angles (zero ± 90), so migrate those on read.
        if "min_rel_deg" in data or "max_rel_deg" in data:
            min_rel = float(data.get("min_rel_deg", -90.0))
            max_rel = float(data.get("max_rel_deg", 90.0))
        else:
            min_rel = float(data.get("min_deg", zero - 90.0)) - zero
            max_rel = float(data.get("max_deg", zero + 90.0)) - zero
        return {"zero_deg": round(zero, 2),
                "min_rel_deg": round(min_rel, 2),
                "max_rel_deg": round(max_rel, 2),
                "min_deg": round(zero + min_rel, 2),
                "max_deg": round(zero + max_rel, 2),
                "model_offset_deg": round(
                    float(data.get("model_offset_deg", 0.0)), 2),
                "pan_zero_deg": round(float(data.get("pan_zero_deg", 0.0)), 2),
                "calibrated": bool(data.get("calibrated")),
                "updated": data.get("updated")}

    def wrist_state(self, state: dict | None = None) -> dict:
        """Calibration plus the live roll, so the tab needs no extra poll."""
        cal = self.wrist_cal()
        if state is None:
            try:
                state = self.bridge.state()
            except BridgeError:
                state = {}
        joints = state.get("joints") or {}
        out = dict(cal)
        if "wrist_roll" in joints:
            roll = float(joints["wrist_roll"]) - self.frame_offset.offset_deg
            out["roll_deg"] = round(roll, 2)
            out["roll_rel_deg"] = round(roll - cal["zero_deg"], 2)
            out["inside"] = (cal["min_rel_deg"] - 0.01
                             <= out["roll_rel_deg"]
                             <= cal["max_rel_deg"] + 0.01)
        return out

    def set_wrist_zero(self, zero_deg: float | None = None) -> dict:
        """Accept the current roll as wrist 0, saving to the servo when possible.

        The servo write redefines its joint frame; the old servo zero is kept
        as ``model_offset_deg`` so the MJCF/hand-eye frame does not move.
        """
        state = self.bridge.state()
        joints = state.get("joints") or {}
        if "wrist_roll" not in joints:
            raise BridgeError("bridge has no wrist_roll state")
        offset = self.frame_offset.offset_deg
        servo_roll = (float(joints["wrist_roll"]) if zero_deg is None
                      else float(zero_deg)) - offset
        servo = None
        servo_error = None
        try:
            servo = self.bridge.inner.wrist_zero(90.0)
        except BridgeError as exc:
            text = str(exc)
            # The endpoint exists only after the bridge restart.  Until then,
            # keep the dashboard zero local instead of pretending it persisted.
            if "404" not in text and "Not Found" not in text:
                raise BridgeError(f"servo calibration failed: {exc}") from exc
            servo_error = text
        if servo:
            self.frame_offset.offset_deg = servo_roll
            zero = 0.0
            model_offset = servo_roll
        else:
            zero = servo_roll
            model_offset = offset
        current = self.wrist_cal()
        data = {"zero_deg": round(zero, 2),
                "min_rel_deg": -90.0,
                "max_rel_deg": 90.0,
                "model_offset_deg": round(model_offset, 2),
                "pan_zero_deg": current["pan_zero_deg"],
                "calibrated": True,
                "servo": servo,
                "updated": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        self.wrist_path.parent.mkdir(parents=True, exist_ok=True)
        self.wrist_path.write_text(json.dumps(data, indent=2) + "\n")
        result = self.wrist_state()
        result["servo_saved"] = bool(servo)
        if servo_error:
            result["servo_error"] = servo_error
        return result

    # ------------------------------------------------------- camera yaw offset
    def camera_yaw_state(self) -> dict:
        """Offset plus the last board measurement (None until one is taken)."""
        result = dict(self.yaw)
        measured = self.yaw_measured
        result["measured_deg"] = None if measured is None else round(measured, 2)
        result["measured_offset_deg"] = (
            None if self.yaw_measured_offset is None
            else round(self.yaw_measured_offset, 2))
        result["measured_stale"] = (
            measured is not None
            and round(self.yaw_measured_offset or 0.0, 3)
            != round(self.yaw["offset_deg"], 3))
        result["measured_at"] = self.yaw_measured_at
        result["path"] = str(self.yaw_path)
        result["calibration_id"] = self.calibration.calibration_id
        return result

    def _store_camera_yaw(self, offset_deg: float, source: str) -> dict:
        data = camera_yaw.save(self.yaw_path, offset_deg, source)
        self.yaw = {key: data[key] for key in ("schema", "offset_deg",
                                               "updated", "source")}
        return data

    def set_camera_yaw(self, offset_deg=None, delta_deg=None,
                       source: str = "manual") -> dict:
        """Set or nudge the offset; the pinned calibration is not touched."""
        if offset_deg is None and delta_deg is None:
            raise ValueError("offset_deg or delta_deg is required")
        with self.lock:
            if self.scan_running or self.board_scan_running:
                raise BridgeError("a scan is running; camera yaw change refused")
        previous = float(self.yaw["offset_deg"])
        if offset_deg is not None:
            value = float(offset_deg)
        else:
            value = previous + float(delta_deg)
        self._store_camera_yaw(value, source)
        result = self.camera_yaw_state()
        result["previous_offset_deg"] = round(previous, 3)
        return result

    def _board_target(self) -> Board:
        return Board({"intrinsics": self.calibration.intrinsics,
                      "distortion": self.calibration.distortion,
                      "chain": self.calibration.chain,
                      "camera_in_tool": self.calibration.camera_in_tool})

    def measure_camera_yaw(self) -> dict:
        """Board x-axis rotation in one fresh frame: capture and PnP only.

        No motion, no joint command and no calibration write.  The measurement
        is cached so the tab can show it and "accept as yaw 0" can use it.
        """
        with self.lock:
            if self.scan_running or self.board_scan_running:
                raise BridgeError(
                    "a scan is running; camera yaw measurement refused")
        # /capture goes quiet unless something consumes the bridge stream (the
        # same reason _hold_stream_client exists); hold a stream reference and
        # give a few frames a chance before reporting the bridge's own error.
        self.attach()
        try:
            error = None
            for _ in range(3):
                try:
                    frame = self.frames.capture()
                    break
                except BridgeError as exc:
                    error = exc
                    time.sleep(0.6)
            else:
                raise error
        finally:
            self.detach()
        if frame.image is None:
            raise BridgeError("bridge capture returned no image")
        image = np.asarray(frame.image)
        if image.ndim != 3 or image.shape[2] != 3:
            raise BridgeError(
                f"capture is not RGB HxWx3 (shape {image.shape})")
        if image.shape[:2] != (self.calibration.height,
                               self.calibration.width):
            raise BridgeError(
                f"capture is {image.shape[1]}x{image.shape[0]}, calibration is "
                f"{self.calibration.width}x{self.calibration.height}")
        board = self._board_target()
        solution, detections = board.solve(
            board.rectify(cv2.cvtColor(image, cv2.COLOR_RGB2BGR)))
        if solution is None or len(detections) < 2:
            raise BridgeError(
                "board PnP needs at least two ChArUco markers in view "
                f"(found {len(detections)}); nothing was changed")
        # The image yaw of the board alone is not the camera error: it includes
        # the board's pose relative to the arm.  Compare the PnP board pose in
        # the camera with the chain-predicted one and take the residual roll
        # about the camera's optical axis.
        rotation_cv, _ = cv2.Rodrigues(np.asarray(solution["rvec"], dtype=float))
        flip = np.diag([1.0, -1.0, -1.0])
        measured_board = np.eye(4)
        measured_board[:3, :3] = flip @ rotation_cv
        measured_board[:3, 3] = flip @ np.ravel(np.asarray(solution["tvec"], dtype=float))
        # Build the camera pose without WristCalibration.camera(): its hard
        # limit guard still carries the upstream flex limit (95 deg) while the
        # home pose sits at 98, so calling it here would reject a valid frame.
        # kinematics.fk is the same chain without that guard.
        camera_pose = self.kinematics.fk(frame.joints_deg) @ np.asarray(
            self.calibration.camera_in_tool)
        camera_pose[:3, :3] = (camera_pose[:3, :3]
                               @ camera_yaw.rotation_matrix(float(self.yaw["offset_deg"])))
        predicted_board = np.linalg.inv(camera_pose) @ np.asarray(self.kinematics.board)
        measured = camera_yaw.yaw_error_deg(measured_board[:3, :3],
                                           predicted_board[:3, :3])
        self.yaw_measured = float(measured)
        self.yaw_measured_offset = float(self.yaw["offset_deg"])
        self.yaw_measured_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        return {"measured_deg": round(float(measured), 2),
                "measured_offset_deg": round(self.yaw_measured_offset, 2),
                "rms_px": round(float(solution["rms_px"]), 2),
                "markers": [int(d.marker_id) for d in detections],
                "measured_at": self.yaw_measured_at}

    def accept_camera_yaw(self) -> dict:
        """offset -= measured, so the board reads level afterwards."""
        measurement = None
        measured = self.yaw_measured
        stale = (measured is None or self.yaw_measured_offset is None
                 or round(self.yaw_measured_offset, 3)
                 != round(self.yaw["offset_deg"], 3))
        if stale:
            # A cached angle taken at a different offset no longer nulls this
            # camera; measure again rather than apply the wrong correction.
            measurement = self.measure_camera_yaw()
            measured = float(measurement["measured_deg"])
        previous = float(self.yaw["offset_deg"])
        try:
            self._store_camera_yaw(previous - float(measured), "accept")
        except ValueError as exc:
            raise BridgeError(f"camera yaw not saved: {exc}") from exc
        result = self.camera_yaw_state()
        result["previous_offset_deg"] = round(previous, 3)
        result["measured_deg"] = round(float(measured), 2)
        if measurement:
            result["measurement"] = measurement
        return result

    def auto_measure_camera_yaw(self) -> dict:
        """Fresh ArUco measurement, then null it."""
        measurement = self.measure_camera_yaw()
        previous = float(self.yaw["offset_deg"])
        try:
            self._store_camera_yaw(
                previous - float(measurement["measured_deg"]), "aruco")
        except ValueError as exc:
            raise BridgeError(f"camera yaw not saved: {exc}") from exc
        result = self.camera_yaw_state()
        result["previous_offset_deg"] = round(previous, 3)
        result["measurement"] = measurement
        return result

    def _wrist_window(self) -> tuple[float, float] | None:
        cal = self.wrist_cal()
        if not cal.get("calibrated"):
            return None
        return cal["min_deg"], cal["max_deg"]

    @staticmethod
    def _marker_tilt(image) -> tuple[float | None, list]:
        """Mean tilt of the ArUco edges away from horizontal, in degrees.

        A square marker's edges are equivalent every 90°, so each edge angle is
        folded into [-45, 45] and averaged on the doubled angle to survive the
        ±45 seam.  Positive means the marker's top edge runs down to the right.
        """
        detections = fiducials.detect_markers(image)
        angles = []
        for detection in detections:
            corners = np.asarray(detection.corners, dtype=float)
            for a, b in ((0, 1), (3, 2)):
                vector = corners[b] - corners[a]
                if float(np.hypot(vector[0], vector[1])) < 1e-6:
                    continue
                angle = math.degrees(math.atan2(float(vector[1]), float(vector[0])))
                angles.append((angle + 45.0) % 90.0 - 45.0)
        if not angles:
            return None, detections
        sin_sum = sum(math.sin(math.radians(2.0 * a)) for a in angles)
        cos_sum = sum(math.cos(math.radians(2.0 * a)) for a in angles)
        return math.degrees(math.atan2(sin_sum, cos_sum)) / 2.0, detections

    def auto_align_wrist(self, tolerance_deg: float = 0.8,
                         max_iterations: int = 8) -> dict:
        """Rotate the wrist until the ArUco marker edges are horizontal.

        The roll->image gain and its sign are measured from the first move
        instead of assumed: the camera mount can invert or scale the rotation,
        and a wrong guess would drive the wrist the wrong way until the window
        stopped it.  Each iteration is a normal speed-capped streamed move, so
        the usual limit and fault guards still apply.
        """
        if not self.motion.acquire(timeout=5):
            raise BridgeError("another motion is running")
        try:
            state = self._require_remote()
            self._refresh_limits(state)
            current = [float(state["joints"][name]) for name in JOINTS]
            window = self._wrist_window()
            frame = self.frames.capture()
            tilt, detections = self._marker_tilt(frame.image)
            if tilt is None:
                raise BridgeError("no ArUco marker in view")
            before = tilt
            gain: float | None = None
            history: list[dict] = []
            iterations = 0
            for _ in range(max_iterations):
                if abs(tilt) <= tolerance_deg:
                    break
                if gain is None:
                    delta = max(-8.0, min(8.0, tilt))
                else:
                    delta = max(-12.0, min(12.0, -tilt / gain))
                if abs(delta) < 0.25:
                    break
                target = list(current)
                target[4] += delta
                if window:
                    target[4] = max(window[0], min(window[1], target[4]))
                previous_roll = current[4]
                if abs(target[4] - previous_roll) < 0.2:
                    raise BridgeError(
                        "wrist roll is at the edge of its calibrated window")
                self._keep_in_check(target)
                moved = self._stream_to(target, current)
                current = [float(moved["joints"][name]) for name in JOINTS]
                actual = current[4] - previous_roll
                iterations += 1
                time.sleep(0.45)
                frame = self.frames.capture()
                new_tilt, detections = self._marker_tilt(frame.image)
                if new_tilt is None:
                    raise BridgeError("marker left the view during auto-align")
                if abs(actual) > 0.2:
                    observed = (new_tilt - tilt) / actual
                    if 0.15 <= abs(observed) <= 5.0:
                        gain = observed
                history.append({"roll_deg": round(actual, 2),
                                "tilt_deg": round(new_tilt, 2)})
                tilt = new_tilt
            return {"ok": abs(tilt) <= max(tolerance_deg * 1.5, 1.2),
                    "tilt_before": round(before, 2),
                    "tilt_after": round(tilt, 2),
                    "tolerance_deg": tolerance_deg,
                    "iterations": iterations,
                    "roll_deg": round(current[4], 2),
                    "markers": [d.marker_id for d in detections],
                    "history": history}
        finally:
            self.motion.release()

    def derive_board(self, save: bool = True) -> dict:
        """Board pose in the arm base frame from one known marker sighting.

        PnP against the marker's *board-frame* corners gives the camera pose in
        board coordinates; FK plus the hand-eye mount gives it in base
        coordinates; composing the two yields the board transform.  No tape
        measure and no marker-orientation assumption needed.
        """
        frame = self.frames.capture()
        detections = fiducials.detect_markers(frame.image)
        if not detections:
            raise BridgeError("no marker in view")
        state = self.bridge.state()
        joints = state.get("joints") or {}
        if len(joints) != 6:
            raise BridgeError("bridge has no joint state")
        self._refresh_limits(state)
        values = [float(joints[name]) for name in JOINTS]
        base_camera = self.kinematics.camera_pose(values)
        candidates = []
        for detection in detections:
            try:
                # marker_corners_world already returns metres; dividing again
                # made the marker 1000x too small and PnP collapsed to a
                # degenerate solution with the camera at the board origin.
                corners = np.asarray(marker_corners_world(
                    self.layout, detection.marker_id, origin_xy=(0, 0),
                    yaw=0, marker_z=0), dtype=float)
            except (ValueError, KeyError):
                continue
            # Both IPPE branches, same calibrated lens for the fit and the
            # residual, lower residual wins (the first branch produced a 32 deg
            # outlier on a 10-marker view, reports/rot_scan_20261127.md).
            solved = pnp_tool_frame(corners,
                                    np.asarray(detection.corners, dtype=float),
                                    self.calibration)
            if solved is None:
                continue
            # pnp_tool_frame returns board -> calibrated camera (-z forward,
            # +y up), which is the frame base_camera lives in.  The old code
            # skipped that flip *and* inverted the transform: on the sim bench
            # that put the board plane 641 mm above the true surface and turned
            # yaw by 20 deg, while x/y looked deceptively plausible.
            camera_from_board = solved[0]
            board = base_camera @ camera_from_board
            camera_pts = (camera_from_board @ np.c_[corners, np.ones(len(corners))].T).T[:, :3]
            projected = project_tool_frame(camera_pts, self.calibration)
            error_px = per_component_rms(projected, detection.corners)
            candidates.append({
                "marker_id": detection.marker_id,
                "reproj_px": round(error_px, 2),
                "x_mm": round(float(board[0, 3]) * 1000.0, 1),
                "y_mm": round(float(board[1, 3]) * 1000.0, 1),
                "z_mm": round(float(board[2, 3]) * 1000.0, 1),
                "yaw_deg": round(float(math.degrees(
                    math.atan2(board[1, 0], board[0, 0]))), 2),
            })
        if not candidates:
            raise BridgeError("markers in view are not in the board layout")
        candidates.sort(key=lambda item: item["reproj_px"])
        best = candidates[0]
        spread = max(max(c[k] for c in candidates) - min(c[k] for c in candidates)
                     for k in ("x_mm", "y_mm", "z_mm")) if len(candidates) > 1 else 0.0
        if save:
            self.set_board({"x_mm": best["x_mm"], "y_mm": best["y_mm"],
                            "z_mm": best["z_mm"], "yaw_deg": best["yaw_deg"]})
        return {"board": {k: best[k] for k in ("x_mm", "y_mm", "z_mm", "yaw_deg")},
                "from_marker": best["marker_id"], "reproj_px": best["reproj_px"],
                "cross_check_mm": round(spread, 1), "candidates": candidates,
                "saved": bool(save)}

    def cart_limits(self, step_mm: float = 4.0) -> dict:
        """Reachable span of each board axis at the current pose.

        The jaws-down solution for a given (y, z) usually has a contiguous
        range in x, so bisection from the current point finds the ends without
        a full workspace scan.  Orientation is held at jaws-down throughout.
        """
        state = self._require_remote()
        self._refresh_limits(state)
        start = [float(state["joints"][name]) for name in JOINTS]
        here = self.kinematics.to_board(self.kinematics.fk(start)[:3, 3]) * 1000.0
        ranges = {}
        for axis, index in (("x", 0), ("y", 1), ("z", 2)):
            limits = []
            for direction in (-1.0, 1.0):
                reach = float(here[index])
                probe = reach
                for _ in range(28):
                    probe = reach + direction * step_mm
                    target = here.copy()
                    target[index] = probe
                    # Hold the arm's *current* orientation: a jog panel asks how
                    # far this pose can slide, not how far a jaws-down pose could.
                    goal = self.kinematics.fk(start).copy()
                    goal[:3, 3] = self.kinematics.board_point(
                        float(target[0]), float(target[1]), float(target[2]))
                    solved = self.solver.solve(goal, start)
                    if (solved["position_error_mm"] > 2.0
                            or solved["approach_error_deg"] > 2.0):
                        break
                    reach = probe
                limits.append(round(reach, 1))
            ranges[axis] = {"min": min(limits), "max": max(limits),
                            "now": round(float(here[index]), 1)}
        return {"axes": ranges, "step_mm": step_mm}

    def pose_now(self) -> dict:
        state = self.bridge.state()
        joints = state.get("joints") or {}
        if len(joints) != 6:
            raise BridgeError("bridge has no joint state")
        limits = state.get("limits") or {}
        if limits:
            self.kinematics.limits = {n: (float(v[0]), float(v[1]))
                                      for n, v in limits.items()}
        values = [float(joints[name]) for name in JOINTS]
        tip = (self.kinematics.fk(values) @ self.jaw_frame())[:3, 3]
        tip_mm = [round(float(v), 1) for v in self.kinematics.to_board(tip) * 1000.0]
        return dict(self.kinematics.report(values), tip_mm=tip_mm,
                    joints_deg={n: round(float(joints[n]), 2) for n in JOINTS},
                    board=self.kinematics.board_mm())

    # ------------------------------------------------------------------ scan
    def start_scan(self) -> dict:
        if not self.scan_lock.acquire(blocking=False):
            raise HTTPException(409, "a scan is already running")
        try:
            state = self.bridge.state()
            if state.get("mode") != "REMOTE" or not state.get("armed"):
                self.scan_lock.release()
                raise HTTPException(409, "bridge must be REMOTE and armed")
        except BridgeError as exc:
            self.scan_lock.release()
            raise HTTPException(502, str(exc)) from exc
        thread = threading.Thread(target=self._scan_worker, name="scan", daemon=True)
        thread.start()
        return {"started": True}

    def _scan_worker(self) -> None:
        with self.lock:
            self.scan_running = True
            self.last_error = None
        try:
            controller = ScanController(
                self.bridge, self.frames,
                self.camera_from_joints, self.plan,
                layout=self.layout,
                square_m=self.config["square_m"],
                board_z_m=self.config["board_z_m"],
                marker_z_m=self.config["marker_z_m"],
                base_xy_m=(self.config["base_x_m"], self.config["base_y_m"]),
                board_half_mm=self.config["board_half_mm"],
                settle_s=self.config["settle_s"],
                pose_timeout_s=self.config["pose_timeout_s"],
                speed_scale=self.config["speed_scale"])
            report = controller.run()
            with self.lock:
                self.last_scan = report.as_dict()
                if not report.ok:
                    self.last_error = report.note or "scan failed"
        except Exception as exc:  # noqa: BLE001 - surface, never die silently
            with self.lock:
                self.last_error = f"{type(exc).__name__}: {exc}"
        finally:
            with self.lock:
                self.scan_running = False
            self.scan_lock.release()

    def camera_from_joints(self, joints_deg):
        camera = self.calibration.camera(
            tuple(math.radians(v) for v in joints_deg))
        # Camera yaw: camera-local rotation about the optical axis, applied
        # on top of the pinned calibration the app loaded at startup.
        return camera_yaw.apply(camera, self.yaw["offset_deg"])

    # ------------------------------------------------------- wrist perception
    def start_board_scan(self) -> dict:
        """Run the wrist perception plan and classify all 64 squares."""
        if not self.scan_lock.acquire(blocking=False):
            raise HTTPException(409, "a scan is already running")
        try:
            state = self.bridge.state()
            if state.get("mode") != "REMOTE" or not state.get("armed"):
                raise HTTPException(409, "bridge must be REMOTE and armed")
        except BridgeError as exc:
            self.scan_lock.release()
            raise HTTPException(502, str(exc)) from exc
        except Exception:
            self.scan_lock.release()
            raise
        threading.Thread(target=self._board_scan_worker, name="board-scan",
                         daemon=True).start()
        return {"started": True}

    def _board_scan_worker(self) -> None:
        with self.lock:
            self.board_scan_running = True
            self.board_scan_error = None
        stop_stream = threading.Event()
        stream_reader = threading.Thread(
            target=self._hold_stream_client, args=(stop_stream,),
            name="board-scan-stream", daemon=True)
        try:
            # Home before the task (AGENTS invariant), then hold the motion
            # lock so /move cannot interleave with the pose sequence.
            self.recover_to_home(reason="pre-board-scan")
            stream_reader.start()
            if not self.motion.acquire(timeout=10):
                raise BridgeError("another motion is running; cannot scan")
            try:
                board_z = float(self.kinematics.board[2, 3])
                controller = ScanController(
                    self.bridge, self.frames, self.camera_from_joints,
                    self.wrist_plan,
                    layout=self.layout,
                    square_m=self.config["square_m"],
                    board_z_m=board_z,
                    marker_z_m=board_z,
                    base_xy_m=(self.config["base_x_m"], self.config["base_y_m"]),
                    board_half_mm=self.config["board_half_mm"],
                    settle_s=self.config["settle_s"],
                    pose_timeout_s=self.config["pose_timeout_s"],
                    speed_scale=self.config["speed_scale"])
                report = controller.run()
                self._save_board_captures(controller.captured)
                if not report.ok:
                    raise RuntimeError(report.note or "wrist scan failed")
                base = load_inputs()
                inputs = ScanInputs(
                    calibration=base.calibration, poses=base.poses,
                    model=base.model, square=base.square,
                    board_z=board_z, marker_z=board_z,
                    layout=base.layout, config_path=base.config_path)
                frames = frames_from_captured(controller.captured)
                if not frames:
                    raise RuntimeError("wrist scan captured no fresh frames")
                result = run_wrist_scan(frames, inputs)
                with self.lock:
                    self.last_board = summarize_wrist_scan(result)
            finally:
                self.motion.release()
        except Exception as exc:                       # noqa: BLE001
            with self.lock:
                self.board_scan_error = f"{type(exc).__name__}: {exc}"
        finally:
            stop_stream.set()
            try:
                self.recover_to_home(reason="post-board-scan")
            except Exception:                          # noqa: BLE001
                pass
            with self.lock:
                self.board_scan_running = False
            self.scan_lock.release()

    def _hold_stream_client(self, stop: threading.Event) -> None:
        """Keep one /stream client attached: the bridge stops producing frames
        for /capture when nothing consumes the stream (measured 2026-11-25:
        pose 10 of a live wrist scan failed with 503 'no camera frame')."""
        import urllib.request

        port = int(self.config.get("port", 8080))
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/stream",
                                        timeout=10) as response:
                while not stop.is_set():
                    if not response.read(65536):
                        break
        except Exception:                              # noqa: BLE001
            pass

    def _save_board_captures(self, captured) -> None:
        """Persist the accepted scan frames so a failed fusion can be diagnosed."""
        from PIL import Image

        stamp = time.strftime("%Y%m%dT%H%M%S")
        out = self.capture_dir.parent / "captures_board_scan" / stamp
        out.mkdir(parents=True, exist_ok=True)
        rows = []
        for name, frame in captured:
            filename = f"{name}.png"
            Image.fromarray(frame.image).save(out / filename)
            rows.append({"pose": name, "file": filename,
                         "joints_deg": [float(v) for v in frame.joints_deg],
                         "timestamp": float(frame.timestamp)})
        (out / "manifest.json").write_text(json.dumps(rows, indent=2) + "\n")
        with self.lock:
            self.last_board_dir = str(out)

    # ------------------------------------------------------------- poses/jog
    def poses(self) -> dict:
        if not self.poses_path.exists():
            return {}
        try:
            return json.loads(self.poses_path.read_text())
        except (OSError, ValueError):
            return {}

    def next_pose_name(self) -> str:
        store = self.poses()
        index = 1
        while f"pos_{index}" in store:
            index += 1
        return f"pos_{index}"

    def progress(self) -> dict:
        counts: dict[int, int] = {}
        total = 0
        if self.capture_dir.exists():
            for path in self.capture_dir.glob("*.json"):
                try:
                    record = json.loads(path.read_text())
                except ValueError:
                    continue
                if record.get("duplicate_of"):
                    continue
                ids = {m["id"] for m in record.get("markers", [])}
                if not ids:
                    continue
                total += 1
                for marker_id in ids:
                    counts[marker_id] = counts.get(marker_id, 0) + 1
        return {"views": total, "target": 12,
                "per_tag": {str(k): counts[k] for k in sorted(counts)}}

    def save_pose(self, name: str = "") -> dict:
        state = self.bridge.state()
        joints = state.get("joints") or {}
        if len(joints) != 6:
            raise BridgeError("bridge has no joint state")
        store = self.poses()
        if not name:
            name = self.next_pose_name()
        store[name] = {k: float(joints[k]) for k in joints}
        self.poses_path.parent.mkdir(parents=True, exist_ok=True)
        # Keep one rolling backup: this file is runtime state and has been
        # clobbered once by a stale copy synced from a dev machine.
        if self.poses_path.exists():
            try:
                self.poses_path.with_suffix(".json.bak").write_bytes(
                    self.poses_path.read_bytes())
            except OSError:
                pass
        self.poses_path.write_text(json.dumps(store, indent=2) + "\n")
        return {"name": name, "joints": store[name]}

    def capture_view(self, label: str = "") -> dict:
        # Two frames 300 ms apart must agree: a capture taken while the arm is
        # still moving pairs an image with joint angles from a different pose,
        # which silently poisons hand-eye calibration.
        first = self.frames.capture()
        first_state = self.bridge.state().get("joints") or {}
        time.sleep(0.3)
        frame = self.frames.capture()
        moved_px = 0.0
        first_det = fiducials.detect_markers(first.image)
        now_det = fiducials.detect_markers(frame.image)
        first_by_id = {d.marker_id: d.corners for d in first_det}
        for detection in now_det:
            if detection.marker_id in first_by_id:
                moved_px = max(moved_px, float(np.abs(
                    np.asarray(detection.corners)
                    - np.asarray(first_by_id[detection.marker_id])).max()))
        state_now = self.bridge.state().get("joints") or {}
        moved_deg = max((abs(float(state_now[n]) - float(first_state[n]))
                         for n in first_state if n in state_now), default=0.0)
        if moved_px > 1.5 or moved_deg > 0.4:
            return {"name": None, "moving": True,
                    "moved_px": round(moved_px, 2), "moved_deg": round(moved_deg, 3),
                    "markers": []}
        state = self.bridge.state()
        joints = state.get("joints") or {}
        drift = max(abs(float(joints[n]) - frame.joints_deg[i])
                    for i, n in enumerate(("shoulder_pan", "shoulder_lift",
                                           "elbow_flex", "wrist_flex",
                                           "wrist_roll", "gripper")))
        detections = fiducials.detect_markers(frame.image)
        markers = []
        for detection in detections:
            edges = [float(np.linalg.norm(detection.corners[i]
                                          - detection.corners[(i + 1) % 4]))
                     for i in range(4)]
            markers.append({"id": detection.marker_id,
                            "edge_px": round(float(np.mean(edges)), 1),
                            "corners": [[round(float(x), 2), round(float(y), 2)]
                                        for x, y in detection.corners]})
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        # Second resolution collides when captures are looped faster than 1 Hz:
        # the second write overwrites the first and a caller reading by name
        # silently measures a stale frame.  The counter makes every name unique.
        self._cap_seq = getattr(self, "_cap_seq", 0) + 1
        stamp = time.strftime("%Y%m%dT%H%M%S") + f"{self._cap_seq % 1000:03d}"
        name = f"{stamp}_{label}".strip("_")
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise BridgeError("could not encode frame")
        (self.capture_dir / f"{name}.jpg").write_bytes(encoded.tobytes())
        record = {"name": name, "label": label, "timestamp": frame.timestamp,
                  "joints_deg": {n: float(v) for n, v in zip(
                      ("shoulder_pan", "shoulder_lift", "elbow_flex",
                       "wrist_flex", "wrist_roll", "gripper"), frame.joints_deg)},
                  "state_joints_deg": joints, "joint_drift_deg": round(drift, 3),
                  "mode": state.get("mode"), "markers": markers,
                  "frame_age_s": round(time.time() - frame.timestamp, 3)}
        record["duplicate_of"] = None
        for path in sorted(self.capture_dir.glob("*.json")):
            if path.stem == name:
                continue
            try:
                other = json.loads(path.read_text())
            except ValueError:
                continue
            if not other.get("markers") or not markers:
                continue
            if {m["id"] for m in other["markers"]} != {m["id"] for m in markers}:
                continue
            delta = max(abs(float(joints[n]) - float(other["state_joints_deg"][n]))
                        for n in joints if n in other.get("state_joints_deg", {}))
            else_delta = max(abs(a - b) for a, b in zip(
                [m["edge_px"] for m in markers],
                [m["edge_px"] for m in other["markers"]]))
            if delta < 2.0 and else_delta < 3.0:
                record["duplicate_of"] = other.get("name", path.stem)
                break
        (self.capture_dir / f"{name}.json").write_text(json.dumps(record, indent=2) + "\n")
        return record

    def captures(self) -> dict:
        items = []
        if self.capture_dir.exists():
            for path in sorted(self.capture_dir.glob("*.json")):
                try:
                    record = json.loads(path.read_text())
                except ValueError:
                    continue
                items.append({"name": record.get("name", path.stem),
                              "markers": [m["id"] for m in record.get("markers", [])],
                              "edges": [m["edge_px"] for m in record.get("markers", [])],
                              "drift": record.get("joint_drift_deg")})
        return {"count": len(items), "items": items}

    def _require_remote(self) -> dict:
        state = self.bridge.state()
        if state.get("mode") != "REMOTE" or not state.get("armed"):
            raise BridgeError("bridge must be REMOTE and armed")
        return state

    def jog(self, joint: str, delta_deg: float) -> dict:
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll", "gripper")
        if joint not in names:
            raise BridgeError(f"unknown joint {joint!r}")
        if abs(delta_deg) > 15.0:
            raise BridgeError("a single jog is limited to 15 deg")
        if not self.motion.acquire(timeout=5):
            raise BridgeError("another motion is running")
        try:
            state = self._require_remote()
            start = [float(state["joints"][n]) for n in names]
            target = list(start)
            target[names.index(joint)] += delta_deg
            note = None
            window = self._wrist_window()
            if window and joint == "wrist_roll":
                lo, hi = window
                offset = self.frame_offset.offset_deg
                clamped = max(lo, min(hi, target[4] - offset))
                if abs(clamped - (target[4] - offset)) > 1e-9:
                    note = (f"wrist roll clamped to {clamped:.1f}° "
                            f"(window {lo:.1f}..{hi:.1f}°)")
                    target[4] = clamped + offset
            # Refuse a jog that runs past a joint stop instead of clamping it and
            # letting the servo sit still.  Clamping silently is what made a
            # +5 deg wrist_flex jog at 97.5 deg (limit 98.79) look like a dead
            # joint: the goal was clamped to the stop, nothing actuated, and only
            # the stall guard's message revealed why (measured 2026-11-24).
            limits = self.kinematics.limits.get(joint)
            if limits:
                asked = target[names.index(joint)]
                if asked < limits[0] - 0.5 or asked > limits[1] + 0.5:
                    raise BridgeError(
                        f"{joint} jog to {asked:.2f}° is outside its limits "
                        f"{limits[0]:.2f}..{limits[1]:.2f}°")
            self._keep_in_check(target)
            result = self._stream_with_take_up(target, start)
            if note:
                result["note"] = note
            return result
        finally:
            self.motion.release()

    def run_pose(self, name: str) -> dict:
        store = self.poses()
        if name not in store:
            raise BridgeError(f"no saved pose {name!r}")
        if not self.motion.acquire(timeout=5):
            raise BridgeError("another motion is running")
        try:
            state = self._require_remote()
            names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                     "wrist_flex", "wrist_roll", "gripper")
            start = [float(state["joints"][n]) for n in names]
            target = [float(store[name][n]) for n in names]
            return self._stream_to(target, start)
        finally:
            self.motion.release()

    # ------------------------------------------------------------- cartesian
    def sanity_check(self, execute: bool = True, z_mm: float = 120.0,
                     tolerance_mm: float = 6.0) -> dict:
        """Visit the four board corners and the centre, reporting each result.

        A quick proof that the board frame, the IK and the arm all agree: if the
        corners are reachable and the moves land where asked, Cartesian control
        is trustworthy.  Corners are square centres (a1, h1, a8, h8), not the
        board edge, so the jaws stay clear of the border.
        """
        state = self._require_remote()
        self._refresh_limits(state)
        half = float(self.config.get("square_m", 0.028)) * 3.5 * 1000.0
        targets = [("a1", -half, -half), ("h1", half, -half),
                   ("a8", -half, half), ("h8", half, half), ("centre", 0.0, 0.0)]
        rows = []
        for name, x_mm, y_mm in targets:
            state = self.bridge.state()
            start = [float(state["joints"][n]) for n in JOINTS]
            solved, _goal = self.solver.solve_tip(
                self.kinematics.board_point(x_mm, y_mm, z_mm), start,
                self.jaw_frame())
            row = {"target": name, "x_mm": round(x_mm, 1), "y_mm": round(y_mm, 1),
                   "z_mm": round(z_mm, 1),
                   "ik_mm": solved["position_error_mm"],
                   "ik_deg": solved["approach_error_deg"],
                   "reachable": solved["position_error_mm"] <= tolerance_mm
                                 and solved["approach_error_deg"] <= max(4.0, tolerance_mm / 3.0)}
            if execute and row["reachable"]:
                try:
                    moved = self.move({"mode": "xyz", "x_mm": x_mm, "y_mm": y_mm,
                                       "z_mm": z_mm})
                    pose = moved.get("pose") or {}
                    # The command is tip-space, so the landing is the tip too.
                    tool = pose.get("tip_mm") or [None, None, None]
                    row["moved"] = True
                    row["settle_deg"] = moved.get("error_deg")
                    row["landed_mm"] = [round(float(v), 1) if v is not None else None
                                        for v in tool]
                    row["landed_error_mm"] = round(
                        math.hypot(float(tool[0]) - x_mm, float(tool[1]) - y_mm), 1) \
                        if tool[0] is not None else None
                except BridgeError as exc:
                    row["moved"] = False
                    row["error"] = str(exc)
            rows.append(row)
        ok = sum(1 for r in rows if r["reachable"] and r.get("moved", True))
        return {"z_mm": round(z_mm, 1), "targets": rows,
                "passed": ok, "total": len(rows),
                "executed": bool(execute)}

    def home(self, execute: bool = True) -> dict:
        """Move to the stored middle pose, or report where that pose lands."""
        home = {name: float(self.config["home_joints"].get(name, 0.0)) for name in JOINTS}
        state = self._require_remote()
        self._refresh_limits(state)
        start = [float(state["joints"][name]) for name in JOINTS]
        target = [home[name] for name in JOINTS]
        pose_tool = self.kinematics.to_board(
            self.kinematics.fk(target)[:3, 3]) * 1000.0
        zone = self.keep_in()
        inside = (abs(pose_tool[0]) <= zone["x_mm"] and abs(pose_tool[1]) <= zone["y_mm"]
                  and zone["z_min_mm"] <= pose_tool[2] <= zone["z_max_mm"])
        result = {"home_joints": {k: round(v, 1) for k, v in home.items()},
                  "tool_mm": [round(float(v), 1) for v in pose_tool],
                  "inside_zone": bool(inside), "executed": False}
        if not inside:
            result["note"] = "home pose lies outside the keep-in zone; adjust home_joints"
            return result
        bad = self.kinematics.violations(target)
        if bad:
            result["note"] = "home pose outside joint limits"
            return result
        if execute:
            moved = self._stream_to(target, start)
            result["executed"] = True
            result["settle_deg"] = moved.get("error_deg")
            result["pose"] = self.pose_now()
        return result

    def jaw_frame(self) -> np.ndarray:
        """Chain->tip transform: the single convention, never a config override."""
        return TIP_IN_CHAIN

    def tip_pose_mm(self) -> list[float]:
        """Jaw-tip position in board coordinates, which is what targets mean."""
        state = self.bridge.state()
        joints = [float(state["joints"][n]) for n in JOINTS]
        tip = self.kinematics.fk(joints) @ self.jaw_frame()
        return [round(float(v), 1) for v in self.kinematics.to_board(tip[:3, 3]) * 1000.0]

    def keep_in(self) -> dict:
        return {"x_mm": float(self.config.get("keep_in_x_mm", 118.0)),
                "y_mm": float(self.config.get("keep_in_y_mm", 118.0)),
                "z_min_mm": float(self.config.get("keep_in_z_min_mm", -5.0)),
                "z_max_mm": float(self.config.get("keep_in_z_max_mm", 320.0)),
                "enforce": bool(self.config.get("keep_in_enforce", True))}

    def _keep_in_check(self, joints_deg) -> None:
        """Refuse a pose that leaves the working envelope.

        Checked on the measured joints *after* solving, not on the request, so a
        Cartesian target that is inside the box but needs the jaws to swing out
        to reach it is still caught.
        """
        # The envelope is expressed at the JAW TIP, but the chain frame rides
        # above it, so compare the tip height or every valid pick looks like a
        # ceiling violation (a 240 mm tip read as 325 mm and was refused).
        problem = self._keep_in_problem(joints_deg)
        if problem:
            raise StuckError(problem)

    def _excess(self, joints) -> float:
        """Total degrees outside the measured joint limits."""
        total = 0.0
        for index, name in enumerate(JOINTS[:5]):
            if index >= len(joints):
                break
            lo, hi = self.kinematics.limits.get(name, (-180.0, 180.0))
            value = float(joints[index])
            total += max(0.0, lo - value) + max(0.0, value - hi)
        return total

    def _refresh_limits(self, state: dict) -> None:
        limits = state.get("limits") or {}
        if limits:
            self.kinematics.limits = {name: (float(v[0]), float(v[1]))
                                      for name, v in limits.items()}
            # Keep the mink model on the same measured ranges as the chain, so
            # a flag-on move plans inside the servo's actual travel.
            self.solver.sync_limits(limits)

    def move(self, body: dict) -> dict:
        """One motion request: ``xyz``, ``look_at`` or ``joint``.

        All three resolve to joint angles and then stream through the same
        speed-capped path as a jog, so limit and fault guards are unchanged.
        """
        mode = str(body.get("mode") or "xyz").lower()
        if not self.motion.acquire(timeout=5):
            raise BridgeError("another motion is running")
        try:
            return self._move_locked(body, mode)
        finally:
            self.motion.release()

    def _solved(self, call, *args, **kwargs):
        """Run one solve; a solver exception is a stuck condition, not a 500.

        A crashed solver is exactly the case the recovery primitive exists for,
        so it is reported as ``StuckError`` (which the move path may recover
        from) rather than escaping as an unhandled error.
        """
        try:
            return call(*args, **kwargs)
        except StuckError:
            raise
        except Exception as exc:                     # noqa: BLE001
            raise StuckError(
                f"solver failed: {type(exc).__name__}: {exc}") from exc

    def _move_locked(self, body: dict, mode: str) -> dict:
        """Guards, solve and stream, with the motion lock held by ``move``."""
        state = self._require_remote()
        self._refresh_limits(state)
        start = [float(state["joints"][name]) for name in JOINTS]
        span = body.get("span_deg")
        span_arg = None if span is None else float(span)
        detail: dict = {"mode": mode}
        if mode == "joint":
            wanted = body.get("joints") or {}
            target = [float(wanted.get(name, start[i]))
                      for i, name in enumerate(JOINTS)]
            detail = {"mode": "joint"}
        elif mode == "aim":
            # Keep the camera where it is and only rotate it onto the point.
            # "look_at" pins the camera to a sphere around the target, which
            # this arm cannot always reach; aiming from the current position
            # is a far smaller ask and is what framing actually needs.
            point = self.kinematics.board_point(
                float(body.get("x_mm", 0.0)), float(body.get("y_mm", 0.0)),
                float(body.get("z_mm", 0.0)))
            camera = self.kinematics.camera_pose(start)
            span = point - camera[:3, 3]
            distance = float(np.linalg.norm(span))
            if distance < 0.02:            # metres, not millimetres
                raise BridgeError("aim point is too close to the camera")
            view = span / distance
            up = body.get("up") or "y"
            from kinematics import _orthonormal
            hint = self.kinematics.board_axis(str(up))
            rotation = _orthonormal(view, hint)
            if body.get("roll_deg"):
                from scipy.spatial.transform import Rotation as _R
                rotation = rotation @ _R.from_euler(
                    "z", float(body["roll_deg"]), degrees=True).as_matrix()
            goal = self.kinematics.tool_from_camera(
                np.block([[rotation, camera[:3, 3:4]], [np.zeros(3), 1.0]]))
            solved = self._solved(self.solver.solve_multi, goal, start)
            if (solved["position_error_mm"] > 8.0
                    or solved["approach_error_deg"] > 6.0):
                raise StuckError(
                    "cannot aim there: off by "
                    f"{solved['position_error_mm']} mm / "
                    f"{solved['approach_error_deg']} deg")
            target = [float(solved["joints_deg"][name]) for name in JOINTS]
            detail = dict(solved, aim_distance_mm=round(distance * 1000.0, 1))
        elif mode == "aim_point":
            # Aim the RAY, not the tip: drive until the tool axis meets the
            # board on the requested square.  The grasp path above is
            # untouched; this is the mode that says "put the aim there".
            return self._aim_point_locked(body, state, start)
        elif mode == "rel":
            # Nudge the tool along the board axes while keeping the current
            # orientation.  Needs the board's axes, not its origin, so it
            # works before the board pose has been derived.
            delta = np.array([float(body.get("dx_mm", 0.0)),
                              float(body.get("dy_mm", 0.0)),
                              float(body.get("dz_mm", 0.0))]) / 1000.0
            if float(np.linalg.norm(delta)) <= 1e-9:
                raise BridgeError("a relative move needs dx_mm, dy_mm or dz_mm")
            if float(np.linalg.norm(delta)) > 0.12:
                raise BridgeError("a relative move is limited to 120 mm")
            current = self.kinematics.fk(
                [float(state["joints"][n]) for n in JOINTS])
            goal = current.copy()
            goal[:3, 3] = current[:3, 3] + self.kinematics.board[:3, :3] @ delta
            solved = self.solver.solve_multi(
                goal, start, freeze_roll=bool(body.get("roll_lock", True)))
            # A small nudge near the edge of reach lands a few millimetres
            # short; tolerate that rather than refusing the whole move.
            allowed = max(5.0, 0.6 * float(np.linalg.norm(delta)) * 1000.0)
            if (solved["position_error_mm"] > allowed
                    or solved["approach_error_deg"] > 3.0):
                raise BridgeError(
                    "no reachable pose: off by "
                    f"{solved['position_error_mm']} mm / "
                    f"{solved['approach_error_deg']} deg")
            target = [float(solved["joints_deg"][name]) for name in JOINTS]
            detail = solved
        elif mode in ("xyz", "look_at"):
            x_mm = float(body.get("x_mm", 0.0))
            y_mm = float(body.get("y_mm", 0.0))
            z_mm = float(body.get("z_mm", 0.0))
            if mode == "xyz":
                if not -40.0 <= z_mm <= 260.0:
                    raise BridgeError("tool height must be within -40..260 mm")
                # z is the jaw-tip height.  solve_tip owns the whole
                # search (fast path, pan seeds, quarter-turn spans) and the
                # one chain->tip transform, so the app cannot drift from
                # the regression test.
                point = self.kinematics.board_point(x_mm, y_mm, z_mm)
                solved, goal = self._solved(
                    self.solver.solve_tip, point, start, self.jaw_frame(),
                    float(span_arg or 0.0))
            else:
                distance = float(body.get("distance_mm", 220.0))
                if not 120.0 <= distance <= 650.0:
                    raise BridgeError("standoff distance must be 120..650 mm")
                goal = self.kinematics.look_at_target(
                    x_mm, y_mm, z_mm, distance,
                    float(body.get("pitch_deg", 0.0)),
                    float(body.get("azimuth_deg", 0.0)),
                    float(body.get("roll_deg", 0.0)))
            if mode != "xyz":
                # xyz already carries its own searched solution; re-solving
                # here would silently discard it and drop back to the seed
                # basin, which is exactly how good corner solutions vanished.
                solved = self._solved(
                    partial(self.solver.solve_multi, goal, start,
                            span_deg=span_arg,
                            freeze_roll=bool(body.get("roll_lock", False))))
            # Tolerance is measured at the tip now (solve() takes the jaw
            # frame).  Refusing a 6.5 mm landing would waste a reachable
            # square, while the reported error still tells the truth.
            gate = float(self.config.get("move_tolerance_mm", 20.0))
            if (solved["position_error_mm"] > gate
                    or solved["approach_error_deg"] > 8.0):
                raise StuckError(
                    "no reachable pose: off by "
                    f"{solved['position_error_mm']} mm / "
                    f"{solved['approach_error_deg']} deg")
            target = [float(solved["joints_deg"][name]) for name in JOINTS]
            detail = solved
        else:
            raise BridgeError(f"unknown move mode {mode!r}")
        if body.get("jaw") is not None:
            target[5] = float(body["jaw"])
        return self._execute_target(target, start, body, detail)

    def _execute_target(self, target: list[float], start: list[float],
                        body: dict, detail: dict) -> dict:
        """Guard, then stream to a joint target — the tail of every move.

        Split out of ``_move_locked`` so the closed-loop aim controller
        (``mode: aim_point``) drives through exactly the same limits, wrist
        window, keep-in check and workspace stream as a Cartesian move.  One
        code path, one set of guards.
        """
        if body.get("jaw") is not None and len(target) > 5:
            target = list(target)
            target[5] = float(body["jaw"])
        window = self._wrist_window()
        if window and abs(target[4] - start[4]) > 1e-9:
            lo, hi = window
            servo_target = target[4] - self.frame_offset.offset_deg
            if not lo <= servo_target <= hi:
                raise BridgeError(
                    f"wrist roll {servo_target:.1f}° outside calibrated window "
                    f"{lo:.1f}..{hi:.1f}°")
        bad = self.kinematics.violations(target)
        if bad and self._excess(target) > self._excess(start) + 0.05:
            # A joint sitting a hair outside its limit (post-jam resets do
            # this) must not lock the arm out of every move: refuse only
            # when the target makes the violation worse than it already is.
            raise BridgeError("target outside joint limits: " + ", ".join(
                f"{name} {value} not in {lo}..{hi}"
                for name, value, (lo, hi) in bad))
        delta = max(abs(a - b) for a, b in zip(target[:5], start[:5]))
        if body.get("dry_run"):
            # Solve without moving: lets a large reorientation be inspected
            # before the arm swings through anything.  The take-up is planned
            # too, so a dry run answers "what would this move actually do"
            # including the overshoot leg and any reason it was refused.
            plan = self._take_up_request(target, start)
            return {"dry_run": True, "requested": detail,
                    "target_joints": dict(zip(JOINTS, [round(v, 2) for v in target])),
                    "max_joint_delta_deg": round(delta, 1),
                    "take_up": self._take_up_receipt(plan)}
        self._keep_in_check(target)
        settle = float(body.get("settle_deg", 0.7))
        result = self._stream_with_take_up(target, start, tolerance=settle)
        result.update(requested=detail, pose=self.pose_now())
        return result

    # ------------------------------------------------------- the aim ray
    def feed_aim_plane(self, snap: dict) -> bool:
        """Add one marker board solve to the running plane estimate.

        The plane must be carried into the base frame with the tool pose that
        was current *when the frame was solved* — the arm moves between solves,
        so pairing a fresh pose with an old board solve would tilt the plane.
        """
        board = ((snap.get("result") or {}).get("board")) or {}
        pose = snap.get("tool_at_solve")
        key = snap.get("solve_key")
        if not board or pose is None or key is None:
            return False
        normal, point, _info = plane_from_solution(board, pose, self.mount,
                                                   self.kinematics.board)
        if normal is None:
            return False
        with self.aim_lock:
            return self.plane_tracker.add(normal, point, key=key,
                                          rms_px=board.get("rms_px"),
                                          ids=len(board.get("ids") or []))

    def aim_readout(self, pose=None, joints_deg=None, *,
                    plane_max_age_s: float | None = None) -> dict:
        """`AIM_WORLD`, `AIM_PIXEL`, `TIP_PIXEL` and a validity flag.

        The board plane comes from the markers (averaged over recent solves),
        never from the configured board placement: the placement is a seed for
        planning, not a measurement of where the board is.
        """
        if pose is None:
            if joints_deg is None:
                state = self.bridge.state()
                joints_deg = [float(state["joints"][name]) for name in JOINTS]
            pose = self.kinematics.fk(joints_deg)
        with self.aim_lock:
            plane = self.plane_tracker.plane()
        readout = aim_evaluate(
            pose, self.mount, plane,
            board_frame=self.kinematics.board,
            board_half_mm=float(self.config.get("board_half_mm", 122.0)),
            square_mm=self.aim_square_mm,
            plane_max_age_s=float(self.config.get("aim_plane_max_age_s", 30.0)
                                  if plane_max_age_s is None else plane_max_age_s))
        if joints_deg is not None:
            readout["joints_deg"] = {name: round(float(joints_deg[i]), 2)
                                     for i, name in enumerate(JOINTS)}
        return readout

    def aim_point_move(self, body: dict) -> dict:
        """Dry-run an aim-point correction without touching the arm."""
        return self.move(dict(body, mode="aim_point", dry_run=True))

    def _aim_solve(self, goal, seed, freeze_roll: bool) -> dict:
        """Solve a small aim correction with the continuity-regularised IK.

        Deliberately **not** the ``MoveSolver``/mink path: measured on 2026-11-24
        at a 97 deg wrist_flex pose, a 12 mm translation returned ``pos_err 1.20
        mm, approach 43.4 deg`` from the mink path (and a target that FK puts
        88 mm from the goal) while the least-squares path returned ``0.30 mm,
        0.12 deg`` with a 4 deg joint change.  The mink state is reused between
        calls and at this pose it lands in the wrong basin; the grasp path
        (``solve_tip``) is a different method and is unaffected and untouched.
        The error is measured at the jaw tip, so the number the gate checks is
        the number that matters.
        """
        return self._solved(self.kinematics.solve_multi, goal, seed,
                            freeze_roll=freeze_roll, tool_frame=self.jaw_frame())

    def _aim_point_locked(self, body: dict, state: dict, start: list[float]) -> dict:
        """Closed loop: translate the tool until the aim ray lands on target.

        One step is exact in principle — a rigid translation of the tool moves
        the plane intersection by the in-plane part of that translation — so the
        only residual is IK error and the servo settle, and two or three steps
        take it out.  Each step goes through `_execute_target`, so limits, the
        wrist window, keep-in and the workspace stream are the same ones every
        other move uses.  Orientation is held (`freeze_roll`), so this never
        silently re-aims the arm while it is centring.
        """
        target_xy = (float(body.get("x_mm", 0.0)), float(body.get("y_mm", 0.0)))
        if target_xy == (0.0, 0.0) and "x_mm" not in body:
            raise BridgeError("aim_point needs x_mm and y_mm")
        tolerance = float(body.get("tolerance_mm", 5.0))
        iterations = max(1, min(6, int(body.get("iterations", 4))))
        step_cap = float(body.get("max_step_mm", 45.0))
        dry = bool(body.get("dry_run"))
        # Wrist roll does not move the aim — the jaw origin and the tip both sit
        # on the rolled axis (tip-center.md §1) — so holding roll is a preference.
        # If the held-roll solve cannot keep the approach, retry with roll free
        # rather than refusing a correction the arm could plainly make.
        roll_lock = bool(body.get("roll_lock", True))
        roll_freed = False
        max_age = float(body.get("plane_max_age_s",
                                self.config.get("aim_plane_max_age_s", 15.0)))
        measured = [float(v) for v in start]
        steps: list[dict] = []
        first = None
        last = None
        base_pose = self.kinematics.fk(measured)
        base_z = float(base_pose[2, 3])
        previous_error = None
        stalled = False
        for index in range(iterations):
            pose = self.kinematics.fk(measured)
            readout = self.aim_readout(pose, measured, plane_max_age_s=max_age)
            if readout.get("aim_board_mm") is None:
                raise BridgeError("aim_point: " + (readout.get("reason")
                                                   or "no usable board plane"))
            if readout["plane"]["age_s"] > max_age:
                raise BridgeError(
                    "aim_point needs a fresh marker board solve: last one is "
                    f"{readout['plane']['age_s']:.0f} s old (limit {max_age:.0f} s)")
            if first is None:
                first = readout
            last = readout
            aim_mm = readout["aim_board_mm"]
            error = float(np.hypot(target_xy[0] - aim_mm[0], target_xy[1] - aim_mm[1]))
            step = {"iteration": index,
                    "aim_board_mm": aim_mm,
                    "error_mm": round(error, 2),
                    "aim_pixel": readout.get("aim_pixel"),
                    "axis_board": [round(float(v), 4) for v in
                                   (self.kinematics.board[:3, :3].T
                                    @ (-pose[:3, 2]))]}
            steps.append(step)
            if error <= tolerance:
                step["done"] = True
                break
            if previous_error is not None and error > previous_error * 0.9:
                # Two steps without a 10% gain means the servo settle is moving
                # the tool more than the correction: stop and report the number
                # rather than grinding the arm sideways for nothing.
                step["stalled"] = True
                stalled = True
                break
            previous_error = error
            with self.aim_lock:
                plane = self.plane_tracker.plane()
            delta = aim_translation(pose, plane, target_xy, self.kinematics.board)
            length = float(np.linalg.norm(delta)) * 1000.0
            if length > step_cap:
                delta = delta * (step_cap / length)
                length = step_cap
            # Solve from the pose the FIRST step started at, with the height and
            # orientation pinned: the stream settles 2-3 deg short, so a goal that
            # chases the current pose ratchets the sag into a slow descent and
            # ratchets the tilt into aim drift (measured 2026-11-24: z 110 -> 81 mm
            # and the axis 1.8 -> 8.4 deg over four steps when it did).
            goal = base_pose.copy()
            goal[:3, 3] = pose[:3, 3] + delta
            goal[2, 3] = base_z
            solved = self._aim_solve(goal, measured, roll_lock)
            if solved["position_error_mm"] > 8.0:
                raise StuckError(
                    "aim_point: no reachable pose for a "
                    f"{length:.0f} mm aim correction (off by "
                    f"{solved['position_error_mm']} mm)")
            # Hold the approach too.  A solve that reaches the position by
            # flipping the tool over would move the aim somewhere else entirely
            # (the axis, not the point, is what aims), so it is refused rather
            # than executed.
            if solved["approach_error_deg"] > 6.0 and roll_lock and not roll_freed:
                retry = self._aim_solve(goal, measured, False)
                if retry["approach_error_deg"] <= 6.0:
                    solved, roll_lock, roll_freed = retry, False, True
            if solved["approach_error_deg"] > 6.0:
                raise StuckError(
                    "aim_point: the correcting pose would tilt the tool "
                    f"{solved['approach_error_deg']:.0f} deg; refusing to move")
            target = [float(solved["joints_deg"][name]) for name in JOINTS]
            if body.get("jaw") is not None:
                target[5] = float(body["jaw"])
            detail = dict(solved, mode="aim_point", step_mm=round(length, 1))
            step.update(step_mm=round(length, 1),
                        solve_position_mm=solved.get("position_error_mm"),
                        solve_approach_deg=solved.get("approach_error_deg"),
                        solve_starts=solved.get("starts"),
                        exact=solved.get("joints_deg") is None)
            if dry:
                step["target_joints"] = dict(
                    zip(JOINTS, [round(v, 2) for v in target]))
                step["step_mm"] = round(length, 1)
                step["dry_run"] = True
                break
            executed = self._execute_target(target, measured, body, detail)
            measured = [float(executed["joints"][name]) for name in JOINTS]
            step["executed"] = {"error_deg": executed.get("error_deg"),
                                "commands": executed.get("commands")}
        if not dry:
            final = self.aim_readout(self.kinematics.fk(measured), measured,
                                     plane_max_age_s=max_age)
            last = final
        final_mm = (last or {}).get("aim_board_mm") or [float("nan"), float("nan")]
        return {"mode": "aim_point",
                "dry_run": dry,
                "target_mm": [round(target_xy[0], 1), round(target_xy[1], 1)],
                "tolerance_mm": tolerance,
                "start_aim_mm": (first or {}).get("aim_board_mm"),
                "final_aim_mm": final_mm,
                "final_error_mm": round(float(np.hypot(
                    target_xy[0] - final_mm[0], target_xy[1] - final_mm[1])), 2)
                if np.isfinite(final_mm[0]) else None,
                "square": (last or {}).get("square"),
                "aim_pixel": (last or {}).get("aim_pixel"),
                "offset_px": (last or {}).get("offset_px"),
                "plane": (last or {}).get("plane"),
                "iterations": len([s for s in steps if not s.get("dry_run")]),
                "stalled": stalled,
                "steps": steps,
                "joints_deg": {name: round(measured[i], 2)
                               for i, name in enumerate(JOINTS)},
                "pose": self.pose_now()}

    def _stream_to(self, target: list[float], start: list[float],
                   tolerance: float = 0.7, trace: list | None = None,
                   settle_allow: float | None = None,
                   deadline_s: float | None = None,
                   stall_lead_deg: float | None = None) -> dict:
        """Drive to the target and stop when the MEASURED joints are close.

        The original loop compared its own interpolated command against the
        target and stopped there, so a move ended wherever the command happened
        to be: with ~2 deg of gravity sag on the shoulder that left every move
        about 5 mm short while reporting error_deg 0.3 (measured 2026-11-23).
        So the stop test uses the encoder readback, and the command is
        ``measured + residual``.

        Hardened after the 2026-11-23 "accepted and never executed" report
        (see ``reports/phase2_flip_20261124.md``). Three rules make a silent
        non-execution impossible:

        1. ``start`` is a *seed*, never a measurement. ``measured`` is only
           ever read from the bridge, and the first read happens before any
           stop decision. A missing/short readback is a hard error, not a
           silent freeze at ``start``.
        2. Zero commands sent is never reported as success for a move that
           needed travel. If the required joint delta is beyond tolerance and
           nothing was actuated, that is an error.
        3. Ending on the deadline with a large residual raises, rather than
           returning a number the caller can mistake for "done".
        """
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll", "gripper")
        step = 10.5 * 0.066   # degrees per tick at the conservative 10.5 deg/s
        # The servos have a measured 2-4.4 deg directional deadband: a goal less
        # than that ahead of the present position is ignored outright.  The old
        # loop commanded ``measured + clamp(step)``, i.e. a 0.693 deg lead, which
        # no servo ever acted on -- so ``measured`` never changed, the lead never
        # grew, and every move sat still until the deadline (measured
        # 2026-11-24: 0.00 deg on all six joints with --sim --deadband 2.5,
        # while identical absolute targets moved shoulder_lift 25.36 -> -13.02).
        # The lead must therefore ACCUMULATE from the last command until it is
        # wide enough to cross the deadband.  The bridge still bounds it to
        # MAX_LAG_DEG ahead of the measured pose, so it cannot wind up a backlog.
        if settle_allow is None:
            settle_allow = float(self.config.get("move_settle_deadband_deg", 3.5))
        settle_allow = float(settle_allow)
        # The lead that means "not actuating" rather than "resting inside the
        # friction band".  The ordinary loop uses the settle allowance for both;
        # the take-up final leg needs them apart (see TAKE_UP_* above).
        stall_lead = (settle_allow if stall_lead_deg is None
                      else float(stall_lead_deg))
        stall_ticks = int(self.config.get("move_stall_ticks", 20))

        def read_measured() -> list[float]:
            """One encoder read. Short or empty state is a hard failure."""
            state = self.bridge.state()
            if state.get("mode") != "REMOTE" or not state.get("armed"):
                raise BridgeError("bridge left REMOTE during motion")
            joints = state.get("joints") or {}
            if len(joints) != len(names) or any(n not in joints for n in names):
                # 9252a5b did `if len(joints) == 6: measured = ...`, so an empty
                # dict left `measured` frozen at `start` forever. That is a
                # code-level frozen readback; refuse it instead.
                raise BridgeError(
                    "bridge returned no usable joint readback"
                    f" (got {sorted(joints)}); cannot close the move loop")
            return [float(joints[n]) for n in names]

        # Rule 1: measure before deciding anything. `start` is only a seed.
        measured = read_measured()
        origin = list(measured)
        # The commanded pose accumulates toward the target (see the deadband note
        # above); it starts at the measured pose, never at the caller's seed, so
        # a stale `start` cannot inject an unvalidated jump.
        current = list(measured)
        needed = max(abs(a - b) for a, b in zip(target[:5], origin[:5]))
        started = time.time()
        deadline = started + (30.0 if deadline_s is None else float(deadline_s))
        sent = 0
        last_error = max(abs(a - b) for a, b in zip(measured, target))
        unchanged = 0
        while True:
            error = max(abs(a - b) for a, b in zip(measured, target))
            last_error = error
            if error <= max(0.15, tolerance):
                break
            # A residual inside the servo deadband will never shrink: stop
            # cleanly instead of grinding to the deadline.  It is a settle, not
            # a failure, and the honest residual is what gets reported.
            if error <= settle_allow and unchanged >= 8:
                break
            if time.time() >= deadline:
                break
            # Advance the COMMAND (not the measurement) so the lead grows; a
            # static sag is still taken out because `current` is re-seeded from
            # the measurement whenever the arm has actually caught up.
            for index, (sent_value, target_value, measured_value) in enumerate(
                    zip(current, target, measured)):
                if abs(sent_value - measured_value) < 1e-9:
                    sent_value = measured_value
                current[index] = sent_value + max(
                    -step, min(step, target_value - sent_value))
            self.command_seq += 1
            self.bridge.command(current, seq=self.command_seq, ttl_ms=200,
                                speed_scale=self.config.get("speed_scale", 0.3))
            sent += 1
            if trace is not None:
                trace.append({"t": round(time.time(), 3),
                              "sent": [round(v, 3) for v in current],
                              "measured": [round(v, 3) for v in measured]})
            time.sleep(0.066)
            previous = measured
            measured = read_measured()
            moved = max(abs(a - b) for a, b in zip(measured, previous))
            unchanged = unchanged + 1 if moved < 0.05 else 0
            # If the arm has not budged while the command is well ahead of it,
            # the joint is not actuating (torque off, jammed, or physically
            # blocked).  Say so now rather than at the deadline.
            lead = max(abs(a - b) for a, b in zip(current, measured))
            if unchanged >= stall_ticks and lead > stall_lead:
                raise StuckError(
                    f"joint not actuating: commanded {lead:.2f} deg ahead of "
                    f"measured for {unchanged} ticks with no motion "
                    f"(residual {error:.2f} deg of {needed:.2f} needed)")

        travel = max(abs(a - b) for a, b in zip(measured[:5], origin[:5]))
        # Rule 2: a move that needed travel but actuated nothing did not happen.
        if needed > max(0.15, tolerance) and sent == 0:
            raise StuckError(
                "move accepted no command: measured pose was already within "
                f"{max(0.15, tolerance):.2f} deg of the target, so nothing moved "
                f"(needed {needed:.2f} deg)")
        # Rule 3: a deadline stop that barely moved the arm is a failure, not a
        # result. Deliberately loose on residual and strict on travel: a move
        # that sits 2 deg out from sag has still executed, while one that covered
        # half the distance in 30 s did not.
        if travel < needed * 0.5 and last_error > max(5.0 * tolerance, 3.0):
            raise StuckError(
                f"move did not execute: residual {last_error:.2f} deg after "
                f"{sent} commands ({time.time() - started:.1f} s); "
                f"the arm moved {travel:.2f} of {needed:.2f} deg")
        return {"joints": dict(zip(names, measured)),
                "error_deg": round(last_error, 2),
                "commands": sent,
                "travel_deg": round(travel, 2),
                "needed_deg": round(needed, 2),
                "origin_deg": dict(zip(names, origin))}

    # ------------------------------------------------------------ take-up
    def _take_up_request(self, target: list[float],
                         start: list[float]) -> dict:
        """Plan the backlash take-up leg for an approach to ``target``.

        The servos settle 0.2-0.7 deg away from a commanded goal and the sign
        depends on the direction the joint arrived from, so a target that is
        only 1-2 deg away can be ignored outright and a direct approach stops
        short (or long) by a pose-dependent amount.  Driving PAST the target by
        a bounded margin and then commanding the target makes the final leg a
        short move from one known side, which is the primitive that took a
        1.49 deg wrist_roll stall to -0.26 deg by hand.

        This only plans; it does not move.  ``applied`` is False, with a
        reason, whenever the overshoot cannot be made safely: disabled,
        nothing but the gripper to move, the joint envelope, the calibrated
        wrist window, or the keep-in box.  The caller falls back to a direct
        move and records the reason.
        """
        margin = float(self.config.get("backlash_take_up_deg", 3.5))
        plan = {"deg": round(margin, 2), "applied": False, "reason": "",
                "target": None, "moving": []}
        if margin <= 0:
            plan["reason"] = "disabled (backlash_take_up_deg <= 0)"
            return plan
        moving = [index for index in range(5)
                  if abs(float(target[index]) - float(start[index])) >= 0.2]
        if not moving:
            plan["reason"] = "no non-gripper travel to take up"
            return plan
        overshoot = list(target)
        for index in moving:
            travel = float(target[index]) - float(start[index])
            overshoot[index] = float(target[index]) + math.copysign(margin, travel)
        # The gripper is a jaw, not a joint: it is held at the target in both
        # legs and never included in the overshoot.
        overshoot[5] = float(target[5])
        bad = self.kinematics.violations(overshoot)
        if bad and self._excess(overshoot) > self._excess(start) + 0.05:
            plan["reason"] = ("overshoot would leave the joint envelope: "
                              + ", ".join(f"{name} {value} not in {lo}..{hi}"
                                          for name, value, (lo, hi) in bad))
            return plan
        window = self._wrist_window()
        if window and 4 in moving:
            servo_target = overshoot[4] - self.frame_offset.offset_deg
            if not window[0] <= servo_target <= window[1]:
                plan["reason"] = (f"overshoot wrist roll {servo_target:.1f} deg "
                                  f"outside the calibrated window "
                                  f"{window[0]:.1f}..{window[1]:.1f}")
                return plan
        problem = self._keep_in_problem(overshoot)
        if problem:
            plan["reason"] = "overshoot " + problem
            return plan
        plan.update(applied=True, target=overshoot, moving=moving)
        return plan

    @staticmethod
    def _take_up_receipt(plan: dict) -> dict:
        """The plan as it appears in a move result: no hidden state, no guesses."""
        receipt = {"deg": plan["deg"], "applied": bool(plan["applied"]),
                   "reason": plan["reason"],
                   "moving": [JOINTS[index] for index in plan["moving"]]}
        if plan.get("target"):
            receipt["overshoot_deg"] = {name: round(float(plan["target"][i]), 2)
                                        for i, name in enumerate(JOINTS)}
        return receipt

    def _stream_with_take_up(self, target: list[float], start: list[float],
                             tolerance: float = 0.7,
                             trace: list | None = None) -> dict:
        """``_stream_to`` with the take-up leg in front of the final approach.

        Used by every Cartesian/joint move (``_execute_target``) and by
        ``jog``.  When the plan is refused, or when the overshoot leg itself
        stalls, the direct move still runs under the ordinary guards - the
        fallback is recorded in ``take_up.reason`` rather than hidden.
        """
        plan = self._take_up_request(target, start)
        if not plan["applied"]:
            result = self._stream_to(target, start, tolerance=tolerance,
                                     trace=trace)
            result["take_up"] = self._take_up_receipt(plan)
            return result
        try:
            leg = self._stream_to(plan["target"], start, trace=trace)
        except StuckError as exc:
            result = self._stream_to(target, start, tolerance=tolerance,
                                     trace=trace)
            receipt = self._take_up_receipt(plan)
            receipt.update(applied=False, fell_back=True,
                           reason=f"take-up leg did not execute: {exc}")
            result["take_up"] = receipt
            return result
        approach = [float(leg["joints"][name]) for name in JOINTS]
        result = self._stream_to(
            target, approach,
            tolerance=min(float(tolerance), TAKE_UP_SETTLE_DEG),
            settle_allow=TAKE_UP_SETTLE_ALLOW_DEG,
            stall_lead_deg=TAKE_UP_STALL_LEAD_DEG,
            deadline_s=TAKE_UP_FINAL_DEADLINE_S, trace=trace)
        receipt = self._take_up_receipt(plan)
        receipt.update(overshoot_error_deg=leg["error_deg"],
                       overshoot_commands=leg["commands"],
                       final_commands=result["commands"])
        # travel_deg/needed_deg describe the REQUEST the caller made, not the
        # last 3.5 deg leg: the two legs move in opposite directions.
        origin = leg["origin_deg"]
        final = result["joints"]
        result["travel_deg"] = round(max(
            abs(float(final[name]) - float(origin[name])) for name in JOINTS[:5]), 2)
        result["needed_deg"] = round(max(
            abs(float(target[i]) - float(origin[JOINTS[i]]))
            for i in range(5)), 2)
        result["take_up"] = receipt
        result["commands"] = leg["commands"] + result["commands"]
        return result

    # -------------------------------------------------------------- recovery
    def recovery_pose(self) -> list[float]:
        """The stored joint pose recovery drives to. Never solved at runtime."""
        pose = self.config.get("recovery_joints") or {}
        missing = [name for name in JOINTS if name not in pose]
        if missing:
            raise BridgeError("recovery_joints is missing " + ", ".join(missing))
        return [float(pose[name]) for name in JOINTS]

    def _keep_in_problem(self, joints_deg) -> str | None:
        """Why this joint pose is outside the keep-in box, or None."""
        zone = self.keep_in()
        if not zone["enforce"]:
            return None
        tip = (self.kinematics.fk(list(joints_deg)) @ self.jaw_frame())[:3, 3]
        tool = self.kinematics.to_board(tip) * 1000.0
        problems = []
        if abs(float(tool[0])) > zone["x_mm"]:
            problems.append(f"x {tool[0]:.0f} outside ±{zone['x_mm']:.0f}")
        if abs(float(tool[1])) > zone["y_mm"]:
            problems.append(f"y {tool[1]:.0f} outside ±{zone['y_mm']:.0f}")
        if not zone["z_min_mm"] <= float(tool[2]) <= zone["z_max_mm"]:
            problems.append(f"z {tool[2]:.0f} outside "
                            f"{zone['z_min_mm']:.0f}..{zone['z_max_mm']:.0f}")
        if not problems:
            return None
        return "keep-in zone: " + "; ".join(problems)

    def _clamp_to_limits(self, target: list[float],
                         start: list[float]) -> tuple[list[float], list[str]]:
        """Clamp a stored pose into the measured joint envelope.

        Recovery may skip the keep-in target check (the pose is inside by
        construction) but it may NOT skip joint-limit clamping: a stored pose is
        still only a request.  The gripper is never part of a recovery move, so
        it is held at its measured value.
        """
        clamped = list(target)
        notes = []
        for index, name in enumerate(JOINTS[:5]):
            limits = self.kinematics.limits.get(name)
            if not limits:
                continue
            lo, hi = float(limits[0]), float(limits[1])
            if not lo <= clamped[index] <= hi:
                value = min(max(clamped[index], lo), hi)
                notes.append(f"{name} {clamped[index]:.2f} -> {value:.2f}")
                clamped[index] = value
        clamped[5] = float(start[5])
        return clamped, notes

    def recover_to_home(self, reason: str = "manual") -> dict:
        """Drive to the stored home pose above the board centre.

        Deliberately dumb, and deliberately not an IK target: the things that
        get stuck include the solver, so a recovery that needs a solve is not a
        recovery.  The pose comes from ``recovery_joints`` in config (derived
        once by ``tools/recovery_pose.py``).

        Two legs when the pan has to move: raise the shoulder/elbow/wrist with
        pan held, then pan at height.  A single interpolated move from a folded
        pose can sweep the jaws sideways at low z, through the pieces.
        """
        target = self.recovery_pose()
        if not self.motion.acquire(timeout=10):
            raise BridgeError("another motion is running; cannot recover")
        started = time.time()
        record: dict = {"at": time.strftime("%Y-%m-%dT%H:%M:%S"), "reason": reason,
                        "stored_joints": dict(zip(JOINTS, [round(v, 2) for v in target]))}
        try:
            state = self._require_remote()
            self._refresh_limits(state)
            start = [float(state["joints"][name]) for name in JOINTS]
            target, notes = self._clamp_to_limits(target, start)
            record.update(from_joints=dict(zip(JOINTS, [round(v, 2) for v in start])),
                          target_joints=dict(zip(JOINTS, [round(v, 2) for v in target])),
                          clamped=notes)
            legs = []
            if abs(target[0] - start[0]) > 0.5:
                legs.append(("raise", [start[0], target[1], target[2],
                                       target[3], target[4], start[5]]))
            legs.append(("home", target))
            current = start
            for name, leg in legs:
                moved = self._stream_to(leg, current)
                current = [float(moved["joints"][joint]) for joint in JOINTS]
                legs_record = record.setdefault("legs", [])
                legs_record.append({"leg": name, "error_deg": moved["error_deg"],
                                    "travel_deg": moved["travel_deg"],
                                    "needed_deg": moved["needed_deg"]})
            problem = self._keep_in_problem(current)
            record.update(ok=problem is None, landed=problem,
                          seconds=round(time.time() - started, 2))
            if problem:
                raise StuckError(f"recovery landed outside keep-in: {problem}")
            return record
        except BridgeError as exc:
            record.update(ok=False, error=str(exc),
                          seconds=round(time.time() - started, 2))
            raise
        finally:
            self.motion.release()
            self._log_recovery(record)

    def move_with_recovery(self, body: dict) -> dict:
        """One move, with the stored home pose as the recovery path.

        A stuck condition (see ``StuckError``) or a move that ends outside
        keep-in drives the arm home once and retries the original request once.
        The retry budget and the recovery cap both come from config, and both
        are hard bounds: a stuck robot must never loop.
        """
        dry = bool(body.get("dry_run"))
        budget = max(1, int(self.config.get("recovery_max_attempts", 3)))
        retries = max(0, int(self.config.get("move_retries_after_recovery", 1)))
        used = 0
        if body.get("recover_first") and not dry:
            used += 1
            self.recover_to_home(reason="recover_first (explicit)")
        while True:
            try:
                result = self.move(body)
            except StuckError as exc:
                if dry or used >= budget or retries <= 0:
                    raise
                used += 1
                retries -= 1
                self.recover_to_home(reason=f"move failed: {exc}")
                continue
            if dry:
                return result
            state = self.bridge.state()
            measured = [float(state["joints"][name]) for name in JOINTS]
            problem = self._keep_in_problem(measured)
            if problem is None:
                return result
            if used >= budget or retries <= 0:
                raise StuckError(f"move ended {problem}; recovery budget spent")
            used += 1
            retries -= 1
            self.recover_to_home(reason=f"move ended {problem}")

    def _log_recovery(self, record: dict) -> None:
        """Append one recovery event to the receipt path (never raises)."""
        path = Path(self.config.get("recovery_log", "live/recovery_log.jsonl"))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record) + "\n")
        except OSError as exc:
            print(f"recovery log unwritable: {exc}", file=sys.stderr)

    # ---------------------------------------------------------------- stream
    def _orient(self, image: np.ndarray) -> np.ndarray:
        rotate = int(self.config.get("rotate_deg", 0)) % 360
        if rotate == 90:
            return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        if rotate == 180:
            return cv2.rotate(image, cv2.ROTATE_180)
        if rotate == 270:
            return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return image

    def _fps_locked(self) -> float | None:
        """Frame rate from the served-frame timestamps; caller holds the lock."""
        times = list(self.frame_times)
        if len(times) < 2 or times[-1] <= times[0]:
            return None
        return round((len(times) - 1) / (times[-1] - times[0]), 2)

    def stream_fps(self) -> float | None:
        with self.lock:
            return self._fps_locked()

    def annotate(self) -> bytes | None:
        """One served frame: capture, undistort, draw the overlay, encode.

        The marker pass that used to run here on every frame is now part of the
        overlay's detection, which runs at ``target_refresh_s`` cadence on its
        own thread; this method only draws the last result.  That keeps the
        served frame rate set by the camera, not by the detector.
        """
        started = time.perf_counter()
        try:
            frame = self.frames.capture()
        except BridgeError as exc:
            with self.lock:
                self.last_error = str(exc)
            return None
        raw = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
        snap = {"result": None}
        if self.overlay is not None:
            tool_now, tip_board, flange = self.tool_and_tip()
            self.overlay.submit(raw.copy(), time.time(), tool_now)
            self.overlay.set_aim(self.aim_tuple())
            # Draw on the undistorted frame: the detector solves pose there, so
            # corners and centroid only line up on the same geometry.
            image = self._orient(self.overlay.rectify(raw))
            snap = self.overlay.snapshot()
            snap = self._propagate_snapshot(snap, tool_now)
            # Board plane for the aim ray: one marker solve per fresh pose, so
            # the aim does not use the configured board placement at all.
            self.feed_aim_plane(snap)
            aim_point = (None if flange is None
                         else self.aim_readout(flange))
            # The tip pixel is a MOUNT CONSTANT, not a projection: in sim the
            # camera and the fingertip plane sit on one rigid body, so the pixel
            # is identical across a 60 deg pitch sweep and a 90 deg roll sweep
            # (0.0 px, 2026-11-23). Projecting the FK tip instead inherits the
            # 6.6 deg FK/pose error and moves the marker where the hand-eye is
            # wrong, which is exactly what we do not want.
            tip_px = self.tip_center().get("pixel")
            draw_target(image, snap, layout_hw=raw.shape[:2],
                        stream_fps=self.stream_fps(),
                        tip_center=tip_px, aim_point=aim_point)
        else:
            image = self._orient(raw)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with self.lock:
            self.frame_times.append(time.time())
            self.frame_cost_ms.append((time.perf_counter() - started) * 1000.0)
        return encoded.tobytes() if ok else None

    def stream_loop(self) -> None:
        while True:
            with self.lock:
                if self.clients <= 0:
                    self.stream_thread = None
                    return
            frame = self.annotate()
            if frame is not None:
                with self.lock:
                    self.annotated = frame
                    self.annotated_ts = time.time()
            time.sleep(0.12)

    def attach(self) -> None:
        with self.lock:
            self.clients += 1
            if self.stream_thread is None:
                self.stream_thread = threading.Thread(target=self.stream_loop,
                                                      name="stream", daemon=True)
                self.stream_thread.start()

    def detach(self) -> None:
        with self.lock:
            self.clients = max(0, self.clients - 1)


live = Live()
app = FastAPI(title="Robot arm live", version="0.1.0", docs_url=None, redoc_url=None)


def _authorised(request: Request) -> None:
    """Token check, or same-origin-only when the token requirement is off.

    Turning the token off removes the shared secret, so the remaining guard is
    that a POST must not come from another origin.  That stops a stray page in a
    browser from jogging the arm; it does not stop a LAN host that speaks HTTP
    directly.  Acceptable on a bench LAN, not on a shared network.
    """
    if not live.config.get("auth_required", True):
        origin = request.headers.get("origin")
        if origin is not None and origin.rstrip("/") != str(request.base_url).rstrip("/"):
            raise HTTPException(403, "cross-origin control blocked")
        return
    token = Path(live.config["token_file"]).read_text().strip()
    if request.headers.get("x-bridge-token") != token:
        raise HTTPException(403, "bad token")


def _target_summary() -> dict | None:
    if live.overlay is None:
        return None
    snap = live.overlay.snapshot()
    result = snap.get("result") or {}
    return {"status": result.get("status", "starting"),
            "reason": result.get("reason", ""),
            "piece": result.get("piece"),
            "board": result.get("board"),
            "age_s": None if snap["age_s"] is None else round(snap["age_s"], 2),
            "runs": snap["runs"],
            "mean_cost_ms": snap["mean_cost_ms"],
            "errors": snap["errors"],
            "last_error": snap["last_error"]}


def _aim_summary(bridge_state: dict) -> dict | None:
    """The aim ray for ``/state``, from the joints in the state we already have.

    Kept out of ``live.lock`` for the same lock-ordering reason as the target
    summary: it reads the overlay snapshot.  A missing bridge state is not
    fatal — the aim then reports why it cannot be computed.
    """
    try:
        joints = bridge_state.get("joints") or {}
        if len(joints) != len(JOINTS):
            return {"ok": False, "reason": "no joint state for the aim ray",
                    "schema": "aim/1"}
        values = [float(joints[name]) for name in JOINTS]
        return live.aim_readout(joints_deg=values)
    except Exception as exc:            # display state must never 500 the console
        return {"ok": False, "reason": f"aim unavailable: {type(exc).__name__}: {exc}",
                "schema": "aim/1"}


@app.get("/state")
def state():
    try:
        bridge_state = live.bridge.state()
    except BridgeError as exc:
        bridge_state = {"error": str(exc)}
    # Taken before ``live.lock``: the stream thread holds the overlay lock while
    # it asks for the frame rate, so the opposite order here would deadlock.
    target = _target_summary()
    aim_state = _aim_summary(bridge_state)
    with live.lock:
        return {"bridge": bridge_state,
                "wrist": live.wrist_state(bridge_state),
                "camera_yaw": live.camera_yaw_state(),
                "tool_tip": describe_tip_frame(),
                "aim_point": aim_state,
                "scan": live.last_scan,
                "scan_running": live.scan_running,
                "board_scan": live.last_board,
                "board_scan_dir": live.last_board_dir,
                "board_scan_running": live.board_scan_running,
                "board_scan_error": live.board_scan_error,
                "error": live.last_error,
                "stream_age_s": round(time.time() - live.annotated_ts, 2)
                if live.annotated_ts else None,
                "stream_fps": live._fps_locked(),
                "frame_cost_ms": round(float(np.mean(live.frame_cost_ms)), 1)
                if live.frame_cost_ms else None,
                "target": target}


@app.post("/scan")
async def scan(request: Request):
    _authorised(request)
    return await run_in_threadpool(live.start_scan)


@app.post("/board/scan")
async def board_scan(request: Request):
    """Wrist perception scan: 64 square labels from the plan captures."""
    _authorised(request)
    return await run_in_threadpool(live.start_board_scan)


@app.get("/camera/controls")
def camera_controls():
    """Proxy the bridge's control list; ranges come from the device."""
    try:
        return live.bridge.camera_controls()
    except BridgeError as exc:
        raise HTTPException(502, str(exc)) from exc


@app.post("/camera/controls")
async def camera_controls_set(request: Request):
    _authorised(request)
    body = await request.json()
    controls = body.get("controls")
    if not isinstance(controls, dict) or not controls:
        raise HTTPException(400, "controls must be a non-empty object")
    try:
        return live.bridge.set_camera_controls(controls,
                                               save=bool(body.get("save")))
    except BridgeError as exc:
        raise HTTPException(502, str(exc)) from exc


@app.post("/arm")
async def arm(request: Request):
    _authorised(request)
    body = await request.json()
    if body.get("confirm") is not True:
        raise HTTPException(400, "confirm must be true")
    try:
        return live.bridge.arm()
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/mode")
async def mode(request: Request):
    _authorised(request)
    body = await request.json()
    target = str(body.get("mode", "")).upper()
    if target not in ("HOLD", "TELEOP", "REMOTE"):
        raise HTTPException(400, "mode must be HOLD, TELEOP or REMOTE")
    try:
        return live.bridge.set_mode(target)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/estop")
async def estop(request: Request):
    _authorised(request)
    try:
        return live.bridge.estop()
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.get("/focus-score")
def focus_score():
    try:
        return live.bridge._request("/focus")
    except BridgeError as exc:
        raise HTTPException(502, str(exc)) from exc


@app.get("/metrics")
def metrics():
    """Focus score, marker sizes and a rough lens-to-surface distance."""
    try:
        frame = live.frames.capture()
    except BridgeError as exc:
        raise HTTPException(502, str(exc)) from exc
    oriented = live._orient(frame.image)
    gray = cv2.cvtColor(oriented, cv2.COLOR_RGB2GRAY)
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    detections = fiducials.detect_markers(oriented)
    markers = []
    focal = float(live.config.get("assumed_focal_px", 690.0))
    for detection in detections:
        edges = [float(np.linalg.norm(detection.corners[i] - detection.corners[(i + 1) % 4]))
                 for i in range(4)]
        edge = float(np.mean(edges))
        markers.append({"id": detection.marker_id, "edge_px": round(edge, 1),
                        "distance_mm": round(focal * 40.0 / edge, 1)})
    distance = (sum(m["distance_mm"] for m in markers) / len(markers)) if markers else None
    return {"score": round(float(lap.var()), 1),
            "markers": markers,
            "distance_mm": None if distance is None else round(distance, 1),
            "frame_age_s": round(time.time() - frame.timestamp, 3)}


@app.post("/capture-view")
async def capture_view(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return await run_in_threadpool(live.capture_view,
                                       str(body.get("label", ""))[:24])
    except BridgeError as exc:
        raise HTTPException(502, str(exc)) from exc


@app.get("/progress")
def progress():
    return live.progress()


@app.get("/captures")
def captures():
    return live.captures()


@app.get("/poses")
def poses():
    return live.poses()


@app.post("/pose/save")
async def pose_save(request: Request):
    _authorised(request)
    saved = live.save_pose()
    try:
        name = live.next_pose_name()
        shot = live.capture_view(name)
        if shot.get("moving"):
            return {"saved": None, "reason": "moving",
                    "moved_px": shot.get("moved_px"),
                    "moved_deg": shot.get("moved_deg"),
                    "progress": live.progress()}
        if not shot.get("markers"):
            for suffix in ("json", "jpg"):
                try:
                    (live.capture_dir / f"{shot['name']}.{suffix}").unlink()
                except OSError:
                    pass
            return {"saved": None, "reason": "no_tag", "progress": live.progress()}
        if shot.get("duplicate_of"):
            for suffix in ("json", "jpg"):
                try:
                    (live.capture_dir / f"{shot['name']}.{suffix}").unlink()
                except OSError:
                    pass
            return {"saved": None, "duplicate_of": shot["duplicate_of"],
                    "markers": shot.get("markers", []), "progress": live.progress()}
        saved = live.save_pose(name)
        return {"saved": saved["name"], "joints": saved["joints"],
                "capture": shot, "progress": live.progress()}
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/pose/run")
async def pose_run(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return await run_in_threadpool(live.run_pose, str(body.get("name", "")))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


# Every endpoint that can spend real time (a cold solve, a streamed move, a
# scan cycle) dispatches to a worker thread.  This one used to be a bare
# ``async def`` around a blocking ``live.move``: while a cold h1 solve ran
# (~169 s on least_squares, 2026-11-24) the whole event loop was parked inside
# it, so even ``GET /state`` hung and the console looked dead.  The motion lock
# inside ``Live`` is still the single-writer guard; this only stops one slow
# request from freezing every other route.  FastAPI already runs the plain
# ``def`` routes in a threadpool, so this makes the two styles behave alike.
@app.post("/move")
async def move(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        # Recovery is part of the move path, not a separate operator step: the
        # owner's rule is that a stuck robot resets itself and starts again.
        return await run_in_threadpool(live.move_with_recovery, body)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc
    except (TypeError, ValueError) as exc:
        raise HTTPException(400, str(exc)) from exc


@app.post("/recover")
async def recover(request: Request):
    """Drive to the stored home pose above the board centre.

    No IK, no gripper, one or two joint-space legs.  Also the automatic path a
    stuck move takes; useful on its own after a jam or a refused move.
    """
    _authorised(request)
    try:
        body = await request.json()
    except Exception:                                # noqa: BLE001
        body = {}
    reason = str((body or {}).get("reason") or "manual (/recover)")
    try:
        return await run_in_threadpool(live.recover_to_home, reason)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/sanity")
async def sanity(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return await run_in_threadpool(
            partial(live.sanity_check, execute=bool(body.get("execute", True)),
                    z_mm=float(body.get("z_mm", 120.0)),
                    tolerance_mm=float(body.get("tolerance_mm", 6.0))))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/home")
async def home(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return await run_in_threadpool(
            partial(live.home, execute=bool(body.get("execute", True))))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.get("/keep-in")
def keep_in():
    zone = live.keep_in()
    try:
        pose = live.pose_now()
        # The motion guard evaluates the JAW TIP, not the chain frame: compare
        # the same point here or the endpoint says "inside" while every move is
        # refused (block 5c, tip y 118.6 mm against a +/-118 mm box).
        tip = pose["tip_mm"]
        zone["inside"] = (abs(tip[0]) <= zone["x_mm"]
                          and abs(tip[1]) <= zone["y_mm"]
                          and zone["z_min_mm"] <= tip[2] <= zone["z_max_mm"])
        zone["tip_mm"] = tip
        zone["tool_mm"] = pose["tool_mm"]
        zone["checked"] = "jaw_tip"
    except BridgeError:
        zone["inside"] = None
    return zone


@app.get("/board")
def board():
    return live.board()


@app.post("/board")
async def board_set(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return live.set_board(body)
    except (TypeError, ValueError) as exc:
        raise HTTPException(400, str(exc)) from exc


@app.get("/cart-limits")
def cart_limits(step_mm: float = 4.0):
    try:
        return live.cart_limits(step_mm=float(step_mm))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/board/assume")
async def board_assume(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return live.assume_board(body)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/board/derive")
async def board_derive(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return await run_in_threadpool(
            partial(live.derive_board, save=bool(body.get("save", True))))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.get("/pose")
def pose():
    try:
        return live.pose_now()
    except BridgeError as exc:
        raise HTTPException(502, str(exc)) from exc


@app.post("/jog")
async def jog(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return await run_in_threadpool(live.jog, str(body.get("joint", "")),
                                       float(body.get("delta", 0.0)))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc


@app.get("/wrist")
def wrist():
    try:
        return live.wrist_state()
    except BridgeError as exc:
        return JSONResponse({**live.wrist_cal(), "error": str(exc)})


@app.post("/wrist/zero")
async def wrist_zero(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        zero = body.get("zero_deg")
        return await run_in_threadpool(
            live.set_wrist_zero, None if zero is None else float(zero))
    except (BridgeError, ValueError) as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/wrist/pan-zero")
async def wrist_pan_zero(request: Request):
    _authorised(request)
    try:
        pan = float(live.wrist_cal().get("pan_zero_deg", 0.0))
        return await run_in_threadpool(
            live.move, {"mode": "joint", "joints": {"shoulder_pan": pan},
                        "settle_deg": 0.1})
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/wrist/auto-align")
async def wrist_auto_align(request: Request):
    _authorised(request)
    try:
        return await run_in_threadpool(live.auto_align_wrist)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.get("/wrist/yaw")
def wrist_yaw():
    """Current offset plus the last measured camera yaw error, if any."""
    return live.camera_yaw_state()


@app.post("/wrist/yaw")
async def wrist_yaw_set(request: Request):
    _authorised(request)
    body = await request.json()

    def _optional(name: str):
        value = body.get(name)
        return None if value is None else float(value)

    try:
        offset = _optional("offset_deg")
        delta = _optional("delta_deg")
        if offset is None and delta is None:
            raise ValueError("offset_deg or delta_deg is required")
        return await run_in_threadpool(
            live.set_camera_yaw, offset, delta,
            str(body.get("source") or "manual"))
    except (BridgeError, ValueError) as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/wrist/yaw/zero")
async def wrist_yaw_zero(request: Request):
    """Accept the measured yaw as the new zero (measures once if unknown)."""
    _authorised(request)
    try:
        return await run_in_threadpool(live.accept_camera_yaw)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/wrist/yaw/auto")
async def wrist_yaw_auto(request: Request):
    """Measure the board with a fresh capture, then null the offset."""
    _authorised(request)
    try:
        return await run_in_threadpool(live.auto_measure_camera_yaw)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


def _mjpeg():
    live.attach()
    try:
        last = None
        while True:
            with live.lock:
                frame = live.annotated
                age = time.time() - live.annotated_ts if live.annotated_ts else 99
            if frame is not None and frame is not last and age < 1.0:
                last = frame
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + frame + b"\r\n")
            time.sleep(0.05)
    finally:
        live.detach()


@app.get("/tip-center")
def tip_center_get():
    return live.tip_center()


@app.get("/aim")
def aim_get():
    return live.aim()


@app.get("/aim-point")
def aim_point_get():
    """The aim ray in one call, so no client re-derives the geometry.

    Returns ``AIM_WORLD`` (arm-base mm), ``AIM_PIXEL``, the constant
    ``TIP_PIXEL``, the aim-minus-tip offset, the board-plane intersection in
    board mm, and ``ok`` with ``reason`` when there is no usable plane.
    Read-only: no motion state is touched.
    """
    return live.aim_readout()


@app.post("/aim")
async def aim_set(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return live.set_aim(body["x_mm"], body["y_mm"], body.get("z_mm", 0.0),
                            body.get("label", "aim"))
    except (KeyError, TypeError, ValueError) as exc:
        raise HTTPException(400, str(exc)) from exc


@app.post("/aim/clear")
async def aim_clear(request: Request):
    _authorised(request)
    return live.clear_aim()


@app.post("/tip-center")
async def tip_center_set(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        u = float(body["u_mm"]) if "u_mm" in body else float(body["u"])
        v = float(body["v_mm"]) if "v_mm" in body else float(body["v"])
        return live.set_tip_center(u, v)
    except (KeyError, ValueError, TypeError) as exc:
        raise HTTPException(400, str(exc)) from exc


@app.post("/tip-center/clear")
async def tip_center_clear(request: Request):
    _authorised(request)
    try:
        if live.tip_center_path.exists():
            live.tip_center_path.unlink()
    except OSError:
        pass
    return live.tip_center()


@app.get("/stream")
def stream():
    return StreamingResponse(_mjpeg(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot arm · live</title>
<style>
:root { color-scheme: dark; }
body { margin:0; font:14px/1.45 ui-sans-serif,system-ui,sans-serif; background:#0b0f14; color:#d8e1ea; }
header { padding:10px 14px; border-bottom:1px solid #1d2733; display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
h1 { font-size:14px; margin:0; letter-spacing:.05em; text-transform:uppercase; color:#8fb3d9; }
main { display:grid; grid-template-columns:minmax(300px,430px) 400px; gap:14px;
       padding:14px; justify-content:center; }
@media (max-width:980px){ main { grid-template-columns:1fr; } }
.cam { background:#000; border:1px solid #1d2733; border-radius:6px; overflow:hidden;
       display:flex; justify-content:center; align-items:center; }
.camframe { position:relative; max-width:100%; line-height:0; }
.cam img { display:block; width:auto; height:auto; max-width:100%;
           max-height:clamp(220px, calc(100vh - 360px), 520px); }
.panel { background:#111823; border:1px solid #1d2733; border-radius:6px; padding:10px; margin-bottom:10px; }
.panel h2 { font-size:10px; letter-spacing:.09em; text-transform:uppercase; color:#6f8aa6; margin:0 0 8px; }
.row { display:flex; justify-content:space-between; padding:1px 0; }
.mono { font-family:ui-monospace,monospace; }
button { background:#1a2635; color:#d8e1ea; border:1px solid #2b3b4f; padding:6px 9px;
         border-radius:4px; cursor:pointer; font:inherit; }
button:hover { background:#223247; }
button.danger { border-color:#7a2a2a; color:#ffb4b4; }
button.go { border-color:#2c5f41; color:#9fe3b8; }
.camcheck { display:flex; gap:6px; align-items:center; text-transform:none;
            letter-spacing:0; font-size:12px; color:#d8e1ea; }
.camcheck input { width:auto; }
.camrow { margin:0 0 9px; }
.camrow.off { opacity:.45; }
.camrow .head { display:flex; justify-content:space-between; gap:6px; }
.camrow input[type=range] { padding:0; height:18px; }
.camrow input[type=number] { width:74px; }
.camrow select { width:100%; padding:4px 6px; background:#0b0f14; color:#d8e1ea;
                 border:1px solid #2b3b4f; border-radius:4px; font:inherit; }
button.tab { padding:4px 10px; font-size:12px; }
button.tab.on { background:#223247; border-color:#3f6288; color:#bcd8f6; }
.grid { display:grid; grid-template-columns:repeat(3,1fr); gap:6px; }
.grid2 { display:grid; grid-template-columns:repeat(2,1fr); gap:6px; }
.badge { padding:2px 8px; border-radius:999px; border:1px solid #2b3b4f; font-size:12px; }
.ok { color:#9fe3b8; border-color:#2c5f41; } .bad { color:#ffb4b4; border-color:#7a2a2a; }
input { width:100%; padding:5px 6px; background:#0b0f14; color:#d8e1ea; border:1px solid #2b3b4f;
        border-radius:4px; font:12px ui-monospace,monospace; }
label { display:block; font-size:10px; color:#7f97ae; text-transform:uppercase; letter-spacing:.06em; }
.big { font:600 36px/1 ui-monospace,monospace; color:#9fe3b8; }
.bar { height:10px; background:#1a2635; border-radius:5px; overflow:hidden; margin:6px 0; }
.bar i { display:block; height:100%; width:0; background:linear-gradient(90deg,#7a2a2a,#c9a227,#2c5f41); }
#log { max-height:120px; overflow:auto; font-size:12px; color:#8fa6bd; }
.hint { font-size:11px; color:#7f97ae; }
.cross { position:absolute; inset:0; pointer-events:none; }
.cross svg { width:100%; height:100%; display:block; }
.cross line { stroke:#7fd4a0; stroke-width:1; vector-effect:non-scaling-stroke; opacity:.9; }
.cross line.minor { opacity:.32; }
.cross circle { fill:none; stroke:#7fd4a0; stroke-width:1; vector-effect:non-scaling-stroke; opacity:.9; }
.legend { display:flex; gap:14px; flex-wrap:wrap; font-size:11px; opacity:.85; margin:6px 0 0 2px; }
.legend span { display:flex; align-items:center; gap:5px; }
.legend .sw { width:10px; height:10px; border-radius:2px; display:inline-block; }
.legend .sw.aim { border:2px solid #ffd23f; background:transparent; }
.legend .sw.tip { background:#35e0e8; }
.legend .sw.cmd { background:#ff4fd8; }
.tipmark { position:absolute; width:22px; height:22px; margin:-11px 0 0 -11px; pointer-events:none; display:none; }
.tipmark::before, .tipmark::after { content:""; position:absolute; background:#ffd23f; }
.tipmark::before { left:10px; top:0; width:2px; height:22px; }
.tipmark::after { top:10px; left:0; height:2px; width:22px; }
.camframe.tipset { cursor:crosshair; }
</style></head><body>
<header><h1>Robot arm</h1>
<span id="mode" class="badge">—</span><span id="link" class="badge">—</span>
<span id="markers" class="badge">no marker</span><span id="dist" class="badge">—</span>
<span id="err" class="badge bad" style="display:none"></span>
<span id="keepin" class="badge">zone —</span>
<nav style="margin-left:auto;display:flex;gap:6px">
  <button class="tab on" id="vtab-live">Live</button>
  <button class="tab" id="vtab-wrist">Wrist calibration</button>
  <button class="tab" id="vtab-camera">Camera</button>
  <button class="tab" id="vtab-tip">Tip set</button>
</nav></header>
<main>
  <div>
    <div class="cam"><div class="camframe" id="camframe"><img id="cam" src="/stream" alt="wrist camera">
      <div id="tipmark" class="tipmark" title="set tip centre"></div>
      <div id="cross" class="cross" style="display:none">
        <svg viewBox="0 0 100 100" preserveAspectRatio="none" aria-label="alignment crosshatch">
          <line x1="50" y1="0" x2="50" y2="100"></line>
          <line x1="0" y1="50" x2="100" y2="50"></line>
          <line class="minor" x1="10" y1="48.4" x2="10" y2="51.6"></line>
          <line class="minor" x1="20" y1="48.4" x2="20" y2="51.6"></line>
          <line class="minor" x1="30" y1="48.4" x2="30" y2="51.6"></line>
          <line class="minor" x1="40" y1="48.4" x2="40" y2="51.6"></line>
          <line class="minor" x1="60" y1="48.4" x2="60" y2="51.6"></line>
          <line class="minor" x1="70" y1="48.4" x2="70" y2="51.6"></line>
          <line class="minor" x1="80" y1="48.4" x2="80" y2="51.6"></line>
          <line class="minor" x1="90" y1="48.4" x2="90" y2="51.6"></line>
          <line class="minor" x1="48.4" y1="10" x2="51.6" y2="10"></line>
          <line class="minor" x1="48.4" y1="20" x2="51.6" y2="20"></line>
          <line class="minor" x1="48.4" y1="30" x2="51.6" y2="30"></line>
          <line class="minor" x1="48.4" y1="40" x2="51.6" y2="40"></line>
          <line class="minor" x1="48.4" y1="60" x2="51.6" y2="60"></line>
          <line class="minor" x1="48.4" y1="70" x2="51.6" y2="70"></line>
          <line class="minor" x1="48.4" y1="80" x2="51.6" y2="80"></line>
          <line class="minor" x1="48.4" y1="90" x2="51.6" y2="90"></line>
          <circle cx="50" cy="50" r="2.4"></circle>
        </svg>
      </div></div>
    </div>
    <div class="legend mono">
      <span><i class="sw aim"></i>aim ray (where the tool axis meets the board)</span>
      <span><i class="sw tip"></i>tip pixel (mount constant)</span>
      <span><i class="sw cmd"></i>commanded point</span>
    </div>
    <div class="panel" style="margin-top:10px">
      <div class="row"><span class="hint">focus score</span><span class="mono hint" id="peak">peak —</span></div>
      <div class="big" id="score">—</div>
      <div class="bar"><i id="fill"></i></div>
      <div class="row hint"><span>target</span><span class="mono">≥ 300 good</span></div>
    </div>
    <div class="panel"><h2>Position (board frame, mm / deg)</h2>
      <div id="posetable" class="mono" style="font-size:11px"></div>
    </div>
  </div>
  <div>
   <div id="pane-live">
    <div class="panel"><h2>Session</h2>
      <div class="grid" style="grid-template-columns:repeat(3,1fr)">
        <button id="arm">Arm</button>
        <button id="remote">Remote</button>
        <button id="teleop">Teleop</button>
      </div>
      <div class="grid2" style="margin-top:6px">
        <button id="scan" class="go">Scan board</button>
        <button id="estop" class="danger">E-STOP</button>
      </div>
      <div class="grid2" style="margin-top:6px">
        <button id="home" class="go">Home (middle pose)</button>
        <button id="homedry">Home (check)</button>
      </div>
      <div class="grid2" style="margin-top:6px">
        <button id="sanity" class="go">Sanity: 4 corners + centre</button>
        <button id="sanitydry">Sanity (dry run)</button>
      </div>
      <div id="sanityOut" class="mono" style="margin-top:6px;font-size:11px;color:#8fa6bd"></div>
    </div>

    <div class="panel"><h2>Move</h2>
      <div style="display:flex;gap:6px;margin-bottom:8px">
        <button class="tab on" id="tab-xyz">x / y / z</button>
        <button class="tab" id="tab-look">look at</button>
        <button class="tab" id="tab-joint">joints</button>
      </div>
      <div id="pane-xyz">
        <div class="grid" style="grid-template-columns:repeat(4,1fr);gap:6px">
          <div><label>square</label><input id="sq" placeholder="e4"></div>
          <div><label>x mm</label><input id="mx" value="0"></div>
          <div><label>y mm</label><input id="my" value="0"></div>
          <div><label>z mm</label><input id="mz" value="60"></div>
        </div>
        <div class="grid2" style="margin-top:6px">
          <div><label>span deg</label><input id="mspan" value="0"></div>
          <div><label>jaw</label><input id="mjaw" value="40"></div>
        </div>
        <div class="hint" style="margin-top:6px">Tool point on the board with the jaws pointing straight down.</div>
      </div>
      <div id="pane-look" style="display:none">
        <div class="grid" style="grid-template-columns:repeat(4,1fr);gap:6px">
          <div><label>x mm</label><input id="lx" value="0"></div>
          <div><label>y mm</label><input id="ly" value="0"></div>
          <div><label>z mm</label><input id="lz" value="0"></div>
          <div><label>dist mm</label><input id="ld" value="240"></div>
        </div>
        <div class="grid2" style="margin-top:6px">
          <div><label>pitch deg</label><input id="lp" value="0"></div>
          <div><label>azimuth deg</label><input id="la" value="0"></div>
        </div>
        <div class="hint" style="margin-top:6px">Camera aimed at the point from that standoff. pitch 0 = straight down.</div>
      </div>
      <div id="pane-joint" style="display:none">
        <div class="grid" style="grid-template-columns:repeat(3,1fr);gap:6px">
          <div><label>pan</label><input id="j0"></div>
          <div><label>lift</label><input id="j1"></div>
          <div><label>elbow</label><input id="j2"></div>
          <div><label>flex</label><input id="j3"></div>
          <div><label>roll</label><input id="j4"></div>
          <div><label>jaw</label><input id="j5"></div>
        </div>
        <div class="hint" style="margin-top:6px">Absolute angles; the escape hatch when IK has no solution.</div>
      </div>
      <div class="grid2" style="margin-top:8px">
        <button id="gobtn" class="go">Go</button>
        <button id="fillbtn">Fill from arm</button>
      </div>
      <div class="mono" id="moveStatus" style="margin-top:6px;font-size:12px;color:#9fe3b8"></div>
    </div>

    <div class="panel"><h2>Jog</h2>
      <div style="display:flex;gap:6px;margin-bottom:8px">
        <button class="tab on" id="jtab-joints">joints</button>
        <button class="tab" id="jtab-xyz">x / y / z</button>
        <button class="tab" id="jtab-look">look at</button>
      </div>
      <div id="jpane-joints">
        <div class="grid">
          <button onclick="jog('shoulder_pan',2)">pan +</button>
          <button onclick="jog('shoulder_lift',2)">lift +</button>
          <button onclick="jog('elbow_flex',2)">elbow +</button>
          <button onclick="jog('shoulder_pan',-2)">pan −</button>
          <button onclick="jog('shoulder_lift',-2)">lift −</button>
          <button onclick="jog('elbow_flex',-2)">elbow −</button>
          <button onclick="jog('wrist_flex',2)">flex +</button>
          <button onclick="jog('wrist_roll',5)">roll +</button>
          <button onclick="jog('gripper',5)">jaw +</button>
          <button onclick="jog('wrist_flex',-2)">flex −</button>
          <button onclick="jog('wrist_roll',-5)">roll −</button>
          <button onclick="jog('gripper',-5)">jaw −</button>
        </div>
      </div>
      <div id="jpane-xyz" style="display:none">
        <div class="grid2" style="margin-bottom:6px">
          <div><label>step mm</label><input id="jstep" value="5"></div>
          <div><label>centre</label><button style="width:100%" onclick="jogToCentre()">e4 · 60 mm</button></div>
        </div>
        <div id="xyzrows" class="mono" style="font-size:11px"></div>
        <div class="grid2" style="margin-top:6px">
          <button onclick="jawJog(5)">jaw +</button>
          <button onclick="jawJog(-5)">jaw −</button>
        </div>
        <div class="hint" id="limhint" style="margin-top:6px">ranges measured at the current pose · refresh after big moves</div>
        <button id="limbtn" style="margin-top:6px;width:100%">Refresh reachable range</button>
      </div>
      <div id="jpane-look" style="display:none">
        <div id="lookrows" class="mono" style="font-size:11px"></div>
        <div class="hint" style="margin-top:6px">aims the camera at the board centre from that standoff</div>
      </div>
    </div>

    <div class="panel"><h2>Joints (deg · limits)</h2><div id="joints" class="mono"></div></div>

    <div class="panel"><h2>Board frame</h2>
      <div class="grid" style="grid-template-columns:repeat(4,1fr);gap:6px">
        <div><label>x mm</label><input id="bx"></div>
        <div><label>y mm</label><input id="by"></div>
        <div><label>z mm</label><input id="bz"></div>
        <div><label>yaw deg</label><input id="byaw"></div>
      </div>
      <div class="grid" style="grid-template-columns:repeat(2,1fr);margin-top:6px">
        <button id="boardsave">Save board</button>
        <button id="boardderive" class="go">Derive from tag</button>
      </div>
      <div class="grid2" style="margin-top:6px">
        <button id="boardload">Reload</button>
        <button id="boardfill">Fill from arm pose</button>
      </div>
      <div class="hint" style="margin-top:6px" id="boardStatus">origin = mat centre; +x files, +y ranks</div>
    </div>

    <div class="panel"><h2>Captures</h2>
      <div class="grid" style="grid-template-columns:repeat(3,1fr)">
        <button id="capture">Capture</button>
        <button onclick="savePose()">Save pose</button>
        <button onclick="listCaptures()">List</button>
      </div>
      <div class="mono" id="capStatus" style="margin-top:6px;font-size:12px;color:#9fe3b8"></div>
      <div class="mono" id="caps" style="margin-top:4px;font-size:11px;color:#8fa6bd"></div>
      <div class="mono" id="progress" style="margin-top:4px;font-size:11px;color:#8fa6bd"></div>
    </div>

    <div class="panel"><h2>Poses</h2>
      <div id="poseStatus" style="padding:5px 7px;border-radius:4px;background:#16241f;color:#9fe3b8;font:600 12px ui-monospace,monospace">press Save pose to capture a view</div>
      <div id="poses" style="margin-top:8px;display:flex;flex-wrap:wrap;gap:6px"></div>
    </div>

    <div class="panel"><h2>Log</h2><div id="log" class="mono"></div></div>
   </div>
   <div id="pane-wrist" style="display:none">
    <div class="panel"><h2>Wrist calibration</h2>
      <div class="row"><span class="hint">wrist roll (0 = calibrated)</span><span class="mono" id="wcRoll">—</span></div>
      <div class="row"><span class="hint">servo angle</span><span class="mono" id="wcRel">—</span></div>
      <div class="bar"><i id="wcBar"></i></div>
      <div class="row hint"><span id="wcMin">min —</span><span id="wcMax">max —</span></div>
      <div id="wcStatus" class="mono hint" style="margin-top:6px">—</div>
    </div>
    <div class="panel"><h2>Alignment overlay</h2>
      <div class="grid2">
        <button id="wcCross">Crosshair on</button>
        <button id="wcPan" class="go">Zero pan (→ 0°)</button>
      </div>
      <button id="wcAuto" class="go" style="width:100%;margin-top:6px">Auto-align horizontally (ArUco)</button>
      <div class="hint" style="margin-top:6px">The crosshatch marks the camera centre and square axes. Line the tool up with them, then zero the pan.</div>
    </div>
    <div class="panel"><h2>Roll</h2>
      <div class="hint">coarse · 10°</div>
      <div class="grid2" style="margin:4px 0 8px">
        <button onclick="rollJog(-10)">coarse −10°</button>
        <button onclick="rollJog(10)">coarse +10°</button>
      </div>
      <div class="hint">fine · 1°</div>
      <div class="grid2" style="margin-top:4px">
        <button onclick="rollJog(-1)">fine −1°</button>
        <button onclick="rollJog(1)">fine +1°</button>
      </div>
    </div>
    <div class="panel"><h2>Zero</h2>
      <button id="wcAccept" class="go" style="width:100%">Accept current position as wrist 0</button>
      <div class="hint" style="margin-top:6px">Sets the wrist window to ±90° around the current roll and saves it to live/wrist_calibration.json. The window is enforced on roll jogs and joint moves.</div>
      <div id="wcZeroInfo" class="mono hint" style="margin-top:4px">—</div>
    </div>

    <div class="panel"><h2>Camera yaw</h2>
      <div class="row"><span class="hint">yaw offset (0 = calibrated)</span><span class="mono" id="wcYaw">—</span></div>
      <div class="row"><span class="hint">measured camera yaw error</span><span class="mono" id="wcYawMeasured">—</span></div>
      <div class="hint">coarse · 10°</div>
      <div class="grid2" style="margin:4px 0 8px">
        <button onclick="yawJog(-10)">coarse −10°</button>
        <button onclick="yawJog(10)">coarse +10°</button>
      </div>
      <div class="hint">fine · 1°</div>
      <div class="grid2" style="margin-top:4px">
        <button onclick="yawJog(-1)">fine −1°</button>
        <button onclick="yawJog(1)">fine +1°</button>
      </div>
      <div class="grid2" style="margin-top:8px">
        <button id="wcYawAccept" class="go">Accept current as yaw 0</button>
        <button id="wcYawAuto" class="go">Auto-measure (ArUco)</button>
      </div>
      <button id="wcYawReset" style="width:100%;margin-top:6px">Reset to 0°</button>
      <div id="wcYawStatus" class="mono hint" style="margin-top:6px">—</div>
      <div class="hint" style="margin-top:6px">A camera-local rotation about the optical axis, stacked on the saved calibration. It never rewrites the hand-eye file: the offset lives in <span class="mono">live/camera_yaw.json</span> and is applied to every wrist camera built from measured joints.</div>
      <div id="wcYawPath" class="mono hint" style="margin-top:4px">—</div>
    </div>
   </div>
   <div id="pane-camera" style="display:none">
    <div class="panel"><h2>Auto</h2>
      <div class="grid2">
        <label class="camcheck"><input type="checkbox" id="camAutoExp"> Auto exposure</label>
        <label class="camcheck"><input type="checkbox" id="camAutoWb"> Auto white balance</label>
      </div>
      <div class="hint" style="margin-top:6px">Off pins exposure and white balance, so consecutive frames agree. On, the camera hunts and a dark piece on a dark square stops separating.</div>
    </div>
    <div class="panel"><h2>Camera controls</h2>
      <div class="row hint"><span class="mono" id="camDevice">—</span><span class="mono" id="camSavedAt">—</span></div>
      <div id="camControls" style="margin-top:8px"></div>
      <div class="grid2" style="margin-top:10px">
        <button id="camApply" class="go">Apply</button>
        <button id="camReset">Reset to defaults</button>
      </div>
      <div class="grid2" style="margin-top:6px">
        <button id="camSave" class="go">Save as startup</button>
        <button id="camReload">Reload from device</button>
      </div>
      <div id="camStatus" class="mono hint" style="margin-top:8px">—</div>
      <div class="hint" style="margin-top:6px">A change applies as soon as you make it — no Apply needed; Apply just forces one now. Every write is read back from the device. Save as startup is the only thing that stores the profile the bridge re-applies whenever it reopens the camera.</div>
    </div>
   </div>

   <div id="pane-tip" style="display:none">
    <div class="panel"><h2>Set jaw-tip centre</h2>
      <div class="hint">Jog the arm until both finger tips are clearly visible, then click the camera image at the midpoint between the tips. The jaws are rigid to the camera, so that pixel is fixed for every frame and becomes the true tool-centre marker (no hand-eye needed).</div>
      <div class="grid" style="margin-top:8px">
        <button onclick="jog('shoulder_pan',2)">pan +</button>
        <button onclick="jog('shoulder_lift',2)">lift +</button>
        <button onclick="jog('elbow_flex',2)">elbow +</button>
        <button onclick="jog('wrist_flex',2)">flex +</button>
        <button onclick="jog('wrist_roll',5)">roll +</button>
        <button onclick="jog('gripper',5)">jaw +</button>
        <button onclick="jog('shoulder_pan',-2)">pan −</button>
        <button onclick="jog('shoulder_lift',-2)">lift −</button>
        <button onclick="jog('elbow_flex',-2)">elbow −</button>
        <button onclick="jog('wrist_flex',-2)">flex −</button>
        <button onclick="jog('wrist_roll',-5)">roll −</button>
        <button onclick="jog('gripper',-5)">jaw −</button>
      </div>
      <div class="row mono" style="margin-top:8px"><span>stored pixel (raw 640×480)</span><span id="tipPixel">—</span></div>
      <div class="row mono hint"><span>updated</span><span id="tipUpdated">—</span></div>
      <div class="grid2" style="margin-top:8px">
        <button id="tipClear" class="danger">Clear</button>
        <button id="tipReload">Reload</button>
      </div>
      <div id="tipStatus" class="mono hint" style="margin-top:8px">click the image to set</div>
    </div>
    <div class="panel"><h2>How it is used</h2>
      <div class="hint">The saved pixel is the reference for the cyan jaw-centre cross drawn by the live detection overlay. It replaces the old FK-tip-through-PnP projection, which was invalid because the camera-to-jaw transform was never calibrated. It does not command motion and does not touch the arm.</div>
    </div>
   </div>
  </div>
</main>
<script>
const el = id => document.getElementById(id);
const JOINTS = ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'];
const SHORT = {shoulder_pan:'pan',shoulder_lift:'lift',elbow_flex:'elbow',wrist_flex:'flex',wrist_roll:'roll',gripper:'jaw'};
let peak = 0, moveMode = 'xyz';
function log(m){ el('log').textContent = new Date().toLocaleTimeString()+'  '+m+'\\n'+el('log').textContent; }
async function post(path, body){
  const r = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body||{})});
  const text = await r.clone().text();
  if (!r.ok) log(path+' -> '+r.status+' '+text.slice(0,160));
  return r;
}
async function jog(j,d){ await post('/jog',{joint:j,delta:d}); }
function tabs(mode){
  moveMode = mode;
  for (const m of ['xyz','look','joint']){
    el('tab-'+m).classList.toggle('on', m===mode);
    el('pane-'+m).style.display = (m===mode) ? 'block' : 'none';
  }
}
el('tab-xyz').onclick = () => tabs('xyz');
el('tab-look').onclick = () => tabs('look');
el('tab-joint').onclick = () => tabs('joint');
const JOGTABS = ['joints','xyz','look'];
function jogTab(mode){
  for (const m of JOGTABS){
    el('jtab-'+m).classList.toggle('on', m === mode);
    el('jpane-'+m).style.display = (m === mode) ? 'block' : 'none';
  }
  if (mode === 'xyz') refreshLimits();
}
el('jtab-joints').onclick = () => jogTab('joints');
el('jtab-xyz').onclick = () => jogTab('xyz');
el('jtab-look').onclick = () => jogTab('look');

// ---------------------------------------------------------- wrist calibration
// Off by default.  This is a fixed screen-centre hairline, NOT the aim: it
// sits at 50/50 of the frame whatever the arm is doing, so leaving it on
// put a green cross near the baked cyan tip cross and the yellow aim
// reticle and made three unrelated things look like one marker
// (Brian, 2026-11-24).
let crossOn = false;
function showCross(on){
  const wristVisible = el('pane-wrist').style.display !== 'none';
  el('cross').style.display = (on && wristVisible) ? 'block' : 'none';
  el('wcCross').textContent = 'Crosshair ' + (on ? 'on' : 'off');
}
function view(mode){
  el('vtab-live').classList.toggle('on', mode === 'live');
  el('vtab-wrist').classList.toggle('on', mode === 'wrist');
  el('vtab-camera').classList.toggle('on', mode === 'camera');
  el('vtab-tip').classList.toggle('on', mode === 'tip');
  el('pane-live').style.display = mode === 'live' ? 'block' : 'none';
  el('pane-wrist').style.display = mode === 'wrist' ? 'block' : 'none';
  el('pane-camera').style.display = mode === 'camera' ? 'block' : 'none';
  el('pane-tip').style.display = mode === 'tip' ? 'block' : 'none';
  el('camframe').classList.toggle('tipset', mode === 'tip');
  if (mode === 'camera' && !camState) camLoad();
  if (mode === 'tip') tipLoad();
  showCross(crossOn);
}
el('vtab-live').onclick = () => view('live');
el('vtab-wrist').onclick = () => view('wrist');
el('vtab-camera').onclick = () => view('camera');
el('vtab-tip').onclick = () => view('tip');

// ---------------------------------------------------------- tip centre (manual)
let tipPixel = null;
function tipRender(d){
  tipPixel = d.set ? d.pixel : null;
  el('tipPixel').textContent = d.set ? d.pixel.join(', ') : '—';
  el('tipUpdated').textContent = d.updated || '—';
  const mark = el('tipmark');
  // Only in the Tip-set pane.  On the live view the calibrated tip is already
  // the baked cyan cross, and a second yellow cross 12 px away read as a
  // different marker (Brian, 2026-11-24).
  const tipPaneOpen = el('pane-tip').style.display !== 'none';
  if (d.set && tipPaneOpen) {
    mark.style.display = 'block';
    mark.style.left = (d.pixel[0]/640*100) + '%';
    mark.style.top  = (d.pixel[1]/480*100) + '%';
  } else {
    mark.style.display = 'none';
  }
}
async function tipLoad(){
  const r = await fetch('/tip-center');
  if (r.ok) tipRender(await r.json());
}
el('camframe').onclick = async (e) => {
  if (el('pane-tip').style.display === 'none') return;
  const box = el('cam').getBoundingClientRect();
  const u = (e.clientX - box.left) / box.width  * 640;
  const v = (e.clientY - box.top)  / box.height * 480;
  if (u < 0 || u > 640 || v < 0 || v > 480) return;
  const r = await post('/tip-center', {u: u, v: v});
  if (r.ok) { tipRender(await r.json()); el('tipStatus').textContent = 'set at '+u.toFixed(0)+', '+v.toFixed(0); }
};
el('tipClear').onclick = async () => { await post('/tip-center/clear',{}); tipLoad(); };
el('tipReload').onclick = () => tipLoad();
tipLoad();
el('wcCross').onclick = () => { crossOn = !crossOn; showCross(crossOn); };
function renderWrist(w){
  if (!w) return;
  if (w.roll_deg !== undefined && w.roll_deg !== null){
    const rel = w.roll_rel_deg;
    el('wcRoll').textContent = (rel >= 0 ? '+' : '') + rel.toFixed(1) + '°';
    el('wcRel').textContent = w.roll_deg.toFixed(1) + '°';
    const span = Math.max(1e-6, w.max_rel_deg - w.min_rel_deg);
    el('wcBar').style.width = Math.max(0, Math.min(100, (rel - w.min_rel_deg) / span * 100)) + '%';
  }
  el('wcMin').textContent = 'min ' + w.min_rel_deg.toFixed(1) + '°';
  el('wcMax').textContent = 'max ' + w.max_rel_deg.toFixed(1) + '°';
  el('wcZeroInfo').textContent = w.calibrated
    ? 'zero at servo ' + w.zero_deg.toFixed(1) + '° · window −90…+90°'
    : 'not calibrated — default window −90…+90°';
}
async function rollJog(delta){
  const r = await post('/jog', {joint: 'wrist_roll', delta: delta});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){ el('wcStatus').textContent = 'roll refused: ' + (d.detail || r.status); return; }
  el('wcStatus').textContent = (d.note ? d.note : 'roll ' + (delta > 0 ? '+' : '') + delta + '°') +
    ' · settle ' + d.error_deg + '°';
}
el('wcPan').onclick = async () => {
  el('wcStatus').textContent = 'zeroing pan…';
  const r = await post('/wrist/pan-zero', {});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){ el('wcStatus').textContent = 'pan zero failed: ' + (d.detail || r.status); return; }
  el('wcStatus').textContent = 'pan zeroed · settle ' + d.error_deg + '°';
};
el('wcAccept').onclick = async () => {
  const r = await post('/wrist/zero', {});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){ el('wcStatus').textContent = 'zero failed: ' + (d.detail || r.status); return; }
  el('wcStatus').textContent = 'wrist 0 ' +
    (d.servo_saved ? 'saved to servo' : 'saved locally (servo write unavailable)') +
    ' · window ' + d.min_rel_deg.toFixed(0) + '…' + d.max_rel_deg.toFixed(0);
  if (!d.servo_saved && d.servo_error) log('wrist servo: ' + d.servo_error);
  renderWrist(d);
};
el('wcAuto').onclick = async () => {
  el('wcStatus').textContent = 'auto-aligning on ArUco markers…';
  const r = await post('/wrist/auto-align', {});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){ el('wcStatus').textContent = 'auto-align failed: ' + (d.detail || r.status); return; }
  el('wcStatus').textContent = 'auto-align ' + (d.ok ? 'ok' : 'incomplete') +
    ' · tilt ' + d.tilt_before.toFixed(1) + '° → ' + d.tilt_after.toFixed(1) +
    '° in ' + d.iterations + ' moves';
};

// ---------------------------------------------------------- camera yaw panel
function yawSigned(v){ return (v >= 0 ? '+' : '') + Number(v).toFixed(1) + '°'; }
function renderYaw(y){
  if (!y) return;
  el('wcYaw').textContent = yawSigned(y.offset_deg);
  if (y.measured_deg === null || y.measured_deg === undefined){
    el('wcYawMeasured').textContent = '— (press Auto-measure)';
  } else {
    el('wcYawMeasured').textContent = yawSigned(y.measured_deg) +
      (y.measured_stale ? ' (stale · measured at ' + yawSigned(y.measured_offset_deg) + ')' : '');
  }
  el('wcYawPath').textContent = 'file ' + (y.path || 'live/camera_yaw.json') +
    (y.updated ? ' · saved ' + y.updated : ' · not saved yet');
}
async function yawPost(path, body){
  const r = await post(path, body || {});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){ el('wcYawStatus').textContent = 'yaw failed: ' + (d.detail || r.status); return null; }
  renderYaw(d); return d;
}
async function yawJog(delta){
  const d = await yawPost('/wrist/yaw', {delta_deg: delta, source: 'button'});
  if (d) el('wcYawStatus').textContent = 'offset ' + yawSigned(d.offset_deg) +
    ' (was ' + yawSigned(d.previous_offset_deg) + ')';
}
el('wcYawAccept').onclick = async () => {
  el('wcYawStatus').textContent = 'nulling the measured yaw…';
  const d = await yawPost('/wrist/yaw/zero');
  if (d) el('wcYawStatus').textContent = 'yaw 0 accepted · offset ' +
    yawSigned(d.offset_deg) + ' (was ' + yawSigned(d.previous_offset_deg) +
    ', measured ' + yawSigned(d.measured_deg) + ')';
};
el('wcYawAuto').onclick = async () => {
  el('wcYawStatus').textContent = 'measuring the board with a fresh capture…';
  const d = await yawPost('/wrist/yaw/auto');
  if (d) el('wcYawStatus').textContent = 'ArUco measured ' +
    yawSigned(d.measurement.measured_deg) + ' · offset ' + yawSigned(d.offset_deg) +
    ' (was ' + yawSigned(d.previous_offset_deg) + ') · rms ' +
    d.measurement.rms_px + ' px · ' + d.measurement.markers.length + ' markers';
};
el('wcYawReset').onclick = async () => {
  const d = await yawPost('/wrist/yaw', {offset_deg: 0, source: 'reset'});
  if (d) el('wcYawStatus').textContent = 'offset reset to 0° (was ' +
    yawSigned(d.previous_offset_deg) + ')';
};

// ----------------------------------------------------------------- camera tab
// Ranges, menus and the inactive flags all come from the device, so the tab
// cannot offer a value the driver would refuse.  Moving a control applies it;
// the profile is only written when Save as startup is pressed.
let camState = null;
let camRows = {};
let camTimer = null;
let camBusy = false;
let camPending = false;
let camTouched = {name: null, at: 0};
function camFind(name){
  return (camState && camState.controls.find(c => c.name === name)) || null;
}
function camBuild(control){
  const row = document.createElement('div');
  row.className = 'camrow' + (control.inactive ? ' off' : '');
  const head = document.createElement('div');
  head.className = 'head';
  const name = document.createElement('span');
  name.className = 'hint';
  name.textContent = control.name.replace(/_/g, ' ');
  const shown = document.createElement('span');
  shown.className = 'mono hint';
  head.append(name, shown);
  row.append(head);
  const line = document.createElement('div');
  line.style.cssText = 'display:flex;gap:6px;align-items:center';
  const hint = document.createElement('div');
  hint.className = 'hint';
  hint.style.cssText = 'font-size:10px;color:#5f7b96';
  const describe = (c) => (c.type === 'bool'
      ? 'bool · default ' + (c.default ? 'on' : 'off')
      : 'range ' + c.min + '…' + c.max +
        ' · step ' + (c.step || 1) + ' · default ' + c.default)
    + (c.inactive ? ' · inactive while its auto is on' : '');

  let read, set;
  if (control.type === 'bool'){
    const box = document.createElement('input');
    box.type = 'checkbox';
    box.checked = !!control.value;
    box.disabled = control.inactive;
    box.onchange = () => { shown.textContent = box.checked ? 'on' : 'off'; camTouch(control.name); camQueue(); };
    line.append(box);
    read = () => (box.checked ? 1 : 0);
    set = (c) => { box.checked = !!c.value; box.disabled = c.inactive; };
    shown.textContent = control.value ? 'on' : 'off';
  } else if (control.type === 'menu'){
    const sel = document.createElement('select');
    sel.disabled = control.inactive;
    for (const item of control.menu){
      const opt = document.createElement('option');
      opt.value = item.value;
      opt.textContent = item.value + ' · ' + item.label;
      if (item.value === control.value) opt.selected = true;
      sel.append(opt);
    }
    sel.onchange = () => { shown.textContent = sel.value; camTouch(control.name); camQueue(0); };
    line.append(sel);
    read = () => parseInt(sel.value, 10);
    set = (c) => { sel.value = c.value; sel.disabled = c.inactive; };
    shown.textContent = control.value;
  } else {
    const range = document.createElement('input');
    range.type = 'range';
    range.min = control.min; range.max = control.max;
    range.step = control.step || 1;
    range.value = control.value;
    range.disabled = control.inactive;
    range.style.flex = '1';
    const num = document.createElement('input');
    num.type = 'number';
    num.min = control.min; num.max = control.max;
    num.step = control.step || 1;
    num.value = control.value;
    num.disabled = control.inactive;
    // Dragging is a stream of values: apply during the drag, but let the last
    // one land instead of firing a write per pixel.
    range.oninput = () => { num.value = range.value; shown.textContent = range.value; camTouch(control.name); camQueue(); };
    range.onchange = () => { camTouch(control.name); camQueue(0); };
    num.oninput = () => {
      if (num.value !== '') range.value = num.value;
      shown.textContent = num.value;
      camTouch(control.name); camQueue();
    };
    num.onchange = () => { camTouch(control.name); camQueue(0); };
    line.append(range, num);
    read = () => parseInt(num.value, 10);
    set = (c) => {
      range.value = c.value; num.value = c.value;
      range.disabled = c.inactive; num.disabled = c.inactive;
    };
    shown.textContent = control.value;
  }
  row.append(line);
  hint.textContent = describe(control);
  row.append(hint);
  camRows[control.name] = {
    read: read,
    // Skip the row the hand is on, so a rounded read-back cannot pull the
    // slider out from under a drag.
    set: (c) => {
      const recent = camTouched.name === c.name && Date.now() - camTouched.at < 1500;
      if (!recent) set(c);
      shown.textContent = c.value === null ? '—'
        : (c.type === 'bool' ? (c.value ? 'on' : 'off') : c.value);
      row.classList.toggle('off', c.inactive);
      hint.textContent = describe(c);
    },
  };
  return row;
}
function camRender(){
  if (!camState) return;
  const box = el('camControls');
  box.innerHTML = '';
  camRows = {};
  el('camDevice').textContent = camState.device || '—';
  el('camSavedAt').textContent = camState.saved_at
    ? 'profile ' + camState.saved_at
    : 'no saved profile';
  for (const control of camState.controls) box.append(camBuild(control));
  const exp = camFind('auto_exposure');
  const wb = camFind('white_balance_automatic');
  el('camAutoExp').checked = exp ? exp.value !== 1 : false;
  el('camAutoExp').disabled = !exp;
  el('camAutoWb').checked = wb ? !!wb.value : false;
  el('camAutoWb').disabled = !wb;
}
function camUpdate(controls){
  for (const control of controls || []){
    const row = camRows[control.name];
    if (row) row.set(control);
  }
}
function camGather(){
  const exp = camFind('auto_exposure');
  const wb = camFind('white_balance_automatic');
  const manualExp = exp ? !el('camAutoExp').checked : false;
  const manualWb = wb ? !el('camAutoWb').checked : false;
  const controls = {};
  for (const control of camState.controls){
    // A dependent control is only writable while its auto is off: skip it
    // whenever this request leaves it inactive, whatever the device says now.
    if (control.name === 'exposure_time_absolute' && !manualExp) continue;
    if (control.name === 'white_balance_temperature' && !manualWb) continue;
    if (control.inactive) continue;
    const row = camRows[control.name];
    if (!row) continue;
    const value = row.read();
    if (!Number.isFinite(value)) continue;
    controls[control.name] = value;
  }
  if (exp) controls.auto_exposure = manualExp ? 1 : 3;
  if (wb) controls.white_balance_automatic = manualWb ? 0 : 1;
  return controls;
}
function camReport(d, saved){
  const read = Object.entries(d.applied || {}).map(([k, v]) => k + ' ' + v);
  // The C270 snaps some values onto its own grid (exposure moves in 5s, gain
  // in 2s) whatever the descriptor advertises.  A difference that small is the
  // device rounding, not a value that failed to take, so the two are reported
  // separately.
  const bad = [], rounded = [];
  for (const [k, v] of Object.entries(d.mismatched || {})){
    const control = camFind(k);
    const step = (control && control.step) || 1;
    const text = k + ' ' + v.requested + '→' + v.read_back;
    (Math.abs(v.read_back - v.requested) <= Math.max(2 * step, 5) ? rounded : bad).push(text);
  }
  const skipped = Object.entries(d.skipped || {}).map(([k, why]) => k + ' (' + why + ')');
  el('camStatus').textContent = 'read back: ' + (read.join(' · ') || 'nothing')
    + (rounded.length ? ' · rounded by the device: ' + rounded.join(', ') : '')
    + (bad.length ? ' · did not take: ' + bad.join(', ') : '')
    + (skipped.length ? ' · skipped: ' + skipped.join(', ') : '')
    + (saved ? ' · saved as startup' : '');
}
async function camApply(save){
  if (!camState) return;
  clearTimeout(camTimer); camTimer = null;
  if (camBusy){ camPending = true; return; }
  camBusy = true;
  const payload = camGather();
  el('camStatus').textContent = 'applying…';
  try {
    const r = await post('/camera/controls', {controls: payload, save: !!save});
    const d = await r.json().catch(() => ({}));
    if (!r.ok){
      el('camStatus').textContent = 'apply failed: ' + (d.detail || r.status);
      return;
    }
    camState = {device: d.device, controls: d.controls,
                saved_at: d.saved_at || camState.saved_at};
    camUpdate(d.controls);
    el('camSavedAt').textContent = camState.saved_at
      ? 'profile ' + camState.saved_at
      : 'no saved profile';
    camReport(d, !!save);
  } finally {
    camBusy = false;
    if (camPending){ camPending = false; camQueue(0); }
  }
}
// One write per gesture: a drag queues the next apply instead of sending one
// per pixel, and releasing the slider sends it at once.
function camQueue(delay){
  if (camTouched.name) camTouched.at = Date.now();
  clearTimeout(camTimer);
  camTimer = setTimeout(() => camApply(false), delay === undefined ? 350 : delay);
}
function camTouch(name){ camTouched = {name: name, at: Date.now()}; }
async function camLoad(){
  const r = await fetch('/camera/controls');
  const d = await r.json().catch(() => ({}));
  if (!r.ok){
    el('camStatus').textContent = 'load failed: ' + (d.detail || r.status);
    return;
  }
  camState = d;
  camRender();
  const saved = d.saved ? Object.keys(d.saved).length : 0;
  el('camStatus').textContent = saved
    ? 'device reports its current values · saved profile holds ' + saved + ' controls'
    : 'device reports its current values · no saved profile yet';
}
el('camApply').onclick = () => camApply(false);
el('camSave').onclick = () => camApply(true);
el('camReload').onclick = () => camLoad();
el('camAutoExp').onchange = () => camApply(false);
el('camAutoWb').onchange = () => camApply(false);
el('camReset').onclick = async () => {
  if (!camState) return;
  const exp = camFind('auto_exposure');
  const wb = camFind('white_balance_automatic');
  const manualExp = exp ? exp.default === 1 : false;
  const manualWb = wb ? !wb.default : false;
  const controls = {};
  for (const control of camState.controls){
    if (control.name === 'exposure_time_absolute' && !manualExp) continue;
    if (control.name === 'white_balance_temperature' && !manualWb) continue;
    controls[control.name] = control.default;
  }
  el('camStatus').textContent = 'resetting to defaults…';
  const r = await post('/camera/controls', {controls: controls, save: true});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){
    el('camStatus').textContent = 'reset failed: ' + (d.detail || r.status);
    return;
  }
  camState = {device: d.device, controls: d.controls,
              saved_at: d.saved_at || camState.saved_at};
  camRender();
  camReport(d, true);
};

let limits = null, look = {pitch: 0, az: 0, dist: 240};
function bar(lo, hi, now){
  const span = Math.max(1e-6, hi - lo), pct = Math.max(0, Math.min(100, (now - lo)/span*100));
  return '<div style="position:relative;height:5px;background:#1b2836;border-radius:3px;margin:3px 0 2px">'+
    '<div style="position:absolute;left:'+pct+'%;top:-2px;width:2px;height:9px;background:#7fd4a0"></div></div>';
}
function renderLimits(){
  if (!limits) { el('xyzrows').innerHTML = '<div style="opacity:.6">no range yet</div>'; return; }
  const step = Math.max(1, +el('jstep').value || 5);
  el('xyzrows').innerHTML = ['x','y','z'].map(a => {
    const L = limits.axes[a]; if (!L) return '';
    const lo = Math.round(L.min), hi = Math.round(L.max), now = Math.round(L.now);
    const span = hi - lo;
    return '<div style="margin-bottom:6px"><div style="display:flex;justify-content:space-between">'+
      '<span>'+a+'</span><span>'+(span > 0 ? (now-lo)+' of '+span+' mm' : 'locked')+'</span></div>'+
      '<div class="grid" style="grid-template-columns:repeat(3,1fr);gap:4px;margin-top:3px">'+
      '<button style="padding:3px 0" onclick="moveRel(\\''+a+'\\',-'+step+')">−'+step+'</button>'+
      '<div style="text-align:center;line-height:26px">'+now+'</div>'+
      '<button style="padding:3px 0" onclick="moveRel(\\''+a+'\\','+step+')">+'+step+'</button></div>'+
      bar(lo, hi, now)+
      '<div style="display:flex;justify-content:space-between;opacity:.45;font-size:10px"><span>'+lo+'</span><span>'+hi+'</span></div></div>';
  }).join('');
}
async function refreshLimits(){
  const step = Math.max(2, +el('jstep').value || 5);
  el('limhint').textContent = 'measuring reachable range…';
  try {
    const r = await fetch('/cart-limits?step_mm=' + step);
    if (!r.ok){ el('limhint').textContent = 'range failed: ' + r.status; return; }
    limits = await r.json();
    renderLimits();
    el('limhint').textContent = 'ranges measured at the current pose';
  } catch(e){ el('limhint').textContent = 'range error: ' + e; }
}
el('limbtn').onclick = refreshLimits;
el('jstep').oninput = () => renderLimits();
async function moveRel(axis, delta){
  const body = {mode:'rel'}; body['d'+axis+'_mm'] = delta;
  const r = await post('/move', body);
  const d = await r.json().catch(()=>({}));
  if (!r.ok){ el('limhint').textContent = 'refused: ' + (d.detail||r.status); return; }
  el('limhint').textContent = 'moved '+axis+' '+delta+' mm';
  limits = null; refreshLimits();
}
async function jawJog(delta){ await post('/jog',{joint:'gripper',delta:delta}); }
function jogToCentre(){ post('/move',{mode:'xyz',x_mm:14,y_mm:-14,z_mm:60}); }
function renderLook(){
  el('lookrows').innerHTML = [['pitch','lp',5],['azimuth','la',10],['distance','ld',20]].map(([n,k,s])=>{
    const v = Math.round(look[k==='lp'?'pitch':k==='la'?'az':'dist']);
    return '<div style="display:flex;justify-content:space-between;margin-bottom:4px">'+
      '<span>'+n+'</span><span>'+v+'</span></div>'+
      '<div class="grid" style="grid-template-columns:repeat(2,1fr);gap:4px;margin-bottom:6px">'+
      '<button style="padding:3px 0" onclick="lookAdjust(\\''+k+'\\','+(-s)+')">−'+s+'</button>'+
      '<button style="padding:3px 0" onclick="lookAdjust(\\''+k+'\\','+s+')">+'+s+'</button></div>';
  }).join('');
}
async function lookAdjust(k, delta){
  if (k === 'lp') look.pitch = Math.max(-80, Math.min(80, look.pitch + delta));
  if (k === 'la') look.az = look.az + delta;
  if (k === 'ld') look.dist = Math.max(120, Math.min(600, look.dist + delta));
  renderLook();
  const r = await post('/move',{mode:'look_at', x_mm:0, y_mm:0, z_mm:0,
    distance_mm: look.dist, pitch_deg: look.pitch, azimuth_deg: look.az});
  const d = await r.json().catch(()=>({}));
  if (!r.ok) el('limhint').textContent = 'aim refused: ' + (d.detail||r.status);
}
renderLook();
el('sq').oninput = () => {
  const m = el('sq').value.trim().toLowerCase().match(/^([a-h])([1-8])$/);
  if (!m) return;
  const x = (m[1].charCodeAt(0)-97+0.5)*28-112, y = (parseInt(m[2],10)-0.5)*28-112;
  el('mx').value = x.toFixed(0); el('my').value = y.toFixed(0);
};
function moveBody(){
  if (moveMode === 'xyz'){
    return {mode:'xyz', x_mm:+el('mx').value, y_mm:+el('my').value, z_mm:+el('mz').value,
            span_deg: el('mspan').value === '' ? null : +el('mspan').value,
            jaw:+el('mjaw').value};
  }
  if (moveMode === 'look'){
    return {mode:'look_at', x_mm:+el('lx').value, y_mm:+el('ly').value, z_mm:+el('lz').value,
            distance_mm:+el('ld').value, pitch_deg:+el('lp').value, azimuth_deg:+el('la').value};
  }
  const joints = {};
  JOINTS.forEach((n,i) => { const v = el('j'+i).value; if (v !== '') joints[n] = +v; });
  return {mode:'joint', joints: joints};
}
el('gobtn').onclick = async () => {
  el('moveStatus').textContent = 'solving…';
  const r = await post('/move', moveBody());
  const d = await r.json().catch(() => ({}));
  if (!r.ok){ el('moveStatus').textContent = 'refused: ' + (d.detail || r.status); return; }
  const q = d.requested || {};
  el('moveStatus').textContent = (q.mode||'') + ' ok · ik ' + (q.position_error_mm ?? 0) + ' mm / ' +
    (q.approach_error_deg ?? 0) + ' deg · settle ' + d.error_deg + ' deg';
  log('move ' + (q.mode||'') + ' err ' + d.error_deg + ' deg');
};
el('fillbtn').onclick = async () => {
  const d = await (await fetch('/pose')).json();
  if (d.detail) { log('pose: '+d.detail); return; }
  el('mx').value = d.tool_mm[0].toFixed(0); el('my').value = d.tool_mm[1].toFixed(0);
  el('mz').value = d.tool_mm[2].toFixed(0);
  el('lx').value = d.camera_mm[0].toFixed(0); el('ly').value = d.camera_mm[1].toFixed(0);
  el('lz').value = '0'; el('ld').value = d.camera_height_mm.toFixed(0);
  el('lp').value = d.view_pitch_deg.toFixed(1);
  JOINTS.forEach((n,i) => { if (d.joints_deg) el('j'+i).value = d.joints_deg[n]; });
  log('filled from arm');
};
function renderPose(p){
  const jaw = p.joints_deg ? p.joints_deg.gripper : null;
  const row = (label, a, b, c, d) =>
    '<tr><td style="padding-right:8px;color:#7f97ae">'+label+'</td>'+
    '<td style="text-align:right;padding-right:6px">'+a+'</td>'+
    '<td style="text-align:right;padding-right:6px">'+b+'</td>'+
    '<td style="text-align:right;padding-right:6px">'+c+'</td>'+
    '<td style="text-align:right">'+d+'</td></tr>';
  const fmt = v => (v === null || v === undefined) ? '—' : Number(v).toFixed(1);
  el('posetable').innerHTML =
    '<table style="width:100%;border-collapse:collapse">'+
    '<tr style="color:#6f8aa6"><td></td><td style="text-align:right">x</td>'+
    '<td style="text-align:right">y</td><td style="text-align:right">z</td>'+
    '<td style="text-align:right">jaw</td></tr>'+
    row('tool', fmt(p.tool_mm[0]), fmt(p.tool_mm[1]), fmt(p.tool_mm[2]), fmt(jaw))+
    row('camera', fmt(p.camera_mm[0]), fmt(p.camera_mm[1]), fmt(p.camera_mm[2]), fmt(jaw))+
    '</table>'+
    '<div style="margin-top:4px;opacity:.65">cam tilt '+fmt(p.view_pitch_deg)+'° · jaw azimuth '+
    fmt(p.jaw_azimuth_deg)+'°</div>';
}
function renderJoints(b){
  el('joints').innerHTML = JOINTS.map(n => {
    const v = b.joints ? b.joints[n] : null, L = b.limits ? b.limits[n] : null;
    if (v == null || !L) return '<div style="opacity:.5">'+SHORT[n]+': --</div>';
    const lo = L[0], hi = L[1], span = Math.max(1e-6, hi-lo), pct = Math.max(0, Math.min(100, (v-lo)/span*100));
    const col = Math.min(v-lo, hi-v) < 0.06*span ? '#ffb4b4' : '#7fd4a0';
    return '<div style="margin:0 0 5px"><div style="display:flex;justify-content:space-between">'+
      '<span>'+SHORT[n]+'</span><span style="color:'+col+'">'+v.toFixed(1)+'</span></div>'+
      '<div style="position:relative;height:5px;background:#1b2836;border-radius:3px;margin-top:2px">'+
      '<div style="position:absolute;left:'+pct+'%;top:-2px;width:2px;height:9px;background:'+col+'"></div></div>'+
      '<div style="display:flex;justify-content:space-between;opacity:.45;font-size:10px"><span>'+
      lo.toFixed(0)+'</span><span>'+hi.toFixed(0)+'</span></div></div>';
  }).join('');
}
function flash(msg, ok){ const s = el('poseStatus'); s.textContent = msg;
  s.style.background = ok ? '#16351f' : '#3a1b1b'; s.style.color = ok ? '#b6f2c8' : '#ffb4b4'; }
async function refreshProgress(){
  const p = await (await fetch('/progress')).json();
  const tags = [0,1,2,3,4].map(i => i+':'+(p.per_tag[i]||0)).join('  ');
  el('progress').textContent = 'views '+p.views+'/'+p.target+'   tags '+tags;
}
async function savePose(){
  try {
    const r = await post('/pose/save',{});
    const d = await r.json();
    if (!r.ok){ flash('save failed: '+(d.detail||r.status), false); return; }
    if (d.duplicate_of) flash('NOT SAVED — same view as '+d.duplicate_of+', move the arm', false);
    else if (d.reason === 'moving') flash('NOT SAVED — arm moved ('+d.moved_deg+'° / '+d.moved_px+'px)', false);
    else if (d.reason === 'no_tag') flash('NOT SAVED — no tag in view', false);
    else { const m = d.capture.markers[0]; flash('saved '+d.saved+' — tag '+m.id+' at '+m.edge_px+'px', true); }
    refreshProgress(); listPoses(); listCaptures();
  } catch(e){ flash('save error: '+e, false); }
}
async function goTo(n){
  const r = await post('/pose/run',{name:n});
  if (r.ok){ const d = await r.json(); el('poseStatus').textContent = 'at '+n+' (error '+d.error_deg+' deg)'; }
  else el('poseStatus').textContent = 'move failed';
}
async function listPoses(){
  const r = await (await fetch('/poses')).json();
  const names = Object.keys(r);
  el('poses').innerHTML = names.length ? names.map(n => '<button onclick="goTo(\\''+n+'\\')">'+n+'</button>').join(' ') : 'none';
}
el('capture').onclick = async () => {
  const r = await post('/capture-view',{label:''});
  if (r.ok){ const d = await r.json();
    el('capStatus').textContent = 'captured '+d.name+' · markers '+d.markers.map(m=>m.id).join(',')+
      ' · drift '+d.joint_drift_deg+'°';
    listCaptures(); }
};
async function listCaptures(){
  const r = await (await fetch('/captures')).json();
  el('caps').textContent = r.count+' views: '+r.items.map(i => i.name.replace(/^\d{8}T\d{6}_?/,'')+
    '['+i.markers.join('')+']').join(' ');
}
async function loadBoard(){
  const b = await (await fetch('/board')).json();
  el('bx').value = b.x_mm.toFixed(1); el('by').value = b.y_mm.toFixed(1);
  el('bz').value = b.z_mm.toFixed(1); el('byaw').value = b.yaw_deg.toFixed(2);
}
el('boardsave').onclick = async () => {
  const r = await post('/board',{x_mm:+el('bx').value, y_mm:+el('by').value,
    z_mm:+el('bz').value, yaw_deg:+el('byaw').value});
  el('boardStatus').textContent = r.ok ? 'board saved' : 'board save failed';
  if (r.ok) loadBoard();
};
el('boardload').onclick = loadBoard;
el('boardderive').onclick = async () => {
  const r = await post('/board/derive', {save:true});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){ el('boardStatus').textContent = 'derive failed: ' + (d.detail || r.status); return; }
  el('boardStatus').textContent = 'derived from id '+d.from_marker+' · '+d.reproj_px+' px · cross-check '+
    d.cross_check_mm+' mm';
  loadBoard();
};
el('boardfill').onclick = async () => {
  const p = await (await fetch('/pose')).json();
  if (!p.tool_mm){ el('boardStatus').textContent = 'no pose'; return; }
  el('bx').value = p.tool_mm[0].toFixed(0);
  el('by').value = p.tool_mm[1].toFixed(0);
  el('bz').value = (p.tool_mm[2] + 0).toFixed(0);
  el('byaw').value = '0';
  el('boardStatus').textContent = 'filled from tool position — edit z to the board surface';
};
el('arm').onclick = () => post('/arm',{confirm:true}).then(()=>log('arm'));
el('remote').onclick = () => post('/mode',{mode:'REMOTE'}).then(()=>log('remote'));
el('teleop').onclick = () => post('/mode',{mode:'TELEOP'}).then(()=>log('teleop'));
async function runHome(execute){
  const r = await post('/home', {execute: execute});
  const d = await r.json().catch(()=>({}));
  if (!r.ok){ el('sanityOut').textContent = 'home failed: ' + (d.detail || r.status); return; }
  const where = d.tool_mm ? d.tool_mm.join(', ') : '?';
  el('sanityOut').textContent = execute
    ? 'home: tool at ' + where + ' (settle ' + d.settle_deg + '°)'
    : 'home would put tool at ' + where + (d.inside_zone ? ' · inside zone' : ' · OUTSIDE zone');
  log('home ' + (execute ? 'moved' : 'checked'));
}
el('home').onclick = () => runHome(true);
el('homedry').onclick = () => runHome(false);

async function runSanity(execute){
  el('sanityOut').textContent = execute ? 'moving through targets…' : 'checking reachability…';
  const r = await post('/sanity', {execute: execute, z_mm: 120});
  const d = await r.json().catch(()=>({}));
  if (!r.ok){ el('sanityOut').textContent = 'sanity failed: ' + (d.detail || r.status); return; }
  const lines = d.targets.map(t => {
    const base = t.target.padEnd(7) + ' ik ' + t.ik_mm + 'mm/' + t.ik_deg + '°';
    if (!t.reachable) return base + '  UNREACHABLE';
    if (!t.moved) return base + '  move failed: ' + (t.error || '');
    return base + '  landed ' + (t.landed_mm ? t.landed_mm.join(',') : '?') +
           ' (' + t.landed_error_mm + ' mm off)';
  });
  el('sanityOut').innerHTML = '<b>' + d.passed + '/' + d.total + ' passed</b><br>' + lines.join('<br>');
  log('sanity ' + d.passed + '/' + d.total);
}
el('sanity').onclick = () => runSanity(true);
el('sanitydry').onclick = () => runSanity(false);
el('scan').onclick = () => post('/scan',{}).then(()=>log('scan started'));
el('estop').onclick = () => post('/estop',{}).then(()=>log('E-STOP'));
async function tick(){
  try {
    const s = await (await fetch('/state')).json();
    const b = s.bridge || {};
    renderJoints(b);
    renderWrist(s.wrist);
    renderYaw(s.camera_yaw);
    el('mode').textContent = (b.mode||'?') + (b.armed ? ' · armed' : ' · disarmed');
    el('mode').className = 'badge ' + (b.mode === 'FAULT' ? 'bad' : (b.mode === 'REMOTE' ? 'ok' : ''));
    el('link').textContent = b.follower_open ? 'follower open' : (b.lelab_active ? 'teleop' : 'idle');
    if (s.error){ el('err').style.display = 'inline'; el('err').textContent = s.error; }
    else el('err').style.display = 'none';
  } catch(e){ log('state: '+e); }
  try {
    const m = await (await fetch('/metrics')).json();
    el('score').textContent = Math.round(m.score);
    peak = Math.max(peak, m.score); el('peak').textContent = 'peak '+Math.round(peak);
    el('fill').style.width = Math.min(100, m.score/6)+'%';
    el('markers').textContent = m.markers.length ? m.markers.map(x=>'id'+x.id+' '+x.edge_px+'px').join(' · ') : 'no marker';
    el('dist').textContent = m.distance_mm ? Math.round(m.distance_mm)+' mm' : '—';
  } catch(e){ log('metrics: '+e); }
  try {
    const k = await (await fetch('/keep-in')).json();
    el('keepin').textContent = 'zone ' + Math.round(k.x_mm) + '×' + Math.round(k.y_mm) +
      ' · ' + (k.inside === true ? 'inside' : k.inside === false ? 'OUTSIDE' : '—');
    el('keepin').className = 'badge ' + (k.inside === false ? 'bad' : (k.inside ? 'ok' : ''));
  } catch(e){}
  try {
    const p = await (await fetch('/pose')).json();
    if (p.tool_mm){
      renderPose(p);
    }
  } catch(e){}
  setTimeout(tick, 900);
}
tick(); listPoses(); listCaptures(); refreshProgress(); loadBoard(); view('live');

</script></body></html>
"""

PAGE_FOCUS = PAGE

NO_CACHE = {"Cache-Control": "no-store, max-age=0"}


@app.get("/")
def index():
    return HTMLResponse(PAGE, headers=NO_CACHE)


# Click-to-label UI for the real-piece dataset, served on the same reachable
# port as the console (:8080) because :8123 is filtered off-box.
try:
    import label_routes
    label_routes.install(app)
except Exception as _exc:  # never let the labeler break the console
    print(f'label_routes not installed: {_exc}', file=sys.stderr)


@app.get("/focus")
def focus_page():
    # The focus helper is no longer a separate page: score, joints, moves and
    # captures all live on one dashboard.
    return RedirectResponse("/", status_code=307)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(live.config.get("port", 8080)),
                log_level="info")
