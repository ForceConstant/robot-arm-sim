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
from dataclasses import replace
from pathlib import Path
import threading
import time
from collections import deque

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import (HTMLResponse, JSONResponse, RedirectResponse,
                               Response, StreamingResponse)

from bridge_client import BridgeError, HttpBridge, HttpFrames
from kinematics import JOINTS, SQUARE_MM, ArmKinematics, square_xy
from scan_controller import ScanController, plan_in_degrees
from target_overlay import TargetOverlay, draw as draw_target
from tip_frame import TIP_IN_CHAIN, describe as describe_tip_frame
from chess_sim import fiducials
from chess_sim.marker_scan import (MarkerLayout, board_offset, load_plan,
                                   marker_corners_world)
from chess_sim.product import current_layout
from chess_sim.wrist_calibration import WristCalibration

CONFIG_PATH = Path(os.environ.get("LIVE_CONFIG", "live/live_config.json"))
DEFAULTS = {
    "bridge_url": "http://192.168.1.167:8100",
    "token_file": "live/bridge_token.txt",
    "calibration": ".private/phase11d/calibration.json",
    "plan": "chess_sim/assets/marker_scan_plan.json",
    "board_half_mm": 127.0,
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
    # Working envelope in board-frame mm.  The arm should stay over the playing
    # surface, both so the camera keeps the board in view and so a Cartesian
    # mistake cannot drive the jaws into the table beside it.
    "keep_in_x_mm": 118.0,
    "keep_in_y_mm": 118.0,
    "keep_in_z_min_mm": -5.0,
    "keep_in_z_max_mm": 320.0,
    "keep_in_enforce": True,
    # Acceptance gate for a solved Cartesian target. Tight it to prove the
    # model; loosen it deliberately when measuring the model's bias.
    "move_tolerance_mm": 20.0,
    # Cartesian targets mean the jaw tip.  The one chain->tip transform lives
    # in ``tip_frame.TIP_IN_CHAIN`` (MJCF-derived, see that module); it is not
    # a config value because three competing offsets in the tree is exactly
    # the bug this replaced.
    # Relaxed middle pose the controller can always return to: joint targets, so
    # homing never depends on IK finding a solution from a bad posture.
    "home_joints": {"shoulder_pan": 0.0, "shoulder_lift": -55.0, "elbow_flex": 55.0,
                    "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 40.0},
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
        self.lock = threading.Lock()
        self.scan_lock = threading.Lock()
        self.last_scan: dict | None = None
        self.scan_running = False
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
        self.wrist_path = Path(self.config.get(
            "wrist_file", "live/wrist_calibration.json"))
        self.frame_offset.offset_deg = float(
            self.wrist_cal().get("model_offset_deg", 0.0))
        self.kinematics = ArmKinematics(self.calibration, {}, self._board_frame())
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
        fx, fy, cx, cy = self.calibration.intrinsics
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
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
            ok, rvec, tvec = cv2.solvePnP(
                corners, np.asarray(detection.corners, dtype=float),
                camera_matrix, None, flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            # cv2.solvePnP returns board -> OpenCV camera (+z forward, +y down).
            # The calibrated camera frame is MuJoCo's (-z forward, +y up), so
            # flip y/z and compose base<-camera with board->camera.  The old
            # code skipped the flip *and* inverted the transform: on the sim
            # bench that put the board plane 641 mm above the true surface and
            # turned yaw by 20 deg, while x/y looked deceptively plausible.
            flip = np.diag([1.0, -1.0, -1.0])
            camera_from_board = np.eye(4)
            camera_from_board[:3, :3] = flip @ rotation
            camera_from_board[:3, 3] = flip @ np.ravel(tvec)
            board = base_camera @ camera_from_board
            projected, _ = cv2.projectPoints(corners, rvec, tvec, camera_matrix, None)
            error_px = float(np.mean(np.linalg.norm(
                projected.reshape(-1, 2) - np.asarray(detection.corners, dtype=float),
                axis=1)))
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
                    solved = self.kinematics.solve(goal, start)
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
        return self.calibration.camera(tuple(math.radians(v) for v in joints_deg))

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
            self._keep_in_check(target)
            result = self._stream_to(target, start)
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
            solved, _goal = self.kinematics.solve_tip(
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
        zone = self.keep_in()
        if not zone["enforce"]:
            return
        tool = self.kinematics.to_board(self.kinematics.fk(joints_deg)[:3, 3]) * 1000.0
        # The envelope is expressed at the JAW TIP, but the chain frame rides
        # above it, so compare the tip height or every valid pick looks like a
        # ceiling violation (a 240 mm tip read as 325 mm and was refused).
        tip = (self.kinematics.fk(joints_deg) @ self.jaw_frame())[:3, 3]
        tool = self.kinematics.to_board(tip) * 1000.0
        problems = []
        if abs(float(tool[0])) > zone["x_mm"]:
            problems.append(f"x {tool[0]:.0f} outside ±{zone['x_mm']:.0f}")
        if abs(float(tool[1])) > zone["y_mm"]:
            problems.append(f"y {tool[1]:.0f} outside ±{zone['y_mm']:.0f}")
        if not zone["z_min_mm"] <= float(tool[2]) <= zone["z_max_mm"]:
            problems.append(f"z {tool[2]:.0f} outside "
                            f"{zone['z_min_mm']:.0f}..{zone['z_max_mm']:.0f}")
        if problems:
            raise BridgeError("keep-in zone: " + "; ".join(problems))

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

    def move(self, body: dict) -> dict:
        """One motion request: ``xyz``, ``look_at`` or ``joint``.

        All three resolve to joint angles and then stream through the same
        speed-capped path as a jog, so limit and fault guards are unchanged.
        """
        mode = str(body.get("mode") or "xyz").lower()
        if not self.motion.acquire(timeout=5):
            raise BridgeError("another motion is running")
        try:
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
                solved = self.kinematics.solve_multi(goal, start)
                if (solved["position_error_mm"] > 8.0
                        or solved["approach_error_deg"] > 6.0):
                    raise BridgeError(
                        "cannot aim there: off by "
                        f"{solved['position_error_mm']} mm / "
                        f"{solved['approach_error_deg']} deg")
                target = [float(solved["joints_deg"][name]) for name in JOINTS]
                detail = dict(solved, aim_distance_mm=round(distance * 1000.0, 1))
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
                solved = self.kinematics.solve_multi(
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
                    solved, goal = self.kinematics.solve_tip(
                        point, start, self.jaw_frame(), float(span_arg or 0.0))
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
                    solved = self.kinematics.solve_multi(
                        goal, start, span_deg=span_arg,
                        freeze_roll=bool(body.get("roll_lock", False)))
                # Tolerance is measured at the tip now (solve() takes the jaw
                # frame).  Refusing a 6.5 mm landing would waste a reachable
                # square, while the reported error still tells the truth.
                gate = float(self.config.get("move_tolerance_mm", 20.0))
                if (solved["position_error_mm"] > gate
                        or solved["approach_error_deg"] > 8.0):
                    raise BridgeError(
                        "no reachable pose: off by "
                        f"{solved['position_error_mm']} mm / "
                        f"{solved['approach_error_deg']} deg")
                target = [float(solved["joints_deg"][name]) for name in JOINTS]
                detail = solved
            else:
                raise BridgeError(f"unknown move mode {mode!r}")
            if body.get("jaw") is not None:
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
                # before the arm swings through anything.
                return {"dry_run": True, "requested": detail,
                        "target_joints": dict(zip(JOINTS, [round(v, 2) for v in target])),
                        "max_joint_delta_deg": round(delta, 1)}
            self._keep_in_check(target)
            settle = float(body.get("settle_deg", 0.7))
            result = self._stream_to(target, start, tolerance=settle)
            result.update(requested=detail, pose=self.pose_now())
            return result
        finally:
            self.motion.release()

    def _stream_to(self, target: list[float], start: list[float],
                   tolerance: float = 0.7) -> dict:
        """Interpolate at the bridge's speed cap and stop when close."""
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll", "gripper")
        step = 10.5 * 0.066   # degrees per tick at the conservative 10.5 deg/s
        current = list(start)
        deadline = time.time() + 20.0
        while time.time() < deadline:
            error = max(abs(a - b) for a, b in zip(current, target))
            if error <= max(0.05, tolerance):
                break
            # The bridge rejects any command whose sequence is not newer than
            # the last one it accepted, so the counter must survive across calls.
            self.command_seq += 1
            current = [c + max(-step, min(step, t - c)) for c, t in zip(current, target)]
            self.bridge.command(current, seq=self.command_seq, ttl_ms=200,
                                speed_scale=self.config.get("speed_scale", 0.3))
            time.sleep(0.066)
            state = self.bridge.state()
            if state.get("mode") != "REMOTE" or not state.get("armed"):
                raise BridgeError("bridge left REMOTE during motion")
        return {"joints": dict(zip(names, current)),
                "error_deg": round(max(abs(a - b) for a, b in zip(current, target)), 2)}

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
            self.overlay.submit(raw.copy(), time.time())
            # Draw on the undistorted frame: the detector solves pose there, so
            # corners and centroid only line up on the same geometry.
            image = self._orient(self.overlay.rectify(raw))
            snap = self.overlay.snapshot()
            draw_target(image, snap, layout_hw=raw.shape[:2],
                        stream_fps=self.stream_fps())
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


@app.get("/state")
def state():
    try:
        bridge_state = live.bridge.state()
    except BridgeError as exc:
        bridge_state = {"error": str(exc)}
    # Taken before ``live.lock``: the stream thread holds the overlay lock while
    # it asks for the frame rate, so the opposite order here would deadlock.
    target = _target_summary()
    with live.lock:
        return {"bridge": bridge_state,
                "wrist": live.wrist_state(bridge_state),
                "tool_tip": describe_tip_frame(),
                "scan": live.last_scan,
                "scan_running": live.scan_running,
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
    return live.start_scan()


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
        return live.capture_view(str(body.get("label", ""))[:24])
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
        return live.run_pose(str(body.get("name", "")))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/move")
async def move(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return live.move(body)
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc
    except (TypeError, ValueError) as exc:
        raise HTTPException(400, str(exc)) from exc


@app.post("/sanity")
async def sanity(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return live.sanity_check(execute=bool(body.get("execute", True)),
                                 z_mm=float(body.get("z_mm", 120.0)),
                                 tolerance_mm=float(body.get("tolerance_mm", 6.0)))
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/home")
async def home(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        return live.home(execute=bool(body.get("execute", True)))
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
        return live.derive_board(save=bool(body.get("save", True)))
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
        return live.jog(str(body.get("joint", "")), float(body.get("delta", 0.0)))
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
        return live.set_wrist_zero(None if zero is None else float(zero))
    except (BridgeError, ValueError) as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/wrist/pan-zero")
async def wrist_pan_zero(request: Request):
    _authorised(request)
    try:
        pan = float(live.wrist_cal().get("pan_zero_deg", 0.0))
        return live.move({"mode": "joint", "joints": {"shoulder_pan": pan},
                          "settle_deg": 0.1})
    except BridgeError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/wrist/auto-align")
async def wrist_auto_align(request: Request):
    _authorised(request)
    try:
        return live.auto_align_wrist()
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
</nav></header>
<main>
  <div>
    <div class="cam"><div class="camframe"><img id="cam" src="/stream" alt="wrist camera">
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
      <div class="hint" style="margin-top:6px">Values shown are read back from the device after every write. Save as startup writes the profile the bridge re-applies whenever it reopens the camera.</div>
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
let crossOn = true;
function showCross(on){
  const wristVisible = el('pane-wrist').style.display !== 'none';
  el('cross').style.display = (on && wristVisible) ? 'block' : 'none';
  el('wcCross').textContent = 'Crosshair ' + (on ? 'on' : 'off');
}
function view(mode){
  el('vtab-live').classList.toggle('on', mode === 'live');
  el('vtab-wrist').classList.toggle('on', mode === 'wrist');
  el('vtab-camera').classList.toggle('on', mode === 'camera');
  el('pane-live').style.display = mode === 'live' ? 'block' : 'none';
  el('pane-wrist').style.display = mode === 'wrist' ? 'block' : 'none';
  el('pane-camera').style.display = mode === 'camera' ? 'block' : 'none';
  if (mode === 'camera' && !camState) camLoad();
  showCross(crossOn);
}
el('vtab-live').onclick = () => view('live');
el('vtab-wrist').onclick = () => view('wrist');
el('vtab-camera').onclick = () => view('camera');
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

// ----------------------------------------------------------------- camera tab
// Ranges, menus and the inactive flags all come from the device, so the tab
// cannot offer a value the driver would refuse.  After every write the panel
// redraws from what the device reported, never from what was requested.
let camState = null;
let camRows = {};
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
  let read;
  if (control.type === 'bool'){
    const box = document.createElement('input');
    box.type = 'checkbox';
    box.checked = !!control.value;
    box.disabled = control.inactive;
    box.onchange = () => { shown.textContent = box.checked ? 'on' : 'off'; };
    line.append(box);
    read = () => (box.checked ? 1 : 0);
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
    sel.onchange = () => { shown.textContent = sel.value; };
    line.append(sel);
    read = () => parseInt(sel.value, 10);
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
    range.oninput = () => { num.value = range.value; shown.textContent = range.value; };
    num.oninput = () => {
      if (num.value !== '') range.value = num.value;
      shown.textContent = num.value;
    };
    line.append(range, num);
    read = () => parseInt(num.value, 10);
    shown.textContent = control.value;
  }
  row.append(line);
  const hint = document.createElement('div');
  hint.className = 'hint';
  hint.style.cssText = 'font-size:10px;color:#5f7b96';
  hint.textContent = (control.type === 'bool'
      ? 'bool · default ' + (control.default ? 'on' : 'off')
      : 'range ' + control.min + '…' + control.max +
        ' · step ' + (control.step || 1) + ' · default ' + control.default)
    + (control.inactive ? ' · inactive while its auto is on' : '');
  row.append(hint);
  camRows[control.name] = {read: read};
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
  // in 2s).  A difference inside two steps is the device rounding, not a
  // value that failed to take, so the two are reported separately.
  const bad = [], rounded = [];
  for (const [k, v] of Object.entries(d.mismatched || {})){
    const control = camFind(k);
    const step = (control && control.step) || 1;
    const text = k + ' ' + v.requested + '→' + v.read_back;
    (Math.abs(v.read_back - v.requested) <= 2 * step ? rounded : bad).push(text);
  }
  const skipped = Object.entries(d.skipped || {}).map(([k, why]) => k + ' (' + why + ')');
  el('camStatus').textContent = 'read back: ' + (read.join(' · ') || 'nothing')
    + (rounded.length ? ' · rounded by the device: ' + rounded.join(', ') : '')
    + (bad.length ? ' · did not take: ' + bad.join(', ') : '')
    + (skipped.length ? ' · skipped: ' + skipped.join(', ') : '')
    + (saved ? ' · saved as startup' : '');
  log('camera ' + read.join(' '));
}
async function camApply(save){
  if (!camState) return;
  el('camStatus').textContent = 'applying…';
  const r = await post('/camera/controls', {controls: camGather(), save: !!save});
  const d = await r.json().catch(() => ({}));
  if (!r.ok){
    el('camStatus').textContent = 'apply failed: ' + (d.detail || r.status);
    return;
  }
  camState = {device: d.device, controls: d.controls,
              saved_at: d.saved_at || camState.saved_at};
  camRender();
  camReport(d, !!save);
}
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


@app.get("/focus")
def focus_page():
    # The focus helper is no longer a separate page: score, joints, moves and
    # captures all live on one dashboard.
    return RedirectResponse("/", status_code=307)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(live.config.get("port", 8080)),
                log_level="info")
