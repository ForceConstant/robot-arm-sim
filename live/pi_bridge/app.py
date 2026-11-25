#!/usr/bin/env python3
"""so101-bridge — one owner of the SO-101 follower, exactly two modes.

Why this exists: LeLab can *stop* teleoperation and release both serial ports,
but it has no joint-target endpoint.  Phase 11d needs the BC-250 to command
joints, so something on the Pi has to own the follower bus.

Modes (exactly one owns the bus, and both are entered on request):

* REMOTE — the bridge owns the follower and applies bounded joint commands.
* TELEOP — the bridge reads the leader and mirrors it onto the follower.

There is deliberately no third state.  HOLD was a room you could walk into and
have to be let out of, and FAULT was a latch that only a power cycle (or a
`/arm` that itself needed REMOTE) could clear; both are gone.  Anything that
used to latch — a bus exception, an e-stop, a leader that is far from the
follower — is now a *reported status* that still accepts the next command:

* ``error``  the last worker/bus exception, cleared by the next accepted
  command or successful mode entry.  The worker keeps running and keeps
  retrying.
* ``estop``  "estop requested" until a command or mode request clears it; the
  arm holds its pose meanwhile, and nothing is latched.
* ``click_in`` per joint, which TELEOP joints are following the leader yet.

Safety properties that are deliberate, not incidental:

* Every target — command or leader — is clamped to the calibrated envelope and
  to a per-cycle step budget, so nothing jumps.
* Commands expire (``ttl_ms``).  A stale target holds the last accepted pose; it
  never queues and never replays after a reconnect.  A dead client therefore
  fails safe to a stop.
* TELEOP takes control immediately: no armed handshake, no leader/follower
  alignment gate.  Per-joint click-in does the work instead — every joint that
  is already close follows at once, and a joint that is far out holds its pose
  and starts following as soon as the leader is brought near it.
* Sequence numbers are advisory.  Freshness is arrival time, so a client whose
  counter restarts (or wrapped) is never wedged out of the bridge.
* The token still gates every mutating endpoint.
* Losing the camera is not a motion fault; losing the bus is a report.

Run: ``python app.py`` (systemd unit ``so101-bridge.service``).
"""
from __future__ import annotations

import asyncio
import base64
from contextlib import asynccontextmanager
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import Response, StreamingResponse

from protocol import (JOINTS, PROTOCOL_VERSION, Command, ProtocolError,
                      apply_target, limits_from_calibration, parse_command)

LELAB = os.environ.get("SO101_LELAB_URL", "http://127.0.0.1:8000")
PORTS_DIR = Path(os.environ.get("SO101_PORTS_DIR",
                                str(Path.home() / ".cache/huggingface/lerobot/ports")))
CAL_DIR = Path(os.environ.get("SO101_CAL_DIR",
                              str(Path.home() / ".cache/huggingface/lerobot/calibration")))
TOKEN_FILE = Path(os.environ.get("SO101_TOKEN_FILE", "/home/pi/so101-bridge/token.txt"))
CAM_INDEX = int(os.environ.get("SO101_CAM_INDEX", "0"))
# A fixed index is not stable across reboots: after 2026-11-17 the C270 came
# back as /dev/video1 and the bridge reported "camera read failed" with the
# camera sitting right there.  Prefer a by-id path when one is configured.
CAM_DEVICE = os.environ.get("SO101_CAM_DEVICE", "").strip()
# The set of V4L2 controls the Camera tab last saved.  A UVC driver forgets
# every control when the device is reopened, and this supervisor closes it a
# few seconds after the last viewer, so a hand-tuned image would silently
# revert to auto between sessions.  This file is the profile that makes a
# tuning session stick; it is re-applied on every open.
CAM_CONTROLS_FILE = Path(os.environ.get(
    "SO101_CAM_CONTROLS_FILE",
    str(Path(__file__).with_name("camera_controls.json"))))
CAM_W, CAM_H, CAM_FPS = 640, 480, 15
LOOP_HZ = 30.0
MAX_JOINT_SPEED_DEG_S = float(os.environ.get("SO101_MAX_SPEED", "35"))
MAX_LAG_DEG = float(os.environ.get("SO101_MAX_LAG", "12"))
# A TELEOP joint starts following the leader once leader and follower are within
# this many degrees.  Joints further away hold while the rest follow, so one
# misaligned wrist never refuses the whole handover.
CLICK_IN_DEG = float(os.environ.get("SO101_CLICK_IN_DEG", "10"))
TELEOP_SPEED_DEG_S = float(os.environ.get(
    "SO101_TELEOP_SPEED", str(MAX_JOINT_SPEED_DEG_S)))
READ_EVERY = 3

MOTOR = {"shoulder_pan": "shoulder_pan", "shoulder_lift": "shoulder_lift",
         "elbow_flex": "elbow_flex", "wrist_flex": "wrist_flex",
         "wrist_roll": "wrist_roll", "gripper": "gripper"}


def _log(message: str) -> None:
    print(f"{time.strftime('%Y-%m-%dT%H:%M:%S')} {message}", flush=True)


def _get(url: str, timeout: float = 3.0) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode())


def _post(url: str, payload: dict | None = None, timeout: float = 5.0) -> dict:
    body = json.dumps(payload or {}).encode()
    request = urllib.request.Request(url, data=body,
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode() or "{}")


def _read_text(path: Path) -> str:
    return path.read_text().strip()


class Bridge:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        # Two states only.  REMOTE is the boot state: the bridge owns the
        # follower and holds its pose until a command arrives, which is what a
        # dead client should get.  TELEOP is entered on request, immediately.
        self.mode = "REMOTE"
        self.desired: str | None = None
        self._entered = False
        self.joints: dict[str, float] = {}
        self.joints_ts = 0.0
        self.joints_source = "none"
        self.follower = None
        self.leader = None
        self.hold: dict[str, float] | None = None
        self.command: Command | None = None
        self.command_ts = 0.0
        self.last_seq = -1
        self.last_seq_ts = 0.0
        self.commands_applied = 0
        self.commands_rejected = 0
        self.commands_accepted = 0
        self.seq_resets = 0
        self.last_clamped: tuple[str, ...] = ()
        self.teleop_engaged: set[str] = set()
        self.click_in: tuple[str, ...] = ()
        self.estop = False
        self.limits: dict[str, tuple[float, float]] = {}
        self.follower_port = ""
        self.leader_port = ""
        self.follower_cal = ""
        self.leader_cal = ""
        self.lelab_active = False
        self.error: str | None = None
        self.error_source: str | None = None
        self.error_at = 0.0
        self.transition_error: str | None = None
        self.reopen_after = 0.0
        self._ticks = 0
        self.started = time.time()
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None
        self.calib_request: dict | None = None
        self.calib_result: dict | None = None
        self.calib_event = threading.Event()

    # ---------------------------------------------------------------- config
    def load(self) -> None:
        self.follower_port = _read_text(PORTS_DIR / "follower_port.txt")
        self.leader_port = _read_text(PORTS_DIR / "leader_port.txt")
        follower_dir = CAL_DIR / "robots" / "so_follower"
        leader_dir = CAL_DIR / "teleoperators" / "so_leader"
        self.follower_cal = self._pick_config(follower_dir)
        self.leader_cal = self._pick_config(leader_dir)
        data = json.loads((follower_dir / f"{self.follower_cal}.json").read_text())
        self.limits = limits_from_calibration(
            data, margin_by_joint={"wrist_roll": 0.0})
        _log(f"config follower={self.follower_port} leader={self.leader_port} "
             f"cal={self.follower_cal}/{self.leader_cal}")

    @staticmethod
    def _pick_config(directory: Path) -> str:
        preferred = directory / "so-arm-101.json"
        if preferred.exists():
            return preferred.stem
        candidates = sorted(p.stem for p in directory.glob("*.json"))
        if not candidates:
            raise RuntimeError(f"no calibration in {directory}")
        return candidates[0]

    # ---------------------------------------------------------------- worker
    def start(self) -> None:
        self.thread = threading.Thread(target=self._loop, name="bridge-worker", daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        period = 1.0 / LOOP_HZ
        while not self.stop.is_set():
            started = time.time()
            self._step(started)
            time.sleep(max(0.0, period - (time.time() - started)))

    def _step(self, now: float) -> None:
        """One worker cycle.  Separated from ``_loop`` so a test can drive it.

        Nothing here can latch: every exception is recorded as a reported error
        and the next cycle (or the next request) proceeds from the same state.
        """
        try:
            if not self._entered:
                # Run the normal entry path once so the follower bus is open and
                # the hold pose is recorded, exactly as a /mode REMOTE would.
                self._entered = True
                self._transition(self.mode)
            with self.lock:
                desired = self.desired
            if self.calib_request is not None:
                self._process_calib_request()
            if desired and desired != self.mode:
                self._transition(desired)
            # Self-heal a lost bus: a replugged or reset follower is reopened
            # automatically instead of needing a process restart or a mode
            # round-trip.
            if self.follower is None and now >= self.reopen_after:
                try:
                    self._open_follower()
                except Exception as exc:  # noqa: BLE001
                    self.reopen_after = now + 3.0
                    self._note_error(f"follower reopen failed: {exc!r}",
                                     source="follower")
            if self.mode == "REMOTE":
                self._remote_tick(now)
            else:
                # TELEOP is entered on request, so a leader that was unplugged
                # mid-session must come back on its own rather than needing a
                # mode round-trip the operator may not know about.
                if self.leader is None and now >= self.reopen_after:
                    try:
                        self._open_leader()
                    except Exception as exc:  # noqa: BLE001
                        self.reopen_after = now + 3.0
                        self._note_error(f"leader reopen failed: {exc!r}",
                                         source="leader")
                self._teleop_tick()
            self._ticks += 1
            if self.follower is not None and self._ticks % READ_EVERY == 0:
                self._read_follower()
        except Exception as exc:  # noqa: BLE001 - the worker must not die
            _log(f"worker error: {exc!r}")
            self._note_error(f"{type(exc).__name__}: {exc}")
            self.reopen_after = now + 1.0

    def _transition(self, target: str) -> None:
        with self.lock:
            self.transition_error = None
        try:
            if target == "REMOTE":
                self._enter_remote()
            elif target == "TELEOP":
                self._enter_teleop()
            else:
                raise ValueError(f"unknown mode {target!r}")
            with self.lock:
                self.error = None
                self.error_source = None
                self.error_at = 0.0
                self.desired = None
        except Exception as exc:  # noqa: BLE001
            # Reported, not latched: the current mode and the bus stay as they
            # are and the next request is tried again.  There is no state a
            # command cannot leave.
            _log(f"transition to {target} failed: {exc!r}")
            self._note_error(str(exc), source="transition")
            with self.lock:
                self.transition_error = str(exc)
                self.desired = None

    def _note_error(self, message: str, *, source: str = "worker") -> None:
        with self.lock:
            self.error = message
            self.error_source = source
            self.error_at = time.time()

    def _clear_error(self, source: str) -> None:
        """Clear a report only when the thing that failed now succeeds.

        Otherwise a healthy follower read would erase a dead-leader report and
        the operator would lose the one clue about what is actually wrong.
        """
        with self.lock:
            if self.error_source == source:
                self.error = None
                self.error_source = None
                self.error_at = 0.0

    def switch_mode(self, target: str, timeout: float = 2.5) -> dict:
        """Request a mode and wait briefly for the worker to finish.

        The transition must run on the worker thread because that thread owns
        every serial port; an HTTP thread that opened the bus would race it.  So
        this posts ``desired`` and waits, which makes the response honest in the
        two cases that matter:

        * a transition that *failed* returns its error (the caller maps it to
          409 and the old mode is intact), and
        * a transition still in progress returns ``pending: true`` rather than a
          false failure.  Opening a servo bus can take seconds, and the caller
          (the :8080 console, with a 6 s request timeout) must not be told the
          handover failed when it is merely still opening ports.
        """
        with self.lock:
            self.estop = False
            self.error = None
            self.error_source = None
            self.error_at = 0.0
            self.transition_error = None
            if self.mode == target and self.desired is None:
                return {"requested": target, "mode": self.mode, "changed": False,
                        "pending": False}
            self.desired = target
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if self.desired is None:
                    if self.mode == target:
                        return {"requested": target, "mode": self.mode,
                                "changed": True, "pending": False}
                    return {"requested": target, "mode": self.mode,
                            "pending": False,
                            "error": self.transition_error
                            or self.error or f"could not enter {target}"}
            time.sleep(0.02)
        with self.lock:
            # Leave ``desired`` set: the worker keeps working on it and the
            # caller can poll /state.  No latch either way.
            return {"requested": target, "mode": self.mode, "changed": False,
                    "pending": True}

    # ------------------------------------------------------------- transitions
    def _enter_remote(self) -> None:
        self._stop_lelab_teleop()
        self._close_leader()
        self._open_follower()
        with self.lock:
            self.hold = dict(self.joints) if self.joints else None
            self.command = None
            self.command_ts = 0.0
            self.teleop_engaged = set()
            self.click_in = ()
            self.mode = "REMOTE"
        _log("mode REMOTE")

    def _enter_teleop(self) -> None:
        """Take control now.  No armed handshake, no alignment gate.

        The bridge mirrors the leader itself (rather than delegating to LeLab)
        because per-joint click-in is the handover rule: every joint already
        near the follower follows at once, and a joint that is far out holds its
        pose and joins in as soon as the leader brings it within
        ``CLICK_IN_DEG``.  A single misaligned wrist cannot refuse the handover.
        """
        self._stop_lelab_teleop()
        self._open_follower()
        self._read_follower()
        leader = self._open_leader()
        goal = self._read_leader_action(leader)
        with self.lock:
            here = dict(self.joints)
            engaged = {name for name in JOINTS
                       if name in here and name in goal
                       and abs(goal[name] - here[name]) <= CLICK_IN_DEG}
            self.teleop_engaged = engaged
            self.click_in = tuple(name for name in JOINTS if name not in engaged)
            self.hold = here or None
            self.command = None
            self.command_ts = 0.0
            self.mode = "TELEOP"
        _log(f"mode TELEOP engaged={sorted(engaged)} "
             f"click-in={list(self.click_in)}")

    def _stop_lelab_teleop(self) -> None:
        try:
            status = _get(f"{LELAB}/teleoperation-status")
        except urllib.error.URLError:
            with self.lock:
                self.lelab_active = False
            return
        if not status.get("teleoperation_active"):
            with self.lock:
                self.lelab_active = False
            return
        _log("stopping LeLab teleoperation")
        try:
            _post(f"{LELAB}/stop-teleoperation")
        except urllib.error.URLError as exc:
            raise RuntimeError(f"LeLab stop failed: {exc}") from exc
        deadline = time.time() + 8.0
        while time.time() < deadline:
            try:
                if not _get(f"{LELAB}/teleoperation-status").get("teleoperation_active"):
                    with self.lock:
                        self.lelab_active = False
                    time.sleep(0.4)  # let the worker finish disconnecting the bus
                    return
            except urllib.error.URLError:
                pass
            time.sleep(0.2)
        raise RuntimeError("LeLab did not release teleoperation within 8 s")

    # ------------------------------------------------------------- follower IO
    def _open_follower(self) -> None:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        if self.follower is not None:
            return
        # Position gains are written to all six servos on every bus open by
        # SO101Follower.configure(), so this config - not the servo EEPROM - is
        # the source of truth.  LeRobot 0.6.1 ships P=16, I=0, D=32; the Feetech
        # STS3215 factory default for the position loop is P=32, D=32, I=0.
        # With I=0 the loop cannot null a constant load, so the steady-state
        # shortfall is residual = load/Kp: measured 2026-11-25 at P=16,
        # shoulder_pan +0.33/-0.24 deg, loaded elbow_flex +0.50/-1.56 deg, with
        # tail_spread 0.000 (settled, not mid-flight).  This step is the
        # single-variable experiment: restore the factory P and nothing else.
        robot = SO101Follower(SO101FollowerConfig(port=self.follower_port,
                                                  id=self.follower_cal,
                                                  position_p_coefficient=32,
                                                  position_i_coefficient=0,
                                                  position_d_coefficient=32))
        try:
            robot.bus.connect()
            robot.bus.write_calibration(robot.calibration)
            robot.configure()
            # Torque is a RAM register and LeRobot's disconnect() turns it OFF.
            # Every mode cycle, service restart or bus reopen therefore used to
            # leave the follower limp while /command kept answering
            # "accepted": the arm reported positions, accepted goals, and never
            # moved (measured 2026-11-24: elbow achieved 0.18 deg of 3-4 deg;
            # all six servos read Torque_Enable=0 with a healthy 12.2 V supply).
            # Enabling it here -- after configure(), which is what resets the
            # bus -- is what makes a reopened follower actually actuate.
            try:
                robot.bus.enable_torque()
            except Exception as exc:  # noqa: BLE001 - never fail the open for this
                _log(f"torque enable failed: {exc!r}")
            else:
                _log("follower torque enabled")
        except Exception:
            try:
                robot.disconnect()
            except Exception:  # noqa: BLE001
                pass
            raise
        with self.lock:
            self.follower = robot
        self._read_follower()
        _log(f"follower open at {self.follower_port}")

    def _close_follower(self) -> None:
        robot = self.follower
        with self.lock:
            self.follower = None
        if robot is None:
            return
        try:
            robot.disconnect()
        except Exception as exc:  # noqa: BLE001
            _log(f"follower disconnect: {exc!r}")
        _log("follower closed")

    def _read_follower(self) -> None:
        robot = self.follower
        if robot is None:
            return
        observation = robot.get_observation()
        values = {name: float(observation[f"{motor}.pos"])
                  for name, motor in MOTOR.items()}
        with self.lock:
            self.joints = values
            self.joints_ts = time.time()
            self.joints_source = "follower"
        # The bus answered, so a reported bus error is no longer true.
        self._clear_error("follower")

    # ---------------------------------------------------------- calibration
    def request_wrist_zero(self, span_deg: float = 90.0, dry_run: bool = False,
                           timeout: float = 20.0) -> dict:
        """Ask the worker to make the current wrist_roll pose the servo zero.

        The worker owns all serial I/O, so the HTTP thread only posts a request
        and waits.  A failed write returns an error; it does not latch FAULT,
        because the arm is still usable and the old calibration is intact.
        """
        with self.lock:
            if self.mode != "REMOTE":
                raise ProtocolError("bridge must be REMOTE to write the servo zero")
            if self.follower is None:
                raise ProtocolError("follower is not open")
            if self.calib_request is not None:
                raise ProtocolError("a calibration write is already running")
            self.calib_request = {"span_deg": float(span_deg),
                                  "dry_run": bool(dry_run)}
            self.calib_result = None
            self.calib_event.clear()
        if not self.calib_event.wait(timeout):
            with self.lock:
                self.calib_request = None
            raise ProtocolError("calibration write timed out")
        with self.lock:
            result = self.calib_result
            self.calib_result = None
        if not result:
            raise ProtocolError("calibration write returned no result")
        if result.get("error"):
            raise ProtocolError(result["error"])
        return result

    @staticmethod
    def _cal_entry(cal) -> dict:
        return {"id": int(cal.id), "drive_mode": int(cal.drive_mode),
                "homing_offset": int(cal.homing_offset),
                "range_min": int(cal.range_min),
                "range_max": int(cal.range_max)}

    def _process_calib_request(self) -> None:
        request = self.calib_request
        if request is None:
            return
        result: dict = {}
        robot = self.follower
        torque_off = False
        try:
            if self.mode != "REMOTE":
                raise RuntimeError("bridge left REMOTE before the write")
            if robot is None:
                raise RuntimeError("follower is not open")
            span_deg = float(request.get("span_deg", 90.0))
            dry_run = bool(request.get("dry_run", False))
            if not 5.0 <= span_deg <= 170.0:
                raise RuntimeError("span_deg must be within 5..170")
            bus = robot.bus
            name = "wrist_roll"
            cal = robot.calibration[name]
            if not dry_run:
                # EEPROM registers are write-protected while torque is enabled.
                bus.disable_torque(name)
                torque_off = True
                time.sleep(0.15)
            raw_before = int(bus.read("Present_Position", name, normalize=False))
            old_offset = int(cal.homing_offset)
            # Present_Position = Actual_Position - Homing_Offset.  Shift the
            # offset so this pose reads exactly 2048, then centre the firmware
            # limits on 2048.  Protocol 1 stores homing elsewhere; refuse rather
            # than write a register that version treats as inert.
            if int(bus.protocol_version) != 0:
                raise RuntimeError("servo protocol is not 0; unsupported homing write")
            new_offset = old_offset + raw_before - 2048
            span_counts = int(round(span_deg / 360.0 * 4095.0))
            low = 2048 - span_counts
            high = 2048 + span_counts
            if low < 0 or high > 4095:
                raise RuntimeError("requested span does not fit the encoder")
            if dry_run:
                result = {"ok": True, "dry_run": True, "joint": name,
                          "raw_before": raw_before,
                          "old_homing_offset": old_offset,
                          "homing_offset": new_offset,
                          "range_min": low, "range_max": high,
                          "span_deg": span_deg}
                return
            bus.write("Homing_Offset", name, new_offset, normalize=False)
            bus.write("Min_Position_Limit", name, low, normalize=False)
            bus.write("Max_Position_Limit", name, high, normalize=False)
            cal.homing_offset = new_offset
            cal.range_min = low
            cal.range_max = high
            robot.calibration[name] = cal
            bus.calibration[name] = cal
            data = {key: self._cal_entry(value)
                    for key, value in robot.calibration.items()}
            path = CAL_DIR / "robots" / "so_follower" / f"{self.follower_cal}.json"
            if path.exists():
                backup = path.with_name(
                    path.name + ".bak-" + time.strftime("%Y%m%d-%H%M%S"))
                shutil.copy2(path, backup)
            path.write_text(json.dumps(data, indent=4) + "\n")
            self.limits = limits_from_calibration(
                data, margin_by_joint={"wrist_roll": 0.0})
            # The degrees just changed frame: re-read before the next hold tick
            # so the bridge does not command the old angle in the new frame.
            self._read_follower()
            with self.lock:
                self.hold = dict(self.joints)
                self.command = None
                self.command_ts = 0.0
            result = {"ok": True, "joint": name,
                      "raw_before": raw_before,
                      "old_homing_offset": old_offset,
                      "homing_offset": new_offset,
                      "range_min": low, "range_max": high,
                      "span_deg": span_deg, "path": str(path),
                      "limits": {k: [round(v[0], 2), round(v[1], 2)]
                                 for k, v in self.limits.items()}}
        except Exception as exc:  # noqa: BLE001 - report, never kill the worker
            result = {"error": f"{type(exc).__name__}: {exc}"}
        finally:
            if torque_off and robot is not None:
                try:
                    robot.bus.enable_torque("wrist_roll")
                except Exception as exc:  # noqa: BLE001
                    _log(f"calibration re-enable torque failed: {exc!r}")
            with self.lock:
                self.calib_result = result
                self.calib_request = None
            self.calib_event.set()

    # --------------------------------------------------------------- leader IO
    def _open_leader(self):
        """Open the leader bus.  TELEOP mirrors it here, not through LeLab.

        LeLab's ``/move-arm`` starts a mirror the moment it connects, which is
        exactly what per-joint click-in must not do.  Owning the leader here also
        removes a whole remote service from the handover path.
        """
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

        if self.leader is not None:
            return self.leader
        leader = SO101Leader(SO101LeaderConfig(port=self.leader_port,
                                               id=self.leader_cal))
        try:
            leader.bus.connect()
            leader.bus.write_calibration(leader.calibration)
            leader.configure()
        except Exception:
            try:
                leader.disconnect()
            except Exception:  # noqa: BLE001
                pass
            raise
        with self.lock:
            self.leader = leader
        _log(f"leader open at {self.leader_port}")
        return leader

    def _close_leader(self) -> None:
        leader = self.leader
        with self.lock:
            self.leader = None
            self.teleop_engaged = set()
            self.click_in = ()
        if leader is None:
            return
        try:
            leader.disconnect()
        except Exception as exc:  # noqa: BLE001
            _log(f"leader disconnect: {exc!r}")
        _log("leader closed")

    @staticmethod
    def _read_leader_action(leader) -> dict[str, float]:
        action = None
        for _ in range(5):
            action = leader.get_action()
            if action:
                break
            time.sleep(0.05)
        if not action:
            raise RuntimeError("leader returned no action")
        return {name: float(action[f"{motor}.pos"])
                for name, motor in MOTOR.items()}

    @staticmethod
    def _action(values: dict[str, float]) -> dict[str, float]:
        """Joint-name degrees -> lerobot motor keys."""
        return {f"{MOTOR[name]}.pos": float(value) for name, value in values.items()}

    def _hold_tick(self) -> None:
        if self.hold is None or self.follower is None:
            return
        self.follower.send_action(self._action(self.hold))

    def _teleop_tick(self) -> None:
        """Mirror the leader with per-joint click-in.

        Every joint within ``CLICK_IN_DEG`` of the follower follows immediately;
        a joint further out holds the follower where it is and joins in on the
        first cycle the leader brings it close enough.  Nothing refuses the
        handover and nothing latches.
        """
        robot = self.follower
        leader = self.leader
        if robot is None or leader is None:
            return
        try:
            goal = self._read_leader_action(leader)
        except Exception as exc:  # noqa: BLE001 - a bad read holds, never latches
            self._note_error(f"leader read failed: {exc!r}", source="leader")
            self._hold_tick()
            return
        self._clear_error("leader")
        with self.lock:
            current = dict(self.joints)
            anchor = dict(self.hold) if self.hold else dict(current)
            engaged = set(self.teleop_engaged)
            estop = self.estop
        if not current or estop:
            self._hold_tick()
            return
        for name in JOINTS:
            if name not in engaged and abs(goal[name] - current[name]) <= CLICK_IN_DEG:
                engaged.add(name)
        # Only the engaged joints carry a target; the rest are pinned to the
        # measured pose so a single misaligned joint cannot drag the arm.
        targets = tuple(goal[name] if name in engaged else current[name]
                        for name in JOINTS)
        acceptance = apply_target(Command(seq=0, ttl_ms=0, joints=targets,
                                          speed_scale=1.0),
                                  anchor, self.limits,
                                  dt_s=1.0 / LOOP_HZ,
                                  max_joint_speed_deg_s=TELEOP_SPEED_DEG_S)
        target = dict(acceptance.joints)
        lagged = []
        for name, value in target.items():
            # Same lead guard as REMOTE: the goal may run at most MAX_LAG_DEG
            # ahead of what the servos actually report, so a fast leader swing
            # cannot wind up a large unexecuted backlog.
            measured = current[name]
            bounded = min(max(value, measured - MAX_LAG_DEG),
                          measured + MAX_LAG_DEG)
            if abs(bounded - value) > 0.01:
                lagged.append(name)
            target[name] = bounded
        robot.send_action(self._action(target))
        with self.lock:
            self.teleop_engaged = engaged
            self.click_in = tuple(name for name in JOINTS if name not in engaged)
            self.hold = target
            self.last_clamped = tuple(acceptance.clamped) + tuple(
                name for name in lagged if name not in acceptance.clamped)
            self.commands_applied += 1

    def _remote_tick(self, now: float) -> None:
        robot = self.follower
        if robot is None:
            return
        with self.lock:
            command, stamp = self.command, self.command_ts
            current = dict(self.joints)
            anchor = dict(self.hold) if self.hold else dict(current)
        if not current:
            return
        fresh = command is not None and (now - stamp) * 1000.0 <= command.ttl_ms
        if not fresh:
            self._hold_tick()
            return
        # The step budget accumulates against the last COMMANDED goal, not the
        # measured position: a servo ignores sub-deadband steps, so anchoring on
        # measurement can stall a 0.2 deg/cycle ramp forever.
        acceptance = apply_target(command, anchor, self.limits,
                                  dt_s=1.0 / LOOP_HZ,
                                  max_joint_speed_deg_s=MAX_JOINT_SPEED_DEG_S)
        target = dict(acceptance.joints)
        lagged = []
        for name, value in target.items():
            measured = current[name]
            bounded = min(max(value, measured - MAX_LAG_DEG),
                          measured + MAX_LAG_DEG)
            if abs(bounded - value) > 0.01:
                lagged.append(name)
            target[name] = bounded
        robot.send_action(self._action(target))
        with self.lock:
            self.hold = target
            self.last_clamped = tuple(acceptance.clamped) + tuple(
                name for name in lagged if name not in acceptance.clamped)
            self.commands_applied += 1

    # -------------------------------------------------------------- commands
    def submit(self, command: Command) -> dict:
        """Accept a joint target.  Freshness is arrival time, not ``seq``.

        ``seq`` used to have to be strictly newer than the last accepted value,
        which permanently wedged any client whose counter fell behind — a
        second console, or the same console after a restart, 409'd forever on a
        gap of millions.  Now any ``seq`` is accepted: a lower value is merely
        reported as a reset and the command still applies.  Expiry (``ttl_ms``)
        is what keeps a dead client from holding a stale target, and it is
        measured from arrival, so no counter can wedge the bridge.
        """
        with self.lock:
            if self.mode != "REMOTE":
                raise ProtocolError(
                    f"mode is {self.mode}, not REMOTE; POST /mode {{mode:REMOTE}}")
            seq_reset = command.seq < self.last_seq
            if seq_reset:
                self.seq_resets += 1
            self.last_seq = max(self.last_seq, command.seq)
            self.last_seq_ts = time.time()
            self.command = command
            self.command_ts = time.time()
            self.commands_accepted += 1
            # A command is an unambiguous "we are driving this" statement, so it
            # clears a reported error or a standing e-stop.  Nothing latches.
            self.error = None
            self.error_source = None
            self.error_at = 0.0
            self.estop = False
        return {"accepted": True, "seq": command.seq,
                "ttl_ms": command.ttl_ms, "seq_reset": seq_reset}

    def arm(self) -> dict:
        """Kept for API compatibility; nothing needs arming any more.

        Both modes are entered on request and both accept commands, so this only
        confirms the follower pose is known and clears a reported error/e-stop.
        It never fails for "already armed" or "wrong mode".
        """
        with self.lock:
            if not self.limits or not self.joints:
                raise ProtocolError("follower pose unknown; enter REMOTE first")
            self.error = None
            self.error_source = None
            self.error_at = 0.0
            self.estop = False
        return {"armed": True, "mode": self.mode}

    def estop_request(self) -> dict:
        """Stop driving and hold the pose.  Not a latch: a command resumes.

        The owner secures the arm by powering it off, so a software e-stop that
        could only be cleared by a power cycle made the arm unusable for no
        safety gain.  This holds the last pose and reports ``estop: true`` until
        a command or a mode request clears it.
        """
        with self.lock:
            self.estop = True
            self.command = None
            self.command_ts = 0.0
            if self.follower is not None and self.joints:
                self.hold = dict(self.joints)
            mode = self.mode
        _log("estop: holding the pose (no latch)")
        return {"mode": mode, "estop": True, "latched": False}

    def state(self) -> dict:
        with self.lock:
            return {
                "protocol": PROTOCOL_VERSION,
                "mode": self.mode,
                "desired": self.desired,
                # There is no armed/disarmed distinction any more: both modes
                # accept their next request immediately.  The key stays so the
                # console and the scan controller keep working unchanged; it
                # now means "the bridge is in a mode that takes input".
                "armed": True,
                # There is also no FAULT latch.  A condition is reported here and
                # the bridge keeps accepting commands; ``fault`` stays null so a
                # stale consumer cannot mistake a report for a latch.
                "fault": None,
                "error": self.error,
                "error_age_s": (round(time.time() - self.error_at, 1)
                                if self.error_at else None),
                "error_source": self.error_source,
                "estop": self.estop,
                "joints": {name: round(self.joints.get(name, 0.0), 2) for name in JOINTS},
                "joints_age_s": round(time.time() - self.joints_ts, 3) if self.joints_ts else None,
                "joints_source": self.joints_source,
                "last_seq": self.last_seq,
                "last_seq_age_s": (round(time.time() - self.last_seq_ts, 3)
                                   if self.last_seq_ts else None),
                "seq_resets": self.seq_resets,
                "commands_accepted": self.commands_accepted,
                "commands_applied": self.commands_applied,
                "commands_rejected": self.commands_rejected,
                "last_clamped": list(self.last_clamped),
                "teleop_engaged": [name for name in JOINTS
                                   if name in self.teleop_engaged],
                "click_in": list(self.click_in),
                "lelab_active": self.lelab_active,
                "follower_open": self.follower is not None,
                "leader_open": self.leader is not None,
                "limits": {name: [round(low, 2), round(high, 2)]
                           for name, (low, high) in self.limits.items()},
                "uptime_s": round(time.time() - self.started, 1),
            }


bridge = Bridge()


class Camera:
    """One supervisor thread owns the device; viewers only attach and read.

    Opening the C270 costs seconds, so the device is kept while anyone is
    watching and released a few seconds after the last viewer leaves.  The
    supervisor model matters: spawning a thread per attach raced the previous
    thread's release and produced "can't open camera by index" intermittently.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.frame: bytes | None = None
        self.frame_ts = 0.0
        self.clients = 0
        self.error: str | None = None
        self.last_used = 0.0
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.thread is None:
            self.thread = threading.Thread(target=self._loop, name="camera", daemon=True)
            self.thread.start()

    def attach(self) -> None:
        with self.cond:
            self.clients += 1
            self.last_used = time.time()

    def detach(self) -> None:
        with self.cond:
            self.clients = max(0, self.clients - 1)
            self.last_used = time.time()

    def wait_frame(self, since: float = 0.0, timeout: float = 6.0):
        deadline = time.time() + timeout
        with self.cond:
            while self.frame is None or self.frame_ts <= since:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self.cond.wait(timeout=min(remaining, 0.2))
            return self.frame, self.frame_ts

    def _apply_controls(self) -> None:
        """V4L2 controls reset when the device is reopened; reapply them.

        The profile saved by the Camera tab wins; ``SO101_CAM_CONTROLS`` is only
        the fallback for a device that has never been saved.  Applying it here,
        on every open, is the whole point: the supervisor releases the device
        when nobody is watching, so a one-off ``v4l2-ctl`` call by hand would be
        gone by the next capture.
        """
        settings = load_saved_controls()
        if not settings:
            text = os.environ.get("SO101_CAM_CONTROLS",
                                  "contrast=64,sharpness=64").strip()
            for item in text.split(","):
                name, sep, value = item.partition("=")
                if sep:
                    try:
                        settings[name.strip()] = int(value)
                    except ValueError:
                        _log(f"camera control {item!r} is not an integer")
        if not settings:
            return
        try:
            applied = apply_camera_controls(settings)
        except HTTPException as exc:
            _log(f"camera controls refused: {exc.detail}")
            return
        except Exception as exc:  # noqa: BLE001 - controls are best-effort
            _log(f"camera controls not applied: {exc!r}")
            return
        _log("camera controls applied on open: "
             + ",".join(f"{name}={value}"
                        for name, value in applied["applied"].items()))

    def _open(self):
        capture = (cv2.VideoCapture(CAM_DEVICE, cv2.CAP_V4L2) if CAM_DEVICE
                   else cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2))
        if not capture.isOpened():
            capture.release()
            return None
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        capture.set(cv2.CAP_PROP_FPS, CAM_FPS)
        self._apply_controls()
        return capture

    def _loop(self) -> None:
        capture = None
        while not self.stop.is_set():
            with self.cond:
                wanted = self.clients > 0
                idle_for = time.time() - self.last_used
            if not wanted and capture is not None and idle_for > 5.0:
                capture.release()
                capture = None
                with self.cond:
                    self.frame = None
                    self.frame_ts = 0.0
            if not wanted:
                time.sleep(0.2)
                continue
            if capture is None:
                capture = self._open()
                if capture is None:
                    with self.cond:
                        self.error = "camera open failed"
                    time.sleep(1.0)
                    continue
            ok, image = capture.read()
            if not ok:
                with self.cond:
                    self.error = "camera read failed"
                time.sleep(0.2)
                continue
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with self.cond:
                    self.frame = encoded.tobytes()
                    self.frame_ts = time.time()
                    self.error = None
                    self.cond.notify_all()
        if capture is not None:
            capture.release()


camera = Camera()


# ------------------------------------------------------------ camera controls
# The wrist image was unusable because auto-exposure and auto-white-balance
# hunted between frames: identical code produced frame means of 83.6 / 40.9 /
# 39.6 and 8 / 4 / 2 detected markers.  Ranges come from the driver, never from
# the UI, so the tab cannot offer a value the device would refuse.

CONTROL_LINE = re.compile(
    r"^\s*([a-z0-9_]+)\s+0x[0-9a-fA-F]+\s+\((int|bool|menu)\)\s*:\s*(.*)$")
MENU_LINE = re.compile(r"^\s*(\d+):\s*(.+?)\s*$")
# An inactive control cannot be written until the switch that owns it is in the
# manual position, so these are always written first.
CONTROL_ORDER = ("auto_exposure", "white_balance_automatic")
CONTROL_SCHEMA = "camera-controls/1"


def _v4l2_device() -> str:
    return CAM_DEVICE or f"/dev/video{CAM_INDEX}"


def _v4l2(*args: str, timeout: float = 6.0) -> str:
    result = subprocess.run(["v4l2-ctl", "-d", _v4l2_device(), *args],
                            capture_output=True, text=True, timeout=timeout)
    if result.returncode != 0:
        raise RuntimeError((result.stderr or result.stdout).strip()
                           or f"v4l2-ctl {' '.join(args)} failed")
    return result.stdout


def read_camera_controls() -> list[dict]:
    """Every control the device advertises, with its live value.

    ``v4l2-ctl --list-ctrls-menus`` prints one line per control followed by the
    numbered items of a menu; the parse keeps the device's own order so the tab
    lists controls the way the driver does.
    """
    controls: list[dict] = []
    current: dict | None = None
    for line in _v4l2("--list-ctrls-menus").splitlines():
        found = CONTROL_LINE.match(line)
        if found:
            name, kind, rest = found.groups()
            entry: dict = {"name": name, "type": kind, "value": None,
                           "min": None, "max": None, "step": None,
                           "default": None, "inactive": False, "menu": []}
            for key in ("min", "max", "step", "default", "value"):
                match = re.search(rf"\b{key}=(-?\d+)", rest)
                if match:
                    entry[key] = int(match.group(1))
            # "flags=inactive" (int/bool) or a bare "(inactive)" (menu).
            entry["inactive"] = "inactive" in rest
            if kind == "bool":
                entry["min"], entry["max"], entry["step"] = 0, 1, 1
            controls.append(entry)
            current = entry
            continue
        if current is not None and current["type"] == "menu":
            item = MENU_LINE.match(line)
            if item:
                current["menu"].append({"value": int(item.group(1)),
                                        "label": item.group(2)})
    return controls


def _coerce(entry: dict, value) -> int:
    """Validate one requested value against the driver's own description."""
    if entry["type"] == "bool":
        if isinstance(value, bool):
            return int(value)
        if value in (0, 1):
            return int(value)
        text = str(value).strip().lower()
        if text in ("true", "on", "yes"):
            return 1
        if text in ("false", "off", "no"):
            return 0
        raise ValueError("expects true or false")
    if isinstance(value, bool):
        raise ValueError("expects an integer, not a boolean")
    if isinstance(value, str):
        try:
            value = int(value.strip())
        except ValueError:
            raise ValueError("expects an integer") from None
    if not isinstance(value, int):
        raise ValueError("expects an integer")
    low, high = entry["min"], entry["max"]
    if low is None or high is None:
        raise ValueError("the device reports no range for this control")
    if not low <= value <= high:
        raise ValueError(f"{value} is outside {low}..{high}")
    step = entry["step"] or 1
    if step > 1 and (value - low) % step:
        raise ValueError(f"{value} is not {low} + n*{step}")
    if entry["type"] == "menu":
        allowed = [item["value"] for item in entry["menu"]]
        if allowed and value not in allowed:
            raise ValueError(f"menu value must be one of {allowed}")
    return value


def apply_camera_controls(requested: dict) -> dict:
    """Validate, write, then read back.  The device answers, not the request.

    Unknown names and out-of-range values are refused before anything is
    written, so a typo cannot half-apply a profile.
    """
    if not isinstance(requested, dict) or not requested:
        raise HTTPException(400, "controls must be a non-empty object")
    known = {control["name"]: control for control in read_camera_controls()}
    clean: dict[str, int] = {}
    errors: list[str] = []
    for name, value in requested.items():
        entry = known.get(str(name))
        if entry is None:
            errors.append(f"{name}: unknown control")
            continue
        try:
            clean[str(name)] = _coerce(entry, value)
        except ValueError as exc:
            errors.append(f"{name}: {exc}")
    if errors:
        raise HTTPException(400, "rejected: " + "; ".join(errors))
    # Two phases.  Switching an auto off is what makes its dependent control
    # writable, and switching one *on* makes it unwritable again, so the
    # enablers go first and the rest are filtered against what the device then
    # reports rather than failing the whole write on one permission error.
    enablers = [name for name in CONTROL_ORDER if name in clean]
    try:
        if enablers:
            _v4l2(*sum((["-c", f"{name}={clean[name]}"]
                        for name in enablers), []))
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        raise HTTPException(502, f"the device refused the write: {exc}") from exc
    now = {control["name"]: control for control in read_camera_controls()}
    skipped: dict[str, str] = {}
    args: list[str] = []
    for name in clean:
        if name in CONTROL_ORDER:
            continue
        entry = now.get(name)
        if entry is not None and entry["inactive"]:
            skipped[name] = "inactive after the auto switch"
            continue
        args += ["-c", f"{name}={clean[name]}"]
    if args:
        try:
            _v4l2(*args)
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            raise HTTPException(502, f"the device refused the write: {exc}") from exc
    after = {control["name"]: control["value"]
             for control in read_camera_controls()}
    applied = {name: after.get(name) for name in clean}
    mismatched = {name: {"requested": clean[name], "read_back": applied[name]}
                  for name in clean
                  if name not in skipped and applied[name] != clean[name]}
    return {"schema": CONTROL_SCHEMA, "device": _v4l2_device(),
            "requested": clean, "applied": applied,
            "mismatched": mismatched, "skipped": skipped,
            "controls": read_camera_controls()}


def load_saved_controls() -> dict:
    """The saved profile, or {} when the device has never been saved."""
    try:
        data = json.loads(CAM_CONTROLS_FILE.read_text())
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as exc:
        _log(f"saved camera controls unreadable: {exc!r}")
        return {}
    controls = data.get("controls")
    return dict(controls) if isinstance(controls, dict) else {}


def save_saved_controls(controls: dict) -> dict:
    payload = {"schema": CONTROL_SCHEMA, "device": _v4l2_device(),
               "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "controls": dict(controls)}
    CAM_CONTROLS_FILE.parent.mkdir(parents=True, exist_ok=True)
    scratch = CAM_CONTROLS_FILE.with_suffix(".tmp")
    scratch.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    scratch.replace(CAM_CONTROLS_FILE)
    return payload


def _authorised(request: Request) -> None:
    token = os.environ.get("SO101_BRIDGE_TOKEN", "")
    if not token and TOKEN_FILE.exists():
        token = TOKEN_FILE.read_text().strip()
    if not token:
        raise HTTPException(503, "bridge token is not configured")
    if request.headers.get("x-bridge-token") != token:
        raise HTTPException(403, "bad bridge token")


@asynccontextmanager
async def lifespan(app: FastAPI):
    bridge.load()
    bridge.start()
    camera.start()
    yield
    bridge.stop.set()
    camera.stop.set()
    bridge._close_follower()
    bridge._close_leader()


app = FastAPI(title="SO-101 bridge", version="1.0.0", lifespan=lifespan,
              docs_url=None, redoc_url=None)


@app.get("/health")
def health():
    with camera.cond:
        camera_state = {"clients": camera.clients,
                        "frame_age_s": round(time.time() - camera.frame_ts, 2)
                        if camera.frame_ts else None,
                        "error": camera.error}
    # "ok" used to mean "not latched in FAULT".  There is no such state now, so
    # it means what it says: the bridge knows the joint envelope and is running.
    return {"ok": bool(bridge.limits),
            "mode": bridge.mode, "estop": bridge.estop,
            "error": bridge.error, "camera": camera_state}


@app.get("/state")
def state():
    return bridge.state()


@app.get("/camera/controls")
def camera_controls_get():
    """Control metadata and live values; the tab renders from this alone."""
    try:
        controls = read_camera_controls()
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        raise HTTPException(502, f"cannot read camera controls: {exc}") from exc
    saved_at = None
    try:
        saved_at = json.loads(CAM_CONTROLS_FILE.read_text()).get("saved_at")
    except (OSError, ValueError):
        saved_at = None
    return {"schema": CONTROL_SCHEMA, "device": _v4l2_device(),
            "controls": controls, "saved": load_saved_controls(),
            "saved_at": saved_at, "saved_file": str(CAM_CONTROLS_FILE)}


@app.post("/camera/controls")
async def camera_controls_post(request: Request):
    """Apply a validated subset, then report the values read back."""
    _authorised(request)
    try:
        body = await request.json()
    except ValueError:
        raise HTTPException(400, "body must be JSON") from None
    result = apply_camera_controls(body.get("controls"))
    if body.get("save"):
        payload = save_saved_controls(result["applied"])
        result["saved"] = payload["controls"]
        result["saved_at"] = payload["saved_at"]
        _log(f"camera profile saved: {CAM_CONTROLS_FILE} "
             f"({len(payload['controls'])} controls)")
    return result


@app.post("/mode")
async def set_mode(request: Request):
    """Switch to REMOTE or TELEOP, taking effect before the response.

    There are two states and no queue: the transition runs here, so a caller
    that gets 200 knows which mode the arm is in and a caller that gets an error
    knows why.  ``HOLD`` is accepted as an alias for REMOTE — it means "stop
    following a leader and hold where you are", which is exactly REMOTE with no
    fresh command, so an old console button can never strand the arm.
    """
    _authorised(request)
    body = await request.json()
    mode = str(body.get("mode", "")).upper()
    if mode == "HOLD":
        mode = "REMOTE"
    if mode not in ("TELEOP", "REMOTE"):
        raise HTTPException(400, "mode must be TELEOP or REMOTE")
    result = await asyncio.to_thread(bridge.switch_mode, mode)
    if result.get("error"):
        raise HTTPException(409, result["error"])
    return result


@app.post("/command")
async def command(request: Request):
    _authorised(request)
    body = await request.json()
    try:
        parsed = parse_command(body)
        return bridge.submit(parsed)
    except ProtocolError as exc:
        with bridge.lock:
            bridge.commands_rejected += 1
        raise HTTPException(409, str(exc)) from exc


@app.post("/arm")
async def arm(request: Request):
    _authorised(request)
    body = await request.json()
    if body.get("confirm") is not True:
        raise HTTPException(400, "confirm must be true")
    try:
        return bridge.arm()
    except ProtocolError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/estop")
async def estop(request: Request):
    """Hold the pose.  Not latched: the next command or /mode resumes.

    The owner secures the arm by switching power off, so a latch that needed a
    power cycle only made the bridge unusable.  See ``Bridge.estop_request``.
    """
    _authorised(request)
    return bridge.estop_request()


@app.post("/calibration/wrist-zero")
async def wrist_zero(request: Request):
    """Make the current wrist_roll pose servo 0 and write ±span limits.

    This is an EEPROM write: it changes Homing_Offset and the firmware
    Min/Max_Position_Limit, then rewrites the LeRobot calibration JSON.  The
    request is executed by the bridge worker, which owns the serial bus.
    """
    _authorised(request)
    body = await request.json()
    try:
        span = float(body.get("span_deg", 90.0))
        dry = bool(body.get("dry_run", False))
        return await asyncio.to_thread(bridge.request_wrist_zero, span, dry)
    except (ProtocolError, ValueError) as exc:
        raise HTTPException(409, str(exc)) from exc


@app.websocket("/joints")
async def joints(socket: WebSocket):
    await socket.accept()
    try:
        while True:
            await socket.send_json(bridge.state())
            await asyncio.sleep(1 / 20)
    except WebSocketDisconnect:
        return


def _mjpeg():
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    camera.attach()
    try:
        last = None
        while True:
            with camera.cond:
                camera.cond.wait(timeout=1.0)
                frame = camera.frame
            if frame is None or frame is last:
                continue
            last = frame
            yield boundary + frame + b"\r\n"
    finally:
        camera.detach()


@app.get("/camera.mjpg")
def camera_stream():
    return StreamingResponse(_mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/capture")
def capture():
    """Frame plus the joints, atomically enough to pair them.

    The JPEG can be one camera period older than the joint snapshot; callers
    must compare timestamps and reject a stale pair rather than assume they were
    sampled at the same instant.
    """
    camera.attach()
    try:
        wanted = camera.frame_ts
        # The C270 needs a couple of seconds to open at 640x480 MJPG on a Pi 4;
        # a 2 s budget fails the first capture after every idle period.
        got = camera.wait_frame(since=wanted, timeout=6.0)
    finally:
        camera.detach()
    if got is None:
        with camera.cond:
            detail = camera.error or "no frame"
        raise HTTPException(503, f"no camera frame within 6 s ({detail})")
    frame, frame_ts = got
    snapshot = bridge.state()
    return {"jpeg_b64": base64.b64encode(frame).decode("ascii"),
            "frame_ts": frame_ts,
            "joints": snapshot["joints"],
            "joints_ts": time.time() - (snapshot["joints_age_s"] or 0.0),
            "mode": snapshot["mode"]}


@app.get("/focus")
def focus():
    """Laplacian variance of the latest frame; use it while turning the lens.

    Higher is sharper.  A defocused C270 at close range sits around 40; a
    focused close-up is several hundred.
    """
    camera.attach()
    try:
        got = camera.wait_frame(timeout=6.0)
    finally:
        camera.detach()
    if got is None:
        raise HTTPException(503, "no camera frame")
    frame, ts = got
    image = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise HTTPException(503, "frame did not decode")
    laplacian = cv2.Laplacian(image, cv2.CV_64F)
    return {"score": round(float(laplacian.var()), 1),
            "frame_age_s": round(time.time() - ts, 3),
            "size": [int(image.shape[1]), int(image.shape[0])]}


@app.get("/camera.jpg")
def camera_frame():
    with camera.cond:
        frame = camera.frame
    if frame is None:
        raise HTTPException(503, "no frame yet; open /camera.mjpg first")
    return Response(frame, media_type="image/jpeg")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("SO101_PORT", "8100")),
                log_level="info")
