#!/usr/bin/env python3
"""64-square aim sweep: aim the RAY at every square, then the tip, same session.

Two measured columns, one pass, one lighting state:

``tip``  ``/move mode=xyz`` puts the **jaw tip** at the square centre at a fixed
         tip height, jaws down.  That is the historical way
         (``phase3_squares_irl.py``) and it is re-measured here rather than
         trusted from an old receipt.
``aim``  ``/move mode=aim_point`` translates the tool until the **aim ray**
         (jaw origin -> tip centre -> out, ``live/aim.py``) meets the square
         centre, then a short closed-loop ``rel`` refinement takes out the
         residual the servo deadband leaves behind.

Why the extra refinement: measured 2026-11-24 on this arm, an ``aim_point``
step of 10 mm leaves a joint delta of ~2.9 deg, which is inside the servos'
3-4 deg directional deadband, so the arm does not move at all (aim moved
0.6 mm).  A 12 mm ``rel`` command (3.49 deg) executes 3.34 deg, i.e. ~95% of
the ask.  The refinement therefore commands ``residual + shortfall`` and
estimates the shortfall from the previous step's observed transfer.  It is a
plain ``/move mode=rel`` request; the live service is not modified.

Measured per leg:

* ``aim_board_mm`` from ``/aim-point`` (board mm in the console's configured
  board frame) and ``aim_err_mm`` against the commanded centre;
* ``aim_pixel`` and the centre projected two ways: through the **marker** board
  solve of that instant (``pixel_delta_pnp``, what the overlay grid shows, only
  on views with 2+ tags) and through the configured board frame used for
  commanding (``pixel_delta_cfg``, available everywhere, a chain-consistency
  check rather than an independent measurement);
* joints, plane samples/spread/rms/age, marker ids, refusal text verbatim.

``TIP_PIXEL`` is recorded as a regression only (it is a mount constant, see
``tip-center.md``); its invariance is never quoted as a result.

The board plane is fed by the console's ``annotate()`` tick, so this tool holds
``/stream`` open for the whole run.  Before every command it re-reads
``/aim-point``; if the plane is stale (the jaws often hide the tags at working
height) it lifts to ``--refresh-z-mm`` at the same x/y, waits for a fresh solve,
and only then commands.  If it cannot get one, the square is skipped with the
reason recorded -- it is never commanded blind.

    PYTHONPATH=. python tools/aim_squares.py --base http://127.0.0.1:8080 \\
        --out reports/aim_squares_20261124.json

Resumable: the receipt is rewritten atomically after every square, and a square
with both legs recorded is skipped unless ``--force``.  Nothing here touches the
gripper: no ``jaw`` is ever sent.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for path in (str(ROOT), str(ROOT / "live")):
    if path not in sys.path:
        sys.path.insert(0, path)

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
FILES = "abcdefgh"
SQUARE_MM = 28.0
BOARD_HALF_MM = 4.0 * SQUARE_MM


# ----------------------------------------------------------------- the squares
def targets(z_mm: float) -> list[tuple[str, float, float, float]]:
    """``a1..h8`` in the same order and frame as ``phase3_squares_irl.targets``."""
    return [(f"{FILES[i]}{rank}", (i + 0.5) * SQUARE_MM - BOARD_HALF_MM,
             (rank - 0.5) * SQUARE_MM - BOARD_HALF_MM, z_mm)
            for rank in range(1, 9) for i in range(8)]


def square_xy(name: str) -> tuple[float, float]:
    index = FILES.index(name[0])
    return ((index + 0.5) * SQUARE_MM - BOARD_HALF_MM,
            (int(name[1]) - 0.5) * SQUARE_MM - BOARD_HALF_MM)


# ------------------------------------------------------------------- http api
class ApiError(RuntimeError):
    def __init__(self, status: int, text: str) -> None:
        self.status = status
        self.text = text
        super().__init__(f"HTTP {status}: {text}")

    @property
    def detail(self) -> str:
        try:
            data = json.loads(self.text)
            if isinstance(data, dict) and "detail" in data:
                return str(data["detail"])
        except ValueError:
            pass
        return self.text.strip()[:400]


class Api:
    def __init__(self, base: str, token: str = "") -> None:
        self.base = base.rstrip("/")
        self.token = token

    def _request(self, method: str, path: str, body=None, timeout: float = 30.0):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.base + path, data=data, method=method)
        if data is not None:
            request.add_header("Content-Type", "application/json")
        if self.token:
            request.add_header("x-bridge-token", self.token)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return json.loads(response.read())
        except urllib.error.HTTPError as exc:
            raise ApiError(exc.code, exc.read().decode("utf-8", "replace")) from None

    def get(self, path: str, timeout: float = 20.0):
        return self._request("GET", path, None, timeout)

    def post(self, path: str, body: dict, timeout: float = 300.0):
        return self._request("POST", path, body, timeout)


# --------------------------------------------------------------- stream pull
class StreamPull(threading.Thread):
    """Hold ``/stream`` open and keep the newest annotated JPEG.

    The console only feeds the aim-plane tracker from ``annotate()``, which only
    runs while a stream client is attached.  This thread is part of the
    measurement, not a convenience: without it the plane ages out.
    """

    def __init__(self, api: Api) -> None:
        super().__init__(name="stream-pull", daemon=True)
        self.api = api
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame: bytes | None = None
        self._ts = 0.0
        self.frames = 0
        self.errors = 0
        self.last_error = ""
        self.connected = False

    def stop(self) -> None:
        self._stop.set()

    def latest(self) -> tuple[bytes | None, float]:
        with self._lock:
            return self._frame, self._ts

    def run(self) -> None:                     # pragma: no cover - live thread
        while not self._stop.is_set():
            try:
                request = urllib.request.Request(self.api.base + "/stream")
                if self.api.token:
                    request.add_header("x-bridge-token", self.api.token)
                with urllib.request.urlopen(request, timeout=20) as response:
                    self.connected = True
                    buffer = b""
                    while not self._stop.is_set():
                        chunk = response.read(65536)
                        if not chunk:
                            break
                        buffer += chunk
                        start = buffer.rfind(b"\xff\xd8")
                        end = buffer.rfind(b"\xff\xd9")
                        if 0 <= start < end:
                            with self._lock:
                                self._frame = buffer[start:end + 2]
                                self._ts = time.time()
                            self.frames += 1
                        if len(buffer) > 2_000_000:
                            buffer = buffer[-500_000:]
            except Exception as exc:           # keep pulling; report, never raise
                self.connected = False
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                time.sleep(1.0)
            finally:
                self.connected = False


# --------------------------------------------------------------- geometry ctx
class Context:
    """Calibration mount, board frame and FK — the same numbers the console uses."""

    def __init__(self, config_path: Path, calibration_path: Path) -> None:
        from types import SimpleNamespace

        import cv2                                            # noqa: F401
        from aim import CameraMount
        from kinematics import ArmKinematics, frame_at

        self.calibration = json.loads(calibration_path.read_text())
        self.mount = CameraMount.from_calibration(self.calibration)
        self.K = np.array([[self.mount.fx, 0.0, self.mount.cx],
                           [0.0, self.mount.fy, self.mount.cy],
                           [0.0, 0.0, 1.0]], float)
        place = {}
        board_path = ROOT / "live" / "board.json"
        if board_path.exists():
            place = json.loads(board_path.read_text())
        self.board_frame = frame_at(float(place.get("x_mm", 0.0)),
                                    float(place.get("y_mm", 0.0)),
                                    float(place.get("z_mm", 0.0)),
                                    float(place.get("yaw_deg", 0.0)))
        # ``ArmKinematics`` wants attribute access; chess_sim's WristCalibration
        # would drag the simulator package in, so carry the same fields here.
        shim = SimpleNamespace(
            camera_in_tool=np.asarray(self.calibration["camera_in_tool"], float),
            intrinsics=tuple(float(v) for v in self.calibration["intrinsics"]),
            distortion=tuple(float(v) for v in
                             self.calibration.get("distortion", (0,) * 5)),
            chain=tuple(self.calibration["chain"]),
            width=int(self.calibration.get("width", 640)),
            height=int(self.calibration.get("height", 480)))
        self.kinematics = ArmKinematics(shim, {}, self.board_frame)
        self._cv2 = cv2

    def pose(self, joints_deg):
        return self.kinematics.fk([float(v) for v in joints_deg])

    def project_configured(self, pose, xy_mm):
        world = (self.board_frame @ np.array(
            [float(xy_mm[0]) / 1000.0, float(xy_mm[1]) / 1000.0, 0.0, 1.0]))[:3]
        uv, depth = self.mount.project(pose, world)
        if not np.isfinite(uv[0]) or depth <= 0.0:
            return None
        return [round(float(uv[0]), 2), round(float(uv[1]), 2)]

    def project_markers(self, board: dict, xy_mm):
        if not board or "rvec" not in board or "tvec" not in board:
            return None
        try:
            point = np.array([[[float(xy_mm[0]) / 1000.0,
                                float(xy_mm[1]) / 1000.0, 0.0]]], np.float64)
            uv, _ = self._cv2.projectPoints(
                point, np.asarray(board["rvec"], float).reshape(3, 1),
                np.asarray(board["tvec"], float).reshape(3, 1), self.K, None)
            return [round(float(v), 2) for v in uv.reshape(2)]
        except Exception:
            return None


# ------------------------------------------------------------- plane freshness
def read_aim(api: Api) -> dict:
    return api.get("/aim-point", timeout=15)


def plane_ready(aim: dict, max_age: float, min_samples: int = 1) -> bool:
    plane = aim.get("plane") or {}
    age = plane.get("age_s")
    return (bool(aim.get("ok")) and age is not None and float(age) <= max_age
            and int(plane.get("samples") or 0) >= min_samples)


def wait_fresh(api: Api, max_age: float, wait_s: float,
               min_samples: int = 1) -> dict:
    deadline = time.time() + max(wait_s, 0.0)
    aim = read_aim(api)
    while not plane_ready(aim, max_age, min_samples) and time.time() < deadline:
        time.sleep(0.6)
        aim = read_aim(api)
    return aim


def tip_now(api: Api):
    try:
        pose = api.get("/pose", timeout=10)
        tip = pose.get("tip_mm")
        return None if not tip else [float(v) for v in tip]
    except ApiError:
        return None


def refresh_plane(api: Api, ctx: Context, z_mm: float, wait_s: float,
                  max_age: float) -> tuple[dict, list[dict]]:
    """Get a fresh marker solve, using the stored home pose when needed.

    The jaws hide the tags at working height on many squares, so a stale plane
    is normal.  The plane is a static board property; refreshing it from a
    marker-visible pose and then descending is exactly what the scan does.
    Measured 2026-11-24: a 150 mm tip-height lift at e4 is refused by the
    approach guard (16 deg), while the stored home pose sees 6 tags and a
    stable plane ([-0.8, 11.5] mm, age < 1 s).  Home is therefore the primary
    refresh pose and the lift is only the fallback.
    """
    aim = wait_fresh(api, max_age, 3.0)
    if plane_ready(aim, max_age):
        return aim, []
    legs: list[dict] = []
    try:
        result = api.post("/recover", {"reason": "aim sweep: marker refresh"},
                          timeout=180)
        legs.append({"leg": "recover", "ok": True,
                     "landed": result.get("landed")})
    except ApiError as exc:
        legs.append({"leg": "recover", "ok": False, "refused": exc.detail})
        tip = tip_now(api)
        if tip is None:
            return aim, legs
        body = {"mode": "xyz", "x_mm": tip[0], "y_mm": tip[1],
                "z_mm": z_mm, "span_deg": 0.0}
        try:
            result = api.post("/move", body, timeout=300)
            legs.append({"leg": "refresh-lift", "ok": True, "z_mm": z_mm,
                         "error_deg": result.get("error_deg")})
        except ApiError as exc2:
            legs.append({"leg": "refresh-lift", "ok": False,
                         "refused": exc2.detail})
            return aim, legs
    aim = wait_fresh(api, max_age, wait_s)
    return aim, legs


# -------------------------------------------------------------- measurements
def _median(values) -> float | None:
    clean = [float(v) for v in values if v is not None and np.isfinite(float(v))]
    return round(float(np.median(clean)), 2) if clean else None


def measure(api: Api, ctx: Context, target_xy, samples: int = 3,
            gap: float = 0.4, settle_s: float = 0.4) -> dict:
    """Post-move snapshot: aim readout, marker solve, both projected centres."""
    time.sleep(max(0.0, settle_s))
    aims, states = [], []
    for index in range(max(1, samples)):
        aims.append(read_aim(api))
        states.append(api.get("/state", timeout=12))
        if index + 1 < samples:
            time.sleep(gap)
    last_aim = aims[-1]
    joints_deg = ((states[-1].get("bridge") or {}).get("joints") or {})
    joints = [float(joints_deg[name]) for name in NAMES] if len(joints_deg) == 6 else None
    aim_x = _median((a.get("aim_board_mm") or [None])[0] for a in aims)
    aim_y = _median((a.get("aim_board_mm") or [None, None])[1] for a in aims)
    aim_px = _median((a.get("aim_pixel") or [None])[0] for a in aims)
    aim_py = _median((a.get("aim_pixel") or [None, None])[1] for a in aims)
    plane_ages = [((a.get("plane") or {}).get("age_s")) for a in aims]
    plane_ages = [float(v) for v in plane_ages if v is not None]
    plane = last_aim.get("plane") or {}

    out: dict = {
        "aim_ok": bool(last_aim.get("ok")),
        "aim_reason": last_aim.get("reason") or "",
        "aim_board_mm": ([aim_x, aim_y] if aim_x is not None else None),
        "aim_square": last_aim.get("square"),
        "aim_pixel": ([aim_px, aim_py] if aim_px is not None else None),
        "tip_pixel": last_aim.get("tip_pixel"),
        "offset_px": last_aim.get("offset_px"),
        "ray_length_mm": last_aim.get("ray_length_mm"),
        "plane": {"source": plane.get("source"),
                  "samples": plane.get("samples"),
                  "spread_mm": plane.get("spread_mm"),
                  "age_s": _median(plane_ages),
                  "rms_px": plane.get("rms_px"),
                  "ids": plane.get("ids")},
        "plane_age_spread_s": (None if not plane_ages else
                               round(max(plane_ages) - min(plane_ages), 2)),
        "joints_deg": {name: round(float(joints_deg[name]), 2)
                       for name in NAMES} if joints else None,
        "board_ids": None,
        "board_rms_px": None,
        "square_uv": None,          # marker-visible views only
        "pixel_delta": None,        # vs the marker-projected centre
        "pixel_delta_uv": None,
        "square_uv_cfg": None,      # configured board frame, every view
        "pixel_delta_cfg": None,
        "pixel_delta_cfg_uv": None,
        "aim_err_mm": None,
        "samples": len(aims),
        "aim_board_spread_mm": None,
        "aim_pixel_spread_px": None,
    }
    board_xs = [(a.get("aim_board_mm") or [None])[0] for a in aims]
    board_ys = [(a.get("aim_board_mm") or [None, None])[1] for a in aims]
    clean_x = [v for v in board_xs if v is not None]
    clean_y = [v for v in board_ys if v is not None]
    if len(clean_x) >= 2 and len(clean_y) >= 2:
        out["aim_board_spread_mm"] = round(float(max(
            max(clean_x) - min(clean_x), max(clean_y) - min(clean_y))), 2)
    pixel_us = [(a.get("aim_pixel") or [None])[0] for a in aims]
    pixel_vs = [(a.get("aim_pixel") or [None, None])[1] for a in aims]
    clean_u = [v for v in pixel_us if v is not None]
    clean_v = [v for v in pixel_vs if v is not None]
    if len(clean_u) >= 2 and len(clean_v) >= 2:
        out["aim_pixel_spread_px"] = round(float(max(
            max(clean_u) - min(clean_u), max(clean_v) - min(clean_v))), 2)
    if aim_x is not None:
        out["aim_err_mm"] = round(float(np.hypot(
            float(target_xy[0]) - aim_x, float(target_xy[1]) - aim_y)), 2)

    # Marker ground truth: the freshest solve among the samples.
    best = None
    for state in states:
        target = state.get("target") or {}
        board = target.get("board") or {}
        age = target.get("age_s")
        if not board or age is None or float(age) > 2.5:
            continue
        if best is None or float(age) < float(best[1]):
            best = (board, age)
    if best is not None:
        board, age = best
        out["board_ids"] = len(board.get("ids") or []) or None
        out["board_rms_px"] = (None if board.get("rms_px") is None
                               else round(float(board["rms_px"]), 3))
        out["board_age_s"] = round(float(age), 2)
        out["square_uv"] = ctx.project_markers(board, target_xy)

    if out["square_uv"] and out["aim_pixel"]:
        du = float(out["aim_pixel"][0] - out["square_uv"][0])
        dv = float(out["aim_pixel"][1] - out["square_uv"][1])
        out["pixel_delta_uv"] = [round(du, 2), round(dv, 2)]
        out["pixel_delta"] = round(float(math.hypot(du, dv)), 2)

    # Configured-board projection: same frame the command uses.
    if joints is not None:
        pose = ctx.pose(joints)
        out["square_uv_cfg"] = ctx.project_configured(pose, target_xy)
        if out["square_uv_cfg"] and out["aim_pixel"]:
            du = float(out["aim_pixel"][0] - out["square_uv_cfg"][0])
            dv = float(out["aim_pixel"][1] - out["square_uv_cfg"][1])
            out["pixel_delta_cfg_uv"] = [round(du, 2), round(dv, 2)]
            out["pixel_delta_cfg"] = round(float(math.hypot(du, dv)), 2)
    return out


# ------------------------------------------------------------------- the legs
def tip_leg(api: Api, ctx: Context, x: float, y: float, z: float,
            settle_s: float) -> dict:
    """``mode=xyz`` — the jaw tip to the square centre, the historical way."""
    body = {"mode": "xyz", "x_mm": x, "y_mm": y, "z_mm": z, "span_deg": 0.0}
    started = time.time()
    try:
        result = api.post("/move", body, timeout=300)
    except ApiError as exc:
        return {"done": True, "ok": False, "refused": exc.detail,
                "http_status": exc.status,
                "move_s": round(time.time() - started, 1)}
    record = {"done": True, "ok": True,
              "move_s": round(time.time() - started, 1),
              "error_deg": result.get("error_deg"),
              "commands": result.get("commands"),
              "travel_deg": result.get("travel_deg"),
              "tip_mm": (result.get("pose") or {}).get("tip_mm")}
    if record["tip_mm"]:
        record["tip_err_mm"] = round(float(np.hypot(
            record["tip_mm"][0] - x, record["tip_mm"][1] - y)), 2)
    record.update(measure(api, ctx, (x, y), settle_s=settle_s))
    return record


def aim_leg(api: Api, x: float, y: float, tolerance: float, iterations: int,
            plane_max_age_s: float) -> dict:
    """``mode=aim_point`` — one app-side closed loop on the aim ray."""
    started = time.time()
    body = {"mode": "aim_point", "x_mm": x, "y_mm": y,
            "tolerance_mm": tolerance, "iterations": iterations,
            "plane_max_age_s": plane_max_age_s,
            "roll_lock": True, "max_step_mm": 45.0}
    try:
        result = api.post("/move", body, timeout=300)
    except ApiError as exc:
        return {"done": True, "ok": False, "refused": exc.detail,
                "http_status": exc.status,
                "move_s": round(time.time() - started, 1)}
    steps = result.get("steps") or []
    converged = (result.get("final_error_mm") is not None
                 and float(result["final_error_mm"]) <= tolerance
                 and not result.get("stalled")
                 and any(step.get("done") for step in steps))
    return {"done": True, "ok": True,
            "move_s": round(time.time() - started, 1),
            "converged": bool(converged),
            "tolerance_mm": tolerance,
            "iterations": result.get("iterations"),
            "stalled": bool(result.get("stalled")),
            "start_aim_mm": result.get("start_aim_mm"),
            "final_aim_mm": result.get("final_aim_mm"),
            "final_error_mm": result.get("final_error_mm"),
            "aim_pixel_from_move": result.get("aim_pixel"),
            "square_from_move": result.get("square"),
            "steps": [{key: step.get(key) for key in
                       ("iteration", "aim_board_mm", "error_mm", "aim_pixel",
                        "step_mm", "solve_position_mm", "solve_approach_deg",
                        "stalled", "done", "executed")}
                      for step in steps],
            "joints_deg": result.get("joints_deg"),
            "tip_mm": (result.get("pose") or {}).get("tip_mm")}


def refine_leg(api: Api, x: float, y: float, tolerance: float,
               plane_max_age_s: float, max_steps: int = 3,
               settle_s: float = 0.35, start_shortfall: float = 2.0,
               max_step_mm: float = 40.0) -> dict:
    """Deadband-compensated nudge loop on ``mode=rel``.

    Each step commands ``residual + shortfall`` along the error direction: the
    exact Newton step plus the amount the servos are expected to fall short of
    it.  The shortfall is re-estimated from the observed transfer after every
    step, so it adapts instead of assuming one number for the whole board.
    """
    started = time.time()
    aim = read_aim(api)
    if not plane_ready(aim, plane_max_age_s):
        return {"done": True, "ok": False,
                "refused": "refine needs a fresh plane: "
                           + (aim.get("reason") or "stale"),
                "move_s": round(time.time() - started, 1)}
    shortfall = float(start_shortfall)
    history: list[dict] = []
    current = aim.get("aim_board_mm")
    if not current:
        return {"done": True, "ok": False, "refused": "no aim_board_mm",
                "move_s": round(time.time() - started, 1)}
    error = math.hypot(current[0] - x, current[1] - y)
    for step in range(max_steps):
        if error <= tolerance:
            break
        ex, ey = float(current[0] - x), float(current[1] - y)
        magnitude = error + shortfall
        if magnitude > max_step_mm:
            scale = max_step_mm / magnitude
        else:
            scale = 1.0
        dx, dy = -ex * scale, -ey * scale
        commanded = math.hypot(dx, dy)
        body = {"mode": "rel", "dx_mm": round(dx, 3), "dy_mm": round(dy, 3),
                "dz_mm": 0.0, "roll_lock": True}
        try:
            result = api.post("/move", body, timeout=300)
        except ApiError as exc:
            history.append({"step": step, "commanded_mm": round(commanded, 2),
                            "refused": exc.detail})
            return {"done": True, "ok": False, "refused": exc.detail,
                    "history": history, "steps": len(history),
                    "final_error_mm": round(error, 2),
                    "move_s": round(time.time() - started, 1), "shortfall_mm": round(shortfall, 2)}
        time.sleep(max(0.0, settle_s))
        aim = read_aim(api)
        if not plane_ready(aim, plane_max_age_s):
            history.append({"step": step, "plane_lost": aim.get("reason")})
            return {"done": True, "ok": False,
                    "refused": "plane went stale mid-refine: "
                               + (aim.get("reason") or ""),
                    "history": history, "steps": len(history),
                    "final_error_mm": round(error, 2),
                    "move_s": round(time.time() - started, 1),
                    "shortfall_mm": round(shortfall, 2)}
        updated = aim.get("aim_board_mm")
        if not updated:
            history.append({"step": step, "no_aim": True})
            break
        achieved = math.hypot(updated[0] - current[0], updated[1] - current[1])
        # Progress along the commanded direction, signed.
        ux, uy = (-ex / error, -ey / error)
        progress = (updated[0] - current[0]) * ux + (updated[1] - current[1]) * uy
        observed = commanded - progress
        shortfall = float(np.clip(0.5 * shortfall + 0.5 * observed, 0.0, 12.0))
        current = updated
        previous_error = error
        error = math.hypot(current[0] - x, current[1] - y)
        history.append({"step": step, "commanded_mm": round(commanded, 2),
                        "achieved_mm": round(achieved, 2),
                        "progress_mm": round(progress, 2),
                        "shortfall_mm": round(shortfall, 2),
                        "error_mm": round(error, 2),
                        "error_deg": result.get("error_deg")})
        # Stop when a step stops paying: below the readout/actuator floor a
        # further nudge only moves noise around.
        if progress < 0.5 or error > previous_error * 0.92:
            break
    return {"done": True, "ok": True,
            "converged": error <= tolerance,
            "final_error_mm": round(error, 2),
            "final_aim_mm": current,
            "steps": len(history),
            "shortfall_mm": round(shortfall, 2),
            "history": history,
            "move_s": round(time.time() - started, 1)}


# ------------------------------------------------------------------ receipts
def load_receipt(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except ValueError:
        return {}


def save_receipt(path: Path, receipt: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(receipt, indent=2) + "\n")
    os.replace(temporary, path)


# --------------------------------------------------------------------- stats
def summarise(values) -> dict:
    clean = np.array([float(v) for v in values if v is not None
                      and np.isfinite(float(v))], float)
    if not clean.size:
        return {"n": 0, "p50": None, "p95": None, "max": None, "mean": None}
    return {"n": int(clean.size),
            "p50": round(float(np.percentile(clean, 50)), 2),
            "p95": round(float(np.percentile(clean, 95)), 2),
            "max": round(float(clean.max()), 2),
            "mean": round(float(clean.mean()), 2)}


# ------------------------------------------------------------------- figures
_RAMP = ((68, 1, 84), (59, 82, 139), (33, 145, 140),
         (94, 201, 98), (253, 231, 37))


def _colour(value: float, vmax: float):
    if value is None or not np.isfinite(value):
        return (58, 63, 68)
    scale = max(1e-9, vmax)
    t = max(0.0, min(1.0, float(value) / scale)) * (len(_RAMP) - 1)
    index = min(int(t), len(_RAMP) - 2)
    fraction = t - index
    a, b = _RAMP[index], _RAMP[index + 1]
    return tuple(int(round(a[i] + (b[i] - a[i]) * fraction)) for i in range(3))


def _text_colour(colour) -> tuple[int, int, int]:
    luminance = 0.299 * colour[0] + 0.587 * colour[1] + 0.114 * colour[2]
    return (16, 16, 16) if luminance > 130 else (245, 245, 245)


def _panel(canvas, origin, cell, array, title, vmax, subtitle="") -> None:
    import cv2
    x0, y0 = origin
    cv2.putText(canvas, title, (x0, y0 - 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (235, 240, 245), 1, cv2.LINE_AA)
    if subtitle:
        cv2.putText(canvas, subtitle, (x0, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (150, 170, 190), 1, cv2.LINE_AA)
    for rank in range(1, 9):
        for index in range(8):
            value = array[rank - 1, index]
            colour = _colour(value, vmax)
            left = x0 + index * cell
            top = y0 + (8 - rank) * cell
            cv2.rectangle(canvas, (left + 1, top + 1),
                          (left + cell - 2, top + cell - 2), colour, -1)
            label = "--" if value is None or not np.isfinite(value) else f"{value:.0f}"
            cv2.putText(canvas, label, (left + 5, top + cell - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                        _text_colour(colour), 1, cv2.LINE_AA)
    for index, file in enumerate(FILES):
        cv2.putText(canvas, file, (x0 + index * cell + cell // 2 - 4,
                                   y0 + 8 * cell + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (170, 185, 200), 1)
    for rank in range(1, 9):
        cv2.putText(canvas, str(rank), (x0 - 13, y0 + (8 - rank) * cell + cell // 2 + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (170, 185, 200), 1)


def _grid_array(receipt: dict, leg: str, field: str) -> np.ndarray:
    array = np.full((8, 8), np.nan)
    rows = receipt.get("squares") or {}
    for rank in range(1, 9):
        for index, file in enumerate(FILES):
            record = (rows.get(f"{file}{rank}") or {}).get(leg) or {}
            value = record.get(field)
            if value is not None and np.isfinite(float(value)):
                array[rank - 1, index] = float(value)
    return array


def _colourbar(canvas, origin, width, vmax) -> None:
    import cv2
    x0, y0 = origin
    for i in range(width):
        colour = _colour(vmax * i / max(1, width - 1), vmax)
        cv2.line(canvas, (x0 + i, y0), (x0 + i, y0 + 10), colour, 1)
    for fraction in (0.0, 0.5, 1.0):
        label = f"{vmax * fraction:.0f}"
        cv2.putText(canvas, label, (x0 + int(width * fraction) - 8, y0 + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (170, 185, 200), 1)


def status_card(receipt: dict, path: Path, title: str) -> bool:
    import cv2
    cell = 46
    canvas = np.full((2 * 8 * cell + 150, 8 * cell + 360, 3), 22, np.uint8)
    summary = receipt.get("summary") or {}
    aim = _grid_array(receipt, "aim", "aim_err_mm")
    tip = _grid_array(receipt, "tip", "aim_err_mm")
    _panel(canvas, (20, 44), cell, aim, "aim mode: |aim - square centre| mm",
           10.0, "yellow reticle commanded")
    _panel(canvas, (20, 44 + 8 * cell + 60), cell, tip,
           "tip mode: where the AIM landed (mm)", 10.0,
           "tip commanded to centre")
    _colourbar(canvas, (20, 2 * 8 * cell + 110), 8 * cell, 10.0)
    done = int(summary.get("aim_done") or 0)
    lines = [f"{title}  {done}/64 squares",
             f"aim p50 {summary.get('aim_err_mm', {}).get('p50')} mm  "
             f"p95 {summary.get('aim_err_mm', {}).get('p95')} mm",
             f"tip p50 {summary.get('tip_err_mm', {}).get('p50')} mm  "
             f"p95 {summary.get('tip_err_mm', {}).get('p95')} mm"]
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (20 + 8 * cell + 40, 60 + index * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 232, 240), 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(path), canvas))


def final_heatmap(receipt: dict, path: Path) -> bool:
    import cv2
    cell = 62
    width = 2 * (8 * cell) + 120
    height = 2 * (8 * cell) + 220
    canvas = np.full((height, width, 3), 20, np.uint8)
    summary = receipt.get("summary") or {}
    panels = (
        ("aim", "aim_err_mm", "aim mode: |aim - square centre| (mm)", 20.0),
        ("tip", "aim_err_mm", "tip mode: where the AIM landed (mm)", 20.0),
        ("aim", "pixel_delta", "aim mode: aim px vs marker grid centre (px)", 120.0),
        ("tip", "pixel_delta", "tip mode: aim px vs marker grid centre (px)", 120.0),
    )
    for index, (leg, field, title, vmax) in enumerate(panels):
        column, row = index % 2, index // 2
        origin = (40 + column * (8 * cell + 60), 46 + row * (8 * cell + 90))
        _panel(canvas, origin, cell, _grid_array(receipt, leg, field),
               title, vmax)
    _colourbar(canvas, (40, height - 55), 8 * cell, 20.0)
    _colourbar(canvas, (40 + 8 * cell + 60, height - 55), 8 * cell, 120.0)
    header = (f"aim-ray 64-square sweep  {summary.get('stamp', '')}   "
              f"aim p50 {summary.get('aim_err_mm', {}).get('p50')} mm   "
              f"tip p50 {summary.get('tip_err_mm', {}).get('p50')} mm")
    cv2.putText(canvas, header, (40, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (225, 232, 240), 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(path), canvas))


def build_gif(frames_dir: Path, out_path: Path, every: int = 2) -> bool:
    """Small GIF of the annotated stream, one frame per sampled square."""
    if not frames_dir.exists():
        return False
    frames = sorted(frames_dir.glob("*.jpg"))
    if len(frames) < 3:
        return False
    frames = frames[::max(1, every)]
    staging = frames_dir.parent / (frames_dir.name + "_gif")
    staging.mkdir(parents=True, exist_ok=True)
    for old in staging.glob("*.jpg"):
        old.unlink()
    for index, frame in enumerate(frames):
        (staging / f"{index:03d}.jpg").write_bytes(frame.read_bytes())
    try:
        import subprocess
        out_path.parent.mkdir(parents=True, exist_ok=True)
        command = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", "3",
                   "-i", str(staging / "%03d.jpg"),
                   "-vf", ("scale=420:-1:flags=lanczos,split[a][b];"
                           "[a]palettegen=stats_mode=diff[p];"
                           "[b][p]paletteuse=dither=bayer"),
                   "-loop", "0", str(out_path)]
        subprocess.run(command, check=True, timeout=180)
        return out_path.exists()
    except Exception as exc:                    # pragma: no cover
        print(f"gif skipped: {exc}", flush=True)
        return False


# ------------------------------------------------------------------ summary
def build_summary(receipt: dict, plan) -> dict:
    rows = receipt.get("squares") or {}
    names = [row[0] for row in plan]
    aim_records = [(rows.get(name) or {}).get("aim") or {} for name in names]
    tip_records = [(rows.get(name) or {}).get("tip") or {} for name in names]
    aim_done = [r for r in aim_records if r.get("done")]
    tip_done = [r for r in tip_records if r.get("done")]

    # The aim column's final number is the refined aim when refinement ran,
    # otherwise whatever the app's aim_point left.  Both are kept separately.
    def aim_error(record):
        refine = record.get("refine") or {}
        if refine.get("ok") and refine.get("final_error_mm") is not None:
            return refine["final_error_mm"]
        return record.get("final_error_mm")

    summary = {
        "stamp": receipt.get("stamp"),
        "squares": len(names),
        "aim_done": len(aim_done),
        "aim_ok": sum(1 for r in aim_done if r.get("ok")),
        "aim_refused": sum(1 for r in aim_done if r.get("refused")),
        "aim_converged": sum(1 for r in aim_done
                             if r.get("converged")
                             or (r.get("refine") or {}).get("converged")),
        "aim_refined": sum(1 for r in aim_done if (r.get("refine") or {}).get("ok")),
        "aim_point_only_p50": summarise(
            r.get("final_error_mm") for r in aim_done if r.get("ok")),
        "tip_done": len(tip_done),
        "tip_ok": sum(1 for r in tip_done if r.get("ok")),
        "tip_refused": sum(1 for r in tip_done if r.get("refused")),
        "plane_failures": sum(len((rows.get(name) or {}).get("plane_failures") or [])
                              for name in names),
        # aim_err_mm is re-measured AFTER everything (independent /aim-point
        # read), so it is the number to quote; final_error_mm is the app's own.
        "aim_err_mm": summarise(r.get("aim_err_mm") for r in aim_done if r.get("ok")),
        "tip_err_mm": summarise(r.get("aim_err_mm") for r in tip_done if r.get("ok")),
        "aim_px": summarise(r.get("pixel_delta") for r in aim_done if r.get("ok")),
        "tip_px": summarise(r.get("pixel_delta") for r in tip_done if r.get("ok")),
        "aim_px_cfg": summarise(r.get("pixel_delta_cfg") for r in aim_done if r.get("ok")),
        "tip_px_cfg": summarise(r.get("pixel_delta_cfg") for r in tip_done if r.get("ok")),
        "aim_tip_err_mm": summarise(r.get("tip_err_mm") for r in aim_done if r.get("ok")),
        "tip_tip_err_mm": summarise(r.get("tip_err_mm") for r in tip_done if r.get("ok")),
        "aim_controller_err_mm": summarise(aim_error(r) for r in aim_done if r.get("ok")),
        "readout_spread_mm": summarise(r.get("aim_board_spread_mm")
                                       for r in aim_done if r.get("ok")),
        "readout_spread_px": summarise(r.get("aim_pixel_spread_px")
                                       for r in aim_done if r.get("ok")),
    }
    failures = []
    for name in names:
        record = rows.get(name) or {}
        for leg in ("tip", "aim"):
            entry = record.get(leg) or {}
            if entry.get("refused"):
                failures.append({"square": name, "leg": leg,
                                 "reason": entry["refused"]})
            elif entry.get("done") and not entry.get("ok") and leg == "aim":
                failures.append({"square": name, "leg": leg,
                                 "reason": entry.get("aim_reason") or "no aim"})
            refine = entry.get("refine") or {}
            if refine.get("refused"):
                failures.append({"square": name, "leg": "aim/refine",
                                 "reason": refine["refused"]})
        for failure in record.get("plane_failures") or []:
            failures.append({"square": name, "leg": "plane",
                             "reason": failure.get("reason", "")})
    summary["failures"] = failures
    summary["square_total"] = len(names)
    return summary


def markdown_table(receipt: dict) -> str:
    rows = receipt.get("squares") or {}
    lines = ["| sq | tip aim err mm | tip px | aim err mm | aim px | aim conv | iters/refine | plane |",
             "|---|---|---|---|---|---|---|---|"]
    for name, *_ in targets(float(receipt.get("z_mm") or 70.0)):
        record = rows.get(name) or {}
        tip = record.get("tip") or {}
        aim = record.get("aim") or {}
        refine = aim.get("refine") or {}
        def fmt(value, digits=1):
            return "--" if value is None else f"{float(value):.{digits}f}"
        conv = ("yes" if aim.get("converged") else
                ("refined" if refine.get("converged") else
                 ("no" if aim.get("done") else "--")))
        iters = aim.get("iterations")
        if refine:
            iters = f"{iters}/{refine.get('steps')}"
        plane = record.get("plane_failures")
        lines.append(f"| {name} | {fmt(tip.get('aim_err_mm'))} | "
                     f"{fmt(tip.get('pixel_delta'))} | {fmt(aim.get('aim_err_mm'))} | "
                     f"{fmt(aim.get('pixel_delta'))} | {conv} | {iters or '--'} | "
                     f"{'FAILED' if plane else 'ok'} |")
    return "\n".join(lines)


# ------------------------------------------------------------------ the sweep
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="http://127.0.0.1:8080")
    parser.add_argument("--token-file", default=str(ROOT / "live" / "bridge_token.txt"))
    parser.add_argument("--config", default=str(ROOT / "live" / "live_config.json"))
    parser.add_argument("--out", default=str(ROOT / "reports" / "aim_squares_20261124.json"))
    parser.add_argument("--frames-dir",
                        default=str(ROOT / "reports" / "aim_squares_frames_20261124"))
    parser.add_argument("--z-mm", type=float, default=70.0)
    parser.add_argument("--refresh-z-mm", type=float, default=150.0)
    parser.add_argument("--tolerance-mm", type=float, default=4.0)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--refine-steps", type=int, default=3)
    parser.add_argument("--plane-age-s", type=float, default=8.0,
                        help="a command needs a marker solve younger than this")
    parser.add_argument("--plane-max-age-s", type=float, default=25.0,
                        help="value handed to aim_point / refine for their guard")
    parser.add_argument("--plane-wait-s", type=float, default=12.0)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--squares", default="")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=3600.0)
    parser.add_argument("--status-every", type=int, default=8)
    parser.add_argument("--no-stream", action="store_true")
    args = parser.parse_args()

    receipt_path = Path(args.out)
    frames_dir = Path(args.frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True)

    token = ""
    if Path(args.token_file).exists():
        token = Path(args.token_file).read_text().strip()
    api = Api(args.base, token)

    config_path = Path(args.config)
    calibration_path = Path(ROOT / ".private/phase11d/calibration.json")
    if config_path.exists():
        config = json.loads(config_path.read_text())
        candidate = Path(config.get("calibration", ""))
        if not candidate.is_absolute():
            candidate = ROOT / candidate
        if candidate.exists():
            calibration_path = candidate
    ctx = Context(config_path, calibration_path)

    stream = None
    if not args.no_stream:
        stream = StreamPull(api)
        stream.start()
        time.sleep(1.0)
        print(f"stream pulling: connected={stream.connected}", flush=True)

    state = api.get("/state", timeout=12)
    bridge = state.get("bridge") or {}
    if bridge.get("mode") != "REMOTE":
        print(f"bridge is not in REMOTE (mode={bridge.get('mode')!r}); "
              "refusing to sweep", file=sys.stderr)
        if stream is not None:
            stream.stop()
        return 2

    receipt = load_receipt(receipt_path) if not args.force else {}
    receipt.setdefault("schema", "aim-squares/2")
    receipt.setdefault("started", time.strftime("%Y-%m-%dT%H:%M:%S"))
    receipt["stamp"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    receipt = {"schema": "aim-squares/2", **receipt}
    receipt["base"] = args.base
    receipt["z_mm"] = args.z_mm
    receipt["tolerance_mm"] = args.tolerance_mm
    receipt["iterations"] = args.iterations
    receipt["board_frame"] = {"x_mm": round(float(ctx.board_frame[0, 3]) * 1000, 2),
                              "y_mm": round(float(ctx.board_frame[1, 3]) * 1000, 2),
                              "z_mm": round(float(ctx.board_frame[2, 3]) * 1000, 2),
                              "yaw_deg": round(math.degrees(math.atan2(
                                  ctx.board_frame[1, 0], ctx.board_frame[0, 0])), 2)}
    receipt.setdefault("squares", {})

    wanted = [name for name in (args.squares.split(",") if args.squares else [])
              if name]
    plan = [row for row in targets(args.z_mm) if not wanted or row[0] in wanted]
    if args.limit:
        plan = plan[:args.limit]
    started = time.time()
    milestone = 0
    plane_streak = 0

    print(f"sweep: {len(plan)} squares, z {args.z_mm} mm, tolerance "
          f"{args.tolerance_mm} mm, iterations {args.iterations}, "
          f"refine<= {args.refine_steps}", flush=True)
    for name, x, y, z in plan:
        record = receipt["squares"].get(name) or {"square": name,
                                                  "target_mm": [x, y], "z_mm": z}
        receipt["squares"][name] = record
        need_tip = args.force or not (record.get("tip") or {}).get("done")
        need_aim = args.force or not (record.get("aim") or {}).get("done")
        if not need_tip and not need_aim:
            print(f"{name}: already recorded, skipped", flush=True)
            continue
        if time.time() - started > args.max_seconds:
            print("time budget reached; stopping", flush=True)
            break

        aim, refresh_legs = refresh_plane(api, ctx, args.refresh_z_mm,
                                          args.plane_wait_s, args.plane_age_s)
        if refresh_legs:
            record.setdefault("refresh", []).extend(refresh_legs)
            for leg in refresh_legs:
                print(f"{name} refresh: {leg}", flush=True)
        if not plane_ready(aim, args.plane_age_s):
            reason = aim.get("reason") or "no usable board plane"
            print(f"{name}: plane not usable ({reason}); not commanding", flush=True)
            record.setdefault("plane_failures", []).append(
                {"at": time.strftime("%H:%M:%S"), "reason": reason,
                 "plane": aim.get("plane")})
            save_receipt(receipt_path, receipt)
            plane_streak += 1
            if plane_streak >= 3:
                print("board plane unavailable for 3 squares; stopping rather "
                      "than commanding blind", flush=True)
                receipt["stopped"] = {"reason": "board plane unavailable",
                                      "at": name}
                break
            continue
        plane_streak = 0

        if need_tip:
            record["tip"] = tip_leg(api, ctx, x, y, z, args.settle_s)
            tip = record["tip"]
            print(f"{name} tip: ok {tip.get('ok')} tip_err {tip.get('tip_err_mm')} "
                  f"aim_err {tip.get('aim_err_mm')} px {tip.get('pixel_delta')} "
                  f"'{tip.get('aim_square')}'"
                  + (f"  REFUSED {tip['refused']}" if tip.get("refused") else ""),
                  flush=True)
            save_receipt(receipt_path, receipt)

        # A refused leg may have driven the arm home; never measure or refine
        # from a pose that is not over the square.
        tip_position = tip_now(api)
        near = (tip_position is not None
                and math.hypot(tip_position[0] - x,
                               tip_position[1] - y) <= 30.0
                and abs(float(tip_position[2]) - z) <= 25.0)
        if need_aim and not near:
            record["aim"] = {"done": False, "ok": False,
                             "refused": "arm is not over the square after the "
                                        f"tip leg (tip {tip_position} vs "
                                        f"{[x, y, z]}); skipped"}
            save_receipt(receipt_path, receipt)
            print(f"{name} aim: arm not near the square; skipped", flush=True)
        elif need_aim:
            aim_now = wait_fresh(api, args.plane_age_s, args.plane_wait_s)
            if not plane_ready(aim_now, args.plane_age_s):
                reason = aim_now.get("reason") or "no usable board plane"
                record["aim"] = {"done": False, "ok": False,
                                 "refused": "plane before aim_point: " + reason}
                record.setdefault("plane_failures", []).append(
                    {"at": time.strftime("%H:%M:%S"), "leg": "aim",
                     "reason": reason})
                save_receipt(receipt_path, receipt)
                print(f"{name} aim: plane not usable ({reason}); not commanding",
                      flush=True)
            else:
                record["aim"] = aim_leg(api, x, y, args.tolerance_mm,
                                        args.iterations, args.plane_max_age_s)
                aim_record = record["aim"]
                print(f"{name} aim_point: ok {aim_record.get('ok')} "
                      f"conv {aim_record.get('converged')} "
                      f"err {aim_record.get('final_error_mm')} "
                      f"iters {aim_record.get('iterations')} "
                      f"{'stalled' if aim_record.get('stalled') else ''}"
                      + (f"  REFUSED {aim_record['refused']}"
                         if aim_record.get("refused") else ""), flush=True)
                should_refine = not aim_record.get("converged")
                if should_refine and aim_record.get("refused"):
                    should_refine = False
                if should_refine:
                    aim_record["refine"] = refine_leg(
                        api, x, y, args.tolerance_mm, args.plane_max_age_s,
                        max_steps=args.refine_steps)
                    refine = aim_record["refine"]
                    print(f"{name} refine: ok {refine.get('ok')} "
                          f"conv {refine.get('converged')} "
                          f"err {refine.get('final_error_mm')} "
                          f"steps {refine.get('steps')} "
                          f"shortfall {refine.get('shortfall_mm')}"
                          + (f"  REFUSED {refine['refused']}"
                             if refine.get("refused") else ""), flush=True)
                # Re-measure independently of the app's own final numbers.  A
                # refused aim_point may have been recovered to home, so there
                # is nothing meaningful to measure in that case.
                if aim_record.get("ok"):
                    aim_record.update(measure(api, ctx, (x, y),
                                              settle_s=args.settle_s))
                    print(f"{name} aim final: aim_err "
                          f"{aim_record.get('aim_err_mm')} "
                          f"px {aim_record.get('pixel_delta')} "
                          f"cfg_px {aim_record.get('pixel_delta_cfg')} "
                          f"plane {aim_record.get('plane')}", flush=True)
                save_receipt(receipt_path, receipt)

        frame, _ = stream.latest() if stream is not None else (None, 0.0)
        if frame:
            (frames_dir / f"{name}.jpg").write_bytes(frame)

        done = len([r for r in receipt["squares"].values()
                    if (r.get("aim") or {}).get("done")])
        if args.status_every and done and done % args.status_every == 0 and done > milestone:
            milestone = done
            receipt["summary"] = build_summary(receipt, plan)
            card = Path(args.out).with_name(f"aim_squares_status_{done:02d}.png")
            if status_card(receipt, card, f"aim sweep {done}/64"):
                print(f"status card: {card}", flush=True)
            save_receipt(receipt_path, receipt)

    receipt["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    receipt["summary"] = build_summary(receipt, targets(args.z_mm))
    save_receipt(receipt_path, receipt)

    heatmap = Path(args.out).with_name("aim_squares_heatmap.png")
    made_heatmap = final_heatmap(receipt, heatmap)
    gif = Path(args.out).with_name("aim_squares.gif")
    made_gif = build_gif(frames_dir, gif, every=2)
    receipt["artifacts"] = {"heatmap": str(heatmap) if made_heatmap else None,
                            "gif": str(gif) if made_gif else None,
                            "frames": str(frames_dir)}
    save_receipt(receipt_path, receipt)

    summary = receipt["summary"]
    print("\n=== summary")
    for key in ("aim_done", "aim_ok", "aim_converged", "aim_refined",
                "aim_refused", "tip_done", "tip_refused", "plane_failures"):
        print(f"  {key:18s} {summary.get(key)}")
    for tag in ("aim_err_mm", "tip_err_mm", "aim_px", "tip_px",
                "aim_px_cfg", "tip_px_cfg", "aim_tip_err_mm", "tip_tip_err_mm",
                "readout_spread_mm", "readout_spread_px"):
        block = summary.get(tag) or {}
        print(f"  {tag:18s} n {block.get('n')}  p50 {block.get('p50')}  "
              f"p95 {block.get('p95')}  max {block.get('max')}  mean {block.get('mean')}")
    for failure in summary.get("failures") or []:
        print(f"  FAIL {failure['square']} {failure['leg']}: {failure['reason']}")
    print(f"  receipt  {receipt_path}")
    print(f"  heatmap  {heatmap if made_heatmap else 'not made'}")
    print(f"  gif      {gif if made_gif else 'not made'}")
    report = Path(args.out).with_suffix(".md")
    report.write_text("\n".join([
        "# Aim-ray 64-square sweep",
        "",
        f"- session: {receipt.get('started')} -> {receipt.get('finished')}",
        f"- base: {receipt['base']}, z {receipt['z_mm']} mm, tolerance "
        f"{receipt['tolerance_mm']} mm",
        f"- board frame: {receipt['board_frame']}",
        "",
        "## Summary",
        "",
        "| metric | n | p50 | p95 | max | mean |",
        "|---|---|---|---|---|---|",
    ] + [f"| {tag} | {(summary.get(tag) or {}).get('n')} | "
         f"{(summary.get(tag) or {}).get('p50')} | "
         f"{(summary.get(tag) or {}).get('p95')} | "
         f"{(summary.get(tag) or {}).get('max')} | "
         f"{(summary.get(tag) or {}).get('mean')} |"
         for tag in ("aim_err_mm", "tip_err_mm", "aim_px", "tip_px",
                     "aim_px_cfg", "tip_px_cfg", "readout_spread_mm",
                     "readout_spread_px")] + [
        "",
        "## Per square",
        "",
        markdown_table(receipt),
        "",
        "## Failures (verbatim)",
        "",
    ] + [f"- {failure['square']} {failure['leg']}: {failure['reason']}"
         for failure in summary.get("failures") or []] + [""]) + "\n")
    print(f"  report   {report}")
    if stream is not None:
        stream.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
