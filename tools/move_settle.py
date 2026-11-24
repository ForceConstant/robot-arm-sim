#!/usr/bin/env python3
"""Joint-read helpers for move verification — one `settle()` for every probe.

Why this module exists: the 2026-11-23 regression was *not* a bad target. The
console accepted a relocation (perr 1.5 mm), reported success, and the arm never
moved. Any test that reads the joints once, immediately after the call, cannot
tell "arrived" from "never started", and any test that reads them 1.6 s later
measures a half-travelled arm and calls it coupling (see
`tools/sim_jacobian_sweep.py`). So: poll until the joints STOP changing, and
only then measure.

Everything here is transport and timing. Motion policy stays in `live/app.py`.
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


class MoveError(RuntimeError):
    """Transport or protocol failure; the caller decides whether to abort."""


# --------------------------------------------------------------------- HTTP
def get_json(url: str, timeout: float = 8.0) -> dict:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            return json.loads(response.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        raise MoveError(f"GET {url} -> {exc.code}: "
                        f"{exc.read().decode(errors='replace')[:300]}") from exc
    except urllib.error.URLError as exc:
        raise MoveError(f"GET {url} unreachable: {exc}") from exc


def post_json(url: str, body: dict, timeout: float = 120.0) -> dict:
    """POST JSON; an HTTP error comes back as a dict, not an exception.

    A 409 from the console is a *refusal* (unreachable pose, keep-in, wrist
    window) and the harness treats it as a skipped case, not a crash.
    """
    data = json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors="replace")
        return {"http_error": exc.code, "detail": detail[:500]}
    except urllib.error.URLError as exc:
        raise MoveError(f"POST {url} unreachable: {exc}") from exc


# -------------------------------------------------------------------- reads
def bridge_state(bridge_url: str, timeout: float = 6.0) -> dict:
    return get_json(bridge_url.rstrip("/") + "/state", timeout=timeout)


def joints_of(state: dict) -> list[float]:
    joints = state.get("joints") or {}
    if len(joints) != 6:
        raise MoveError(f"bridge state has no 6-joint readback: {sorted(joints)}")
    return [float(joints[name]) for name in JOINTS]


def read_joints(bridge_url: str) -> tuple[list[float], float | None]:
    """(joints, joints_age_s) from one bridge read."""
    state = bridge_state(bridge_url)
    return joints_of(state), state.get("joints_age_s")


def tip_mm(console_url: str) -> list[float] | None:
    """Jaw-tip position in board millimetres, from the console's own FK.

    Read through the console so the harness cannot drift from `TIP_IN_CHAIN`
    or the deployed board frame.
    """
    try:
        return [float(v) for v in get_json(console_url.rstrip("/") + "/keep-in")["tip_mm"]]
    except (MoveError, KeyError, TypeError, ValueError):
        return None


# ------------------------------------------------------------------ measures
def deltas(before: list[float], after: list[float]) -> dict:
    """Per-joint signed change, plus the summary the verdicts use."""
    change = [b - a for a, b in zip(before, after)]
    return {"per_joint_deg": [round(v, 3) for v in change],
            "max_abs_deg": round(max(abs(v) for v in change), 3),
            "travel_deg": round(sum(abs(v) for v in change), 3),
            "joints_ge_25": sum(1 for v in change if abs(v) >= 25.0)}


def distance_mm(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None or len(a) != len(b):
        return None
    return round(sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5, 2)


# -------------------------------------------------------------------- settle
def settle(read, *, quiet_s: float = 0.8, timeout_s: float = 25.0,
           tol_deg: float = 0.15, poll_s: float = 0.1,
           min_samples: int = 4) -> dict:
    """Poll `read` until the joints stop changing, then report what they are.

    ``read`` is any zero-argument callable returning ``list[float]`` (or a
    ``(list[float], age_s)`` tuple). Every future measurement must go through
    this: reading once measures the transient, and the 2026-11-23 "coupling
    matrix" was exactly a transient read 1.6 s after the command.

    Returns ``{"joints", "settled", "elapsed_s", "quiet_s", "samples",
    "last_change_deg", "age_s"}``. ``settled=False`` means the timeout expired
    while the arm was still moving — an unfinished move, not a settled one.
    """
    started = time.time()
    samples: list[list[float]] = []
    ages: list[float] = []
    last_change = started
    last_change_deg = None
    while True:
        value = read()
        age = None
        if isinstance(value, tuple):
            value, age = value
        joints = [float(v) for v in value]
        if samples:
            step = max(abs(a - b) for a, b in zip(joints, samples[-1]))
            if step > tol_deg:
                last_change = time.time()
                last_change_deg = round(step, 3)
        samples.append(joints)
        if age is not None:
            ages.append(float(age))
        now = time.time()
        if (now - started >= quiet_s and len(samples) >= min_samples
                and now - last_change >= quiet_s):
            return {"joints": joints, "settled": True,
                    "elapsed_s": round(now - started, 2),
                    "quiet_s": round(now - last_change, 2),
                    "samples": len(samples),
                    "last_change_deg": last_change_deg,
                    "age_s": round(ages[-1], 3) if ages else None}
        if now - started > timeout_s:
            return {"joints": joints, "settled": False,
                    "elapsed_s": round(now - started, 2),
                    "quiet_s": round(now - last_change, 2),
                    "samples": len(samples),
                    "last_change_deg": last_change_deg,
                    "age_s": round(ages[-1], 3) if ages else None}
        time.sleep(poll_s)


def wait_for_motion(read, before: list[float], *, threshold_deg: float = 0.5,
                    timeout_s: float = 8.0, poll_s: float = 0.05) -> dict:
    """Wait until the measured joints move at least ``threshold_deg``.

    This is the assertion the 2026-11-23 change lacked: not "did the loop
    report success" but "did the arm actually go anywhere".
    """
    started = time.time()
    best = 0.0
    while time.time() - started <= timeout_s:
        value = read()
        joints = [float(v) for v in (value[0] if isinstance(value, tuple) else value)]
        best = max(best, max(abs(a - b) for a, b in zip(joints, before)))
        if best >= threshold_deg:
            return {"moved": True, "elapsed_s": round(time.time() - started, 2),
                    "max_deg": round(best, 3)}
        time.sleep(poll_s)
    return {"moved": False, "elapsed_s": round(time.time() - started, 2),
            "max_deg": round(best, 3)}


def require_fresh(bridge_url: str, *, max_age_s: float = 1.5,
                  timeout_s: float = 10.0, poll_s: float = 0.1) -> dict:
    """Block until the bridge's joint readback is actually recent.

    The Pi bridge polls the follower only in REMOTE and only every
    ``READ_EVERY`` tick (`live/pi_bridge/app.py`), and it does not poll at all in
    HOLD — a live HOLD bridge reported ``joints_age_s: 47327`` on 2026-11-24.
    Measuring a frozen snapshot is how "the arm never moved" can look like a
    clean settle, so every IRL measurement waits for a fresh sample first.
    """
    started = time.time()
    age = None
    while time.time() - started <= timeout_s:
        state = bridge_state(bridge_url)
        age = state.get("joints_age_s")
        if age is not None and float(age) <= max_age_s:
            return {"fresh": True, "age_s": round(float(age), 3),
                    "waited_s": round(time.time() - started, 2)}
        time.sleep(poll_s)
    return {"fresh": False, "age_s": None if age is None else round(float(age), 1),
            "waited_s": round(time.time() - started, 2)}
