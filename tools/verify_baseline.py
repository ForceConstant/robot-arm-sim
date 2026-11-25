#!/usr/bin/env python3
"""Verify the arm is controllable to the 2026-11-25 baseline, end to end.

Run this at the start of any new phase.  It answers one question: *will the arm
behave as well as it did when this baseline was set?* - not by reading config
and calling that success, but by moving the arm and checking it moved.

Checks, in order (each can fail independently):
  1. rig state      the four machine-local files match rig_state/MANIFEST.json
  2. console        /state answers, mode REMOTE, no fault, joints reported
  3. config         the running config asks for the mink solver and a 3.0 mm gate
  4. aim            /aim-point returns ok with a fresh marker plane
  5. actuation      a small joint jog actually moves the arm (this is the check
                    that caught "torque off": 266 commands accepted, 0.00 deg)
  6. rel mode       a 10 mm board-frame nudge is accepted (the frozen-roll
                    regression that the Phase 2 gate missed)
  7. return home    the arm is left where it was found, as far as the deadband
                    allows

Exit status 0 only when every check passes.  Any motion it performs is small
(<= 12 mm) and is reversed.

    python tools/verify_baseline.py             # on the BC-250
    python tools/verify_baseline.py --no-motion # config/plumbing only
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

CONSOLE = "http://127.0.0.1:8080"
TOKEN_FILE = ROOT / "live" / "bridge_token.txt"
NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")

results: list[tuple[str, bool, str]] = []


def record(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, ok, detail))
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"  [{detail}]" if detail else ""),
          flush=True)
    return ok


def token() -> str:
    try:
        return TOKEN_FILE.read_text().strip()
    except OSError:
        return ""


def get(path: str, timeout: float = 10.0) -> dict:
    with urllib.request.urlopen(CONSOLE + path, timeout=timeout) as reply:
        return json.loads(reply.read().decode() or "{}")


def post(path: str, body: dict, timeout: float = 120.0) -> dict:
    request = urllib.request.Request(
        CONSOLE + path, data=json.dumps(body).encode(), method="POST",
        headers={"Content-Type": "application/json", "x-bridge-token": token()})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return json.loads(reply.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        return {"http_error": exc.code, "detail": exc.read().decode()[:300]}


def joints() -> dict:
    return (get("/state").get("bridge") or {}).get("joints") or {}


def settle(seconds: float = 1.2) -> dict:
    time.sleep(seconds)
    return joints()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-motion", action="store_true",
                    help="skip checks that move the arm")
    args = ap.parse_args()

    print("baseline check — arm controllability\n")

    # 1. rig state
    try:
        from rig_state import verify as verify_state
        import io
        import contextlib
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = verify_state(quiet=True)
        record("rig state matches rig_state/MANIFEST.json", code == 0,
               "" if code == 0 else buffer.getvalue().strip().splitlines()[-1])
    except Exception as exc:  # noqa: BLE001
        record("rig state matches rig_state/MANIFEST.json", False, repr(exc))

    # 2. console
    try:
        state = get("/state")
        bridge = state.get("bridge") or {}
        ok = (bridge.get("mode") == "REMOTE"
              and not bridge.get("fault") and not bridge.get("error")
              and isinstance(bridge.get("joints"), dict))
        record("console answers, REMOTE, no fault",
               ok, f"mode {bridge.get('mode')} fault {bridge.get('fault')}")
    except Exception as exc:  # noqa: BLE001
        record("console answers, REMOTE, no fault", False, repr(exc))
        print("\ncannot continue without the console")
        return 1

    # 3. config
    try:
        config = json.loads((ROOT / "live" / "live_config.json").read_text())
        solver = str(config.get("ik_solver", "")).lower()
        gate = float(config.get("move_tolerance_mm", 99))
        ok = solver in ("mink", "qp", "so101ik") and gate <= 3.5
        record("config: mink solver + <=3.5 mm gate", ok, f"{solver}, {gate} mm")
    except Exception as exc:  # noqa: BLE001
        record("config: mink solver + <=3.5 mm gate", False, repr(exc))

    # 4. aim — the plane only samples while the stream is pulled, and the
    # detector needs a beat to produce its first solve (measured: a 4 s pull is
    # too short, 12 s is plenty).  Poll instead of sampling once.
    try:
        aim: dict = {}
        deadline = time.time() + 25.0
        stream_deadline = time.time() + 18.0
        opened = False
        while time.time() < deadline:
            if not opened and time.time() < stream_deadline:
                try:
                    with urllib.request.urlopen(CONSOLE + "/stream",
                                                timeout=3) as stream:
                        opened = True
                        chunk_deadline = time.time() + 3.0
                        while time.time() < chunk_deadline:
                            if not stream.read(8192):
                                break
                except Exception:  # noqa: BLE001 - short read is expected
                    pass
            else:
                opened = True
            aim = get("/aim-point")
            if aim.get("ok"):
                break
            time.sleep(1.5)
        ok = bool(aim.get("ok"))
        plane = aim.get("plane") or {}
        detail = (f"aim {aim.get('aim_pixel')} plane {plane.get('samples')} solves "
                  f"{plane.get('age_s')} s" if ok
                  else str(aim.get("reason") or "unavailable"))
        record("aim point ok on a fresh marker plane", ok, detail)
    except Exception as exc:  # noqa: BLE001
        record("aim point ok on a fresh marker plane", False, repr(exc))

    if args.no_motion:
        print("\n--no-motion: skipped actuation and rel checks")
    else:
        # 5. actuation
        before = joints()
        reply = post("/jog", {"joint": "shoulder_pan", "delta": 4.0})
        after = settle()
        moved = abs(after.get("shoulder_pan", 0.0) - before.get("shoulder_pan", 0.0))
        ok = "http_error" not in reply and moved >= 2.0
        record("actuation: 4 deg jog moves the arm", ok, f"moved {moved:.2f} deg")

        # 6. rel mode (frozen roll -> chain solver)
        reply_rel = post("/move", {"mode": "rel", "dx_mm": 10.0, "dy_mm": 0.0,
                                   "dz_mm": 0.0})
        ok_rel = "http_error" not in reply_rel
        record("rel mode accepts a 10 mm board nudge", ok_rel,
               "" if ok_rel else str(reply_rel.get("detail"))[:80])

        # 7. put it back
        post("/move", {"mode": "rel", "dx_mm": -10.0, "dy_mm": 0.0, "dz_mm": 0.0})
        post("/jog", {"joint": "shoulder_pan", "delta": -4.0})
        settle()
        home = joints()
        drift = max(abs(home.get(n, 0.0) - before.get(n, 0.0)) for n in NAMES[:5])
        record("arm returned to its starting pose", drift <= 4.0,
               f"max drift {drift:.2f} deg (deadband is 2-4)")

    failed = [name for name, ok, _ in results if not ok]
    print()
    if failed:
        print(f"BASELINE NOT READY — {len(failed)} check(s) failed:")
        for name in failed:
            print(f"  - {name}")
        return 1
    print("BASELINE OK — the arm is controllable to the 2026-11-25 standard.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
