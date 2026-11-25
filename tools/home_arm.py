#!/usr/bin/env python3
"""Drive the arm to its stored home pose and prove it arrived.

Every task that moves the arm starts here (AGENTS.md, 2026-11-25): home is the
one pose from which the wrist camera is known to see the board, so leaving an
arm parked at an experiment pose makes the next vision run report "no markers"
and look like a camera fault.

Home is `recovery_joints` in the console config - a stored joint pose, never an
IK target, so this works even when the solver is what is broken. The gripper is
never actuated: its current value is carried through.

    python tools/home_arm.py            # home and verify
    python tools/home_arm.py --check     # report only, do not move

Exit status 0 only when the tip ends within tolerance of board centre.
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
CONSOLE = "http://127.0.0.1:8080"
TOKEN_FILE = ROOT / "live" / "bridge_token.txt"
#: How close to board centre counts as "at home". The stored pose was derived
#: before the Phase 3 board delta moved the board frame, so the same joints now
#: read ~15 mm off centre in board coordinates; 20 mm accepts that without
#: accepting a genuinely different pose. Re-derive recovery_joints if the offset
#: grows (the value is printed every run so drift is visible).
CENTRE_TOLERANCE_MM = 20.0
HEIGHT_TOLERANCE_MM = 20.0


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


def tip() -> list[float]:
    return get("/keep-in").get("tip_mm") or [float("nan")] * 3


def report(prefix: str, tip_mm: list[float]) -> tuple[bool, str]:
    x, y, z = tip_mm
    lateral = (x ** 2 + y ** 2) ** 0.5
    ok = lateral <= CENTRE_TOLERANCE_MM and abs(z - 107.0) <= HEIGHT_TOLERANCE_MM
    return ok, (f"{prefix} tip ({x:.1f}, {y:.1f}, {z:.1f}) mm | "
                f"{lateral:.1f} mm from centre, z {z:.1f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="report only, do not move")
    ap.add_argument("--reason", default="pre-task home")
    args = ap.parse_args()

    state = get("/state").get("bridge") or {}
    if state.get("mode") != "REMOTE":
        print(f"bridge is {state.get('mode')}, needs REMOTE before homing")
        return 2

    before = tip()
    ok_before, line = report("before:", before)
    print("  " + line)

    if args.check:
        return 0 if ok_before else 1

    if ok_before:
        print("  already home; nothing to move")
        return 0

    result = post("/recover", {"reason": args.reason})
    if "http_error" in result:
        print(f"  /recover refused: {result.get('detail')}")
        return 1
    time.sleep(1.5)
    after = tip()
    ok_after, line = report("after: ", after)
    print("  " + line)
    legs = result.get("legs") or []
    print(f"  legs: {len(legs)}"
          + ("" if not legs else " | last error "
             f"{legs[-1].get('error_deg', '?')} deg"))
    return 0 if ok_after else 1


if __name__ == "__main__":
    sys.exit(main())
