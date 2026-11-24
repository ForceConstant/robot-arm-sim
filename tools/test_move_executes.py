#!/usr/bin/env python3
"""Phase 0 — does a commanded move actually MOVE the arm?

The 2026-11-23 regression (commit 9252a5b, reverted 9aa123c) closed the move loop
on measured joints. Tracking improved (2.08 -> 0.69 deg) but a large relocation
was then *accepted* (perr 1.5 mm), reported a small error, and the arm never
moved. Nothing in the tree asserted that the joints changed — every existing
check read the console's own reply, which is exactly the thing that lied.

This harness asserts the physical fact instead:

  1. read the joints, and wait for them to go quiet (`move_settle.settle`)
  2. command the move on a worker thread (the console blocks while moving)
  3. watch the measured joints until they move, or fail on a timeout
  4. let them settle again and report commanded vs measured per joint

A LARGE relocation (>100 mm of tool travel, >=2 joints moving a lot) is the case
that broke; small steps are included because they are the case that looked fine
and therefore proved nothing.

Usage
-----
    # sim, self-contained: starts a sim bridge + a console, runs every case
    .venv/bin/python tools/test_move_executes.py --sim

    # prove the harness can fail: sim mechanism that accepts and ignores big moves
    .venv/bin/python tools/test_move_executes.py --sim --stall-above 25

    # IRL: dry-run first, then one relocation (REMOTE, keep-in respected)
    .venv/bin/python tools/test_move_executes.py --irl --dry-run
    .venv/bin/python tools/test_move_executes.py --irl --case large

Exit status is 0 only when every executed case moved and settled. A refused
case (409 from the console: unreachable pose, keep-in, wrist window) is reported
as ``refused`` and is NOT a pass.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from move_settle import (JOINTS, MoveError, deltas, bridge_state, distance_mm,  # noqa: E402
                         get_json, joints_of, post_json, settle, tip_mm,
                         require_fresh, wait_for_motion)

# A large relocation across the playing surface.  Both ends are inside the
# keep-in box (jaw tip, board frame mm) and are real solved poses (see
# reports/phase0_move_verification_*.md), not joint guesses.
#
# The case is measured against a DECLARED start pose, not "whatever the arm was
# doing": on the first IRL run the arm had already been left at the far corner
# by a probe, so the same accepted target travelled only 89.9 mm and the
# relocation bar failed. The start is now commanded first, so the case means the
# same thing every run.
IRL_START = {"mode": "xyz", "x_mm": 65.0, "y_mm": 70.0, "z_mm": 110.0}
# Corner to corner, not centre to corner: a solved landing sits up to ~10 mm off
# nominal (measured 2026-11-24), which ate a 100 mm bar that started at the
# centre. 163 mm nominal leaves room for that error.
# Both ends solve on the SAME wrist_roll branch (87.56) — the -x, +y corner
# solves at -2.44 and 67.6, and a roll-branch change is not a move this test is
# meant to make (AGENTS.md: prefer targets that keep the current branch).
IRL_LARGE = {"mode": "xyz", "x_mm": -90.0, "y_mm": 20.0, "z_mm": 110.0}
# h1 is the far-edge square the Phase 1 approach trade-off is worst on (mink
# 7.35 deg of approach error, inside the 8 deg gate by 0.65 deg). It is in the
# IRL verification set precisely because it sits closest to that edge.
IRL_EDGE = {"name": "edge_h1_approach",
            "mode": "xyz", "x_mm": 98.0, "y_mm": -98.0, "z_mm": 60.0}
# The sim reaches further than the real arm, so the sim case keeps a wider
# sweep.  NOTE (Phase 2): the original sim endpoint (-85,85,120) is NOT
# solver-grade — it solves at 11.5 mm on least_squares and 25.05 mm/9.01 deg on
# mink, i.e. it only ever "passed" because the old 20 mm gate laundered an
# 11.5 mm miss.  Under the 3.0 mm gate BOTH solvers refuse it
# (reports/phase2_sim_flip.json), so it is replaced by a far-corner target that
# is genuinely solver-grade (mink 0.24 mm / 0.83 deg) and still clears the
# relocation bar (>100 mm tip travel, >=2 moving joints, >=60 deg).
SIM_START = {"mode": "xyz", "x_mm": 0.0, "y_mm": 0.0, "z_mm": 70.0}
SIM_LARGE = {"mode": "xyz", "x_mm": 95.0, "y_mm": 70.0, "z_mm": 110.0}
SMALL = [
    # Body key is ``delta``: the console's /jog reads ``body["delta"]`` and
    # silently defaults a missing field to 0.0, which returns error_deg 0.0 and
    # moves nothing — the exact "reported success" shape this harness exists to
    # catch. (Found the hard way on the first sim run.)
    {"name": "small_step_pan", "mode": "jog", "joint": "shoulder_pan", "delta": 6.0},
    {"name": "small_step_pan_back", "mode": "jog", "joint": "shoulder_pan", "delta": -6.0},
    # -5, not +5: the parked pose leaves wrist_flex at ~97.5 deg against a
    # 98.79 deg stop, so a positive step is refused (correctly) as out of limits.
    # Step away from the stop so the case exercises actuation, not clamping.
    {"name": "small_step_wrist", "mode": "jog", "joint": "wrist_flex", "delta": -5.0},
]


# --------------------------------------------------------------------- mover
class _StopCases(Exception):
    """Abort the remaining cases without escaping the cleanup block."""


class ConsoleMover:
    """Command moves through the console HTTP API — the path under test."""

    def __init__(self, console: str, timeout: float = 150.0) -> None:
        self.console = console.rstrip("/")
        self.timeout = timeout

    def move(self, body: dict) -> dict:
        return post_json(self.console + "/move", body, timeout=self.timeout)

    def jog(self, joint: str, delta_deg: float) -> dict:
        return post_json(self.console + "/jog",
                         {"joint": joint, "delta": float(delta_deg)},
                         timeout=self.timeout)

    def set_mode(self, mode: str) -> dict:
        return post_json(self.console + "/mode", {"mode": mode}, timeout=20.0)

    def arm(self) -> dict:
        return post_json(self.console + "/arm", {"confirm": True}, timeout=20.0)

    def state(self) -> dict:
        return get_json(self.console + "/state", timeout=15.0)

    def keep_in(self) -> dict:
        return get_json(self.console + "/keep-in", timeout=10.0)


# ---------------------------------------------------------------------- case
def run_case(name: str, command: dict, mover: ConsoleMover, bridge: str,
             console: str, *, motion_threshold_deg: float = 1.0,
             move_timeout_s: float = 10.0, settle_timeout_s: float = 30.0,
             dry_run: bool = False, require_fresh_reads: bool = False) -> dict:
    """Command one move and prove the arm moved. Returns the case record."""
    read = lambda: _read(bridge)  # noqa: E731

    freshness = {"fresh": None, "age_s": None, "skipped": True}
    if require_fresh_reads:
        freshness = require_fresh(bridge)
    record = {"case": name, "command": command, "freshness": freshness,
              # Refuse to measure a frozen snapshot: a stale readback makes
              # "did it move" unanswerable, and silently passing on it is the
              # bug this whole harness exists to prevent.
              "ok": False if (require_fresh_reads and not freshness["fresh"]) else None}
    if require_fresh_reads and not freshness["fresh"]:
        record["blocked"] = ("bridge joint readback is stale "
                             f"(age {freshness['age_s']} s); refusing to measure")
        record["ok"] = False
        return record

    before_settle = settle(read, timeout_s=settle_timeout_s)
    before = before_settle["joints"]
    tip_before = tip_mm(console)
    record.update(before_deg=[round(v, 3) for v in before],
                  tip_before_mm=tip_before,
                  before_settled=before_settle["settled"],
                  before_age_s=before_settle.get("age_s"))

    if dry_run:
        body = dict(command)
        body["dry_run"] = True
        reply = _post(mover, body)
        record.update(dry_run=True, reply=reply,
                      ok=not reply.get("http_error") and not reply.get("detail"))
        record["max_joint_delta_deg"] = reply.get("max_joint_delta_deg")
        record["target_joints"] = reply.get("target_joints")
        return record

    box: dict = {}

    def work() -> None:
        try:
            box["reply"] = _post(mover, command)
        except Exception as exc:                      # noqa: BLE001
            box["error"] = f"{type(exc).__name__}: {exc}"

    thread = threading.Thread(target=work, daemon=True)
    started = time.time()
    thread.start()

    # Watch the encoders while the console is still inside its stream loop.
    motion = watch_motion(read, before, done=lambda: not thread.is_alive(),
                          threshold_deg=motion_threshold_deg,
                          timeout_s=move_timeout_s)
    thread.join(timeout=max(0.0, move_timeout_s))
    record["motion"] = motion
    record["reply"] = box.get("reply", {"error": box.get("error")})
    record["reply_wall_s"] = round(time.time() - started, 2)

    after_settle = settle(read, timeout_s=settle_timeout_s)
    after = after_settle["joints"]
    record.update(after_deg=[round(v, 3) for v in after],
                  tip_after_mm=tip_mm(console),
                  after_settled=after_settle["settled"],
                  settle_s=after_settle["elapsed_s"],
                  movement=deltas(before, after))

    reply = record["reply"] or {}
    record["refused"] = bool(reply.get("http_error"))
    record["reported_error_deg"] = reply.get("error_deg")
    record["tip_travel_mm"] = distance_mm(record["tip_before_mm"], record["tip_after_mm"])

    travelled = record["movement"]["max_abs_deg"]
    # A move that was never executed reports a tiny error and moves nothing.
    record["ok"] = bool(motion["moved"]) and travelled >= motion_threshold_deg \
        and after_settle["settled"] and not record["refused"]

    # The large case must be a real RELOCATION, not one joint twitching.
    #
    # Bar: >100 mm of tool travel, >=2 arm joints that each moved >=5 deg, and
    # >=60 deg of total arm-joint travel.
    #
    # The first version of this bar demanded two joints each >=25 deg and it
    # REJECTED A GOOD MOVE: a pan-dominant sweep across the board is a
    # legitimate large relocation (measured 2026-11-24: shoulder_pan 68.2 deg,
    # shoulder_lift 8.6 deg, 154.4 mm of tip travel). The bar is about distance
    # covered, not about spreading the motion evenly over the joints.
    #
    # "park" is a prepositioning move and is deliberately exempt, so match the
    # exact case name rather than a substring.
    if name == "large_relocation_large":
        tip_travel = record["tip_travel_mm"]
        per_joint = record["movement"]["per_joint_deg"][:5]   # arm joints only
        moving_joints = sum(1 for v in per_joint if abs(v) >= 5.0)
        joint_travel = round(sum(abs(v) for v in per_joint), 2)
        record["relocation"] = {"min_tip_mm": 100.0,
                                "min_moving_joints": 2,
                                "min_joint_travel_deg": 60.0,
                                "tip_mm": tip_travel,
                                "moving_joints": moving_joints,
                                "joint_travel_deg": joint_travel,
                                "satisfied": bool(tip_travel is not None
                                                  and tip_travel >= 100.0
                                                  and moving_joints >= 2
                                                  and joint_travel >= 60.0)}
        record["ok"] = record["ok"] and record["relocation"]["satisfied"]
    return record


def _read(bridge: str):
    state = bridge_state(bridge)
    return joints_of(state), state.get("joints_age_s")


def watch_motion(read, before: list[float], *, done, threshold_deg: float,
                 timeout_s: float, poll_s: float = 0.05) -> dict:
    """Watch for joint motion, stopping early once the command has returned.

    A cold `solve_tip` on the real console took ~100 s on 2026-11-24, so the
    motion window has to be generous -- but when a move is REFUSED it returns at
    once and waiting the full window wastes minutes. So: stop as soon as motion
    is seen, or the worker is done and has stopped moving.
    """
    started = time.time()
    best = 0.0
    quiet_after_done = 0.0
    while time.time() - started <= timeout_s:
        value = read()
        joints = [float(v) for v in (value[0] if isinstance(value, tuple) else value)]
        best = max(best, max(abs(a - b) for a, b in zip(joints, before)))
        if best >= threshold_deg:
            return {"moved": True, "elapsed_s": round(time.time() - started, 2),
                    "max_deg": round(best, 3), "before_return": not done()}
        if done():
            # Give the mechanism a moment to react to the last command, then
            # stop: nothing more is coming.
            quiet_after_done += poll_s
            if quiet_after_done > 1.0:
                return {"moved": False, "elapsed_s": round(time.time() - started, 2),
                        "max_deg": round(best, 3), "after_return": True}
        time.sleep(poll_s)
    return {"moved": False, "elapsed_s": round(time.time() - started, 2),
            "max_deg": round(best, 3), "timeout": True}


def _post(mover: ConsoleMover, body: dict) -> dict:
    if body.get("mode") == "jog" and "joint" in body and "delta" in body:
        return mover.jog(body["joint"], body["delta"])
    return mover.move(body)


def _normalise(case: dict) -> dict:
    """A case may carry a name/mode pair; split the harness bookkeeping out."""
    command = dict(case.pop("command", {})) or {
        k: v for k, v in case.items() if k != "name"}
    return {"name": case.get("name") or command.get("mode", "move"),
            "command": command}


# ------------------------------------------------------------------ sim glue
def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_http(url: str, timeout: float = 60.0) -> None:
    started = time.time()
    while time.time() - started < timeout:
        try:
            get_json(url, timeout=3.0)
            return
        except MoveError:
            time.sleep(0.3)
    raise SystemExit(f"timed out waiting for {url}")


def start_sim(*, stall_above: float | None, rate: float, home: list[float],
              workdir: Path, stale_readback: bool = False,
              sag: str = "", deadband: float = 0.0) -> tuple[subprocess.Popen, str]:
    """Boot the sim bridge plus a console pointed at it. Returns (console proc, url)."""
    bridge_port = free_port()
    console_port = free_port()
    ready = workdir / "bridge.pid"
    python = sys.executable
    cmd = [python, str(ROOT / "tools/sim_move_bridge.py"),
           "--port", str(bridge_port), "--rate-deg-s", str(rate),
           # "=" form: a value starting with "-13.0,..." otherwise parses as a flag
           "--home=" + ",".join(str(v) for v in home),
           "--ready-file", str(ready)]
    if stall_above is not None:
        cmd += ["--stall-above", str(stall_above)]
    if stale_readback:
        cmd += ["--stale-readback"]
    if sag:
        for item in sag.split(","):
            if item.strip():
                cmd += ["--sag", item.strip()]
    if deadband:
        cmd += ["--deadband", str(deadband)]
    log = open(workdir / "sim_bridge.log", "w")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            cwd=str(ROOT), env=dict(os.environ))
    wait_http(f"http://127.0.0.1:{bridge_port}/state", timeout=120.0)

    config = {
        "bridge_url": f"http://127.0.0.1:{bridge_port}",
        "token_file": str(workdir / "token.txt"),
        "calibration": str(ROOT / ".private/phase11d/calibration.json"),
        "plan": str(ROOT / "chess_sim/assets/marker_scan_plan_charuco.json"),
        "board_file": str(ROOT / "live/board.json"),
        "wrist_file": str(ROOT / "live/wrist_calibration.json"),
        "poses_file": str(workdir / "poses.json"),
        "capture_dir": str(workdir / "captures"),
        "tip_center_file": str(workdir / "tip_center.json"),
        "aim_file": str(workdir / "aim.json"),
        "target_overlay": False,
        "auth_required": False,
        "settle_s": 0.4,
        "port": console_port,
    }
    (workdir / "token.txt").write_text("test-token")
    (workdir / "live_sim.json").write_text(json.dumps(config))
    env = dict(os.environ, LIVE_CONFIG=str(workdir / "live_sim.json"),
               PYTHONPATH=os.pathsep.join((str(ROOT), str(ROOT / "live"))))
    log2 = open(workdir / "console.log", "w")
    console = subprocess.Popen([python, str(ROOT / "live/app.py")],
                               stdout=log2, stderr=subprocess.STDOUT,
                               cwd=str(ROOT), env=env)
    url = f"http://127.0.0.1:{console_port}"
    wait_http(url + "/state", timeout=180.0)
    return console, url, proc


# ---------------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="boot a sim bridge + console")
    ap.add_argument("--irl", action="store_true", help="drive the real bridge/console")
    ap.add_argument("--console", default="http://192.168.1.171:8080")
    ap.add_argument("--bridge", default="http://192.168.1.167:8100")
    ap.add_argument("--case", default="all",
                    help="all | large | small (comma separated)")
    ap.add_argument("--dry-run", action="store_true",
                    help="solve and report, do not move")
    ap.add_argument("--stall-above", type=float, default=None,
                    help="sim only: accept-and-ignore moves bigger than this")
    ap.add_argument("--stale-readback", action="store_true",
                    help="sim only: freeze /state.joints (reproduces 2026-11-23)")
    ap.add_argument("--rate", type=float, default=60.0, help="sim servo deg/s")
    ap.add_argument("--sag", default="", help="sim only: joint=deg,comma separated")
    ap.add_argument("--deadband", type=float, default=0.0,
                    help="sim only: ignore joint steps smaller than this many deg")
    ap.add_argument("--motion-threshold-deg", type=float, default=1.0)
    # A cold IRL solve took ~100 s on 2026-11-24; the window must outlast the
    # solve plus the travel. Sim answers in well under a second, so this only
    # costs time on a real refusal.
    ap.add_argument("--move-timeout-s", type=float, default=180.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    if not args.sim and not args.irl:
        ap.error("choose --sim or --irl")

    selected = {item.strip() for item in args.case.split(",") if item.strip()}
    if "all" in selected:
        selected = {"large", "small"}

    procs: list[subprocess.Popen] = []
    if args.sim:
        workdir = Path(tempfile.mkdtemp(prefix="move_verify_"))
        print(f"sim workdir {workdir}", flush=True)
        console, url, sim_proc = start_sim(stall_above=args.stall_above,
                                           rate=args.rate,
                                           home=[-13.0, -11.0, 25.0, 78.0, 67.0, 44.0],
                                           workdir=workdir,
                                           stale_readback=args.stale_readback,
                                           sag=args.sag, deadband=args.deadband)
        procs = [console, sim_proc]
        console_url = url
        # Read the joints from the bridge the console is actually driving, so the
        # measurement is the mechanism's, not the console's opinion of it.
        bridge_url = _bridge_of(workdir)
    else:
        console_url = args.console
        bridge_url = args.bridge

    mover = ConsoleMover(console_url)
    records: list[dict] = []
    exit_code = 0
    try:
        state = mover.state()
        bridge_state_now = state.get("bridge") or {}
        print(f"bridge mode={bridge_state_now.get('mode')} "
              f"armed={bridge_state_now.get('armed')} "
              f"joints={bridge_state_now.get('joints')}", flush=True)
        if args.irl and (bridge_state_now.get("mode") != "REMOTE"
                         or not bridge_state_now.get("armed")):
            # HOLD -> REMOTE is the documented pre-move transition, and the
            # bridge only accepts /command in REMOTE+armed. Arm, switch, then
            # re-read: never move on an assumed state.
            print("bridge is not REMOTE+armed; arming and switching", flush=True)
            if bridge_state_now.get("mode") == "FAULT":
                raise SystemExit("refusing to move: bridge is in FAULT; "
                                 "clear it by hand first")
            mover.arm()
            mover.set_mode("REMOTE")
            time.sleep(1.2)
            bridge_state_now = (mover.state().get("bridge") or {})
            print(f"after switch: mode={bridge_state_now.get('mode')} "
                  f"armed={bridge_state_now.get('armed')} "
                  f"fault={bridge_state_now.get('fault')}", flush=True)
            if (bridge_state_now.get("mode") != "REMOTE"
                    or not bridge_state_now.get("armed")):
                raise SystemExit("refusing to move: bridge did not reach REMOTE+armed")

        cases: list[dict] = []
        if "large" in selected:
            # Park at a declared start first, so "large relocation" means the
            # same distance every run instead of "wherever the last probe left
            # the arm".  Then command the far corner: the sim reaches a wider
            # corner than the real arm (which refused the sim endpoint at
            # 24.4 mm on 2026-11-24), but both clear >100 mm and >=2 joints.
            target = dict(SIM_LARGE) if args.sim else dict(IRL_LARGE)
            park = dict(SIM_START) if args.sim else dict(IRL_START)
            park_record = run_case("park_at_start", park, mover,
                                   bridge_url, console_url,
                                   motion_threshold_deg=args.motion_threshold_deg,
                                   move_timeout_s=args.move_timeout_s,
                                   dry_run=args.dry_run,
                                   require_fresh_reads=args.irl and not args.dry_run)
            records.append(park_record)
            print(json.dumps({k: park_record[k] for k in (
                "case", "ok", "movement", "tip_after_mm") if k in park_record},
                indent=1), flush=True)
            if not park_record.get("ok"):
                print("start pose not reached; the relocation case cannot mean "
                      "anything, so it is not attempted", flush=True)
                exit_code = 1
                raise _StopCases()
            cases.append({"name": "large_relocation_large",
                          "command": target})
        if "small" in selected:
            for small in SMALL:
                cases.append({"name": small.pop("name"), "command": dict(small)})
        if "edge" in selected:
            cases.append({"name": IRL_EDGE["name"],
                          "command": {k: v for k, v in IRL_EDGE.items()
                                      if k != "name"}})

        for case in cases:
            normalised = _normalise(dict(case))
            name, command = normalised["name"], normalised["command"]
            print(f"\n=== {name}: {command}", flush=True)
            record = run_case(name, command, mover, bridge_url, console_url,
                              motion_threshold_deg=args.motion_threshold_deg,
                              move_timeout_s=args.move_timeout_s,
                              dry_run=args.dry_run,
                              require_fresh_reads=args.irl and not args.dry_run)
            records.append(record)
            print(json.dumps({k: record[k] for k in (
                "case", "ok", "refused", "movement", "motion", "reported_error_deg",
                "tip_travel_mm", "settle_s") if k in record}, indent=1), flush=True)
            if not record.get("ok"):
                exit_code = 1
    except _StopCases:
        exit_code = 1
    finally:
        if args.irl and records:
            # Even a dry run leaves the bridge in REMOTE (it has to be, to
            # solve), so put it back where it was found.
            try:
                post_json(console_url + "/mode", {"mode": "HOLD"}, timeout=20.0)
                print("bridge set back to HOLD", flush=True)
            except MoveError as exc:
                print(f"could not restore HOLD: {exc}", flush=True)
        for proc in procs:
            proc.terminate()
        if procs:
            for proc in procs:
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()

    payload = {"mode": "sim" if args.sim else "irl", "dry_run": args.dry_run,
               "console": console_url, "bridge": bridge_url,
               "stall_above": args.stall_above,
               "stale_readback": args.stale_readback, "sag": args.sag,
               "deadband": args.deadband,
               "passed": exit_code == 0, "cases": records,
               "when": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    out = args.out or str(ROOT / ("reports/phase0_move_executes_sim.json" if args.sim
                                  else "reports/phase0_move_executes_irl.json"))
    Path(out).write_text(json.dumps(payload, indent=1))
    print(f"\n{'PASS' if exit_code == 0 else 'FAIL'} -> {out}", flush=True)
    return exit_code


def _bridge_of(workdir: Path) -> str:
    config = json.loads((workdir / "live_sim.json").read_text())
    return config["bridge_url"]


if __name__ == "__main__":
    raise SystemExit(main())
