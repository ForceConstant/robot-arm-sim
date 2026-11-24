#!/usr/bin/env python3
"""Phase 2 — the flip, measured on the REAL console (sim bridge), both flags.

`tools/phase2_corner_check.py` showed that from one fixed seed BOTH solvers miss
the Phase 0 sim relocation endpoint `(-85,85,120)` by 25-32 mm. That means the
endpoint's earlier pass did not come from that seed — so the honest question is
not "which solver is better from my seed" but "does the deployed console, with
its own seed chain, still execute the move under each flag".

This boots the same sim bridge + `live/app.py` pair `tools/test_move_executes.py`
uses, twice: `ik_solver=least_squares` with the 20 mm gate, and `ik_solver=mink`
with the 3.0 mm gate. Each run parks then commands the corner, and records the
console's answer verbatim. It exists so the flip is accepted or rejected on the
real code path rather than on a probe.

    robot-arm-sim python tools/phase2_sim_flip.py
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from move_settle import MoveError, get_json, post_json, settle, joints_of  # noqa: E402
from test_move_executes import (ConsoleMover, SIM_LARGE, SIM_START,  # noqa: E402
                                free_port, wait_http)

HOME = [-13.0, -11.0, 25.0, 78.0, 67.0, 44.0]


def boot(workdir: Path, *, solver: str, gate: float) -> tuple[list, str, str]:
    bridge_port = free_port()
    console_port = free_port()
    ready = workdir / "bridge.pid"
    python = sys.executable
    cmd = [python, str(ROOT / "tools/sim_move_bridge.py"),
           "--port", str(bridge_port), "--rate-deg-s", "60",
           "--home=" + ",".join(str(v) for v in HOME),
           "--ready-file", str(ready)]
    log = open(workdir / "sim_bridge.log", "w")
    bridge = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
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
        "ik_solver": solver,
        "move_tolerance_mm": gate,
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
    return [console, bridge], url, f"http://127.0.0.1:{bridge_port}"


def run_one(workdir: Path, solver: str, gate: float) -> dict:
    procs, console_url, bridge_url = boot(workdir, solver=solver, gate=gate)
    mover = ConsoleMover(console_url)
    try:
        read = lambda: (joints_of((s := get_json(bridge_url + "/state"))),  # noqa: E731
                        s.get("joints_age_s"))
        park = post_json(console_url + "/move", dict(SIM_START), timeout=300.0)
        settle(read, quiet_s=0.6, timeout_s=20.0)
        park_after = read()[0]
        before_tip = get_json(console_url + "/keep-in").get("tip_mm")
        large = post_json(console_url + "/move", dict(SIM_LARGE), timeout=600.0)
        time.sleep(0.3)
        after_settle = settle(read, quiet_s=0.8, timeout_s=30.0)
        after = after_settle["joints"]
        after_tip = get_json(console_url + "/keep-in").get("tip_mm")
        travel = max(abs(a - b) for a, b in zip(after, park_after))
        return {"solver": solver, "gate_mm": gate,
                "park": {k: park.get(k) for k in ("error_deg", "http_error",
                                                  "detail", "requested")},
                "large": {k: large.get(k) for k in ("error_deg", "http_error",
                                                    "detail", "requested",
                                                    "commands", "travel_deg")},
                "large_refused": bool(large.get("http_error")),
                "large_max_joint_travel_deg": round(travel, 2),
                "large_settled": after_settle["settled"],
                "tip_before_mm": before_tip, "tip_after_mm": after_tip}
    finally:
        for proc in procs:
            proc.terminate()
        for proc in procs:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="reports/phase2_sim_flip.json")
    args = ap.parse_args()
    results = []
    # THREE arms, not two: the flip bundles a solver change with a gate change,
    # so the fixture's fate has to be attributed to one or the other. If the
    # 3.0 mm gate refuses it on BOTH solvers, the endpoint is not solver-grade
    # and the fixture is what must change — not the flip.
    arms = [("least_squares", 20.0), ("least_squares", 3.0), ("mink", 3.0)]
    for solver, gate in arms:
        workdir = Path(tempfile.mkdtemp(prefix=f"flip_{solver}_{int(gate)}_"))
        print(f"\n=== {solver} @ {gate} mm  ({workdir})", flush=True)
        try:
            result = run_one(workdir, solver, gate)
        except MoveError as exc:
            result = {"solver": solver, "gate_mm": gate, "error": str(exc)}
        results.append(result)
        print(json.dumps(result, indent=1), flush=True)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    payload = {"tool": "tools/phase2_sim_flip.py",
               "start_mm": list(SIM_START.values())[1:],
               "large_mm": list(SIM_LARGE.values())[1:],
               "results": results}
    Path(args.out).write_text(json.dumps(payload, indent=1) + "\n")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
