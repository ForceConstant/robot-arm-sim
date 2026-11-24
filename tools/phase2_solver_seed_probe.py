#!/usr/bin/env python3
"""Phase 2, step 1b — can the SOLVER hand back its own seed?

Every injected readback fault produces a LARGE `error_deg` (67 deg), so none of
them matches the 2026-11-23 report of "accepted, small error, arm never moved".
The only code path that produces that exact shape is:

    target ≈ measured at tick 0  ->  `_stream_to` breaks immediately, sends
    zero commands, returns error_deg ~0, and the caller cannot tell it apart
    from a real arrival.

`_stream_to` is not where that goes wrong; it is handed a target that is where
the arm already is. So this tool asks the upstream question directly: for the
documented failing request — board (0,0,70) from the posture at board
(48,27,40) — does `solve_tip` return a pose at the requested point, or its own
seed?

It runs on the same chain + board frame the console uses, both solvers, and
prints the solved joints against the seed so a jump-to-seed is unmissable.

    robot-arm-sim python tools/phase2_solver_seed_probe.py
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
ARM = JOINTS[:5]


def board_frame() -> np.ndarray:
    spec = json.loads((ROOT / "live/board.json").read_text())
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = np.eye(4)
    frame[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]]
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    return frame


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start-square", default="48,27,40",
                    help="board mm the arm is really at, x,y,z")
    ap.add_argument("--target", default="0,0,70", help="board mm to go to")
    ap.add_argument("--out", default="reports/phase2_solver_seed.json")
    args = ap.parse_args()

    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from chess_sim.wrist_calibration import WristCalibration
    from solver import MoveSolver

    config = json.loads((ROOT / "live/live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    limits = {n: (float(np.degrees(calibration.chain[i + 1]["limits"][0])),
                  float(np.degrees(calibration.chain[i + 1]["limits"][1])))
              for i, n in enumerate(ARM)}
    chain = ArmKinematics(calibration, limits, board_frame())

    sx, sy, sz = (float(v) for v in args.start_square.split(","))
    tx, ty, tz = (float(v) for v in args.target.split(","))

    # The posture the arm is in at the start point, from the chain's own solve.
    start_goal, _ = chain.solve_tip(chain.board_point(sx, sy, sz),
                                    [0.0, -55.0, 72.0, 39.0, 0.0, 0.0],
                                    TIP_IN_CHAIN)
    seed = [float(start_goal["joints_deg"][j]) for j in JOINTS]
    seed_tip = (chain.to_board((chain.fk(seed) @ TIP_IN_CHAIN)[:3, 3]) * 1000.0)

    point = chain.board_point(tx, ty, tz)
    want_tip = chain.to_board(point) * 1000.0

    rows = []
    for label, use_mink in (("least_squares", False), ("mink", True)):
        solver = MoveSolver(chain, calibration=calibration,
                            tip_in_chain=TIP_IN_CHAIN, use_mink=use_mink)
        t0 = time.perf_counter()
        solved, goal = solver.solve_tip(point, seed, TIP_IN_CHAIN, 0.0)
        ms = (time.perf_counter() - t0) * 1000.0
        joints = [float(solved["joints_deg"][j]) for j in JOINTS]
        jump = max(abs(a - b) for a, b in zip(joints[:5], seed[:5]))
        achieved = (chain.to_board((chain.fk(joints) @ TIP_IN_CHAIN)[:3, 3]) * 1000.0)
        # How far the SOLVED pose is from where the arm actually is
        achieved_from_seed = float(np.linalg.norm(achieved - seed_tip))
        rows.append({
            "solver": label,
            "ms": round(ms, 1),
            "reported_position_error_mm": solved["position_error_mm"],
            "reported_approach_error_deg": solved["approach_error_deg"],
            "seed_joints": [round(v, 2) for v in seed[:5]],
            "solved_joints": [round(v, 2) for v in joints[:5]],
            "max_joint_jump_deg": round(jump, 2),
            "seed_tip_mm": [round(float(v), 1) for v in seed_tip],
            "achieved_tip_mm": [round(float(v), 1) for v in achieved],
            "wanted_tip_mm": [round(float(v), 1) for v in want_tip],
            # The failure this probe hunts: a solve that did not move the arm.
            "returned_seed": bool(jump < 1.0 and achieved_from_seed < 3.0),
            "achieved_vs_seed_mm": round(achieved_from_seed, 2),
        })
        print(f"{label:14} perr {solved['position_error_mm']:6.2f}mm  "
              f"appr {solved['approach_error_deg']:5.2f}°  "
              f"joint jump {jump:6.2f}°  returned_seed={rows[-1]['returned_seed']}")

    payload = {"tool": "tools/phase2_solver_seed_probe.py",
               "when": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "start_board_mm": [sx, sy, sz], "target_board_mm": [tx, ty, tz],
               "seed_tip_mm": [round(float(v), 1) for v in seed_tip],
               "want_tip_mm": [round(float(v), 1) for v in want_tip],
               "rows": rows}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=1) + "\n")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
