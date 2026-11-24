#!/usr/bin/env python3
"""Phase 2 — does the mink flip make any *previously executing* target refuse?

The Phase 0 sim relocation endpoint `(-85, 85, 120)` is at the sim arm's far
corner. Under the flip the console refused it (25.05 mm / 9.008 deg). Before
concluding "mink regressed reach", this measures the honest question:

    from the SAME seed, on the SAME model, with the SAME limits,
    what does each solver report for that target?

If both are far off, the old endpoint was only "reachable" because the 20 mm
gate was laundering a bad solve — which is exactly what Phase 1's gate
decomposition said the 20 mm gate does. If LS is genuinely inside 20 mm and mink
is not, that is a real regression and the flip must not land.

    robot-arm-sim python tools/phase2_corner_check.py
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--park", default="0,0,70")
    ap.add_argument("--targets", default="-85,85,120;-90,20,110;-70,85,110;-85,-85,120")
    ap.add_argument("--out", default="reports/phase2_corner_check.json")
    args = ap.parse_args()

    import mujoco
    from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
    from chess_sim.env import ChessSimEnv
    from chess_sim.wrist_calibration import WristCalibration
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from solver import MoveSolver

    config = json.loads((ROOT / "live/live_config.json").read_text()) if \
        (ROOT / "live/live_config.json").exists() else {}
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    limits = {n: (float(np.degrees(calibration.chain[i + 1]["limits"][0])),
                  float(np.degrees(calibration.chain[i + 1]["limits"][1])))
              for i, n in enumerate(ARM)}
    live_dir = ROOT / "live"
    spec = json.loads((live_dir / "board.json").read_text())
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = np.eye(4)
    frame[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    chain = ArmKinematics(calibration, limits, frame)
    ls = MoveSolver(chain, calibration=calibration, tip_in_chain=TIP_IN_CHAIN,
                    use_mink=False)
    mink = MoveSolver(chain, calibration=calibration, tip_in_chain=TIP_IN_CHAIN,
                      use_mink=True)
    mink.sync_limits({n: list(v) for n, v in limits.items()})

    # The sim arm, so the SEED is the posture the sim really holds after park.
    board = BoardConfig(face="charuco", yaw=math.pi)
    env = ChessSimEnv(EnvConfig(board=board,
                               appearance=AppearanceConfig(board_markers=False)))
    env.reset("8/8/8/8/8/8/8/8")
    model, data = env.model, env.data
    px, py, pz = (float(v) for v in args.park.split(","))
    park_goal, _ = chain.solve_tip(chain.board_point(px, py, pz),
                                   [0.0, -55.0, 72.0, 39.0, 0.0, 0.0], TIP_IN_CHAIN)
    park_q = [float(park_goal["joints_deg"][j]) for j in JOINTS]
    data.qpos[env._joint_qpos] = np.radians(park_q)
    mujoco.mj_forward(model, data)
    seed = [float(data.qpos[env._joint_qpos[i]]) for i in range(6)]
    seed_tip = chain.to_board((chain.fk(seed) @ TIP_IN_CHAIN)[:3, 3]) * 1000.0

    rows = []
    for spec_xyz in args.targets.split(";"):
        if not spec_xyz.strip():
            continue
        tx, ty, tz = (float(v) for v in spec_xyz.split(","))
        point = chain.board_point(tx, ty, tz)
        row = {"target_mm": [tx, ty, tz], "seed_tip_mm":
               [round(float(v), 1) for v in seed_tip]}
        for label, solver in (("least_squares", ls), ("mink", mink)):
            t0 = time.perf_counter()
            solved, _goal = solver.solve_tip(point, seed, TIP_IN_CHAIN, 0.0)
            row[label] = {
                "position_error_mm": round(float(solved["position_error_mm"]), 3),
                "approach_error_deg": round(float(solved["approach_error_deg"]), 3),
                "ms": round((time.perf_counter() - t0) * 1000.0, 1),
                "pass20": bool(solved["position_error_mm"] <= 20.0
                               and solved["approach_error_deg"] <= 8.0),
                "pass3": bool(solved["position_error_mm"] <= 3.0
                              and solved["approach_error_deg"] <= 8.0),
            }
        row["regression_at_20mm"] = bool(row["least_squares"]["pass20"]
                                         and not row["mink"]["pass20"])
        row["both_off_gt_20mm"] = bool(not row["least_squares"]["pass20"]
                                        and not row["mink"]["pass20"])
        rows.append(row)
        print(f"{str(row['target_mm']):22}  LS {row['least_squares']['position_error_mm']:6.2f}/"
              f"{row['least_squares']['approach_error_deg']:5.2f}° "
              f"{'P20' if row['least_squares']['pass20'] else '   '}   "
              f"mink {row['mink']['position_error_mm']:5.2f}/"
              f"{row['mink']['approach_error_deg']:5.2f}° "
              f"{'P20' if row['mink']['pass20'] else '   '}")

    env.close()
    payload = {"tool": "tools/phase2_corner_check.py", "park_mm": args.park,
               "seed_tip_mm": [round(float(v), 1) for v in seed_tip],
               "rows": rows}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=1) + "\n")
    regress = [r["target_mm"] for r in rows if r["regression_at_20mm"]]
    print(f"\nREGRESSIONS at 20 mm: {regress or 'none'}")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
