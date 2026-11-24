#!/usr/bin/env python3
"""Phase 2 — is the Phase 0 sim endpoint actually reachable, and by whom?

`tools/phase2_sim_flip.py` showed the real console refusing `(-85,85,120)` under
`mink`+3.0 mm while `least_squares`+20 mm accepted it at 11.5 mm of error. Two
readings are possible and they demand different actions:

  (a) mink narrowed the arm's reach -> do not flip.
  (b) the old endpoint was never reachable; the 20 mm gate passed an 11.5 mm
      miss -> the flip is right and the FIXTURE is wrong.

This scans a set of candidate relocations from the park posture and reports,
per target, each solver's best residual and whether a >=100 mm relocation is
achievable inside 3.0 mm. Deciding on evidence, not on a single number.

    robot-arm-sim python tools/phase2_reach_scan.py
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

CANDIDATES = [
    (-85.0, 85.0, 120.0),    # Phase 0 sim fixture (old)
    (65.0, 70.0, 110.0),     # Phase 0 IRL park
    (-90.0, 20.0, 110.0),    # Phase 0 IRL relocation
    (-95.0, 60.0, 110.0),
    (-80.0, 70.0, 110.0),
    (-70.0, 60.0, 110.0),
    (95.0, 70.0, 110.0),
    (-95.0, -20.0, 110.0),
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--park", default="0,0,70")
    ap.add_argument("--out", default="reports/phase2_reach_scan.json")
    args = ap.parse_args()

    import mujoco
    from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
    from chess_sim.env import ChessSimEnv
    from chess_sim.wrist_calibration import WristCalibration
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from solver import MoveSolver

    calibration = WristCalibration.load(
        str(ROOT / ".private/phase11d/calibration.json"))
    limits = {n: (float(np.degrees(calibration.chain[i + 1]["limits"][0])),
                  float(np.degrees(calibration.chain[i + 1]["limits"][1])))
              for i, n in enumerate(ARM)}
    spec = json.loads((ROOT / "live/board.json").read_text())
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

    board = BoardConfig(face="charuco", yaw=math.pi)
    env = ChessSimEnv(EnvConfig(board=board,
                               appearance=AppearanceConfig(board_markers=False)))
    env.reset("8/8/8/8/8/8/8/8")
    model, data = env.model, env.data
    px, py, pz = (float(v) for v in args.park.split(","))
    park_solved, _ = chain.solve_tip(chain.board_point(px, py, pz),
                                     [0.0, -55.0, 72.0, 39.0, 0.0, 0.0],
                                     TIP_IN_CHAIN)
    park_q = [float(park_solved["joints_deg"][j]) for j in JOINTS]
    data.qpos[env._joint_qpos] = np.radians(park_q)
    mujoco.mj_forward(model, data)
    seed = [float(data.qpos[env._joint_qpos[i]]) for i in range(6)]
    park_tip = chain.to_board((chain.fk(seed) @ TIP_IN_CHAIN)[:3, 3]) * 1000.0

    rows = []
    for tx, ty, tz in CANDIDATES:
        point = chain.board_point(tx, ty, tz)
        row = {"target_mm": [tx, ty, tz],
               "nominal_travel_mm": round(float(np.linalg.norm(
                   np.array([tx, ty, tz]) - park_tip)), 1)}
        for label, solver in (("least_squares", ls), ("mink", mink)):
            t0 = time.perf_counter()
            solved, _ = solver.solve_tip(point, seed, TIP_IN_CHAIN, 0.0)
            row[label] = {"mm": round(float(solved["position_error_mm"]), 3),
                          "deg": round(float(solved["approach_error_deg"]), 3),
                          "ms": round((time.perf_counter() - t0) * 1000.0, 1),
                          "pass3": bool(solved["position_error_mm"] <= 3.0
                                        and solved["approach_error_deg"] <= 8.0),
                          "pass20": bool(solved["position_error_mm"] <= 20.0
                                         and solved["approach_error_deg"] <= 8.0)}
        rows.append(row)
        print(f"{str(row['target_mm']):22} travel {row['nominal_travel_mm']:6.1f}  "
              f"LS {row['least_squares']['mm']:6.2f}/{row['least_squares']['deg']:5.2f}"
              f"{' P3' if row['least_squares']['pass3'] else '   '}   "
              f"mink {row['mink']['mm']:5.2f}/{row['mink']['deg']:5.2f}"
              f"{' P3' if row['mink']['pass3'] else '   '}")

    env.close()
    payload = {"tool": "tools/phase2_reach_scan.py", "park_mm": args.park,
               "park_tip_mm": [round(float(v), 1) for v in park_tip],
               "rows": rows}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=1) + "\n")
    good = [r["target_mm"] for r in rows if r["mink"]["pass3"]]
    print(f"\nmink pass@3mm: {good}")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
