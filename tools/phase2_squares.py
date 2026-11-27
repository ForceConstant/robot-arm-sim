#!/usr/bin/env python3
"""Phase 2 — the 64-square sweep through the REAL `MoveSolver` wiring.

`tools/phase1_squares.py` compared `ArmKinematics.solve_tip` against a bare
`MinkArm.solve`. That answered "is the mink *solver* better"; it did not exercise
the adapter the console actually calls. This tool runs both flag states through
`live/solver.MoveSolver`, i.e. the same object `/move` uses, and reports:

* pass counts under the live 20 mm gate and under the proposed 3.0 mm gate,
* regressions (LS pass -> mink fail) under the 3.0 mm gate *and* the 20 mm gate,
* per-square solver residual and solve time,
* the approach angle, flagged when it is inside the 8° gate but close to it.

The 3.0 mm gate is a bound on the SOLVER, not on physical landing: the ~19 mm
model bias is invisible inside FK and neither column can see it.

    robot-arm-sim python tools/phase2_squares.py --out reports/phase2_squares.json
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
FILES = "abcdefgh"
GATE_DEG = 8.0
GATES_MM = (20.0, 3.0)
# How close to the 8° approach gate counts as "at the edge" and is reported.
EDGE_DEG = 7.0


def square_targets(hover_mm: float = 60.0, square_mm: float = 28.0):
    half = square_mm * 4.0
    out = []
    for rank in range(1, 9):
        for index, file in enumerate(FILES):
            out.append((f"{file}{rank}",
                        (index + 0.5) * square_mm - half,
                        (rank - 0.5) * square_mm - half,
                        hover_mm, 0.0))
    return out


def board_frame(live_dir: Path) -> np.ndarray:
    spec = json.loads((live_dir / "board.json").read_text())
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = np.eye(4)
    frame[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]]
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    return frame


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live-dir", default=str(ROOT / "live"))
    ap.add_argument("--hover-mm", type=float, default=60.0)
    ap.add_argument("--out", default="reports/phase2_squares.json")
    args = ap.parse_args()

    import mujoco
    from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
    from chess_sim.env import ChessSimEnv
    from chess_sim import fiducials
    from chess_sim.wrist_calibration import WristCalibration
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from solver import MoveSolver

    live_dir = Path(args.live_dir)
    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    limits = {n: (float(np.degrees(calibration.chain[i + 1]["limits"][0])),
                  float(np.degrees(calibration.chain[i + 1]["limits"][1])))
              for i, n in enumerate(ARM)}
    chain = ArmKinematics(calibration, limits, board_frame(live_dir))

    # The two flag states the console can be in.
    solver_ls = MoveSolver(chain, calibration=calibration,
                           tip_in_chain=TIP_IN_CHAIN, use_mink=False)
    solver_mink = MoveSolver(chain, calibration=calibration,
                             tip_in_chain=TIP_IN_CHAIN, use_mink=True)
    solver_mink.sync_limits({n: list(v) for n, v in limits.items()})

    board = BoardConfig(face="charuco", yaw=math.pi)
    env = ChessSimEnv(EnvConfig(board=board,
                                appearance=AppearanceConfig(board_markers=False)))
    env.reset("8/8/8/8/8/8/8/8")
    model, data = env.model, env.data

    rows = []
    for name, x, y, z, span in square_targets(args.hover_mm):
        point = chain.board_point(x, y, z)
        seed = [float(data.qpos[env._joint_qpos[i]]) for i in range(6)]
        row = {"square": name, "target": [x, y, z]}
        for label, solver in (("least_squares", solver_ls), ("mink", solver_mink)):
            t0 = time.perf_counter()
            solved, goal = solver.solve_tip(point, seed, TIP_IN_CHAIN, span)
            elapsed = (time.perf_counter() - t0) * 1000.0
            joints = [float(solved["joints_deg"][j]) for j in JOINTS]
            row[label] = {
                "mm": round(float(solved["position_error_mm"]), 3),
                "deg": round(float(solved["approach_error_deg"]), 3),
                "ms": round(elapsed, 1),
                "starts": solved.get("starts"),
                "joints": {j: round(joints[i], 2) for i, j in enumerate(JOINTS)},
                "pass20": bool(solved["position_error_mm"] <= 20.0
                               and solved["approach_error_deg"] <= GATE_DEG),
                "pass3": bool(solved["position_error_mm"] <= 3.0
                              and solved["approach_error_deg"] <= GATE_DEG),
            }
        row["approach_edge"] = bool(
            row["mink"]["deg"] >= EDGE_DEG and row["mink"]["deg"] <= GATE_DEG)
        row["regress20"] = bool(row["least_squares"]["pass20"]
                                and not row["mink"]["pass20"])
        row["regress3"] = bool(row["least_squares"]["pass3"]
                               and not row["mink"]["pass3"])

        # Achieved columns, so this run can be paired with the IRL sweep instead
        # of only counted.  Same construction the legacy `cartesian_squares.py
        # --backend sim` column used: drive the model to the MoveSolver's joints,
        # read the tip through the chain->tip transform the live app uses, and
        # count wrist-camera markers in the render.  The difference is that these
        # numbers now come from the production solver, not the LS one.
        achieved = None
        markers = 0
        if row["mink"]["pass3"]:
            joints_mink = [row["mink"]["joints"][j] for j in JOINTS]
            data.qpos[env._joint_qpos] = np.radians(joints_mink)
            mujoco.mj_forward(model, data)
            tip = chain.fk(joints_mink) @ TIP_IN_CHAIN
            achieved = [round(float(v), 1)
                        for v in (chain.to_board(tip[:3, 3]) * 1000.0)]
            image = env.render("wrist")
            markers = len([d for d in fiducials.detect_markers(image)
                           if 5 <= d.marker_id <= 36])
        row["achieved"] = achieved
        row["markers"] = markers
        rows.append(row)
        # Seed the next square from where the incumbent (/move today) left the
        # arm, so neither column gets a warm start from its own answer.
        data.qpos[env._joint_qpos] = np.radians(
            [row["least_squares"]["joints"][j] for j in JOINTS])
        mujoco.mj_forward(model, data)
        print(f"{name:>3}  LS {row['least_squares']['mm']:6.2f}/{row['least_squares']['deg']:5.2f}° "
              f"{row['least_squares']['ms']:7.0f}ms   mink {row['mink']['mm']:5.2f}/"
              f"{row['mink']['deg']:5.2f}° {row['mink']['ms']:6.1f}ms"
              f"  {'EDGE' if row['approach_edge'] else ''}")

    env.close()

    def stat(label, field, agg):
        values = [r[label][field] for r in rows]
        return float(agg(values))

    summary = {
        "ls_pass_20": sum(r["least_squares"]["pass20"] for r in rows),
        "ls_pass_3": sum(r["least_squares"]["pass3"] for r in rows),
        "mink_pass_20": sum(r["mink"]["pass20"] for r in rows),
        "mink_pass_3": sum(r["mink"]["pass3"] for r in rows),
        "mink_regress20": sum(r["regress20"] for r in rows),
        "mink_regress3": sum(r["regress3"] for r in rows),
        "ls_median_mm": round(stat("least_squares", "mm", np.median), 2),
        "mink_median_mm": round(stat("mink", "mm", np.median), 2),
        "ls_max_mm": round(stat("least_squares", "mm", np.max), 2),
        "mink_max_mm": round(stat("mink", "mm", np.max), 2),
        "ls_median_ms": round(stat("least_squares", "ms", np.median), 1),
        "mink_median_ms": round(stat("mink", "ms", np.median), 1),
        "ls_max_ms": round(stat("least_squares", "ms", np.max), 1),
        "mink_max_ms": round(stat("mink", "ms", np.max), 1),
        "mink_max_deg": round(stat("mink", "deg", np.max), 2),
        "edge_squares": [r["square"] for r in rows if r["approach_edge"]],
    }
    payload = {"tool": "tools/phase2_squares.py", "backend": "sim",
               "wiring": "live/solver.MoveSolver (the /move adapter)",
               "hover_mm": args.hover_mm, "gates_mm": list(GATES_MM),
               "gate_deg": GATE_DEG, "summary": summary, "squares": rows}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=1) + "\n")

    print(f"\n20mm gate: LS {summary['ls_pass_20']}/64  mink {summary['mink_pass_20']}/64"
          f"  regressions {summary['mink_regress20']}")
    print(f" 3mm gate: LS {summary['ls_pass_3']}/64  mink {summary['mink_pass_3']}/64"
          f"  regressions {summary['mink_regress3']}")
    print(f"median ms LS {summary['ls_median_ms']}  mink {summary['mink_median_ms']}   "
          f"max ms LS {summary['ls_max_ms']}  mink {summary['mink_max_ms']}")
    print(f"approach edge squares: {summary['edge_squares']}")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
