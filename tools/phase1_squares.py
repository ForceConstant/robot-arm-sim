#!/usr/bin/env python3
"""Phase 1: the identical 64-square sweep, solved by both solvers.

``tools/cartesian_squares.py`` measures one backend with ``live/kinematics``.
This runs the *same* 64 board-frame targets through

* ``least_squares`` — ``ArmKinematics.solve_tip``, exactly what ``/move`` runs
  today, with the same fast-path-then-escalate logic;
* ``mink``           — ``live/mink_arm.MinkArm`` over the chain-derived model,
  through ``chess_sim/ik.So101Ik``'s QP stack.

and records, per square: the gate verdict (20 mm / 8 deg, the live gate), the
solver's own reported residual, the *achieved* tip error measured by
``ArmKinematics.fk`` after snapping the joints to the solution (so both columns
mean the same thing), and the solve time in milliseconds.

Two residuals are recorded per solver, because they answer different questions:

* ``tip_vs_goal_mm`` — the solver's own tip error against the pose it built.
  Solver quality.
* ``tip_vs_target_mm`` — the achieved tip against the *commanded* board point,
  through the chain + ``TIP_IN_CHAIN``.  This is the number the gate should be
  judged on, and it is where the documented ~19 mm model bias lives (see
  ``reports/T2_frame_refit_20261123.md``).  Both solvers inherit it; neither can
  remove it.

    robot-arm-sim python tools/phase1_squares.py --out reports/phase1_squares.json
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
ARM_JOINTS = JOINTS[:5]
FILES = "abcdefgh"
GATE_MM = 20.0
GATE_DEG = 8.0


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


def _board_frame(live_dir: Path) -> np.ndarray:
    spec = json.loads((live_dir / "board.json").read_text())
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = np.eye(4)
    frame[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]]
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    return frame


def _setup(live_dir: Path):
    from kinematics import ArmKinematics, _orthonormal, transform
    from tip_frame import TIP_IN_CHAIN
    from chess_sim.wrist_calibration import WristCalibration
    from mink_arm import MinkArm

    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    limits = {n: (float(np.degrees(calibration.chain[i + 1]["limits"][0])),
                  float(np.degrees(calibration.chain[i + 1]["limits"][1])))
              for i, n in enumerate(ARM_JOINTS)}
    chain = ArmKinematics(calibration, limits, _board_frame(live_dir))
    arm = MinkArm(calibration, TIP_IN_CHAIN, limits=limits)
    return chain, arm, TIP_IN_CHAIN, transform, _orthonormal


def _achieved(chain, joints, jaw) -> np.ndarray:
    return chain.to_board((chain.fk(joints) @ jaw)[:3, 3]) * 1000.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--hover-mm", type=float, default=60.0)
    parser.add_argument("--limit", type=int, default=0,
                        help="only the first N squares (0 = all 64)")
    parser.add_argument("--out", default="reports/phase1_squares.json")
    args = parser.parse_args()

    import mujoco
    from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
    from chess_sim.env import ChessSimEnv

    live_dir = Path(args.live_dir)
    chain, arm, jaw, transform, orthonormal = _setup(live_dir)

    board = BoardConfig(face="charuco", yaw=math.pi)
    env = ChessSimEnv(EnvConfig(board=board,
                                appearance=AppearanceConfig(board_markers=False)))
    env.reset("8/8/8/8/8/8/8/8")
    model, data = env.model, env.data

    targets = square_targets(args.hover_mm)
    if args.limit:
        targets = targets[:args.limit]

    rows = []
    for name, x, y, z, span in targets:
        point = chain.board_point(x, y, z)
        seed = [float(data.qpos[env._joint_qpos[i]]) for i in range(6)]

        # -- least_squares: exactly what /move runs
        t0 = time.perf_counter()
        solved, goal = chain.solve_tip(point, seed, jaw, span)
        ls_ms = (time.perf_counter() - t0) * 1000.0
        ls_q = [solved["joints_deg"][j] for j in JOINTS]
        ls_tip = _achieved(chain, ls_q, jaw)
        ls_target = chain.to_board(point) * 1000.0

        # -- mink: the chain-derived model.
        # ``solve_tip`` returns a CHAIN-frame goal (it pre-multiplies by
        # ``inv(TIP_IN_CHAIN)``); mink solves for the *tip*, so the goal handed
        # over is chain @ TIP_IN_CHAIN.  Passing the chain goal straight through
        # would ask the tip to sit where the wrist is.
        tip_goal = goal @ jaw
        t0 = time.perf_counter()
        mink_result = arm.solve(tip_goal, seed, span_deg=span)
        mink_ms = (time.perf_counter() - t0) * 1000.0
        mink_q = [mink_result["joints_deg"][j] for j in JOINTS]
        mink_tip = _achieved(chain, mink_q, jaw)

        def verdict(res, mm, deg):
            return "PASS" if mm <= GATE_MM and deg <= GATE_DEG else "FAIL"

        ls_mm = float(np.linalg.norm(ls_tip - ls_target))
        mink_mm = float(np.linalg.norm(mink_tip - ls_target))
        ls_ok = verdict(solved, solved["position_error_mm"],
                        solved["approach_error_deg"])
        mink_ok = verdict(mink_result, mink_result["position_error_mm"],
                          mink_result["approach_error_deg"])
        delta = "same"
        if ls_ok == "PASS" and mink_ok == "FAIL":
            delta = "REGRESS"
        elif ls_ok == "FAIL" and mink_ok == "PASS":
            delta = "gain"

        rows.append({
            "square": name, "target": [x, y, z],
            "least_squares": {"verdict": ls_ok, "mm": solved["position_error_mm"],
                              "deg": solved["approach_error_deg"], "ms": round(ls_ms, 1),
                              "starts": solved.get("starts"),
                              "achieved_mm": round(ls_mm, 2),
                              "joints": {j: round(ls_q[i], 2) for i, j in enumerate(JOINTS)}},
            "mink": {"verdict": mink_ok, "mm": mink_result["position_error_mm"],
                     "deg": mink_result["approach_error_deg"], "ms": round(mink_ms, 1),
                     "starts": mink_result.get("starts"),
                     "achieved_mm": round(mink_mm, 2),
                     "joints": {j: round(mink_q[i], 2) for i, j in enumerate(JOINTS)}},
            "delta": delta,
        })
        # Snap the arm to the *incumbent's* solution so the next square seeds
        # from where ``/move`` would actually have left the arm.  Both solvers
        # then start every square from the same seed, and mink gets no
        # warm-start advantage from its own previous answer.
        data.qpos[env._joint_qpos] = np.radians(ls_q)
        mujoco.mj_forward(model, data)
        print(f"{name:>3}  LS {solved['position_error_mm']:5.2f}mm/{solved['approach_error_deg']:5.2f}° "
              f"{ls_ms:6.0f}ms {ls_ok:4}   mink {mink_result['position_error_mm']:5.2f}mm/"
              f"{mink_result['approach_error_deg']:5.2f}° {mink_ms:6.0f}ms {mink_ok:4}  {delta}")

    env.close()

    def stats(key, field):
        values = [r[key][field] for r in rows]
        return {"median": round(float(np.median(values)), 2),
                "p95": round(float(np.percentile(values, 95)), 2),
                "max": round(float(np.max(values)), 2)}

    summary = {
        "ls_median_ms": stats("least_squares", "ms")["median"],
        "mink_median_ms": stats("mink", "ms")["median"],
        "ls_median_mm": stats("least_squares", "mm")["median"],
        "mink_median_mm": stats("mink", "mm")["median"],
        "ls_max_mm": stats("least_squares", "mm")["max"],
        "mink_max_mm": stats("mink", "mm")["max"],
    }
    report = {"tool": "tools/phase1_squares.py", "backend": "sim",
              "hover_mm": args.hover_mm, "gate": [GATE_MM, GATE_DEG],
              "summary": summary, "squares": rows}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2) + "\n")

    ls_ok = sum(1 for r in rows if r["least_squares"]["verdict"] == "PASS")
    mink_ok = sum(1 for r in rows if r["mink"]["verdict"] == "PASS")
    regress = [r["square"] for r in rows if r["delta"] == "REGRESS"]
    print(f"\nleast_squares {ls_ok}/{len(rows)}   mink {mink_ok}/{len(rows)}   "
          f"regressions {len(regress)} {regress}")
    print(f"median ms  LS {summary['ls_median_ms']:.0f}  mink {summary['mink_median_ms']:.0f}")
    print(f"median mm  LS {summary['ls_median_mm']:.2f}  mink {summary['mink_median_mm']:.2f}")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
