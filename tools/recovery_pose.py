#!/usr/bin/env python3
"""Derive the one stored recovery pose: jaw tip high above the board centre.

The owner's rule is "if the robot gets stuck, always reset to home high above the
middle square and start again".  That pose must be a STORED joint pose, because
the thing that gets stuck is often the solver — recovery cannot route through
IK.  This tool derives it once, offline, and checks it with FK:

  * solver: whichever ``--solver`` says (default least_squares, the incumbent),
  * seed: the current/reference posture, so the roll stays on the play branch,
  * verification: FK(tip) must land within ``--tolerance-mm`` of (0, 0, z).

    .venv/bin/python tools/recovery_pose.py --solver least_squares
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "live"))
sys.path.insert(0, str(ROOT))

from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from kinematics import ArmKinematics, _orthonormal, frame_at, transform  # noqa: E402
from solver import MoveSolver  # noqa: E402
from tip_frame import TIP_IN_CHAIN  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


def load() -> tuple[dict, ArmKinematics]:
    config = json.loads((ROOT / "live/live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config["calibration"]))
    spec = {"x_mm": 0.0, "y_mm": 0.0, "z_mm": 0.0, "yaw_deg": 0.0}
    board_path = ROOT / "live/board.json"
    if board_path.exists():
        spec.update({k: float(v) for k, v in
                     json.loads(board_path.read_text()).items() if k in spec})
    kinematics = ArmKinematics(calibration, {}, frame_at(
        spec["x_mm"], spec["y_mm"], spec["z_mm"], spec["yaw_deg"]))
    return config, kinematics


def tip_mm(kinematics: ArmKinematics, joints) -> np.ndarray:
    tip = (kinematics.fk(list(joints)) @ TIP_IN_CHAIN)[:3, 3]
    return kinematics.to_board(tip) * 1000.0


def _approach_deg(kinematics: ArmKinematics, joints) -> float:
    """Angle between the jaw approach axis and the board normal, in degrees."""
    tip = kinematics.fk(list(joints)) @ TIP_IN_CHAIN
    axis = tip[:3, 2] / max(1e-9, float(np.linalg.norm(tip[:3, 2])))
    up = kinematics.board_axis("z")
    return float(np.degrees(np.arccos(np.clip(axis @ up, -1.0, 1.0))))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--z", type=float, default=130.0)
    ap.add_argument("--solver", default="least_squares",
                    choices=("least_squares", "mink"))
    ap.add_argument("--seed", default="-16.0,51.3,16.04,94.99,87.74,43.88",
                    help="seed posture; its wrist_roll fixes the branch")
    ap.add_argument("--tolerance-mm", type=float, default=3.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--freeze-roll", action="store_true",
                    help="keep the seed's wrist_roll (the play branch) instead "
                         "of letting the multi-start search pick another one")
    ap.add_argument("--from-leader", default="",
                    help="JSON from tools/leader_pose.py: verify the leader's "
                         "own posture (wrist_flex forced to 0) as the recovery "
                         "pose, and fall back to the solved pose if it fails")
    ap.add_argument("--limits-json", default="",
                    help="JSON file with {joint: [lo, hi]} to check against "
                         "(e.g. the bridge's /state limits)")
    args = ap.parse_args()

    config, kinematics = load()
    seed = [float(v) for v in args.seed.split(",")]
    solver = MoveSolver(kinematics, use_mink=args.solver == "mink",
                        tip_in_chain=TIP_IN_CHAIN,
                        calibration=kinematics.calibration)

    if args.from_leader:
        raw = json.loads(Path(args.from_leader).read_text())
        leader = {name: float(value) for name, value in raw["joints"].items()}
        candidate = [leader[name] for name in JOINTS[:5]] + [leader.get("gripper", 0.0)]
        candidate[3] = 0.0                     # wrist_flex at its calibrated zero
        landed = tip_mm(kinematics, candidate)
        tip_angle = _approach_deg(kinematics, candidate)
        limits = (json.loads(Path(args.limits_json).read_text())
                  if args.limits_json else {})
        outside = [f"{name} {value:.2f} not in {limits[name][0]:.2f}.."
                   f"{limits[name][1]:.2f}"
                   for name, value in zip(JOINTS[:5], candidate)
                   if name in limits
                   and not limits[name][0] <= value <= limits[name][1]]
        reasons = []
        if abs(landed[0]) > 15.0 or abs(landed[1]) > 15.0:
            reasons.append(f"tip is not above the board centre "
                           f"({landed[0]:.0f}, {landed[1]:.0f} mm)")
        if landed[2] < 110.0:
            reasons.append(f"tip too low ({landed[2]:.0f} mm < 110)")
        if tip_angle > 5.0:
            reasons.append(f"jaws {tip_angle:.1f} deg off vertical")
        if outside:
            reasons.append("outside joint limits: " + ", ".join(outside))
        record = {"source": "leader", "leader_raw": leader,
                  "candidate_joints": {name: round(value, 2)
                                       for name, value in zip(JOINTS, candidate)},
                  "fk_tip_mm": [round(float(v), 2) for v in landed],
                  "approach_deg": round(tip_angle, 2),
                  "accepted": not reasons, "rejected_because": reasons}
        if reasons:
            # Fall back to the solved pose; the caller keeps what config already
            # has. Say plainly that the human's leader posture is not over the
            # middle square — that is information, not an error.
            fallback, _goal = solver.solve_tip(kinematics.board_point(0.0, 0.0, args.z),
                                              seed, TIP_IN_CHAIN, 0.0)
            record["fallback_solver"] = args.solver
            record["fallback_joints"] = {
                name: round(float(fallback["joints_deg"][name]), 2)
                for name in JOINTS}
        print(json.dumps(record, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(record, indent=1) + "\n")
        return 0

    point = kinematics.board_point(0.0, 0.0, args.z)
    if args.freeze_roll:
        # Same construction as ``ArmKinematics.solve_tip``: jaws-down tip pose on
        # the board, expressed as the chain goal the solver actually takes.
        tip_goal = transform(point, _orthonormal(kinematics.board_axis("z"),
                                                 kinematics.board_axis("x")))
        chain_goal = tip_goal @ np.linalg.inv(TIP_IN_CHAIN)
        solved = solver.solve_multi(chain_goal, seed, freeze_roll=True)
    else:
        solved, _goal = solver.solve_tip(point, seed, TIP_IN_CHAIN, 0.0)
    target = [float(solved["joints_deg"][name]) for name in JOINTS]
    target[5] = seed[5]                      # gripper unchanged, by design
    landed = tip_mm(kinematics, target)
    error_mm = float(np.linalg.norm(landed - np.array([0.0, 0.0, args.z])))
    inside = abs(landed[0]) <= float(config.get("keep_in_x_mm", 118.0)) \
        and abs(landed[1]) <= float(config.get("keep_in_y_mm", 118.0)) \
        and float(config.get("keep_in_z_min_mm", -5.0)) <= landed[2] \
        <= float(config.get("keep_in_z_max_mm", 320.0))
    record = {"solver": args.solver, "target_board_mm": [0.0, 0.0, args.z],
              "seed": seed,
              "recovery_joints": {name: round(value, 2)
                                  for name, value in zip(JOINTS, target)},
              "fk_tip_mm": [round(float(v), 2) for v in landed],
              "fk_error_mm": round(error_mm, 3),
              "solver_position_error_mm": solved["position_error_mm"],
              "solver_approach_error_deg": solved["approach_error_deg"],
              "wrist_roll_seed": seed[4], "wrist_roll_pose": round(target[4], 2),
              "inside_keep_in": bool(inside), "ok": bool(error_mm <= args.tolerance_mm
                                                         and inside)}
    print(json.dumps(record, indent=1))
    if args.out:
        Path(args.out).write_text(json.dumps(record, indent=1) + "\n")
    return 0 if record["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
