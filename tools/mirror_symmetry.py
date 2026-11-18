#!/usr/bin/env python3
"""Mirror-pair symmetry diagnostic for the 64-square Cartesian sweep.

Brian's point (2026-11-18): with a symmetric wrist, mirror squares must solve
alike -- ``a1 == h1`` and ``a8 == h8`` at the same seed.  A large split is a
model asymmetry, not reach, so this tool resolves the four corner squares from
ONE neutral seed and prints, per square:

* tip position error (mm, board frame) and approach error (deg)
* the full joint vector and each joint's margin to its calibration limit
* the achieved tip in board coordinates

It also quantifies the *expected* geometric asymmetry: the base link and the
shoulder-pan pivot are converted into board coordinates, and the straight-line
distance to each square is reported.  The board is not exactly symmetric
(x offset and yaw), so a few millimetres of split are physical; the distances
say how many.

Finally it re-runs the pairs with the ``TIP_IN_CHAIN`` rotation sign flipped in
memory (90 deg -> -90 deg).  The tip origin stays on the wrist axis, so a
correct model gives identical tip/approach errors either way; a difference
means the rotation sign is carrying orientation into the position residual.

    robot-arm-sim python tools/mirror_symmetry.py --out reports/mirror_2026-11-18.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from kinematics import JOINTS, ArmKinematics  # noqa: E402
from tip_frame import TIP_IN_CHAIN  # noqa: E402

PAIRS = (("a1", -98.0, -98.0), ("h1", 98.0, -98.0),
         ("a8", -98.0, 98.0), ("h8", 98.0, 98.0))
MIRROR = {"a1": "h1", "h1": "a1", "a8": "h8", "h8": "a8"}


def board_frame(spec: dict) -> np.ndarray:
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = np.eye(4)
    frame[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]]
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    return frame


def flipped_tip(tip: np.ndarray) -> np.ndarray:
    """Same origin on the wrist axis, rotation sign reversed about chain z."""
    out = np.array(tip, dtype=float, copy=True)
    out[:3, :3] = np.array([[0.0, -1.0, 0.0],
                            [1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0]])
    return out


def chain_limits(calibration) -> dict:
    """Joint limits as degrees, straight from the calibration chain."""
    out = {}
    for link in calibration.chain:
        if "joint_index" not in link or "limits" not in link:
            continue
        index = int(link["joint_index"])
        if index >= len(JOINTS):
            continue
        lo, hi = (float(v) for v in link["limits"])
        out[JOINTS[index]] = (math.degrees(lo), math.degrees(hi))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--hover-mm", type=float, default=60.0)
    parser.add_argument("--out", default="reports/mirror_2026-11-18.json")
    parser.add_argument("--no-flip", action="store_true",
                        help="skip the rotation-sign block (it is provably "
                             "irrelevant when the tip origin is on the axis)")
    args = parser.parse_args()

    live = Path(args.live_dir)
    config = json.loads((live / "live_config.json").read_text())
    spec = json.loads((live / "board.json").read_text())

    from chess_sim.wrist_calibration import WristCalibration
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))

    limits = chain_limits(calibration)
    kin = ArmKinematics(calibration, limits, board_frame(spec))

    seed = [float(config["home_joints"][name]) for name in JOINTS]

    # Base link origin and the shoulder-pan pivot in board coordinates.
    base = calibration.chain[0]["position"]
    base_board = kin.to_board(np.asarray(base, dtype=float)) * 1000.0
    pan_board = kin.to_board(kin.fk([0.0] * 6)[:3, 3]) * 1000.0

    def distance(point_board, x, y, z):
        target = kin.to_board(kin.board_point(x, y, z)) * 1000.0
        return float(np.linalg.norm(np.asarray(point_board) - target))

    report = {
        "hover_mm": args.hover_mm,
        "seed_deg": {name: seed[i] for i, name in enumerate(JOINTS)},
        "board_mm": spec,
        "base_board_mm": [round(float(v), 1) for v in base_board],
        "pan_pivot_board_mm": [round(float(v), 1) for v in pan_board],
        "limits_deg": {k: [round(v, 2) for v in pair] for k, pair in limits.items()},
        "tip_rotation_sign": {},
        "geometry": {},
        "pairs": {},
    }

    for name, x, y in PAIRS:
        report["geometry"][name] = {
            "xy": [x, y],
            "base_to_hover_mm": round(distance(base_board, x, y, args.hover_mm), 1),
            "base_to_board_mm": round(distance(base_board, x, y, 0.0), 1),
            "pan_to_hover_mm": round(distance(pan_board, x, y, args.hover_mm), 1),
            "pan_to_board_mm": round(distance(pan_board, x, y, 0.0), 1),
        }

    signs = ["+90"] if args.no_flip else ["+90", "-90"]
    for sign in signs:
        jaw = (np.asarray(TIP_IN_CHAIN, dtype=float) if sign == "+90"
               else flipped_tip(TIP_IN_CHAIN))
        rows = {}
        for name, x, y in PAIRS:
            point = kin.board_point(x, y, args.hover_mm)
            solved, goal = kin.solve_tip(point, seed, jaw, 0.0)
            joints = [solved["joints_deg"][j] for j in JOINTS]
            tip = (kin.fk(joints) @ jaw)[:3, 3]
            tip_board = kin.to_board(tip) * 1000.0
            margin = {}
            for index, joint in enumerate(JOINTS[:5]):
                lo, hi = limits.get(joint, (-180.0, 180.0))
                margin[joint] = round(min(joints[index] - lo, hi - joints[index]), 2)
            rows[name] = {
                "tip_error_mm": solved["position_error_mm"],
                "approach_error_deg": solved["approach_error_deg"],
                "joints_deg": [round(v, 3) for v in joints],
                "limit_margin_deg": margin,
                "achieved_board_mm": [round(float(v), 1) for v in tip_board],
                "starts": solved.get("starts"),
            }
        report["tip_rotation_sign"][sign] = rows
        for name, _, _ in PAIRS:
            other = rows[MIRROR[name]]
            rows[name]["mirror_of"] = MIRROR[name]
            rows[name]["mirror_tip_split_mm"] = round(
                abs(rows[name]["tip_error_mm"] - other["tip_error_mm"]), 2)

    # Summary table for humans.
    print(f"seed (deg): {report['seed_deg']}")
    print(f"board: {spec}")
    print(f"base in board mm: {report['base_board_mm']}  "
          f"pan pivot: {report['pan_pivot_board_mm']}")
    print()
    print(f"{'sq':>3} {'tip mm':>7} {'appr':>6} {'split':>6} "
          f"{'base->sq':>9} {'pan->sq':>8}  joints_deg")
    for sign in signs:
        print(f"--- TIP_IN_CHAIN rotation {sign} ---")
        for name, _, _ in PAIRS:
            row = report["tip_rotation_sign"][sign][name]
            geo = report["geometry"][name]
            print(f"{name:>3} {row['tip_error_mm']:>7.2f} "
                  f"{row['approach_error_deg']:>6.2f} "
                  f"{row['mirror_tip_split_mm']:>6.2f} "
                  f"{geo['base_to_hover_mm']:>9.1f} "
                  f"{geo['pan_to_hover_mm']:>8.1f}  "
                  f"{[round(v, 1) for v in row['joints_deg'][:5]]}")
            print(f"    margins: {row['limit_margin_deg']}")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
