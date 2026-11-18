#!/usr/bin/env python3
"""March a ray outward and report where the tip can no longer reach.

The sweep leaves one or two squares just outside the 20 mm gate.  The question
that decides "reach" versus "model" is whether the target sits beyond the
arm's envelope: if a square just inboard solves to ~1 mm and the square itself
cannot, the residual is geometry, not a solver or frame defect.

This marches along +y at a fixed x and hover, re-solving from the previous
posture (a single local solve, no multi-start), and prints the tip error at
each step.  Near the boundary the error rises steeply; the crossing point is
the reachable extent for that posture family.

    robot-arm-sim python tools/reach_envelope.py --x 98 --start 80 --stop 108 --step 2
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


def board_frame(spec: dict) -> np.ndarray:
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = np.eye(4)
    frame[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]]
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    return frame


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--x", type=float, default=98.0)
    parser.add_argument("--z", type=float, default=60.0)
    parser.add_argument("--start", type=float, default=80.0)
    parser.add_argument("--stop", type=float, default=108.0)
    parser.add_argument("--step", type=float, default=2.0)
    parser.add_argument("--limits", choices=("none", "calibration"), default="none",
                        help="enforce the calibration's joint limits (the arm's "
                             "real envelope) or leave the joints unbounded")
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    live = Path(args.live_dir)
    config = json.loads((live / "live_config.json").read_text())
    spec = json.loads((live / "board.json").read_text())

    from chess_sim.wrist_calibration import WristCalibration
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    limits = {}
    if args.limits == "calibration":
        for link in calibration.chain:
            if "joint_index" in link and "limits" in link:
                index = int(link["joint_index"])
                if index < len(JOINTS):
                    limits[JOINTS[index]] = tuple(
                        math.degrees(v) for v in link["limits"])
    kin = ArmKinematics(calibration, limits, board_frame(spec))
    jaw = np.asarray(TIP_IN_CHAIN, dtype=float)
    seed = [float(config["home_joints"][name]) for name in JOINTS]

    rows = []
    y = args.start
    while y <= args.stop + 1e-9:
        target = kin.xyz_target(args.x, y, args.z)
        # ``solve`` expects the *chain-frame* goal; it multiplies by the
        # chain->tip transform itself.
        solved = kin.solve(target @ np.linalg.inv(jaw), seed, tool_frame=jaw)
        joints = [solved["joints_deg"][j] for j in JOINTS]
        seed = joints  # walk outward from the last posture
        tip = (kin.fk(joints) @ jaw)[:3, 3]
        achieved = kin.to_board(tip) * 1000.0
        rows.append({"x": args.x, "y": y, "z": args.z,
                     "tip_error_mm": solved["position_error_mm"],
                     "approach_error_deg": solved["approach_error_deg"],
                     "achieved": [round(float(v), 1) for v in achieved],
                     "shortfall_y_mm": round(y - float(achieved[1]), 1)})
        print(f"y={y:>6.1f}  tip={solved['position_error_mm']:>6.2f} mm  "
              f"approach={solved['approach_error_deg']:>5.2f} deg  "
              f"achieved={[round(float(v), 1) for v in achieved]}")
        y += args.step

    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(
            {"x": args.x, "z": args.z, "rows": rows}, indent=2) + "\n")
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
