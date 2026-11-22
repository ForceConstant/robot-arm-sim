#!/usr/bin/env python3
"""Read-only preflight: FK each plan pose, check limits/keep-in and transit.

Runs on the BC-250.  No motion, no writes.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.kinematics import ArmKinematics, frame_at  # noqa: E402
from live.scan_controller import plan_in_degrees  # noqa: E402
from chess_sim.marker_scan import load_plan  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")


def main() -> int:
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    board = json.loads((ROOT / "live/board.json").read_text())
    plan = load_plan(str(ROOT / cfg["plan"]))
    plan = plan_in_degrees(plan)
    kin = ArmKinematics(cal, {}, frame_at(board["x_mm"], board["y_mm"],
                                          board["z_mm"], board["yaw_deg"]))
    limits = {"shoulder_pan": (-108.99, 121.38), "shoulder_lift": (-107.05, 108.64),
              "elbow_flex": (-96.68, 94.75), "wrist_flex": (-104.59, 98.79),
              "wrist_roll": (-89.98, 90.07), "gripper": (0.0, 100.0)}

    rows = []
    for pose in plan.poses:
        j = list(pose.joints)
        tip = (kin.fk(j) @ np.eye(4))[:3, 3]
        tip_mm = kin.to_board(tip) * 1000.0
        bad = [(n, v, limits[n]) for n, v in zip(NAMES, j)
               if not (limits[n][0] - 0.5 <= v <= limits[n][1] + 0.5)]
        rows.append((pose.name, j, tip_mm, bad))
        print(f"{pose.name}: joints {[round(v, 1) for v in j]} "
              f"tip_mm {np.round(tip_mm, 1).tolist()} "
              f"limits {'OK' if not bad else bad}")

    print("\ntransit legs (linear joint interpolation, 0.5 deg steps):")
    prev = rows[0]
    for cur in rows[1:]:
        a = np.array(prev[1])
        b = np.array(cur[1])
        span = float(np.max(np.abs(b - a)))
        steps = max(2, int(span / 0.5))
        zmin = 1e9
        for k in range(steps + 1):
            j = a + (b - a) * (k / steps)
            tip = kin.to_board((kin.fk(list(j)))[:3, 3]) * 1000.0
            zmin = min(zmin, float(tip[2]))
        print(f"  {prev[0]} -> {cur[0]}: max_joint_delta {span:.1f} deg, "
              f"tip_z_min {zmin:.1f} mm")
        prev = cur
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
