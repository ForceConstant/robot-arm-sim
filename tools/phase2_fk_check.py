#!/usr/bin/env python3
"""Phase 2 retry — read-only geometry: where is the tip, and which way does the elbow take it?

No motion, no bridge writes. Reads the live config + calibration + board pose,
computes the jaw-tip position in the board frame for the joints given on the
command line (or for the live bridge's measured joints), the keep-in margins,
and the numerical sensitivity d(tip)/d(joint) so a small single-joint probe can
be aimed at the direction that RETRACTS the arm instead of pushing it further
outside the keep-in box.

    .venv/bin/python tools/phase2_fk_check.py --from-bridge
    .venv/bin/python tools/phase2_fk_check.py --joints -16.0,51.3,16.04,94.99,87.74,43.88
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "live"))
sys.path.insert(0, str(ROOT))

from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from kinematics import ArmKinematics, frame_at  # noqa: E402
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge", default="http://192.168.1.167:8100")
    ap.add_argument("--from-bridge", action="store_true")
    ap.add_argument("--joints", default="")
    ap.add_argument("--elbow-scan", default="-24,24,4",
                    help="start,stop,step relative to the current elbow")
    args = ap.parse_args()

    config, kinematics = load()
    if args.from_bridge:
        with urllib.request.urlopen(args.bridge + "/state", timeout=8) as reply:
            state = json.loads(reply.read().decode())
        joints = [float(state["joints"][name]) for name in JOINTS]
    elif args.joints:
        joints = [float(v) for v in args.joints.split(",")]
    else:
        ap.error("give --from-bridge or --joints")

    print("board", {k: round(float(kinematics.board[i, 3]) * 1000.0, 1)
                    for i, k in enumerate("xyz")})
    print("joints", dict(zip(JOINTS, [round(v, 3) for v in joints])))
    tip = tip_mm(kinematics, joints)
    print(f"tip board mm  x {tip[0]:8.2f}  y {tip[1]:8.2f}  z {tip[2]:8.2f}")
    kx = float(config.get("keep_in_x_mm", 118.0))
    ky = float(config.get("keep_in_y_mm", 118.0))
    print(f"keep-in margins  x {kx - abs(tip[0]):+7.2f}  y {ky - abs(tip[1]):+7.2f}"
          f"   (negative = outside)")

    print("\nd(tip)/d(joint), mm per degree (board axes):")
    rows = {}
    for index, name in enumerate(JOINTS[:5]):
        probe = list(joints)
        probe[index] += 0.5
        delta = (tip_mm(kinematics, probe) - tip) / 0.5
        rows[name] = [round(float(v), 3) for v in delta]
        print(f"  {name:<14} dx {delta[0]:+7.3f}  dy {delta[1]:+7.3f}"
              f"  dz {delta[2]:+7.3f}")

    start, stop, step = (float(v) for v in args.elbow_scan.split(","))
    print(f"\nelbow_flex sweep (relative to {joints[2]:.2f}), tip y/z:")
    value = start
    while value <= stop + 1e-9:
        probe = list(joints)
        probe[2] += value
        point = tip_mm(kinematics, probe)
        print(f"  elbow {probe[2]:8.2f} ({value:+6.1f})  "
              f"x {point[0]:8.2f}  y {point[1]:8.2f}  z {point[2]:7.2f}"
              f"  y-margin {ky - abs(point[1]):+7.2f}")
        value += step
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
