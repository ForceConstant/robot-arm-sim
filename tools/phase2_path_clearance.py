#!/usr/bin/env python3
"""Phase 2 — clearance along the interpolated path of ONE joint-space leg.

A target can be inside keep-in while the arm sweeps out of it on the way. This
walks the joint-space straight line from the measured pose to the solved target
in 2 deg steps and reports, for each sample, the jaw-tip position in board mm
and whether it stays inside keep-in and above the table (z >= 0).

Refuses (exit 2) if any sample leaves the envelope.

    robot-arm-sim python tools/phase2_path_clearance.py --targets-json /tmp/h1.json
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

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
ARM = JOINTS[:5]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", required=True, help="6 joint deg, comma separated")
    ap.add_argument("--target", required=True, help="6 joint deg, comma separated")
    ap.add_argument("--step-deg", type=float, default=2.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    from chess_sim.wrist_calibration import WristCalibration
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / cfg["calibration"]))
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

    start = [float(v) for v in args.start.split(",")]
    target = [float(v) for v in args.target.split(",")]
    span = max(abs(a - b) for a, b in zip(target[:5], start[:5]))
    steps = max(2, int(math.ceil(span / args.step_deg)))
    keep = {"x": 118.0, "y": 118.0, "z_min": -5.0, "z_max": 320.0}

    worst = {"x": 0.0, "y": 0.0, "zmin": 1e9}
    violations = []
    samples = []
    for k in range(steps + 1):
        t = k / steps
        q = [a + (b - a) * t for a, b in zip(start, target)]
        tip = chain.to_board((chain.fk(q) @ TIP_IN_CHAIN)[:3, 3]) * 1000.0
        tip = [float(v) for v in tip]
        worst["x"] = max(worst["x"], abs(tip[0]))
        worst["y"] = max(worst["y"], abs(tip[1]))
        worst["zmin"] = min(worst["zmin"], tip[2])
        inside = (abs(tip[0]) <= keep["x"] and abs(tip[1]) <= keep["y"]
                  and keep["z_min"] <= tip[2] <= keep["z_max"])
        if not inside:
            violations.append({"t": round(t, 3), "tip_mm": [round(v, 1) for v in tip]})
        samples.append({"t": round(t, 3), "tip_mm": [round(v, 1) for v in tip],
                        "inside": bool(inside)})

    record = {"start_joints": start, "target_joints": target,
              "samples": steps + 1, "step_deg": args.step_deg,
              "worst": {k: round(v, 1) for k, v in worst.items()},
              "keep_in": keep, "violations": violations[:20],
              "violation_count": len(violations),
              "clear": not violations,
              "path": samples}
    print(f"steps {steps + 1}  worst |x| {worst['x']:.1f}  |y| {worst['y']:.1f}  "
          f"min z {worst['zmin']:.1f}   violations {len(violations)}")
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(record, indent=1) + "\n")
        print(f"-> {args.out}")
    if violations:
        print("PATH LEAVES KEEP-IN — do not execute")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
