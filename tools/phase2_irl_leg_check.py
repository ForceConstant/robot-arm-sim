#!/usr/bin/env python3
"""Phase 2 — pre-move safety checks for ONE IRL leg, printed before it executes.

AGENTS.md requires, per leg: print it, run the sim clearance check, prefer
targets that keep the current wrist_roll branch, and stop on an unexpected
branch change. This does exactly that for a single Cartesian target and refuses
(does not move) if any check fails.

    robot-arm-sim python tools/phase2_irl_leg_check.py --x 98 --y -98 --z 60
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
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--z", type=float, required=True)
    ap.add_argument("--console", default="http://127.0.0.1:8081")
    ap.add_argument("--bridge", default="http://192.168.1.167:8100")
    ap.add_argument("--out", default="reports/phase2_irl_leg_check.json")
    args = ap.parse_args()

    import urllib.request

    def get(url):
        with urllib.request.urlopen(url, timeout=10) as r:
            return json.loads(r.read().decode())

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    state = get(args.bridge + "/state")
    current = [float(state["joints"][n]) for n in JOINTS]
    keep = get(args.console + "/keep-in")
    dry = None
    # The console's own solver, via its dry-run: no motion.
    body = json.dumps({"mode": "xyz", "x_mm": args.x, "y_mm": args.y,
                       "z_mm": args.z, "dry_run": True}).encode()
    req = urllib.request.Request(args.console + "/move", data=body,
                                 method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            dry = json.loads(r.read().decode())
    except Exception as exc:                                    # noqa: BLE001
        dry = {"error": f"{type(exc).__name__}: {exc}"}

    solved = (dry or {}).get("requested") or {}
    target = (dry or {}).get("target_joints") or {}
    target_list = [float(target.get(n, current[i])) for i, n in enumerate(JOINTS)]

    # Keep-in, computed the way the console computes it (jaw tip, board frame).
    from chess_sim.wrist_calibration import WristCalibration
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
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
    tip = chain.to_board((chain.fk(target_list) @ TIP_IN_CHAIN)[:3, 3]) * 1000.0
    inside = (abs(tip[0]) <= keep["x_mm"] and abs(tip[1]) <= keep["y_mm"]
              and keep["z_min_mm"] <= tip[2] <= keep["z_max_mm"])

    roll_now, roll_target = current[4], target_list[4]
    branch_delta = abs(((roll_target - roll_now) + 180.0) % 360.0 - 180.0)
    max_delta = max(abs(a - b) for a, b in zip(target_list[:5], current[:5]))

    ok = bool(not dry.get("error") and inside and branch_delta <= 5.0
              and solved.get("position_error_mm", 99) <= 3.0
              and solved.get("approach_error_deg", 99) <= 8.0)
    record = {
        "target_board_mm": [args.x, args.y, args.z],
        "current_joints": [round(v, 2) for v in current],
        "target_joints": {k: round(v, 2) for k, v in zip(JOINTS, target_list)},
        "max_joint_delta_deg": round(max_delta, 2),
        "solver": solved.get("solver"),
        "position_error_mm": solved.get("position_error_mm"),
        "approach_error_deg": solved.get("approach_error_deg"),
        "solved_tip_mm": [round(float(v), 1) for v in tip],
        "keep_in": {"x": keep["x_mm"], "y": keep["y_mm"],
                    "z": [keep["z_min_mm"], keep["z_max_mm"]], "inside": bool(inside)},
        "wrist_roll_now": round(roll_now, 2),
        "wrist_roll_target": round(roll_target, 2),
        "wrist_roll_branch_delta_deg": round(branch_delta, 2),
        "branch_preserved": bool(branch_delta <= 5.0),
        "clear_for_motion": ok,
        "checks": {
            "solved": bool(not dry.get("error")),
            "inside_keep_in": bool(inside),
            "same_roll_branch": bool(branch_delta <= 5.0),
            "perr_le_3mm": bool(solved.get("position_error_mm", 99) <= 3.0),
            "appr_le_8deg": bool(solved.get("approach_error_deg", 99) <= 8.0),
        },
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(record, indent=1) + "\n")
    print(json.dumps(record, indent=1))
    if not ok:
        print("\nREFUSED: one or more pre-move checks failed; do not move.")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
