#!/usr/bin/env python3
"""Update the scan plan from the two measured capture passes.

Keeps only poses that actually decoded markers on the rig, and adds the same
targets at a second wrist-roll branch (roll + 46.3 deg).  Writes the plan file
plus a receipt.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

ROOT = Path("/home/brian/robot-arm-sim")
PLAN = ROOT / "chess_sim/assets/marker_scan_plan_charuco.json"
B = ROOT / "live/captures_handeye_20261121b"
C = ROOT / "live/captures_handeye_20261121c"
ROLL_OFFSET = 46.3


def records(directory):
    out = {}
    for path in sorted(directory.glob("*.json")):
        rec = json.loads(path.read_text())
        out.setdefault(rec["label"], []).append(rec)
    return out


def main() -> int:
    plan = json.loads(PLAN.read_text())
    base = records(B)
    roll68 = records(C)

    def best(store, label):
        rows = store.get(label) or []
        return max(rows, key=lambda r: len(r.get("markers", [])), default=None)

    poses = []
    dropped = []
    for pose in plan["poses"]:
        rec = best(base, pose["name"])
        if rec is None or not rec.get("markers"):
            dropped.append(pose["name"])
            continue
        item = dict(pose)
        item["visible"] = [m["id"] for m in rec["markers"]]
        item["edge_px"] = [m["edge_px"] for m in rec["markers"]]
        item["measured_roll_deg"] = round(float(rec["joints_deg"]["wrist_roll"]), 2)
        poses.append(item)

    added = []
    for pose in plan["poses"]:
        rec = best(roll68, pose["name"])
        if rec is None or not rec.get("markers"):
            continue
        joints = list(pose["joints"])
        joints[4] = joints[4] + ROLL_OFFSET
        item = {"name": f"{pose['name']}_r68", "joints": joints,
                "visible": [m["id"] for m in rec["markers"]],
                "edge_px": [m["edge_px"] for m in rec["markers"]],
                "measured_roll_deg": round(float(rec["joints_deg"]["wrist_roll"]), 2)}
        if "camera_position" in pose:
            item["camera_position"] = pose["camera_position"]
        added.append(item)
        poses.append(item)

    counts = {str(m): 0 for m in plan.get("marker_ids", [])}
    for pose in poses:
        for marker in pose["visible"]:
            counts[str(marker)] = counts.get(str(marker), 0) + 1

    out = dict(plan)
    out["poses"] = poses
    out["views_per_marker"] = counts
    out["generated_at"] = time.strftime("%Y-%m-%d")
    out["note"] = (f"updated 2026-11-21 from two measured capture passes: "
                   f"{len(poses) - len(added)} roll-21.7 poses + {len(added)} roll-68 "
                   f"poses; dropped {dropped} (0 markers decoded on the rig); "
                   f"visible/edge_px come from the real captures, not the simulator")
    out["measured_from"] = {
        "roll_21_7": str(B.relative_to(ROOT)),
        "roll_68": str(C.relative_to(ROOT)),
        "roll_offset_deg": ROLL_OFFSET,
    }
    PLAN.write_text(json.dumps(out, indent=2) + "\n")
    print(f"wrote {PLAN}: {len(poses)} poses "
          f"({len(poses) - len(added)} at roll 21.7, {len(added)} at roll 68)")
    print(f"dropped: {dropped}")
    print("views per marker: " + json.dumps(counts))
    return 0


raise SystemExit(main())
