#!/usr/bin/env python3
"""Block 5q prep — wide-volume capture poses for a hand-eye re-solve.

The camera model is ~50 mm out at the scan postures (block 5p), so this file
does NOT trust its tag predictions.  It picks joint-space poses that span the
working volume, keeps only those whose FK camera is near the board and roughly
downward (a coarse sanity filter, not a coverage claim), and lets the real
detector decide what is visible.

    # dry run (default): prints the pose list, margins and duration
    robot-arm-sim python tools/block5q_wide_cal.py

    # execute (needs Brian's go-ahead for the wide-volume re-solve)
    robot-arm-sim python tools/block5q_wide_cal.py --execute --out live/captures/wide_5q

Nothing here modifies the plan, the detector or any gate.
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

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
ARM = NAMES[:5]

PAN = (-40, -20, 0, 20, 40)
LIFT = (-80, -55, -30, 0, 40)
WRIST_FLEX = (20, 45, 70, 95)
GRIPPER_OPEN = 80.0     # keep the jaws out of the way for every capture


def bridge_limits(url: str) -> dict:
    import urllib.request
    with urllib.request.urlopen(url.rstrip("/") + "/state", timeout=10) as r:
        state = json.loads(r.read())
    raw = state.get("limits") or {}
    if not raw:
        raise SystemExit(f"bridge at {url} returned no limits")
    return {k: (float(v[0]), float(v[1])) for k, v in raw.items()}


def margins(joints, limits):
    out = {}
    for i, name in enumerate(ARM):
        lo, hi = limits.get(name, (-180.0, 180.0))
        v = float(joints[i])
        out[name] = round(min(v - lo, hi - v), 2)
    return out


def camera_of(joints_deg, calibration):
    return calibration.camera(tuple(math.radians(v) for v in joints_deg))


def candidates(limits, calibration, board_z, keep_in):
    """Joint-space grid, then a coarse FK sanity filter (no tag prediction)."""
    elbow_grid = (45.0, 70.0, 92.0)
    out = []
    for pan in PAN:
        for lift in LIFT:
            for elbow in elbow_grid:
                for wflex in WRIST_FLEX:
                    for roll in (0.0, -45.0, 45.0):
                        joints = [pan, lift, elbow, wflex, roll, GRIPPER_OPEN]
                        bad = False
                        for i, name in enumerate(ARM):
                            lo, hi = limits.get(name, (-180.0, 180.0))
                            if not lo + 1.0 <= joints[i] <= hi - 1.0:
                                bad = True
                                break
                        if bad:
                            continue
                        cam = camera_of(joints, calibration)
                        pos = np.asarray(cam.camera_position) * 1000.0
                        axis = -np.asarray(cam.camera_rotation).reshape(3, 3)[:, 2]
                        height = pos[2] - board_z * 1000.0   # pos is mm
                        if not 90.0 <= height <= 300.0:
                            continue
                        if abs(pos[0]) > 260.0 or abs(pos[1]) > 260.0:
                            continue
                        if abs(float(axis[2])) < 0.30:      # not looking down enough
                            continue
                        tip_ok = (abs(pos[0]) <= keep_in["x_mm"] + 20
                                  and abs(pos[1]) <= keep_in["y_mm"] + 20)
                        if not tip_ok:
                            continue
                        out.append({"joints": [round(v, 1) for v in joints],
                                    "height_mm": round(float(height), 1),
                                    "camera_mm": [round(float(v), 1) for v in pos]})
    return out


def spread(items, count):
    """Greedy farthest-point on normalised joint vectors: broad volume coverage."""
    if len(items) <= count:
        return items
    scale = np.array([40.0, 60.0, 25.0, 40.0, 45.0])
    pts = [(np.asarray(it["joints"][:5]) / scale) for it in items]
    chosen = [0]
    while len(chosen) < count:
        best, best_d = None, -1.0
        for i, p in enumerate(pts):
            if i in chosen:
                continue
            d = min(float(np.linalg.norm(p - pts[j])) for j in chosen)
            if d > best_d:
                best, best_d = i, d
        if best is None:
            break
        chosen.append(best)
    return [items[i] for i in chosen]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bridge", default="http://192.168.1.167:8100")
    ap.add_argument("--calibration", default=".private/phase11d/calibration.json")
    ap.add_argument("--poses", type=int, default=36)
    ap.add_argument("--out", default="live/captures/wide_5q")
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--settle-s", type=float, default=2.0)
    ap.add_argument("--execute", action="store_true",
                    help="actually move and capture (default: dry run, no motion)")
    args = ap.parse_args()

    from chess_sim.wrist_calibration import WristCalibration
    calibration = WristCalibration.load(str(ROOT / args.calibration))
    limits = bridge_limits(args.bridge)
    board = json.loads((ROOT / "live/board.json").read_text())
    board_z = board["z_mm"] / 1000.0
    keep_in = {"x_mm": 118.0, "y_mm": 118.0, "z_min_mm": -5.0, "z_max_mm": 320.0}

    pool = candidates(limits, calibration, board_z, keep_in)
    poses = spread(pool, args.poses)
    if not poses:
        print("no candidate pose passed the limits/FK filter — widen the grid "
              "or the filter; nothing to list, nothing to run")
        return 1
    print(f"candidates after limits/FK filter: {len(pool)}; selected: {len(poses)}")

    rows = []
    worst = 999.0
    for i, pose in enumerate(poses):
        m = margins(pose["joints"], limits)
        tight = min(m.items(), key=lambda kv: kv[1])
        worst = min(worst, tight[1])
        rows.append({**pose, "margins_deg": m, "tightest": tight[0],
                     "tightest_margin_deg": tight[1]})
        print(f'{i:2d} pan {pose["joints"][0]:+6.1f} lift {pose["joints"][1]:+6.1f} '
              f'elbow {pose["joints"][2]:6.1f} wflex {pose["joints"][3]:5.1f} '
              f'roll {pose["joints"][4]:+6.1f}  height {pose["height_mm"]:5.1f} mm  '
              f'tightest {tight[0]} {tight[1]:+.1f}')

    # Conservative duration: 3.15 deg/s at speed_scale 0.3, plus settle and capture.
    span = 0.0
    cur = np.asarray(poses[0]["joints"][:5])
    for pose in poses[1:]:
        nxt = np.asarray(pose["joints"][:5])
        span += float(np.max(np.abs(nxt - cur))) / (10.5 * args.speed)
        cur = nxt
    total_s = span + len(poses) * (args.settle_s + 1.5)
    print(f"\nworst joint margin: {worst:+.1f} deg")
    print(f"estimated duration: {total_s/60:.1f} min "
          f"({span:.0f} s of travel + {len(poses)} x {args.settle_s + 1.5:.1f} s)")
    print("mode:", "EXECUTE" if args.execute else "DRY RUN (no motion)")

    report = {"schema": "block5q/wide-cal-poses/1", "poses": rows,
              "worst_joint_margin_deg": round(worst, 2),
              "estimated_duration_s": round(total_s, 1),
              "speed_scale": args.speed, "bridge_limits": limits}
    out_report = ROOT / "reports/block5q_pose_list.json"
    out_report.write_text(json.dumps(report, indent=2) + "\n")
    print("wrote", out_report)

    if not args.execute:
        return 0

    # ---- execute (only with Brian's go-ahead) --------------------------------
    from bridge_client import HttpBridge
    bridge = HttpBridge(args.bridge, token_file=str(ROOT / "live/bridge_token.txt"))
    out_dir = ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%S")
    for i, pose in enumerate(poses):
        target = pose["joints"]
        body = json.dumps({"mode": "joint",
                           "joints": {n: v for n, v in zip(NAMES, target)}}).encode()
        import urllib.request
        req = urllib.request.Request("http://127.0.0.1:8080/move", data=body,
                                     headers={"Content-Type": "application/json"},
                                     method="POST")
        try:
            with urllib.request.urlopen(req, timeout=90) as r:
                move = json.loads(r.read())
        except Exception as exc:  # noqa: BLE001
            print(f"{i:2d} move failed: {exc}")
            continue
        time.sleep(args.settle_s)
        frame = bridge._request("/capture")
        import base64
        raw = base64.b64decode(frame["jpeg_b64"])
        (out_dir / f"wide_{stamp}_{i:02d}.jpg").write_bytes(raw)
        (out_dir / f"wide_{stamp}_{i:02d}.json").write_text(json.dumps({
            "index": i, "target_deg": dict(zip(NAMES, target)),
            "measured_deg": {n: float(frame["joints"][n]) for n in NAMES},
            "move_error_deg": move.get("error_deg")}, indent=2) + "\n")
        print(f"{i:2d} captured {out_dir}/wide_{stamp}_{i:02d}.jpg "
              f"move_err {move.get('error_deg')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
