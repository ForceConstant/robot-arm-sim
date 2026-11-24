#!/usr/bin/env python3
"""Phase 3 capture: run the roll-spread plan on the real arm.

Joint moves and camera frames only.  The gripper is held at its measured value
every leg -- it is never commanded in this script.  Each pose gets one settle
(``ScanController._goto``: command, wait for the band, hold, correct the
standing residual) and one fresh frame whose joint snapshot agrees with the
arm's current state.  Exposure and white balance are locked for the whole
session and written into every record.

    .venv/bin/python tools/phase3_capture.py --plan reports/phase3_plan_20261124.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.marker_scan import MarkerScanPlan  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.bridge_client import HttpBridge, HttpFrames  # noqa: E402
from live.kinematics import ArmKinematics, frame_at  # noqa: E402
from live.scan_controller import ScanController  # noqa: E402
from live.tip_frame import TIP_IN_CHAIN  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
TONES = {"exposure_time_absolute": 2500, "gain": 96, "brightness": 160,
         "contrast": 48, "white_balance_automatic": 0}

# One historical pose replayed unchanged.  If the board (or the mount) moved
# since 2026-11-21, the board estimate from this pose will differ from the one
# the same pose produced then.
SANITY_POSE = {"label": "sanity_marker13_r68",
               "joints": [-7.82, -59.91, 78.9, 40.84, 67.96]}


def tip_board(kin, board_frame, joints_deg):
    tip = (kin.fk(list(joints_deg)) @ TIP_IN_CHAIN)[:3, 3]
    return (np.linalg.inv(board_frame) @ np.r_[tip, 1.0])[:3] * 1000.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", default=str(ROOT / "reports" / "phase3_plan_20261124.json"))
    ap.add_argument("--out", default=str(ROOT / "live" / "captures_phase3"))
    ap.add_argument("--stamp", default=time.strftime("%Y%m%d"))
    ap.add_argument("--wb", type=int, default=4600)
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--max-poses", type=int, default=0)
    ap.add_argument("--skip-sanity", action="store_true")
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    token = (ROOT / cfg["token_file"]).read_text().strip()
    bridge = HttpBridge(cfg["bridge_url"], token=token, timeout=6.0)
    frames = HttpFrames(cfg["bridge_url"], token=token, timeout=15.0)
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    board_json = json.loads((ROOT / "live/board.json").read_text())
    board_frame = frame_at(board_json["x_mm"], board_json["y_mm"],
                           board_json["z_mm"], board_json["yaw_deg"])
    kin = ArmKinematics(cal, {}, board_frame)

    plan = json.loads(Path(args.plan).read_text())
    poses = list(plan.get("poses", []))
    if args.max_poses:
        poses = poses[:args.max_poses]
    if not poses and args.skip_sanity:
        print("plan has no poses")
        return 2

    state = bridge.state()
    if state.get("mode") != "REMOTE" or not state.get("armed"):
        print(f"bridge {state.get('mode')} armed {state.get('armed')} -> abort")
        return 3

    ctrl = ScanController(
        bridge, frames, lambda j: None,
        MarkerScanPlan(poses=(), min_views=1), layout=None,
        square_m=cfg["square_m"], board_z_m=cfg["board_z_m"],
        marker_z_m=cfg["marker_z_m"], base_xy_m=(0.0, 0.0),
        board_half_mm=cfg["board_half_mm"], settle_s=0.6,
        pose_timeout_s=args.timeout, pose_tolerance_deg=2.5,
        speed_scale=args.speed, ttl_ms=200)
    ctrl._seq = max(ctrl._seq, int(state.get("last_seq") or 0))

    tone = dict(TONES)
    tone["white_balance_temperature"] = int(args.wb)
    bridge.set_camera_controls(tone, save=False)
    time.sleep(1.0)
    applied = {c["name"]: c["value"] for c in bridge.camera_controls()["controls"]}
    print("camera controls applied:", {k: applied.get(k) for k in tone})

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    receipt = {"schema": "phase3-capture/1",
               "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "plan": args.plan, "tone": tone, "applied": applied,
               "poses": []}

    def capture(label, note=""):
        best = None
        for attempt in range(2):
            for _ in range(4):
                frame = frames.capture()
                st = bridge.state()
                cur = tuple(float(st["joints"][n]) for n in NAMES)
                drift = max(abs(a - b) for a, b in zip(cur, frame.joints_deg))
                if drift <= 1.5:
                    break
                time.sleep(0.25)
            dets = fiducials.detect_markers(frame.image)
            if best is None or len(dets) > best[0]:
                best = (len(dets), frame, st, drift, dets, attempt)
            time.sleep(0.3)
        _, frame, st, drift, dets, attempt = best
        gray = cv2.cvtColor(frame.image, cv2.COLOR_RGB2GRAY)
        markers = []
        for d in dets:
            edges = [float(np.linalg.norm(d.corners[i] - d.corners[(i + 1) % 4]))
                     for i in range(4)]
            markers.append({"id": int(d.marker_id),
                            "edge_px": round(float(np.mean(edges)), 1),
                            "corners": [[round(float(x), 2), round(float(y), 2)]
                                        for x, y in np.asarray(d.corners).reshape(-1, 2)]})
        ctrl._seq += 1
        name = f"{time.strftime('%Y%m%dT%H%M%S')}{ctrl._seq % 1000:03d}_{label}"
        ok, enc = cv2.imencode(".jpg", cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 92])
        (out / f"{name}.jpg").write_bytes(enc.tobytes())
        rec = {"name": name, "label": label, "note": note,
               "timestamp": frame.timestamp,
               "joints_deg": {n: float(v) for n, v in zip(NAMES, frame.joints_deg)},
               "state_joints_deg": {n: float(st["joints"][n]) for n in NAMES},
               "joint_drift_deg": round(float(drift), 3), "mode": st.get("mode"),
               "tool": st.get("tool_tip"),
               "markers": markers,
               "tone": tone,
               "frame": {"mean": round(float(gray.mean()), 1),
                         "clip": round(float(100 * (gray >= 250).mean()), 2)},
               "wrist_roll_deg": float(frame.joints_deg[4])}
        (out / f"{name}.json").write_text(json.dumps(rec, indent=2) + "\n")
        print(f"  {label}: markers {[m['id'] for m in markers]} "
              f"roll {frame.joints_deg[4]:.2f} drift {drift:.2f} "
              f"clip {rec['frame']['clip']}%", flush=True)
        return rec

    def leg(label, target5, note=""):
        target = list(target5) + [float(bridge.state()["joints"]["gripper"])]
        tip = tip_board(kin, board_frame, target)
        print(f"LEG {label}: {[round(float(v), 1) for v in target]}  "
              f"tip(board) {np.round(tip, 1).tolist()} {note}", flush=True)
        if abs(tip[0]) > 112 or abs(tip[1]) > 112 or not 15 <= tip[2] <= 320:
            print(f"  SKIP: tip outside keep-in/clearance (z {tip[2]:.1f})")
            receipt["poses"].append({"pose": label, "skipped": "keep-in",
                                     "tip": tip.tolist()})
            return False
        reached, error, _ = ctrl._goto(label, target)
        st = bridge.state()
        roll = float(st["joints"]["wrist_roll"])
        print(f"  reached {reached} err {error:.2f} roll {roll:.2f}", flush=True)
        if abs(roll - target[4]) > 2.5:
            receipt["poses"].append({"pose": label, "reached": False,
                                     "error_deg": round(error, 2), "roll": roll,
                                     "note": "roll branch changed - stop"})
            print(f"  STOP: roll {roll:.2f} left the commanded branch {target[4]:.2f}")
            return False
        note = note if reached else (note + " (short of target, captured as measured)")
        rec = capture(label, note)
        receipt["poses"].append({"pose": label, "reached": bool(reached),
                                 "error_deg": round(error, 2), "roll": round(roll, 2),
                                 "markers": [m["id"] for m in rec["markers"]],
                                 "capture": rec["name"],
                                 "tip_board_mm": [round(float(v), 1) for v in tip]})
        return True

    if not args.skip_sanity:
        print("\n--- sanity: replay one historical roll-68 pose ---")
        if not leg(SANITY_POSE["label"], SANITY_POSE["joints"],
                   "(board-moved check)"):
            print("sanity pose failed to execute; continuing if plan poses exist")

    last_roll = None
    for pose in poses:
        roll = float(pose["roll"])
        if last_roll is not None and roll != last_roll:
            print(f"\n--- roll group {roll:+.0f} ---", flush=True)
        last_roll = roll
        ok = leg(f"r{int(round(roll)):+03d}_{pose['target']}",
                 [float(v) for v in pose["joints_deg"][:5]],
                 f"(target {pose['target']}, pixel "
                 f"{[round(v) for v in (pose.get('pixel') or [])]})")
        if not ok:
            print("stopping: a leg did not execute; receipt kept")
            break

    receipt["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    receipt_path = ROOT / "reports" / f"phase3_capture_{args.stamp}.json"
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    done = [p for p in receipt["poses"] if p.get("reached")]
    print(f"\n{len(done)}/{len(poses) + (0 if args.skip_sanity else 1)} legs captured "
          f"-> {out}")
    for p in receipt["poses"]:
        print(f"  {p['pose']}: reached {p.get('reached')} roll {p.get('roll')} "
              f"markers {p.get('markers')} {p.get('note', '')}")
    print(f"receipt: {receipt_path}")
    return 0 if done else 4


if __name__ == "__main__":
    raise SystemExit(main())
