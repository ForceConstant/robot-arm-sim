#!/usr/bin/env python3
"""Third pass: the good scan poses re-run at a second wrist-roll branch.

Keeps every joint except the wrist roll, which is offset by --roll-offset so the
same board targets are seen from a different roll.  Captures go to their own
directory for the hand-eye solve.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.marker_scan import load_plan  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.bridge_client import HttpBridge, HttpFrames  # noqa: E402
from live.scan_controller import ScanController, plan_in_degrees  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
CONSOLE = "http://127.0.0.1:8080"
OUT = ROOT / "live" / "captures_handeye_20261121c"


def api(path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(CONSOLE + path, data=data,
                                 headers={"Content-Type": "application/json"} if data else {})
    return json.loads(urllib.request.urlopen(req, timeout=20).read())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--roll-offset", type=float, default=46.3)
    ap.add_argument("--only", default="")
    ap.add_argument("--exposure", type=int, default=705)
    ap.add_argument("--gain", type=int, default=64)
    ap.add_argument("--brightness", type=int, default=160)
    ap.add_argument("--contrast", type=int, default=32)
    args = ap.parse_args()
    only = {v.strip() for v in args.only.split(",") if v.strip()}

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    token = (ROOT / cfg["token_file"]).read_text().strip()
    bridge = HttpBridge(cfg["bridge_url"], token=token, timeout=6.0)
    frames = HttpFrames(cfg["bridge_url"], token=token, timeout=12.0)
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    plan = plan_in_degrees(load_plan(str(ROOT / cfg["plan"])))
    ctrl = ScanController(
        bridge, frames,
        lambda j: cal.camera(tuple(float(np.radians(v)) for v in j)),
        plan, layout=None, square_m=cfg["square_m"],
        board_z_m=cfg["board_z_m"], marker_z_m=cfg["marker_z_m"],
        base_xy_m=(0.0, 0.0), board_half_mm=cfg["board_half_mm"],
        settle_s=cfg["settle_s"], pose_timeout_s=75.0,
        speed_scale=cfg["speed_scale"])
    state = bridge.state()
    if state.get("mode") != "REMOTE" or not state.get("armed"):
        print(f"bridge {state.get('mode')} armed {state.get('armed')} -> abort")
        return 3
    ctrl._seq = max(ctrl._seq, int(state.get("last_seq") or 0))
    api("/camera/controls", {"controls": {
        "exposure_time_absolute": args.exposure, "gain": args.gain,
        "brightness": args.brightness, "contrast": args.contrast}, "save": False})
    time.sleep(0.8)
    OUT.mkdir(parents=True, exist_ok=True)
    receipt = {"schema": "handeye-capture/1",
               "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "roll_offset": args.roll_offset, "poses": []}

    def capture(label):
        for _ in range(3):
            frame = frames.capture()
            st = bridge.state()
            cur = tuple(float(st["joints"][n]) for n in NAMES)
            drift = max(abs(a - b) for a, b in zip(cur, frame.joints_deg))
            if drift <= 2.0:
                break
            time.sleep(0.3)
        dets = fiducials.detect_markers(frame.image)
        gray = cv2.cvtColor(frame.image, cv2.COLOR_RGB2GRAY)
        markers = []
        for d in dets:
            edges = [float(np.linalg.norm(d.corners[i] - d.corners[(i + 1) % 4]))
                     for i in range(4)]
            markers.append({"id": int(d.marker_id), "edge_px": round(float(np.mean(edges)), 1),
                            "corners": [[round(float(x), 2), round(float(y), 2)]
                                        for x, y in d.corners]})
        ctrl._seq += 1
        name = f"{time.strftime('%Y%m%dT%H%M%S')}{ctrl._seq % 1000:03d}_{label}"
        ok, enc = cv2.imencode(".jpg", cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 92])
        (OUT / f"{name}.jpg").write_bytes(enc.tobytes())
        record = {"name": name, "label": label, "timestamp": frame.timestamp,
                  "joints_deg": {n: float(v) for n, v in zip(NAMES, frame.joints_deg)},
                  "state_joints_deg": {n: float(st["joints"][n]) for n in NAMES},
                  "joint_drift_deg": round(float(drift), 3), "mode": st.get("mode"),
                  "markers": markers,
                  "tone": {"exposure": args.exposure, "gain": args.gain,
                           "brightness": args.brightness, "contrast": args.contrast},
                  "frame": {"mean": round(float(gray.mean()), 1),
                            "clip": round(float(100 * (gray >= 250).mean()), 2)},
                  "wrist_roll_deg": float(frame.joints_deg[4]),
                  "duplicate_of": None}
        (OUT / f"{name}.json").write_text(json.dumps(record, indent=2) + "\n")
        print(f"  {label}: markers {[m['id'] for m in markers]} "
              f"roll {frame.joints_deg[4]:.2f} clip {record['frame']['clip']}%", flush=True)
        return record

    for pose in plan.poses:
        if only and pose.name not in only:
            continue
        target = list(pose.joints)
        target[4] = target[4] + args.roll_offset
        cur = tuple(float(bridge.state()["joints"][n]) for n in NAMES)
        print(f"LEG {pose.name}: {[round(v, 1) for v in cur]} -> "
              f"{[round(v, 1) for v in target]} (roll {cur[4]:.1f} -> {target[4]:.1f})",
              flush=True)
        reached, error, settled = ctrl._goto(pose.name, target)
        st = bridge.state()
        roll = float(st["joints"]["wrist_roll"])
        print(f"  reached {reached} err {error:.2f} roll {roll:.2f}", flush=True)
        if not reached or abs(roll - target[4]) > 2.5:
            receipt["poses"].append({"pose": pose.name, "reached": reached,
                                     "error_deg": round(error, 2), "roll": roll,
                                     "note": "not reached / branch change - stop"})
            break
        rec = capture(pose.name)
        receipt["poses"].append({"pose": pose.name, "reached": True,
                                 "error_deg": round(error, 2), "roll": round(roll, 2),
                                 "markers": [m["id"] for m in rec["markers"]],
                                 "capture": rec["name"]})

    receipt["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (ROOT / "reports" / "handeye_capture_20261121c.json").write_text(
        json.dumps(receipt, indent=2) + "\n")
    print("\nsummary:")
    for row in receipt["poses"]:
        print(f"  {row['pose']}: reached {row.get('reached')} roll {row.get('roll')} "
              f"markers {row.get('markers')} {row.get('note', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
