#!/usr/bin/env python3
"""Drive the qualified /scan pose list and persist every view for hand-eye.

Uses the qualified ScanController motion path (``_goto``) unchanged; this only
adds per-pose persistence, a roll-branch guard and an exposure settle.

Writes only live/captures_handeye_20261121/ and one reports/ receipt.
No project file, plan, board or calibration is edited.
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
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.marker_scan import load_plan  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.bridge_client import HttpBridge, HttpFrames  # noqa: E402
from live.scan_controller import ScanController, plan_in_degrees  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
CONSOLE = "http://127.0.0.1:8080"
OUT = ROOT / "live" / "captures_handeye_20261121"
GRID = [(1000, 64), (1500, 64), (2000, 64), (2600, 64), (3400, 64), (2600, 96)]


def api(path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(CONSOLE + path, data=data,
                                 headers={"Content-Type": "application/json"} if data else {})
    return json.loads(urllib.request.urlopen(req, timeout=20).read())


def frame_metrics(image_rgb) -> dict:
    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    return {"mean": round(float(gray.mean()), 1),
            "median": round(float(np.median(gray)), 1),
            "clip": round(float(100 * (gray >= 250).mean()), 2),
            "dark": round(float(100 * (gray <= 5).mean()), 2),
            "std": round(float(gray.std()), 1)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--stop-after", type=int, default=0)
    ap.add_argument("--settle-only", action="store_true")
    args = ap.parse_args()

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
    if state.get("mode") != "REMOTE":
        print(f"bridge mode {state.get('mode')}, not REMOTE -> abort")
        return 3
    if not state.get("armed"):
        print("bridge not armed -> abort")
        return 3
    ctrl._seq = max(ctrl._seq, int(state.get("last_seq") or 0))
    start_joints = tuple(float(state["joints"][n]) for n in NAMES)
    grip_hold = start_joints[5]
    print(f"start joints {[round(v, 2) for v in start_joints]} "
          f"roll {start_joints[4]:.2f} gripper {grip_hold:.2f} (held, not actuated)")

    OUT.mkdir(parents=True, exist_ok=True)
    receipt = {"schema": "handeye-capture/1", "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "start_joints": dict(zip(NAMES, start_joints)),
               "gripper_held": grip_hold, "poses": [], "settle": []}

    def capture(pose_name: str, exposure, gain):
        for attempt in range(3):
            frame = frames.capture()
            st = bridge.state()
            cur = tuple(float(st["joints"][n]) for n in NAMES)
            drift = max(abs(a - b) for a, b in zip(cur, frame.joints_deg))
            if drift <= 2.0:
                break
            print(f"  stale frame drift {drift:.2f} deg, retry")
            time.sleep(0.3)
        dets = fiducials.detect_markers(frame.image)
        markers = []
        for d in dets:
            edges = [float(np.linalg.norm(d.corners[i] - d.corners[(i + 1) % 4]))
                     for i in range(4)]
            markers.append({"id": int(d.marker_id), "edge_px": round(float(np.mean(edges)), 1),
                            "corners": [[round(float(x), 2), round(float(y), 2)]
                                        for x, y in d.corners]})
        ctrl._seq += 1
        stamp = time.strftime("%Y%m%dT%H%M%S")
        name = f"{stamp}{ctrl._seq % 1000:03d}_{pose_name}"
        ok, enc = cv2.imencode(".jpg", cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR),
                               [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise RuntimeError("encode failed")
        (OUT / f"{name}.jpg").write_bytes(enc.tobytes())
        record = {"name": name, "label": pose_name, "timestamp": frame.timestamp,
                  "joints_deg": {n: float(v) for n, v in zip(NAMES, frame.joints_deg)},
                  "state_joints_deg": {n: float(st["joints"][n]) for n in NAMES},
                  "joint_drift_deg": round(float(drift), 3),
                  "mode": st.get("mode"),
                  "markers": markers,
                  "settled_exposure": exposure, "settled_gain": gain,
                  "wrist_roll_deg": float(frame.joints_deg[4]),
                  "duplicate_of": None}
        (OUT / f"{name}.json").write_text(json.dumps(record, indent=2) + "\n")
        print(f"  captured {name}: markers {[m['id'] for m in markers]} "
              f"roll {frame.joints_deg[4]:.2f} drift {drift:.2f}")
        return record

    def move_and_capture(pose, exposure, gain):
        target = list(pose.joints)
        target[5] = grip_hold
        cur = tuple(float(bridge.state()["joints"][n]) for n in NAMES)
        delta = [round(t - c, 1) for t, c in zip(target, cur)]
        print(f"LEG {pose.name}: from {[round(v, 1) for v in cur]} to "
              f"{[round(v, 1) for v in target]} delta {delta} "
              f"(roll {cur[4]:.1f} -> {target[4]:.1f})")
        reached, error, settled = ctrl._goto(pose.name, target)
        print(f"  reached {reached} error {error:.2f} deg settle {settled:.1f}s")
        st = bridge.state()
        actual_roll = float(st["joints"]["wrist_roll"])
        if not reached:
            return {"pose": pose.name, "reached": False, "error_deg": round(error, 2),
                    "actual_roll": actual_roll, "note": "pose not reached - stop"}
        if abs(actual_roll - target[4]) > 2.5:
            return {"pose": pose.name, "reached": True, "error_deg": round(error, 2),
                    "actual_roll": actual_roll,
                    "note": f"unexpected roll branch {actual_roll:.2f} vs {target[4]:.2f} - stop"}
        rec = capture(pose.name, exposure, gain)
        return {"pose": pose.name, "reached": True, "error_deg": round(error, 2),
                "actual_roll": round(actual_roll, 2), "markers": [m["id"] for m in rec["markers"]],
                "capture": rec["name"]}

    first = plan.poses[0]
    target = list(first.joints)
    target[5] = grip_hold
    cur = start_joints
    print(f"LEG {first.name}: from {[round(v, 1) for v in cur]} to "
          f"{[round(v, 1) for v in target]} "
          f"(roll {cur[4]:.1f} -> {target[4]:.1f})")
    reached, error, settled = ctrl._goto(first.name, target)
    print(f"  reached {reached} error {error:.2f} deg settle {settled:.1f}s")
    if not reached:
        print("first pose not reached -> abort before any capture")
        receipt["poses"].append({"pose": first.name, "reached": False,
                                 "error_deg": round(error, 2)})
        (ROOT / "reports" / "handeye_capture_20261121.json").write_text(
            json.dumps(receipt, indent=2) + "\n")
        return 4
    roll1 = float(bridge.state()["joints"]["wrist_roll"])
    print(f"arrived roll {roll1:.2f} (commanded {target[4]:.2f})")
    if abs(roll1 - target[4]) > 2.5:
        print(f"unexpected roll branch {roll1:.2f} vs {target[4]:.2f} -> abort")
        return 5

    # ---- settle exposure at the first scan pose, judged by numbers only
    print("\nexposure settle at pose 1:")
    for exposure, gain in GRID:
        api("/camera/controls", {"controls": {"exposure_time_absolute": exposure,
                                              "gain": gain}, "save": False})
        time.sleep(0.9)
        f = frames.capture()
        m = frame_metrics(f.image)
        dets = fiducials.detect_markers(f.image)
        row = {"exposure": exposure, "gain": gain, **m, "markers": len(dets),
               "ids": sorted(int(d.marker_id) for d in dets)}
        receipt["settle"].append(row)
        print(f"  {exposure}/{gain}: mean {m['mean']} median {m['median']} "
              f"clip {m['clip']}% dark {m['dark']}% markers {len(dets)} {row['ids']}")
    # The frame at this standoff includes table outside the board, so the
    # whole-frame dark budget is not meaningful here; the binding rules are no
    # blown highlights and the most decodable markers.
    pool = [r for r in receipt["settle"] if r["clip"] <= 1.5 and r["mean"] >= 40]
    if not pool:
        print("no exposure met the clipping budget -> abort")
        return 6
    pick = max(pool, key=lambda r: (r["markers"], r["median"], -r["clip"]))
    exposure, gain = pick["exposure"], pick["gain"]
    api("/camera/controls", {"controls": {"exposure_time_absolute": exposure,
                                          "gain": gain}, "save": False})
    time.sleep(0.9)
    receipt["settled"] = {"exposure": exposure, "gain": gain,
                          "metrics": {k: pick[k] for k in ("mean", "clip", "dark", "markers")}}
    print(f"settled {exposure}/{gain} (markers {pick['markers']}, clip {pick['clip']}%)")
    if args.settle_only:
        (ROOT / "reports" / "handeye_capture_20261121.json").write_text(
            json.dumps(receipt, indent=2) + "\n")
        print("settle-only: stopping before capture")
        return 0

    print("\ncapture pose 1:")
    rec = capture(first.name, exposure, gain)
    receipt["poses"].append({"pose": first.name, "reached": True,
                             "error_deg": round(error, 2), "actual_roll": round(roll1, 2),
                             "markers": [m["id"] for m in rec["markers"]],
                             "capture": rec["name"]})

    for index, pose in enumerate(plan.poses[1:], start=1):
        if index < args.start:
            continue
        if args.stop_after and index >= args.stop_after:
            break
        row = move_and_capture(pose, exposure, gain)
        receipt["poses"].append(row)
        if not row["reached"] or "stop" in row.get("note", ""):
            print(f"STOP: {row.get('note')}")
            break

    receipt["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (ROOT / "reports" / "handeye_capture_20261121.json").write_text(
        json.dumps(receipt, indent=2) + "\n")
    print("\nsummary:")
    for row in receipt["poses"]:
        print(f"  {row['pose']}: reached {row.get('reached')} "
              f"roll {row.get('actual_roll')} markers {row.get('markers')} "
              f"{row.get('note', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
