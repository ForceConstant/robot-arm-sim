#!/usr/bin/env python3
"""Read-only jaw test: hold the pose, move only the gripper, count decoded markers.

Drives the bridge directly (the console's own sequence counter is behind after a
scan, so /move 409s).
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "live"))
from chess_sim import fiducials
from live.bridge_client import HttpBridge, HttpFrames

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")


def main():
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    token = (ROOT / cfg["token_file"]).read_text().strip()
    bridge = HttpBridge(cfg["bridge_url"], token=token, timeout=8.0)
    frames = HttpFrames(cfg["bridge_url"], token=token, timeout=12.0)
    state = bridge.state()
    seq = int(state.get("last_seq") or 0)
    base = [float(state["joints"][n]) for n in NAMES]
    print(f"base joints {[round(v, 1) for v in base]} seq {seq}", flush=True)
    for jaw in (0, 25, 50, 75, 100):
        target = list(base)
        target[5] = jaw
        print(f"jaw -> {jaw}", flush=True)
        deadline = time.time() + 20
        while time.time() < deadline:
            seq += 1
            bridge.command(target, seq=seq, ttl_ms=200, speed_scale=0.3)
            time.sleep(0.1)
            now = bridge.state()
            cur = [float(now["joints"][n]) for n in NAMES]
            if max(abs(a - b) for a, b in zip(cur, target)) <= 1.5:
                break
        print(f"  now {[round(v, 1) for v in cur]}", flush=True)
        time.sleep(1.0)
        frame = frames.capture()
        dets = fiducials.detect_markers(frame.image)
        gray = cv2.cvtColor(frame.image, cv2.COLOR_RGB2GRAY)
        print(f"  markers {sorted(int(d.marker_id) for d in dets)} "
              f"mean {gray.mean():.1f} clip {100*(gray>=250).mean():.2f}%", flush=True)
    return 0


raise SystemExit(main())
