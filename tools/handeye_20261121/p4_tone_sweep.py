#!/usr/bin/env python3
"""Read-only camera-tone sweep: brightness/contrast vs decoded markers."""
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "live"))
from chess_sim import fiducials
from live.bridge_client import HttpFrames

CONSOLE = "http://127.0.0.1:8080"


def post(path, body):
    req = urllib.request.Request(CONSOLE + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=30).read())


def main():
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    token = (ROOT / cfg["token_file"]).read_text().strip()
    frames = HttpFrames(cfg["bridge_url"], token=token, timeout=12.0)
    grid = [(222, 32), (192, 32), (160, 32), (128, 32), (96, 32),
            (128, 64), (160, 64), (192, 64), (222, 64)]
    for brightness, contrast in grid:
        post("/camera/controls", {"controls": {
            "exposure_time_absolute": 705, "gain": 64,
            "brightness": brightness, "contrast": contrast}, "save": False})
        time.sleep(0.8)
        frame = frames.capture()
        gray = cv2.cvtColor(frame.image, cv2.COLOR_RGB2GRAY)
        dets = fiducials.detect_markers(frame.image)
        print(f"b{brightness}/c{contrast}: mean {gray.mean():5.1f} "
              f"median {np.median(gray):5.1f} clip {100*(gray>=250).mean():5.2f}% "
              f"dark {100*(gray<=5).mean():5.2f}% std {gray.std():5.1f} "
              f"markers {sorted(int(d.marker_id) for d in dets)}", flush=True)
    return 0


raise SystemExit(main())
