#!/usr/bin/env python3
"""Read-only sweep: apply exposure/gain/contrast combos, measure each.

    PYTHONPATH=. .venv/bin/python tools/p4_exposure_sweep.py
"""
from __future__ import annotations

import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.board_target import TargetDetector  # noqa: E402
from p4_board_frame_check import stream_frames  # noqa: E402

CONFIGS = [
    {"exposure_time_absolute": 260, "gain": 80, "contrast": 100},
    {"exposure_time_absolute": 260, "gain": 80, "contrast": 64},
    {"exposure_time_absolute": 320, "gain": 80, "contrast": 64},
    {"exposure_time_absolute": 260, "gain": 80, "contrast": 32},
]
URL = "http://127.0.0.1:8080"


def post(controls: dict) -> dict:
    body = json.dumps({"controls": controls, "save": False}).encode()
    req = urllib.request.Request(URL + "/camera/controls", data=body,
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=15).read())


def main() -> int:
    config = json.loads((ROOT / "live/live_config.json").read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board
    for cfg in CONFIGS:
        applied = post(cfg)["applied"]
        time.sleep(1.0)
        rows = []
        for portrait in stream_frames(URL, 2, 8.0):
            rect = solver.rectify(cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE))
            gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
            b, g, r = (rect[:, :, k].astype(np.float32) for k in range(3))
            dets = fiducials.detect_markers(cv2.cvtColor(rect, cv2.COLOR_BGR2RGB))
            sol, _ = solver.solve(rect)
            rows.append((gray.mean(), np.median(gray), gray.std(),
                         100*(gray >= 250).mean(), 100*(gray <= 5).mean(),
                         100*(g >= 250).mean(), len(dets),
                         sol["rms_px"] if sol else float("nan")))
        m = np.mean(np.array(rows), axis=0)
        print(f"{applied} | mean {m[0]:6.1f} median {m[1]:6.1f} std {m[2]:5.1f} "
              f"| >=250 {m[3]:5.1f}% <=5 {m[4]:5.1f}% G>=250 {m[5]:5.1f}% "
              f"| markers {m[6]:.1f} | PnP rms {m[7]:.2f} px")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
