#!/usr/bin/env python3
"""Read-only marker detection diagnostic: every id, size, on raw vs rectified."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.board_target import TargetDetector, JOINTS  # noqa: E402
from p4_board_frame_check import stream_frames  # noqa: E402

config = json.loads((ROOT / "live/live_config.json").read_text())
detector = TargetDetector(str(ROOT / config["calibration"]),
                          str(ROOT / "chess_sim/assets/scene/generated/board_charuco.png"))
solver = detector.board
print("MARKER_IDS", sorted(fiducials.MARKER_IDS))
print("dict", fiducials._DETECTOR.getPredefinedDictionary if hasattr(fiducials._DETECTOR, "getPredefinedDictionary") else "n/a")

frames = stream_frames("http://127.0.0.1:8080", 2, 8.0)
print("frames", len(frames))
for i, portrait in enumerate(frames):
    land = cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)
    rect = solver.rectify(land)
    for tag, img in (("raw", land), ("rect", rect)):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        d = fiducials._detector()
        corners, ids, _ = d.detectMarkers(gray)
        n = 0 if ids is None else len(ids)
        info = []
        if ids is not None:
            for mid, quad in zip(ids.ravel(), corners):
                q = np.asarray(quad).reshape(4, 2)
                side = float(np.mean([np.linalg.norm(q[k] - q[(k + 1) % 4]) for k in range(4)]))
                info.append((int(mid), round(side, 1), [round(float(v), 1) for v in q.mean(0)]))
        print(f"frame {i} {tag}: strict {n} -> {info}")
        kept = [d for d in fiducials.detect_markers(rgb)]
        print(f"frame {i} {tag}: kept {[(k.marker_id, [round(float(v),1) for v in k.corners.mean(0)]) for k in kept]}")
