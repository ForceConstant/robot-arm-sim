#!/usr/bin/env python3
"""Read-only: independently decode the detected marker's 4x4 bits.

Warp the detected quad to a canonical 6x6 module grid, threshold, and compare
the 16 payload bits against every DICT_4X4_50 marker.  Confirms the id without
trusting cv2.aruco's own decode.

    PYTHONPATH=. .venv/bin/python tools/p4_verify_marker_id.py --image <path>
"""
from __future__ import annotations

import argparse
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
from chess_sim.board_target import TargetDetector  # noqa: E402

DICT = cv2.aruco.DICT_4X4_50
MODULES = 6


def pattern(marker_id: int) -> np.ndarray:
    img = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(DICT),
                                        marker_id, MODULES * 20)
    small = cv2.resize(img, (MODULES, MODULES), interpolation=cv2.INTER_AREA)
    return small < 128


def sample(quad: np.ndarray, frame: np.ndarray, roll: int = 0) -> np.ndarray:
    q = np.roll(quad, roll, axis=0)
    size = MODULES * 20
    inset = size * 0.02
    dst = np.array([[inset, inset], [size - 1 - inset, inset],
                    [size - 1 - inset, size - 1 - inset], [inset, size - 1 - inset]],
                   np.float32)
    warped = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(
        q.astype(np.float32), dst), (size, size), flags=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    cell = size / MODULES
    med = np.zeros((MODULES, MODULES))
    for r in range(MODULES):
        for c in range(MODULES):
            y = int((r + 0.5) * cell)
            x = int((c + 0.5) * cell)
            patch = gray[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
            med[r, c] = float(np.median(patch))
    # uneven lighting: threshold the 36 module medians with Otsu, not a fixed 128
    flat = np.clip(med, 0, 255).astype(np.uint8).reshape(-1, 1)
    cut, _ = cv2.threshold(flat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return med < cut


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--config", default="live/live_config.json")
    args = ap.parse_args()
    config = json.loads((ROOT / args.config).read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board
    raw = cv2.imread(str(ROOT / args.image), cv2.IMREAD_COLOR)
    rect = solver.rectify(raw)
    dets = fiducials.detect_markers(cv2.cvtColor(rect, cv2.COLOR_BGR2RGB))
    print(f"detections {[d.marker_id for d in dets]}")
    for d in dets:
        best = None
        for roll in range(4):
            bits = sample(d.corners, rect, roll)
            scored = sorted((int((pattern(mid) != bits).sum()), mid) for mid in range(50))
            if best is None or scored[0][0] < best[0]:
                best = (scored[0][0], roll, bits, scored)
        hamming, roll, bits, scored = best
        print(f"\ndetected id {d.marker_id}: best roll {roll}, hamming {hamming}")
        for row in bits.astype(int):
            print("   " + "".join("#" if v else "." for v in row))
        print(f"  best matches (hamming, id): {scored[:5]}")
        print(f"  declared pattern for {d.marker_id}:")
        for row in pattern(d.marker_id).astype(int):
            print("   " + "".join("#" if v else "." for v in row))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
