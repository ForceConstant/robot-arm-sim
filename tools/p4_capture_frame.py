#!/usr/bin/env python3
"""Read-only: grab one fresh board frame, report exposure stats, save it.

    PYTHONPATH=. .venv/bin/python tools/p4_capture_frame.py --frames 1
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from p4_board_frame_check import stream_frames  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--frames", type=int, default=1)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    frames = stream_frames(args.url, args.frames, 8.0)
    if not frames:
        print("no frames received")
        return 1
    for i, portrait in enumerate(frames):
        landscape = cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)
        gray = cv2.cvtColor(landscape, cv2.COLOR_BGR2GRAY)
        dark = float((gray < 8).mean() * 100.0)
        blown = float((gray > 247).mean() * 100.0)
        print(f"frame {i}: shape {landscape.shape} mean {gray.mean():.1f} "
              f"median {np.median(gray):.1f} min {gray.min()} max {gray.max()} "
              f"p1 {np.percentile(gray,1):.0f} p99 {np.percentile(gray,99):.0f} "
              f"<8 {dark:.2f}% >247 {blown:.2f}%")
        if args.out:
            path = ROOT / args.out.format(i=i)
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), landscape)
            print(f"saved {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
