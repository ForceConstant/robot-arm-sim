#!/usr/bin/env python3
"""Read-only: rectify the board top-down and report per-square brightness stats.

Runs on the BC-250.  No writes, no saved controls, no motion.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.board_target import TargetDetector  # noqa: E402
from p4_auto_exposure import api, grab, solve_any  # noqa: E402

PPM = 4.0          # rectified pixels per millimetre
SQUARE_MM = 28.0


def square_boxes(n: int, ppm: float):
    """Name -> (x0,y0,x1,y1) in the rectified image, origin at board centre."""
    out = {}
    side = SQUARE_MM * ppm
    centre = n / 2.0
    for rank in range(8):
        for col in range(8):
            x0 = centre + (col * SQUARE_MM - 4.0 * SQUARE_MM) * ppm
            y0 = centre + (rank * SQUARE_MM - 4.0 * SQUARE_MM) * ppm
            out[f"{chr(97 + col)}{rank + 1}"] = (x0, y0, x0 + side, y0 + side)
    return out


def inner_stats(gray, box, inset=0.20):
    x0, y0, x1, y1 = box
    dx, dy = (x1 - x0) * inset, (y1 - y0) * inset
    patch = gray[int(y0 + dy):int(y1 - dy), int(x0 + dx):int(x1 - dx)]
    if patch.size < 16:
        return {}
    return {"mean": float(patch.mean()), "median": float(np.median(patch)),
            "p10": float(np.percentile(patch, 10)),
            "std": float(patch.std())}


def main() -> int:
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    det = TargetDetector(str(ROOT / cfg["calibration"]),
                         str(ROOT / "chess_sim/assets/scene/generated/board_charuco.png"))
    solver = det.board

    api("/camera/controls", {"controls": {"exposure_time_absolute": 705,
                                          "gain": 64}, "save": False})
    time.sleep(1.0)

    frames = grab(solver, 4)
    if not frames:
        print("no frames")
        return 1
    sol = None
    for f in frames:
        s, dets = solve_any(solver, f)
        if s is not None and not s.get("single_marker"):
            sol = s
            break
    if sol is None:
        print("no multi-marker solution")
        return 2
    print(f"ids {sol.get('ids')} rms {sol['rms_px']:.2f} "
          f"cam {np.round(np.asarray(sol['camera_in_board_mm']), 1).tolist()}")

    rect, valid, n, ppm = solver.rectified(frames[0], sol, PPM)
    gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    ok = valid > 128
    boxes = square_boxes(n, ppm)
    rows = []
    for name, box in boxes.items():
        x0, y0, x1, y1 = [int(v) for v in box]
        if x0 < 0 or y0 < 0 or x1 >= n or y1 >= n:
            continue
        if not ok[y0:y1, x0:x1].all():
            continue
        st = inner_stats(gray, box)
        if st:
            rows.append((name, st))
    white = float(np.median([r[1]["median"] for r in rows])) if rows else float("nan")
    print(f"board white median {white:.1f} over {len(rows)} squares")
    print("darkest 12 by median:")
    for name, st in sorted(rows, key=lambda r: r[1]["median"])[:12]:
        print(f"  {name}: median {st['median']:.1f} mean {st['mean']:.1f} "
              f"p10 {st['p10']:.1f} std {st['std']:.1f}")
    print("e5 and neighbours:")
    for name in ["e5", "d5", "f5", "e4", "e6", "d4", "d6", "f4", "f6"]:
        hit = dict(rows).get(name)
        if hit is None:
            print(f"  {name}: outside / invalid")
        else:
            print(f"  {name}: median {hit['median']:.1f} mean {hit['mean']:.1f} "
                  f"p10 {hit['p10']:.1f} std {hit['std']:.1f} "
                  f"drop_vs_white {white - hit['median']:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
