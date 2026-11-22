#!/usr/bin/env python3
"""Read-only: save a rectified top-down board view and project the tool tip.

Runs on the BC-250.  Writes only /tmp images.
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

TIP_MM = (13.9, 18.2, 58.3)


def main() -> int:
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    det = TargetDetector(str(ROOT / cfg["calibration"]),
                         str(ROOT / "chess_sim/assets/scene/generated/board_charuco.png"))
    solver = det.board
    api("/camera/controls", {"controls": {"exposure_time_absolute": 705,
                                          "gain": 64}, "save": False})
    time.sleep(1.0)
    frames = grab(solver, 3)
    sol = None
    for f in frames:
        s, _ = solve_any(solver, f)
        if s is not None and not s.get("single_marker"):
            sol = s
            raw = f
            break
    if sol is None:
        print("no multi-marker solution")
        return 2
    print(f"ids {sol.get('ids')} rms {sol['rms_px']:.2f}")

    tip_uv = solver.project(sol, (TIP_MM[0], TIP_MM[1]), TIP_MM[2])
    print(f"frame {raw.shape[1]}x{raw.shape[0]} tip_uv {np.round(tip_uv, 1).tolist()}")
    centre_uv = solver.project(sol, (0.0, 0.0), 0.0)
    print(f"board-centre uv {np.round(centre_uv, 1).tolist()}")

    rect, valid, n, ppm = solver.rectified(raw, sol, 4.0)
    # annotate the square grid on the rectified view
    out = rect.copy()
    centre = n / 2.0
    for k in range(9):
        p = int(round(centre + (k * 28.0 - 112.0) * ppm))
        cv2.line(out, (p, 0), (p, n - 1), (0, 200, 0), 1)
        cv2.line(out, (0, p), (n - 1, p), (0, 200, 0), 1)
    for rank in range(8):
        for col in range(8):
            x = int(centre + (col * 28.0 - 112.0 + 14.0) * ppm)
            y = int(centre + (rank * 28.0 - 112.0 + 14.0) * ppm)
            cv2.putText(out, f"{chr(97 + col)}{rank + 1}", (x - 14, y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    # tip projection in board millimetres -> rectified pixels
    tx = int(round(centre + TIP_MM[0] * ppm))
    ty = int(round(centre - TIP_MM[1] * ppm))
    cv2.drawMarker(out, (tx, ty), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.imwrite("/tmp/rectified_board.png", out)
    cv2.imwrite("/tmp/raw_wrist.png", raw)
    print("wrote /tmp/rectified_board.png /tmp/raw_wrist.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
