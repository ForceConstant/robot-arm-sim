#!/usr/bin/env python3
"""Read-only focus measurement for the wrist camera.

Settles exposure (standing rule), then compares the service's own focus score
against an independent Laplacian variance on the same frames, and reports
whether the device exposes any focus control at all.

    PYTHONPATH=. .venv/bin/python tools/p4_focus_check.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.board_target import TargetDetector  # noqa: E402
from p4_auto_exposure import api, adequate, measure, GRID  # noqa: E402
from p4_board_frame_check import stream_frames  # noqa: E402

FOCUS_NAMES = ("focus_absolute", "focus_automatic", "focus_auto", "focus_rel")


def settle(solver, grid):
    rows = []
    for exposure, gain in grid:
        applied = api("/camera/controls",
                      {"controls": {"exposure_time_absolute": exposure,
                                    "gain": gain}, "save": False})["applied"]
        time.sleep(0.4)
        m = measure(solver, "", 3)
        rows.append((exposure, gain, applied["exposure_time_absolute"], m))
    pool = [r for r in rows if adequate(r[3])]
    best = max(pool, key=lambda r: r[3]["mean"]) if pool else rows[-1]
    api("/camera/controls",
        {"controls": {"exposure_time_absolute": best[0], "gain": best[1]},
         "save": False})
    time.sleep(0.8)
    return best, rows


def lapvar(gray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def main() -> int:
    config = json.loads((ROOT / "live/live_config.json").read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board

    controls = api("/camera/controls")
    names = [c["name"] for c in controls["controls"]]
    focus = [n for n in names if n in FOCUS_NAMES]
    print(f"device {controls['device']}")
    print(f"controls ({len(names)}): {', '.join(names)}")
    print(f"focus controls present: {focus if focus else 'NONE'}")

    best, rows = settle(solver, GRID)
    exposure, gain, applied_exp, m = best
    print(f"settled: exposure {applied_exp} gain {gain} (requested {exposure})")
    print(f"frame: mean {m['mean']:.1f} std {m['std']:.1f} "
          f"clip {m['clip']:.2f}% dark {m['dark']:.2f}% markers {m['markers']:.0f}")

    scores = []
    for _ in range(3):
        try:
            scores.append(float(api("/focus-score").get("score")))
        except Exception as exc:                     # noqa: BLE001
            print(f"/focus-score failed: {exc!r}")
            break
        time.sleep(0.3)
    if scores:
        print(f"/focus-score: {[round(s, 1) for s in scores]} "
              f"mean {sum(scores)/len(scores):.1f}")

    raw = []
    for portrait in stream_frames("http://127.0.0.1:8080", 3, 8.0):
        raw.append(lapvar(cv2.cvtColor(portrait, cv2.COLOR_BGR2GRAY)))
    print(f"independent Laplacian var (raw stream): {[round(v, 1) for v in raw]} "
          f"mean {sum(raw)/len(raw):.1f}")

    rect = []
    for portrait in stream_frames("http://127.0.0.1:8080", 2, 8.0):
        landscape = cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rect.append(lapvar(cv2.cvtColor(solver.rectify(landscape), cv2.COLOR_BGR2GRAY)))
    print(f"independent Laplacian var (rectified): {[round(v, 1) for v in rect]}")

    live = {c["name"]: c["value"] for c in api("/camera/controls")["controls"]}
    print(f"settled_controls: exposure {live['exposure_time_absolute']} "
          f"gain {live['gain']} contrast {live['contrast']} (save=false)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
