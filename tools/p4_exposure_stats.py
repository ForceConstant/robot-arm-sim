#!/usr/bin/env python3
"""Read-only exposure numbers: mean/std, per-channel clipping, markers, PnP rms.

No vision judgement — only statistics and detections.

    PYTHONPATH=. .venv/bin/python tools/p4_exposure_stats.py --frames 3
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
from p4_board_frame_check import stream_frames  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--config", default="live/live_config.json")
    ap.add_argument("--save", default="")
    args = ap.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board

    frames = stream_frames(args.url, args.frames, args.seconds)
    print(f"frames {len(frames)}")
    for i, portrait in enumerate(frames):
        land = cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rect = solver.rectify(land)
        gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
        b, g, r = (rect[:, :, k].astype(np.float32) for k in range(3))
        dets = fiducials.detect_markers(cv2.cvtColor(rect, cv2.COLOR_BGR2RGB))
        solution, _ = solver.solve(rect)
        rms = f"{solution['rms_px']:.2f}" if solution else "none"
        ids = solution["ids"] if solution else [d.marker_id for d in dets]
        print(f"frame {i}: gray mean {gray.mean():6.1f} std {gray.std():5.1f} "
              f"| >=250% B {100*(b>=250).mean():5.1f} G {100*(g>=250).mean():5.1f} "
              f"R {100*(r>=250).mean():5.1f} "
              f"| >=250% gray {100*(gray>=250).mean():5.1f} "
              f"| markers {len(dets)} ids {ids} | PnP rms {rms} px")
        if args.save:
            path = ROOT / args.save.format(i=i)
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), rect)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
