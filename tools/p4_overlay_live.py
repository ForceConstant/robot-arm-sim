#!/usr/bin/env python3
"""P4 evidence: the dashboard overlay with the board's own square names.

Captures real frames from the wrist camera over the bridge (read only - no
motion, no mode change), runs the same ``TargetDetector`` the dashboard runs,
and draws through the same ``target_overlay.draw`` the stream calls. Writes a
still and a clip of what the overlay renders, and prints the numbers behind
them: markers per frame, PnP RMS, how many square labels were actually drawn
and how tall the glyphs came out at the served resolution.

    PYTHONPATH=. .venv/bin/python tools/p4_overlay_live.py --frames 24
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "live"))

from bridge_client import HttpFrames  # noqa: E402
from target_overlay import TargetOverlay, draw  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--period", type=float, default=0.35)
    parser.add_argument("--out", default="reports/p4_overlay_grid.png")
    parser.add_argument("--clip", default="reports/p4_overlay_grid.mp4")
    parser.add_argument("--config", default="live/live_config.json")
    parser.add_argument("--frame", help="a recorded 640x480 landscape frame instead of the bridge")
    parser.add_argument("--still", default=None, help="write the drawn still here")
    args = parser.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    overlay = TargetOverlay(str(ROOT / config["calibration"]),
                            str(ROOT / "chess_sim/assets/scene/generated/"
                                      "board_charuco.png"),
                            px_per_mm=4.0, refresh_s=0.5)
    overlay.enabled = False                     # drive it by hand, one pass each
    if args.frame:
        image = cv2.imread(str(ROOT / args.frame), cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit(f"cannot read {args.frame}")
        pool = [image] * max(1, args.frames)
    else:
        frames = HttpFrames(config["bridge_url"],
                            token_file=str(ROOT / config["bridge_token"]
                                           if "bridge_token" in config else "live/bridge_token.txt"))
        pool = None

    rows, canvases = [], []
    best = None
    for index in range(args.frames):
        if pool is not None:
            raw = pool[index].copy()
        else:
            captured = frames.capture()
            raw = cv2.cvtColor(captured.image, cv2.COLOR_RGB2BGR)
        started = time.perf_counter()
        result = overlay.detector.detect(raw)
        cost = (time.perf_counter() - started) * 1000.0
        portrait = cv2.rotate(overlay.rectify(raw), cv2.ROTATE_90_CLOCKWISE)
        snap = {"result": result, "age_s": 0.0,
                "state": {"joints": {}, "mode": "HOLD"}, "state_age_s": 0.0,
                "mean_cost_ms": round(cost, 1), "runs": index + 1}
        stats: dict = {}
        canvas = draw(portrait.copy(), snap, layout_hw=raw.shape[:2], stats=stats)
        visible = stats.get("label_names", [])
        row = {"frame": index, "status": result["status"],
               "markers": [m["id"] for m in result["markers"]],
               "rms_px": round(result["board"]["rms_px"], 2) if result["board"] else None,
               "labels": stats.get("labels", 0), "label_px": stats.get("label_px", 0),
               "label_px_min": stats.get("label_px_min", 0),
               "label_px_max": stats.get("label_px_max", 0),
               "label_w_px": stats.get("label_w_px", 0),
               "square_px": stats.get("square_px", 0),
               "labels_over_square": stats.get("labels_over_square", 0),
               "target": (result.get("piece") or {}).get("nearest_square"),
               "cost_ms": round(cost, 1),
               "squares_named": visible,
               "squares_in_view": result.get("squares_in_view", 0)}
        rows.append(row)
        canvases.append(canvas)
        if result["board"] and (best is None or len(result["markers"]) > len(best[0])):
            best = (result["markers"], canvas)
        if index == 0:
            cv2.imwrite(str(ROOT / args.out), canvas)
        time.sleep(max(0.0, args.period - (time.perf_counter() - started)))
    overlay.stop()

    if args.still:
        cv2.imwrite(str(ROOT / args.still), canvases[0])
    if best is not None:
        cv2.imwrite(str(ROOT / args.out), best[1])

    height, width = canvases[0].shape[:2]
    writer = cv2.VideoWriter(str(ROOT / args.clip), cv2.VideoWriter_fourcc(*"mp4v"),
                             max(1.0, 1.0 / args.period), (width, height))
    for canvas in canvases:
        writer.write(canvas)
    writer.release()

    posed = [r for r in rows if r["rms_px"] is not None]
    print(f"portrait {width}x{height} from {raw.shape[1]}x{raw.shape[0]} landscape, "
          f"{len(rows)} frames")
    print(f"posed {len(posed)}/{len(rows)}  markers per frame "
          f"min {min((len(r['markers']) for r in rows), default=0)} "
          f"median {statistics.median([len(r['markers']) for r in rows]) if rows else 0:.0f} "
          f"max {max((len(r['markers']) for r in rows), default=0)}")
    if posed:
        print(f"pnp rms px min {min(r['rms_px'] for r in posed):.2f} "
              f"median {statistics.median([r['rms_px'] for r in posed]):.2f} "
              f"max {max(r['rms_px'] for r in posed):.2f}")
    labels = [r["labels"] for r in rows]
    print(f"square labels drawn per frame: min {min(labels)} median "
          f"{statistics.median(labels):.0f} max {max(labels)}")
    if any(r["label_px"] for r in rows):
        px = [r["label_px"] for r in rows if r["label_px"]]
        print(f"label glyph height px: median {statistics.median(px):.0f} "
              f"min {min(px)} max {max(px)}")
    print(f"label width px median {statistics.median([r['label_w_px'] for r in rows]):.0f}"
          f"  projected square edge px median "
          f"{statistics.median([r['square_px'] for r in rows]):.0f}"
          f"  names wider than their square (min-scale floor): "
          f"{max(r['labels_over_square'] for r in rows)}")
    print(f"cost ms mean {statistics.mean([r['cost_ms'] for r in rows]):.1f} "
          f"max {max(r['cost_ms'] for r in rows):.1f}")
    for row in rows[:6]:
        print(f"  frame {row['frame']:2d} {row['status']:9s} ids {row['markers']} "
              f"rms {row['rms_px']} labels {row['labels']} {row['label_px']}px "
              f"target {row['target']} squares {' '.join(row['squares_named'])}")
    (ROOT / args.out).with_suffix(".json").write_text(json.dumps(rows, indent=1))
    print("wrote", args.out, "and", args.clip)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
