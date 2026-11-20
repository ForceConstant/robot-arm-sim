#!/usr/bin/env python3
"""Draw the dashboard's target overlay on one frame and report the text extents.

The still is produced by the same ``target_overlay.draw`` the stream calls, so a
still that looks right is the drawing the dashboard serves — not a re-drawing.
``--frame`` takes any recorded 640x480 frame; without it the bridge is asked for
a fresh one.

    PYTHONPATH=. .venv/bin/python tools/p2_overlay_still.py \
        --frame live/captures_b5b_ref/ref_pose.jpg --out reports/p2_target_e4.png
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "live"))

from bridge_client import HttpFrames  # noqa: E402
from target_overlay import TargetOverlay, draw  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PAGE_WIDTH = 480          # portrait canvas width the dashboard serves


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame", help="recorded 640x480 BGR frame")
    parser.add_argument("--out", default="reports/p2_overlay_still.png")
    parser.add_argument("--config", default="live/live_config.json")
    args = parser.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    overlay = TargetOverlay(str(ROOT / config["calibration"]),
                            str(ROOT / "chess_sim/assets/scene/generated/"
                                      "board_charuco.png"),
                            px_per_mm=4.0, refresh_s=0.5)
    overlay.enabled = False               # drive it by hand, one pass

    if args.frame:
        raw = cv2.imread(str(ROOT / args.frame), cv2.IMREAD_COLOR)
        if raw is None:
            raise SystemExit(f"cannot read {args.frame}")
        source = args.frame
    else:
        frames = HttpFrames(config["bridge_url"],
                            token_file=str(ROOT / "live/bridge_token.txt"))
        captured = frames.capture()
        raw = cv2.cvtColor(captured.image, cv2.COLOR_RGB2BGR)
        source = "live /capture"

    started = time.perf_counter()
    result = overlay.detector.detect(raw)
    detect_ms = (time.perf_counter() - started) * 1000.0
    portrait = overlay.rectify(raw)
    portrait = cv2.rotate(portrait, cv2.ROTATE_90_CLOCKWISE)
    snap = {"result": result, "age_s": 0.0, "state": {"joints": {}, "mode": "HOLD"},
            "state_age_s": 0.0, "mean_cost_ms": round(detect_ms, 1), "runs": 1}
    canvas = draw(portrait.copy(), snap, layout_hw=raw.shape[:2], stream_fps=None)
    (ROOT / args.out).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(ROOT / args.out), canvas)
    overlay.stop()

    print(f"source {source}  {raw.shape[1]}x{raw.shape[0]} landscape -> "
          f"{canvas.shape[1]}x{canvas.shape[0]} portrait")
    print(f"status {result['status']}  reason {result['reason'] or '-'}")
    if result["board"]:
        print(f"markers {result['board']['ids']} rms {result['board']['rms_px']:.2f} px "
              f"range {result['board']['range_mm']:.1f} mm")
    print(f"piece {json.dumps(result['piece'])}")
    print(f"detect {detect_ms:.1f} ms (one pass, cold caches)")
    for index, line in enumerate(_overlay_lines(result)):
        (width, height), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
        print(f"  line {index}: {width:3d} px  {'fits' if width <= PAGE_WIDTH - 16 else 'CLIPS'}  {line}")
    print("wrote", args.out)
    return 0


def _overlay_lines(result: dict) -> list[str]:
    """The panel lines ``draw`` would print, in order (for the width check)."""
    lines = []
    board = result.get("board")
    lines.append(f"pnp {board['rms_px']:.2f} px  ids {len(board['ids'])}"
                 f"  d {board['range_mm']:.0f} mm" if board else "pnp - no board pose")
    lines.append(f"detect  0.0 s ago  {len(result.get('markers', []))} markers"
                 f"  {result.get('squares_in_view', 0)} squares")
    lines.append("cost 14.3 ms  runs 99  fps 4.5")
    lines.append("joints " + " ".join(["+0.0"] * 6))
    lines.append("joints_age_s none  src none  read 0.0 s ago  HOLD")
    if result["status"] == "target":
        piece = result["piece"]
        lines.append(f"target {piece['nearest_square']}  {piece['x_mm']:.1f},"
                     f"{piece['y_mm']:.1f} mm")
    else:
        lines.append(f"no target - {result.get('reason') or 'detection failed'}")
    return lines


if __name__ == "__main__":
    raise SystemExit(main())
