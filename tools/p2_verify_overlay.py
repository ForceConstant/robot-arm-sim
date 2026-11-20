#!/usr/bin/env python3
"""Prove the served stream actually carries the overlay, without looking at it.

Two numeric checks on one served frame:

* **status line present** — render the exact status string the dashboard should
  print (read from ``/state``), with the same font and scale, and template-match
  it inside the panel region.  A high correlation means those characters are in
  the served bytes.
* **overlay ink present** — count pixels at the marker (green) and target/no
  target (red) colours, in the served frame and in the raw bridge frame, so a
  drawing that silently stopped being applied shows as zero.

    PYTHONPATH=. .venv/bin/python tools/p2_verify_overlay.py
"""
from __future__ import annotations

import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "live"))

from bridge_client import HttpFrames  # noqa: E402
from chess_sim.board_target import TargetDetector  # noqa: E402
from target_overlay import GREEN, RED  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def served_frame(url: str) -> np.ndarray:
    with urllib.request.urlopen(url + "/stream", timeout=20) as response:
        buffer = b""
        while True:
            buffer += response.read(4096)
            head, sep, rest = buffer.partition(b"--frame")
            if not sep:
                continue
            end = rest.find(b"--frame")
            start = rest.find(b"\r\n\r\n")
            if start < 0 or end < 0:
                continue
            payload = rest[start + 4:end].rstrip(b"\r\n")
            image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                return image


def ink(image: np.ndarray, colour: tuple[int, int, int], tol: int = 40) -> int:
    diff = np.abs(image.astype(np.int16) - np.array(colour, np.int16))
    return int((diff.max(axis=2) <= tol).sum())


def main() -> int:
    url = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8080"
    # Let the stream run a few seconds: the overlay draws the *last* detection,
    # and on the very first served frame that is still empty.
    drain_until = time.time() + 3.5
    while time.time() < drain_until:
        served_frame(url)
    config = json.loads((ROOT / "live/live_config.json").read_text())
    state = json.loads(urllib.request.urlopen(url + "/state", timeout=10).read().decode())
    target = state.get("target") or {}
    served = served_frame(url)
    frames = HttpFrames(config["bridge_url"],
                       token_file=str(ROOT / "live/bridge_token.txt"))
    raw = cv2.cvtColor(frames.capture().image, cv2.COLOR_RGB2BGR)
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    result = detector.detect(raw)
    raw_portrait = cv2.rotate(detector.board.rectify(raw), cv2.ROTATE_90_CLOCKWISE)

    status = target.get("status", "starting")
    if status == "target":
        piece = target.get("piece") or {}
        line = (f"target {piece.get('nearest_square')}  {piece.get('x_mm'):.1f},"
                f"{piece.get('y_mm'):.1f} mm")
    elif status == "starting":
        line = "no target - starting"
    else:
        line = f"no target - {target.get('reason') or 'detection failed'}"

    # the panel sits at the bottom-left; render the line and look for it there
    panel_top = served.shape[0] - 200
    strip = served[panel_top:, :]
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2
    (tw, th), _ = cv2.getTextSize(line, font, scale, thick)
    template = np.zeros((th + 8, tw + 8), np.uint8)
    cv2.putText(template, line, (4, th + 2), font, scale, 255, thick, cv2.LINE_AA)
    score = cv2.matchTemplate(cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY), template,
                              cv2.TM_CCOEFF_NORMED)
    best = float(score.max())
    _, _, _, (mx, my) = cv2.minMaxLoc(score)

    print(f"served frame {served.shape[1]}x{served.shape[0]}  (portrait)")
    print(f"dashboard status '{status}'  expected line: {line!r}")
    print(f"status-line match: NCC {best:.3f} at x={mx} y={my + panel_top}")
    print(f"marker ink (green) served {ink(served, GREEN)} px vs raw {ink(raw_portrait, GREEN)} px")
    print(f"target ink (red)   served {ink(served, RED)} px vs raw {ink(raw_portrait, RED)} px")
    print(f"detector on the same raw frame: {result['status']}"
          f"  rms {result['board']['rms_px'] if result['board'] else 'n/a'} px")

    receipt = {
        "checked_at": state.get("target", {}).get("status"),
        "dashboard_status": status,
        "expected_line": line,
        "status_line_ncc": round(best, 3),
        "status_line_at": [mx, my + panel_top],
        "green_served": ink(served, GREEN),
        "green_raw": ink(raw_portrait, GREEN),
        "red_served": ink(served, RED),
        "red_raw": ink(raw_portrait, RED),
        "text_width_px": tw,
        "text_height_px": th,
        "frame": [int(served.shape[1]), int(served.shape[0])],
    }
    (ROOT / "reports/p2_verify.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print("wrote reports/p2_verify.json")
    return 0 if best > 0.55 else 1


if __name__ == "__main__":
    raise SystemExit(main())
