#!/usr/bin/env python3
"""Is the live board frame the same frame the arm moves in?

Two independent answers to "where is the camera over the board":

* the printed markers, through solvePnP against the declared ChArUco layout;
* the joints, through the calibrated chain and `live/board.json`.

They must agree. If the board file and the print disagree by a half turn the
two camera positions land on opposite sides of the board (~2x their distance
from its centre); a weak view biases them by a square instead. Both are worth
knowing: the first is a convention error, the second says the square name on
screen is not trustworthy yet.

Read only: asks the bridge for frames, changes nothing, moves nothing.

    PYTHONPATH=. .venv/bin/python tools/p4_board_frame_check.py
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "live"))

from chess_sim.board_target import TargetDetector, SQUARE_MM    # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
BOUNDARY = b"--frame"


def stream_frames(url: str, count: int, seconds: float) -> list[np.ndarray]:
    frames, buffer, started = [], b"", time.time()
    with urllib.request.urlopen(url + "/stream", timeout=20) as response:
        while time.time() - started < seconds and len(frames) < count:
            chunk = response.read(8192)
            if not chunk:
                break
            buffer += chunk
            while True:
                _, sep, rest = buffer.partition(BOUNDARY)
                if not sep:
                    break
                header_end = rest.find(b"\r\n\r\n")
                if header_end < 0:
                    break
                next_at = rest.find(BOUNDARY, header_end + 4)
                if next_at < 0:
                    break
                payload = rest[header_end + 4:next_at].rstrip(b"\r\n")
                buffer = rest[next_at:]
                image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                if image is not None:
                    frames.append(image)
    return frames


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--config", default="live/live_config.json")
    args = parser.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    board = json.loads((ROOT / "live/board.json").read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board

    state = json.loads(urllib.request.urlopen(args.url + "/state", timeout=10).read())
    joints = [float(state["bridge"]["joints"][name]) for name in JOINTS]
    yaw = math.radians(board["yaw_deg"])
    frame_in_arm = np.array([
        [math.cos(yaw), -math.sin(yaw), 0, board["x_mm"] / 1000],
        [math.sin(yaw), math.cos(yaw), 0, board["y_mm"] / 1000],
        [0, 0, 1, board["z_mm"] / 1000], [0, 0, 0, 1]])
    from_joints = (np.linalg.inv(frame_in_arm) @ solver.camera_from_joints(joints))[:3, 3] * 1000
    print(f"board.json {board}")
    print(f"joints {[round(v, 2) for v in joints]}")
    print(f"FK   camera in board mm {np.round(from_joints, 1)}")

    for index, portrait in enumerate(stream_frames(args.url, args.frames, args.seconds)):
        landscape = cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)
        solution, detections = solver.solve(solver.rectify(landscape))
        if solution is None:
            print(f"frame {index}: {len(detections)} markers, no board pose")
            continue
        pnp = np.asarray(solution["camera_in_board_mm"], dtype=float)
        delta = pnp - from_joints
        flipped = delta + 2 * from_joints[:2]
        print(f"frame {index}: ids {solution['ids']} rms {solution['rms_px']:.2f} px  "
              f"PnP camera in board mm {np.round(pnp, 1)}  "
              f"delta {np.round(delta, 1)} mm ({np.linalg.norm(delta):.1f})  "
              f"a half turn would be {np.linalg.norm(flipped):.1f} mm  "
              f"bias in squares {np.linalg.norm(delta[:2]) / SQUARE_MM:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
