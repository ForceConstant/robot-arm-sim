#!/usr/bin/env python3
"""P4 evidence: the board's facing, rendered, with the numbers printed on it.

Renders the overhead camera on the start position and captions the picture with
the numbers that decide the orientation: the arm base's y, the y of the rank
nearest the arm and the rank farthest from it. Nothing here is read back from
the image; the caption is the same arithmetic `BoardConfig.square_center` does.

    PYTHONPATH=. .venv/bin/python tools/p4_board_orientation.py --out reports/x.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from chess_sim.config import Camera, EnvConfig, START_FEN      # noqa: E402
from chess_sim.env import ChessSimEnv                          # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="reports/p4_board_orientation.png")
    parser.add_argument("--fen", default=START_FEN)
    args = parser.parse_args()

    env = ChessSimEnv(EnvConfig())
    try:
        env.reset(args.fen)
        frame = env.render(Camera.TOP).copy()
        board = env.board
        _, arm_y, _ = board.arm_base
        y1 = board.square_center("a1")[1]
        y8 = board.square_center("a8")[1]
        near = "rank 8 (black)" if abs(y8 - arm_y) < abs(y1 - arm_y) else "rank 1 (white)"
        lines = [
            f"board yaw {np.degrees(board.yaw):+.1f} deg   facing {board.facing}",
            f"arm base y {arm_y * 1000:+.1f} mm",
            f"rank 1 y {y1 * 1000:+.1f} mm   rank 8 y {y8 * 1000:+.1f} mm",
            f"nearest the arm: {near}",
        ]
        canvas = np.full((frame.shape[0] + 96, frame.shape[1], 3), 24, np.uint8)
        canvas[96:] = frame
        for index, line in enumerate(lines):
            cv2.putText(canvas, line, (10, 24 + 22 * index),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1, cv2.LINE_AA)
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), canvas)
        print("\n".join(lines))
        print("wrote", out, canvas.shape[1], "x", canvas.shape[0])
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
