"""How much room may sit between the near edge of the squares and the pan axis?

Sweeps the arm-to-board gap for the 28 mm board and reports how many squares the
controller can still work. Two constraints fight each other: too close and the
near rank falls inside the 100 mm near-field exclusion, too far and the far
corners leave the 330 mm reach.
"""
from __future__ import annotations

import chess
import numpy as np

from chess_sim.config import BoardConfig, EnvConfig
from chess_sim.env import ChessSimEnv

S = 0.028
B = 0.015
print(" gap  border | field-edge->axis | reachable | missing")
for gap in np.arange(0.040, 0.1401, 0.010):
    env = ChessSimEnv(EnvConfig(board=BoardConfig(square=S, border=B, arm_gap=float(gap))))
    b = env.board
    ax, ay, _ = b.arm_base
    d = float(gap) + b.border
    bad = []
    for sq in chess.SQUARES:
        if sq not in env.reach.squares:
            x, y = b.square_center(sq)
            bad.append((chess.square_name(sq), round(float(np.hypot(x - ax, y - ay)) * 1000)))
    print(f"{gap*1000:4.0f}  {B*1000:5.0f} | {d*1000:13.0f} mm | "
          f"{len(env.reach.squares):3d}/64   | {bad if bad else '-'}", flush=True)
