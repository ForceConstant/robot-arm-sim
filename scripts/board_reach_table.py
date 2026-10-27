"""How many of the 64 squares the controller can actually work, per board size.

Uses the same ReachMap the scripted expert uses (IK + tilt cone + near-field),
rebuilding the arm mount for each board because the base sits off the edge.
"""
from __future__ import annotations

import chess
import numpy as np

from chess_sim.config import BoardConfig, EnvConfig
from chess_sim.env import ChessSimEnv

rows = []
for s in (0.024, 0.026, 0.028, 0.030, 0.031, 0.032, 0.034, 0.036):
    cfg = EnvConfig(board=BoardConfig(square=s))
    env = ChessSimEnv(cfg)
    b = env.board
    ax, ay, _ = b.arm_base
    bad = []
    for sq in chess.SQUARES:
        if sq not in env.reach.squares:
            x, y = b.square_center(sq)
            bad.append((chess.square_name(sq), round(float(np.hypot(x - ax, y - ay)) * 1000)))
    far = max(np.hypot(*(np.array(b.square_center(q)) - np.array([ax, ay])))
              for q in chess.SQUARES)
    rows.append((s, b.width, len(env.reach.squares), far, bad))
    print(f"{s*1000:5.1f}mm sq | board {b.width*1000:5.1f}mm | "
          f"{len(env.reach.squares):2d}/64 reachable | farthest centre {far*1000:6.1f}mm | "
          f"missing {bad if bad else '-'}", flush=True)
