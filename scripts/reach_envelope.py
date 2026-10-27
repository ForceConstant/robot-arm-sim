"""Measure the SO-101's usable reach from the built model, then solve for the
largest board that still works.

Sweeps a polar grid around the pan axis at the grasp plane (board top + the
tool height the controller actually uses) and keeps only points the real IK can
hit with the tool inside the tilt cone. The reachable set is an annulus; its
outer edge, minus the base gap and a working margin, is the board limit.
"""
from __future__ import annotations

import sys

import numpy as np

from chess_sim.config import BoardConfig, EnvConfig
from chess_sim.env import GRASP_HEIGHT, ChessSimEnv

MAX_IK_ERROR = 0.008
MAX_TILT = np.radians(30)
NEAR_FIELD = 0.10


def sweep(env, board, step_deg=5.0, r0=0.04, r1=0.40, dr=0.005):
    z = board.top + GRASP_HEIGHT
    bx, by, _ = board.arm_base
    rows = []
    thetas = np.radians(np.arange(0.0, 360.0, step_deg))
    radii = np.arange(r0, r1 + 1e-9, dr)
    for i, th in enumerate(thetas):
        c, s = np.cos(th), np.sin(th)
        for r in radii:
            target = np.array([bx + c * r, by + s * r, z])
            res, mat = env._tool_query(target)
            ok = res.position_error <= MAX_IK_ERROR and res.tilt <= MAX_TILT and r >= NEAR_FIELD
            rows.append((np.degrees(th), r, res.position_error, np.degrees(res.tilt), ok))
        if i % 12 == 0:
            print(f"  {np.degrees(th):5.0f} deg", file=sys.stderr, flush=True)
    return np.array(rows, dtype=float)


def annulus(rows):
    """min/max reachable radius per angle."""
    out = {}
    for th in np.unique(rows[:, 0]):
        sel = rows[(rows[:, 0] == th) & (rows[:, 4] > 0.5)]
        out[th] = (sel[:, 1].min(), sel[:, 1].max()) if len(sel) else (np.nan, np.nan)
    return out


def reachable(ann, r, th_deg):
    keys = np.array(sorted(ann))
    lo = np.nanpercentile([ann[k][0] for k in keys], 5)
    # outer edge varies a little with yaw; interpolate it
    hi = float(np.interp(th_deg % 360.0, keys, [ann[k][1] for k in keys]))
    return lo <= r <= max(hi, 0.0)


def board_report(env, ann, square, gap=0.08, border=0.015, margin=0.0):
    board = BoardConfig(square=square, border=border, arm_gap=gap,
                        origin=(0.0, 0.0), yaw=0.0)
    ax, ay, _ = board.arm_base
    reach, worst = [], 1e9
    import chess
    for sq in chess.SQUARES:
        x, y = board.square_center(sq)
        r = float(np.hypot(x - ax, y - ay))
        th = float(np.degrees(np.arctan2(y - ay, x - ax)))
        ok = reachable(ann, r, th)
        if ok:
            reach.append(sq)
            worst = min(worst, (ann_hi_at(ann, th) - r))
    return board, len(reach), worst


def ann_hi_at(ann, th_deg):
    keys = np.array(sorted(ann))
    return float(np.interp(th_deg % 360.0, keys, [ann[k][1] for k in keys]))


if __name__ == "__main__":
    env = ChessSimEnv(EnvConfig())
    board = env.board
    print(f"board square={board.square*1000:.1f}mm width={board.width*1000:.1f}mm "
          f"top={board.top*1000:.1f}mm base={tuple(round(v,4) for v in board.arm_base)}")
    rows = sweep(env, board)
    np.save("/tmp/reach_rows.npy", rows)
    ann = annulus(rows)
    his = np.array([v[1] for v in ann.values()])
    los = np.array([v[0] for v in ann.values()])
    print(f"\nreach at z={ (board.top+GRASP_HEIGHT)*1000:.1f}mm: "
          f"inner {np.nanpercentile(los,5)*1000:.0f}mm..{np.nanmax(los)*1000:.0f}mm, "
          f"outer {np.nanmin(his)*1000:.0f}..{np.nanmax(his)*1000:.0f}mm "
          f"(median {np.nanmedian(his)*1000:.0f}mm)")
    # outer edge per quadrant
    for q, name in [((0, 90), "right"), ((90, 180), "front"), ((180, 270), "left"), ((270, 360), "back")]:
        sel = [v[1] for k, v in ann.items() if q[0] <= k < q[1] and not np.isnan(v[1])]
        if sel:
            print(f"  {name:5s}: {min(sel)*1000:5.0f}..{max(sel)*1000:5.0f}mm")

    print("\nsquare  width  reachable/64  far-square r  margin")
    for sq in np.arange(0.020, 0.0451, 0.002):
        b, n, worst = board_report(env, ann, float(sq))
        ax, ay, _ = b.arm_base
        far = max(np.hypot(*(np.array(b.square_center(s)) - np.array([ax, ay])))
                  for s in range(64))
        print(f"{sq*1000:6.1f} {b.width*1000:6.1f} {n:5d}        {far*1000:6.1f}mm   {worst*1000:+6.1f}mm")
