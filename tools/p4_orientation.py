"""P4: print the sim board convention as numbers. No images, no opinions."""
from __future__ import annotations
import sys, math, json
import chess
from chess_sim.config import BoardConfig, SQUARES, START_FEN
from chess_sim import charuco_face as cf
from chess_sim import fiducials

def table(board):
    rows = []
    for name in chess.SQUARE_NAMES:
        wx, wy = board.square_center(name)
        f = chess.square_file(chess.parse_square(name)); r = chess.square_rank(chess.parse_square(name))
        bx, by = (f - 3.5) * board.square * 1000.0, (r - 3.5) * board.square * 1000.0
        rows.append((name, round(bx,1), round(by,1), round(wx*1000,1), round(wy*1000,1)))
    return rows

def main():
    b = BoardConfig()
    print("## defaults")
    print("yaw_deg", round(math.degrees(b.yaw),3), "arm_side", b.arm_side,
          "origin", b.origin, "square_mm", b.square*1000, "border_mm", round(b.border*1000,1),
          "field_mm", b.field*1000, "width_mm", round(b.width*1000,1), "face", b.face)
    ax, ay, az = b.arm_base
    print("arm_base_mm", [round(v*1000,1) for v in (ax, ay, az)])
    # board frame -> world for the four field corners
    for nm in ("a1","h1","a8","h8"):
        wx, wy = b.square_center(nm)
        print("corner", nm, "world_mm", round(wx*1000,1), round(wy*1000,1))
    rows = table(b)
    # nearest / farthest rank from the arm
    import numpy as np
    arm = np.array([ax, ay])
    per_rank = {}
    for name, bx, by, wx, wy in rows:
        per_rank.setdefault(name[1], []).append(math.hypot(wx-ax*1000, wy-ay*1000))
    for r in sorted(per_rank):
        print("rank", r, "mean_dist_to_arm_mm", round(sum(per_rank[r])/8,1),
              "y_mm", round(sum(p[4] for p in rows if p[0][1]==r)/8,1))
    print("a1_is_corner", min(rows, key=lambda t: math.hypot(t[3]-rows[0][3], t[4]-rows[0][4]))[0])
    print()
    print("## 64 square centres (board mm, world mm)")
    print("name,board_x,board_y,world_x,world_y")
    for r in rows:
        print(",".join(str(v) for v in r))

    print()
    print("## marker id -> square (board frame)")
    for mid in range(0, 37):
        sq = cf.square_of(mid)
        if sq is None:
            continue
        col, rank = sq
        print("id", mid, "col", col, "rank", rank, "square", f"{chr(97+col)}{rank+1}",
              "from", "left-near" if col<4 and rank<4 else "")

    print()
    print("## mat tags 0..3 world corners")
    off = b.field/2*1000 + b.border/2*1000
    for mid, (sx, sy) in fiducials._CENTRE_SIGNS.items():
        bx, by = sx*off, sy*off
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        wx = b.origin[0]*1000 + c*bx - s*by
        wy = b.origin[1]*1000 + s*bx + c*by
        sq = f"{chr(97+min(7,max(0,int(round((bx/ (b.square*1000))+3.5)))))}{int(round(by/(b.square*1000)+3.5))+1}"
        print("tag", mid, "board_mm", round(bx,1), round(by,1), "world_mm", round(wx,1), round(wy,1), "near_square", sq)

    print()
    print("## start position piece colours by rank (START_FEN)")
    st = chess.Board(START_FEN)
    for r in range(8):
        line = []
        for f in range(8):
            p = st.piece_at(chess.square(f, r))
            line.append(("w" if p.color == chess.WHITE else "b") + p.symbol().upper() if p else ".")
        print("rank", r+1, " ".join(line), "y_mm_world", round(sum(p[4] for p in rows if p[0][1]==str(r+1))/8,1))
    print("turn", "white" if st.turn == chess.WHITE else "black")

if __name__ == "__main__":
    main()
