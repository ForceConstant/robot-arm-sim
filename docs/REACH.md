# How far the arm actually reaches, and how big a board that allows

Measured from the built model, not from a datasheet. Two scripts:

    .venv/bin/python scripts/reach_envelope.py     # polar sweep of the workspace
    .venv/bin/python scripts/board_reach_table.py   # squares the controller can work, per board size

## Workspace

At the plane the controller actually targets (board top + `GRASP_HEIGHT` = 16 mm),
with the tool inside the same 30° tilt cone the expert uses:

- **near field: 100 mm** from the pan axis. Inside that the servos cannot track
  under load, so squares there are excluded (`reach.NEAR_FIELD_RADIUS`).
- **outer limit: 330 mm** in front. Beyond that the IK cannot hold the tool
  vertical; 335 mm has no solution at any yaw.
- usable band in front: **100–330 mm**. The pan joint covers ±110°, so about
  57 % of the yaw circle is usable at all - irrelevant for a board, which sits
  in one sector, but it is why a naive whole-circle sweep looks half-empty.

Link lengths for reference: 39 mm base offset + 113 mm upper arm + 135 mm
forearm + 61 mm wrist = 347 mm of planar chain. The 330 mm ceiling is that chain
minus what the vertical tool and the tilt cone cost.

## Board size

The base sits `arm_gap` (80 mm) beyond the board edge, so a square's distance
from the pan axis is `r = hypot(3.5·s, 3.5·s + 4·s + border + gap)`. The far
back-rank corners are what fail first.

| square | board | reachable | farthest square centre | first failures |
| --- | --- | --- | --- | --- |
| 24 mm | 222 mm | 64/64 | 288 mm | - |
| 26 mm | 238 mm | 64/64 | 304 mm | - |
| **28 mm** | **254 mm** | **64/64** | **320 mm** | - |
| 30 mm | 270 mm | 61/64 | 337 mm | a8, b8, h8 |
| 31 mm | 278 mm | 60/64 | 345 mm | a8, b8, g8, h8 |
| 32 mm | 286 mm | 56/64 | 353 mm | whole back rank |
| 36 mm | 318 mm | 50/64 | 386 mm | back two ranks |

So **28 mm squares - the current published board - is the practical maximum**:
it is 97 % of reach at the far corners, with about 10 mm of margin. 29 mm is the
absolute ceiling; 30 mm already loses three squares and 32 mm loses the entire
back rank. A standard 57 mm tournament board is nearly twice too big.

Levers, all small:

- trimming the 15 mm border to 5 mm buys about +1 mm of square size;
- pulling the base 20 mm closer would buy about +2.4 mm, but at 60 mm gap the
  near rank falls inside the 100 mm near-field exclusion, so only ~9 mm of that
  is available;
- nothing else helps - the board cannot be turned out of the annulus it sits in.

## Border: how much room may sit in front of the base

For the 28 mm board the distance from the **near edge of the squares to the pan
axis** is a narrow two-sided window (`scripts/border_window.py`):

| field edge to axis | reachable | what fails |
| --- | --- | --- |
| 55 mm | 56/64 | near rank c1-g1, d2-e2 - inside the near field |
| 65 mm | 60/64 | c1, d1, e1, f1 |
| 75 mm | 60/64 | c1, d1, e1, f1 |
| 85 mm | 62/64 | d1, e1 at exactly 100 mm |
| **95 mm** | **64/64** | - |
| 105 mm | 63/64 | a8 at 330 mm |
| 115 mm | 62/64 | a8, h8 |
| 125 mm | 56/64 | whole back rank |

So **85-105 mm, target 95 mm**, which is exactly the published board (15 mm
border + 80 mm arm gap). Close than 85 mm puts the near rank inside the
near-field exclusion - the files in front of the base, where the forearm is most
upright; further than 105 mm starts costing back-rank corners.

A border is free on the far and side edges. Only the near edge counts, and the
arm's own foot needs ~40-60 mm of that budget, leaving 35-50 mm for a decorative
near border.

## Consequence worth remembering

The far corners are at 97 % of reach, which is exactly where IK error, servo
tracking and settle time are worst. Any failure whose source or target square is
on rank 8 (or a/h files) deserves a reach check first: `f8` is 314 mm from the
pan axis, so the bishop incident of 2026-10-27 was executed in that last 5 %.
