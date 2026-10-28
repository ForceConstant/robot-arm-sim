# Grasp analysis for the printed Dubrovnik set

Measured from the supplied meshes at the deployed 50% scale, against the
NormaCore tool's documented envelope. Piece geometry is measured; the finger
envelope (16 mm wide, 3 mm thick, 8 mm tip reach, 32 mm proximal reach) comes
from the tool profile and is not re-measured here.

## What the jaw has to clear

At the deployed 28 mm pitch, the neighbour is never the binding constraint: the
smallest free gap to an adjacent king-sized piece is 8.4 mm at base level, and
the fingers are 3 mm thick. Every piece has a geometrically feasible grasp.
The failures are therefore not "the board is too crowded".

## Where the neck is

Diameter at height fractions of each piece (mm):

| piece | height | 30% | 40% | 50% | 60% | 70% | 80% |
| --- | --- | --- | --- | --- | --- | --- | --- |
| pawn | 23.2 | 11.3 | 6.5 | 4.8 | 6.1 | 4.4 | 7.6 |
| rook | 26.2 | 13.7 | 11.1 | 10.0 | 9.0 | 8.2 | 11.2 |
| knight | 32.3 | 12.2 | 14.0 | 13.1 | 16.0 | 15.7 | 14.3 |
| bishop | 36.8 | 6.7 | 5.0 | 7.3 | 5.8 | 7.4 | 6.5 |
| queen | 40.7 | 9.7 | 7.1 | 6.0 | 10.2 | 7.7 | 11.3 |
| king | 46.8 | 9.8 | 8.3 | 7.6 | 11.0 | 9.7 | 12.1 |

The classic Staunton grip is the narrow waist just under the head: pawn 40–50 %,
rook 60–70 %, queen 50 %, king 50 %, bishop 40–60 %, and the knight has no waist
at all — its body is 12–16 mm across everywhere.

## The bug this exposes

The controller's grasp height for absolute-scale sets is the centre of the
*widest* profile slab above 25 % height. For these shapes that is the crown, and
because the imported slab radii were taken as the maximum over all vertices in
each slab, the chosen height lands too high. Measured against the deployed build:

| piece | controller TCP | piece height | verdict |
| --- | --- | --- | --- |
| pawn | 28.2 mm | 23.2 mm | 5 mm **above the top** — grips air |
| rook | 31.2 mm | 26.2 mm | 5 mm **above the top** — grips air |
| knight | 15.8 mm | 32.3 mm | on the body; picks, places badly |
| bishop | 18.0 mm | 36.8 mm | plausible height |
| queen | 45.7 mm | 40.7 mm | 5 mm **above the top** — grips air |
| king | 39.5 mm | 46.8 mm | on the crown ball, above the neck |

The three pieces clamped above their own top explain the dominant failure
category. Their placement errors were exactly the source-to-target distance
(56 mm for a two-square move), which is the signature of a piece that never left
its square — a missed grasp, not bad aiming. The king's crown grip explains its
toppling; the bishop's low grip explains its neighbour disturbance.

## Recommended plan

Grasp at the measurable waist, with the pads on a cylindrical section and the
fingers straddling the widest feature in the band `[h-8 mm, h+32 mm]`:

| piece | TCP height | contact dia | widest in band | opening | hold | neighbour gap |
| --- | --- | --- | --- | --- | --- | --- |
| pawn | 9.5 mm | 6.3 | 15.7 | 21.7 | 5.8 | 7.4 |
| rook | 18.0 mm | 8.3 | 12.6 | 18.6 | 7.8 | 12.2 |
| knight | 10.0 mm | 11.8 | 17.9 | 23.9 | 11.3 | 6.3 |
| bishop | 15.0 mm | 5.0 | 12.3 | 18.3 | 4.5 | 11.5 |
| queen | 20.5 mm | 6.0 | 11.4 | 17.4 | 5.5 | 14.2 |
| king | 23.5 mm | 7.6 | 12.9 | 18.9 | 7.1 | 12.1 |

Opening is `widest part the fingers straddle + 6 mm`; hold is `contact - 0.5 mm`
squeeze; neighbour gap is the tightest free space the 3 mm finger passes through
against an adjacent king. All openings sit inside the 0–50.95 mm aperture, every
grasp is at least 4 mm below the piece top, and no neighbour gap is negative.

Two caveats belong with these numbers:

- **Bishop contact is 5.0 mm**, so the grip is marginal for a 1.1 g piece with
  bare PLA faces. It may need the collar rather than the waist.
- **Pawn and knight have the least room** (7.4 and 6.3 mm past the finger). They
  are the cases to re-check first after applying the change.
- **The knight has no waist.** It must be gripped on the body, lower than the
  head, with the span across the nose; a tall opening is unavoidable there.

## Fix required in the controller

1. Clamp the grasp height to the piece's usable height (`<= H - 4 mm`).
2. Choose the *narrowest* cylindrical section within 35–75 % of height, subject
   to a 6–14 mm contact diameter, instead of the widest.
3. Compute imported slab radii from the section at the slab centre rather than
   the maximum over the slab, which is what inflates the current profile.
4. Keep a per-piece override for the knight, which has no usable neck.

These are geometric recommendations, not validated grasps. The next gate is to
apply them and re-run `crowded-v1`; a pass rate, not these numbers, is the
evidence.
