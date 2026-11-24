# Aim-ray 64-square sweep — tip mode vs aim mode

Session 2026-11-24T17:12:38 → 2026-11-24T17:30:46; z 70.0 mm tip height; tolerance 4.0 mm; http://127.0.0.1:8080.

Board frame (console): {"x_mm": -34.92, "y_mm": -18.13, "z_mm": 431.35, "yaw_deg": -178.08}

## Columns

| column | what was commanded | what is measured |
|---|---|---|
| tip | jaw tip → square centre (mode xyz) | aim readout after the move |
| aim_point | aim ray → square centre (mode aim_point) | the app's own final error |
| aim_final | + deadband-compensated rel refinement | independent 3-sample readout |

## Accuracy

| metric | n | p50 | p95 | max | mean | ≤5 mm | ≤10 mm |
|---|---|---|---|---|---|---|---|
| tip |aim − centre| mm | 61 | 9.59 | 14.74 | 18.29 | 8.9 | 12 | 34 |
| aim_point |aim − centre| mm | 60 | 5.46 | 11.93 | 12.48 | 6.77 | 26 | 43 |
| aim_final |aim − centre| mm | 60 | 7.78 | 11.23 | 12.4 | 6.9 | — | — |

Paired (n 60): aim_point better on 31 squares; median improvement 0.19 mm, mean 2.24 mm.

## Pixel evidence (marker-projected square centre)

| metric | n | p50 | p95 | max |
|---|---|---|---|---|
| tip aim px | 49 | 48.05 | 106.23 | 113.79 |
| aim aim px | 39 | 56.47 | 132.74 | 140.82 |
| tip px vs configured | 61 | 56.9 | 81.63 | 107.98 |
| aim px vs configured | 60 | 57.23 | 78.23 | 83.76 |

Marker grid centre minus console's configured centre (px):

- tip: n 49, mean [82.5, -14.8] px, sd [49.0, 19.3] px, distance p50 84.3 px, max 182.1 px
- aim: n 39, mean [89.6, -16.6] px, sd [48.7, 19.9] px, distance p50 93.4 px, max 191.4 px

## Servo deadband

First aim_point step across 51 squares: commanded p50 10.2 mm, achieved p50 0.72 mm; 23 squares moved 0.5 mm or less.

## Refinement

Ran on 36 squares: better on 18, worse on 11.

## Refusals and plane failures (verbatim)

- a1 aim/refine: no reachable pose: off by 6.3 mm / 0.47 deg
- b1 aim/refine: no reachable pose: off by 5.9 mm / 0.47 deg
- c1 aim/refine: no reachable pose: off by 6.2 mm / 0.51 deg
- d1 aim/refine: no reachable pose: off by 6.8 mm / 0.56 deg
- e1 aim/refine: no reachable pose: off by 6.7 mm / 0.54 deg
- f1 aim/refine: no reachable pose: off by 7.8 mm / 0.58 deg
- h1 tip: no reachable pose: off by 0.27 mm / 8.742 deg
- h1 aim: arm is not over the square after the tip leg (tip [1.8, -1.2, 109.1] vs [98.0, -98.0, 70.0]); skipped
- h2 aim/refine: no reachable pose: off by 6.8 mm / 0.54 deg
- b8 aim: plane before aim_point: no usable board plane
- b8 plane: no usable board plane
- c8 tip: no reachable pose: off by 6.65 mm / 6.166 deg
- c8 aim: arm is not over the square after the tip leg (tip [-1.3, 2.1, 111.3] vs [-42.0, 98.0, 70.0]); skipped
- d8 tip: no reachable pose: off by 6.07 mm / 6.38 deg
- d8 aim: arm is not over the square after the tip leg (tip [-1.3, 2.1, 111.3] vs [-14.0, 98.0, 70.0]); skipped

## Per square

| sq | tip mm | aim_point mm | aim_final mm | tip px | aim px |
|---|---|---|---|---|---|
| a1 | 9.2 | 9.5 | 9.5 | — | — |
| b1 | 8.6 | 9.2 | 9.2 | 7.6 | 10.3 |
| c1 | 10.0 | 9.3 | 9.3 | 9.9 | — |
| d1 | 10.8 | 10.3 | 10.3 | 17.6 | 16.2 |
| e1 | 10.0 | 9.9 | 9.8 | 25.5 | 25.5 |
| f1 | 11.3 | 10.4 | 10.4 | — | — |
| g1 | 3.8 | 3.9 | 4.1 | 79.7 | 68.7 |
| h1 | — | — | — | — | — |
| a2 | 11.1 | 3.9 | 3.9 | 40.1 | — |
| b2 | 11.3 | 3.1 | 3.1 | 29.0 | 44.1 |
| c2 | 13.2 | 5.2 | 7.8 | 31.3 | — |
| d2 | 13.6 | 4.9 | 9.7 | 33.6 | 30.6 |
| e2 | 14.2 | 3.9 | 3.8 | 10.4 | 71.0 |
| f2 | 11.2 | 10.1 | 8.4 | 27.3 | 47.5 |
| g2 | 12.3 | 7.2 | 9.6 | 61.4 | — |
| h2 | 9.0 | 9.8 | 9.8 | — | — |
| a3 | 12.2 | 2.6 | 2.6 | 61.7 | 25.6 |
| b3 | 10.6 | 3.2 | 3.2 | 43.6 | 22.8 |
| c3 | 9.8 | 3.5 | 3.5 | 32.1 | 28.6 |
| d3 | 9.6 | 3.2 | 3.3 | — | 53.6 |
| e3 | 12.8 | 4.0 | 3.9 | 25.4 | 67.4 |
| f3 | 12.4 | 4.0 | 4.0 | 35.4 | — |
| g3 | 14.8 | 5.4 | 7.2 | 54.6 | — |
| h3 | 11.8 | 11.0 | 10.5 | 58.8 | — |
| a4 | 10.4 | 4.0 | 6.7 | 52.6 | — |
| b4 | 9.1 | 4.2 | 8.5 | 32.6 | 36.3 |
| c4 | 7.8 | 8.4 | 8.4 | 22.8 | 27.2 |
| d4 | 8.8 | 5.5 | 8.6 | 27.2 | 31.0 |
| e4 | 9.6 | 3.8 | 3.8 | 37.1 | 76.3 |
| f4 | 9.1 | 9.4 | 6.5 | 65.9 | 74.8 |
| g4 | 5.2 | 7.9 | 7.7 | 85.5 | — |
| h4 | 12.6 | 3.3 | 3.3 | — | — |
| a5 | 8.2 | 10.0 | 7.8 | 53.1 | — |
| b5 | 6.8 | 10.6 | 10.3 | 31.1 | 46.9 |
| c5 | 5.6 | 11.0 | 8.8 | 31.0 | 41.2 |
| d5 | 6.2 | 11.4 | 10.4 | 40.2 | 45.4 |
| e5 | 7.5 | 11.2 | 8.2 | 52.7 | 56.5 |
| f5 | 7.2 | 10.0 | 7.6 | — | — |
| g5 | 1.9 | 1.9 | 2.0 | 99.4 | 103.5 |
| h5 | 9.0 | 8.3 | 8.0 | — | — |
| a6 | 7.0 | 12.0 | 12.3 | — | 64.6 |
| b6 | 3.8 | 3.8 | 3.9 | 20.9 | 20.1 |
| c6 | 2.3 | 2.3 | 2.3 | 48.0 | 48.0 |
| d6 | 2.7 | 2.7 | 2.6 | 74.5 | 72.9 |
| e6 | 10.3 | 12.3 | 12.1 | 68.7 | — |
| f6 | 8.2 | 11.7 | 10.1 | 87.7 | 74.1 |
| g6 | 11.7 | 10.7 | 7.9 | 77.2 | — |
| h6 | 18.3 | 3.5 | 3.3 | 64.5 | 133.0 |
| a7 | 4.1 | 7.4 | 12.4 | 11.3 | 68.8 |
| b7 | 1.6 | 1.6 | 1.7 | 20.8 | 26.7 |
| c7 | 1.0 | 1.0 | 1.0 | 63.6 | 63.8 |
| d7 | 1.5 | 1.5 | 1.5 | 97.5 | 96.6 |
| e7 | 10.5 | 11.0 | 10.5 | 80.4 | 83.3 |
| f7 | 13.0 | 5.4 | 11.2 | — | — |
| g7 | 7.0 | 12.5 | 10.6 | 110.8 | 93.6 |
| h7 | 15.7 | 3.9 | 3.9 | 84.5 | 132.7 |
| a8 | 1.5 | 1.5 | 1.5 | — | — |
| b8 | 2.4 | — | — | — | — |
| c8 | — | — | — | — | — |
| d8 | — | — | — | — | — |
| e8 | 4.5 | 5.4 | 6.9 | — | — |
| f8 | 10.2 | 11.9 | 10.2 | 113.8 | 105.1 |
| g8 | 14.7 | 3.6 | 3.6 | 99.3 | 140.8 |
| h8 | 12.1 | 11.8 | 11.1 | 111.7 | 121.5 |

## Interpretation

1. **Aiming at the ray roughly halves the aim-point error.** Same session, same
   board, same height: tip-commanded p50 **9.59 mm** (p95 14.74, max 18.29, n 61)
   versus aim_point p50 **5.46 mm** (p95 11.93, max 12.48, n 60). The share of
   squares inside 5 mm doubles, 12/61 -> 26/60; inside 10 mm 34/61 -> 43/60.
   Where the loop closes (the 17 squares that needed no refinement) it lands at
   p50 **3.55 mm**, i.e. inside the 3-5 mm design tolerance.
2. **It wins where it can actuate, and stalls where it cannot.** The first
   aim_point step was p50 10.2 mm and moved the aim p50 0.72 mm; on 23 of 51
   squares it moved 0.5 mm or less. That is the servos' 3-4 deg directional
   deadband: a 10 mm correction at this reach is ~2.9 deg of joint travel, below
   the threshold at which the arm moves at all. The paired median improvement is
   only 0.19 mm while the mean is 2.24 mm - the distribution is bimodal (big
   wins where the correction clears the deadband, nothing elsewhere), not a
   uniform gain.
3. **The remaining absolute error is not the aim mode - it is the board frame.**
   Projecting each square centre through the marker solve and through the
   console's configured frame (same image, same instant) puts them a median
   **84 px** apart, up to 191 px. At this 60-130 mm viewing range a ~10 mm board
   frame / FK translation projects exactly like that: perspective amplifies it
   across the board, so a constant error looks like a gradient. The aim mode
   moves the ray to the *commanded* coordinate, so it improves in board mm but
   not in the image; on the 37 squares where both columns have a marker grid it
   is 16 px worse (56.5 vs 40.2 px) because the tip mode falls part-way back
   toward the true board as the arm lands short in the direction of the frame
   error. This is the same ~8.6 mm residual the phase-3 alternating solve named
   and deliberately left unapplied (reports/phase3_constant_20261124.md 5, 7).
4. **Consequence for grasp targeting.** Commanding the ray is the right
   primitive and is measurably better in the frame the console commands in, but
   it cannot put the jaws on a visible square centre until the board pose (and
   the FK/hand-eye residual behind it) is corrected. Applying the phase-3 board
   delta (+7.3, -13.5 mm, +2.4 deg yaw) is the next worthwhile step; it is worth
   more than any further aim-loop tuning.
5. **Servo-level floor.** Sub-5 mm centring needs corrections the servos do not
   execute. The deadband-compensated rel refinement (command residual + estimated
   shortfall) recovered 18 squares, degraded 11 and was refused on the six
   rank-1 squares already at the reach limit, so it is reported as an experiment
   rather than as the protocol. A dither / approach-from-one-side move primitive
   in the console is the clean fix.

## Failures, with reasons

- h1 tip: `no reachable pose: off by 0.27 mm / 8.742 deg` (approach tilt limit);
  aim skipped because the arm stayed at the recovery pose.
- c8, d8 tip: `no reachable pose: off by 6.65 / 6.07 mm / 6.17 / 6.38 deg`;
  aim skipped for the same reason.
- b8: `no usable board plane` (marker-poor view; the arm was not commanded).
- a1-f1, h2 refinement: `no reachable pose: off by 5.9-7.8 mm / ~0.5 deg` - the
  correction solve cannot translate further at the near edge with roll frozen.

## Where it wins and where it loses (by rank)

Median aim error, per rank, over the 8 files:

| rank | tip mode mm | aim_point mm | n |
|---|---|---|---|
| 1 | 10.0 | 9.5 | 7 / 7 |
| 2 | 11.8 | 5.0 | 8 / 8 |
| 3 | 12.0 | 3.8 | 8 / 8 |
| 4 | 9.1 | 4.9 | 8 / 7 |
| 5 | 7.0 | 10.3 | 8 / 8 |
| 6 | 7.6 | 7.3 | 8 / 7 |
| 7 | 5.6 | 4.6 | 8 / 8 |
| 8 | 7.4 | 5.4 | 6 / 6 |

Aim mode is a large, unambiguous win on ranks 2-4 (11.6 -> 4.6 mm median), which
is where the reach geometry tilts the tool and the ray is furthest from the tip.
It is a small loss on rank 5 (7.0 -> 10.3 mm), where the tip already lands
inside 5-8 mm and the correction is below the deadband: the attempt adds a few
millimetres of noise and no travel. It also loses by more than 1 mm on 13
squares in total (a5, a6, a7, b5, c5, d5, e5, e6, f5, f6, f8, g4, g7).

**Operating rule this supports:** command the aim ray only when the initial aim
offset exceeds ~10 mm (about the deadband in board millimetres at this height).
Below that, leave the tip placement alone - the correction cannot be actuated and
the loop only perturbs a good pose.
