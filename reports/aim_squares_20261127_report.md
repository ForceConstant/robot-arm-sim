# Aim-ray 64-square sweep — tip mode vs aim mode

Session 2026-11-27T10:47:17 → 2026-11-27T11:05:30; z 70.0 mm tip height; tolerance 4.0 mm; http://127.0.0.1:8080.

Board frame (console): {"x_mm": -11.5, "y_mm": -37.72, "z_mm": 423.77, "yaw_deg": -165.5}

## Columns

| column | what was commanded | what is measured |
|---|---|---|
| tip | jaw tip → square centre (mode xyz) | aim readout after the move |
| aim_point | aim ray → square centre (mode aim_point) | the app's own final error |
| aim_final | + deadband-compensated rel refinement | independent 3-sample readout |

## Accuracy

| metric | n | p50 | p95 | max | mean | ≤5 mm | ≤10 mm |
|---|---|---|---|---|---|---|---|
| tip |aim − centre| mm | 60 | 4.55 | 5.8 | 6.02 | 4.07 | 44 | 60 |
| aim_point |aim − centre| mm | 59 | 3.62 | 6.63 | 6.98 | 3.52 | 39 | 59 |
| aim_final |aim − centre| mm | 59 | 3.14 | 6.75 | 7.2 | 3.53 | — | — |

Paired (n 59): aim_point better on 22 squares; median improvement 0.0 mm, mean 0.59 mm.

## Pixel evidence (marker-projected square centre)

| metric | n | p50 | p95 | max |
|---|---|---|---|---|
| tip aim px | 49 | 95.82 | 144.95 | 167.03 |
| aim aim px | 50 | 97.95 | 157.02 | 190.29 |
| tip px vs configured | 60 | 47.75 | 59.72 | 63.29 |
| aim px vs configured | 59 | 42.83 | 81.8 | 92.58 |

Marker grid centre minus console's configured centre (px):

- tip: n 49, mean [-1.7, 15.4] px, sd [49.8, 71.8] px, distance p50 80.0 px, max 180.6 px
- aim: n 50, mean [-3.1, 10.7] px, sd [50.7, 70.5] px, distance p50 76.4 px, max 180.8 px

## Servo deadband

First aim_point step across 40 squares: commanded p50 4.85 mm, achieved p50 0.27 mm; 24 squares moved 0.5 mm or less.

## Refinement

Ran on 24 squares: better on 10, worse on 9.

## Refusals and plane failures (verbatim)

- a8 aim: plane before aim_point: no usable board plane
- a8 plane: no usable board plane
- d8 tip: no reachable pose: off by 2.3 mm / 13.638 deg
- d8 aim: arm is not over the square after the tip leg (tip [17.9, -24.0, 120.5] vs [-14.0, 98.0, 70.0]); skipped
- e8 tip: no reachable pose: off by 4.58 mm / 23.545 deg
- e8 aim: arm is not over the square after the tip leg (tip [17.9, -24.0, 120.5] vs [14.0, 98.0, 70.0]); skipped
- f8 tip: no reachable pose: off by 4.22 mm / 21.835 deg
- f8 aim: arm is not over the square after the tip leg (tip [17.9, -24.0, 120.5] vs [42.0, 98.0, 70.0]); skipped
- g8 tip: no reachable pose: off by 1.35 mm / 8.873 deg
- g8 aim: arm is not over the square after the tip leg (tip [17.9, -24.0, 120.5] vs [70.0, 98.0, 70.0]); skipped

## Per square

| sq | tip mm | aim_point mm | aim_final mm | tip px | aim px |
|---|---|---|---|---|---|
| a1 | 3.9 | 3.9 | 3.9 | 14.6 | 25.8 |
| b1 | 5.5 | 7.0 | 6.7 | 28.9 | 37.0 |
| c1 | 4.8 | 5.8 | 6.7 | 65.4 | 94.0 |
| d1 | 5.1 | 6.6 | 6.8 | 82.6 | 116.5 |
| e1 | 5.0 | 6.4 | 5.9 | 109.5 | 129.2 |
| f1 | 4.5 | 5.5 | 6.6 | 128.8 | 153.2 |
| g1 | 4.4 | 6.9 | 7.2 | 140.0 | 179.6 |
| h1 | 2.8 | 2.8 | 2.8 | — | — |
| a2 | 5.4 | 6.9 | 7.0 | 22.7 | 40.5 |
| b2 | 5.5 | 6.2 | 6.4 | 34.3 | 56.6 |
| c2 | 5.6 | 5.4 | 6.1 | 49.2 | 72.0 |
| d2 | 3.9 | 3.9 | 3.9 | 65.6 | 65.9 |
| e2 | 4.7 | 5.9 | 5.5 | 90.6 | 111.5 |
| f2 | 4.0 | 5.6 | 5.3 | 104.4 | 131.0 |
| g2 | 3.7 | 3.7 | 3.7 | 131.6 | 136.5 |
| h2 | 4.7 | 6.5 | 6.5 | 167.0 | 190.3 |
| a3 | 5.8 | 5.4 | 6.2 | 45.8 | 48.8 |
| b3 | 4.8 | 5.7 | 5.4 | 41.3 | 58.8 |
| c3 | 4.7 | 5.4 | 5.8 | 49.8 | 62.3 |
| d3 | 4.8 | 5.3 | 5.5 | 63.3 | 78.6 |
| e3 | 5.6 | 5.5 | 4.9 | 83.4 | 100.9 |
| f3 | 4.7 | 5.5 | 5.3 | 105.5 | 114.5 |
| g3 | 4.3 | 5.2 | 5.7 | 116.7 | 136.1 |
| h3 | 1.6 | 1.6 | 1.6 | 142.4 | 145.9 |
| a4 | 1.9 | 1.9 | 1.9 | 80.4 | 79.8 |
| b4 | 4.5 | 3.6 | 3.6 | 64.3 | 68.5 |
| c4 | 4.8 | 5.1 | 3.7 | 64.1 | 69.9 |
| d4 | 4.9 | 4.4 | 4.2 | 64.7 | 71.1 |
| e4 | 4.4 | 4.6 | 4.0 | 80.0 | 81.7 |
| f4 | 5.5 | 1.3 | 1.2 | 103.9 | 82.7 |
| g4 | 0.8 | 0.8 | 0.8 | 100.7 | 99.8 |
| h4 | 3.7 | 3.7 | 3.7 | — | 144.2 |
| a5 | 4.5 | 4.2 | 5.0 | 105.8 | 105.8 |
| b5 | 4.8 | 4.3 | 3.0 | 95.8 | 99.5 |
| c5 | 5.2 | 2.8 | 2.8 | 89.8 | 90.5 |
| d5 | 4.6 | 2.2 | 2.2 | 89.6 | 79.6 |
| e5 | 6.0 | 1.9 | 1.9 | 94.0 | 72.1 |
| f5 | 5.3 | 2.3 | 2.3 | 92.7 | 72.4 |
| g5 | 1.7 | 1.7 | 1.7 | 102.2 | 101.6 |
| h5 | 2.4 | 2.4 | 2.4 | 139.8 | 141.9 |
| a6 | 3.8 | 3.8 | 3.8 | — | — |
| b6 | 6.0 | 2.9 | 2.9 | 119.4 | 129.3 |
| c6 | 5.7 | 2.4 | 2.4 | 118.1 | 120.5 |
| d6 | 5.8 | 1.2 | 1.2 | 113.2 | 106.6 |
| e6 | 5.5 | 1.3 | 1.3 | 108.5 | 85.2 |
| f6 | 0.9 | 0.9 | 0.9 | 78.7 | 79.3 |
| g6 | 1.4 | 1.4 | 1.4 | — | — |
| h6 | 1.1 | 1.1 | 1.1 | 126.1 | 126.9 |
| a7 | 2.1 | 2.1 | 2.1 | — | — |
| b7 | 5.0 | 1.9 | 1.9 | 150.8 | 160.1 |
| c7 | 4.7 | 2.1 | 2.1 | 146.6 | — |
| d7 | 4.3 | 0.3 | 0.3 | 139.9 | 137.7 |
| e7 | 5.5 | 0.4 | 0.4 | 103.4 | 96.5 |
| f7 | 4.4 | 0.8 | 0.8 | — | 53.5 |
| g7 | 0.5 | 0.5 | 0.5 | 101.2 | 101.1 |
| h7 | 0.9 | 0.9 | 0.9 | — | — |
| a8 | 1.5 | — | — | — | — |
| b8 | 4.2 | 1.8 | 2.1 | — | — |
| c8 | 3.1 | 3.1 | 3.1 | — | — |
| d8 | — | — | — | — | — |
| e8 | — | — | — | — | — |
| f8 | — | — | — | — | — |
| g8 | — | — | — | — | — |
| h8 | 2.8 | 2.8 | 2.8 | — | — |
