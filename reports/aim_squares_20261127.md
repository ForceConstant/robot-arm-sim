# Aim-ray 64-square sweep

- session: 2026-11-27T10:47:17 -> 2026-11-27T11:05:30
- base: http://127.0.0.1:8080, z 70.0 mm, tolerance 4.0 mm
- board frame: {'x_mm': -11.5, 'y_mm': -37.72, 'z_mm': 423.77, 'yaw_deg': -165.5}

## Summary

| metric | n | p50 | p95 | max | mean |
|---|---|---|---|---|---|
| aim_err_mm | 59 | 3.14 | 6.75 | 7.2 | 3.53 |
| tip_err_mm | 60 | 4.55 | 5.8 | 6.02 | 4.07 |
| aim_px | 50 | 97.95 | 157.02 | 190.29 | 98.87 |
| tip_px | 49 | 95.82 | 144.95 | 167.03 | 93.0 |
| aim_px_cfg | 59 | 42.83 | 81.8 | 92.58 | 48.48 |
| tip_px_cfg | 60 | 47.75 | 59.72 | 63.29 | 44.51 |
| readout_spread_mm | 59 | 0.0 | 0.1 | 0.3 | 0.02 |
| readout_spread_px | 59 | 0.16 | 3.41 | 4.3 | 0.79 |

## Per square

| sq | tip aim err mm | tip px | aim err mm | aim px | aim conv | iters/refine | plane |
|---|---|---|---|---|---|---|---|
| a1 | 3.9 | 14.6 | 3.9 | 25.8 | yes | 1 | ok |
| b1 | 5.5 | 28.9 | 6.7 | 37.0 | no | 2/1 | ok |
| c1 | 4.8 | 65.4 | 6.7 | 94.0 | no | 2/1 | ok |
| d1 | 5.1 | 82.6 | 6.8 | 116.5 | no | 2/1 | ok |
| e1 | 5.0 | 109.5 | 5.9 | 129.2 | no | 2/1 | ok |
| f1 | 4.5 | 128.8 | 6.6 | 153.2 | no | 2/1 | ok |
| g1 | 4.4 | 140.0 | 7.2 | 179.6 | no | 2/1 | ok |
| h1 | 2.8 | -- | 2.8 | -- | yes | 1 | ok |
| a2 | 5.4 | 22.7 | 7.0 | 40.5 | no | 2/1 | ok |
| b2 | 5.5 | 34.3 | 6.4 | 56.6 | no | 2/1 | ok |
| c2 | 5.6 | 49.2 | 6.1 | 72.0 | no | 2/1 | ok |
| d2 | 3.9 | 65.6 | 3.9 | 65.9 | yes | 1 | ok |
| e2 | 4.7 | 90.6 | 5.5 | 111.5 | no | 2/1 | ok |
| f2 | 4.0 | 104.4 | 5.3 | 131.0 | no | 2/1 | ok |
| g2 | 3.7 | 131.6 | 3.7 | 136.5 | yes | 1 | ok |
| h2 | 4.7 | 167.0 | 6.5 | 190.3 | no | 2/1 | ok |
| a3 | 5.8 | 45.8 | 6.2 | 48.8 | no | 2/1 | ok |
| b3 | 4.8 | 41.3 | 5.4 | 58.8 | no | 2/1 | ok |
| c3 | 4.7 | 49.8 | 5.8 | 62.3 | no | 2/1 | ok |
| d3 | 4.8 | 63.3 | 5.5 | 78.6 | no | 2/1 | ok |
| e3 | 5.6 | 83.4 | 4.9 | 100.9 | no | 2/2 | ok |
| f3 | 4.7 | 105.5 | 5.3 | 114.5 | no | 2/1 | ok |
| g3 | 4.3 | 116.7 | 5.7 | 136.1 | no | 2/1 | ok |
| h3 | 1.6 | 142.4 | 1.6 | 145.9 | yes | 1 | ok |
| a4 | 1.9 | 80.4 | 1.9 | 79.8 | yes | 1 | ok |
| b4 | 4.5 | 64.3 | 3.6 | 68.5 | yes | 2 | ok |
| c4 | 4.8 | 64.1 | 3.7 | 69.9 | refined | 2/1 | ok |
| d4 | 4.9 | 64.7 | 4.2 | 71.1 | no | 2/1 | ok |
| e4 | 4.4 | 80.0 | 4.0 | 81.7 | refined | 2/1 | ok |
| f4 | 5.5 | 103.9 | 1.2 | 82.7 | yes | 3 | ok |
| g4 | 0.8 | 100.7 | 0.8 | 99.8 | yes | 1 | ok |
| h4 | 3.7 | -- | 3.7 | 144.2 | yes | 1 | ok |
| a5 | 4.5 | 105.8 | 5.0 | 105.8 | no | 2/1 | ok |
| b5 | 4.8 | 95.8 | 3.0 | 99.5 | refined | 2/1 | ok |
| c5 | 5.2 | 89.8 | 2.8 | 90.5 | yes | 2 | ok |
| d5 | 4.6 | 89.6 | 2.2 | 79.6 | yes | 2 | ok |
| e5 | 6.0 | 94.0 | 1.9 | 72.1 | yes | 2 | ok |
| f5 | 5.3 | 92.7 | 2.3 | 72.4 | yes | 2 | ok |
| g5 | 1.7 | 102.2 | 1.7 | 101.6 | yes | 1 | ok |
| h5 | 2.4 | 139.8 | 2.4 | 141.9 | yes | 1 | ok |
| a6 | 3.8 | -- | 3.8 | -- | yes | 1 | ok |
| b6 | 6.0 | 119.4 | 2.9 | 129.3 | yes | 2 | ok |
| c6 | 5.7 | 118.1 | 2.4 | 120.5 | yes | 2 | ok |
| d6 | 5.8 | 113.2 | 1.2 | 106.6 | yes | 2 | ok |
| e6 | 5.5 | 108.5 | 1.3 | 85.2 | yes | 2 | ok |
| f6 | 0.9 | 78.7 | 0.9 | 79.3 | yes | 1 | ok |
| g6 | 1.4 | -- | 1.4 | -- | yes | 1 | ok |
| h6 | 1.1 | 126.1 | 1.1 | 126.9 | yes | 1 | ok |
| a7 | 2.1 | -- | 2.1 | -- | yes | 1 | ok |
| b7 | 5.0 | 150.8 | 1.9 | 160.1 | yes | 2 | ok |
| c7 | 4.7 | 146.6 | 2.1 | -- | yes | 2 | ok |
| d7 | 4.3 | 139.9 | 0.3 | 137.7 | yes | 2 | ok |
| e7 | 5.5 | 103.4 | 0.4 | 96.5 | yes | 2 | ok |
| f7 | 4.4 | -- | 0.8 | 53.5 | yes | 2 | ok |
| g7 | 0.5 | 101.2 | 0.5 | 101.1 | yes | 1 | ok |
| h7 | 0.9 | -- | 0.9 | -- | yes | 1 | ok |
| a8 | 1.5 | -- | -- | -- | -- | -- | FAILED |
| b8 | 4.2 | -- | 2.1 | -- | yes | 2 | ok |
| c8 | 3.1 | -- | 3.1 | -- | yes | 1 | ok |
| d8 | -- | -- | -- | -- | -- | -- | ok |
| e8 | -- | -- | -- | -- | -- | -- | ok |
| f8 | -- | -- | -- | -- | -- | -- | ok |
| g8 | -- | -- | -- | -- | -- | -- | ok |
| h8 | 2.8 | -- | 2.8 | -- | yes | 1 | ok |

## Failures (verbatim)

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
