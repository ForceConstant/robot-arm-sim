# Aim-ray 64-square sweep

- session: 2026-11-25T06:43:21 -> 2026-11-25T07:00:47
- base: http://127.0.0.1:8080, z 70.0 mm, tolerance 4.0 mm
- board frame: {'x_mm': -27.62, 'y_mm': -31.63, 'z_mm': 432.65, 'yaw_deg': -175.68}

## Summary

| metric | n | p50 | p95 | max | mean |
|---|---|---|---|---|---|
| aim_err_mm | 60 | 7.44 | 11.6 | 12.01 | 6.48 |
| tip_err_mm | 61 | 9.25 | 15.8 | 16.53 | 9.0 |
| aim_px | 35 | 48.42 | 102.37 | 129.44 | 50.66 |
| tip_px | 47 | 56.25 | 101.48 | 115.4 | 60.4 |
| aim_px_cfg | 60 | 52.92 | 74.95 | 79.73 | 47.47 |
| tip_px_cfg | 61 | 59.82 | 95.17 | 97.89 | 54.43 |
| readout_spread_mm | 60 | 0.0 | 0.1 | 0.3 | 0.02 |
| readout_spread_px | 60 | 0.1 | 2.74 | 6.5 | 0.7 |

## Per square

| sq | tip aim err mm | tip px | aim err mm | aim px | aim conv | iters/refine | plane |
|---|---|---|---|---|---|---|---|
| a1 | 7.5 | -- | 2.7 | -- | refined | 2/2 | ok |
| b1 | 12.9 | 89.1 | 8.3 | 68.2 | no | 3/1 | ok |
| c1 | 12.3 | 85.0 | 8.5 | -- | no | 3/1 | ok |
| d1 | 13.0 | 85.5 | 8.6 | -- | no | 3/1 | ok |
| e1 | 11.3 | 63.6 | 8.6 | 48.4 | no | 3/1 | ok |
| f1 | 11.4 | 35.9 | 10.7 | 37.5 | no | 2/1 | ok |
| g1 | 14.6 | 58.7 | 10.2 | 38.5 | no | 3/1 | ok |
| h1 | 9.2 | 23.2 | 11.9 | 22.9 | no | 2/1 | ok |
| a2 | 13.7 | -- | 7.3 | 94.1 | no | 3/2 | ok |
| b2 | 12.8 | 97.7 | 2.8 | 45.2 | refined | 3/0 | ok |
| c2 | 13.8 | -- | 3.2 | 21.0 | refined | 3/0 | ok |
| d2 | 13.4 | 78.2 | 8.8 | -- | no | 3/1 | ok |
| e2 | 12.8 | -- | 2.8 | 11.7 | refined | 3/0 | ok |
| f2 | 11.9 | 62.1 | 3.1 | 6.0 | refined | 3/0 | ok |
| g2 | 12.4 | 52.3 | 2.7 | -- | refined | 3/0 | ok |
| h2 | 13.1 | 45.1 | 8.8 | -- | no | 3/1 | ok |
| a3 | 12.8 | -- | 2.9 | -- | refined | 3/0 | ok |
| b3 | 4.1 | 52.8 | 3.9 | 62.3 | yes | 1 | ok |
| c3 | 8.5 | 83.5 | 7.5 | -- | no | 2/2 | ok |
| d3 | 9.9 | 76.9 | 3.2 | -- | yes | 3 | ok |
| e3 | 10.4 | 63.6 | 3.5 | 13.9 | yes | 3 | ok |
| f3 | 4.4 | 48.9 | 8.4 | -- | no | 2/2 | ok |
| g3 | 13.3 | 48.0 | 6.5 | 22.7 | no | 3/1 | ok |
| h3 | 12.8 | 47.1 | 3.8 | 42.7 | yes | 3 | ok |
| a4 | 10.0 | 115.4 | 3.1 | -- | yes | 3 | ok |
| b4 | 8.5 | 103.1 | 8.9 | 96.6 | no | 2/2 | ok |
| c4 | 7.4 | 84.5 | 6.6 | 63.8 | no | 2/3 | ok |
| d4 | 7.2 | 53.5 | 8.3 | -- | no | 2/2 | ok |
| e4 | 8.6 | 48.8 | 6.9 | -- | no | 2/3 | ok |
| f4 | 7.4 | 36.6 | 8.0 | 42.5 | no | 2/1 | ok |
| g4 | 14.1 | -- | 3.3 | -- | yes | 3 | ok |
| h4 | 8.1 | 35.7 | 7.7 | 36.5 | no | 2/1 | ok |
| a5 | 9.1 | -- | 9.7 | -- | no | 2/1 | ok |
| b5 | 5.2 | 96.6 | 10.8 | 115.9 | no | 2/1 | ok |
| c5 | 3.6 | 70.7 | 3.8 | 65.8 | yes | 1 | ok |
| d5 | 2.3 | 42.0 | 2.3 | 41.0 | yes | 1 | ok |
| e5 | 5.9 | 47.4 | 10.7 | -- | no | 2/1 | ok |
| f5 | 15.9 | 77.8 | 9.2 | -- | no | 3/1 | ok |
| g5 | 9.2 | 42.1 | 9.2 | -- | no | 2/1 | ok |
| h5 | 16.1 | 56.0 | 3.2 | 54.1 | yes | 3 | ok |
| a6 | 5.2 | 105.0 | 11.1 | 129.4 | no | 2/1 | ok |
| b6 | 2.9 | 73.4 | 2.9 | 69.0 | yes | 1 | ok |
| c6 | 1.2 | 34.9 | 1.2 | 34.8 | yes | 1 | ok |
| d6 | 1.3 | 10.0 | 1.3 | 10.4 | yes | 1 | ok |
| e6 | 7.3 | 36.0 | 11.1 | 57.7 | no | 2/1 | ok |
| f6 | 13.2 | 61.0 | 9.6 | 49.6 | no | 3/1 | ok |
| g6 | 15.8 | 60.4 | 4.0 | 53.5 | refined | 3/0 | ok |
| h6 | 16.5 | 56.2 | 2.7 | -- | refined | 3/0 | ok |
| a7 | 2.1 | -- | 2.1 | -- | yes | 1 | ok |
| b7 | 0.9 | 82.0 | 0.9 | 80.2 | yes | 1 | ok |
| c7 | 0.7 | 42.5 | 0.7 | 41.9 | yes | 1 | ok |
| d7 | 0.9 | 9.6 | 0.9 | 9.8 | yes | 1 | ok |
| e7 | 6.5 | 42.3 | 10.2 | 58.6 | no | 2/1 | ok |
| f7 | 10.6 | -- | 12.0 | -- | no | 2/1 | ok |
| g7 | 11.0 | 54.0 | 11.6 | 60.5 | no | 2/1 | ok |
| h7 | 11.1 | -- | 11.4 | -- | no | 2/1 | ok |
| a8 | 1.4 | -- | 1.4 | -- | yes | 1 | ok |
| b8 | 5.5 | -- | -- | -- | -- | -- | FAILED |
| c8 | -- | -- | -- | -- | -- | -- | ok |
| d8 | -- | -- | -- | -- | -- | -- | ok |
| e8 | -- | -- | -- | -- | -- | -- | ok |
| f8 | 6.8 | 64.2 | 10.0 | 66.3 | no | 2/1 | ok |
| g8 | 11.0 | -- | 11.1 | -- | no | 2/1 | ok |
| h8 | 6.1 | -- | 11.9 | -- | no | 2/1 | ok |

## Failures (verbatim)

- b1 aim/refine: no reachable pose: off by 5.1 mm / 0.44 deg
- f1 aim/refine: no reachable pose: off by 6.9 mm / 0.6 deg
- g1 aim/refine: no reachable pose: off by 7.3 mm / 0.58 deg
- h1 aim/refine: no reachable pose: off by 8.6 mm / 0.65 deg
- b8 aim: plane before aim_point: no usable board plane
- b8 plane: no usable board plane
- c8 tip: no reachable pose: off by 15.77 mm / 3.471 deg
- c8 aim: arm is not over the square after the tip leg (tip [4.7, -11.5, 109.3] vs [-42.0, 98.0, 70.0]); skipped
- d8 tip: no reachable pose: off by 15.39 mm / 10.427 deg
- d8 aim: arm is not over the square after the tip leg (tip [4.7, -11.5, 109.3] vs [-14.0, 98.0, 70.0]); skipped
- e8 tip: no reachable pose: off by 14.29 mm / 2.737 deg
- e8 aim: arm is not over the square after the tip leg (tip [4.7, -11.5, 109.3] vs [14.0, 98.0, 70.0]); skipped

