# Aim-ray 64-square sweep

- session: 2026-11-24T17:12:38 -> 2026-11-24T17:30:46
- base: http://127.0.0.1:8080, z 70.0 mm, tolerance 4.0 mm
- board frame: {'x_mm': -34.92, 'y_mm': -18.13, 'z_mm': 431.35, 'yaw_deg': -178.08}

## Summary

| metric | n | p50 | p95 | max | mean |
|---|---|---|---|---|---|
| aim_err_mm | 60 | 7.78 | 11.23 | 12.4 | 6.9 |
| tip_err_mm | 61 | 9.59 | 14.74 | 18.29 | 8.9 |
| aim_px | 39 | 56.47 | 132.74 | 140.82 | 61.45 |
| tip_px | 49 | 48.05 | 106.23 | 113.79 | 52.07 |
| aim_px_cfg | 60 | 57.23 | 78.23 | 83.76 | 51.21 |
| tip_px_cfg | 61 | 56.9 | 81.63 | 107.98 | 54.6 |
| readout_spread_mm | 60 | 0.0 | 0.3 | 0.5 | 0.05 |
| readout_spread_px | 60 | 0.29 | 4.37 | 11.9 | 1.18 |

## Per square

| sq | tip aim err mm | tip px | aim err mm | aim px | aim conv | iters/refine | plane |
|---|---|---|---|---|---|---|---|
| a1 | 9.2 | -- | 9.5 | -- | no | 2/1 | ok |
| b1 | 8.6 | 7.6 | 9.2 | 10.3 | no | 2/1 | ok |
| c1 | 10.0 | 9.9 | 9.3 | -- | no | 2/1 | ok |
| d1 | 10.8 | 17.6 | 10.3 | 16.2 | no | 2/1 | ok |
| e1 | 10.0 | 25.5 | 9.8 | 25.5 | no | 2/1 | ok |
| f1 | 11.3 | -- | 10.4 | -- | no | 2/1 | ok |
| g1 | 3.8 | 79.7 | 4.1 | 68.7 | yes | 1 | ok |
| h1 | -- | -- | -- | -- | -- | -- | ok |
| a2 | 11.1 | 40.1 | 3.9 | -- | yes | 3 | ok |
| b2 | 11.3 | 29.0 | 3.1 | 44.1 | refined | 3/0 | ok |
| c2 | 13.2 | 31.3 | 7.8 | -- | no | 3/1 | ok |
| d2 | 13.6 | 33.6 | 9.7 | 30.6 | no | 3/1 | ok |
| e2 | 14.2 | 10.4 | 3.8 | 71.0 | yes | 3 | ok |
| f2 | 11.2 | 27.3 | 8.4 | 47.5 | no | 2/2 | ok |
| g2 | 12.3 | 61.4 | 9.6 | -- | no | 3/1 | ok |
| h2 | 9.0 | -- | 9.8 | -- | no | 2/1 | ok |
| a3 | 12.2 | 61.7 | 2.6 | 25.6 | refined | 3/0 | ok |
| b3 | 10.6 | 43.6 | 3.2 | 22.8 | refined | 3/0 | ok |
| c3 | 9.8 | 32.1 | 3.5 | 28.6 | yes | 3 | ok |
| d3 | 9.6 | -- | 3.3 | 53.6 | refined | 3/0 | ok |
| e3 | 12.8 | 25.4 | 3.9 | 67.4 | yes | 3 | ok |
| f3 | 12.4 | 35.4 | 4.0 | -- | yes | 3 | ok |
| g3 | 14.8 | 54.6 | 7.2 | -- | no | 3/1 | ok |
| h3 | 11.8 | 58.8 | 10.5 | -- | no | 2/1 | ok |
| a4 | 10.4 | 52.6 | 6.7 | -- | no | 3/1 | ok |
| b4 | 9.1 | 32.6 | 8.5 | 36.3 | no | 3/1 | ok |
| c4 | 7.8 | 22.8 | 8.4 | 27.2 | no | 2/2 | ok |
| d4 | 8.8 | 27.2 | 8.6 | 31.0 | no | 3/1 | ok |
| e4 | 9.6 | 37.1 | 3.8 | 76.3 | yes | 3 | ok |
| f4 | 9.1 | 65.9 | 6.5 | 74.8 | no | 2/3 | ok |
| g4 | 5.2 | 85.5 | 7.7 | -- | no | 2/2 | ok |
| h4 | 12.6 | -- | 3.3 | -- | refined | 3/0 | ok |
| a5 | 8.2 | 53.1 | 7.8 | -- | no | 2/2 | ok |
| b5 | 6.8 | 31.1 | 10.3 | 46.9 | no | 2/1 | ok |
| c5 | 5.6 | 31.0 | 8.8 | 41.2 | no | 2/3 | ok |
| d5 | 6.2 | 40.2 | 10.4 | 45.4 | no | 2/1 | ok |
| e5 | 7.5 | 52.7 | 8.2 | 56.5 | no | 2/3 | ok |
| f5 | 7.2 | -- | 7.6 | -- | no | 2/3 | ok |
| g5 | 1.9 | 99.4 | 2.0 | 103.5 | yes | 1 | ok |
| h5 | 9.0 | -- | 8.0 | -- | no | 2/1 | ok |
| a6 | 7.0 | -- | 12.3 | 64.6 | no | 2/1 | ok |
| b6 | 3.8 | 20.9 | 3.9 | 20.1 | yes | 1 | ok |
| c6 | 2.3 | 48.0 | 2.3 | 48.0 | yes | 1 | ok |
| d6 | 2.7 | 74.5 | 2.6 | 72.9 | yes | 1 | ok |
| e6 | 10.3 | 68.7 | 12.1 | -- | no | 2/2 | ok |
| f6 | 8.2 | 87.7 | 10.1 | 74.1 | no | 2/2 | ok |
| g6 | 11.7 | 77.2 | 7.9 | -- | no | 2/2 | ok |
| h6 | 18.3 | 64.5 | 3.3 | 133.0 | yes | 3 | ok |
| a7 | 4.1 | 11.3 | 12.4 | 68.8 | no | 2/1 | ok |
| b7 | 1.6 | 20.8 | 1.7 | 26.7 | yes | 1 | ok |
| c7 | 1.0 | 63.6 | 1.0 | 63.8 | yes | 1 | ok |
| d7 | 1.5 | 97.5 | 1.5 | 96.6 | yes | 1 | ok |
| e7 | 10.5 | 80.4 | 10.5 | 83.3 | no | 2/1 | ok |
| f7 | 13.0 | -- | 11.2 | -- | no | 3/1 | ok |
| g7 | 7.0 | 110.8 | 10.6 | 93.6 | no | 2/2 | ok |
| h7 | 15.7 | 84.5 | 3.9 | 132.7 | yes | 3 | ok |
| a8 | 1.5 | -- | 1.5 | -- | yes | 1 | ok |
| b8 | 2.4 | -- | -- | -- | -- | -- | FAILED |
| c8 | -- | -- | -- | -- | -- | -- | ok |
| d8 | -- | -- | -- | -- | -- | -- | ok |
| e8 | 4.5 | -- | 6.9 | -- | no | 2/1 | ok |
| f8 | 10.2 | 113.8 | 10.2 | 105.1 | no | 2/2 | ok |
| g8 | 14.7 | 99.3 | 3.6 | 140.8 | refined | 3/0 | ok |
| h8 | 12.1 | 111.7 | 11.1 | 121.5 | no | 2/1 | ok |

## Failures (verbatim)

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

