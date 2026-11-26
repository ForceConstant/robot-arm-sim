# Wide-volume mount solve — 2026-11-26 (5q arm, real hardware)

**Verdict: capture and solve completed. The candidate mount must NOT be adopted.**

The wide volume does not pin a single rigid camera mount. Per-view mount estimates
scatter by ~2–3 cm even though each view's own PnP is fine, which means the
pose-dependence comes from the arm pose model, not from the camera mount.

## What ran

| step | command | result |
| --- | --- | --- |
| 1 | `tools/block5q_wide_cal.py` dry run → `reports/block5q_pose_list.json` | 36 poses, worst joint margin +2.8° |
| 2 | capture volume A (mixed 5-DOF grid) → `live/captures_wide_20261126` | 36 legs, **29 captured / 7 refused (409)**, but only 5 views had ≥3 tags → solve stopped |
| 3 | capture volume B (wrist-scan plan poses × roll {0, +45, −45}) → `live/captures_wide2_20261126` | 36 legs, **30 captured / 6 refused (409)**, **21 views with ≥3 tags** |
| 4 | `tools/block5r_wide_solve.py --captures live/captures_wide2_20261126` | solved → `reports/block5r_wide2_20261126.json` |

Refusals are all `HTTP 409` from the bridge at pan ±57° and at extreme
lift/elbow combinations — the guard doing its job, not a fault.

## Solve result

```
candidate translation mm : [ 57.69  -3.72  -4.80]
current   translation mm : [ 63.59  11.10 -13.15]
delta                    : [-5.91 -14.82   8.35]  |d| = 18.0 mm
rotation delta           : 12.25 deg
held-out reprojection    : candidate median 26.78 px (max 46.73, n=10)
                           current   median 60.60 px (max 77.38)
```

So the candidate halves the held-out reprojection error. On its own that looks
like an improvement worth taking. It is not.

## Why it must not be adopted

Per-view mounts (each view solved independently against the fixed board pose),
compared with the averaged candidate:

```
dT (view mount vs candidate)   median 20.5 mm   p90 30.6 mm   max 36.7 mm
per-view reprojection rms      7.20 – 79.58 px   median 19.79 px
corr(tilt, rms) = 0.31         corr(dT, rms) = 0.17
tilt span 2.8° – 32.7°         markers/view 3 – 11
```

A rigid camera mount produces the *same* mount from every view up to corner
noise (a few mm). A median disagreement of 20.5 mm, spread over views that are
each individually well-conditioned (7–11 markers, 2.4–6.1 k px² marker area),
is not corner noise. It is the arm pose model disagreeing with itself across
the volume: the same pose-dependent chain error already measured at ~1.4° per
10° of motion, which is cm-level over these 40–90° swings.

Averaging that scatter produces a mount that absorbs the chain error at the
midpoint of the sampled volume and is wrong everywhere the chain error has the
opposite sign. That is exactly the failure the project invariants forbid: a
refit closing a residual that came from somewhere else.

Note the tilt correlation is weak (0.31) — oblique-view corner bias is *not* the
main driver, so restricting capture to low-tilt views would not fix this.

## What would actually pin the mount

Fit the pose model and the mount together. Two joint offsets that explain the
20 mm scatter are far more plausible than a camera that moves. Concretely, a
bundle adjustment over these already-captured images that frees a small set of
per-joint angle offsets (and optionally link lengths) alongside the 6-DoF mount
would either collapse the scatter — confirming chain bias and yielding numbers
for `wrist_calibration` — or fail to, which would point at the marker layout
scale instead.

No further arm motion is needed for that experiment; the 21 images are on disk.

## State left behind

- Arm homed after both volumes (`/recover` at the end of each run, tip verified
  back at the home pose).
- Camera yaw offset `0.0` (source `reset`), wrist zero accepted at the current
  roll — unchanged by this work.
- `.private/phase11d/calibration.json` **unchanged**. Candidate lives only in
  `reports/block5r_wide2_20261126.json`.
