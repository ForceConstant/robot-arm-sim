# Deadband step 5 (blocked) → re-run after the wrist-camera remount

**Date:** 2026-11-25 10:23–10:50 EST
**Why this exists:** step 5 was blocked because the console could not keep a
marker board plane alive. The wrist camera had physically fallen off its mount;
Brian remounted it. This receipt re-runs the *same* 64-square sweep with the
*same* settings as the committed baseline and quantifies whether the mount
calibration still describes the camera.

**Configuration:** P=32, I=0, D=32, take-up ON at 3.5°. z 70 mm,
tolerance 4.0 mm, iterations 3, refine 3. Nothing in P/I/D, `live/board.json`,
the mount calibration or the gripper was touched. Nothing committed.

- new receipt: `reports/aim_squares_out/aim_squares_remount_20261125.json`
- baseline: `reports/aim_squares_out/aim_squares_boardfix_20261125.json`
- log: `reports/aim_squares_out/aim_squares_remount_20261125.log`
- run: 2026-11-25T10:25:16 → 10:46:03 (20 m 47 s), BC-250, `robot-live` not restarted

## 0. Preconditions, verified before any command

| check | result |
|---|---|
| home first (`tools/home_arm.py`) | tip (6.2, −14.9, 112.7) mm, **16.1 mm** from centre, rc 0 |
| `/metrics` with a `/stream` client open | **7 markers** (19,18,22,35,29,34,33), focus score **1146–1158** |
| `/metrics` with **no** client | score 6.2, 0 markers — the overlay/`capture()` path only produces usable frames while a stream client is attached |
| `/aim-point` | ok, plane 9 samples, spread **0.06 mm**, rms 1.19 px, 7 ids, aim e4 at board **(6.0, −13.4)** |

A persistent `/stream` client was held for the whole run (`aim_squares.py` also
opens its own). This is why the plane never aged out; the earlier "no board
plane" failures are gone (`plane_failures` 1 → **0**).

## 1. Comparison vs the committed baseline

| metric | before (06:43) | after remount | Δ |
|---|---|---|---|
| aim board-mm p50 | 7.44 | **2.62** | −65% |
| aim board-mm p95 | 11.60 | **6.13** | −47% |
| aim board-mm max | 12.01 | **6.56** | −45% |
| tip FK-mm (xy) p50 | 9.25 | **4.44** | −52% |
| tip FK-mm p95 | 15.80 | 6.25 | −60% |
| tip FK-mm max | 16.53 | 7.63 | −54% |
| aim px p50 (marker projection) | 48.42 | 58.28 | +20% |
| aim px p50 (configured frame) | 52.92 | **23.37** | −56% |
| aim px p95 (configured) | 74.95 | 60.70 | −19% |
| final aim ≤ 5 mm | 29/60 (48%) | **45/61 (74%)** | +26 pp |
| both legs completed | 60 | 61 | +1 |
| aim converged | 26 | 45 | +19 |
| plane failures | 1 | **0** | — |
| tip leg `error_deg` p50 / max | 2.27 / 3.47 | **1.18 / 2.36** | −48% |
| failures recorded | 12 | 7 | −5 |

`tools/aim_squares_compare.py` output is the source for every row above.

## 2. Mount-validity metric (the explicit question)

Metric as asked: per-leg **|`square_uv` − `square_uv_cfg`|** — the marker-solve
projection of the square centre vs the configured-frame projection.

| | before | after | earlier degraded state |
|---|---|---|---|
| p50 | 43.7 px | **53.2 px** | 89.9 px |
| p90 | 80.3 | 98.1 | — |
| max | 104.0 | 119.0 | — |
| legs with both projections | 82 | 99 | — |
| tip / aim split | 41.5 / 48.8 | 47.0 / 54.4 | — |

**Reading:** 53.2 px is **marginally above** the 50 px line (+22% vs baseline,
+9.5 px) but nowhere near the 89.9 px that the broken state produced. It is not
a "ballooned" signature. Three checks say the change is mostly measurement
conditioning, not a physical move:

1. **Paired common-leg analysis** (73 legs present in both runs): median `du`
   barely moved (+6.3 → +4.6 px); median `dv` moved −2.4 → **+14.8 px**. So it
   is one axis, ≈ 11 mm at 440 mm.
2. **Offset coherence** (|mean offset| / mean |offset|) is 0.30 before and 0.44
   after — the offsets scatter with the view, not a single rigid shift. The
   baseline's `square_uv` came from **1–2 marker solves**, this run's from
   **2–7** (including ids 33/34/35 at 480–525 mm); a 2-marker solve constrains
   the plane poorly, so projected points far from the visible tags move tens of
   pixels. Part of 43.7 → 53.2 is the marker-set changing, not the camera.
3. **Like-for-like by marker count** (the conditioning control): legs with
   3 markers in view give p50 **47.6 px before vs 46.3 px after** — identical;
   2-marker legs 46.6 vs 54.4; 4-marker legs 34.9 vs 74.4 (n=11–12, noisy).
   At equal conditioning the metric does not move at all.
4. **Independent 3D check at home** (read-only, joints −12.48/−12.70/6.20/
   98.07/−0.18): marker solve puts the camera at board `[26.6, −65.6, 204.1]` mm,
   joints + mount calibration predict `[23.5, −76.8, 206.8]` mm →
   Δ `[3.1, 11.2, −2.7]` mm, **norm 12.0 mm**. That is *inside* the ~19 mm
   frame/model bias already documented in step 6, and it is on the same axis
   (+y) as the `dv` shift.

**Verdict on the mount: it held.** The 12 mm residual is smaller than the frame
bias the calibration already had, there is no coherent rigid shift, and every
vision-referenced number improved — impossible for a mount that had moved enough
to invalidate aim. The one real signal is the ≈11 mm +y term: if the ~19 mm frame
bias is ever tightened, re-fitting the mount transform is the first thing to
redo (not done here; the calibration was deliberately left alone).

## 3. Failures (verbatim)

```
a2 aim/refine: plane went stale mid-refine: board plane is stale: last marker solve 30 s ago (limit 30 s)
c8 tip: no reachable pose: off by 15.78 mm / 3.458 deg
c8 aim: arm is not over the square after the tip leg (tip [6.0, -13.4, 112.1] vs [-42.0, 98.0, 70.0]); skipped
d8 tip: no reachable pose: off by 15.39 mm / 10.427 deg
d8 aim: arm is not over the square after the tip leg (tip [6.0, -13.4, 112.1] vs [-14.0, 98.0, 70.0]); skipped
e8 tip: no reachable pose: off by 13.73 mm / 3.523 deg
e8 aim: arm is not over the square after the tip leg (tip [6.0, -13.4, 112.1] vs [14.0, 98.0, 70.0]); skipped
```

- **c8/d8/e8** are the same geometric refusals as the baseline (15.77 / 15.39 /
  14.29 mm then, 15.78 / 15.39 / 13.73 mm now): the back-corner reach limit at
  y=+98 mm, **unchanged by the camera**. The tip leg was refused, so the aim leg
  was correctly skipped rather than commanded blind; the arm stayed at home.
- **a2** is new and benign: a refine step was refused when the solve went 30 s
  stale. The square still recorded its aim read (6.6 mm) — no blind command.
- Baseline failures that disappeared: b1/f1/g1/h1 refine refusals and b8 "no
  usable board plane".

## 4. Is the servo tuning visible in placement? Did the camera change it?

**Yes, and yes — the two effects are separable.**

- **Servo (open loop, no vision):** the tip leg measures FK placement from joints
  only. `error_deg` p50 **2.27 → 1.18** and tip FK-mm p50 **9.25 → 4.44 mm**.
  The baseline predates the take-up + P=32 work, so this is that work showing up
  in placement numbers, roughly the same ratio as the joint-level settle
  improvement in step 6.
- **Camera (closed loop on vision):** plane failures 1 → 0, aim legs 60 → 61,
  aim-converged 26 → 45, and the configured-frame pixel delta halved
  (52.9 → 23.4 px). The remount is what restored the ability to *measure* aim at
  every square at all.
- **The ~9 mm frame residual did not mask the improvement.** The vision-referenced
  aim error fell 4.8 mm at p50; that is well above the residual's own scatter.
  What the residual does do is set the floor: with p50 now 2.62 mm, the dominant
  remaining term is no longer servo settle but the board-frame/mount bias
  (12 mm 3D, ~34 mm in the marker-vs-configured projection).
- **Caveat, stated plainly:** aim board-mm is a *convergence* number — the loop
  drives the reported ray/plane intersection onto the target, so a constant
  frame bias is partly absorbed. The bias-free placement truth is the tip
  FK-mm (4.44) plus the 12 mm mount residual, not the 2.62. Also, `aim px
  (marker)` rose 48.4 → 58.3 px: that column is |aim_pixel − square_uv| and
  inherits the same per-view marker conditioning as the mount metric, so it is
  not evidence that aim got worse. The clean pixel column is
  `aim px (configured)`, which halved.

## 5. Verdict

- **Sweep completed:** yes — 64/64 tip legs attempted, 61/64 aim legs completed,
  61 numbers on both legs, 0 plane failures, 7 refusals all recorded with reasons.
- **Placement improved materially:** aim p50 7.44 → **2.62 mm**, ≤5 mm 48% → 74%.
- **Mount held:** mount metric 43.7 → **53.2 px** (marginal, not ballooned);
  3D camera residual **12.0 mm**, within the existing ~19 mm frame bias, no
  coherent rigid shift. Aim numbers from this run are valid.
- **Next limit is the frame/mount bias, not the servo.** The elbow stiction band
  (0.6–0.8°) is now below the measurement floor of this sweep; further gain
  would come from re-fitting the mount/board frame, which is a separate decision
  and was not touched here.
