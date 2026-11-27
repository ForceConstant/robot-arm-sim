# Escalation outcome and the two tests it recommended — 2026-11-27

Reviewer: GPT-6.1 Sol at max effort, clean context, brief in the escalation run
(`escalate-pose-error`). Its verdict: **do not start with chain bundle adjustment.**
Two cheaper decisive tests first. I ran both, plus a third that fell out of the
same data. The picture changes materially.

## Verdict from the reviewer

- Chain BA is a "reasonable conditional third step, not the best first step".
- Most likely root cause (moderate confidence): mismatch between reported joint
  coordinates and the physical chain — joint scale, reference or axis/frame
  convention.
- It also correctly weakened my own argument: **with a stale fixed board pose, a
  perfectly good chain still produces pose-dependent inferred mounts.** The mount
  scatter alone never proved chain error.
- It flagged a real inconsistency in the checked-in solver and a calibration
  artifact question. Both checked below.

## Test A — relative rotations, no motion (`reports/wide3_rottest_20261127.json`)

For a correct rigid chain, `F_i^-1 F_j == M_i M_j^-1`, so the *rotation angles*
must agree. This is invariant to the board pose, to the mount, and to any constant
joint zero offset — it tests joint **scale/gain** and axis geometry only.

```
pairs 253 (23 views), FK rotation angle median 54.7 deg, max 178.0
pairs with FK rotation > 8 deg: n=240
angle(B) = 1.000 * angle(A)    corr 0.999
residual |angle(B) - angle(A)|: median 0.67 deg, p90 2.56, max 8.83
worst pairs: 06-23 27.6->36.4 (8.8); 05-06 19.0->27.2 (8.2); 09-23 21.7->28.6 (6.9)
```

**Slope 1.000 over 240 pairs spanning up to 178°.** There is no angular gain error
and no axis/frame error of the kind hypothesised. The reviewer's leading
hypothesis is eliminated, and so is my "1.4° per 10° chain error" framing: across
the volume, chain rotations track the images essentially exactly.

## Test B — bidirectional elbow returns (`live/rep_test_20261127/returns.json`)

Centre posture from the scan plan (pan -0.2, lift -58.7, elbow 45.4, flex 73.7,
roll 0), gripper held at its measured value, 8 markers visible throughout. Three
returns from elbow +6° and three from -6°, captured at 0.5 / 2 / 5 s dwell.

```
stationary jitter at a fixed pose        0.12 mm / 0.14 deg
dwell dependence (0.5 vs 5 s)            < 0.2 mm      -> nothing is creeping
returns from +6 (3 reps)                 6.46 / 6.50 / 6.60 mm   repeatable
returns from -6 (3 reps)                 0.66 / 0.73 / 0.77 mm   repeatable
residual joint error on return           +6: up to 1.06 deg    -6: 0.18 deg
```

Repeatable to ~0.1 mm, identical at every dwell. The +6 returns differ from centre
by 6.5 mm while the −6 returns differ by 0.7 mm — but the +6 returns also left the
elbow 1.06° away from the centre value, so the comparison is not at matched joints
and the difference is consistent with that. **Deterministic, not history-dependent,
not time-dependent.**

## Test C — is the FK-vs-image error constant or posture-dependent?

Using the measured joints of every capture in test B, predict `camera_from_board`
through the calibrated mount and the stored board, and compare with the
image-derived pose:

```
centre           dT median 16.24 mm   dR 11.38 deg   elbow 46.55
return from +6   dT median 17.39 mm   dR 11.18 deg   elbow 45.49
return from -6   dT median 16.14 mm   dR 11.43 deg   elbow 46.46
corr(elbow, dT) -0.98     corr(elbow, dR) 0.95
dT spread over the whole test 1.57 mm
```

So the discrepancy is dominated by a **constant 16 mm / 11.4°**, with only 1.3 mm
of variation across a 1.06° elbow change.

That constant is the size of the term the mount solve kept trying to take
(rotation delta 11.83°) and of the board yaw refit (+8.0°). A constant rotation
error is equally consistent with a wrong mount rotation and with a misplaced mat —
and this is exactly where Brian's rule bites: the mat is placed by hand each
session, so a constant must not be assumed to be hardware.

## What that means for the plan

- Chain bundle adjustment is **not** the next step; test A removes its main
  motivation (angular mapping) and test C shows the dominant term is constant.
- The remaining question is what the constant *belongs to*: camera mount rotation
  (11.8°) or board placement (yaw ~8-11°). One session cannot separate them.
- The separating experiment needs no new calibration and no new solver: **re-place
  the mat** (rotate it by a visible amount, e.g. 10-15°, or lift and put it back),
  capture a handful of views, and see whether the constant follows the board. If it
  follows, it is placement and the mount is fine; if it stays put, it is the mount
  and a one-time mount correction is justified — with evidence, not as a fit.

## Corrections to my own earlier work

1. **Mount scatter does not prove chain error.** The reviewer is right and my
   wide3 report should be read with that caveat: a stale board makes the inferred
   mount pose-dependent without any chain fault.
2. **`validate()` mixes models.** `view_mount()` solves PnP *with* distortion;
   `validate()` projects through a distortion-free pinhole. Measured at ≤0.2 px
   difference on these views (median 40.33 vs 40.46 px per-axis RMS), so no
   conclusion changes, but the two paths should be made consistent before more
   fitting.
3. **Normalisation.** Numbers in this report are per-axis RMS; `block5r_wide_solve`
   reports `sqrt(mean over 2N components)`, which is √2 larger. Earlier reports
   used the tool's convention. They are not interchangeable.
4. **Robust loss explains a reviewer query.** My z-only board fit returning
   38.94 px against 38.31 px at its start is `least_squares(loss="soft_l1",
   f_scale=2.0)`: the robust objective downweights outliers, so the plain RMS can
   rise. The start is not guaranteed to be worse under the *robust* cost.
5. **Calibration artifact identity.** The agent workspace copy of
   `.private/phase11d/calibration.json` is a *different* fixture
   (`sim-1.15.0-model.xml-462.4mm-2026-11-15`, fx 312.77, zero distortion). Every
   measurement above ran on the workstation copy,
   `sha256 70f6c51893d294fdf1512c896f5f7171694b06fce8c888e6c04b00a788b144f4`,
   `calibration_id handeye-2026-11-18T11-block4c`, intrinsics 831.897 / 833.014.
   This is exactly the trap the reviewer warned about; nothing here is affected,
   but the local copy must never be used for a run.

## State

Arm homed before and after test B; gripper not actuated (held at its measured
value, 79.6°); no calibration file written; `camera_in_tool` untouched.
