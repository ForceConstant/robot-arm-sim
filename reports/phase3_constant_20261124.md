# Phase 3 — killing the ~19 mm constant (2026-11-24)

**Outcome in one line:** a five-roll wrist sweep made the two candidate errors
separable, and the answer is that **the stored camera-in-tool orientation was
wrong by 23.4°** — a mount error, not a board error, and far larger than the
18.7 mm translation it was hiding behind. Correcting the mount in
`calibration.json` is applied and deployed. The FK-vs-image mean falls
**20.38 → 15.75 mm** on the calibration views (held-out 17.5 mm), and the tip
projection falls from 128 px to 33 px of Brian's measured tip pixel. The
**≤ 5 mm target is NOT met**: the residual is a board-origin offset that the
one-place rule deliberately left untouched, and its depth component is only
weakly identifiable at this geometry.

Nothing committed. All sim/arm calculations ran on the BC-250.

---

## 1. Sanity — has the board moved?

Two independent read-only checks, both before any motion:

| check | today | 2026-11-21 | delta |
|---|---|---|---|
| board pose from markers, roll ≈ 68 | — | (-61.3, -45.4, 425.2) yaw 169.3 | — |
| board pose from markers, roll 87.7 (today) | (-45.6, -58.9, 439.4) yaw 163.5 | — | 25.0 mm vs the roll-68 set |
| board pose vs `live/board.json` (-34.9, -18.1, 431.4, 181.9) | 42.9 mm | — | — |
| marker 14 pixel, same joint pose replayed | (260.5, 256.7) | (252.6, 284.2) | (7.5, -32.0) px |
| marker 14 edge length, same pose | 91.4 px | 90.9 px | +0.5 px |

The edge length is unchanged (standoff unchanged) but the marker sits 32 px
away from where the same joint pose put it three days ago. That is either a
board shift (~6 mm) or a camera-mount change (~2.2°). It could not be resolved
from a single pose, which is exactly what the roll sweep was for — and the
sweep says the mount: the implied mount *rotation* is 21.3–22.6° off in every
roll group, a constant tool-frame rotation that no board pose can produce.
Board depth is within a few millimetres of the stored value (§5).

Verdict: **the board has not moved in a way that poisons the fit.** The 32 px
shift is the mount error. Board translation is addressed in §7.

## 2. Capture plan (written before executing)

`reports/phase3_plan_20261124.json` (all candidates) and
`reports/phase3_plan_selected.json` (executed):

* rolls **0, +40, −40, +68, +87** — 127° of wrist-roll spread, where every
  previous capture ever taken sat at ≈67°;
* 16 board targets across both halves (ranks 2–7, both files), roll pinned and
  the other four joints solved so the lens sits over the target at the
  historical 160 mm standoff;
* 71 of 77 candidates solved inside the bridge's own joint limits with ≥1.5°
  margin; then 8 per roll chosen by greedy farthest-point spread;
* predicted 2–11 markers per view; tip clearance ≥ 38 mm above the board and
  inside keep-in (±112 mm) at every selected pose.

Milestone map: `reports/phase3_plan.png`.

## 3. Capture (IRL)

`live/captures_phase3/`, receipt `reports/phase3_capture_20261124.json`:

* **41/41 legs reached**, roll held within 0.3° of target, joint drift 0.00° at
  every frame; one replay of the 2026-11-21 roll-68 pose first (the sanity leg);
* one settle per pose (`ScanController._goto`: command → band → hold → residual
  correction), two frames per pose with the better detection kept;
* **tone locked for the whole session and recorded in every file**: exposure
  2500, gain 96, brightness 160, contrast 48, white balance 4600 K, auto
  exposure off, auto WB off. (705/gain 64 with auto off was underexposed —
  frame mean 62 and markers failed to decode; 2500/96 restores the historical
  sharpness: marker-crop Laplacian 472 vs 525 on the 2026-11-21 reference.)
* **63 marker sightings over 37 views**, 21 views with ≥2 tags, max 4. Three
  chess pieces sit on the board (no gripper actuation authorised), and the
  soft focus of the fixed-focus lens at this standoff keeps counts low.
* no gripper actuation; the jaw was held at its measured value every leg.

Milestone card: `reports/phase3_captures.png`.

## 4. Fit — three models, and why the reprojection fit is not the referee

34 views (new sweep + the 2026-11-21 captures), multi-marker PnP, stored lens.

| model | parameters | reproj RMS | FK-vs-image mean | Jacobian cond |
|---|---|---|---|---|
| baseline | stored mount, stored board | 143.7 px | 20.38 mm | — |
| A | board fixed, mount 6-DoF | 46.8 px | 58.96 mm | 1777 |
| B | mount fixed, board 6-DoF | 72.9 px | 82.84 mm | 1486 |
| C | both 6-DoF | 16.2 px | 30.81 mm | 3256 |

The reprojection fits are badly conditioned (cond 1.5–3.3 k in deg/mm units)
and their **camera positions are worse than baseline while their reprojection
is better** — the classic weak-perspective trade of rotation against
translation when the target subtends a limited field. They are reported for
completeness but they are not the referee.

The referee is the per-view absolute estimator: every view with ≥2 tags gives a
complete camera pose, hence one implied mount and one implied board pose. No
global compromise, no hidden pose-dependent error.

## 5. Identifiability result

Alternating rigid solve (the documented staged method) over 34 views:

| parameter | solved | delta vs stored |
|---|---|---|
| mount rotation | — | **23.4°** (per-roll 21.3–22.6° before convergence) |
| mount translation | (63.59, 11.10, −13.15) mm | (−4.1, +14.0, −2.3) mm |
| board translation | (−27.6, −31.7, 432.6) mm | (+7.3, −13.5, +1.3) mm |
| board yaw | −174.3° | +2.4° |

Residual over the 34 views: **20.38 → 8.56 mm** (p50 20.03 → 7.80).

Identifiability, stated plainly:

1. **The rotation is fully identified.** A board error would have to rotate with
   the tool; this offset does not. It is constant to ±0.7° over 127° of roll
   (21.3–22.6° measured per group; 23.4° after the alternating solve converges).
2. **The translation split is only partly identified.** Both blocks are real —
   leave-one-roll-out gives mount-only 17.50 mm, board-only 17.45 mm, both
   9.78 mm, so neither single model predicts an unseen roll. The translation
   design `[R_tool | −I]` has condition number **15.2** (each block alone is
   cond 1.0, i.e. trivially fittable and therefore untestable), which is
   acceptable for *separation* but the split has a shallow valley along the
   optical axis: starting the alternating solve with the board shifted ±20–40 mm
   moves the answer between mount z −10.9…−17.6 mm and board z 434.7…428.5 mm
   while the residual stays 8.33–8.68 mm.
3. **What spread would settle the split:** more *standoff* variation, not more
   roll. The depth direction is constrained by target distance; with the arm
   limited to ~160 mm (the reach limit recorded in the 2026-11-21 review) the
   remaining depth freedom is ~6 mm. A second standoff (or a stereo/known-size
   cue) is required for the last few millimetres.

Milestone card: `reports/phase3_fit.png`.

## 6. Tip constant — stock vs cad_parallel, and T2's −6.7 mm

**Answer: 110.450 mm is right for the installed tool; the stock 104.425 mm is a
different part; and T2's −6.7 mm was never the tip constant.**

Evidence, in order of strength:

1. **Code path.** The tip constant enters only `live/tip_frame.py::TIP_IN_CHAIN`,
   used for tip targets and the tip report. Camera forward kinematics is
   `kinematics.camera_pose = fk() @ camera_in_tool` — it never reads
   `TIP_IN_CHAIN`. A tip-length error therefore **cannot** produce a camera
   observation, so a fitted camera-mount z offset cannot be the tip constant.
   They happen to be along the same axis, which is why they looked related.
2. **Mesh profile** (`reports/phase3_tip.json`): the installed pack's jaw
   vertices run from 45.95 to 110.450 mm in the `so101:gripper` frame with a
   continuous taper — ~2 vertices per millimetre from 95 to 110 mm, and the
   deepest 8 vertices sit at exactly 110.450 mm at radial 7.85–8.52 mm. That is
   a slender blade, not a stray spike, and there is no 6 mm step anywhere. The
   pack's own profile cross-check (`|tcp_z| + tip_reach`) agrees at 110.450 with
   a 0.0 mm delta.
3. **Independent projection test.** Projecting the jaw tip through the *old*
   mount lands at (367, 369) px; through the *new* mount at (313, 218) px.
   Brian's measured tip pixel is (320, 250). The old error was 128 px — the
   magnitude T2 saw and could not explain — and it was the mount, not the tool
   length. The new 33 px residual (~6 mm) is the same board/mount valley as §5.

So: keep `cad_parallel` / 110.450 mm. If the stock fingers are ever fitted,
`tip_frame.py` refuses to run `cad_parallel` with a stock stamp, which is the
right failure.

## 7. What was applied, and where

**One place: `calibration.json`** (`live_config.json → calibration` →
`.private/phase11d/calibration.json`). `camera_in_tool` replaced with the solved
mount (translation 63.59, 11.10, −13.15 mm; rotation 23.4° from the stored one).
Backup: `.private/phase11d/calibration.json.bak-phase3-20261124T154154`.
`robot-live.service` restarted (user unit) and reloads it — `run, active`.
Nothing in code was special-cased; no other file carries the correction.

**Not applied, on purpose:** the board delta (+7.3, −13.5, +1.3) mm, +2.4° yaw.
The task allows exactly one place, and the board's depth is degenerate with the
mount z (§5). It is recorded here as the next, separately-authorised step — it
is worth roughly the difference between 15.75 mm and 8.6 mm.

### Before / after (identical images, only the mount differs)

| set | before mean | after mean | before p50 | after p50 |
|---|---|---|---|---|
| calibration views (34, ~160 mm) | 20.38 | **15.75** | 20.03 | 14.05 |
| 64-square IRL, quality views (34, 60 mm) | 24.81 | **19.87** | 23.01 | 16.53 |

64-square IRL: `reports/phase3_squares_irl.json` — **64/64 moves executed**
through `/move` (mink, 3.0 mm gate), 51 squares produced at least one marker,
34 passed the quality cut (≥2 markers, PnP RMS ≤ 4 px).
Before/after evaluation: `reports/phase3_squares_eval.json`.
Chart: `reports/phase3_beforeafter.png`.

64-square sim: **not re-run.** `live/solver.py` and `live/mink_arm.py` never
read `camera_in_tool` (verified by grep: only `live/kinematics.py` report/overlay
paths do), so the solver residual is provably unchanged by this correction; the
committed `reports/phase2_squares.json` (mink 64/64, p50 0.34 mm) still stands.

## 8. What did NOT improve

* **The ≤ 5 mm target.** 15.75 mm after the mount fix, 8.56 mm with the joint
  solution — neither reaches 5 mm. The board origin and the depth valley are the
  remaining terms.
* **The 64-square sweep at a 60 mm hover.** It reads ~20–24 mm both before and
  after, worse than the 160 mm views. The lens is fixed-focus for ~160 mm; at
  60 mm the corners are soft and single/two-marker PnP is biased. Treat that
  sweep as a *reach/execution* check (64/64 executed), not as a calibration
  metric.
* **Marker yield.** 63 sightings over 37 views is thin; three pieces on the
  board and the soft lens are the cause. More tags per view would tighten every
  number in §5 by a factor of ~2.
* **`calibration_id`** still says `handeye-2026-11-18T11-block4c` even though the
  mount changed; left alone because plan caches key on it.

## 9. Files

New tools (uncommitted): `tools/phase3_sanity.py`, `phase3_plan.py`,
`phase3_select.py`, `phase3_capture.py`, `phase3_fit.py`, `phase3_linear.py`,
`phase3_perview.py`, `phase3_mount.py`, `phase3_pair.py`, `phase3_tip.py`,
`phase3_apply.py`, `phase3_squares_irl.py`, `phase3_squares_eval.py`,
`phase3_card.py`.

Receipts: `reports/phase3_sanity.json`, `phase3_plan_20261124.json`,
`phase3_plan_selected.json`, `phase3_capture_20261124.json`, `phase3_fit.json`,
`phase3_fit_hist.json`, `phase3_linear.json`, `phase3_perview.json`,
`phase3_mount.json`, `phase3_pair.json`, `phase3_tip.json`,
`phase3_squares_irl.json`, `phase3_squares_eval.json`.

Data: `live/captures_phase3/` (41 poses), `live/captures_squares_phase3/`
(64 frames, 51 with markers). The aborted first pass is kept as
`live/captures_phase3_trial1/` (underexposed; not used).
