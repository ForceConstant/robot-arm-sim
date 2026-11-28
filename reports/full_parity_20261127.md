# FULL-PARITY-01 — receipt refresh, 2026-11-27 (second run)

**Test:** FULL-PARITY-01 per `docs/TESTS.md` §0, §0c, T4–T7, §5.
**Result:** T4 PASS (count and distribution unchanged), T5 PASS (62/64, both refusals
expected and explained), T6 agreement 64/64 with no §5 hard-gate failure observed,
T7 PASS. The aim cross-check's millimetre columns are inside the previous band; its
marker-PnP pixel column moved and contains two corrupted two-tag board solves
(`a3`, `a4`) — flagged for diagnosis, not waived. T8/T9 human gates were **not**
re-run in this pass (see *Not run*).

## 1. Where and what

| item | value |
| --- | --- |
| host | `cachyos-x8664` / CachyOS, kernel `7.3.4-1.2-bc250-test` (BC-250) |
| tree | `/home/brian/robot-arm-sim` |
| `git log --oneline -1` at run time | `c260e86 fiducials: one detection per marker id - fixes the hand-eye regression` |
| tracked changes at run time | none (`git status --porcelain` shows only the ~449 untracked scratch entries in `live/`, `tools/`, `hardware/`) |
| working tree change made to get there | one tracked file was stashed to allow the fast-forward: `chess_sim/fiducials.py` carried an uncommitted copy of the very change that `c260e86` lands. `git stash push -m "parity rerun 20261127: local fiducials one-per-id" -- chess_sim/fiducials.py`, then `git pull --ff-only` (`872f9a4..c260e86`). The stashed blob is byte-identical to `HEAD:chess_sim/fiducials.py` (`git diff stash@{0}:chess_sim/fiducials.py HEAD:chess_sim/fiducials.py` = 0 lines), so it was left in the stash list rather than popped — popping would have conflicted on content already in HEAD. |
| workspace mirror | this container, same repo, `c260e86` |

Environment on every run (unchanged from the plan, §0b):

```
ROBOT_GRIPPER=cad_parallel
ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y
ROBOT_PIECE_SET=printed
ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight
```

### Rig state

`tools/rig_state.py verify` before motion:

```
  ok        live/board.json
  ok        live/live_config.json
  ok        live/wrist_calibration.json
  ok        .private/phase11d/calibration.json
rig state: MATCHES the manifest
```

- `board.json`: x −11.50, y −37.72, z 423.77, yaw −165.50 — the re-pinned session
  registration of 2026-11-27 (`PIN_LOG.md` 2026-11-27T15:31:35Z, 18 views, median
  yaw −165.50°, `434db96e8d…`). Same board the brief quotes.
- `live_config.json`: `ik_solver mink`, `move_tolerance_mm 3.0`,
  `gripper cad_parallel`, `gripper_assets .private/gripper-assets-5y` (`8c8303a1…`).
- No calibration, board, wrist-calibration or `rig_state/` file was edited by this
  run. Nothing under `.private/` was touched except reading the pinned mount
  calibration.

### Arm state before the first motion

`tools/home_arm.py --reason "FULL-PARITY-01 T5 start 2026-11-27"`:

```
  before: tip (18.4, -22.6, 120.4) mm | 29.1 mm from centre, z 120.4
  after:  tip (18.4, -22.6, 120.4) mm | 29.1 mm from centre, z 120.4
  legs: 1 | last error 1.49 deg
```

The joints were already at the stored `recovery_joints` pose
(`pan −12.57, lift −12.88, elbow 6.29, flex 98.07, roll 0.00` vs stored
−12.37 / −13.02 / 4.80 / 98.34 / 0.00), so `/recover` executed one 1.49° leg and
did not move the arm. **The tip reads 29.1 mm from board centre in board
coordinates, which is outside the ~15 mm the AGENTS.md invariant asks for.** That
offset is the consequence of the 2026-11-27 board re-registration (the same joints
read 14.0 mm from centre against the previous board pose in the 2026-11-25
receipt); it is board-frame geometry, not a different arm pose. Reported, not
adjusted — see *Disagreements / notes* 3. No gripper actuation: the bridge gripper
value stayed at its measured 50.38 % throughout.

## 2. T4 — sim-side functional sweep (`tools/phase2_squares.py`)

Command (BC-250, env above):

```
~/.local/bin/robot-arm-sim python tools/phase2_squares.py \
    --out reports/phase2_squares_20261127b.json
```

Gates untouched: 20 mm and 3.0 mm position, 8° approach, `MoveSolver`/mink, the
same 64-square list at 60 mm hover. Tail:

```
20mm gate: LS 64/64  mink 62/64  regressions 2
 3mm gate: LS 15/64  mink 62/64  regressions 0
median ms LS 1811.5  mink 1.4   max ms LS 125666.9  mink 265.7
approach edge squares: []
```

| metric | this run | baseline (`reports/phase2_squares_20261127.json`, c7c9c88) | Δ |
| --- | --- | --- | --- |
| mink solved @ 20 mm / 8° | 62/64 | 62/64 | 0 |
| mink solved @ 3.0 mm / 8° | 62/64 | 62/64 | 0 |
| least-squares @ 20 mm | 64/64 | 64/64 | 0 |
| least-squares @ 3.0 mm | 15/64 | 15/64 | 0 |
| mink residual median / max (mm) | 0.48 / 1.50 | 0.48 / 1.50 | 0 |
| achieved tip error p50 / p95 / max (mm) | 0.46 / 1.09 / 1.43 | 0.46 / 1.10 / 1.43 | ≤ 0.01 |
| per-square achieved tip delta vs baseline | 0.00 mm on all 62 solved squares | — | 0 |
| wrist-view marker count, sum over 62 solved squares | 353 | 261 | +92 |
| squares with 0 markers | 1 (`d8`) | 4 (`a8`, `d8`, `g8`, `h8`) | −3 |

**Failures (both gates, unchanged):**

| square | residual | approach | reason |
| --- | --- | --- | --- |
| e8 | 1.49 mm | 15.55° | approach 15.55° > 8° gate |
| f8 | 1.32 mm | 14.02° | approach 14.02° > 8° gate |

Both are the same two squares and the same two angles as the baseline, to the
third decimal. Position residual is 1.49 / 1.32 mm — inside 3.0 mm; the refusal
is entirely the approach-angle gate, which is what the IRL column also refuses
(e8 10.9°, f8 9.1°).

**Marker column.** The count rose and three of the four zero-marker squares
became visible. This is *not* drift and not a regression: the committed baseline
report was produced at `c7c9c88` (2026-11-27 11:59), which is an ancestor of the
appearance commits in this batch — the comparison-FOV widening (`07509b1`), the
wrist-roll zero (`24bb955`) and above all `872f9a4` (aim laser off by default).
The aim laser is modelled geometry in the wrist render and hides marker faces;
with it off, `a8`, `g8` and `h8` go from 0 to 1/1/2 markers. `d8` stays at 0 in
both receipts. One detection per id (`c260e86`) can only remove duplicates, it
cannot create markers, so it cannot explain the rise either.

## 3. T5 — IRL functional sweep (`tools/cartesian_squares.py --backend irl`)

Command (BC-250, env above, arm homed, same 64 targets and the same MoveSolver):

```
~/.local/bin/robot-arm-sim python -u tools/cartesian_squares.py --backend irl \
    --bridge-limits http://192.168.1.167:8100 \
    --out reports/squares_irl_20261127b.json
```

- Stop rule unchanged (no `--continue-on-refusal`): local pre-check refusals are
  recorded and the sweep continues, an HTTP refusal/error stops it, a wrist-roll
  branch change stops it. The open §T5 decision on stop-on-409 was **not** touched.
- `/stream` held for the whole sweep: `stream: attached http://192.168.1.171:8080/stream (detector active)`.
- Every leg printed before sending (target, solve mm/deg, roll, tightest joint
  margin); the per-leg keep-in clearance walk ran as part of the runner.
- No gripper actuation. The bridge gripper stayed at 50.38 %.

Tail:

```
leg 61/64 e8: … solve 0.83 mm / 10.9 deg | roll +81.5 | tightest wrist_flex +0.0 deg
  REFUSED before send: approach 10.9 deg > 8 deg
leg 62/64 f8: … solve 0.64 mm / 9.1 deg | roll +0.0 | tightest wrist_flex +0.0 deg
  REFUSED before send: approach 9.1 deg > 8 deg
leg 63/64 g8: … done: settle 0.28 deg | tip [71.4, 98.3, 59.6]
leg 64/64 h8: … done: settle 0.31 deg | tip [97.5, 98.2, 59.4]
irl: 62/64 solved -> reports/squares_irl_20261127b.json
  FAIL e8: pre-check: approach 10.9 deg > 8 deg
  FAIL f8: pre-check: approach 9.1 deg > 8 deg
```

| metric | this run | baseline (`reports/squares_irl.json`, 2026-11-27 11:09) |
| --- | --- | --- |
| attempted / solved | 64 / 62 | 64 / 62 |
| stopped early | no (`stopped: null`) | no |
| HTTP refusals / 409 / 5xx | 0 | 0 |
| local pre-check refusals | 2 — e8 (approach 10.9°), f8 (9.1°) | 2 — e8 (10.9°), f8 (9.1°) |
| achieved tip err p50 / p95 / max (mm) | 4.83 / 6.80 / 9.11 | 4.82 / 6.32 / 9.21 |
| settle p50 / p95 / max (°) | 1.09 / 1.58 / 2.00 | 1.11 / 1.56 / 1.91 |
| roll delta (deg) | 0.0 on every executed leg | 0.0 |
| marker / aim columns from `/move` | absent (0/62 rows) — the app still returns no `square`/`aim_pixel`/`offset_px` | absent |

The two refusals are the same two squares T4 fails, for the same reason, at the
same angles in the same order. Both are refusals *before* any motion: nothing was
sent, no 409, no branch change. e8's solver plan has roll +81.5° (a different
branch for that target); it was never executed and the arm's roll stayed at 0.0.

## 4. T6 — paired drift comparison

`tools/cartesian_squares.py --compare` needs `solved`-shaped rows, and T4's
receipt is the `phase2_squares` shape. To keep one solver and one gate on both
sides (the reason `cartesian_squares --backend sim` is not quoted as T4, §T4), the
T4 receipt was converted verbatim: `solved = mink.pass3` (3.0 mm / 8°),
`ik_mm`/`ik_deg` = the MoveSolver residual. No gate recomputed, nothing relaxed.
That file is `/tmp/sim_from_t4.json` (scratch, not committed).

```
~/.local/bin/robot-arm-sim python tools/cartesian_squares.py \
    --compare /tmp/sim_from_t4.json reports/squares_irl_20261127b.json
…
agreement 64/64
```

| paired quantity | value |
| --- | --- |
| agreement (same solve/refuse verdict) | **64/64** |
| solved on both sides | 62 |
| refused on both sides | 2 (e8, f8) |
| sim achieved tip err p50 / p95 / max (mm) | 0.46 / 1.09 / 1.43 |
| IRL achieved tip err p50 / p95 / max (mm) | 4.83 / 6.80 / 9.11 |
| mean IRL − sim bias (mm) | x +1.39, y +2.32, z −2.87 |
| mean absolute IRL − sim distance (mm) | 4.21 |
| fail list (square + axis + magnitude) | **empty** — no square disagrees in verdict, and no paired metric leaves the recorded distribution |

§5 verdict: hard gates that this pass could observe all hold — agreement 64/64,
0 false confirmations (no leg was reported confirmed while refused or vice versa),
IRL tip error p50 4.83 mm / p95 6.80 mm against a recorded baseline of 9.25 / 15.80
(no regression), settle inside its previous band, roll branch unchanged, rig state
MATCHES after motion (T7). **T3 model parity (0.1 mm) was not re-run in this pass**
— it is outside this brief and is the one §3 gate this receipt cannot assert.

Aim columns: see §5 below.

## 5. Aim columns (`tools/aim_squares.py`)

Command (BC-250, env above, arm homed, `/stream` via the tool itself):

```
~/.local/bin/robot-arm-sim python -u tools/aim_squares.py \
    --base http://127.0.0.1:8080 --out reports/aim_squares_20261127b.json
```
Session 18:29:47 → 19:07 (38 min, 64 squares, z 70 mm, tolerance 4 mm).
Baseline for the deltas: `reports/aim_squares_20261127.json`, run 10:47–11:05 the
same day — **before** `07509b1` (comparison FOV 32.1445 → 35.6995°), `24bb955`
(wrist-roll zero) and `872f9a4` (aim laser off), so it is not a like-for-like
baseline for a pixel column.

| metric | this run | 10:47 run | note |
| --- | --- | --- | --- |
| aim legs done / ok | 57 / 57 | 59 / 59 | 2 fewer: `a7`, `c8` had no usable board plane |
| tip legs done / ok | 64 / 60 | 64 / 60 | same 4 refusals: d8, e8, f8, g8 |
| plane failures | 3 (`a7`, `a8`, `c8`) + `a3` stale mid-refine | 1 (`a8`) | |
| aim err p50 / p95 / max (mm) | 3.74 / 6.84 / 7.11 | 3.14 / 6.75 / 7.20 | p50 +0.60 mm, tail unchanged |
| tip err p50 / p95 / max (mm) | 4.69 / 5.81 / 7.87 | 4.55 / 5.80 / 6.02 | p50 +0.14, max +1.85 |
| aim px (marker PnP) p50 / p95 / max | 118.05 / 177.23 / **1045.72** | 97.95 / 157.02 / 190.29 | moved — see below |
| tip px (marker PnP) p50 / p95 / max | 116.57 / 165.13 / **611.27** | 95.82 / 144.95 / 167.03 | moved — see below |
| aim px_cfg p50 / p95 / max | 48.42 / 85.35 / 95.50 | 42.83 / 81.80 / 92.58 | +5.6 mm p50, cfg column |
| tip px_cfg p50 / p95 / max | 50.50 / 64.25 / 69.30 | 47.75 / 59.72 / 63.29 | +2.8 px p50 |
| readout spread p50 / p95 (mm) | 0.0 / 0.1 | 0.0 / 0.1 | unchanged |

**The mm columns are inside the previous band** (aim p95/max identical, p50 +0.6 mm
— under one servo deadband step; tip p95 identical, max +1.85 mm). **The marker-PnP
pixel column is not, and it has a concrete cause on the board-solve side:**

| leg | board solve rms | square projected to | value |
| --- | --- | --- | --- |
| a3 tip | 1597.8 px (2 tags) | u,v = [271.1, −337.8] (off-image) | 611.3 px |
| a4 aim | 942.2 px (2 tags) | u,v = [670.1, 1202.2] (off-image) | 1045.7 px |
| b3 aim / b2 aim / d5 / d6 / e6 | 141.3 / 133.5 / 76.8 / 52.1 / 51.7 px | on-image | 87 / 68 / … |

Every other leg solved its board at 0.7–4 px. In the 10:47 run the worst board rms
for the whole sweep was 4.70 px (median 1.61). A two-tag solve that places the
board so the square projects off-image is a wrong association, not arm error — the
aim and tip *mm* numbers on those same legs (a3 tip 4.94 mm, a4 aim 4.88 mm) are
normal. The PnP column therefore measures a detection/plane-diagnosis defect on
2–7 legs, not drift of the arm or of the solver.

Verdict on this column: **moved outside the recorded distribution; flagged for
diagnosis, not waived and not tuned.** §7 order puts it at step 1–2 (transform
chain / intrinsics association), not at the solver. I did not investigate further
or touch the detector, the plane policy, `--refresh-z-mm` or the FOV — that is a
decision for Brian, and the brief forbids tuning to make a number come back.

## 6. T7 — rig-state re-verify after motion

```
  ok        live/board.json
  ok        live/live_config.json
  ok        live/wrist_calibration.json
  ok        .private/phase11d/calibration.json
rig state: MATCHES the manifest
```

**T7 PASS** — no `DRIFT` line after 64 square moves plus the 64-square aim sweep.
Arm left as found:

```
  before: tip (95.6, 99.4, 67.1) mm | 137.9 mm from centre, z 67.1
  after:  tip (18.7, -22.6, 120.4) mm | 29.3 mm from centre, z 120.4
  legs: 2 | last error 1.49 deg
```

Joints after homing `pan −12.66, lift −12.88, elbow 6.29, flex 98.07, roll 0.09` —
the stored `recovery_joints` pose, roll branch preserved. Gripper 50.38 % → 50.29 %
(read noise; never commanded).

## 7. Not run

- **T1/T2/T2b/T3** — out of scope for this brief (source pins, stock suite, frame
  pins, model parity). Not asserted here.
- **T8/T9 human gates** — not re-run; T8 needs Brian's approval of a contact sheet
  at `c260e86`, T9 needs gripper actuation, which is a specific ask.

## 8. Notes and disagreements with the brief

1. The brief's T4/T5 baseline (62/64, achieved p50 0.46/1.10/1.43, refusals
   e8 10.9° / f8 9.1°) reproduces exactly. The baseline *report file*
   `reports/phase2_squares_20261127.json` is older than its numbers suggest: it was
   committed at `c7c9c88`, before the FOV / roll / aim-laser commits, so its marker
   column is not comparable. Only the marker column differs; every solver and
   achieved-tip number is identical.
2. `tools/cartesian_squares.py --compare` cannot read a `phase2_squares` receipt
   directly. Rather than run the legacy least-squares sim column (explicitly not
   parity-comparable, §T4), the T4 rows were re-shaped. This is a format adapter,
   not a gate change — it is reported because it is code the plan does not name.
3. **Home is 29.1 mm from board centre in board coordinates**, above the ~15 mm the
   AGENTS.md invariant asks to confirm. The joints are exactly `recovery_joints`;
   the offset is the new session board pose (the same joints read 14.0 mm against
   the 2026-11-25 board). The brief says not to touch `board.json`, so the value is
   recorded and left alone. Consider re-deriving `recovery_joints` for the new
   board, or raising `CENTRE_TOLERANCE_MM` deliberately — a decision, not a fix.
4. `tools/home_arm.py` exits non-zero in this state (29.1 mm > 20 mm tolerance) even
   though `/recover` succeeded and the arm was already home. Its exit code is
   therefore not a usable pass signal against the current board.
5. `/move` returns an empty `joints_deg` object, so the T6 "joint vector" column is
   unavailable on the IRL side — only the sim side has joint vectors. Either the app
   stops echoing joints or the runner reads the wrong key; that is a tool/app gap, not
   a measurement.
6. The T6 `--compare` agreement number is a verdict comparison only. Paired tip-error
   and bias numbers were computed on the achieved columns by a scratch script
   (`/tmp/t6_pair.py`, not committed) because `--compare` prints verdicts, not deltas.
