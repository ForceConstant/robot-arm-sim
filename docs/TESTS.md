# Full unit test — sim and IRL arm stay functionally equivalent

**Status:** proposed test plan (Brian, 2026-11-25). Numbers marked *baseline* are
measured values to check against, not new claims. This is a **test plan**, not a
green result — nothing here has been run as part of writing this document.

## Purpose

One regression test that answers a single question on every release:

> Does the simulator still behave like the real arm, and does the real arm still
> behave like the simulator — within the measured tolerance — after this change?

It exists because the two sides can drift independently while every isolated
check stays green. Two different forward-kinematics models, a stale board pose,
a moved camera mount or a reverted solver all look fine from one side. The test
therefore runs **paired** (identical targets, identical solver, identical
rig-state inputs on both sides), measures the gap, and then **stops for a human
gate**: numeric parity is necessary but not sufficient, because a render made
from the wrong gripper pack or tip constant looks like clean evidence while
being worthless (see `PLAN_2026-11-19_SIM_IRL_FINALIZE.md` §Visual review gates).

**Scope discipline:** this test is a gate, not a qualification. It does not
certify new capability, retrain anything, or expand the accepted envelope. It
detects *change* against a frozen baseline. A fail means stop, diagnose at the
earliest layer (§7), and report — never widen a threshold to make it pass.

## 0. Preconditions and invariants (non-negotiable)

1. **Execution host:** every simulation, render, sweep, solve or test runs on
   the BC-250 in CachyOS (`cachyos`, `brian@192.168.1.171`), never on the
   Unraid/Docker host — not even one frame. Local work is editing, `git`,
   `py_compile`/lint and reading only.
2. **Sync before test:** mirror the active working tree to the BC-250 and check
   its Git revision/status. Do not test an old clean clone while the source has
   uncommitted changes. Never mirror over generated data, `.private`, datasets,
   `dist` or virtualenvs. Record both heads in the receipt.
3. **Home before every task:** any step that moves the arm starts with
   `tools/home_arm.py` to the stored `recovery_joints` pose; confirm the tip is
   within ~15 mm of board centre first. Home does not actuate the gripper.
4. **Rig state verified:** `tools/rig_state.py verify` must match
   `rig_state/MANIFEST.json` before any motion. A drifted rig invalidates the
   whole run. Restore/verify per `docs/RIG_STATE.md`; re-snapshot (and commit)
   only after a deliberate recalibration.
5. **Same inputs both sides:** the board pose, `live_config.json`, wrist
   calibration and mount calibration used by sim and IRL are the *same four
   files* from `rig_state/`. A comparison across different rig snapshots is not
   a comparison.
6. **Branch preservation:** keep the current wrist-roll branch; stop on a 409
   or an unexpected branch change instead of improvising.
7. **Arm motion authorization:** IRL motion is permitted only under the
   standing rule (Raspberry Pi 4 bridge powered on and reachable). If the Pi is
   off or unreachable, motion is **not** authorized — report the blocker and
   stop. Print each leg before executing it and run the sim clearance check.

## 0b. Execution profile — measured 2026-11-25 (first pass, sim side)

Copy-paste setup so a run starts executing instead of rediscovering the
environment. The launcher default tool pack is the **pre-5y** pack; the pins and
T3/T4 require the **5y** pack, and the printed-piece tests need the printed
piece set. Getting this wrong looks like geometry drift (see below).

```bash
# on the BC-250, after the tree sync
export ROBOT_GRIPPER=cad_parallel
export ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y
# printed-piece tests only (test_dual knight profile, test_layout_search):
export ROBOT_PIECE_SET=printed
export ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight
```

| Step | Command (with the exports above where noted) | Expected on 805dec0a |
| --- | --- | --- |
| T2 | `robot-arm-sim test --tb=no --junitxml=/tmp/t2.xml` — **stock, do not export the gripper vars** | 274 collected: 221 passed, 11 failed, 42 skipped |
| T2b | `robot-arm-sim test tests/test_frame_pins.py tests/test_board_facing.py tests/test_aim_geometry.py tests/test_vision_geometry.py tests/test_product_twin.py tests/test_hand_eye.py tests/test_tool_orientation.py --junitxml=/tmp/t2b.xml` — 5y pack | 47: 44 passed, 3 documented fails |
| T3 | `robot-arm-sim python tools/phase1_parity.py --out reports/phase1_parity_<stamp>.json` — 5y pack | 5292 samples, worst Δ 0.000000 mm, rot 3.4e−6° |
| T4 | `robot-arm-sim python tools/cartesian_squares.py --backend sim --bridge-limits http://192.168.1.167:8100 --out reports/squares_sim_<stamp>.json` — 5y pack | 64/64 solved; p50 4.21 / p95 10.29 / max 12.83 mm |
| T4b | `robot-arm-sim python tools/phase2_squares.py --out reports/phase2_squares_<stamp>.json` — 5y pack | 20 mm gate mink 61/64; 3 mm gate 61/64 |
| T5 | `robot-arm-sim python tools/cartesian_squares.py --backend irl --bridge-limits http://192.168.1.167:8100 --out reports/squares_irl_<stamp>.json` — 5y pack, stream attached, arm homed | first run 58/59, stopped at c8 (409); see the T5 notes |

Notes that cost time on the first pass:

- **`pytest`'s totals line can be missing** from the log even when the exit code
  is 1. Read counts from `--junitxml`, not the tail.
- **Never run the pins with `ROBOT_GRIPPER=cad_parallel` but no pack override.**
  The launcher then uses `.private/gripper-assets` (pre-5y) and six
  tool-orientation checks fail with a systematic 10 mm tip / 8 mm jaw offset.
  That is the wrong-pack trap, not drift.
- **T3 does not run on the stock gripper** (`KeyError:
  so101:moving_jaw_so101_v1`). It needs `cad_parallel` + 5y; the error is a
  configuration requirement, not a finding.
- `live_config.json` keys are `ik_solver: mink` and `move_tolerance_mm: 3.0`
  (not `solver`). `TIP_IN_CHAIN` resolves from `live/tool_tip.json`, stamped
  for the 5y pack (0.11045 m, `ok: true`).
- `cartesian_squares --compare A B` labels A as the sim column and B as IRL.
  Comparing two sim reports works mechanically but is not a paired T6.
- **T4's 3.0 mm is a solver setting, not a pass gate in that tool.**
  `cartesian_squares.py` accepts a square on solve success under its 20 mm / 8°
  criteria, so residuals up to 12.83 mm are recorded as solved. If 3.0 mm must
  be enforced, that is a tool change and a decision — do not assume the count
  implies it.
- First-pass receipt: `reports/full_parity_20261125_firstpass.json` (sim side,
  result BLOCKED; T5/T8/T9 not run).

## 1. Test identity and artifacts

```
TEST ID       FULL-PARITY-01
Owner         robot-arm-sim release gate
Cadence       every release / any change touching kinematics, IK, solver,
              camera mount, board pose, tool frame, gripper pack or tip constant
Host          bc250-cachy (EGL); MuJoCo physics on CPU
Receipt       reports/full_parity_<stamp>.json  (+ paired heatmap + contact sheet)
Baseline head 51a403bba532139e8132cfe9f9c3551fb50555a1
Baseline date 2026-11-25 (board-delta correction + mink solver flip)
```

The receipt must record: both Git heads, dirty-file list, rig-state manifest
SHA-256s, the four resolved paths, solver name + tolerance, gripper kind/asset
root, `TIP_IN_CHAIN`, board pose x/y/z/yaw, camera intrinsics + mount source,
joint vectors and hover heights, seed, and tool command lines. **A render may
only be cited from the same commit it was made from**; re-running after a change
invalidates the old sheet — re-render, do not re-use.

## 2. Test steps

### T1 — Source-tree consistency (no motion, scripted)

- `git diff --check`; both trees at the same recorded head, or dirt documented.
- `tools/rig_state.py verify` — zero `DRIFT` lines.
- Assert one source of truth for the tool transform: no scattered identity maps
  / `jaw_frame` / `tool_tip_mm`; `TIP_IN_CHAIN` = `(0, 0, −0.11045)` m in the
  parity receipt, `live_config` solver `mink`, move tolerance ≤ 3.5 mm.
- **Pass:** all green. **Fail:** stop before any motion.

### T2 — Unit / existing suite (sim, no hardware)

- `scripts/ensure_venv.sh`, then `MUJOCO_GL=egl .venv/bin/python -m pytest -q tests`.
- Includes the algebra pins that must not drift: `tests/test_aim_geometry.py`
  (GL↔OpenCV flip, pose-invariant `TIP_PIXEL`, stale/grazing plane refusal),
  plus orientation, tool-frame, perception and product-twin tests.
- **Pass:** existing suite green, no loosened thresholds. **Fail:** fix before
  continuing; never relax a threshold to pass.

### T2b — Frame / rotation pins (the pain-point regression, §3b)

This is the explicit guard against reverting frame references, rotations,
board set-down, jaw placement, sim camera, or the sim↔IRL camera reference.
Each item below has a written-down source of truth **and** a test that fails
if anyone silently changes it. `tests/test_frame_pins.py` is the new
fast, hardware-free check that pins the literal values below; the named tests
check the built model/camera against them. A numeric pass here is necessary
but NOT sufficient — T8 still reviews the pictures.

- **Board frame — IRL:** `rig_state/board.json` = x −27.62, y −31.63,
  z 432.65, yaw **184.32°** (the board-delta correction). Assert these exact
  values, plus `frame_at()` convention in `live/kinematics.py`, and that the
  configured board still agrees with `/scan` as relative offsets (2 mm / 1°).
- **Board frame — sim:** `tests/test_board_facing.py` pins `ARM_PLAYS=black`, the
  `BOARD_TURN` set-down and square reading against the arm's position; this is
  the test that would have caught the half-turn-every-square regression.
  `tests/test_product_twin.py` pins board geometry (28 mm square, 3.2 mm slab,
  top 0.4337 m, arm base y −0.212 m). Do not let sim and IRL board turns
  diverge — compare them through the configured chain, never raw frames.
- **Jaw placement in sim:** `chess_sim/tool_frames.py` is the single written
  transform chain (`so101:gripper → cad:parallel_base → base part file frame →
  jaw part`). `tests/test_tool_orientation.py` measures the *built model*
  against it — fails on identity/90° jaw rotation, non-mirrored jaw pair,
  tip-depth drift, base off mount, moved gear. Pinned numbers: `MOUNT_POS_MM`
  (9.462, 23.429, −62.577), `MOUNT_QUAT` (0.707107, 0, 0, −0.707107),
  `TIP_DEPTH_MM` 110.450 ± 0.05, base bbox, jaw body/geom transforms.
  Requires `ROBOT_GRIPPER=cad_parallel` + the private 5y asset pack.
- **Tip constant:** `live/tip_frame.py` `TIP_IN_CHAIN` translation z =
  `−gripper_origin_to_tip_m` (resolved from the pack stamp), currently
  −0.11045 m. `tests/test_aim_geometry.py` pins its pose-invariance as a
  regression identity. (`docs/PLAN_2026-11-19_SIM_IRL_FINALIZE.md` still quotes
  a stale −0.104425 — flag, do not copy.)
- **Camera location in sim:** `tests/test_vision_geometry.py` round-trips the
  MuJoCo camera calibration pixel→board plane; `tests/test_hand_eye.py` perturbs
  and recovers the mount, proving the solver can re-fit it. Sim camera
  intrinsics source must be recorded in the receipt and match the mount used.
- **Sim↔IRL camera reference:** `chess_sim/tool_frames.py` mount values vs the
  hand-eye solved mount (`mount_calibration.json`) and `wrist_calibration.json`
  roll window. `tests/test_hand_eye.py` + the Block 5 corner-overlay protocol
  (valid views: median ≤ 2.0 px, max ≤ 4.0 px on sim corners drawn on real
  frames) are the cross-reference guard. A wrong board or a 180°/5 mm frame
  flip still produces a clean-looking scan — this is why T8 needs the picture.

- **Pass:** every pinned test green and every constant matches its written
  source of truth. **Fail:** stop before motion; restore the pin or explain the
  change explicitly with a new baseline and a deliberate re-snapshot — never
  let a test be edited to accept a reverted frame.
- **BC-250 tree state matters:** run pins against the *synced* head. On
  2026-11-25 the BC-250 sat at `0d33f8c`, 17 commits behind this source, with
  uncommitted IRL work (`live/app.py`, `solver.py`, `pi_bridge/*`). The compact
  `test_frame_pins.py` passed **5/5** there because its pinned constants are
  identical on both heads, and 27 companion tests
  (`test_board_facing`, `test_aim_geometry`, `test_vision_geometry`,
  `test_product_twin`) passed. Two other guards failed:
  `test_tool_orientation.py::test_base_extents_in_its_own_part_frame` (no
  matching base mesh in the built model) and
  `test_hand_eye.py::test_refinement_recovers_a_perturbed_mount` (detected
  marker 14 is not in `MarkerLayout`'s `_CENTRE_SIGNS`). The hand-eye code is
  identical on both trees, so the cause is not settled — do **not** log these
  as drift findings or waive them. They predate this test and need a separate
  look on a synced tree once the IRL work lands. `docs/TESTS.md` does not claim
  them green; T2b's pass is the 5 literal pins only.

### T3 — Model parity, sim-internal (scripted, no motion)

- `tools/phase1_parity.py`: FK of the live chain solver vs FK of the MuJoCo
  mink model, at the jaw tip, over the joint grid.
- **Baseline:** 5292 samples, tolerance 0.1 mm — two models of one arm.
- **Pass:** max disagreement ≤ 0.1 mm. **Fail:** the two sides are literally
  solving different arms; nothing downstream means anything.

### T4 — Sim-side functional sweep (sim, scripted)

- `tools/cartesian_squares.py --backend sim` (and `phase2_squares.py` through
  the real `MoveSolver` wiring) over the identical 64-square target list with
  the configured solver and `--bridge-limits` from the Pi (honest servo ranges).
- **Baseline:** 64/64 squares, solver gate 3.0 mm, mink. Note the documented
  ~19 mm FK-vs-image model bias is invisible inside FK and neither column can
  remove it — it is a known, recorded constant, not a new drift.
- **Pass:** square count and error distribution match baseline within §5 gates.

### T5 — IRL functional sweep (real arm, paired with T4)

- Home first (§0.3). `tools/cartesian_squares.py --backend irl` with the *same*
  target list, *same* solver, *same* rig state as T4; hold a `/stream` reader
  open (the detector only runs with a stream client attached).
- **Runner updated 2026-11-25 (commit c13dae4).** `run_irl()` now solves each
  target with the app's `MoveSolver` (mink), refuses locally on the move-
  tolerance / 8° approach gate or a joint-limit violation, walks the
  joint-space path against `GET /keep-in`, prints every leg before sending,
  verifies a `/stream` client is registered, and stops on the first HTTP
  refusal/error or wrist-roll branch change. `--dry-run` exercises the whole
  path without sending (63/64 on the first check; d8 refused at 10.2°).
- **Open decision — stop-on-409 vs the recorded baseline.** The baseline
  (61/64 with 3 refusals) came from a runner that continued after a refusal,
  while §0.6 says stop on a 409. The first guarded run stopped at c8:
  58/59 solved, 5 squares not attempted. Decide whether a 409 "no reachable
  pose" (arm did not move) is a recorded refusal that the sweep may continue
  past, or a stop. Do not change the runner's stop rule without that decision.
- **Pre-check is advisory.** On c8 the local pre-check predicted 0.74 mm /
  5.3° and the app refused at 1.83 mm / 12.284° from the same seed family.
  The app's gate remains authoritative; a local pass is not a landing promise.
- Cross-check with `tools/aim_squares.py` + `tools/aim_squares_eval.py` for the
  board-frame/pixel columns.
- **Baseline (2026-11-25, `reports/aim_squares_boardfix_20261125.json`):**
  64 squares; aim done/ok 60/60, tip done/ok 61/64 (3 refusals, 1 plane
  failure); aim err p50 7.44 mm / p95 11.60 mm; tip err p50 9.25 mm /
  p95 15.80 mm; board-frame p50 ~9 mm plus 3–4° servo deadband.
- **Pass:** within §5 gates; every refusal explained, none silently excluded.

### T6 — Paired drift comparison (scripted, the core assertion)

- `tools/cartesian_squares.py --compare reports/squares_sim.json reports/squares_irl.json`
  and `tools/aim_squares_compare.py <before.json> <after.json>` for release-over-
  release deltas. Same seed/targets/poses so position *i* is the same board in
  both — only disagreements carry information.
- Compare per square: achieved tip error (mm), aim error (mm), joint vector,
  marker projection (px), solver residual and reachability verdict; produce the
  paired heatmap and a fail list naming square + axis + magnitude.
- **Pass:** every §5 gate holds on the paired set. **Fail:** diagnose at §7 in
  order; do not call a physical failure a solver problem or vice versa.

### T7 — Rig-state re-verify after motion (no motion)

- `tools/rig_state.py verify` again — motion must not have shifted the four
  files. Home the arm and leave it as found. Any `DRIFT` = fail.

### T8 — HUMAN GATE 1: picture review of sim vs IRL (blocking)

**Do not commit or hand off until Brian approves.** Post a labelled contact
sheet with provenance printed on it (§1), one tile per required view:

> **Scope change (Brian, 2026-11-25): external IRL photographs of the arm are
> not required.** There is no external camera in the rig, so tiles 1 and 2 are
> sim-only and tile 3 carries the sim↔IRL comparison from the wrist camera.
> Do not re-add a photo requirement without a camera to produce it.

1. External orthogonal views — front, side, top — full arm, board visible,
   **sim render only** (no IRL photo).
2. Jaw/base seam close-up and fingertip plane with a **millimetre dimension
   callout**, **sim render only** (no IRL photo).
3. Wrist-camera view as the camera actually sees it, sim and IRL side by side.
4. Aim-ray overlay: simulated aim point projected onto the *real* frame, with
   per-corner residuals; detected-marker overlay from `/scan` with board pose
   before/after.

**Named checks the reviewer makes** (a "looks right" without these is not an
approval):

- **Jaw placement:** base sits under the wrist with no gap; both fingers
  symmetric and coplanar; fingertips at the pinned depth (110.450 mm measured,
  ±0.05); the jaw/base seam callout matches the millimetre number printed.
- **No reverted frame/rotation:** the sim jaw matches the tool-chain constants
  (T2b) and the sim wrist view agrees with the IRL wrist view on board coverage
  and orientation — explicitly check for a 90° or 180° rotation about the
  approach axis, a non-mirrored jaw pair, a half-turn board set-down (rank-1
  nearest the arm), and a sim-vs-IRL camera mount flip. Check the board is in the right place —
  a wrong board (or a 180°/5 mm frame flip) still produces a clean-looking
  scan, so the picture is the only check. Check for a silent gripper-pack swap
  (stock vs `cad_parallel` 5y) or a changed tip constant; the caption must name
  the resolved values, not memory.
- **In-scene aim laser (standard sim env):** `chess_sim/aim_laser.py` adds the
  red jaw-origin → tip → board ray and yellow impact marker as visual-only
  mocap geometry, on by default via `AppearanceConfig.aim_laser` (set False for
  clean perception renders). Because it is scene geometry, it appears in every
  camera render; verify the ray starts at the jaw and the yellow marker sits at
  the impact. Note: a chess piece may occlude the marker from some camera
  angles — that is real scene occlusion, not a laser failure.
- **Camera reference:** the sim camera view and the IRL wrist view show the same
  board coverage and orientation; simulated aim point projected onto the real
  frame lands where the overlay says; per-corner residuals match the reported
  pixel numbers (valid views median ≤ 2.0 px, max ≤ 4.0 px).

**Attribution rule:** every tile carries git head, gripper pack + asset root
(stock vs `cad_parallel`), `TIP_IN_CHAIN`, board pose (sim twin vs live board),
camera intrinsics + mount source (solved/assumed/placeholder), and joint pose +
hover height. If a render disagrees with the numbers, **the render wins** until
the discrepancy is explained. Record the approval (or the named failure) in the
receipt before committing.

### T9 — HUMAN GATE 2: end-to-end grasp/trajectory review (blocking)

One real pick-and-place (e.g. `tools/p4_grasp_capture.py --piece pawn --square
f4 --place-square f6 --aim square --execute --video ...`) plus a like-for-like
sim replay. Deliver a **GIF**, not an MP4 (WebChat does not render MP4 here):
convert on the BC-250 with `ffmpeg` palette/fps/scale and attach with
`MEDIA:<path>`. Verify numerically (`ffprobe`/`ffmpeg`: frame count, duration,
dimensions) — never `view_image` your own review artifacts.

**Allowed fail for the first pass (Brian, 2026-11-25).** The grasping feature
has not been through yet, so T9 is waived for this pass only: record the
attempts in the receipt and do not treat them as a new regression. Re-enable
T9 as a blocking gate once the grasp path is qualified. Every other gate is
unchanged.

Reviewer confirms: grasp succeeds, piece rests on target square, gripper empty,
no fault, trajectory shape matches sim, no unexpected branch change, and the
close/grasp moment is visible (stepped frames). Record verdict in the receipt.

## 3. Frozen baseline and gates (`docs/RIG_STATE.md`, 2026-11-25)

| Quantity | Baseline | Gate |
| --- | --- | --- |
| FK(chain) vs FK(mink) | 0.1 mm tol, 5292 samples | max ≤ 0.1 mm |
| Solver / gate | `mink`, 3.0 mm | unchanged |
| Board frame — IRL (`board.json`) | x −27.62, y −31.63, z 432.65, yaw 184.32° | exact values |
| Board set-down — sim | `ARM_PLAYS=black`, `BOARD_TURN` pinned | `test_board_facing.py` green |
| Jaw placement / tool chain | `tool_frames.py` mount pos (9.462, 23.429, −62.577), quat (0.707107, 0, 0, −0.707107); tip depth 110.450 ± 0.05 mm | `test_tool_orientation.py` green |
| Tip constant (`TIP_IN_CHAIN` z) | −0.11045 m (resolved from pack) | pose-invariant, value recorded |
| Sim camera ↔ IRL mount | hand-eye mount + wrist roll window; corner overlay median 1.252 px / max 3.382 px | `test_hand_eye.py` green, ≤ 2.0 / 4.0 px |
| IRL aim err | p50 7.44 / p95 11.60 mm | no regression beyond §5 |
| IRL tip err | p50 9.25 / p95 15.80 mm | no regression beyond §5 |
| Registration | 8/8 within 2 mm / 1° (Phase 9 envelope) | unchanged |
| Corner overlay (valid views) | median 1.252 px, max 3.382 px | median ≤ 2.0 px, max ≤ 4.0 px |
| `/sanity` corners | — | any corner > 6 mm stops the sweep |
| Move confirmations | 0 false confirmations | **must remain 0** |
| Torque/jog (baseline check) | 4° jog moves arm; 10 mm nudge accepted | both pass |
| Static joint drift | 0.00° over 180 s | unchanged |
| Grasp base drift | < 0.85 mm (4 mm abort) | unchanged |
| Rig state | matches `MANIFEST.json` | zero drift lines |

## 3b. Frame/rotation pain points — where each is caught

The regressions Brian called out, mapped to the exact guard. A silent revert
of any of these must fail a scripted pin (T2b) AND be visible to the human gate
(T8) — one without the other is not enough.

| Pain point | Source of truth | Scripted guard | Human-gate check (T8) |
| --- | --- | --- | --- |
| Board reference / rotation (IRL) | `rig_state/board.json` (yaw 184.32° + delta) | T2b value assert, `/scan` 2 mm/1° | board pose tile before/after |
| Board rotation / set-down (sim) | `BOARD_TURN`, `ARM_PLAYS=black` | `test_board_facing.py` | wrist view: rank-1 nearest arm |
| Jaw placement in sim | `chess_sim/tool_frames.py` chain | `test_tool_orientation.py` (90°/identity/mirror/depth) | jaw seam + mm callout, symmetric fingers |
| Camera location in sim | MuJoCo camera calibration + intrinsics | `test_vision_geometry.py` round-trip | wrist view coverage/framing |
| Sim ↔ IRL camera reference | hand-eye mount + wrist roll window | `test_hand_eye.py`, corner overlay ≤2/4 px | sim aim point on real frame, per-corner residuals |
| Tip constant / tool pack | `tip_frame.py` (−0.11045 m), gripper stamp | `test_aim_geometry.py` pose-invariance | caption names pack + resolved tip value |

**Revert rule:** a change to any of these values is not a test failure to be
waived — it is a deliberate re-baseline. It requires a written reason, an
updated source of truth, a re-rendered contact sheet at the new commit, and
Brian's approval recorded in the receipt. Editing a test or threshold to accept
an unexplained revert is itself a regression.

## 4. Known, accepted limitations (do not re-litigate in this test)

- Landing accuracy ~9 mm (frame residual) + 3–4° servo deadband; corrections
  below ~10 mm of tip travel are not reliably actuated.
- Fixed-focus lens calibrated near 160 mm; 60–70 mm hover understates working-
  distance accuracy.
- `~19 mm` FK-vs-image model bias is a stable constant, collinear with the board
  origin/hand-eye models; it is not new drift and is invisible to FK-only checks.
- The sim board-body frame is 180° + ~5 mm from the marker-layout frame;
  compare through the configured chain, never raw frames.
- Localisation target is nominal square centre (4.06 mm median on 46 real
  frames); sim-trained keypoint loses on real data — do not use it as gate truth.

## 4b. Known failing tests on `origin/main` (2026-11-25, pre-existing)

The full suite (`robot-arm-sim test`) is **not** green on a clean `origin/main`
checkout (`fae2b36`). These 11 failures were reproduced both on the updated
BC-250 and on a fresh detached worktree at `origin/main`, so they are **not**
caused by the BC-250 update, the mast removal or the aim-laser work. Do not log
them as drift findings from those changes, and do not relax a threshold to make
them pass. They need a separate owner and a recorded decision.

| Failing test | Status in this plan | Observed cause / note |
| --- | --- | --- |
| `test_hand_eye.py::test_refinement_recovers_a_perturbed_mount` | **Gated** in §T2b, §3, §3b (`test_hand_eye.py` green, corner overlay ≤2/4 px) | Already called out in §T2b: detected marker 14 is not in `MarkerLayout::_CENTRE_SIGNS`. Predates this plan; explicitly "do not log as drift" until a synced-tree look happens. |
| `test_frame_pins.py::test_rig_state_snapshot_matches_manifest` | **Gated** in §0.4, T1, §3, §3b (rig state matches `MANIFEST.json`, zero DRIFT) | One of the 5 literal pin tests; 4/5 pass. The **test literal** `MANIFEST_HASHES["live_config.json"]` expects `4c28d879…`, while `MANIFEST.json` and the committed snapshot are both `8c8303a1…` (verified 2026-11-25). The manifest is consistent; the test constant is stale and needs a deliberate update, not a re-snapshot. |
| `test_board_registration.py::test_registration_recovers_the_nominal_board` | Conceptual only: §3 "Registration 8/8 within 2 mm / 1°" (no filename) | Registration assertion fails on clean main. |
| `test_board_registration.py::test_registration_tracks_a_shifted_turned_board_and_random_camera` | Conceptual only: same §3 registration line | Shifted/turned board + random camera registration fails on clean main. |
| `test_marker_scan.py::test_plan_reports_missing_markers` | Not named; marker plans referenced conceptually in §3/§4 | Fails on clean main. |
| `test_marker_scan.py::test_plan_round_trip` | Not named | Fails on clean main. |
| `test_marker_scan.py::test_committed_plan_covers_every_marker_twice` | Not named (related to the §3 marker/registration concepts) | Fails on clean main. |
| `test_marker_scan.py::test_tolerant_plan_keeps_nominal_quality` | Not named | Fails on clean main. |
| `test_self_occlusion.py::test_projection_tracks_measured_joint_rotation_and_cache_is_not_mutable` | Not named; occlusion discussed in §T8/§4 only | `SelfOcclusionModel` constructed without the required `calibration` argument (constructor signature changed); 2 other tests in the file pass. |
| `test_self_occlusion.py::test_near_plane_clipping_and_behind_camera_rejection` | Not named | Same missing-`calibration` construction failure. |
| `test_self_occlusion.py::test_pixel_margin_is_conservative_and_invalid_joints_raise` | Not named | Same missing-`calibration` construction failure. |

**Reading this table:** T2 ("existing suite green") is the intent, not the
current state; §T2b already documents that it does not claim its two companion
guards green. A future run that turns any of these green is welcome, but must
come with a reason and a source-of-truth update — not an unexplained threshold
change. Full-suite result on `fae2b36`: **221 passed, 11 failed, 42 skipped**.
The T2b literal pins alone are **4/5** (the rig-state pin above is the fail).

### 4c. Status after the 2026-11-25 fix pass (`70bd237`, `bb6ef15`, `5761feb`)

Eleven failures were fixed, and the two survivors were root-caused:

- Nine stale expectations/assets: the `MANIFEST_HASHES` literal, both
  marker-scan plans (`marker_ids: [0,1,2,3]`), the two marker-scan tests, the
  three `SelfOcclusionModel` stub tests, and the shifted/turned registration
  seed (the prior pose is now always refined).
- **The registration square error was a stale generated face texture.** The
  cached `board_charuco.png` was written for a **15 mm** border while the board
  prints **10 mm**, so the field was stretched and the rendered squares were
  26.9 mm. `ensure_scene_assets` now regenerates the face when its size does not
  match the board; registration fits 28.16 mm again.
- **The hand-eye tool now reads the printed face** (`CharucoLayout`, marker
  plane `board.top`) instead of the 4-tag mat, and compares against the compiled
  model camera. The generated face lands **5.1 mm off the board body origin in
  y** (measured by fitting marker reprojection with a fixed true camera;
  `charuco_face.FIELD_OFFSET_MM`). With that offset the mount solves to
  **0.40 mm / 0.10°** at 8 views.
- The perception model was retrained on the corrected face (same protocol,
  seed 61): answered accuracy 0.975, coverage 0.718, 36 confident errors.

Current full suite (stock): **274 tests, 274 passed, 0 failed, 42 skipped.**
T2b companions (5y pack): **47 tests, 46 passed, 1 failed** — the remaining
failure is `test_tool_orientation::test_base_extents_in_its_own_part_frame`
(no matching base mesh in the built 5y model), which predates this plan.

## 5. Drift decision rule

A change is a **regression** only if, on the paired set:

1. any §3 hard gate fails (especially 0 false confirmations, rig drift,
   model-parity 0.1 mm, thresholds above), **or**
2. a paired metric moves outside the recorded baseline distribution by more
   than the measurement noise for that column (tip err p50/p95, aim err p50/p95,
   pixel residual, square pass count), **or**
3. a human gate (T8/T9) names a visual defect — the render wins over numbers.

Small moves within noise are normal; read the trend, not a single sample (a
u-jump in pixel space was an IK branch change, not drift). Any fail: stop, use
§7, re-run the full test; never partial-pass by skipping a gate or loosening a
threshold. Record the failure, owner and resume condition in the receipt.

## 6. Failure response

- **Sim-only fail** → solver/model/geometry regression; fix in sim, re-run T2–T4.
- **IRL-only fail** → physical (torque, deadband, mount, board pose, camera
  exposure/WB); check `verify_baseline.py` actuation first; re-home; re-run T5.
- **Both fail together** → shared input drift (rig state, board pose, config,
  calibration); `rig_state.py verify`, restore, re-run from T1.
- **Human gate fail** → picture is the finding; do not commit. Explain the
  discrepancy before any re-render or threshold change.

## 7. Diagnosis order (fixed — fail the earliest one that cannot hold)

1. Tool geometry / transform chain (T2 orientation tests, CAD cross-checks).
2. Intrinsics (camera calibration source).
3. Camera mount calibration (`mount_calibration.json`).
4. Board pose / board delta (`board.json`).
5. Solver choice and tolerance (`mink`, 3.0 mm).
6. Servo behaviour (torque on bridge open, P=32, deadband lead accumulation,
   branch changes).

Do **not** re-fit a mount to close a pixel residual that came from a misplaced
finger. Do not widen a gate to absorb a real change.

## 8. Exit criterion

`FULL-PARITY-01` passes only when T1–T2b–T7 are green, both human gates T8/T9 are
explicitly approved by Brian and recorded in the receipt, the paired drift
comparison is inside §5, the arm is homed and rig state still verifies, and the
receipt + heatmap + contact sheet + GIF are committed at the same head they were
produced from. Anything less is a documented blocker, not a pass.
