# 2026-11-18 — IRL Cartesian control and board calibration

**Owner/planner:** librarian session (this session)
**Workers:** other agents, block by block
**Status legend:** `[ ]` pending · `[~]` in progress · `[x]` done · `[!]` blocked

## Starting state (measured, end of 2026-11-17)

- Sim 64-square sweep: **63/64**
- IRL: **63/64** achievable
- Sim/IRL agreement: **59/64**
- Remaining failures: near-edge squares; last blocker was a tip-vs-tool frame
  inconsistency at `a1`
- Saved board pose: x **−13.74 mm**, y **−3.57 mm**, z **409.5 mm**,
  yaw **7.14°**
- Board plane was fitted from markers; the correction moved z by **11.7 mm**
  and was verified to **0.1 mm** at three postures
- `tool_tip_mm` = **83**, live `jaw_frame` z = **−104.827**, app default
  `jaw_frame` z = **−173.027** — three competing tip conventions

### Current working-tree state

`live/` is **untracked**. These files carry uncommitted tip-space edits made
before this plan was agreed; they have **not been run or tested**:

- `live/kinematics.py` — optional `tip_offset` on `solve`/`solve_multi`,
  new `tip_target()` and `solve_tip()` with the app's seed×span search
- `live/app.py` — xyz/rel moves, `/sanity`, `/home` and `/keep-in` now use
  tip-space
- `tools/cartesian_squares.py` — sim/IRL sweep reports tip error and calls
  `solve_tip`

**Decision 2026-11-18 07:01: KEEP.** The tip-space edits are the working basis
for Block 1; review and complete them, do not revert.

## Decisions (settled 2026-11-18 07:01)

- [x] **BC-250 is in CachyOS.** Verified: `PRETTY_NAME="CachyOS"`, kernel
  `7.3.4-1.2-bc250-test`, host `cachyos-x8664`, `systemd` running.
- [x] **Staged tip-space changes: keep.** They are the basis for Block 1.
- [x] **Acceptance gate: 20 mm** for today. The 35 mm in the live config is a
  measurement-loosened value and must be restored.

### Sim host state at 07:01

- Project: `/home/brian/robot-arm-sim`
- Git: `02c9a4e`, working tree dirty with the same project edits as the
  planner checkout (plus the live `live/` files on this partition)
- `live/live_config.json`, `live/board.json`, `live/bridge_token.txt` are
  present here and are the real bench values

### Handoff notes for workers

- The tip-space edits exist **only in the planner checkout** right now
  (`/home/node/.openclaw/workspace/robot-arm-sim`). The BC-250 copy at
  `/home/brian/robot-arm-sim` is still at the pre-edit state for those files.
- Copy only these three files to the BC-250:
  `live/kinematics.py`, `live/app.py`, `tools/cartesian_squares.py`.
- **Do not mirror `live/`.** The BC-250 owns the real `live/live_config.json`
  (no `jaw_frame`, no `tool_tip_mm`, tolerance 20), `board.json`, `poses.json`,
  `bridge_token.txt` and `captures/`. Overwriting them loses the bench setup.
- `live/` is untracked, so `git` will not carry it. Sync explicitly.
- The planner checkout was reconciled to the verified Block 1 files at 07:36
  (`live/kinematics.py`, `live/app.py`, `live/tip_frame.py`,
  `live/tests/test_tip_convention.py`, `tools/cartesian_squares.py`,
  `tools/audit_tip_frame.py`, `docs/TIP_FRAME_CONVENTION.md`,
  `reports/tip_frame_*.json`).

## Repo state (2026-11-18 07:50)

- Block 1, Phase 11d live console, ChArUco calibration and tip-frame work are
  committed on `main`: **`78c460c`**.
- Working tree is clean. Ignored on purpose: `live/live_config.json`,
  `live/board.json`, `live/bridge_token.txt`, `live/captures*/`,
  `sim/captures_wrist/`, `live/*.bak*`.
- The stray older `live/test_tip_convention.py` was moved to
  `/tmp/test_tip_convention.stray.py` on the BC-250, not deleted.
- Not pushed to `origin` yet.

## Block 1 — Settle the tip-frame convention (sim, 60–75 min)

**Owner session:** `robot arm 11/18 block1`
(`agent:librarian:dashboard:0bb50d86-f0ae-4e50-9005-452980a1d192`, running)

Goal: one tip convention, one solver path, proven by a regression.

- [x] Audit the three tip offsets (`83`, `−104.827`, `−173.027`) against the
  MJCF: 8 poses, deepest solid fingertip **104.425 mm** in 7 of 8; the open
  moving-finger corner reaches 106.132 mm
- [x] Choose one convention: `TIP_IN_CHAIN` on the wrist axis,
  z = −104.425 mm, +z up / approach −z; `docs/TIP_FRAME_CONVENTION.md`
- [x] Staged tip-space change: kept, completed, config overrides removed
- [x] Regression `live/tests/test_tip_convention.py`: a1 then h8 mid-sequence
- [x] Re-run affected tests

**Exit gate: PASSED (independently re-run by planner at 07:35).**

- `a1`: solver 3.90 mm, guard tip 3.88 mm, Δ **0.025 mm**
- `h8`: solver 20.80 mm, guard tip 20.80 mm, Δ **0.003 mm**
- `1 passed in 83.53s`
- Live calibration chain vs MJCF `so101:gripper` body: **0.0000 mm /
  0.0000°** at all 8 poses (independent check, not printed by the audit tool)
- `live_config.json` contains no `jaw_frame` and no `tool_tip_mm`; app DEFAULTS
  likewise; `move_tolerance_mm` is already **20.0**

**Hygiene:** the BC-250 has a stray older copy at `live/test_tip_convention.py`
(4.1 kB, differs from `live/tests/test_tip_convention.py`). It is collected by
`pytest live` but not by `pytest live/tests`. Remove or quarantine it before
Block 2; the planner did not delete it.

**Carry-forward for Block 2:** h8 is **20.8 mm** at hover 60, 0.8 mm outside
the 20 mm gate. Brian's symmetry point (2026-11-18) is the first test: with a
symmetric wrist, **a1 ≡ h1 and a8 ≡ h8**. The measured board pose is not exactly
symmetric (base sits ~13 mm off board x=0 and the board is yawed 7.14°), so
allow a few millimetres — but a 20 mm split between mirrored squares is a model
asymmetry, not reach. Investigate the tip-frame rotation sign, joint limits and
board yaw before calling h8 a reach limit. Do not move the tip again to hide it.
The sweep will report 63/64 unless a real improvement is found; that is a valid
stop-and-report under Block 2's gate.

## Block 2 — Sim sweep to 64/64 honestly (45–60 min)

**Owner session:** `robot arm 11/18 block2`
(`agent:librarian:dashboard:2dfc88e3-8039-4e71-8f13-114a6aa9fc1f`, done)

- [x] Run `tools/cartesian_squares.py --backend sim` with the production
  solver (sim and IRL must execute identical maths)
- [x] Record per square: target, IK error, achieved tip, marker count, starts
- [x] **Symmetry check first:** solve a1/h1 and a8/h8 from a neutral seed.
  Equal within the measured board offset/yaw is expected; a ~20 mm split means
  a model asymmetry (tip-frame rotation sign, joint limits, board yaw), not
  reach. Record the joint vectors for the mirrored pair.
- [x] Iterate only on failing squares via dry-run, never a full sweep per tweak
- [x] Confirm `move_tolerance_mm` is 20 (Block 1 already set it)
- [x] Save the receipt under `reports/`

**Result (2026-11-18): 64/64 solved, worst 14.9 mm (f8), worst approach
2.59°, all tips inside keep-in; reverse-order sweep also 64/64 with only 0.1 mm
rounding differences.** h8 was **not** reach and **not** a model asymmetry: the
mirror split was 2.30 mm and a `TIP_IN_CHAIN` sign flip changed nothing
identical to 3 decimals, while a bounded solve from the extended arm branch
returns 12.20 mm inside the real wrist limits. The residual was a seed-basin
miss — `solve_tip` only seeded the folded arm family. Added the extended-branch
seeds (`[pan, 58, −64, 90, ...]`); h8 → 12.30 mm mid-sequence, guard 12.32 mm.
Receipt: `reports/block2_2026-11-18.md`. Carry-forward for Block 5: a8 solves at
wrist_roll 160.8° (2.0° from the limit) — confirm the bridge's roll range before
the IRL sweep.

**Exit gate:** 64/64 solved, achieved tip ≤20 mm, approach ≤8°, every square
inside keep-in, no order-dependent failures. `63/64` with a named physical
cause is a valid stop-and-report, not a pass. **— PASSED.**

### Planner verification (2026-11-18 08:55)

- `reports/squares_sim_2026-11-18.json`: **64/64**, worst **14.9 mm (f8)**,
  zero failures.
- Regression re-run independently: a1 **3.90 / 3.88** mm (Δ0.025),
  h8 **12.30 / 12.32** mm (Δ0.025), `1 passed in 151.74s`.
- Tree clean at `238080a`; gate still **20 mm**; no `jaw_frame` or
  `tool_tip_mm`.
- The extended-branch seed fix is a legitimate search change: it does not alter
  the tip constant, the gate or the acceptance criteria.

## Block 3 — Prove the calibration pipeline in sim (45 min)

- [x] Inject known board placements (±8 mm x/y, ±2.5° yaw, plus a z offset)
  and run the multi-pose `/scan` path
- [x] Measure recovered x/y/z/yaw error per placement
- [x] Compare estimators: `/scan` fusion vs `/board/derive` single-marker vs
  offline median tool
- [x] Repeat the board-plane fit from markers; confirm z correction stable to
  ≤0.2 mm across poses
- [x] Name the estimator that becomes the bench procedure

**Exit gate:** calibration errors quantified per estimator; multi-pose path
inside 2 mm / 1° in sim; one procedure named.

**Result (2026-11-18): PASS.** Ten physical placements (centre, ±8 mm x/y,
±2.5° yaw, +3.0 mm z, two corners) recovered by the multi-pose `/scan` path to
**worst 0.22 mm XY / 0.048° yaw** (median 0.17/0.19 mm, 0.036°). Single-marker
PnP: x median 0.34 mm but **2.23 mm worst**; **z biased −3.1 mm** (worst
−9.4 mm). Offline median: x/y ≤0.37 mm, yaw ≤0.06°, z biased −3.4 mm.
Board-plane z from the same captures: shared four-parameter fit bias
**+0.042 mm**, spread **0.159 mm** across the matrix; per-view spread median
0.102 mm / worst 0.416 mm (a two-tag view). **Named procedure: multi-pose
ChArUco fusion for x/y/yaw + a multi-marker planar z fit over the same
captures** — `/scan` itself does not estimate z. Planes settled: `/scan`
returns the board origin on the supplied marker plane, single-marker PnP and
the offline median return the tag plane (ChArUco → playing surface; mat →
0.5 mm above table); `board.json` z is the playing surface. Receipt:
`reports/block3_2026-11-18.md`.

**Block 5 action from Block 3:** live `marker_z_m` is 0.43056 (mat plane) while
`layout_kind` is `charuco`; with that plane every placement fails the fusion's
1.2 px gate (RMS 1.33–2.56 px). Set it to `board_z_m` 0.4337. The four
scan-controller accuracy tests still load `.private/phase11d/calibration.json`
(real hand-eye mount, fx 312.77) against the sim's 36° camera (fx 738.64); the
sim proof uses the model-exported calibration. `/board/derive` and the offline
tool were composing the PnP transform inverted and without the MuJoCo axis
flip — fixed; the ChArUco face texture was rendering a 19.906 mm tag for a
20 mm model — fixed at 224 px/square.

### Planner verification (2026-11-18 09:15)

- Re-ran the **centre** placement with the documented env (CAD gripper assets,
  piece assets, `PYTHONPATH=.`, `MUJOCO_GL=egl`): scan **ex 0.175, ey −0.185,
  eyaw 0.0385°**, shared plane bias **+0.047 mm** — matches the receipt.
- `reports/block3_matrix.json`: 10 placements; scan max **0.190 / 0.219 mm,
  0.048°**; shared plane spread **0.159 mm**.
- `live_config.json` still has `marker_z_m = 0.43056` (mat plane). This is a
  Block 4 preflight, not changed by Block 3.
- Test status confirmed: `live/tests/test_scan_controller.py` **4 passed /
  4 failed** (the file loads the real hand-eye calibration, so the sim camera
  sees 0 markers); the 11 ChArUco test failures in the three older files remain.

## Block 4 — IRL run sheet and dry-run harness (30 min)

- [ ] **Preflight — ChArUco marker plane:** set `live/live_config.json`
  `marker_z_m = 0.4337` (equal to `board_z_m`). The current **0.43056** is the
  mat plane; with `layout_kind=charuco` a valid scan fails the fusion RMS gate
  (1.33–2.56 px vs 1.2). Mandatory before any `/scan`.
- [ ] **Preflight — test fixture calibration:** `live/tests/test_scan_controller.py`
  loads `.private/phase11d/calibration.json`, which is now the real hand-eye
  mount; in sim, 4 accuracy tests fail with 0 markers. Point the fixture at the
  sim-exported calibration (or add a sim fixture) before trusting the suite.
- [ ] **Preflight — ChArUco test assumptions:** 11 failures in
  `tests/test_marker_scan.py`, `test_hand_eye.py`, `test_wrist_scan.py` assume
  the old 0–4 mat marker set. Update or xfail with a reason; do not use the
  suite as a gate while it is red for the wrong reason.
- [ ] One command to dry-run the 8 known-problem squares with `dry_run` only
- [ ] Distinguish in output: keep-in refusal vs IK failure vs bridge fault
- [ ] Write the bench sequence as a numbered run sheet:
  Pi up → stop LeLab/teleop before the follower port → REMOTE + arm →
  `/board/assume` seed → `/scan` → compare `board.json` → `/sanity` 5 points →
  8-square dry run → full sweep
- [ ] Define abort conditions and who calls them

**Exit gate:** run sheet reviewed; no step requires improvisation at the bench.

## Block 5 — IRL after power-up (2–3 h)

- [ ] Boot Pi and arm; verify port ownership before any motion
- [ ] `/scan` board calibration; compare to saved pose. Large z/yaw change
  means stop and re-fit, not fudge.
- [ ] **`/scan` is x/y/yaw only** — run the multi-marker plane fit for z. Do not
  treat the scan offset as a full 6-DoF pose.
- [ ] **Frame semantics:** `board.json` x/y are absolute in the arm frame
  (base link at y = −217); `/scan` offsets are base-relative. Compare relative
  offsets, never absolute z against the twin's 433.7.
- [ ] **Re-check live intrinsics/mount before trusting `/board/derive`:**
  saved `fx = 312.77` vs the 36° lens comment (~746). If inconsistent, rerun
  hand-eye; single-tag z is biased −3.1 mm median (worst −9.4).
- [ ] `/sanity` corners + centre; any corner over 6 mm stops the sweep
- [ ] 8-square dry run
- [ ] Full 64-square sweep with the same solver and targets as sim
- [ ] Compare sim vs IRL square by square

**Exit gate:** 64/64 agreement with sim at the 20 mm gate; zero guard refusals
misread as unreachability. A disagreement is a hardware/calibration finding —
record it, do not retune the solver to hide it.

### Known carry-forwards from Blocks 2–3

- **a8 is joint-limit-adjacent:** wrist_roll **160.8°** (2.0° from the 162.79°
  limit) and wrist_flex 1.6° from its limit. Check the Pi bridge's actual roll
  range before the IRL sweep; a narrower servo range would make a8 the first
  sim/IRL disagreement.
- **13 squares see zero wrist-camera markers**, including a8 (its roll branch
  flips the camera). Do not expect marker fusion there in Block 3.
- **c8–f8 at 13.5–14.9 mm** are fast-path accepts (the ≤15 mm fast path skips
  escalation and leaves them folded). Harmless at the 20 mm gate; do not treat
  them as solver failures.
- **Test runner gotcha:** `robot-arm-sim test <path>` ignores the path and runs
  the whole suite. Use `robot-arm-sim python -m pytest <path>` for a single
  file. The full suite has 11 pre-existing failures in `tests/test_marker_scan.py`,
  `test_hand_eye.py` and `test_wrist_scan.py` (`unknown marker id 5`, from the
  ChArUco layout change) — unrelated to Block 2.


## Rules for the day

- No full 64-square sweep to discover a defect; dry-run the known failures.
- No loosening a gate to make a number pass.
- No calibration constant changed without a measurement that predicts it.
- Three failed attempts on one defect means stop and report the precise
  inconsistency, not a fourth theory.
