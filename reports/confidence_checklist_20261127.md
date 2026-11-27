# Confidence checklist — code, setup and the unit test (2026-11-27)

Brian: this is a verification exercise to complete a unit test, and the board will
normally go back to near zero. So this lists what is now trustworthy, what is not,
and what remains — with evidence for each line.

## Verified now

**The claim the whole exercise was about.** The mount is a hardware constant; the
board pose is a per-session measurement. That is now proven twice:

- by hardware: with the mat rotated, the stored-vs-true rotation went 11.38° →
  21.6° while the mat turned 13.07°, i.e. the discrepancy followed the board
  (`reports/rot_scan_20261127.md`);
- by algebra, in `tests/test_board_offset_invariance.py`: with the true board every
  view recovers the *same* mount to 1e-9; with a stale board the recovered mount
  is pose-dependent even though nothing moved. A 10° stale yaw scatters it by
  ~10 mm and shifts the mean by ~13 mm — which is why an averaged candidate looks
  plausible and is still wrong.

**The measurement code.** `tools/block5r_wide_solve.py` had two silent defects:

- `view_mount()` solved PnP *with* distortion while `validate()` projected
  without it. Both now cross one `project_tool_frame()` that applies the
  calibrated lens, and points behind the camera return nan instead of scoring.
- `SOLVEPNP_IPPE` returns two solutions for a planar target and the first was
  taken blindly — that produced a 32° outlier on a 10-marker view
  (`reports/rot_scan_20261127.md`). Both branches are now refined and the lower
  residual wins.

Re-solving wide3 with the corrected code: delta 16.5 mm / 11.82° (was 16.1 /
11.83), held-out 28.91 px (was 33.61) against 57.03 for the current mount. Same
conclusion, clean measurement.

**New tests, all passing.** 13 synthetic, no hardware:
`tests/test_block5r_solve.py` (projection convention, behind-camera rejection,
mount recovery to 0.2 mm / 0.05°, branch selection, lens actually applied) and
`tests/test_board_offset_invariance.py` (5 cases).

**Suite:** 279 collected, **254 passed, 22 skipped, 3 failed**, with
`ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y
ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight`.

## The three failures, attributed

1. **`test_frame_pins::test_rig_state_snapshot_matches_manifest`** — real drift, and
   the test is doing its job:
   `live/wrist_calibration.json` canonical = `8ccb4961…`, pinned = `021bc97f…`.
   Diff: `model_offset_deg -0.09 → 0.0`, `servo.homing_offset 630 → 509`,
   `updated 2026-11-19T15:30 → 2026-11-26T09:27`. That is yesterday's accepted
   wrist zero (`/wrist/zero`, "accept current as wrist 0"). A deliberate change
   that was never re-pinned. **Needs a deliberate re-pin — Brian's call.**
2. **`test_dual::test_legacy_fixture_both_arms_physically_lift_and_place`** — fails
   identically at `HEAD~1` (checked in a detached worktree): pre-existing sim
   controller abort, not caused by this work.
3. **`test_wrist_scan_runner::test_runner_registers_and_labels_a_sim_board`** — fails
   identically at `HEAD~1`: pre-existing.

## What remains — code

| # | Item | Acceptance check |
|---|---|---|
| a | Other planar-PnP call sites still take the first IPPE branch (`tools/p4_*`, `b5b_*`, `block4c_mount`, `solve_wrist_live`) | one shared helper; a test that the recovered pose beats the first branch |
| b | RMS conventions: this tool reports per-component (√2 larger than per-axis); reports mix them | one named helper, and `docs/TESTS.md` states which |
| c | Robust loss (`soft_l1`) vs acceptance metrics: a plain RMS can rise as the robust cost falls | acceptance numbers all use one objective |
| d | `wrist_flex` chain limit 95° while home is 98.0° (bridge accepts 98.79) | chain limit matches the bridge; a high-flex pose dry-runs |
| e | Render `fovy 47.0°` vs measured 32.14° (sim fidelity only, not bench) | rendered marker size matches at the corrected pose + FOV |

## What remains — setup

| # | Item | Acceptance check |
|---|---|---|
| f | The agent workspace has no bench calibration now, by design, after quarantining the sim fixture | no test or CI path depends on `.private/phase11d/calibration.json` existing locally |
| g | Nothing stops a `sim-*` calibration being loaded again | `WristCalibration.load` refuses a `sim-` id unless an explicit override says otherwise |
| h | `live/board.json` is stale by ~21° / 14 mm right now; the per-session registration is a manual step | a session-start check that the board has been registered this session |
| i | BC-250 dual-boot rotates SSH host keys on OS switch | use the `bazzite`/`cachyos` aliases only (already in the `start-cachyos` skill) |

## What remains — the unit test itself

| # | Item | Acceptance check |
|---|---|---|
| j | Board-rotation invariance is proven once by hand, and now by algebra; it is not automated on hardware | a scripted two-session check: capture at 0°, rotate the mat, capture again → mount unchanged within X mm, board yaw tracks the rotation |
| k | Nothing pins `camera_in_tool` against silent rewriting | a test asserting the calibration file hash is unchanged by any run |
| l | Acceptance metric for the mount solve is not written down | a stated threshold (e.g. held-out per-axis RMS and board scatter) in `docs/TESTS.md` |
| m | Three failures above | suite green, then the test is trustworthy |

## Shortest path

1. Decide the wrist-calibration re-pin (item 1). Everything else in the rig-state
   pin is already consistent.
2. Fix the two pre-existing failures (items 2, 3) — they are unrelated to this work
   but they are the reason the suite is not green.
3. Consolidate the PnP call sites on the corrected helper (a). That is the same
   class of bug that produced the 32° outlier, and it is still live elsewhere.
4. Then re-run the board-rotation check as a session test (j) — with the mat back
   near zero, it should show a mount unchanged to a few mm and a board pose that
   tracks wherever the mat is put.
