# Subagent brief — pinning as a test step, then finish T5 and T6 (2026-11-27)

Read this whole file, then `docs/TESTS.md` §0 (rig state), §T4, §T5, §T6, §T7 and
§5 (gates). Do the work in the order below. Do NOT commit or push; the parent
reviews and commits.

## Where to work

- Working tree: `/home/node/.openclaw/workspace/robot-arm-sim` (this container).
- All simulation, tests and arm execution happen on the BC-250, never here.
  `rsync` is not installed here; copy changed files one at a time with
  `scp -q <file> cachyos:/home/brian/robot-arm-sim/<same path>`.
- The workstation repo is at commit `a3e4f03`. Read-only git there is fine.
- Runner: `~/.local/bin/robot-arm-sim python <script>` or `... test`.
- Environment for project runs:

```
ROBOT_GRIPPER=cad_parallel
ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y
ROBOT_PIECE_SET=printed
ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight
```

- Three test failures are PRE-EXISTING, leave them:
  `tests/test_dual.py::test_legacy_fixture_both_arms_physically_lift_and_place`,
  `tests/test_wrist_scan_runner.py::test_runner_registers_and_labels_a_sim_board`,
  and any `test_frame_pins` failure that the re-pin step below is supposed to fix.

## Background facts you must not rediscover the hard way

- `camera_in_tool` (`handeye-2026-11-18T11-block4c`, sha256 `70f6c518…`) is a
  **hardware constant**. Never re-fit it. Never edit it.
- The board pose in `live/board.json` is a **per-session measurement**. It is
  currently stale: the mat heading measures −165.50° yaw (2.4° spread) against the
  stored 184.32°, i.e. ~10.2° of rotation.
- `live/wrist_calibration.json` was accepted at 2026-11-26T09:27 and re-pinned
  today (`8ccb4961…`, commit `9e5bbc5`). Do not touch it.
- Recent PnP and projection work lives in `chess_sim/projection.py`
  (`pnp_tool_frame`, `project_tool_frame`, `per_axis_rms`, `per_component_rms`).
  Use it; do not write a private copy.
- Board registration captures already exist on the workstation at
  `live/captures_boardreg_20261127/` (18 views, 143 sightings) and the tool is
  `tools/calibrate_board_charuco.py --layout charuco`.
- `tests/test_frame_pins.py` holds `MANIFEST_HASHES` and `BOARD_EXPECTED` as
  literals; `rig_state/MANIFEST.json` holds the same hashes.

## Task 1 — make re-pinning a step of the unit test, not an ad-hoc edit

Brian's instruction: pinning should be "redone as a step in running unit test".

1. Add `tools/repin_rig_state.py`. It must:
   - take `--why "<sentence>"` (required, no default) and refuse to run without it;
   - copy the named live file(s) into `rig_state/`, recompute sha256 + bytes into
     `rig_state/MANIFEST.json` (preserving existing formatting), and rewrite the
     matching literals in `tests/test_frame_pins.py`;
   - accept a list of files, defaulting to `--file board.json` only, and refuse any
     name that is not one of the four pinned files;
   - append one line to `rig_state/PIN_LOG.md` (create if absent) with UTC time,
     the file, old and new hash prefixes, and the `--why` text;
   - print exactly what changed and exit non-zero if `--why` is blank.
   Keep it under ~120 lines, no new dependencies.
2. Add a short **additive** subsection to `docs/TESTS.md` right after §0b
   (frame/rotation pins) titled `§0c — Rig-state pin as a test step`, saying: the
   test run starts by verifying pins; a legitimate change (accepted wrist zero,
   new session board registration) is re-pinned with `tools/repin_rig_state.py
   --why "<reason>"`; a re-pin without a reason is a failure; the pin exists to
   catch *unexplained* drift, not to freeze per-session data.
3. Add `tests/test_repin_rig_state.py`: unit-test the tool against a temporary
   copy of `rig_state/` and `tests/` (do not touch the real ones) — asserts it
   refuses without `--why`, updates both places consistently, and appends to the
   log.

**Open design question — do not decide it yourself.** `board.json` is per-session
data but sits in the pinned set. Implement the re-pin tool so that either choice
works, and state in your report which policy you implemented by default (pin
board each session, or exclude it) and why. The parent will confirm with Brian.

## Task 2 — register this session's board and re-pin

1. Re-run `tools/calibrate_board_charuco.py --layout charuco --captures
   live/captures_boardreg_20261127 --out reports/board_charuco_20261127.json`.
2. If the mat has visibly moved since those captures (compare the new median yaw
   with −165.50°), capture a fresh set first using the same poses as
   `live/captures_boardreg_20261127` (8 wrist-scan postures + a 12-pose grid; the
   scripts are not in git — write a small capture script following
   `tools/block5q_wide_cal.py` conventions: home first, `/stream` client open,
   print every leg, stop on HTTP 409, hold the measured gripper value).
3. Write `live/board.json` from the median pose (`x_mm`, `y_mm`, `z_mm`,
   `yaw_deg`) and re-pin it with the tool from Task 1, `--why` naming this
   session's board registration.
4. Run `tests/test_frame_pins.py`; it must pass.

## Task 3 — T4 sim baseline

Run T4 exactly as `docs/TESTS.md` describes: `tools/cartesian_squares.py
--backend sim` over the 64-square list with the configured solver and
`--bridge-limits` from the Pi. Save the JSON where T6 expects it
(`reports/squares_sim.json`). Do not change solver, target list, gates or
tolerances.

## Task 4 — T5 IRL sweep

Run T5 as documented: home first, `/stream` reader open, same target list, same
solver, same rig state as T4. Print every leg. **Do not change the runner's
stop rule** — `docs/TESTS.md` records an open decision about stop-on-409 and says
explicitly not to change it without Brian. If it stops at a 409, record exactly
where, leave the arm homed, and report it.

- Gripper: hold the measured value throughout. Do not actuate, open or close it.
- If the bridge is not REMOTE and armed, stop and report rather than recovering.
- Save the IRL JSON where T6 expects it (`reports/squares_irl.json`).

## Task 5 — T6 paired comparison

Run the paired comparison from `docs/TESTS.md` T6
(`tools/cartesian_squares.py --compare reports/squares_sim.json
reports/squares_irl.json`) plus the aim columns if the tools are present. Produce
the paired report, the fail list (square + axis + magnitude) and, if the tool
supports it, the heatmap PNG. Do not invent a pass: state the actual gate result.

## Task 6 — T7 re-verify

Run `tools/rig_state.py verify` after the motion and report pass/DRIFT. Home the
arm and leave it as found.

## Hard constraints

- Do NOT edit `camera_in_tool`, the calibration file, `live/wrist_calibration.json`,
  or any solver/gate/tolerance value to make something pass.
- Do NOT actuate the gripper.
- Do NOT commit, push or branch.
- Keep the arm inside previously exercised postures; stop on any unexpected
  branch change, bridge fault or HTTP 409 where the runner says stop.
- If a step cannot be completed, stop and report — do not substitute a proxy.

## Report back

1. Files created/changed, one line each.
2. Which pinning policy you implemented and why.
3. Board registration numbers (pose, spreads) and the re-pin evidence.
4. T4: counts and gate result.
5. T5: counts, refusals with reasons, where it stopped (if it did).
6. T6: paired deltas, gate result, fail list.
7. T7: PASS or DRIFT.
8. Anything you could not do, and where you disagreed with this brief.
