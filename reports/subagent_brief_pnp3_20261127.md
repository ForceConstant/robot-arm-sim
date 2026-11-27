# Subagent brief — three verification items (2026-11-27)

Read this whole file, then do the work. Do NOT commit or push; the parent reviews
and commits.

## Where to work

- Working tree: `/home/node/.openclaw/workspace/robot-arm-sim` (this container).
- Simulations and tests MUST run on the BC-250, never here. `rsync` is not
  installed here; copy each changed file with
  `scp -q <file> cachyos:/home/brian/robot-arm-sim/<same path>` and run there.
- The workstation repo is at commit `6827b5f`. Read-only git there is fine.
- Test command:

```
ssh -o BatchMode=yes cachyos 'cd /home/brian/robot-arm-sim && env \
  ROBOT_GRIPPER=cad_parallel \
  ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \
  ROBOT_PIECE_SET=printed \
  ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight \
  ~/.local/bin/robot-arm-sim test -q --tb=short'
```

- Three failures are PRE-EXISTING and not yours. Leave them; if they appear in
  your run, say so and move on:
  - `tests/test_dual.py::test_legacy_fixture_both_arms_physically_lift_and_place`
  - `tests/test_frame_pins.py::test_rig_state_snapshot_matches_manifest`
  - `tests/test_wrist_scan_runner.py::test_runner_registers_and_labels_a_sim_board`

## Background

Read `tools/block5r_wide_solve.py` first (commit `6827b5f`), especially `FLIP`,
`camera_matrix`, `project_tool_frame`, `pnp_tool_frame`.

Points live in a flipped camera frame: `depth = -z`,
`u = fx*x/depth + cx`, `v = fy*(-y)/depth + cy`, i.e. `p_tool = diag(1,-1,-1) p_cv`.

Two defects were just fixed there:

1. One path solved PnP *with* lens distortion while the validating path projected
   *without* it.
2. `cv2.solvePnP(..., SOLVEPNP_IPPE)` was called and the **first** solution taken.
   IPPE returns two valid solutions for a planar target; taking the first produced
   a 32° outlier on a 10-marker view (`reports/rot_scan_20261127.md`). The fix uses
   `cv2.solvePnPGeneric`, refines both branches with `solvePnPRefineLM`, and keeps
   the lower reprojection residual.

Defect 2 is still live elsewhere in the tree. Find and fix it.

## Task 1 — consolidate planar PnP

- Create `chess_sim/projection.py` with the single implementation: `FLIP`,
  `camera_matrix(calibration)`, `project_tool_frame(points, calibration)`,
  `pnp_tool_frame(obj, img, calibration)`. Move that code out of
  `tools/block5r_wide_solve.py`; have the tool import from the new module and keep
  its CLI output identical.
- Audit every planar PnP call site:
  `grep -rn "SOLVEPNP_IPPE|solvePnP(" tools/ live/ chess_sim/ --include=*.py`.
  Decide for each whether it is planar (markers, board face, ChArUco) or a general
  3D correspondence problem. Planar sites route through `pnp_tool_frame`. Do not
  rewrite non-planar sites; list them in the report.
- No silent behaviour change: a site that took the first branch now takes the
  lower-residual one. After each change run that tool's own test if it has one, and
  re-run any report command against capture data already on the workstation;
  report before/after numbers.
- Add `tests/test_projection.py` covering the shared module:
  projection convention vs the manual formula with zero distortion; behind-camera
  points returning nan; a synthetic round trip recovering a known pose to better
  than 0.2 mm / 0.05°; and the returned residual never worse than the first raw
  IPPE solution.

## Task 2 — one RMS convention, robust-loss discipline

- Add to `chess_sim/projection.py` two explicitly named helpers, e.g.
  `per_axis_rms(projected, observed)` and `per_component_rms(projected, observed)`,
  with docstrings stating the √2 relationship. Do not change the meaning of any
  existing printed number without saying so.
- Add a short **additive** subsection to `docs/TESTS.md`: which convention each
  reporting tool uses, that they are not interchangeable, and that an acceptance
  metric must use the same objective as the fit it judges (a robust loss such as
  `soft_l1` can raise a plain RMS — see
  `reports/confidence_checklist_20261127.md`).

## Task 3 — board-rotation session test and a mount pin

- `tests/test_board_offset_invariance.py` already proves the algebra synthetically.
  Extend it (or add a sibling) to a TWO-SESSION case: session A with the mat at yaw
  `Y0`, session B with the mat physically rotated by a known amount, both with the
  SAME true mount and the same flange poses. Assert the mount recovered from B
  equals the mount from A, and that the derived board pose differs by exactly the
  applied rotation. A session rotation must move the board, never the mount.
- Add `tools/board_rotation_check.py`: a capture harness you write but do **not**
  run. It captures N poses through the same bridge the other live tools use (app
  `http://127.0.0.1:8080`, bridge `http://192.168.1.167:8100`; see
  `live/wrist_scan_runner.py` and `tools/block5q_wide_cal.py`), solves the board
  pose per view with the mount pinned, and prints the board pose, per-view scatter,
  and the implied session rotation relative to a stated reference. It must refuse
  to move the arm unless `--execute` is passed, home first, print each leg, and
  stop on an HTTP 409.
- Add a test pinning the camera mount against silent rewriting: assert the sha256 of
  the calibration file equals a recorded constant when the file exists, skip
  otherwise. Use the pin style in `tests/test_frame_pins.py`. Deployed bench
  calibration: id `handeye-2026-11-18T11-block4c`, sha256
  `70f6c51893d294fdf1512c896f5f7171694b06fce8c888e6c04b00a788b144f4`, intrinsics
  831.897 / 833.014.

## Hard constraints

- Do NOT modify `camera_in_tool` values, `.private/` calibration contents,
  `live/board.json`, `live/wrist_calibration.json`, `rig_state/`, or anything under
  `.private/` (read-only).
- Do NOT move the arm. Do not call `/move`, `/recover`, `/wrist/*`, `/board/*` or
  any actuating endpoint. `tools/board_rotation_check.py` is written, not run.
- Do NOT change the `wrist_flex` chain limit, the simulator render FOV, or any
  tolerance/gate. Those are held for the parent.
- Do NOT commit, push, or create branches.
- Keep diffs minimal, in existing style, no reformatting of untouched code.

## Report back

1. Files created/changed, one line each with why.
2. The PnP audit: every site found, planar or not, what you did.
3. Before/after numbers for any re-run tool.
4. Test command and its tail (counts, names of failures).
5. Anything you could not do and why.
6. Anywhere you disagreed with these instructions and what you did instead.
