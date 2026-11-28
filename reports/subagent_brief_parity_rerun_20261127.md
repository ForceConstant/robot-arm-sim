# Subagent brief — refresh the FULL-PARITY-01 receipt (2026-11-27)

Read this file, then `docs/TESTS.md` sections §0, §0c, T4, T5, T6, T7 and §5.
Run the test; do NOT commit or push. Report per the last section.

## Where

- All execution on the BC-250. Working tree `/home/brian/robot-arm-sim`.
  This container is `/home/node/.openclaw/workspace/robot-arm-sim`; `rsync` is not
  installed here, copy single files with `scp -q <file> cachyos:/home/brian/robot-arm-sim/<path>`.
- First: `cd /home/brian/robot-arm-sim && git pull --ff-only` and confirm
  `git log --oneline -1` is **c260e86**. If the pull is blocked by local changes,
  `git stash push -u`, pull, `git stash pop`, and preserve the 400-odd untracked
  scratch files in `tools/`.
- Runner: `~/.local/bin/robot-arm-sim python <script>` / `... test`.
- Environment every run:

```
ROBOT_GRIPPER=cad_parallel
ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y
ROBOT_PIECE_SET=printed
ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight
```

## Why this run exists

Six changes landed since the last parity run (`a11cb2e`), three of which change what
a sim render contains:

- `24bb955` wrist roll zero −87.21 → **−94.71**, and `wrist_scan_plan.json` shifted
  by −7.5° so its physical poses are unchanged
- `07509b1` comparison FOV 32.1445 → **35.6995°** for the `wrist_landscape` camera
  (marker-size ratio 1.118 → 1.005)
- `872f9a4` **aim laser off by default** (`AppearanceConfig.aim_laser = False`)
- `c260e86` `detect_markers` keeps **one detection per id** (largest quad)
- `d0abbfb` T4 now runs the real `MoveSolver` at 3.0 mm / 8° (`phase2_squares.py`)
- earlier: `chess_sim/projection.py` consolidated planar PnP and RMS conventions

Any parity number quoted from the old receipt describes a different simulator.

## Baseline to compare against (do not re-derive, just report deltas)

```
T4  MoveSolver 62/64 at 3.0 mm and 8 deg; failures e8, f8 on the approach gate
    achieved tip error p50 0.46 mm, p95 1.10, max 1.43
T5  IRL 62/64; refusals e8 (approach 10.9 deg), f8 (9.1 deg), nothing sent, no 409
T6  agreement 64/64 (62 solved both, e8/f8 refused both), no §5 hard gate failure
T7  rig_state verify MATCHES
session board: live/board.json x −11.50 y −37.72 z 423.77 yaw −165.50 (registered
    2026-11-27, re-pinned; PIN_LOG.md 2026-11-27T15:31:35Z)
```

## Task

1. **T4** — `tools/phase2_squares.py --out reports/phase2_squares_20261127b.json`.
   Report solved counts at both gates, the failures with their reason and residual,
   achieved tip error distribution and marker counts. Do not change the gates.
2. **T5** — `tools/cartesian_squares.py --backend irl` with the same target list and
   solver. Home first (`tools/home_arm.py`), hold a `/stream` reader, print every leg,
   stop on the runner's own stop rule. **Do not change the stop rule** — `docs/TESTS.md`
   T5 records an open decision about stop-on-409 and says explicitly not to change it.
   Do not actuate the gripper; hold its measured value.
3. **T6** — `tools/cartesian_squares.py --compare` plus the aim columns if the tools
   are present. Report agreement, paired deltas, per-axis bias and a fail list
   (square + axis + magnitude). State the actual gate result; never assert a pass.
4. **T7** — `tools/rig_state.py verify`. Home the arm and leave it as found.
5. Write the receipt: `reports/full_parity_20261127.md` (prose, numbers, refusals,
   gate verdict) and `reports/full_parity_20261127.json` (the raw numbers). Do not
   edit the old 2026-11-25 receipt.

## Hard constraints

- Do NOT change any gate, tolerance, solver setting, target list or baseline to make
  something pass. If a number moves, report the move.
- Do NOT edit `camera_in_tool`, the calibration file, `live/board.json`,
  `live/wrist_calibration.json`, `rig_state/`, or anything under `.private/`.
- Do NOT actuate the gripper. Do NOT run `POST /scan`.
- Abort and report on: a bridge 409 where the runner says stop, an unexpected
  wrist-roll branch change, a bridge fault, or an EGL/GPU error. On a GPU error stop
  all GPU work; the project rule is not to trust post-fault renders.
- Do NOT commit, push or branch.

## What to do if T4 regresses

The most likely cause is the comparison-camera changes (FOV 35.7, roll −94.71), not
the arm. Stop, say which squares changed, and include the achieved-tip and marker
numbers. Do **not** tune the camera, the FOV or the roll to bring the number back.

## Report back

1. `git log --oneline -1` at run time, and any working-tree changes you had to make.
2. T4: counts, gate verdict, failures with reasons, achieved-tip and marker stats.
3. T5: counts, refusals with reasons, whether it stopped early, settle stats.
4. T6: agreement, paired deltas, bias, fail list, gate verdict.
5. T7: PASS or DRIFT.
6. The exact commands you ran and the tail of each.
7. Anything you could not do, and anywhere you disagreed with this brief.
