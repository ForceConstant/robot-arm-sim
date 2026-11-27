# Paired squares receipt — 2026-11-27 (T4 / T5 / T6, plus the aim columns)

Run on the BC-250 (`cachyos`) at the workstation tree `a3e4f03` + this session's
edits (`docs/TESTS.md` §0c, `tools/repin_rig_state.py`, `tests/test_repin_rig_state.py`,
board re-pin). Sim and IRL used the *same* rig state: the four files from
`rig_state/`, `live_config` solver `mink`, `move_tolerance_mm` 3.0,
`TIP_IN_CHAIN` = −0.11045 m, `ROBOT_GRIPPER=cad_parallel` +
`.private/gripper-assets-5y`.

## Session board registration

- `tools/calibrate_board_charuco.py --layout charuco --captures live/captures_boardreg_20261127`
  → **143 sightings over 18 captures**, median pose **x −11.50, y −37.72,
  z 423.77 mm, yaw −165.50°**; spreads 27.62 / 27.32 / 20.77 mm, **2.42°**.
- Re-run of the identical captures reproduced the previous numbers exactly
  (−165.50° yaw), so the mat had **not** moved since those captures — no fresh
  capture batch was needed.
- `live/board.json` written from the median; re-pinned with
  `tools/repin_rig_state.py --why "session board registration 2026-11-27 …"`:
  `board.json` `9928f25351` → `434db96e8d`, manifest + `BOARD_EXPECTED` +
  `MANIFEST_HASHES` updated, `rig_state/PIN_LOG.md` line added
  (2026-11-27T15:31:35Z). `tests/test_frame_pins.py` 5/5 pass afterwards.
- The console (`robot-live`) caches the board frame at construction, so it was
  restarted to load the registered pose (verified `GET /board` = the new values).

## T4 — sim baseline

`tools/cartesian_squares.py --backend sim --bridge-limits http://192.168.1.167:8100`
→ `reports/squares_sim.json`

| | value |
| --- | --- |
| solved | **64/64** |
| IK residual mm | p50 3.85, p95 9.98, max 15.30 |
| achieved tip err mm | p50 3.85, p95 9.96, **max 15.35** (e8) |
| joint-limit violations | 0 |
| markers read back | 258 |

Baseline (2026-11-25) was p50 4.21 / p95 10.29 / max 12.83. p50 and p95 are
slightly better; the single worst sample is 2.5 mm above the old max (e8, 15.35),
which is the far-row corner the IRL column refuses. The 3.0 mm figure is the
solver setting, not a pass gate in this tool (§0b).

## T5 — IRL sweep

Arm homed first (`tools/home_arm.py`, 2 legs, last error 1.22°), `/stream`
reader attached, stop rule unchanged (no `--continue-on-refusal`).
→ `reports/squares_irl.json`, log `reports/squares_irl.log`

| | value |
| --- | --- |
| solved | **62/64** |
| refusals | 2, both local pre-check, nothing sent: **e8** approach 10.9° > 8°, **f8** approach 9.1° > 8° |
| HTTP 409 / bridge fault | none |
| stopped early | no — all 64 legs attempted |
| settle | p50 1.11°, max 1.91° |
| achieved tip err mm | p50 **4.82**, p95 **6.32**, max **9.21** (h1) |
| wrist-roll deltas | all 0.0° — one branch for the whole sweep |

Baseline would have been p50 9.25 / p95 15.80 (2026-11-25, on the stale board).
This run is well inside it: the freshly registered board removed most of the
frame error that the old pin was carrying.

## T6 — paired comparison

`tools/cartesian_squares.py --compare reports/squares_sim.json reports/squares_irl.json`

- **agreement 62/64**; the two mismatches are exactly e8 and f8 (sim solves them,
  the IRL pre-check refuses on the 8° approach gate — same squares, same reason).
- Paired tip-error delta (IRL − sim): mean **−0.08 mm**, p50 **+1.19**, p95
  **+2.81**, range −7.29 … +3.75 mm. The IRL column is slightly *better* than
  sim in the far rows (a1 sim 13.1 vs IRL 5.8; g8 8.1 vs 1.7) and slightly worse
  near the base.
- Per axis, paired |error|: x sim 0.80 / IRL 1.55 (p50), y sim 1.45 / IRL 3.00,
  z sim 3.30 / IRL 2.75. The IRL column carries a consistent **+y bias of
  ≈1.5 mm** relative to sim (p50), visible on most rows — a physical offset, not
  a solver one.
- **Fail list** (square + axis + magnitude): no §5 hard gate fails. Explained
  refusals: **e8**, **f8** (approach gate, above). Worst paired disagreements:
  a1 x/y (sim +13.1 vs IRL +5.8), g8 z (+8.1 vs +1.7), f7 (+6.4 vs +0.7),
  d8 (+8.7 vs +3.1), h8 (+5.8 vs +0.6). Sim's worst squares are all row 8 / far
  corners: e8 15.35, f8 13.32, a1 13.12, b1 10.17, d8 8.72, h1 8.70.

## Aim columns (same session, `tools/aim_squares.py`)

→ `reports/aim_squares_20261127.json` (10:47:17 → 11:05:30), eval
`reports/aim_squares_20261127_report.md`, heatmap
`reports/aim_squares_compare_20261127.png`.

| metric | 2026-11-25 before | 2026-11-27 after |
| --- | --- | --- |
| aim board-mm p50 / p95 / max | 7.44 / 11.60 / 12.01 | **3.14 / 6.75 / 7.20** |
| tip FK-mm p50 / p95 / max | 9.25 / 15.80 / 16.53 | **4.55 / 5.80 / 6.02** |
| aim ≤ 5 mm | 29/60 | **39/59** |
| tip leg settle p50 / max | 2.27 / 3.47 | **1.05 / 1.77** |
| px (configured frame) p50 | 52.92 | 42.83 |
| px (marker solve) p50 | 48.42 | 97.95 |

Both millimetre columns improve substantially. **One regression to flag:** the
pixel column measured through the *marker* board solve roughly doubled while the
commanded-frame pixel column improved and the mm errors halved — the marker
plane and the configured board frame now disagree more than they did on the old
pin. That is the §7 layer-4 question (board pose vs marker solve), not a solver
change; it needs a look before this is quoted as a pixel result.

Refusals in the aim run: tip d8 13.6°, e8 23.5°, f8 21.8°, g8 8.9° (the aim leg
of those four and of a8 — plane failure — were skipped, not commanded). All are
recorded in the receipt; none is silently dropped.

## T7 — rig-state re-verify

`tools/rig_state.py verify` after ~450 commanded legs: **MATCHES the manifest**
(no DRIFT on any of the four files). Arm homed and left there; gripper held at
its measured value (50.38) and never actuated.
