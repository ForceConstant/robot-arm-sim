# Phase 9 — wrist-only calibration and complete-board perception

**Status: complete 2026-11-03 for a stated envelope (owner decision, Brian).**
Registration, scan planning, calibration and turn inference are implemented and
measured. Turn resolution is **94.0%**, accepted in place of the initial 95%
target with the limits below named explicitly; the residual `unknown` band is
queued as a Phase 10 refinement (self-model occlusion).

## What was built

| Area | Module | Notes |
| --- | --- | --- |
| Lens + hand–eye calibration | `chess_sim/wrist_calibration.py` | Versioned file: pinhole intrinsics, Brown–Conrady distortion, serial chain with joint anchors/axes/limits, camera-in-tool transform. Camera pose comes from measured joints only. |
| Reference epochs | `chess_sim/wrist_calibration.py` | `ReferenceEpoch` clears empty-board references on any new registration, recalibration or timestamp reversal. Warping a stale reference is not permitted. |
| Printed fiducial mat | `chess_sim/fiducials.py` | Four 40 mm ArUco `DICT_4X4_50` markers (ids 0–3) on a 0.5 mm mat under the slab, built as geometry (not a texture) so a face-mapping flip cannot silently move the markers. |
| Board pose from markers | `chess_sim/fiducials.py` | Per-view planar PnP seeds, one bounded least-squares fit over every detected corner, RMS gate; abstains below two markers or above 1.2 px RMS. |
| Scan planning | `tools/plan_wrist_scan.py` | Wrist-lens targets solved inside hard joint limits — a vertical grid plus tilted, board-facing views — scored by the arm's own occlusion and lens-to-square distance, then greedy set cover. Committed plan: `chess_sim/assets/wrist_scan_plan.json` (6 poses, 3 tilted; every square clear in ≥3, median 5). |
| Multi-pose perception | `chess_sim/wrist_scan.py` | Per-view `predict_board`, element-wise median of class probabilities (a median rejects one occluded view's confident mistake), unknown below the model threshold. |
| Wrist model | `tools/train_perception_wrist.py` | Trained on the product scene at the scan poses with natural arm self-occlusion; occluded/out-of-frame squares are labelled `unknown`, never guessed. |
| Held-out benchmark | `tools/benchmark_phase9.py` | Session-separated registration and turn evaluation; simulator truth only builds cases and scores them. |

## Envelope measured

* Product board, **measured from the table as the datum**: 28 mm squares, 15 mm
  labelled border, 3.2 mm slab on a 0.5 mm printed mat -> playing surface
  **3.7 mm above the table**, marker face **0.5 mm above the table**, arm datum
  **30 mm above the table**. World z is derived (`table_top + height`) and never
  used as the reference for a board measurement. A 55 mm border carrying the
  markers directly was measured and rejected (51/64 squares) — see
  [`BOARD_MARKER_LAYOUT.md`](BOARD_MARKER_LAYOUT.md).
* Mount: `hardware/current_layout.json` — 90 mm board edge to base frame,
  30 mm riser, one follower, custom jaw revision `4dca0986…`.
* Camera: wrist only, 640×480, saved calibration `sim-current-layout-custom-v3`.
* Board placement per session: ±8 mm in x/y, ±2.5° yaw, plus randomized key
  light intensity/direction, fill and shadow softness.
* Positions: alternating full 32-piece starts and sparse 2–8-piece draws.

## Measured results

Registration (`--sessions 8`), each session a fresh board pose and lighting:

| Metric | Result | Gate |
| --- | --- | --- |
| Sessions registered | 8/8 | — |
| Within 2 mm XY and 1° yaw | 8/8 (100%) | ≥95% — **pass** |
| Worst XY error | 0.80 mm | — |
| Worst yaw error | 0.09° | — |

Turns (`--sessions 8 --turns 25`, 200 held-out turns, oblique plan):

| Metric | Result | Gate |
| --- | --- | --- |
| Resolved after ≤3 scans | 188/200 (94.0%) | 95% target — **accepted at 94.0%** |
| False move confirmations | 0/200 | 0 — **pass** |
| Abstentions | 13, all `insufficient` | — |
| Turns resolving on the first scan | 187/200 | — |
| Deliberate invalid rearrangements rejected | 16/16 | all — **pass** |
| Deliberate occluded cases | 30/31 safe (abstained or unregistered); one returned the correct move | all — see note |

By category: quiet 176/188, capture 11/11, promotion 1/1, invalid 16/16.
The 13 abstentions each left 15–61 squares `unknown` on a crowded board.

Plan comparison, same 200-turn suite:

| Plan | Poses | Views per square (min / median) | Resolution |
| --- | --- | --- | --- |
| Vertical only | 7 | 2 / 4 | 72.5% |
| Vertical + oblique | 6 | 3 / 5 | **94.0%** |
| Vertical + oblique, ≥4 views | 8 | 4 / 6 | 92.0% (rejected) |

The 8-pose candidate reached 15/15 safe occluded cases and 31/31 safe
rejections, but resolved fewer quiet moves (172/188 vs 176/188). Extra tilted
views add grazing angles and parallax without removing the remaining unknown
band, so the 6-pose plan stays selected. Receipt:
`.private/phase9/heldout-mv4.json`.

**Rejected: drop reference-matching votes.** A square whose patch matches that
pose's empty reference is either empty or hidden behind the arm, so those views
were made to abstain instead of voting `empty`. Measured 39.5% resolution (121
abstentions): it removes votes that were carrying other turns and adds no
information to the genuinely unclear squares. Reverted; receipt
`.private/phase9/heldout-informed.json`.

Note: in the first vertical-only run one invalid rearrangement was reported
`confirmed`; the generator had compared full FENs, so a rearrangement differing
only in move clocks passed the "illegal" test while being piece-identical to a
legal successor. It now compares piece placement only, and rejects 16/16. The
single "unsafe" occluded case returned the correct move rather than abstaining:
the restricted pose subset still observed the changed squares, so it is a
case-construction limit, not a false confirmation.

## Limits of the accepted envelope

The 95% target was not reached. Brian accepted 94.0% on 2026-11-03 provided the
limit is stated: crowded 32-piece boards can leave 15–61 squares `unknown`, and
about one turn in sixteen then abstains (`insufficient`) instead of guessing.
No wrong move was ever confirmed, so the failure mode is abstention, not error.

## Why 95% was not reached

1. **One point short.** 94.0% needs 190/200; the shortfall is the 13 crowded
   boards whose remaining `unknown` band is 15–61 squares. More poses did not
   help (measured below); the next lever is excluding the arm's own shadow.
2. **Occluded-case construction.** Restricting the scan to poses that hide a
   square is measured by segmentation coverage; a square can still be readable
   through a grazing patch, so such cases occasionally resolve correctly
   instead of abstaining. Tighten by requiring the square's patch to be
   `unknown` in every active view before scoring the case.
3. **Reference sensitivity.** A square that matches its empty reference in a
   view is either empty or occluded, so the fusion depends on enough genuinely
   clear views. The oblique plan raises that from 2–4 to 3–5 per square, which
   is what produced the jump.

## Reproduction

```bash
scripts/ensure_venv.sh
export MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
       ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
       ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
       PYTHONPATH=.
.venv/bin/python tools/plan_wrist_scan.py          # regenerate the pose plan
.venv/bin/python tools/train_perception_wrist.py   # regenerate model_wrist.json
.venv/bin/python tools/benchmark_phase9.py --sessions 8 --turns 25
```

Receipts: `training/perception/…` is not used here; see the private paths
`.private/phase9/heldout-8x25[-v2].json`, `.private/phase9/wrist_training.json`
and `docs/PHASE9_*` for the public summary.

## Scope limits

* Camera calibration is exported from the digital twin, not measured on hardware.
* Fiducial detection needs the printed mat; a marker-free board falls back to
  the checker-lattice fit, which was not accurate enough from wrist views
  (3/6 single-view fits, 3–9 mm origin error).
* No move execution is tested here; this is perception and state inference only.
* Simulated success never passes a physical gate (Phases 16–18).
