# Phase 9 — wrist-only calibration and complete-board perception

**Status: in progress (2026-11-03).** Registration and the scan/calibration
software are complete and measured inside the stated envelope; turn resolution
is **not** yet at the exit gate, so Phase 9 stays open.

## What was built

| Area | Module | Notes |
| --- | --- | --- |
| Lens + hand–eye calibration | `chess_sim/wrist_calibration.py` | Versioned file: pinhole intrinsics, Brown–Conrady distortion, serial chain with joint anchors/axes/limits, camera-in-tool transform. Camera pose comes from measured joints only. |
| Reference epochs | `chess_sim/wrist_calibration.py` | `ReferenceEpoch` clears empty-board references on any new registration, recalibration or timestamp reversal. Warping a stale reference is not permitted. |
| Printed fiducial mat | `chess_sim/fiducials.py` | Four 40 mm ArUco `DICT_4X4_50` markers (ids 0–3) on a 0.5 mm mat under the slab, built as geometry (not a texture) so a face-mapping flip cannot silently move the markers. |
| Board pose from markers | `chess_sim/fiducials.py` | Per-view planar PnP seeds, one bounded least-squares fit over every detected corner, RMS gate; abstains below two markers or above 1.2 px RMS. |
| Scan planning | `tools/plan_wrist_scan.py` | Wrist-lens targets solved for near-vertical camera poses inside hard joint limits, scored by the arm's own occlusion, then greedy set cover. Committed plan: `chess_sim/assets/wrist_scan_plan.json` (7 poses; every square clear in ≥2, median 4). |
| Multi-pose perception | `chess_sim/wrist_scan.py` | Per-view `predict_board`, element-wise median of class probabilities (a median rejects one occluded view's confident mistake), unknown below the model threshold. |
| Wrist model | `tools/train_perception_wrist.py` | Trained on the product scene at the scan poses with natural arm self-occlusion; occluded/out-of-frame squares are labelled `unknown`, never guessed. |
| Held-out benchmark | `tools/benchmark_phase9.py` | Session-separated registration and turn evaluation; simulator truth only builds cases and scores them. |

## Envelope measured

* Product board: 28 mm squares, 15 mm labelled border, 3.2 mm slab, 0.5 mm mat
  (surface at 0.4337 m).
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
| Worst XY error | 0.49 mm | — |
| Worst yaw error | 0.06° | — |
| Markers seen per session | 1–2 of 4 at the accepted pose | — |

Turns (`--sessions 8 --turns 25`, 200 held-out turns):

| Metric | Result | Gate |
| --- | --- | --- |
| Resolved after ≤3 scans | 145/200 (72.5%) | ≥95% — **fail** |
| False move confirmations | 0/200 | 0 — **pass** |
| Abstentions | 56, all `insufficient` | — |
| Turns needing a second scan | 0 | — |
| Turns resolving on scan 3 | 56 | — |
| Deliberate invalid rearrangements rejected | 16/16 | all — **pass** |
| Deliberate occluded cases abstained | 9/10 | all — one correct confirmation, see note |

All 56 abstentions are `insufficient`, i.e. the changed squares were not
observed well enough — not confident contradictions. Roughly 8–19 squares per
board remain `unknown` after fusion, because the arm covers that part of the
board in every pose that reaches it. Rescanning does not help: the same seven
poses produce the same occlusion.

Note: the first run reported one invalid rearrangement as `confirmed`. The
generator had compared full FENs, so a rearrangement differing only in move
clocks passed the "illegal" test while being piece-identical to a legal
successor. It now compares piece placement only; the corrected run
(`heldout-8x25-v2.json`) rejects 16/16. One occluded case returned the correct
move instead of abstaining: the restricted pose subset still observed the
changed squares, so this is a case-construction limit, not a false
confirmation.

## Why it is not complete

1. **Coverage per pose.** Seven near-vertical poses leave 8–19 squares hidden
   behind the arm on crowded boards. The Phase 9 exit needs ≥95% turn
   resolution; 72% is the direct consequence.
2. **No oblique views yet.** Every planned pose is close to vertical. Side
   views would see under the wrist for the near ranks but were not qualified
   here, and would need their own distortion/registration check.
3. **Reference sensitivity.** A square that matches its empty reference in a
   view is either empty or occluded; the fusion therefore relies on having
   enough genuinely clear views per square. With 2–4 clear views per square
   this is marginal on the crowded boards.

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
