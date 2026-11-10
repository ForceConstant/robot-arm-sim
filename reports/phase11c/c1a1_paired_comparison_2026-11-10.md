# Phase 11c C1/A1 paired comparison (2026-11-10)

**Host:** bc250-cachy (`brian@192.168.1.171`, CachyOS BC-250, 16c/14GB). Phase 11 training/sim/analysis runs executed on bc250-cachy (MuJoCo CPU/EGL).  
**Working env (identified):** `/home/brian/robot-arm-sim/.venv/bin/python` (3.11.18, `chess/numpy/mujoco/cv2/scipy` present), run with `MUJOCO_GL=egl PYTHONPATH=. ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5` from `/home/brian/robot-arm-sim-phase11c`.

## Receipts compared
| Label | Path | self_occlusion | Notes |
|---|---|---|---|
| Baseline (Phase 9 accepted) | `.private/phase9/heldout-oblique.json` | (absent) | Historical accepted envelope |
| Paired no self-occ | `.private/phase11c/heldout-oblique-noselfocc-s8t25.json` | `false` | 11c codebase |
| Paired self-occ | `.private/phase11c/heldout-oblique-selfocc-s8t25.json` | `true` | 11c codebase, toggle only |

**Protocol (all):** seed **9300**, **8 sessions × 25 turns = 200**, **6-view** oblique, marker registration, calibration `.private/phase9/calibration.json`, same wrist model hash `96e3e6fe826a97626ee2ad818879816e22ad10ed2af1e35e9dcbf82875985534`. Session layout identical (sessions 0,2,4,6 crowded/cases 29; sessions 1,3,5,7 sparse/cases 29).

**Controlled-toggle evidence (paired runs):** paired receipts record identical `source_sha256` for `tools/benchmark_phase9.py` (`7b6762...`), `chess_sim/wrist_scan.py` (`8400f3...`), `chess_sim/self_occlusion.py` (`9401ea...`), `chess_sim/wrist_calibration.py` (`5e2e4c...`), `.private/phase9/calibration.json` (`151db6...`), `chess_sim/assets/wrist_scan_plan.json` (`bf56ec...`), same model/calibration/plan/seed/sessions/views/method. **Self-occ vs paired no-self-occ is a controlled comparison** (toggle `self_occlusion`/`--no-self-occlusion`). **Baseline vs paired is contextual** (different code era and case/rejection composition).

## Summary metrics

| Metric | Baseline (phase9) | Paired no self-occ | Paired self-occ | Δ self-occ vs paired no | Δ self-occ vs baseline |
|---|---:|---:|---:|---:|---:|
| Turns | 200 | 200 | 200 | — | — |
| Resolved | 188 (94.0%) | 190 (95.0%) | 200 (100.0%) | **+10 (+5.0 pp)** | **+12 (+6.0 pp)** |
| False confirmations | 0 | 0 | 0 | 0 | 0 |
| Move abstentions | 13 | 11 | 1 | **-10** | **-12** |
| Move status | confirmed 187, insufficient 13 | confirmed 189, insufficient 10, ambiguous 1 | confirmed 199, ambiguous 1 | insufficient 10→0, ambiguous 0→1 | improved |
| Registration ≤2mm/1° | 8/8 | 8/8 | 8/8 | 0 | 0 |
| Reg XY mean / max (mm) | 0.6030 / 0.8039 | 0.4941 / 0.6998 | 0.4941 / 0.6998 | 0 | better than baseline |
| Reg yaw mean / max (deg) | 0.0278 / 0.0863 | 0.0283 / 0.0555 | 0.0283 / 0.0555 | 0 | comparable |
| Scans used (1/2/3) | 187 / 0 / 13 | 190 / 0 / 10 | 200 / 0 / 0 | no rescans | no rescans |
| Rejection records (safe/correct/total) | 24/24/31 | 23/23/23 | 23/23/23 | 0 | -8 total records |
| Unknown sq mean (min/max) | 2.4978 (0/17) | 3.0673 (0/12) | 0.7534 (0/29) | **-2.3139** | **-1.7444** |
| Unknown ≥5 / ≥10 / zero | 44 / 3 / 51 | 73 / 6 / 38 | 5 / 3 / 182 | -68 / -3 / +144 | -39 / 0 / +131 |

## By-category and record composition

| Category/kind | Baseline | Paired no self-occ | Paired self-occ |
|---|---:|---:|---:|
| Total records | 231 (move 200, invalid 16, occluded 15) | 223 (move 200, invalid 16, occluded 7) | 223 (move 200, invalid 16, occluded 7) |
| quiet resolved/total | 176/188 | 180/188 | 188/188 |
| capture resolved/total | 11/11 | 9/11 | 11/11 |
| promotion resolved/total | 1/1 | 1/1 | 1/1 |
| invalid resolved/total | 16/16 | 16/16 | 16/16 |
| occluded resolved/total | 8/15 | 7/7 | 7/7 |

**Composition caveat:** baseline includes 15 occluded rejection records (8 resolved/safe per summary semantics) vs 7 in paired runs, and different quiet/capture counts in `by_category` move subsets. This explains part of rejection/record deltas and means baseline-vs-paired improvement is not solely attributable to self-occlusion; **paired toggle comparison remains the primary causal evidence**.

## Crowded vs sparse move slices (from session metadata)

| Slice | Baseline | Paired no self-occ | Paired self-occ |
|---|---:|---:|---:|
| Crowded moves confirmed/total (rate) | 88/100 (88.0%) | 93/100 (93.0%) | 99/100 (99.0%) |
| Crowded abstentions | 12 | 7 | 1 |
| Crowded unknown mean / ≥10 | 4.72 / 3 | 5.62 / 6 | 1.49 / 3 |
| Sparse moves confirmed/total (rate) | 99/100 (99.0%) | 96/100 (96.0%) | 100/100 (100.0%) |
| Sparse abstentions | 1 | 4 | 0 |
| Sparse unknown mean / ≥10 | 0.2756 / 0 | 0.5146 / 0 | 0.0168 / 0 |

Observations: self-occlusion gives the largest gain in crowded sessions (+6 confirmed vs paired no-self, -6 abstentions) and also recovers sparse performance to 100% in this sample (paired no-self regressed vs baseline on sparse moves in this receipt). Unknown burden drops sharply in both slices under self-occlusion.

## Abstention details

- **Baseline (13):** all `insufficient`, scans=3. Reason buckets (unknown count): unk=1×5, unk=4×3, unk=3×2, unk=2×2, unk=5×1.
- **Paired no self-occ (11):** 10 `insufficient` (scans=3; e.g. quiet case expected `c2c3` answered 63/64, unknown_squares [18]) + 1 `ambiguous` (scans=1, unk=5).
- **Paired self-occ (1):** 1 `ambiguous` (scans=1, unk=0, answered 64/64, promotion case expected `a2a1b`, session 3 sparse) — safe abstention, not an `insufficient`/coverage failure.

## R evidence (initial, C1 candidate, bc250-cachy)

- **Reload/determinism (bounded subset):** `benchmark_phase9.py --sessions 2 --turns 3 --seed 9300 --method markers` (self-occlusion on, same env/assets) run twice to `.private/phase11c/r_evidence/det_selfocc_run{1,2}.json`. Normalized outputs (excluding `elapsed_s`) **identical** (same SHA256), summaries identical: `{'resolved':6,'turns':6,'resolution_rate':1.0,'false_confirmations':0,'abstained':0,'registration_sessions':2,'registration_within_2mm_1deg':2,'rejection_cases':0,'rejection_safe':0,'scans_used':{'1':6,'2':0,'3':0}}`.
- **Throughput/memory/thermal:** not yet collected over extended run (resource harness available: `tools/phase11c_resource_harness.py`). Required for 11c exit evidence.

## Assessment (draft, not 11c-closed)

**C1/A1 (frozen/geometry + self-occlusion):** strong positive measured signal on controlled paired protocol — 0 false confirmations preserved, 100% resolution in this 200-turn sample, abstentions 13→11→1, unknown/coverage burden substantially reduced (especially crowded), registration unchanged. This supports the frozen/geometry-first hypothesis as promising for (c) and contributing to (a), but **does not yet close 11c**:
- R: extended throughput/RSS/thermal and broader reload evidence pending.
- A1: robustness under occlusion/shadow/crowding/unknown-cause stress beyond this protocol (and attribution of remaining ambiguous/unknown cases) needs documented analysis and/or stress runs.
- B1 (b): per-piece type + in-square position/orientation incl. knight yaw not yet evaluated with held-out evidence/budgets (`piece_orientation.py` exists as candidate; needs measured eval or rejection+fallback).
- Exit compliance: 11c requires locally runnable training path evidence (throughput/memory/thermal/reload/held-out) **or** specific local route rejected with measured blocker + documented fallback per subtask where applicable. Frozen acceptance must be backed by measured budgets and exit-compliant closure.

## Next evidence steps
1. Collect R throughput/RSS/thermal on bc250-cachy (resource harness over extended/representative run) and record artifact hashes/env.
2. A1 stress/analysis: characterize residual failures/abstentions and shadow/occlusion sensitivity; run targeted stress evals if needed (feasible on bc250-cachy CPU/EGL).
3. B1: design/run held-out orientation/type eval for `piece_orientation.py` (knight yaw mod π, in-square XY, acceptance/confidence, runtime/memory) on bc250-cachy; if insufficient, scoped local training probe or measured rejection+fallback.
4. Produce final 11c receipt docs and **update `docs/ROADMAP.md`** on completion (or document measured blockers/fallbacks per exit contract).
