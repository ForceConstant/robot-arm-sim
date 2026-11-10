# Phase 11c C1/A1 paired comparison (2026-11-10)

**Host:** bc250-cachy (`brian@192.168.1.171`, CachyOS BC-250, 16c/14GB). Phase 11 training/sim/analysis runs executed on bc250-cachy (MuJoCo CPU/EGL).  
**Working env (identified):** `/home/brian/robot-arm-sim/.venv/bin/python` (3.11.18, deps present), run from `/home/brian/robot-arm-sim-phase11c` with `MUJOCO_GL=egl PYTHONPATH=. ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5`.

## Receipts compared
| Label | Path | self_occlusion | Notes |
|---|---|---|---|
| Baseline (Phase 9 accepted) | `.private/phase9/heldout-oblique.json` | (absent) | Historical accepted envelope |
| Paired no self-occ | `.private/phase11c/heldout-oblique-noselfocc-s8t25.json` | `false` | 11c codebase |
| Paired self-occ | `.private/phase11c/heldout-oblique-selfocc-s8t25.json` | `true` | 11c codebase, toggle only |

**Protocol (all):** seed **9300**, **8 sessions × 25 turns = 200**, **6-view** oblique, marker registration, calibration `.private/phase9/calibration.json`, same wrist model hash `96e3e6fe826a97626ee2ad818879816e22ad10ed2af1e35e9dcbf82875985534`. Session layout identical (0,2,4,6 crowded; 1,3,5,7 sparse; 29 cases/session).

**Controlled-toggle evidence (paired runs):** paired receipts share identical `source_sha256` for `tools/benchmark_phase9.py`, `chess_sim/wrist_scan.py`, `chess_sim/self_occlusion.py`, `chess_sim/wrist_calibration.py`, `.private/phase9/calibration.json`, `chess_sim/assets/wrist_scan_plan.json`, plus same model/calibration/plan/seed/sessions/views/method. **Self-occ vs paired no-self-occ is a controlled comparison** (toggle only). **Baseline vs paired is contextual** (different code era and case/rejection composition).

## Summary metrics

| Metric | Baseline (phase9) | Paired no self-occ | Paired self-occ | Δ self-occ vs paired no | Δ self-occ vs baseline |
|---|---:|---:|---:|---:|---:|
| Turns | 200 | 200 | 200 | — | — |
| Resolved | 188 (94.0%) | 190 (95.0%) | 200 (100.0%) | **+10 (+5.0 pp)** | **+12 (+6.0 pp)** |
| False confirmations | 0 | 0 | 0 | 0 | 0 |
| Move abstentions | 13 | 11 | 1 | **-10** | **-12** |
| Move status | confirmed 187, insufficient 13 | confirmed 189, insufficient 10, ambiguous 1 | confirmed 199, ambiguous 1 | insufficient eliminated in this sample | improved |
| Registration ≤2mm/1° | 8/8 | 8/8 | 8/8 | 0 | 0 |
| Reg XY mean / max (mm) | 0.6030 / 0.8039 | 0.4941 / 0.6998 | 0.4941 / 0.6998 | 0 | better than baseline |
| Reg yaw mean / max (deg) | 0.0278 / 0.0863 | 0.0283 / 0.0555 | 0.0283 / 0.0555 | 0 | comparable |
| Scans used (1/2/3) | 187 / 0 / 13 | 190 / 0 / 10 | 200 / 0 / 0 | no rescans | no rescans |
| Rejection records (safe/correct/total) | 24/24/31 | 23/23/23 | 23/23/23 | 0 | -8 total records (composition differs) |
| Unknown sq mean (min/max) | 2.4978 (0/17) | 3.0673 (0/12) | 0.7534 (0/29) | **-2.3139** | **-1.7444** |
| Unknown ≥5 / ≥10 / zero | 44 / 3 / 51 | 73 / 6 / 38 | 5 / 3 / 182 | -68 / -3 / +144 | -39 / 0 / +131 |
| Elapsed (receipt) | 823.1 s | 209.8 s | 292.0 s | +82.2 s (masking cost) | not directly comparable |

## By-category and record composition

| Category/kind | Baseline | Paired no self-occ | Paired self-occ |
|---|---:|---:|---:|
| Total records | 231 (move 200, invalid 16, occluded 15) | 223 (move 200, invalid 16, occluded 7) | 223 (move 200, invalid 16, occluded 7) |
| quiet resolved/total | 176/188 | 180/188 | 188/188 |
| capture resolved/total | 11/11 | 9/11 | 11/11 |
| promotion resolved/total | 1/1 | 1/1 | 1/1 |
| invalid resolved/total | 16/16 | 16/16 | 16/16 |
| occluded resolved/total | 8/15 | 7/7 | 7/7 |

**Composition caveat:** baseline has 15 occluded records vs 7 in paired runs and different move-category totals; this affects rejection/record deltas. Primary causal evidence is the paired controlled toggle.

## Crowded vs sparse move slices (accurate, per-receipt session metadata)

| Slice | Baseline | Paired no self-occ | Paired self-occ |
|---|---:|---:|---:|
| Crowded moves confirmed/total (rate) | 95/100 (95.0%) | 95/100 (95.0%) | 100/100 (100.0%) |
| Crowded abstentions | 5 (insufficient) | 5 (insufficient) | 0 |
| Crowded unknown mean (min/max, ≥5/≥10) | 0.60 (0/2, 0/0) | 0.91 (0/3, 0/0) | 0.00 (0/0, 0/0) |
| Sparse moves confirmed/total (rate) | 92/100 (92.0%) | 94/100 (94.0%) | 99/100 (99.0%) |
| Sparse abstentions | 8 (insufficient) | 6 (5 insufficient, 1 ambiguous) | 1 (ambiguous) |
| Sparse unknown mean (min/max, ≥5/≥10) | 3.98 (1/8, 33/0) | 4.94 (1/10, 64/2) | 0.64 (0/4, 0/0) |

Observations: in this sample self-occlusion removed all insufficient abstentions in both slices and reduced unknown burden to near-zero; one safe ambiguous promotion-related abstention remained in sparse (unk=0, scans=1). The largest baseline/paired-no-self insufficient burden appears in sparse move cases by this per-record slice (consistent with unknown-square counts reported per move record).

## Abstention details

- **Baseline (13):** all `insufficient`, scans=3. Unknown-count buckets: unk=1×5, unk=4×3, unk=3×2, unk=2×2, unk=5×1.
- **Paired no self-occ (11):** 10 `insufficient` (scans=3, varying unknown squares including some unk=0/1 in recorded reason buckets) + 1 `ambiguous` (scans=1, unk=5).
- **Paired self-occ (1):** 1 `ambiguous` (scans=1, unk=0, answered 64/64, promotion category, expected `a2a1b`, session 3 sparse, safe) — no `insufficient` abstentions.

## R evidence (C1 candidate, bc250-cachy)

- **Reload/determinism (bounded feasible subset):** `benchmark_phase9.py --sessions 1 --turns 2 --seed 9401 --method markers` (self-occlusion on, same env/assets as paired runs) executed twice → `.private/phase11c/r_evidence/det_probe_s1t2_seed9401_r{1,2}.json`. Normalized outputs (excluding `elapsed_s`) **identical** (SHA256 `7deba606984428049b390ea49c404e3de627444fbe5624e6ebc55d9c37b704d5`), summaries identical: turns 2, resolved 2, 0 false confirmations, 0 abstained, 4/4 rejection cases safe, 1/1 registration within 2mm/1°, scans 1-view. Elapsed ~18.2/18.3 s.
- **Subset feasibility note:** an attempted fixed subset `--sessions 2 --turns 3 --seed 9300` failed with `ValueError: position needs more q than the set contains` (promotion/case generation produced a position requiring an extra queen beyond the fixed 32-slot scene). This is a case-generation/slot-inventory constraint, not a perception correctness failure; determinism evidence recorded on a feasible subset instead.
- **Harnessed throughput/RSS/thermal sample (completed):** `tools/phase11c_resource_harness.py` (`phase11c-resource-harness/1`, tag `c1_selfocc_s4t5_seed9401`, rc 0) wrapping `benchmark_phase9.py --sessions 4 --turns 5 --seed 9401 --method markers` (self-occlusion on) → `.private/phase11c/r_evidence/harness_selfocc_s4t5_seed9401.json`, `.private/phase11c/r_evidence/bench_selfocc_s4t5_seed9401.json`.
  - Benchmark: 20 turns, **20/20 resolved (1.0)**, 0 false confirmations, 0 abstained, 4/4 registration within 2mm/1°, 13/13 rejection cases safe, scans 1:20, elapsed 81.8 s (~4.09 s/turn incl. full flow). By-category: quiet 20/20, invalid 8/8, occluded 5/5.
  - Harness: wall 85.84 s, **peak RSS 3,640,588 kB (~3.47 GiB)**, 19 samples (5 s interval). Observed temps: `tctl_c`/`k10temp` 48.25→55.625°C max (last 55.0°C), VRM ~41/43°C max, NVMe ~39.85°C, `amdgpu` reported 0.0°C (sensor/readout likely unavailable/not meaningful here); no throttling/guard trip noted in sampled telemetry.
  - Caveat: bounded sample (not an extended multi-hour soak); throughput includes full benchmark/render/perception flow, not isolated inference micro-benchmark. Further sustained/representative R evidence may be warranted for exit budgets.
- **Throughput (200-turn receipts, receipt-derived):** self-occ 292.0 s (~1.46 s/turn) vs no-self-occ 209.8 s (~1.049 s/turn) for 8×25 protocol (different turn/case mix than harness sample); includes full flow.

## Assessment (draft, not 11c-closed)

**C1/A1 (frozen/geometry + self-occlusion):** strong positive measured signal on controlled paired protocol — 0 false confirmations preserved, 200/200 resolved in this sample, abstentions/unknown burden/rescans materially reduced, registration unchanged. Supports frozen/geometry-first hypothesis for (c) and contributes to (a), but **does not close 11c**.

**Remaining for 11c exit:**
- **R:** extended throughput, peak RSS/memory, thermal/CPU envelope on bc250-cachy (harnessed), broader reload evidence as appropriate.
- **A1:** robustness/unknown-cause analysis under occlusion/shadow/crowding beyond this protocol; stress characterization as needed (or scoped probe/rejection+fallback with evidence).
- **B1 (b):** per-piece type + in-square position/orientation including knight yaw — held-out evaluation of `piece_orientation.py` (type/mesh hypothesis, yaw mod π, XY, acceptance/confidence/runtime) or measured rejection + fallback/training probe as required by exit contract.
- **Exit compliance:** accepted measured path(s) meeting budgets with full receipts, or specific local route(s) rejected with measured blocker + documented fallback. `docs/ROADMAP.md` update on completion.

## Source receipts/artifacts (bc250-cachy)
- `/home/brian/robot-arm-sim-phase11c/.private/phase9/heldout-oblique.json`
- `/home/brian/robot-arm-sim-phase11c/.private/phase11c/heldout-oblique-noselfocc-s8t25.json`
- `/home/brian/robot-arm-sim-phase11c/.private/phase11c/heldout-oblique-selfocc-s8t25.json`
- `/home/brian/robot-arm-sim-phase11c/.private/phase11c/r_evidence/det_probe_s1t2_seed9401_r{1,2}.json`
