# Phase 11c C1/A1 paired comparison (draft, 2026-11-10 15:xx ET)

**Host:** bc250-cachy (`brian@192.168.1.171`, CachyOS BC-250, 16c/14GB). Phase 11 runs executed on bc250-cachy (MuJoCo CPU/EGL).  
**Working env (identified):** `/home/brian/robot-arm-sim/.venv/bin/python` (3.11.18, deps present) and `/home/brian/robot-arm-env/bin/python` (3.12.14) on bc250-cachy; paired evals ran from `/home/brian/robot-arm-sim-phase11c` with 11c code/assets and recorded source hashes.

## Receipts compared
- **Historical baseline (Phase 9 accepted envelope):** `.private/phase9/heldout-oblique.json` — seed 9300, 8 sessions × 25 turns = 200, views 6, method markers, no `self_occlusion` field, model/calibration sha recorded.
- **Paired no self-occlusion:** `.private/phase11c/heldout-oblique-noselfocc-s8t25.json` — `self_occlusion=false`, same seed/sessions/turns/views/method/assets context.
- **Paired self-occlusion:** `.private/phase11c/heldout-oblique-selfocc-s8t25.json` — `self_occlusion=true`, same as paired no-self except toggle.

**Paired apples-to-apples evidence:** both paired receipts record identical `source_sha256` for `tools/benchmark_phase9.py`, `chess_sim/wrist_scan.py`, `chess_sim/self_occlusion.py`, `chess_sim/wrist_calibration.py`, `.private/phase9/calibration.json`, `chess_sim/assets/wrist_scan_plan.json`, identical `model_sha256`, seed, session/case layout metadata, views/method. Therefore **self-occ vs paired no-self-occ** is a controlled toggle comparison on the same 11c codebase/protocol. **Baseline vs paired** comparisons are contextual (different code era and case/rejection composition).

## Summary metrics

| Metric | Baseline (phase9) | Paired no self-occ | Paired self-occ | Δ self-occ vs paired no | Δ self-occ vs baseline |
|---|---:|---:|---:|---:|---:|
| Turns | 200 | 200 | 200 | — | — |
| Resolved / resolution rate | 188 / 94.0% | 190 / 95.0% | 200 / 100.0% | +10 (+5.0 pp) | +12 (+6.0 pp) |
| False confirmations | 0 | 0 | 0 | 0 | 0 |
| Move abstentions (non-confirmed) | 13 | 11 | 1 | -10 | -12 |
| Move status | confirmed 187, insufficient 13 | confirmed 189, insufficient 10, ambiguous 1 | confirmed 199, ambiguous 1 | fewer insufficient, 1 ambiguous remains | improved |
| Registration sessions ≤2mm/1° | 8/8 | 8/8 | 8/8 | 0 | 0 |
| Reg XY mean/max (mm) | 0.603 / 0.8039 | 0.4941 / 0.6998 | 0.4941 / 0.6998 | 0 / 0 | improved vs baseline (note: different run context) |
| Reg yaw mean/max (deg) | 0.0278 / 0.0863 | 0.0283 / 0.0555 | 0.0283 / 0.0555 | 0 | comparable |
| Scans used (1/2/3) | 187 / 0 / 13 | 190 / 0 / 10 | 200 / 0 / 0 | no 3-scan cases | fewer rescans |
| Rejection records (safe/correct/total) | 24/24/31 | 23/23/23 | 23/23/23 | 0 | -8 records, safety preserved in paired |
| Unknown squares mean (min/max) | 2.4978 (0/17)* | 3.0673 (0/12) | 0.7534 (0/29) | **-2.3139** | **-1.7444** |
| Unknown ≥5 / ≥10 / zero | 44 / 3 / 51 | 73 / 6 / 38 | 5 / 3 / 182 | -68 / -3 / +144 | -39 / 0 / +131 |

\*Baseline receipt aggregate over 231 records (move+rejection) gave mean 2.4978, min/max 0/17, ≥5 44, ≥10 3, zero 51 in parsed totals; earlier rough summary noted different values — final report should use per-receipt parsed fields consistently and, if needed, separate move-only vs all-record unknown distributions.

## By-category (from receipts)

| Category | Baseline | Paired no self-occ | Paired self-occ |
|---|---:|---:|---:|
| quiet | 176/188 resolved | 180/188 | 188/188 |
| capture | 11/11 | 9/11 | 11/11 |
| promotion | 1/1 | 1/1 | 1/1 |
| invalid (rejection) | 16/16 | 16/16 | 16/16 |
| occluded (rejection) | 8/15 | 7/7 | 7/7 |

Record totals/kinds: baseline 231 (200 move, 16 invalid, 15 occluded); paired runs 223 (200 move, 16 invalid, 7 occluded). Baseline `by_category` reflects different rejection/occlusion case composition than paired runs.

## Abstention details (moves)
- **Baseline:** 13 abstained, all `insufficient`, all scans=3 (unknown counts in examples: 1×5, 4×3, 3×2, 2×2, 5×1).
- **Paired no self-occ:** 11 abstained — 10 `insufficient` (mostly scans=3 with varying unknown squares, incl. some unk=0/1) + 1 `ambiguous` (scans=1, unk=5). Example: `{"answered":63,"case":22,"category":"quiet","expected":"c2c3","status":"insufficient","scans":3,"unknown":1}`.
- **Paired self-occ:** 1 abstained — `ambiguous`, scans=1, unk=0 (`{"case":5,"category":"promotion","expected":"a2a1b","status":"ambiguous","scans":1,"unknown":0,"answered":64,"safe":true}`), no insufficient abstentions.

## Attribution / comparability notes
1. **Controlled toggle (strong):** paired self-occ vs paired no-self-occ share identical protocol metadata and recorded source/calibration/plan/model hashes; observed improvement (+10 resolved, -10 abstentions, unknown burden sharply reduced, 0 three-scan cases) is attributable to self-occlusion handling in this paired comparison.
2. **Baseline delta (contextual):** rejection-record and category composition differ (31 vs 23 rejection records; occluded 15 vs 7; capture/quiet resolved totals differ). Improvements vs 94.0% baseline are consistent with self-occlusion benefit but cannot be attributed solely to the toggle without case-manifest equivalence.
3. **Safety:** 0 false confirmations in all three receipts; paired rejection handling 23/23 safe/correct.
4. **Registration:** unchanged and within budget across paired runs (identical reg stats), all 8/8 within 2 mm / 1°.

## C1/A1 assessment (draft, not exit-closed)
- **Positive evidence:** self-occlusion-aware frozen/perception path materially improves C1 observability and A1 move-resolution outcomes on the measured paired protocol with no safety regression.
- **Remaining for 11c exit evidence:** (R) reload determinism on fixed subset, sustained throughput, peak RSS, thermal/CPU envelope on bc250-cachy; (A1) robustness slices under occlusion/shadow/crowding/unknown-cause stress with clear budgets; (B1) per-piece type + in-square position/orientation including knight yaw evaluation (and A2/B2 probes or measured rejection+fallback where required). A locally runnable training path must be demonstrated/evidenced or specifically rejected with measured blocker and documented fallback per subtask as applicable.

## Source receipts (bc250-cachy paths)
- `/home/brian/robot-arm-sim-phase11c/.private/phase9/heldout-oblique.json`
- `/home/brian/robot-arm-sim-phase11c/.private/phase11c/heldout-oblique-noselfocc-s8t25.json`
- `/home/brian/robot-arm-sim-phase11c/.private/phase11c/heldout-oblique-selfocc-s8t25.json`
