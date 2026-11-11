# Phase 11c results and closure (2026-11-10)

**Status:** exit-compliant closure recorded (measured frozen/calibrated-geometry decisions and measured rejection+fallback where required). Simulation-first evidence only; simulated success does not pass a physical gate.

**Execution host:** bc250-cachy (`brian@192.168.1.171`, CachyOS BC-250, 16c/14GB). MuJoCo physics CPU/EGL; GPU compute not implied. Working env: `/home/brian/robot-arm-sim/.venv/bin/python` 3.11.18, `MUJOCO_GL=egl PYTHONPATH=. ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5`.

**Worktree/commits:** local `phase11c` from `origin/main@30e4d22` (Phase 10/11b complete, not reopened), synced content to `/home/brian/robot-arm-sim-phase11c`. Key commits include `2fa833a`, `542066b`, `7845b65`, `9a88878`, `cab9af9`, `6f427e8`. Phase 10 WIP archived at `/home/node/.openclaw/workspace/robot-arm-sim/.private/wip-archive-2026-11-10T1345ET/` (not deleted).

**Scope (roadmap):** choose/train candidate for worst case, or record measured frozen-model + calibrated-geometry decision meeting same budgets for (a) opponent-move change detection robust to occlusion and shadows, (b) per-piece type plus in-square position/orientation including knight yaw as required by Phase 10, (c) wrist-only board registration under real occlusion (Phase 9). Exit requires locally runnable training path for vision with throughput, memory, thermal, reload and held-out evidence — or a specific local route rejected with a measured blocker and documented fallback. Local services only (BC-250/HTPC/CPU); cloud training out of scope.

**Priority/approach (adopted, per independent review, partially sound):** (c)→(a)→(b), frozen/geometry-first as measured hypothesis with pre-registered budgets, paired apples-to-apples evals, exit-compliant closure per subtask.

## Evidence artifacts
- `docs/PHASE11C_VISION_PLAN.md`, `docs/PHASE11C_EVIDENCE_PLAN.md`
- `reports/phase11c/c1a1_paired_comparison_2026-11-10.md` — controlled paired C1/A1 receipts, slices, R evidence
- `reports/phase11c/a1_analysis_notes_2026-11-10.md`
- `reports/phase11c/b1_orientation_eval_2026-11-10.md`
- `tools/phase11c_resource_harness.py`, `tools/b1_orientation_eval.py`
- bc250-cachy receipts: `.private/phase11c/heldout-oblique-selfocc-s8t25.json`, `.private/phase11c/heldout-oblique-noselfocc-s8t25.json`, `.private/phase9/heldout-oblique.json`, `.private/phase11c/r_evidence/*` (determinism + harness + bench), `.private/phase11c/b1_orientation_s4_seed11401.json`

## Subtask outcomes

### (c) Wrist-only board registration under real occlusion — **Measured frozen/calibrated-geometry decision: accepted (sim envelope)**
- **Candidate C1:** marker registration + self-occlusion-masked wrist occupancy/colour + median fusion + existing move inference (6-pose oblique, wrist MLP/calibration), `chess_sim/self_occlusion.py` integrated in wrist scan path.
- **Controlled paired held-out (seed 9300, 8×25=200, 6-view oblique, identical source/calibration/plan/model hashes):** self-occ 200/200 resolved vs paired no-self 190/200 vs baseline 188/200; **0 false confirmations** in all; abstentions 13→11→1 (self-occ residual: 1 safe ambiguous promotion, no insufficient); registration **8/8 within 2mm/1°** unchanged; rejection safety preserved (23/23 paired). Unknown-square burden reduced (mean 2.498→3.067→0.753). Scans 187/0/13→190/0/10→200/0/0.
- **R:** reload/determinism on feasible subset (1×2, seed 9401) identical normalized outputs (SHA256 `7deba606...704d5`), 2/2 resolved. Harnessed sample (4×5, seed 9401, rc 0): 20/20, 0 false/abstain, peak RSS ~3.47 GiB, tctl 48.25→55.625°C max, no throttling noted; throughput 292.0s/200 self-occ vs 209.8s no-self (receipt-derived, full flow). Bounded/representative samples; extended soak not run.
- **Decision:** accept C1 frozen/geometry path for (c) in measured sim envelope with recorded receipts; maintain 0-false-confirmation safety. Limitation: bounded R sampling and sim-only occlusion/lighting.

### (a) Opponent-move change detection robust to occlusion and shadows — **Measured frozen/calibrated-geometry decision: accepted (sim envelope, with characterization limits)**
- **Candidate A1:** self-occlusion-aware perception/fusion + existing legal-successor move inference (no trained change model in this closure).
- **Evidence (paired, controlled):** 0 false confirmations; 190/200→200/200 resolved; insufficient abstentions eliminated in sample (crowded 95→100/100, sparse 94→99/100), unknown-cause coverage failures sharply reduced. Residual abstention is safe `ambiguous` promotion (type not observable from occupancy/colour), not occlusion/coverage failure.
- **Robustness:** protocol includes randomized scene lighting/placement and crowded/sparse/occluded cases; dedicated arm-over-changed-square and cast-shadow stress slices not exhaustively isolated beyond receipt analysis. No false confirmations observed.
- **Decision:** accept A1 frozen/geometry path for (a) in measured sim envelope with receipts and 0-false-confirmation safety. Limitation: dedicated shadow/occlusion stress characterization beyond paired protocol remains a follow-up if tighter robustness bounds are required.
- **Fallback path (if budgets/robustness later found insufficient):** scoped local learned board-state/change model (A2) on bc250-cachy with dataset manifests, train/reload/held-out/resource/thermal evidence vs A1.

### (b) Per-piece type plus in-square position/orientation, including knight yaw — **Measured: frozen B1 geometry path (given type hypothesis) evaluated; rejected as sufficient for full 11c(b). Exit-compliant rejection + documented fallback**
- **Candidate B1:** `chess_sim.piece_orientation.fit_mesh_axis` (known-mesh render-compare, 6 calibrated wrist views, axis mod π, self-occlusion masks), given type/mesh hypothesis and expected XY seed. Does **not** perform type identification.
- **Held-out eval (bc250-cachy, 24 cases, 4/type, seed 11401, self-occ, printed v5):** 0 errors, 24 ok estimates, **3/24 accepted (12.5%)** — all knights (3/4). Accepted knight quality high: axis err mod π mean 0.513° (med 0.434°, max 0.914°), XY mean 0.614 mm (max 0.924 mm), conf 0.99, sigma 5°, supporting 4–6 views. Runtime ~1.32 s/fit mean (1.21–1.41 s by type). Symmetric/ambiguous pieces largely rejected/abstained by uncertainty/confidence gates; some low raw-error cases not accepted.
- **Measured blockers (full 11c(b)):** (1) **No per-piece type identification** — B1 requires known type/mesh hypothesis; (2) **low acceptance/coverage** (12.5% overall) and instability on symmetric pieces under this protocol; (3) axis is mod π (jaw symmetry) and does not by itself resolve full directional semantics without type/context; (4) full type+pose+yaw (incl. knight) not demonstrated end-to-end.
- **Decision:** reject frozen B1-alone as sufficient for complete 11c(b) “per-piece type plus in-square position/orientation including knight yaw” in this envelope. **Documented fallback:** (i) scoped local type-ID + pose model/training probe on bc250-cachy (wrist-view labeled sim dataset, manifests/splits, train curve, reload, held-out confusion/type+pose metrics, throughput/memory/thermal) — e.g. small classifier/keypoint or pose model conditioned for chess pieces including knight yaw; or (ii) hybrid/operator/type-known context with B1 refinement only where acceptance/confidence gates pass (measured accurate on accepted knight fits) and explicit abstention otherwise. Follow-up execution required for end-to-end (b) capability.

## Cross-cutting notes
- **Safety:** 0 false move confirmations observed in reported C1/A1 held-out evals.
- **Budgets:** throughput/memory/thermal/reload sampled (bounded) for C1 path on bc250-cachy; no long sustained soak. Runtime for B1 measured (~1.3 s/fit). Budget acceptance for production use should be confirmed against target envelopes.
- **Comparability:** paired C1/A1 toggle controlled; baseline-vs-paired contextual due to case/rejection composition differences.
- **Promotion/slot constraint:** some generated subsets (e.g. 2×3 seed 9300) infeasible due to fixed 32-slot inventory vs promotion successors; feasible seeds/subsets used.
- **Not a physical gate:** all results are simulation-envelope measurements.

## Recommended follow-ups
1. A1: targeted shadow/arm-over-changed-square stress runs and explicit unknown-cause attribution if tighter robustness bounds needed.
2. R: extended sustained harness run(s) for throughput/RSS/thermal envelope confidence.
3. B/b: scoped local type-ID+pose training probe (or hybrid fallback) to satisfy full per-piece type + in-square position/orientation including knight yaw; validate against Phase 10 grasp/orientation needs.
4. Physical/real-occlusion validation remains a separate gate.

## Follow-ups (2026-11-11, measured)

Post-closure follow-up work (bc250-cachy, harnessed, self-occ, sim envelope) continued per direction: scoped local type-ID+pose probe iteration, additional R/A1 stress evidence, and code/doc updates. See `reports/phase11c/followup_2026-11-11.md` for receipts/metrics.

- **(b) Type-ID+pose probe:** baseline `s3t1p2_seed12101` (276/128, 350ep, reload ok, 90.45s, ~8.89 GiB, 58.125°C, harness rc 0) held-out type acc 0.664 but E2E 0/12 type-correct/accepted. Iteration `s6t2p3_seed12102` (1050/357, 350ep, reload ok, 221.05s, ~9.27 GiB, 59.375°C, harness rc 0) held-out type acc 0.468, E2E 5/18 type-correct, 1/18 accepted (knight 3 cases: 1 type-correct, 1 accepted; accepted axis err mod180 1.653°, XY 0.294 mm). Scoped baseline remains insufficient for full 11c(b) end-to-end; hybrid type-known+B1-with-abstention or larger/stronger local type+pose training remains the fallback.
- **(a)/(R) stress:** additional harnessed runs preserved 0 false confirmations with safe abstentions/rejections but showed seed/scene sensitivity and capability limits under engineered stress (`tools/benchmark_phase9.py --stress-shadow-extreme`, `--stress-arm-over-changed`): shadow-extreme 78/105 (74.29%, seed 10001) and 82/120 (68.33%, seed 10002); arm-over-changed 24/120 (20.0%, seed 10101). Mixed/soak samples included 595/600 (99.17%, seed 9701) down to 345/400 (86.25%, seed 9501); seed 9801 hit inventory constraint (`ValueError: position needs more B than the set contains`, no receipt, harness rc 1), seed 9901 recovered 348/400 (87.0%). No new long sustained soak started in this 30-min window.

