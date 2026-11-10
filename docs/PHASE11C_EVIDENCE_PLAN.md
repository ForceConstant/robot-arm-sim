# Phase 11c — Evidence plan, budgets, and protocols (draft, 2026-11-10)

**Status:** draft, working plan adopted per Brian (2026-11-10) following independent subagent review (Muse Spark 1.3, partially sound). This pre-registers hypotheses, budgets/thresholds, metrics, protocols, and exit-compliant closure requirements. Numbers below are targets/gates to measure against, not claims.

## Decisions / constraints
- Priority: **(c) → (a) → (b)**.
- Approach: **frozen/calibrated-geometry first as measured hypothesis** where it meets budgets with receipts (resource/thermal/reload as relevant); require **locally trained vision models** only on measured shortfall with dataset manifests, train/reload/held-out/resource/thermal/throughput evidence and clear budgets.
- Exit-compliant closure for every subtask: either (i) accepted measured path meeting budgets with full receipts, or (ii) specific local route **rejected with measured blocker + documented fallback**. A “no-train” closure must still satisfy 11c exit evidence expectations (including scoped training probe or measured rejection evidence where training is the implied alternative).
- Execution host: **bc250-cachy** only (CachyOS BC-250, 16c/14GB), MuJoCo CPU/EGL; record host/env/hashes in receipts. No cloud training.
- Safety invariant: **0 false move confirmations** in held-out turn evaluation is non-negotiable; abstention preferred over guessing.
- Claims scoped to measured simulation envelopes; simulated success never passes a physical gate.

## Baselines (reference)
- Phase 9 accepted envelope (heldout-oblique, 8 sessions × 25 turns = 200, seed 9300, 6-view oblique, marker registration): **188/200 = 94.0% resolved, 0 false confirmations, 13 abstained (all insufficient, crowded 15–61 unknown squares), 8/8 registration within 2 mm/1°, 16/16 invalid rejected**; rejection/occluded cases as recorded in receipt. Ablations: vertical-only 72.5%, 8-pose 92.0%, drop ref-matching votes 39.5%.
- Occupancy/colour: 32-unit NumPy MLP (34 patch features). Move inference: legal-successor comparison (not learned).

## Common metrics (all turn/perception evals)
- **Turn/change outcomes:** resolved/total, resolution rate, **false confirmations (must be 0)**, abstentions by reason (`insufficient`/`ambiguous`/`unchanged`/`unexplained` as applicable), scans used.
- **Correctness/safety:** rejection/invalid cases safe (no false confirmation), occluded/insufficient cases correctly abstained vs incorrectly resolved (paired per-case analysis where possible).
- **Coverage/observability:** per-turn and aggregate unknown-square count/distribution, views-per-square, answered squares, per-view/per-square masking/validity stats when available.
- **Registration:** sessions registered, XY error (mm) and yaw error (deg), pass vs ≤2 mm and ≤1°.
- **By-bucket:** crowded vs sparse, quiet/capture/promotion/castle/invalid/occluded, lighting/shadow variants (when included).
- **Protocol comparability:** identical seeds/sessions/turns/poses/model/calibration/assets/method; record all hashes/configs.

## Resource/reload evidence (R) — required for accepted paths
- **Determinism/reload:** cold load reproduces held-out inference outputs on a fixed eval subset (hash/bitwise or documented numeric tolerance with rationale).
- **Performance:** sustained throughput (s/turn or cases/s) over extended run; peak RSS (and VRAM if used).
- **Thermal/power envelope (bc250-cachy):** CPU/SoC temps, throttling/guard trips, duration; stay within accepted/observed safe envelope and document.
- **Artifact manifest:** code commit/hash, model/checkpoint hashes, config/plan/calibration hashes, dataset manifest hashes, environment summary.

## (c) Wrist-only board registration under real occlusion
**Question:** Does self-occlusion-masked perception/fusion (geometric arm silhouette exclusion) improve or maintain Phase 9 budgets under arm occlusion without increasing false confirmations or harmful abstentions?

**Candidate C1 (frozen/geometry):** marker registration + `self_occlusion`-masked wrist occupancy/colour + median fusion + existing move inference (6-pose oblique plan, existing wrist MLP/calibration).

**Primary budgets (to confirm/falsify vs accepted 94.0% baseline):**
- False confirmations: **0**.
- Resolution: **≥ 94.0%** on paired 200-turn protocol preferred; any statistically meaningful gain at 0 false confirmations is positive. A drop requires explanation (coverage loss) and may fail acceptance.
- Abstentions: not worse in a harmful way; increased abstention acceptable only if it replaces would-be errors/unsafe resolutions and preserves 0 false confirmations (paired case analysis).
- Registration: **8/8 sessions within 2 mm XY and 1° yaw** (consistent with baseline).
- Invalid/rejection safety: no regression from baseline behavior.
- R: reload deterministic on subset; throughput/thermal/RSS recorded on extended run.

**Protocol C1:** paired 200-turn Phase 9 held-out (8 sessions × 25 turns, seed 9300, 6-view oblique, crowded+sparse as in baseline) **with vs without** `--self-occlusion` on identical protocol/assets; report deltas, per-category, unknown-square distributions. Extend with shadow/lighting stress and cases where arm occludes changed squares if needed.

**Decision:** accept frozen/geometry C1 if budgets met with receipts and R evidence (plus exit-compliant closure note). If resolution/coverage materially harmed or budgets unmet: adjust masking/plan/model (scoped) or pursue training probe / measured rejection+fallback.

## (a) Opponent-move change detection robust to occlusion and shadows
**Question:** Given (c)’s observation pipeline, can legal-successor move inference achieve robust, safe change detection under occlusion/shadows?

**Candidate A1 (frozen+geometry/perception):** self-occlusion-aware occupancy/colour + existing `move_inference` (legal-successor comparison, abstain on ambiguity/insufficient/unexplained). Measured from C1 paired runs, sliced by crowded/occluded/shadow buckets and by reason for unknowns (arm shadow vs piece/scene vs truly occluded).

**Budgets:** 0 false confirmations; maximize correct resolutions while keeping abstentions correct/safe; quantify abstention correctness (unavoidable insufficient vs avoidable coverage loss). Compare against 94.0% baseline and C1 deltas.

**Candidate A2 (trained, if A1 insufficient):** small local learned board-state/change model on simulator-generated, contract-aligned labeled wrist data (session/layout-split, manifests, no oracle in inputs). Requires train curve, exact reload, held-out metrics, throughput/memory/thermal, and comparison vs A1. Reject with measured blocker+fallback if infeasible on bc250-cachy.

**Stress slices:** crowded 32-piece, arm-over-changed-square, shadow/low-angle lighting variants, quiet/capture/special-move categories.

## (b) Per-piece type + in-square position/orientation incl. knight yaw
**Question:** Can we locally determine per-piece type and in-square position/orientation (incl. knight yaw) within Phase 10 grasp/orientation budgets?

**Candidate B1 (frozen/geometry):** `piece_orientation.py` known-mesh render-compare fit from ≥3 calibrated views (axis mod π, explicit abstention) for in-square XY/yaw given type/mesh hypothesis and calibrated registration/camera.

**Budgets/metrics:** axis/yaw error (deg, mod π as applicable), in-square XY error (mm), acceptance rate, abstention correctness/calibration (uncertainty/confidence), supporting-view behavior, runtime s/fit and memory on bc250-cachy; evaluated across piece types/poses (esp. knights), occlusion/crowding, and registration noise within Phase 9 envelope.

**Candidate B2 (type/pose, if needed):** joint type identification + pose. Measure type confusion matrix on held-out wrist views (knight/bishop/pawn confusions, occlusion) and conditional yaw/XY given correct vs hypothesized type. If B1 insufficient, evaluate/train small local keypoint/pose/type model on labeled sim data with full evidence.

**Dependency note:** (b) depends on calibrated views/registration from (c) and Phase 10 grasp contracts; end-to-end usefulness judged against Phase 10 needs, but 11c closes on vision evidence/budgets.

## Evidence artifacts per run
- JSON receipt/report: config, seeds, hashes (code/assets/models/calibration/plan), host/env, protocol, raw counts, by-category metrics, per-case records where appropriate, resource/thermal summary.
- Dataset manifest (if generated): generator script+args+seed, counts, splits, label schema, hashes.
- Training (if any): loss/metric curves, checkpoints+hashes, reload check, final/selected model eval.
- Summary doc/update in `docs/PHASE11C_*` and `reports/phase11c_*`; update `docs/ROADMAP.md` on completion.

## Immediate execution queue
1. **C1/A1 paired baseline:** collect/analyze `heldout-oblique-selfocc-s8t25` vs `heldout-oblique-noselfocc-s8t25` (seed 9300) vs Phase 9 oblique baseline; produce comparison receipt + paired deltas.
2. **R harness:** record reload determinism, throughput, RSS, temps for C1 eval on bc250-cachy (extended run or repeated subset).
3. **C1 stress/A1 slices:** shadow/crowding/arm-over-changed-square if paired baseline shows benefit or to characterize failures.
4. **B1 eval:** design held-out orientation/type dataset/protocol and measure `piece_orientation.py` (yaw/XY/runtime/abstention) on bc250-cachy.
5. **A2/B2 probes or measured rejections** only if/where frozen paths show measured shortfall; document blocker+fallback.
6. **Closeout:** 11c receipt, roadmap update, final status.
