# Phase 11c — vision-model training path (plan & evidence scaffold)

**Status:** in progress (planning/assessment). This document tracks 11c scope, baselines, candidate approaches, required evidence, and open questions. No completion is claimed.

## Scope (ROADMAP)
Choose and train a candidate for the worst case, or record a measured frozen-model + geometry decision meeting the same budgets:
- **(a)** Opponent-move change detection robust to occlusion and shadows.
- **(b)** Per-piece type plus in-square position/orientation, including knight yaw, as required by Phase 10.
- **(c)** Wrist-only board registration under real occlusion (Phase 9).

**Exit:** 11b and 11c must have locally runnable training paths for control and vision with throughput, memory, thermal, reload and held-out evidence — or a specific local route rejected with a measured blocker and a documented fallback. Local services only (BC-250, HTPC GTX 1060, CPU). Cloud training out of scope.

## Baselines (floors)
- **Registration (c):** geometric marker-based board pose from wrist views (`fiducials.py`, `wrist_scan.py`); Phase 9 held-out: 8/8 registration within 2mm/1°; turn resolution **94.0%** (188/200) on 6-pose oblique plan, 0 false confirmations, 13 `insufficient` abstentions on crowded/occluded boards (15–61 unknown squares). Self-model occlusion queued as refinement.
- **Occupancy/colour:** 32-unit NumPy MLP over 34 patch features, trained locally on CPU (`perception.py`, wrist model asset). Natural arm occlusion handled poorly before explicit self-occlusion masking.
- **Move inference (a):** `move_inference.py` legal-successor comparison (not learned); abstains on ambiguity/insufficient/unexplained. Coverage limited by unknown squares.
- **Piece type/orientation (b):** no current product path for wrist-view per-piece type + in-square XY/yaw; Phase 10 requires reliable grasp localization including knight yaw.

## Relevant local WIP (main repo, copied/symlinked into 11c worktree for evaluation)
- `chess_sim/self_occlusion.py` + `chess_sim/wrist_scan.py` integration: projects authored arm geometry from measured joints (no sim camera/body truth) to conservatively mask arm-covered square patches so they abstain instead of voting. Unit tests pass.
- `chess_sim/piece_orientation.py`: known-mesh yaw/XY fit from ≥3 calibrated RGB views via render-compare (MuJoCo) with explicit uncertainty/confidence/abstention; no simulation state/pose oracle input. Axis modulo π. Requires known piece mesh/type hypothesis. Unit tests pass.
- `chess_sim/vision.py`, `chess_sim/vision_geometry.py`, benchmark/test diffs: off-center/oblique detection and localization improvements relevant to (b).

## Candidate approaches (to measure)
### (a) Change detection robust to occlusion/shadows
- **A1 (frozen/geometry-informed):** self-occlusion-masked wrist perception + existing legal-successor inference; measure delta vs Phase 9 94% baseline under same/extended held-out protocol (occlusion/shadow/crowding). If budgets/targets met, record measured decision.
- **A2 (trained):** train/refresh wrist occupancy+colour model with self-occlusion-aware labels or a learned board-state/change model; compare held-out resolution, false confirmations, abstention behavior, throughput/memory/thermal/reload.

### (b) Piece type + in-square position/orientation (knight yaw)
- **B1 (frozen/geometry):** known-geometry mesh fitting (`piece_orientation.py`) for yaw/in-square XY given calibrated views and type/mesh; pair with type source (task-known/geometry hypothesis or classifier). Evaluate axis/yaw error, XY error, acceptance rate, abstention correctness across piece types/poses/crowding/occlusion.
- **B2 (trained):** local vision model for per-square/piece type and/or keypoints/orientation (e.g. lightweight CNN/keypoint regressor or fine-tuned small detector/segmenter) on simulator-generated labeled wrist data. Requires dataset contract, labels, and full train/reload/eval evidence.

### (c) Registration under real occlusion
- **C1 (frozen/geometry):** marker-based registration already strong (0.8mm/0.09° worst in Phase 9); quantify robustness when arm/self-occlusion present and with self-occlusion masking in perception pipeline. Registration itself is geometric; "under real occlusion" interacts with usable observations.
- **C2 (trained/assisted):** learned registration aid only if C1/floor fails budgets; likely lower priority given marker results.

## Evidence requirements (per candidate/run)
- Dataset/corpus: generation script/config, seeds, splits, label definitions, counts, hashes/manifests (private paths recorded, no private assets exported improperly).
- Training (if any): pinned env/model, params, steps/epochs, batch, device, samples/s, wall time, peak RSS/VRAM, temps/thermal/throttle, guard behavior.
- Artifacts: checkpoint + config hashes, **exact reload verification** (bit-identical or documented tolerance).
- Held-out eval: protocol, metrics vs budgets, per-case/bucket breakdown, false confirmations/errors, abstentions/rejections, comparison to baseline/floor.
- Rejection receipts: measured blocker (fit/throughput/thermal/quality/missing labels/dependency) and documented fallback.

## Open questions / dependencies
1. **Repo sync/WIP:** Work in `../robot-arm-sim-phase11c` (from origin/main). Incorporate main-repo uncommitted vision WIP (`self_occlusion`, `piece_orientation`, wrist/vision diffs) as candidate baseline? Need Brian’s direction.
2. **Priority:** Order of (a)/(b)/(c) and preference for measured frozen+geometry decisions vs trained models where acceptable.
3. **Data/assets:** Use `.private` gripper/piece assets and Phase 9/10 receipts/plans/calibration as source of truth; generate simulator datasets under Phase 9/10 contracts as needed?
4. **Compute limits:** GTX 1060 3GB (accepted 79–80°C envelope) and CPU available; any time/thermal constraints today?
5. **Phase 10 dependency:** Phase 10 complete per origin/main (owner-accepted). Confirm for 11c gating.

## Immediate next steps (feasible now)
- Run vision-related tests in 11c worktree and fix regressions if introduced by integrated WIP.
- Inventory existing Phase 9 held-outs/models/plans/calibration and define comparable eval protocols for A1/C1.
- Prototype small, bounded data-generation/eval for (b) orientation/type using simulator + existing assets (measure feasibility, throughput, labels).
- Identify smallest evidence-producing candidate per (a,b,c) and execute with receipts.
