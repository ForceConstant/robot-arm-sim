# WristScanner IRL migration — stage 1 (2026-11-25)

Branch `irl-wrist-scanner`, merged at `ebc86e3` + `dab5dfc`. Code-only stage:
no arm motion, no HTTP wiring, no merge of the live controller.

## Why

The IRL app has no full-board classifier; it runs `TargetDetector` only at the
aim. The sim wrist path is `chess_sim/wrist_scan.py` + `model_wrist.json`. The
wrist model had been trained with per-pose empty-board references, which a live
game cannot provide, and the committed model predated the ChArUco face and the
47° wrist camera.

## A/B (seed 9300, 2 sessions × 6 turns, same frames, thresholds unchanged)

| model / references | acc(labelled) | acc(all) | unknown | confident err | turns | reg |
|---|---|---|---|---|---|---|
| old 11-03 model, refs | 0.6062 | 0.3849 | 0.3651 | 304 (25.0%) | 0/12 | 2/2 |
| old 11-03 model, no refs | 0.6048 | 0.3750 | 0.3799 | 298 (24.5%) | 0/12 | 2/2 |
| retrained, refs, refs | 1.0000 | 0.5715 | 0.4285 | 0 (0%) | 4/12 | 2/2 |
| retrained, refs, no refs | 0.8152 | 0.3882 | 0.5238 | 107 (8.8%) | 0/12 | 2/2 |
| **retrained reference-free, no refs (shipped)** | **0.9309** | **0.5206** | **0.4408** | **47 (3.9%)** | 1/12 | 2/2 |

Shipped model metadata (`model_wrist.json`): references=false, threshold 0.4,
answered 1237/3840, coverage 0.322, answered accuracy 0.9838, confident error
rate 0.0052. References are still worth ~7 accuracy points when an empty epoch
exists; the reference-free model is the only one usable in a live game.

## Changes

- `chess_sim/fiducials.py`: the least-squares start is bounded around the best
  seed, not around yaw 0. A board set down at yaw ≈ π was clipped to ±16° and
  every registration gate rejected a good fusion (246 px rms on views that
  project at 0.8 px). Yaw is wrapped to (−π, π].
- `tools/benchmark_phase9.py`: uses the ChArUco face + playing-surface height
  when the board prints it; per-square scoring against sim truth; references
  are opt-in (`--with-references`) because the shipped model is trained without.
- `tools/train_perception_wrist.py`: `--no-references`.
- `live/wrist_scan_runner.py`: pure adapter (no HTTP/motion/bridge) from
  live_config calibration + plan + model to `WristScanner.scan(references=None)`;
  JSON-safe `summarize()`.
- `tests/test_wrist_scan_runner.py`: two plan poses, one position, asserts 64
  predictions, registration ≤ 2 mm/1°, and no occupied square labelled empty
  above threshold; ~5.6 s; skips without the parallel-jaw tool and printed pack.

## Verification

- Full suite (default env): **264 tests, 0 failed, 43 skipped**.
- Product env: 240 passed / 22 skipped / 2 failed, both reproducing on main
  (legacy dual-arm fixture, pinned jaw centroids) — config-driven, not branch.

## Open blockers

1. **The scan plan is stale.** 5 of 6 poses exceed the tightened `wrist_roll`
   range (±78.8°). The benchmark used a roll-clamped copy in `/tmp`; a proper
   replan (`tools/plan_wrist_scan.py` against the live layout/limits) is a
   separate task and must land before any real scan.
2. **Real-frame qualification not done.** The model is sim-trained; no real
   labelled frames exist yet. The adapter is not wired into `live/app.py`.
3. **Coverage is modest** (0.32 answered on validation). The safety gate (no
   occupied square called empty above threshold) holds on the sim sample, not
   yet on real frames.
