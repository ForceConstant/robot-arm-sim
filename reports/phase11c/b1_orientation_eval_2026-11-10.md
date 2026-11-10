# Phase 11c B1 orientation eval (frozen/geometry, given type hypothesis) — 2026-11-10

**Host:** bc250-cachy (CachyOS BC-250, 16c/14GB), MuJoCo CPU/EGL.  
**Env:** `/home/brian/robot-arm-sim/.venv/bin/python` 3.11.18, `MUJOCO_GL=egl PYTHONPATH=. ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5`, cwd `/home/brian/robot-arm-sim-phase11c`.  
**Script:** `tools/b1_orientation_eval.py` (11c). Uses measured/parked joints for wrist capture, registration, `SelfOcclusionModel` masks, and cameras.  
**Candidate:** `chess_sim.piece_orientation.fit_mesh_axis` (known-mesh render-compare, 6 calibrated wrist views from scan plan, axis mod π, self-occlusion masks on). **Given type/mesh hypothesis and expected square/XY seed** (per API); does **not** perform type identification.  
**Protocol:** 4 samples/type × 6 types = 24 cases, seed 11401, randomized board/light/placement per case (registered marker pose), one white piece per case at random in-square offset/yaw on printed v5 assets. Truth axis derived from simulated body yaw (mod π) for comparison; XY truth from piece position.  
**Receipt:** `.private/phase11c/b1_orientation_s4_seed11401.json` (log `.log`), elapsed 128.734 s.

## Overall
| Metric | Value |
|---|---:|
| Cases / errors / ok estimates | 24 / 0 / 24 |
| Accepted | 3 (12.5% of ok) |
| Runtime per fit | mean 1.319 s, median 1.354 s, p90 1.406 s, max 1.413 s |
| Axis error mod 180° (accepted, n=3) | mean 0.513°, median 0.434°, max 0.914° |
| XY error (accepted, n=3) | mean 0.614 mm, median 0.828 mm, max 0.924 mm |

**Acceptance gate (estimator):** sigma ≤ 15°, confidence ≥ 0.7, supporting_views ≥ 3 (see `AxisEstimate.acceptable()`).

## By type

| Type | ok | accepted | abstained/rejected | Runtime mean (s) | Axis err accepted (° mean/med/max) | Axis err all ok (° mean/med/max) | XY err accepted (mm mean/med/max) | Notes |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| pawn | 4 | 0 | 4 | 1.235 | — | 65.044 / 66.512 / 88.616 | — | High raw axis errors; no accepted fits |
| rook | 4 | 0 | 4 | 1.213 | — | 20.417 / 4.594 / 72.465 | — | Some low raw errors (0.016°, 2.585°) but rejected by uncertainty/confidence gates; one large outlier |
| knight | 4 | 3 | 1 | 1.407 | 0.513 / 0.434 / 0.914 | 0.595 / 0.638 / 0.914 | 0.614 / 0.828 / 0.924 | **Best case:** accepted fits very accurate (conf 0.99, sigma 5°, supporting 4–6). 1 case abstained (axis 0.842° but sigma 80°, conf 0.065, supporting 3) — safe abstention despite low geometric error |
| bishop | 4 | 0 | 4 | 1.356 | — | 19.379 / 22.078 / 32.879 | — | Mixed raw errors incl. 0.480° case not accepted (gate/confidence behavior) |
| queen | 4 | 0 | 4 | 1.389 | — | 65.509 / 71.994 / 81.297 | — | Large raw errors |
| king | 4 | 0 | 4 | 1.313 | — | 47.901 / 60.011 / 71.536 | — | Includes 0.047° raw but overall unstable/unaccepted |

## Interpretation (measured, bounded sample)

- **When accepted, quality is high:** sub-degree axis (mod π) and sub-mm XY errors on accepted knight fits, with strong confidence/support — indicates the render-compare geometry path can be accurate given sufficient view evidence and appearance discriminability.
- **Acceptance/coverage is the limiting factor:** only 3/24 cases accepted overall (all knights). Many symmetric/ambiguous pieces yield high uncertainty, low confidence, or inconsistent support under this protocol/asset/lighting/view setup, so the estimator (appropriately) abstains or fails acceptance gates. Some low-raw-error cases (e.g. rook/bishop/king instances) were not accepted, indicating confidence/consistency calibration vs geometric residual is a key issue.
- **Knight yaw (mod π) signal:** promising for the asymmetric knight in this sample when accepted (relevant to Phase 10/b), but sample is small (n=4) and one knight abstained.
- **Runtime:** ~1.21–1.41 s/fit mean by type on BC-250 CPU in this setup (6-view render-compare search), feasible as a bounded per-target measurement but must be budgeted against phase throughput/thermal envelopes for repeated use.
- **Scope gap for full 11c(b):** B1 is **given type/mesh hypothesis** (and seeded near expected XY). Full subtask (b) requires **per-piece type plus in-square position/orientation including knight yaw**. Type identification (and joint type+pose robustness across pieces/occlusion) is **not evidenced** by this run and remains a measured gap requiring either a local training/identification path or a specific measured rejection + documented fallback.

## Evidence status
- **Positive:** B1 candidate is locally runnable/measurable on bc250-cachy with recorded runtime; accurate when accepted (esp. knight) and abstains rather than forcing low-confidence fits in many cases.
- **Not sufficient for 11c(b) closure:** low overall acceptance, poor/unstable behavior on symmetric pieces, no type-ID, and no held-out budgeted acceptance against Phase 10 manipulation needs at scale. Requires either (i) improved/calibrated geometry path + type-ID meeting budgets with full R/held-out evidence, or (ii) scoped local training probe(s), or (iii) **specific local route rejected with measured blocker and documented fallback** per 11c exit contract.

## Artifacts
- bc250-cachy: `.private/phase11c/b1_orientation_s4_seed11401.json`, `.private/phase11c/b1_orientation_s4_seed11401.log`, `tools/b1_orientation_eval.py`
