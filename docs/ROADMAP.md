# Simulation and training roadmap

Reconciled 2026-10-29 against the current source, live release, training scripts,
and experiment log. Priority 1, **make crowded manipulation repeatable**, and
Priority 2, **measure the scripted-control baseline**, are complete on live
v1.10.6/v1.10.7; measured results are in [`EXPERIMENTS.md`](EXPERIMENTS.md) and
CAD/import assumptions are kept with the private tool pack.

## Baseline — what is already working

- **Live simulator v1.10.6:** latest Onshape pack source revision
  `4dca0986f24304c301118aa4`, with **0–38 mm** aperture. Fixed `crowded-v3`
  passed **25/25 in 220 s**; three bounded randomized repeats passed **75/75**
  (seeds 271828, 314159, 161803). This is not proof of full-game or physical
  reliability.
- Browser controls, keyboard per-joint jogging, two-arm joint readouts, camera
  views, incident capture/replay, and suite-exclusive runner control are
  implemented and were verified on the live build.
- The physical-grasp path uses contacts and reports real failures; it does not
  use welds or teleportation.
- The scripted episode/dataset pipeline and LeRobot policy runner exist. The
  MolmoAct2 baseline and DAgger checkpoint are evaluated; their paired
  300-position scores were 227/300 and 233/300 respectively. See the experiment
  log for task-family breakdowns and known evaluation limits.
- **Scripted-expert baseline measured (2026-10-29):** 332/360 (**92.2%**, 95% CI
  89.0–94.6%) over three seeds, board ±5 mm per axis and arm start ±0.1 rad,
  with the vendored and deployed printed piece sets. Moves 229/240 (95%),
  captures 103/120 (86%). The vendored set scored 175/180 (97%) and the printed
  set 157/180 (87%); the gap is captures (98% vs 73%) concentrated in printed
  knights (7/14) and pawns (20/34). All 7 wrong-piece picks, 8 still-on-board
  captures and 6 falls were on the printed set. Details in
  [`EXPERIMENTS.md`](EXPERIMENTS.md) §11; `tools/benchmark_expert.py` reruns it.
- Latest Onshape revision `4dca0986f24304c301118aa4` is captured and imported
  into private pack v3: healthy assembly, three moving mates, one fixed camera
  mount, 19 mm travel per jaw, unchanged jaw/gear meshes, and updated base/mount
  meshes. `SIM_CAMERA_CENTER` is connector-derived and agrees with the
  simulator's `(-60, +80, 0) mm` camera offset. The custom model measured a
  **0–38 mm** aperture in its integration test and is live-qualified for the
  simulated task suite; physical fit remains unmeasured.
- Latest custom CAD+printed local regression: **28 passed**. A targeted failure-injection case
  now returns the pawn upright using 1.5 mm extra physical closure only on the
  emergency source-return. A dual-arm trace found the 0.5 mm ordinary preload
  let a pawn slip during the final carry segment; increasing the real grasp
  squeeze to 2 mm made the official dual-arm lift/place test pass. Neither fix
  relaxes success criteria or changes physical-contact handling.
- Post-deploy v1.10.6 browser checks passed on the GTX 1060: both arms' 12 live joint
  readouts, keyboard jog, automated motion, four camera JPEGs, start/pause/
  resume/stop/reset, sensor selection, CSRF guard, and mobile layout. The live
  simulator is healthy and ready; the last seeded suite is complete. The profile still labels the CAD grasp model
  uncalibrated; fixed and seeded suite success is not hardware qualification.

## Priority 0 — qualify the imported CAD tool

1. **Deployment and simulated-suite gates are complete:** live v1.10.6 reports the
   v3 source revision and 0–38 mm aperture; local tests pass 28/28, browser
   checks pass, live crowded-v3 passes 25/25, and bounded seeded runs pass
   75/75. Keep local v2 as rollback and retain the prior pack artifact. Do not
   silently use either earlier pack's
   different aperture calibration.
2. Sampled v3 collision coverage is complete against exact Onshape source
   microversion `4dca0986f24304c301118aa4`: 0/25,256 base, 0/14,978 jaw,
   0/17,426 gear, and 0/66,410 camera-mount surface samples lie outside their
   hull unions by more than 10 µm; random bbox probes missed 0 source-solid
   points for every part. Base, jaw, and camera mount reached the 128-hull
   CoACD cap (jaw also has its explicit finger envelope). The gear's one-hull
   envelope fills 1,497/3,290 empty bbox probes (conservative tooth
   valley overfill). This is sampled coverage, not a Hausdorff proof. Physical
   camera-mount clearance remains unverified.
3. Latest CAD rack sweep covers 0–38 mm: 19 mm travel per jaw; continuous
   engagement interval supports 25.475 mm per side, leaving about 6.475 mm
   before the one-pitch open-end reserve; sampled base/jaw overlap is 0 mm³.
   Use the tooth-derived mapping: 4.15 mm pitch × 18 teeth = 74.7 mm/rev.
   The 25 mm/rev Onshape rack relation is a placeholder, not a conflicting
   design value. Physical fit/backlash still needs measurement before
   hardware qualification.
4. New-build gates are complete: v1.10.6 live, browser camera/control checks
   pass, and live crowded-v3 passed 25/25 plus 75/75 seeded cases. Preserve the
   rollback pack; this does not qualify the mechanism physically.

## Priority 1 — make crowded manipulation repeatable

1. **Complete — knight transit/IK failure triaged by recapturing the route.**
   Historical recording `f4cc5169` remains unavailable, so no claim is made
   about that exact trace. A seeded `g8f6` recapture picked and placed the
   knight with precise waypoints but displaced `f7`; it was not an unreachable
   target or missed grasp/slip. Contact capture shows the right CAD jaw touched
   `f7` during destination `place_descend` and `release` (peak 0.122 N; 1.49 mm
   penetration), displacing the pawn by 20.2 mm. Same-case controlled replay
   passed when board shift was reduced from the failing 7–9 mm offsets to 5 mm,
   with arm-start jitter unchanged. This confirms destination-side
   gripper-envelope collision for the recaptured failure and sets the supported
   randomized board-shift range; see [`EXPERIMENTS.md`](EXPERIMENTS.md) and the
   private incident ZIP.
2. **Complete — expanded physical dual-arm regressions.** Local regression
   covers pawn, knight, and bishop transfers on both arms with actual contact
   grasps, measured lift, correct placement, and neighbor-displacement checks.
   The fixed live suite retains all 24 `crowded-v2` anchors and adds the black
   corner knight (`crowded-v3`, 25 cases); it passed 25/25.
3. **Complete — repeated bounded perturbation trials.** Three independent
   live `crowded-v3` runs passed 25/25 each (75/75 total), with per-case CRC32
   XOR seeds, actual board/start layouts, and receipts for seeds 271828, 314159,
   and 161803. Supported perturbations are board origin ±5 mm per axis and
   arm start ±0.1 rad; the wider board shift was rejected after controlled
   failure reproduction.
4. **Complete — failure diagnostics and evidence retention.** The contact-body
   classification bug is fixed; raw contacts remain time/phase-aligned with
   commanded/achieved pose, jaw gap, force, and neighbor-motion trace. Only
   phase-qualified non-target piece contacts become risk candidates. Incident
   ZIP routing now works, and seeded verification immediately archives failed
   recordings before bounded recorder retention can prune them.
5. **Complete — grasp and clearance review against the imported v3 tool.**
   Printed-piece grasp heights and openings are recorded in
   [`GRASP_PRINTED.md`](GRASP_PRINTED.md). Seeded A/B raised the printed-knight
   closure allowance from 7 to 9 mm to prevent the corner-knight topple; it is
   per-knight, not a global threshold. The tool's modeled 0–38 mm aperture and
   finger-envelope checks pass the fixed and seeded suites. Physical clearance,
   friction, and safe squeeze force remain outside this simulation gate.

## Priority 2 — measure the scripted-control baseline — complete

1. **Complete — randomized-placement and piece-set baseline.**
   `tools/benchmark_expert.py` (core: `chess_sim/benchmark.py`) scores the
expert over fresh `PositionConfig` draws (2–8 pieces besides the kings), board
origin ±5 mm per axis, arm start ±0.1 rad, and both the vendored `default` set
and the deployed `printed` pack v5, across seeds 271828, 314159 and 161803.
Every verdict comes from `env.evaluate`; the task/piece metadata used to slice
the report is reporting-only and is never available to a policy under test.
2. **Complete — measured result.** 332/360 (92.2%, 95% CI 89.0–94.6%), median
   placement error 1.5 mm, named piece 353/360, 2 neighbour disturbances. Moves
   229/240 (95%); captures 103/120 (86%). By set: default 175/180 (97%), printed
   157/180 (87%). The difference is captures (98% default vs 73% printed),
   concentrated in printed knights (7/14, 50%) and pawns (20/34, 59%). Every
   wrong-piece pick (7), still-on-board capture (8) and fall (6) was on the
   printed set.
3. **Complete — reported per task, per piece, per set, per seed.** The report
   carries per-task (source→target), per-piece-type, per-piece-set, per-seed and
   cross-tabulated tables with Wilson intervals and failure reasons; raw
   per-episode receipts are retained in `.private/expert-baseline.json`. Per-seed
   scores were 90/95/92% with overlapping intervals, so seed differences are not
   evidence. Recorded in [`EXPERIMENTS.md`](EXPERIMENTS.md) §11.
4. **Scope limit.** This is a truth-driven, simulated ceiling on tasks the
   expert's own reach/clearance check calls executable — not all legal moves,
   not the wider board-turn envelope, and not hardware or physical-fit
   qualification. Compare a learned policy only against the matching piece set
   and task family.

## Priority 3 — make verified incident corrections reusable — complete

- `examples/export_incident_corrections.py` exports only linked incident
  corrections whose source is a recorded failure; by default the correction
  itself must have a successful simulator verdict. Optional failed correction
  attempts retain their actual failure labels. Episodes contain the executed
  six-joint state/action transitions and task instruction; the adjacent
  `incident-corrections.jsonl` preserves `correction_of`, source/runtime
  fingerprint, both incident IDs, and actual source/correction outcome labels.
- Incident manifests now store the runtime source fingerprint at capture time,
  including deployed builds without a git checkout. The exporter rejects
  missing links, non-failed source incidents, absent fingerprints, and malformed
  transitions instead of guessing. Existing scripted dataset collection is
  not treated as a policy-failure correction set.
- Scope: current incident traces are proprioceptive (joint-state) observations;
  they do not contain synchronized camera frames. These are reusable state/action
  corrections, not visual-policy examples. Full visual DAgger capture remains
  future work.

## Priority 4 — camera-targeted pickup of off-center pieces

- **Status: complete for the qualified fixed-overhead simulation envelope; no
  learned-policy training is required.** The task may name the source square
  and piece type. General piece/color recognition and learned occupancy remain
  Priority 6; independent physical camera/tool calibration and hardware trials
  remain sim-to-real work.
- **Implementation:** `chess_sim/vision_geometry.py` maps RGB pixel locations
  through the model camera intrinsics/extrinsics and shared board/arm world frame
  to the piece's base plane, propagating both pixel and visible-height parallax
  uncertainty. `chess_sim/vision.py` detects a task-selected piece against an
  empty-scene reference and supports an exclusion mask. The environment can
  render object-ID segmentation; the benchmark masks robot geometry before
  re-observing so arm pixels cannot masquerade as the piece. The controller
  accepts observed source XY, returns to the parked observation pose, and
  abstains on missing/uncertain or >4 mm inconsistent re-observations. Simulator
  truth is used only to create controlled cases and score outcomes, never as a
  detector/controller input.
- **Qualified envelope and evidence (2026-10-29):** two fixed-rig held-out
  sweeps (seeds 20261029 and 412829; 36 total cases) cover all six chess-piece
  types, eight source layouts, centered and ±4/±6 mm offsets, varied yaw, and
  board translation/side variation up to ±5 mm plus yaw up to ±5°. There were 28
  detections / 8 safe initial abstentions; initial localization error averaged
  1.24 mm (max 4.95 mm). Re-observation error averaged 1.29 mm (max 5.23 mm).
  Of the 28 accepted detections, 21 complete moves succeeded; the 7 movement
  failures were separately recorded at lift (5) or place descent (2). There were
  zero false targets and zero collateral displacements. The max-error king case
  still completed successfully. Receipts: `.private/vision-rig-heldout-24.json`
  and `.private/vision-rig-heldout-seed2.json`.
- **View-limit evaluation:** a separate 12-case randomized-camera sweep (seed
  912407) detected 9 cases, completed 6 moves, and recorded zero collateral
  displacements. Initial error reached 5.30 mm and re-observation could differ
  by as much as 27.93 mm; the controller safely rejected the inconsistent
  observation. Randomized oblique views are therefore explicitly outside the
  supported envelope, not qualified by the fixed-rig result. Receipt:
  `.private/vision-random-height-uncertainty.json`.
- **Calibration boundary:** simulated intrinsics/extrinsics are taken from the
  MuJoCo camera model; the board and arm share its world frame. Plane round-trip
  is checked across five board locations, and board pose is randomized in the
  held-out sweeps. This qualifies the simulation transform chain, not an
  independently measured camera calibration or physical camera-to-tool mount.
- Local regression passed (`35 passed, 4 skipped`). Release complete as v1.10.8:
  the versioned runtime bundle/compose/checksums are published, only the
  authorized `robot_arm` service was rebuilt, the live NVIDIA renderer and
  browser controls/cameras/mobile checks passed with no page errors, and the
  live crowded-v3 suite passed 25/25. Receipts are
  `.private/post-deploy-1.10.8.json` and
  `.private/browser-check-1.10.8.json`.

## Priority 5 — establish the local platform and data boundary — complete

**Closed 2026-10-30 for bounded platform/data qualification.** The implementation
direction is **frozen pretrained vision + calibrated geometry/IK/scripted
control**, not mandatory large-network fine-tuning. BC-250 remains the primary
local ML target; GTX 1060 retains simulation. One wrist camera is the IRL input.
The largest-practical-local-model preference is preserved by the passing 35B VLM.

- **Frozen vision platform:** pinned 35B Qwen Vulkan artifacts; 20/20 strict-JSON
  paced requests previously passed. New 4/4 read-only simulator-wrist JPEG fetches
  over LAN plus inference complete in 1.445–1.498 s, peak 72 C. Physical exposure,
  frame freshness and chess recognition accuracy are not measured. Keep >=5 s
  idle, memory/thermal guards and no continuous-operation claim: unpaced runs
  still fail at 85 C.
- **Data/storage:** all 3 simulator recordings/648 frames re-audited; independent
  whole-session/layout train/test groups, private LAN-only storage and transfers,
  simulator/real provenance kept separate. No real-camera dataset is claimed.
- **Actual-task optimizer pilot:** 17,542-parameter FP32 wrist-feature/joint/task
  MLP; 32 training and 16 held-out observations; 20 AdamW updates, finite gradients,
  exact model/optimizer reload and resumed update, plus an independent-process
  inference-only artifact reload with identical held-out predictions. Guarded
  runs pass without swap growth or host changes. This is actual recording
  ingestion and GPU optimization, not the prior synthetic-only test.
- **Model rejected for control:** held-out MSE 0.01572 rad² is worse than the
  zero-delta baseline 0.002867 rad². No learned actions were sent to the arm or
  simulator. Platform success is not policy quality or grasp success.

[Acceptance, artifact hashes, metrics, scope and reproduction](../training/PRIORITY5_ACCEPTANCE.md).
The earlier demand for final trained-controller qualification is **not claimed
passed**: with large-network training no longer prerequisite to the selected
architecture, final perception quality belongs to Priority 6, state inference to 7,
optional learned control to 8, and physical readiness to 10. These remain open.
The pilot does not replace our calibrated controller or qualify MolmoAct2,
LeRobot/PyTorch training, continuous VLM service, or a physical robot.

## Priority 6 — board registration and occupancy/color perception — complete

**Closed 2026-10-30 for the qualified fixed-overhead-rig envelope.** Board
registration is treated as calibrated geometry, not a learned task. The camera
intrinsics/extrinsics, known square size and board-surface height are
calibration inputs; one overhead frame supplies the residual board pose. The
checker-saddle response gives a coarse pose, an edge-consistency term fixes the
lattice's one-square phase, and a local Nelder–Mead refinement returns origin,
yaw and square size. A low-score fit abstains rather than labelling the wrong
pixels.

Perception samples rectified per-square patches, estimates each square's empty
background from same-colour neighbours, and classifies `empty` / `white` /
`black` / `unknown` with a small NumPy MLP over colour, texture, gradient and
empty-reference difference features. A session's empty-board frame is a
calibration observation, never piece truth. Top and wrist views are both
evaluated; the wrist view uses the top-view board pose plus the calibrated
camera-to-arm chain.

- **Qualified envelope:** published overhead rig; board square/height known;
  board placement within about ±12 mm and ±2.5° of nominal (registration
  searches ±16 mm / ±4.5°); sparse positions (two kings plus 2–8 extra pieces);
  board/camera fixed within a session.
- **Held-out result** (`seed 9001`, 8 sessions × 5 positions, 5 sessions
  registered, 3 rejected by the gate): registration median 1.42 mm / max
  1.92 mm origin error, max 1.43° yaw. Top view on unoccluded squares: 96.0%
  coverage, 99.4% answered accuracy, 95.8% occupied accuracy, 99.9% empty
  accuracy, 0.55% confident errors. Natural arm occlusion: 90.6% abstention.
  Wrist: 97.8% answered accuracy, 88.2% occupied accuracy, 99.9% coverage.
  Shadow stress: 98.4% accuracy.
- **Limits:** registration accepted 5/8 held-out sessions; rejected sessions
  are reported, not silently used. Synthetic opaque occluders are only 22.5%
  abstained (natural arm occlusion is 90.6%), so a hand/foreign-object detector
  remains open. The empty reference is valid only while board and camera stay
  fixed. Camera calibration is simulated, not independently physical. Sparse
  positions only; crowded boards and move/state inference remain Priority 7/8.
  No physical capture session is claimed.
- **Implementation:** `chess_sim/board_registration.py`,
  `chess_sim/perception.py`, `chess_sim/perception_data.py`,
  `tools/train_perception.py`, `tools/benchmark_perception.py`; model artifact
  `chess_sim/assets/perception/model.json`. Local regression passed
  **67 passed, 4 skipped**. Receipt `training/PERCEPTION_ACCEPTANCE.md`, private
  receipts `.private/perception/heldout-v8.json` and `training-v8.json`.

## Priority 7 — infer move/state changes from observations

- Separately evaluate move-state inference: compare settled observations with
  legal successor states from the last confirmed position, including captures,
  ambiguous/incomplete moves, occlusion, and board changes. Preserve chess
  history/rights that cannot be recovered from an image. Require a unique,
  sufficiently confident match or an explicit abstain/ask-for-help outcome;
  occupancy classification alone is not move understanding.

## Priority 8 — reassess learned motor-policy grounding

- Reassess unseen-instruction grounding and the failed wide-distribution
  training attempts in [`EXPERIMENTS.md`](EXPERIMENTS.md). Any continuation
  needs a held-out paired evaluation and grasp-aware metrics (right piece,
  untouched/knocked/carried, success), not loss or median placement error
  alone. Keep visual targeting and safe execution gates independent of policy
  scores.

## Priority 9 — consider batched rollout/RL only if justified

- Only if the preceding baselines support it, build the planned batched
  rollout/RL loop and compare architectures on the same held-out set. Do not
  spend another long GPU run until the dataset, action labels, schedule,
  evaluator, and selected compute platform pass their preflight checks.

## Priority 10 — physical sim-to-real gates (blocked until the rig exists)

- Build and measure the physical fixture; calibrate board squares, board pose,
  arm height/mount, tray, and overhead/wrist cameras against the simulated
  config.
- Check CAD fit, rack engagement, jaw travel, servo limits/torque, camera
  clearance, and printed-part masses/friction on the bench. Current values are
  CAD or actuator estimates, not measured hardware facts.
- Run guarded calibration moves and validate camera-based success detection
  before attempting longer tasks. Log real failures with the same replayable
  evidence as simulation.

## Deferred scope

- Full legal-game support for castling, en passant, and promotion.
- Identifying an unknown piece by type/color and recovering pieces lying on
  their side; the current learned tasks name source squares and the scripted
  grasp expert cannot recover toppled pieces.
- Public release of private CAD geometry. The private tool pack may be included
  only in the authorized LAN deployment bundle.

## Release gate

For every requested simulator release: run the local regression suite, build
and deploy only the authorized `robot_arm` stack, verify the exact live version
and cameras/controls, then run the complete crowded-v3 fixed suite and record
its receipt. For repeatability qualification, run the three bounded seeded
crowded-v3 suites and retain per-case receipts and failure archives. Individual
case failures are diagnostics—not a green manipulation qualification.
