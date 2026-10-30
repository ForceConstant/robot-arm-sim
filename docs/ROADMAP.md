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

## Priority 5 — establish the training platform and data boundary

- **Local inference required (Brian, 2026-10-30):** all selected models must
  run on the existing GTX 1060 3 GB host or BC-250; Colab may train them but
  does not become an inference dependency. **Brian superseded the small-first
  recommendation: investigate BC-250 GPU/APU capabilities, then find the largest
  practical local model.** MolmoAct2 remains a candidate; local fit is unmeasured.
  [BC-250 research](../training/BC250_CAPABILITY.md) distinguishes large quantized
  LLM evidence from robot-policy compatibility and compares the RTX 5080. Before substantial training, test exact-size
  model operators/capacity locally. Completion also requires the trained local
  artifact's peak RAM/VRAM under concurrent simulator load, p50/p95 latency,
  checkpoint reload/output agreement and held-out evaluation receipts. See
  [local budgets and gates](../training/PLATFORM_AND_DATA.md#deployment-first-constraint--brian-2026-10-30).

- **In progress (2026-10-30):** platform/data rules and a fail-closed recording
  auditor are implemented in [`training/PLATFORM_AND_DATA.md`](../training/PLATFORM_AND_DATA.md)
  and `tools/audit_training_data.py`. Whole sessions/layout groups cannot cross
  splits; simulator/real origins remain separate and private. Local regression:
  **41 passed, 4 skipped**, including six data-boundary tests.
- **Recording/data pilot passed:** 3/3 verified physical-contact moves; two
  train recordings and one independent held-out recording. Full audit accepted
  all three: 648 frames, 620,193 bytes, no errors. Receipts:
  `.private/priority5-pilot/manifest.json` and `audit.json`. This is a
  recording-integrity pilot, **not** an optimizer/checkpoint training pilot.
- **Live platform evidence:** the simulator workspace has a GTX 1060 3 GB and
  no torch/LeRobot training stack; preflight correctly failed. Brian powered on
  Bazzite and proposed using it: live probe shows BC-250 gfx1013, 8 GiB reserved
  VRAM plus 7.5 GiB CPU-visible RAM from one shared 16 GB pool, no installed
  ROCm/PyTorch training stack. Community native-build training is plausible for
  small perception models, not proof this box works or capacity for the
  approximately 30 GB MolmoAct2 recipe. No OS/driver/firmware changes made.
- **Measured BC-250 local inference (2026-10-30):** official portable Vulkan
  runtime installed user-locally with verified hash. Gemma 3 27B IQ3_XXS passes
  a bounded **text-only** smoke/reload (63/63 layers GPU-offloaded, ~20 tokens/s,
  >=1.50 GiB CPU headroom). Four vision configurations stop on memory guards;
  no Gemma image response or VLA pass. Gemma vision work reached 84 C.
  **Qwen3.5-35B-A3B IQ2_M vision smoke completed** after disabling optional
  server prompt/context caches: 7/7 requests, both color controls correct,
  real camera descriptions 1.97–2.10 s; >=1.07 GiB CPU headroom, peak 76 C.
  Separate reload/pair probe completed 4/4; paired wrist images took 3.34 s,
  >=1.18 GiB headroom, peak 75 C. Largest successfully tested VLM so far, not a
  hardware maximum, calibrated perception or learned robot-controller pass.
  Initial cache-enabled 35B run remains a recorded memory-guard failure.
  [Exact scope and receipts](../training/BC250_CAPABILITY.md#live-experiments--2026-10-30).
- **Hosted GOAT plan available for general LLM/VLM work (Brian, 2026-10-30):**
  the Command Code GOAT subscription is an active hosted option for general
  reasoning, planning, labeling QA, evaluation scoring and authoring — not a
  robot deployment path. Live check: $53.10/$70 monthly credits remaining
  (75.9%), 5h $0.09/$14, weekly $1.54/$35; per-model allowances still cap
  heavy models. Never send private recordings, CAD/printed assets, camera
  frames or checkpoints off-LAN without an explicit transfer decision; cloud
  round-trip rules it out for 10 Hz control. Local BC-250 remains the
  robot-adjacent inference target.
- **Colab candidate (Brian's suggestion):** prepared
  [`colab_preflight.ipynb`](../training/notebooks/colab_preflight.ipynb) to
  identify allocated GPU/VRAM and exercise a tiny synthetic optimizer/checkpoint
  round-trip without private uploads. Syntax verified; cloud execution and
  actual-task training remain unverified. Paid/free tier and compute budget
  await confirmation. HIL-SERL's Panda tutorial is relevant future architecture,
  not a drop-in SO101 environment; details in the platform document.
- **Training host decided: free-tier Colab (Brian, 2026-10-30).** No paid plan
  or compute units. Constraints: T4-class 16 GB typically, no bf16 on Turing,
  ~12 h sessions with idle disconnects, no SSH/remote/distributed workers,
  ephemeral disk. MolmoAct2 is 5.442B parameters — 10.14 GiB in fp16/bf16
  weights and ~40 GiB of AdamW state — so **full fine-tuning cannot fit a T4**.
  The trainer now exposes LeRobot's `PeftConfig` (LoRA method, rank, alpha,
  target modules, full-training modules) and a `--mixed-precision` override, so
  an adapter run is constructible; five new tests cover the flags and the full
  regression is 49 passed / 4 skipped, plus `training/settings.free_tier_colab.env`
  (20 steps, micro-batch 1, accumulation 8, LoRA r16, fp16). Command
  construction and settings only — no GPU run yet, so fit is still an
  arithmetic expectation. A free-tier pilot therefore needs
  an adapter (LoRA/QLoRA) or frozen-backbone option, micro-batch 1 with
  accumulation, and Drive-backed resumable checkpoints. Public dataset use
  discloses nothing new; uploading private recordings/CAD still needs an
  explicit decision.
- **Still required:** select a locally deployable model/runtime and training
  host, confirm storage/access, then pass selected-host runtime preflight, a
  bounded optimizer/checkpoint pilot and the local deployment gates above. The upstream RTX 5090 is
  not confirmed available. Do not mark this priority complete on data checks
  or recording-only pilot evidence.

- Before another substantial training run, choose and document the training
  machine/runtime, available accelerator and memory, storage for datasets and
  checkpoints, software/container environment, and how results move to the
  simulator or eventual arm. The existing training notes record an RTX 5090
  32 GB host; confirm its availability for future runs rather than assuming it
  is the selected/available platform. Do not assume the BC-250 AMD GPU supports
  the current CUDA-oriented training stack.
- Separate simulator-generated labels/episodes from real-camera data. Decide
  which data remain local and how held-out capture sessions/layouts are kept
  out of training. Complete preflight, dataset/action-label checks, and a
  bounded pilot before committing long GPU time or storage.

## Priority 6 — train/evaluate board registration and occupancy/color perception

- Treat board registration (camera intrinsics, board pose, and camera-to-arm
  geometry) as a calibrated geometry problem first, not a single learned task.
  If learned registration is later justified, evaluate it independently from
  object classification.
- Train/evaluate square occupancy and piece color from top/wrist views, with
  explicit unknown/low-confidence outcomes. Include occlusions, hands/shadows,
  board pose/lighting variation, and held-out physical capture sessions when
  available. Compare predictions with simulator truth in evaluation only; the
  controller must not read oracle piece poses or FEN during the vision test.

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
