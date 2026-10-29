# Simulation and training roadmap

Reconciled 2026-10-29 against the current source, live release, training scripts,
and experiment log. Priority 1, **make crowded manipulation repeatable**, is
complete on live v1.10.6; measured results are in [`EXPERIMENTS.md`](EXPERIMENTS.md)
and CAD/import assumptions are kept with the private tool pack.

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

## Priority 2 — measure the scripted-control baseline

- Score the existing scripted expert across randomized placements and piece
  sets. Keep simulator truth confined to the evaluator whenever testing a
  camera-driven controller or policy. Report per-task/per-piece results and
  seeds, not only an aggregate pass rate.

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

- Add controlled tests where an upright piece is deliberately displaced from
  the square center. Sweep offsets and orientations within a declared test
  envelope, including camera/viewpoint and board-pose variation; retain a
  centered-piece baseline and evaluate held-out layouts.
- Implement the non-learned foundations first: camera intrinsics, board-plane
  registration, camera-to-arm/tool transform, piece-base/contact-point
  localization, and a bounded approach/grasp plan that uses the observed piece
  location rather than the nominal square center. Re-observe after motion and
  report uncertainty instead of attempting a low-confidence grasp.
- Measure localization error, grasp success, collateral displacement, and
  false/abstained targets against simulator truth in the evaluator only. Derive
  the acceptable offset envelope from the measured perception, gripper, and
  clearance error budget; do not treat a learned policy as a substitute for
  calibration or collision-safe control. Repeat on physical hardware later.

## Priority 5 — establish the training platform and data boundary

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
