# Chess robot — simulation-first delivery roadmap

**Updated 2026-11-10. Canonical delivery plan.** This replaces the old priority
list and the earlier hardware-first build sequence. Phases 0–8 and the parallel
Phase 11 runtime lane retain their identifiers and measured, scope-limited
completion evidence below. **Phases 9 and 10 are complete** (2026-11-03 at 94.0%
turn resolution; 2026-11-09 owner-accepted at 98.54%). **Phase 11 is complete (2026-11-10): 11a (infrastructure/execution) and 11b
(control-policy training) complete; 11c (vision-model training) closed with
exit-compliant measured evidence in simulation envelope** (c/a frozen/geometry
accepted with bounded throughput/memory/thermal/reload/held-out evidence; b
measured and frozen B1-alone rejected with documented fallback) — see
  - Follow-up (2026-11-11, measured): type-ID+pose probe iterated on bc250-cachy but still insufficient for full 11c(b) E2E (baseline 0.664 acc / 0/12 E2E accepted; iter 0.468 acc / 5/18 type-correct / 1/18 accepted). Additional harnessed R/A1 engineered-stress evidence collected (0 false confirmations, safe abstentions; shadow-extreme 74.29%/68.33%, arm-over-changed 20.0%). See `reports/phase11c/followup_2026-11-11.md`.
[`docs/PHASE11C_RESULTS.md`](docs/PHASE11C_RESULTS.md). **Phases 12–21 are planned, not
completed.
Historical acceptance filenames
such as `PRIORITY5_ACCEPTANCE.md` remain stable evidence links.

## Final working version

One physical follower arm plays a complete human-versus-robot game from a
human-confirmed setup, using **only its wrist camera** for operational vision:
find the board relative to the arm, locate an off-center piece for a clear grasp,
infer the opponent's move, choose a legal reply with Stockfish, execute it and
visually verify it. Handle captures, castling and en passant; promotion may ask
the human to exchange the piece and verify the result. Ambiguity or faults pause
for recovery rather than inventing a state. Save/reload games without losing
castling/en-passant rights or replaying an interrupted move.

**Complete all core software and feature qualification in simulation before
integrating autonomous software with the real arm.** Existing manual LeLab
teleoperation is a bench fact, not an exception granting autonomous integration.
Measuring printed parts for the digital twin is not robot integration. Simulated
success never passes a physical gate.

### Selected roles and policy plan

- **Raspberry Pi 4:** direct USB connection to follower/servo adapter and wrist
  camera; timestamped captures, joint feedback, finite bounded command execution,
  local limits/watchdog, stop/hold and explicit re-arm. No large-model training.
- **BC-250:** central perception, registration, game coordinator/Stockfish,
  planning, and model inference during play; training host where the actual
  runtime/model passes qualification. Training and gameplay need not run at the
  same time. Shared 16 GB is one physical pool, not additive CPU/GPU budgets.
- **HTPC GTX 1060:** retain simulator/rendering/data generation. Test the selected
  single-follower product layout, not just cooperative two-arm coverage. The
  existing two-arm scene remains a regression/debug fixture, not a second-arm
  purchase or a solution to single-arm reach.
- **Vision baseline:** OpenCV geometry for lens/board/hand–eye transforms;
  compact segmentation/keypoints for piece base/grasp region and occupancy/color;
  legal-successor comparison for move inference. Known piece geometry or multiple
  settled views resolve parallax; one RGB frame is not an exact 3D measurement.
- **Learned manipulation:** qualify **goal-conditioned ACT** as the minimum
  action-policy baseline. Supply explicit source/destination in calibrated frames,
  state and wrist images; vanilla ACT is not language-directed by default. Use
  bounded final approach/grasp/place correction, compared with guarded IK/scripted
  control. Neither policy replaces Stockfish or independent verification.
- **Capacity comparison:** preserve Brian's largest-practical-local-model
  direction in parallel with the ACT baseline. SmolVLA, MolmoAct2 or a larger VLA
  needs an exact-runtime fit and a falsifiable improvement hypothesis. A 35B
  language/vision inference pass proves neither VLA action support nor precision.
  The inherited SmolVLA recipe failed; do not call it a fresh proven alternative.

## How to close a phase

Every phase must deliver software or answer a named downstream decision with a
reproducible experiment. Record inputs, held-out seeds/sessions/layouts, runtime
and model hashes, per-case outcomes, failures, timing/memory/thermals, and links
in [EXPERIMENTS.md](EXPERIMENTS.md). Set numeric tolerances **before** evaluation;
future targets below are proposed acceptance targets, not measured results.
A failed experiment can close a feasibility decision with a documented fallback,
but cannot close a required product capability. Do not weaken gates silently.
No next-stage physical work until the simulation release gate passes.

### Phase sequence

- **0–7 — complete foundations:** imported tool; crowded regressions; scripted
  baseline; reusable corrections; off-center targeting; local platform/data;
  sparse fixed-rig perception; sparse move inference. Limits remain below.
- **8 — product digital twin and acceptance contract — complete 2026-11-02.**
- **9 — wrist-only board calibration and full-board observation — complete
  2026-11-03 for the stated envelope (94.0% turn resolution on crowded boards).**
- **10 — grasp localization and robust printed-piece manipulation.**
- **11 — local training and execution for control and vision models — complete
  (2026-11-10, sim envelope):** 11a infrastructure/execution and 11b control-policy
  training complete; 11c vision-model training closed with measured evidence
  (c/a frozen/geometry accepted; b frozen B1-alone rejected with documented fallback).
- **11d — live BC-250/Pi teleop and board-discovery demo (bring-forward, next).**
  A deliberately narrow vertical slice on real hardware. It consumes parts of 9,
  11, 13 and 15 and closes none of them; 12–22 keep their numbers and gates.
- **12 — audited training, corrections and local-policy comparison.**
- **13 — complete simulated game coordinator and UI.**
- **14 — simulated recovery, soak tests and release candidate.**
- **15 — Pi/BC-250 service boundary against fake hardware.**
- **16 — fabricate the physical board and piece set.**
- **17 — physical fixture and camera/arm calibration.**
- **18 — guarded real manipulation and real-data adaptation.**
- **19 — complete real games and product acceptance.**
- **20–22 — simulation-first extensions:** loose-piece setup, avatar, emotes.

Dependencies: 8 → 9 → 10; 12 requires 10 + 11. **Phase 11 is split by
dependency:**

- **11a — infrastructure (after 8, unblocked):** dataset ingestion, wrist-stream
  decoding, training harness, checkpoint/reload, fit and thermal tests on the
  1060 and CPU. This is where the current work sits; it does not need 9 or 10.
- **11b — control-policy training (needs 9 + 10):** the corpus must use Phase 9's
  wrist observation contract and Phase 10's grasp/task buckets and pose targets.
  Training before those exist means training against the wrong distribution and
  repeating it.
- **11c — vision-model training (needs 9 + 10):** registration/occlusion/
  abstention behaviour comes from 9; piece type, in-square position/orientation
  and crowding buckets come from 10. Selecting or training the model earlier has
  no defensible label set or error budget.

13 consumes 9+10+12 (a measured rejection may select scripted control).
11d is a supervised bench demo requested by Brian (2026-11-15) and is the only
planned exception to "no physical deployment before 15/17": it may move the real
arm for camera-only scan poses under the safety gates in its section, but it does
not manipulate pieces, does not replace 14's release gate, and does not count as
17's measured commissioning. 14 → 15 → **hardware boundary** → 16 → 17 → 18 → 19.
Extension software/simulation qualification happens before its physical rollout,
never by testing first on hardware. No new purchases, cloud uploads or
host-driver changes are authorized by this planning document.

## Historical foundation evidence — measured scope, not current live status

The release/version statements below describe their dated qualification runs;
this documentation refactor did not query or change the live service. Later
measurements in each phase supersede earlier counts. Completed means the named
bounded experiment passed, not that the final product gate passed.

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

## Phase 0 — qualify the imported CAD tool — complete (simulation)

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

## Phase 1 — make crowded manipulation repeatable — complete (bounded suite)

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

## Phase 2 — measure the scripted-control baseline — complete

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

## Phase 3 — make verified incident corrections reusable — complete

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

## Phase 4 — camera-targeted pickup of off-center pieces

- **Status: complete for the qualified fixed-overhead simulation envelope; no
  learned-policy training is required.** The task may name the source square
  and piece type. General piece/color recognition and learned occupancy remain
  Phase 6; independent physical camera/tool calibration and hardware trials
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

## Phase 5 — establish the local platform and data boundary — complete

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
architecture, final perception quality belongs to Phase 6, state inference to 7,
learned-control qualification to 11–12, and physical readiness to 17–19. These remain open.
The pilot does not replace our calibrated controller or qualify MolmoAct2,
LeRobot/PyTorch training, continuous VLM service, or a physical robot.

## Phase 6 — board registration and occupancy/color perception — complete

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
  positions only; move/state inference is boundedly complete in Phase 7; wrist-only and crowded-board qualification remain Phase 9.
  No physical capture session is claimed.
- **Implementation:** `chess_sim/board_registration.py`,
  `chess_sim/perception.py`, `chess_sim/perception_data.py`,
  `tools/train_perception.py`, `tools/benchmark_perception.py`; model artifact
  `chess_sim/assets/perception/model.json`. Local regression passed
  **67 passed, 4 skipped**. Receipt `training/PERCEPTION_ACCEPTANCE.md`, private
  receipts `.private/perception/heldout-v8.json` and `training-v8.json`.

## Phase 7 — infer move/state changes from observations — complete

**Closed 2026-11-02 for the qualified fixed-overhead-rig envelope with settled
observations and sparse positions.** See
[`training/MOVE_INFERENCE_ACCEPTANCE.md`](../training/MOVE_INFERENCE_ACCEPTANCE.md).

- `chess_sim/move_inference.py` compares the last confirmed position with every
  legal successor. A move is returned only when exactly one successor is
  verified and no other legal move remains possible under unobserved squares.
  Outcomes are explicitly `confirmed`, `unchanged`, `ambiguous`, `insufficient`
  or `unexplained`; the previous board is never mutated. A confirmation pushes
  the move so castling/en-passant/half-move history is preserved.
- Promotion is reported as `ambiguous` with `promotion_required` rather than
  guessed. En passant and castling use the history on the previous board. A
  confident board change that no legal move explains is `unexplained`.
- `perception.recover_reference_empty()` optionally recovers *empty* squares
  that match the session's empty-board reference; it never guesses white or
  black. `merge_observations()` fuses top/wrist views without letting either
  override a confident disagreement.
- Held-out rendered benchmark (`seed 7001`, 3 sessions, 35 cases, top view):
  **13/13 accepted moves correct, 0 false confirmations**, 37.1% coverage,
  22/35 abstentions, 15/15 expected-abstain cases honoured. Fused top+wrist was
  11/11 correct at 31.4% coverage. The low coverage is the hard arm shadow
  leaving a changed square `unknown`; the module abstains rather than guesses.
- En passant and promotion require every changed square to be visible. In the
  held-out run en passant abstained (one changed square hidden) and one
  promotion returned the intended ask-for-piece outcome. Both are covered by
  exact unit tests. Sparse positions only; no full-game recovery, crowded-board
  evaluation, physical capture or live web-UI integration.
- Code: `tools/benchmark_move_inference.py`, `tests/test_move_inference.py`,
  `tests/test_perception.py`; regression **86 passed, 4 skipped**. Private
  receipts `.private/move-inference/heldout-top.json` and
  `heldout-fused.json`.

## Phase 8 — freeze the product digital twin and acceptance contract

**Status: complete 2026-11-02.** Committed evidence:
[`hardware/product_twin.json`](../hardware/product_twin.json),
[`docs/PRODUCT_CONTRACT.md`](PRODUCT_CONTRACT.md),
[`docs/PRODUCT_TEST_MATRIX.md`](PRODUCT_TEST_MATRIX.md),
[`docs/PHASE8_QUALIFICATION.md`](PHASE8_QUALIFICATION.md) and
[`reports/phase8_product_qualification.json`](../reports/phase8_product_qualification.json).

Measured board: 28 mm squares, 15 mm labelled border, 3.2 mm slab, 254 mm
overall. One white-side follower at an 80 mm board gap and 60 mm riser. Rook
mass is the single measured assembled value, 4.8 g including one M6 nut; all
other assembled masses remain explicit ranges, not invented facts.

**Historical tool decision (80/60 mount):** the stock SO-101 pinch gripper was selected. The NormaCore
custom PLA parallel jaw was rejected for this 28 mm board: it failed 11/64
squares in the path map and spot executions at e1, c1, d1 and h8. Reconsider it
only after a revised jaw passes the same matrix. This dated decision is not a
current-layout rejection: the [updated-offset rerun](PHASE8_UPDATED_OFFSETS.md)
executes the unchanged current CAD jaw at the 90/30 mount, with **62/64 first
pass and 64/64 paths after retries**. A span-only candidate passes **25/25
crowded cases**. Keep that jaw geometry and improve validated span selection;
do not infer a required mechanical redesign from the historical scan. Candidate
parameters are not deployed defaults; the live v1.12.1 receipt remains 21/25.

**Outcome:** a reproducible scene/configuration and test manifest representing
our single physical follower, printed board/pieces, selected tool, wrist camera
and reachable capture/staging tray. Depends on completed foundations, not a new
hardware commissioning session.

- Record actual board pitch/thickness and per-type assembled M6-weighted masses;
  unknown measurements remain parameters with bounded sensitivity sweeps, not
  invented facts. Replace unweighted PLA estimates when measurements arrive.
  Capture COM/inertia, friction/contact uncertainty, jaw compliance and camera/
  cable envelopes; preserve CAD provenance and private-asset boundaries.
- Map every square and tray destination through approach, grasp, lift, transit
  and descent on **one follower**, including neighbor clearance. Two arms jointly
  reaching four corners does not prove this. Compare base/riser/board layouts;
  surface any unreachable legal move as a blocking design decision.
- Freeze observation/action units, joint order, timestamps, camera poses, board
  transforms, supported perturbations and independent physical-success evaluator.
  Inventory script, perception, policy and game interfaces plus regression IDs.
- **Exit (met 2026-11-02):** committed config + reach/clearance report + product
  test matrix. Actual execution succeeded from all 64 squares with at least one
  piece type, and from all 4 tray slots. First-pass pawn failures at a1, c3, a7,
  g7 and e8 passed the same squares with king/rook; they are Phase 10
  grasp-reliability inputs, not silently excluded squares. Geometry scan
  residuals at extreme corners are named in the report and are not hidden
  gameplay exclusions. No blocking redesign remains for the selected stock tool.

## Phase 9 — wrist-only calibration and complete-board perception

**Status: complete 2026-11-03 for a stated envelope — owner decision (Brian).**
See [`PHASE9_WRIST_ONLY.md`](PHASE9_WRIST_ONLY.md) for the implementation,
envelope, numbers and limits. Registration from the printed marker mat meets the
2 mm / 1° target on 8/8 held-out sessions (worst 0.80 mm, 0.09°) with zero false
move confirmations in 200 turns and 16/16 invalid rearrangements rejected.
Turn resolution is **94.0%** (188/200) from six scan poses — three tilted — with
every square clear in ≥3 views (median 5), up from 72.5% for vertical-only poses.

**Bounded acceptance:** the initial 95% exit target was **not** reached. Brian
accepted 94.0% as the qualified envelope on 2026-11-03 on condition that the
limit is stated: crowded 32-piece boards can leave 15–61 squares `unknown`, so
roughly one turn in sixteen abstains (`insufficient`) rather than guessing. Two
variants were measured and rejected — an 8-pose plan at ≥4 clear views per
square (92.0%) and dropping per-view votes that match the empty reference
(39.5%) — and are recorded so they are not repeated. Self-model occlusion
(projecting the known arm geometry to exclude its own shadow) is queued as a
Phase 10 refinement; it is the principled fix for the residual `unknown` band.

**Outcome:** locate a shifted board relative to the arm and obtain trustworthy
observations using only wrist images and measured joints. Depends on 8.

- Implement lens calibration inputs, hand–eye/tool frames and board-plane pose
  estimation (OpenCV corners/edges; printable fiducials are an optional method).
  Do not read simulator board/camera truth at inference; use it only for labels
  and scoring. Re-register after board shifts; invalidate stale empty references.
- Plan settled scan poses/multiple unchanged-scene views covering all 64 squares.
  Train/evaluate occupancy/color and hand/foreign-object/occlusion rejection on
  crowded starting positions, sparse/endgames, lighting, shadows, board movement
  and camera uncertainty, with session/layout-separated holdouts.
- Connect legal move inference; handle captures whose destination stays occupied,
  castling, en passant, unchanged boards, invalid rearrangements and promotion
  ambiguity. A fused overhead view is an ablation, never the product input.
- **Exit:** camera-only board pose and complete-turn reports with errors,
  coverage, false confirmations and abstentions. Initial target: registration
  within 2 mm XY / 1° yaw on ≥95% of in-envelope held-out sessions; ≥95% turn
  resolution after at most three scans, zero false move confirmations in ≥200
  held-out turns, and all deliberate invalid/occluded cases safely rejected.
  Check that measured error fits the Phase 10 grasp-clearance budget; tighten
  these targets if it does not. Current 37.1% move coverage does not pass.

## Phase 10 — precise grasps and reliable printed-piece execution

**Outcome:** convert actual piece position within a square into a reliable,
clearance-aware move, not a square-center assumption. Depends on 8–9.

- Add segmentation/keypoints/base/contact geometry, piece-specific grasp heights,
  knight yaw/approach choices and uncertainty; compare known geometry with extra
  wrist views. Reject unreachable, uncertain or neighbor-colliding grasps.
- **Queued from Phase 9 (2026-11-03): self-model occlusion.** Project the known
  arm geometry into each wrist frame so the arm's own shadow abstains instead of
  voting, then remeasure crowded-board turn resolution against the 95% target.
  Phase 9 closed at 94.0% with that gap named rather than hidden.
- Implement observe → approach → re-observe → grasp → verify lift → transport →
  place → retract → verify. Tune physical closure/contact parameters honestly;
  retain collisions/slips, not welds, teleportation or forgiving success labels.
- Include weighted mass/friction sensitivity, all six types/colors, both sides
  of the board, ±off-center placements, dense starting layouts, captures/tray,
  castling paths, en-passant removal and interrupted compound moves.
- **Exit:** held-out ≥300 paired cases covering these buckets; proposed target
  ≥99% complete transactions with **zero wrong-piece picks or collateral moves**
  and all failed/uncertain outcomes detected before game-state commit. Record
  per-type/task/crowding results, not only a favorable aggregate. Physical
  realism and Phase 14 full-game tests remain separate gates.

**Status: complete 2026-11-09 — owner-accepted at 98.54%, below the 99% target,
with the shortfall recorded rather than rounded up.**

Measured on a fresh held-out set: 480 transactions from new seeds, zero overlap
with the development matrix and with the earlier held-out set, 50 buckets with at
least 6 cases each. Build: printed parallel-jaw gripper with the tip-thin finger
(mass corrected to 17.03 g), slowed precise descent, rank-1 placement tilt,
release containment tolerance.

- **473/480 = 98.54% complete transactions**
- **zero wrong-piece picks and zero collateral moves**
- all seven failures detected before game-state commit — every one a guard refusal
- six buckets short: king/white/crowded/capture 10/12,
  knight/black/crowded/capture 11/12, knight/black/crowded/quiet 11/12,
  knight/white/sparse/capture 5/6, pawn/white/sparse/capture 5/6,
  rook/black/crowded/quiet 11/12

The seven refusals: king e1-d2, king g1-f2, knight f6-h7, knight g8-h6,
knight e2-f4, rook b8-c8 (gripper-neighbour contact during approach or retreat,
0.046-0.297 mm) and pawn d1-e2 (lift waypoint miss). Measured neighbour
displacement at the approach latch on quiet moves is 1.08-1.17 mm — ten times the
0.1 mm containment limit — so the residual is an approach-geometry limit, not a
tuning gap, and no tolerance change would honestly clear it.

Approach-tilt variants at 20, 32 and 60 degrees were measured and rejected: no
conversions at 20 or 32 (32 also broke a previously passing case), unreachable
motion at 60. Owner decision 2026-11-09: accept the measured result and document
the refusals. A true lateral approach from the board edge is the untried fix and
is recorded as future work.

Receipt: `.private/phase10/heldout-v2-verdict.json`. Physical realism, hardware
integration and the Phase 14 full-game gate remain separate and unpassed;
simulated success never passes a physical gate.

## Phase 11 — local training and execution for control and vision models

**Dependencies: Phase 8 (product twin) for 11a; Phases 9 and 10 for 11b and 11c —
11a can start immediately after 8, but neither model can be trained for real
until Phase 9 fixes the wrist observation/registration/abstention contract and
Phase 10 fixes the grasp buckets, task families and piece pose/orientation
targets.** Keep that order: training before those contracts exist means training
on the wrong distribution, and Phase 12 already requires 10 + 11 anyway, so the
gate costs no schedule.

**Outcome:** a measured execution runtime **and a locally runnable training path
for both the control policy and the vision model**, for Phases 12–18. Local
services only — BC-250, HTPC GTX 1060, CPU. Cloud training is out of scope. A
route that cannot train locally is recorded as a rejected path with its measured
blocker, never as a pass.

**Status: complete (2026-11-10, exit-compliant sim envelope).** 11a complete
(2026-11-03). 11b complete (2026-11-10, gate met in aggregate with a named
move-family shortfall). 11c complete (2026-11-10): (c)/(a) closed by measured
frozen/calibrated-geometry decisions with bounded throughput/memory/thermal/
reload/held-out evidence and 0 false confirmations; (b) closed by specific local
route measured and rejected (B1, given type hypothesis, 3/24 accepted, no type-ID)
with documented blocker and fallback. See [`docs/PHASE11C_RESULTS.md`](PHASE11C_RESULTS.md).
Simulated success does not pass a physical gate.

> **Execution host (2026-11-10, Brian):** from now on, all training and simulation/analysis runs for Phase 11 (11b follow-ups and 11c onward) must run on **`bc250-cachy`** (`brian@192.168.1.171`, CachyOS, BC-250, project `/home/brian/robot-arm-sim`, launcher `~/.local/bin/robot-arm-sim`). Synchronize the active working tree before runs (exclude `.private`, generated datasets/runs, `dist`, venv/toolchain dirs, private CAD assets; preserve BC-250 generated data; no destructive mirroring). Use `robot-arm-sim test|python|serve` with explicit scope/duration; MuJoCo physics remains CPU-executed (EGL verified) and GPU compute is not implied. Record this host in 11c receipts.

### 11a — infrastructure and execution (after Phase 8; unblocked)

Scope: dataset ingestion (public LeRobot v3, including the wrist stream),
training harness, checkpoint/reload, GPU fit tests, and thermal soak on the
1060 and the BC-250.

Measured 2026-11-03:

- **CPU ACT** (51,565,446 parameters; ResNet-18, 224×224, 30-step chunks, VAE,
  FP32): batch-1/2 initialized steps, 20 AdamW updates on audited recordings,
  exact checkpoint reload, bit-identical resumed update, independent
  inference-only reload. 30-minute scheduled inference: 1201 cycles, p95 101 ms,
  no deadline or swap-growth violations; capture-age admission remains
  unverified because the simulator publishes no capture timestamp.
- **GTX 1060 3 GB**: full ACT fits at batch 8 under 1.07 GiB reserved with the
  live simulator rendering; 74.9 samples/s on synthetic tensors and **62.6
  samples/s** on real data.
- **Real-data pilot**: public `so101_chess` slice (25 of 6,095 episodes, top
  camera), 400 samples, 200 steps, loss 85.0 → 4.09, checkpoint reload bit-exact.
  Held-out 944.5 vs 465.6 deg² for hold-position — overfit, so this proves the
  loop, not a policy.

**11a complete — 2026-11-03.**

- **Harness:** `training/probes/so101_train.py` (GPU and CPU devices, duty cycle,
  resume, JSONL metrics, guarded temperature monitor) and
  `training/probes/so101_fetch.py`. Smoke-tested on GPU and CPU with bit-exact
  checkpoint reload.
- **Ingest:** all **82/82** pinned files present, **0 missing**, 16.1 GB in
  `phase11_data` — 39 top + 36 wrist videos plus meta/data. Stale partial files
  removed.
- **Fit:** full ACT at batch 1/2/4/8 under 1.07 GiB reserved; 74.9 samples/s on
  synthetic tensors, 62.6 on real data.
- **Thermal, pre-service:** continuous batch 8, duty-cycled batch 4 and batch 1
  all hit the guard (85/85/83 °C) — the card was thermally unusable.
- **Service (Brian, 2026-11-03):** heatsink cleaned, paste replaced. Idle fell to
  **43 °C** (was 56); 20 s at ~60 W reached **62 °C** (was 80 °C at 1.5 s).
- **Thermal, post-service:** two 30-minute soaks completed clean — batch 8 at
  **67.0 samples/s**, batch 6 at **57.8 samples/s**; peak **80 °C**, median
  **79 °C**, no throttle flag, no guard trip, checkpoint reload exact.
- **Accepted operating envelope: 79–80 °C (Brian, 2026-11-03).** This misses the
  originally written ≤78 °C target by 1–2 °C and is recorded as an explicit
  acceptance, not a silent relaxation. Revisit if multi-hour runs drift upward.
- **Wrist stream** validated on CPU and GPU (2 episodes, reload exact).
- Scope limits: one camera per run, 12 episodes per run, heavy overfit (held-out
  worse than hold-position) — this qualifies the pipeline, not policy quality.

### 11b — control-policy training (needs Phase 9 + 10)

Scope: goal-conditioned ACT trained end-to-end on product-distribution data —
Phase 9's wrist observation contract, Phase 10's grasp/task buckets — with
measured samples/s, peak memory, thermals, and a held-out learning curve.

Gate: held-out error must improve on hold-position and be stable across seeds;
only then does Phase 12 compare it with scripted control. A 20-episode/200-step
pilot does not satisfy this.

**11b complete — 2026-11-10.** Full-size ACT (51,565,446 parameters) trained
locally on the BC-250 CPU from the 1.13.0 product build: 48 buckets, 768
recorded / 756 verified-success episodes, 18,144 samples (13,608 train / 4,536
held out, split by whole episode). Three seeds × 3,000 steps at batch 4:
**7.40–7.41 samples/s**, ~27 min/seed, peak RSS **4.70 GB**, peak SoC
**76.6–77.5 °C** (accepted 79–80 °C envelope), exact checkpoint reload (0.0 rad).

- Held-out L1 vs hold-position: **+7.6% / +17.2% / +2.1%** (mean **+9.0%**,
  population SD 6.2 pp); MSE **+37.3% / +42.0% / +32.5%**. All seeds improve in
  the same direction, so the stated stability gate passes.
- **Named shortfall:** the gain is concentrated in capture/tray tasks
  (**+19.7% to +28.1%**). Move buckets are **−10.5% / +4.6% / −18.1%** versus
  hold-position (mean **−8.0%**). Do not adopt this policy for quiet moves;
  Phase 12 must evaluate families separately and may retain scripted control.
- No rollout, task-success or physical result is claimed. Scope limits: 0.88
  epoch, 3,000 steps, 48 simulation buckets, CPU FP32.

Receipt: [PHASE11B_CONTROL_TRAINING.md](PHASE11B_CONTROL_TRAINING.md). Evidence:
`reports/phase11b_*.json`.

### 11c — vision-model training (needs Phase 9 + 10) — closed 2026-11-10 (exit-compliant, sim envelope)

**Status:** complete (2026-11-10). Executed on **bc250-cachy** (`brian@192.168.1.171`, CachyOS BC-250, 16c/14GB), MuJoCo CPU/EGL, env `/home/brian/robot-arm-sim/.venv/bin/python` 3.11.18; active tree synced to `/home/brian/robot-arm-sim-phase11c`. Measured frozen/calibrated-geometry decisions and measured rejection+fallback recorded with local evidence (bounded samples). Full closure: [`docs/PHASE11C_RESULTS.md`](PHASE11C_RESULTS.md), [`docs/PHASE11C_EVIDENCE_PLAN.md`](PHASE11C_EVIDENCE_PLAN.md), [`docs/PHASE11C_VISION_PLAN.md`](PHASE11C_VISION_PLAN.md), [`reports/phase11c/`](../reports/phase11c/).

Scope (met per exit): choose/train candidate for worst case, or record measured
frozen-model + calibrated-geometry decision meeting same budgets for:

- **(a) Opponent-move change detection robust to occlusion and shadows — ACCEPTED (A1 frozen/geometry, sim envelope):** self-occlusion-masked perception/fusion + existing legal-successor inference. Controlled paired held-out (seed 9300, 8×25=200, 6-view, identical hashes): **200/200 resolved, 0 false confirmations**, insufficient/coverage abstentions eliminated in sample (residual safe ambiguous promotion). Dedicated shadow/arm-over-changed stress characterization remains a follow-up for tighter bounds.
- **(b) Per-piece type plus in-square position/orientation, including knight yaw — B1 measured; frozen B1-alone REJECTED for full 11c(b), fallback documented:** `fit_mesh_axis` (given type/mesh, axis mod π, 6-view) eval 24 cases (4/type): **3/24 accepted (12.5%, knights)**; accepted knights accurate (axis err mean 0.513°, XY mean 0.614 mm, ~1.32 s/fit) but low acceptance on symmetric pieces and **no per-piece type identification**. Measured blocker: type-ID absent + 12.5% acceptance/instability. **Documented fallback:** scoped local type-ID+pose training probe on bc250-cachy (or hybrid type-known + B1 refinement with abstention) with dataset manifests/train/reload/held-out/resource/thermal evidence.
- **(c) Wrist-only board registration under real occlusion (Phase 9) — ACCEPTED (C1 frozen/geometry, sim envelope):** marker registration + self-occlusion-masked wrist occupancy/colour + median fusion. Controlled paired held-out: **200/200 vs 190/200 no-self-occ vs 188/200 baseline, 0 false confirmations**, abstentions 13→11→1, reg **8/8 ≤2mm/1°**, unknown/rescans reduced. R (bounded): reload/determinism identical (SHA256 `7deba606...704d5`); harnessed 4×5 rc 0 20/20, peak RSS ~3.47 GiB, tctl 48.25→55.625°C max, no throttling noted.

Prior floor (geometric registration, 32-unit NumPy occupancy/colour MLP, legal-successor inference) remains the basis; C1/A1 extend it with measured self-occlusion handling and evidence. Not a physical gate; cloud training out of scope.

### Rejected routes (receipts retained)

- **Tinygrad/WebGPU ACT**: all parameters transfer, then the forward graph fails
  at `cumsum(dtype=…)` in the 2-D positional embedding.
- **SmolVLA 450M action artifact**: loads and is deterministic, but 5.40–5.49 s
  per chunk against a 1.0 s deadline.
- **MolmoAct2 5.44B**: 21.77 GB F32; ≥10.88 GB even at hypothetical FP16 against
  8.02 GB total CPU memory.
- **Patched ROCm**: not attempted — needs kernel, firmware and driver changes,
  i.e. four pieces: the amdgpu/amdkfd flush patch, the navi12 SDMA blob, a native
  gfx1013 rocBLAS build, and PyTorch built from source. Trackers:
  `drm/amd#5597` (compute TLB), `drm/amd#5590` (SDMA).

### Exit (revised) — satisfied (2026-11-10, sim envelope)

- 11a: reproducible receipts for ingestion, harness, fit and thermal soak. **Complete.**
- 11b: control policy trained locally with throughput, memory, thermal, reload and held-out evidence (aggregate gate met; named move-family shortfall). **Complete.**
- 11c: vision path closed with exit-compliant measured evidence — (c)/(a) measured frozen/calibrated-geometry decisions meeting safety/evidence requirements in sim envelope with throughput/memory/thermal/reload/held-out receipts (bounded); (b) specific local route (B1 frozen geometry, given type hypothesis) measured and **rejected** for full 11c(b) with measured blocker and documented fallback (scoped local type-ID+pose training probe/hybrid). **Complete (exit-compliant).** Local services only; cloud training out of scope.
- Execution-runtime evidence alone does not close this phase; simulated success does not pass a physical gate.

Evidence: [PHASE11_RUNTIME_QUALIFICATION.md](PHASE11_RUNTIME_QUALIFICATION.md)
and [PHASE11_LOCAL_TRAINING_PATHS.md](PHASE11_LOCAL_TRAINING_PATHS.md).

## Phase 11d — live BC-250/Pi teleop and board-discovery demo

**Outcome:** one supervised bench session where the BC-250 is the only brain. It
receives the Pi's wrist-camera stream, switches the arm between leader teleop and
remote joint control, drives a scripted scan that finds all four board ArUco tags,
solves the chess-board pose in the arm-base frame, and serves a browser page on
the BC-250 showing camera, tags and the measured board offset.

**Why 11d and not 12:** 11c is closed, so this is genuinely next, and inserting a
lettered slice keeps 12–22's numbers and gates untouched. It is a vertical-slice
demo, not product acceptance: it borrows the registration math from 9, the
BC-250 runtime from 11, the UI shell from 13 and the transport contract from 15,
and **closes none of them**. It is the one planned exception to "no physical
deployment before 15/17" (Brian, 2026-11-15); it may move the real arm for
camera-only scan poses, never to touch a piece, and it does not replace 14's
release gate or 17's measured commissioning.

**Assumed present, no purchases:** BC-250 `192.168.1.171` (`ssh bazzite`,
CachyOS) with its gaming drivers/kernel/compositor untouched — the app runs in a
container or user-level venv, never system Python; Pi 4B `192.168.1.167`
(`robot_arm`, hostname `robot1`) with the existing LeLab autostart
(`lelab.service`, `so101-teleop.service`), C270 wrist camera at `/dev/video0` and
the `so101-viewer` service on :8099; SO-101 follower with the 30 mm base datum
and 90 mm board gap from `hardware/current_layout.json`; four 40 mm
`DICT_4X4_50` markers at ±152 mm on the mat (Option A in
[`BOARD_MARKER_LAYOUT.md`](BOARD_MARKER_LAYOUT.md)).

**Services (two, one bus owner).**

- Pi `so101-bridge` (:8100, FastAPI/uvicorn, user systemd unit): owns the mode
  arbiter, publishes joint state at 20–30 Hz, accepts bounded joint targets,
  enforces TTL/sequence/envelope, holds the local watchdog and e-stop latch, and
  is the single camera source. **v1 does not touch the Feetech bus directly** —
  LeLab stays the bus owner and the bridge drives it through LeLab's HTTP API.
  Verify `/move-arm`, `/teleoperation-status` and whether teleop can be stopped
  independently before coding; if LeLab cannot yield safely, that is a separate
  decision (bridge-owned lerobot loop), not a silent rewrite.
- BC-250 `robot_live` (:8080): pulls the camera **once** and fans out to browsers,
  runs ArUco detection and pose estimation, orchestrates the scan, solves the
  board pose, records the session, serves the GUI. (The `robot_web` browser service was retired 2026-11-25; use the headless tools.) Reuse the old same-origin
  POST guard, no-store headers and MJPEG pattern; do not expose to the Internet.

**Protocol** (`robot_protocol`, versioned module so Phase 15 inherits it):
`hello {v, roles, limits}`, `state {seq, t_ms, joints_deg[6], mode, bus_ok, cam_fps}`,
`command {v, seq, t_ms, ttl_ms ≤ 250, joints_deg[6], speed_scale ≤ 0.3}`,
`ack {seq, applied, clamped, err}`, `mode {teleop|remote|hold}`, `estop`.
Reject stale, duplicate, out-of-envelope and out-of-order messages; never replay a
queue after reconnect; all angles in degrees at the joint interface, raw servo
counts stay inside the Pi.

**Modes:** `TELEOP` (leader drives follower), `REMOTE` (bridge accepts BC-250
targets), `HOLD` (no new targets, bounded stop/hold — not torque-off),
`FAULT`/`ESTOP` (latched, needs an explicit re-arm). Exactly one owner at a time.
`teleop → remote`: stop the mirroring loop, ramp to hold over ~300 ms, ack, then
accept targets. `remote → teleop`: stop accepting targets, zero velocity, require
the leader to be near the follower pose, then hand back. Any fault, TTL miss,
link drop, Pi reboot or BC-250 restart lands in `HOLD` and requires re-arming,
never auto-resume.

**Scan for the four markers:** the wrist field of view cannot be assumed to hold
all four corner tags at once, so the scan is a set of poses, not one picture.
Generate 9–13 candidate poses (3×3 board grid plus edge mids, fixed height/tilt)
offline in the simulator, verify each for IK, joint limits and collision, then
store them as a versioned pose file. At runtime capture settle → detect → score
coverage; if a tag is missing, visit its predicted pose and re-detect. Fuse tag
corners into the base frame using FK at each capture timestamp, solve once ≥3
tags agree, and report which poses contributed. No closed-loop visual servoing in
this phase.

**Sim-first slice (2026-11-15).** The scan is the engineering content, so it is
planned and measured in the simulator before any live pose file is frozen. Phase
9's committed six-pose plan was selected for the 64 playing squares and, measured
on the current layout, sees only markers 2 and 3 — the near markers (ids 0 and 1
at -y) never appear in any of its frames. That is a coverage failure, not a
detector failure: every marker is individually resolvable from a pose above it.
`chess_sim/marker_scan.py` scores a pose by frame margin, projected marker edge
(a 40 mm, six-module marker needs ~42 px for ~7 px/module) and the authored-mesh
arm silhouette, so planner and runtime share one definition of "usable".
`tools/plan_marker_scan.py` solves wrist-lens targets over the marker ring and a
board grid, then greedily covers every marker twice while rejecting repeat views
from the same camera position. The committed plan is six poses, two independent
views per marker, projected edges 38.7–47.5 px (threshold 36 px): two overhead
views of the far pair (ids 2/3), and four near-side views at 47.5 px plus two
higher, laterally offset views at 38.7 px for ids 0 and 1. No single frame holds
all four: the near pair spans 304 mm and the wrist lens covers ~330 mm at that
height, so the pose set is the mechanism, not a workaround. Predicted marker
visibility matched the real ArUco detector on the rendered frames; in one frame
the detector also resolved id 2 that the conservative planner had not credited.
`tests/test_marker_scan.py` pins the projection, occlusion, diversity and offset
arithmetic, and guards the committed plan's completeness.
This closes the scan *planning* question only; capture, fusion, hand-eye
calibration and live offset reporting remain open. The near-side views carry the
open jaws across a large part of the frame; detection still resolves the markers
there, but a frame-clutter budget is the first tuning knob once live detection
rates exist.

**Board from base offset:** per tag, `solvePnP` with calibrated intrinsics gives
`T_cam_tag`; the board frame is defined by the known ±152 mm marker layout.
Eye-in-hand: `T_base_board = T_base_flange(q) · T_flange_cam · T_cam_board`, where
`T_flange_cam` comes from a one-time hand-eye calibration (`cv2.calibrateHandEye`
over 10–15 poses, refined against the CAD mount transform), and `q` is the
timestamped joint state, not the latest one. Report `x/y/z` of the board centre
relative to the base frame, yaw, residual RMS, tag count and pose age. The demo
also prints the nominal expectation so a mounting error is visible rather than
absorbed. Hand-eye calibration is a hard prerequisite: without it the offset is a
camera-frame number, not a base-frame one.

**GUI (BC-250 :8080):** live camera with tag overlay; teleop ON/OFF plus HOLD and
E-STOP; mode and link banner (RTT, FPS, staleness); board offset in mm/degrees;
tag table (id, corners, reprojection error, age); "Find board" button with scan
progress and coverage; joint readout and command latency; recording indicator.
Every control needs an explicit session "arm" click first.

**Safety gates:** speed ≤30% of teleop and a configured workspace box, both
enforced on the Pi, not just the GUI; z-guard above the board plane plus margin
so a scan pose cannot descend into the pieces; physical servo-power/e-stop within
reach, GUI stop secondary; hands-clear arming every session; no autostart into
`TELEOP` or `REMOTE`; every command and mode transition logged with timestamps;
one upstream camera pull only, or the Pi's USB bus and CPU become the bottleneck.

**Exit gates (proposed tolerances, fix before the run):**

1. Camera: ≥10 fps at 640×480 to the browser through a single upstream pull, added
   latency ≤300 ms, 30 min without a dropped stream or Pi thermal alarm.
2. Transport: 10 min remote session over Wi-Fi with zero executed
   expired/duplicate commands; Wi-Fi drop and BC-250 restart each produce `HOLD`
   and require re-arm; measured RTT and jitter recorded, with wired only if the
   Wi-Fi budget fails.
3. Teleop toggle: 10 clean round trips `TELEOP ↔ REMOTE` with no jump >2° at the
   handover and no bus contention error in the Pi log.
4. Detection: 4/4 tags found within 60 s from cold start on 5 consecutive runs;
   each tag's reprojection error recorded.
5. Board pose: repeated solves agree within ±2 mm and ±1° across ≥5 scans, and
   agree with a tape-measured base-to-board offset within ±5 mm.
6. Recovery: camera unplug, tag removal, Pi service restart and BC-250 service
   restart each surface a visible fault and never leave the arm moving.
7. Evidence: session recording (frames + joints + detections + commands), a
   receipt JSON, and a hand-off note; sim release checks untouched.

**Out of scope:** chess rules/engine, piece or occupancy recognition, grasp
planning, any manipulation, model training, reflashing the Pi, changing BC-250
drivers/kernel, purchases, and Internet exposure.

**Locked with Brian (2026-11-15):** the four tags are already fabricated and
mounted on the board, so the scan gate does not wait on Phase 16; the C270 is the
operational wrist camera and is integrated into the jaw; Wi-Fi is the first path,
with wired Ethernet recorded as a fallback measurement rather than a
prerequisite; teleop on/off is implemented by starting and stopping LeLab's
mirroring loop through its API, not by taking the bus. Both the Pi and the BC-250
were powered off when this was written, so the LeLab API check is the first action
once they are up.

**LeLab boundary — measured 2026-11-15 (LeLab 0.6.1, lerobot 0.6.1, Python
3.12.16, Pi up):** the follower is `ttyACM0` (`...5C93128208`), the leader
`ttyACM1` (`...5C93131956`), and the LeLab process holds both ports for as long
as teleoperation runs. `POST /move-arm` *starts teleoperation* — its body is two
ports plus two calibration names, not joint targets — so **remote joint commands
cannot pass through LeLab**. `POST /stop-teleoperation` sets the worker flag and
the worker disconnects both devices, which releases the ports. Joint state is
already broadcast at 30 Hz on `ws://127.0.0.1:8000/ws/joint-data` (what the
existing viewer consumes); the REST `/joint-positions` poll is the path that
previously killed the teleop loop. `/camera-feed/{cam_key}` exists only during an
active recording, so the bridge must own `/dev/video0` itself.

Consequence for the design: the bridge is the mode arbiter, but it delegates
TELEOP to LeLab and only takes the follower bus in REMOTE. Entering REMOTE stops
teleoperation and waits for `teleoperation_active` to clear. Returning to TELEOP
stops commanding, closes the follower, opens the leader read-only to compare its
pose with the follower's, and refuses to start teleoperation when they are more
than ~10 degrees apart — otherwise LeLab's mirroring loop would jump the wrist
and jaw to the leader's pose the instant it connects.

**Fusion gate measured (2026-11-15, BC-250).** `tools/benchmark_marker_fusion.py`
renders the committed plan against a displaced board, runs the real detector and
fuses through `estimate_board_pose`; no simulator camera or board transform
reaches the estimator. Results: the five named placements pass at 0.30–0.38 mm /
0.07–0.08°; the **±20 mm × ±20 mm × ±5° grid passes 27/27**; the ±40 mm grid
passes 26/27, the outlier being the far edge at +40 mm with zero yaw. Board-pose
gate is 2 mm / 1°.

Two findings worth keeping. First, **installation geometry goes stale**: the
Phase 9 calibration was exported for the 460 mm datum, and on the current
462.4 mm layout it produced a ~0.9 mm board bias and solver rejections at the
placement extremes. `tools/export_wrist_calibration.py` exports and verifies the
chain for the live scene (0.000000 mm reproduction error), and with it the bias
returned to ~0.3 mm. Regenerate calibration and plan together after any mount,
riser or gripper change. Second, the plan now carries per-marker **recovery
poses** from the scored candidates; they help a small miss, but they are aimed at
the nominal marker position, so a 40 mm placement error moves the marker out of
the recovery frame too — the live fallback for that case is a wide re-scan, not a
fixed pose. Set the user-facing tolerance at ±20 mm / ±5° and ask for
repositioning outside it.

**50 mm tolerant plan (2026-11-15).** `--tolerance-mm 50 --min-markers 3` seeds
the selection with the nominal six-pose set, then adds poses until every sampled
placement (±50 mm in 25 mm steps, ±5° yaw) sees at least three markers. Result:
11 poses, nominal coverage still two views per marker, ordered
nearest-neighbour in joint space. Measured with `benchmark_marker_fusion.py`:
**75/75 placements solve**, worst board error 0.51 mm / 0.12°, no recovery pose
needed. One placement misses the planner's 3-marker criterion (74/75) but still
solves from two.

Scan cost is dominated by ordering, not pose count: the six-pose tight plan
travels 649° of slowest-joint motion in its generated order, 399° reordered; the
11-pose tolerant plan travels 398° ordered. At the bridge's current 10.5°/s cap
(35°/s × 0.3) that is ~44 s plus 0.5 s settle per pose; at 35°/s it is ~17 s.
Raising the cap is a safety-envelope change, not a scan change. Candidate scores
are cached in `.private/phase11d/marker_scores.json`, so re-selecting a plan is
seconds instead of a 20-minute IK sweep.

**Scan controller and fake-bridge harness (2026-11-16).** `live/scan_controller.py`
closes the gap between the plan and the fusion math: it commands each pose
through the bridge, waits for arrival, requires the settle dwell, captures a
frame **with the joints that produced it**, rejects a stale pair, fuses one board
pose, falls back to per-marker recovery poses when fewer than two markers are
seen, and reports the offset in the base frame. It refuses to run unless the
bridge is already in REMOTE and armed, and it never changes the speed cap.
`live/bridge_client.py` supplies the `/state`, `/command` and `/capture` (JPEG
plus joint snapshot) adapters. `live/tests/test_scan_controller.py` runs eight
simulator-backed cases — nominal, displaced board, stale-frame rejection,
recovery after hidden markers, TELEOP and unarmed refusal, pose-timeout
reporting, and a degrees-vs-radians guard. 48 tests pass on the BC-250 (8
controller, 21 marker, 19 protocol).

Two things the harness proved that the bench has not: a stale frame is caught by
comparing the frame's joints with the bridge's current state, and a plan supplied
in the wrong units fails immediately instead of driving the arm to 270,000
degrees. Not yet exercised: `/capture` against the Pi — the Pi was offline when
this was written, so the live dry run is the next action.

**Transport verified end to end (2026-11-16, BC-250).** With the Pi switched
off, `live/tests/test_http_pipeline.py` runs the real `HttpBridge`/`HttpFrames`
clients over HTTP against a simulator-backed server that speaks the same API:
`/state` with limits, `/command` with sequence rejection, and `/capture` with a
base64 JPEG plus joint snapshot. The controller drove all 11 poses over the wire
and recovered the nominal layout offset (0 mm, 217 mm, 90 mm gap) within 2 mm.
28 live tests pass. Two unit traps were caught in the process and are now
guards: a radian plan sent as degrees is refused against the bridge's reported
joint limits, and the jaw is **not** a degrees conversion — sim radians map to
lerobot's 0..100 command, and that map's direction is still an unverified
convention to check on hardware. `/capture` itself remains untested against the
real Pi.

**Hand-eye solver and live console (2026-11-16, BC-250).**
`tools/solve_hand_eye.py --sim-validate` recovers `T_flange_cam` from rendered
marker views. OpenCV's closed-form `calibrateHandEye` was abandoned: it returned
a 180 deg solution even on exact synthetic data, because its camera convention
(+Z view axis) differs from this project's (-Z) and a planar target adds a mirror
ambiguity. The tool instead refines the CAD prior on pixel residuals; from a
prior perturbed by **10 deg / 15 mm** it reaches **0.11 deg / 0.11 mm**, and
`tests/test_hand_eye.py` pins that. One instructive bug is recorded in the code:
using the true camera rotation inside the residual leaves rotation unconstrained,
so the fit silently absorbs tool rotation into translation and still looks good.

`live/app.py` is the browser console: annotated camera stream, mode/joint state,
offset readout, and Arm / Remote / Hold / Teleop / Find-board / E-STOP. It holds
no motion authority; every POST carries the bridge token and all scan logic stays
in `scan_controller.py`. `live/tests/test_app.py` drives a full scan through the
page's own endpoint against the simulator-backed bridge. 51 tests pass on the
BC-250 (marker, hand-eye, protocol, controller, HTTP pipeline, app). Not yet
covered: the MJPEG `/stream` path, and any of it against the real Pi.

**Chosen defaults (Brian, 2026-11-16):** the 50 mm tolerant plan is the default
marker scan (`chess_sim/assets/marker_scan_plan.json`); the original six-pose set
is kept as `marker_scan_plan_tight.json` for comparison. Motion stays at the
conservative 10.5°/s cap (35°/s × 0.3) for the demo.

**Future phase — full-speed action qualification (Brian, 2026-11-16).** The scan
is ordering-bound, so raising command speed is the cheapest latency win: the same
11-pose scan is ~44 s at 10.5°/s and ~17 s at 35°/s, with a lower bound near 13 s
at the servos' real maximum. That is not a config flip: it needs measured
overshoot, settle time and stop distance per joint, a raise of the bridge's
`speed_scale` cap, and a re-qualified scan (higher speed changes where the arm is
when the shutter opens). Keep it as its own phase with its own gates rather than
folding it into this demo.

**Bridge v1 verified on hardware (2026-11-15).** `live/pi_bridge/` runs on
`robot1` (LeLab venv, port 8100, token-guarded): mode arbiter, bounded command
path, camera and joint streaming. Measured on the real arm: TELEOP → REMOTE
closed the follower and released LeLab; REMOTE → TELEOP passed the handover
guard and restarted LeLab; a bridge restart adopted an already-running
teleoperation instead of starting a second one; `+3°` shoulder-pan commands
moved the joint and returned within 0.18°; the C270 streamed 640×480 over MJPEG
with no contention against LeLab. One bug worth remembering: the per-cycle step
budget must accumulate against the last **commanded** goal, not the measured
position — a 0.23°/cycle ramp anchored on measurement is smaller than the servo
deadband and stalls forever, which is exactly what the first test did. The
bridge also never pulls a measured pose inward when it sits a degree outside the
calibrated envelope, and a 12° goal-vs-measurement lag guard stops a blocked
joint from running away. Installing `so101-bridge.service` needs a sudo
password; until then it runs under `setsid nohup` and does not survive a reboot
(LeLab's teleop autostart still does, which is the safe default).

**Risks:** hand-eye residual may not reach 2 mm with the CAD mount as the only
prior; the C270's integrated jaw mount fixes its near focus, so measure the
working distance before trusting tag corner precision; Wi-Fi jitter may force the
wired fallback sooner than expected; stopping and restarting teleoperation on
every mode change is a real usability cost that may justify a bridge-owned
teleop loop later.

## Phase 12 — train and compare goal-conditioned ACT

**Outcome:** choose the product manipulation controller on held-out task results.
Depends on 10–11; a justified no-policy decision retains the scripted controller.

- Collect simulated wrist/state/explicit-goal demonstrations using verified
  successes and synchronized executed actions. Split entire sessions/layouts;
  keep oracle FEN/object poses/verdicts as labels/evaluation only. Never export
  private raw assets/datasets by tutorial defaults.
- Start with bounded final-approach/grasp/place skills, expand by piece, approach,
  crowding, offsets, capture and tray. Size data using learning curves and bucket
  coverage, not a promise that 50 episodes covers chess. Store true failures and
  linked corrections separately from imitation-positive demonstrations.
- Train ACT on the qualified BC-250 route; audit wrong-piece, grasp, lift, transit,
  release and collateral failures. Shorten/replan action chunks based on measured
  latency/drift. Incident state-only traces need synchronized images before
  visual DAgger. RL/batched rollouts are conditional on a specific demonstrated
  residual problem, rewards/resets and local runtime support.
- Compare scripted control, ACT and any justified VLA on the same untouched
  ≥300-case positions/rollout seeds and identical success criteria. SmolVLA needs
  a new localization/conditioning/data hypothesis before spending another run.
- **Exit:** inference-only artifact + processor/goal/action config, exact reload,
  local resource/thermal receipt and paired per-family/type/crowding scores.
  Adopt learned control only if it meets Phase 10/14 gates and provides a measured
  benefit; otherwise record rejection and ship the qualified scripted path. If
  BC-250 training fails, leave ACT training blocked/rejected—do not silently move
  it to cloud or claim the tiny pilot qualified it.

## Phase 13 — integrated simulated chess application

**Outcome:** a usable end-to-end game with the selected controller. Depends on
9–10 and controller decision in 12.

- Implement coordinator states: setup-confirmed, await-human, scan, resolve,
  engine-plan, execute, verify, recover, paused and game-over. Wire perception
  and move inference into the browser UI; show uncertainty, phase and reasons.
- Use Stockfish/UCI and python-chess for legal moves/history. Separate planned,
  executed and committed positions; use transaction IDs and observation versions.
  Captures remove the opponent before placement; castling/en passant have explicit
  multi-action plans and partial-failure reconciliation. Human-assisted promotion
  must verify exchanged type before resuming.
- Add PGN/session save/load with turn, castling, en-passant and interrupted-action
  state; never infer historical rights from images alone. Provide turn-complete,
  stop, scan/retry, correct-position and resume controls.
- **Exit:** scripted integration scenarios cover ordinary turns, every special
  rule, invalid human moves, promotion choice, save/resume and partial transaction
  failure. Play full simulated games with wrist-only observations and no hidden
  oracle inputs to the operational coordinator.

## Phase 14 — recovery and simulation release candidate

**Outcome:** qualify the whole core product before connecting physical hardware.
Depends on 13. Single-move success is not full-game reliability.

- Inject missed picks, slip, fallen/displaced neighbors, board movement, hand
  intrusion, bad/old frames, model timeout, engine/service crash, pause/stop and
  interrupted capture/castling. Reconcile visually; retry only bounded safe
  operations, otherwise require explicit human correction/re-arm.
- Run an untouched ≥20-game suite, including dense openings and seeded special-
  move scenarios, plus ≥500 robot transactions across sessions. Proposed target:
  ≥95% games completed without an unplanned intervention, ≥99% successful
  transactions, zero undetected board-state divergence or wrong-piece actions;
  every injected fault detected and contained. Report interventions and retries.
- **Exit:** versioned simulation release, per-game PGNs/incidents, regression
  receipts and sustained BC-250 inference under representative load. Every
  required gate passes; unresolved geometry/vision/compute failures block hardware
  integration. Existing browser/control/crowded-v3 release checks remain required.

## Phase 15 — Pi / BC-250 boundary using fake hardware

**Outcome:** production transport software tested without moving the real arm.
Depends on 14; all captures/feedback/servos are mocked or simulated.

- Define versioned snapshot/joint/command schemas with frames, units, timestamps,
  sequence IDs, expiry, bounded segments and acknowledgements. Pi-side executor
  owns queue limits, local watchdog and stop/hold; BC-250 owns planning/models.
- Exercise fake USB/camera loss, stale/duplicate/out-of-order commands, Wi-Fi
  delay/drop, BC-250 sleep/reboot, queue exhaustion and explicit reconnect/re-arm.
  Reconnection never resumes an interrupted move by itself.
- **Exit:** repeatable two-service simulated acceptance suite and latency budget;
  zero expired/duplicate command execution, deterministic local watchdog behavior,
  no raw conversational/model output reaching servo commands. ARM64 packaging
  can be built/tested without opening actual serial devices. Physical deployment
  starts only at 16; this phase does not authorize a Pi install or reflash.

## Phase 16 — fabricate the physical board and piece set

**Outcome:** the physical board and all 32 pieces exist to the frozen twin's
dimensions, with markers printed and ballast fitted, so the commissioning in
Phase 17 calibrates a known object instead of a prototype. Needs no software, so
it can run in parallel with Phases 9–15; it must finish before 17.

- Freeze the board print: 28 mm squares, 15 mm labelled border, 3.2 mm slab.
  Choose the marker placement from [`BOARD_MARKER_LAYOUT.md`](BOARD_MARKER_LAYOUT.md)
  — mat-mounted at ±152 mm, or border-mounted only after the mount
  re-qualification that a wider border requires.
- Print the full 32-piece set at the recorded scale, fit the M6 ballast, then
  weigh every type and record mass, centre-of-mass height and upright
  stability. The twin currently has one measured type (rook, 4.8 g including
  its nut) and explicit ranges for the rest; replace the ranges with
  measurements and update the twin.
- Dimension check against `hardware/product_twin.json` and
  `hardware/current_layout.json`: square pitch, slab thickness, overall size,
  marker centres, quiet-zone width, piece footprint and height.
- QC the printed parts: board flat and unwarped, pieces stand unaided, M6
  thread starts flush at the pocket floor, labels legible, no stringing in the
  gripper contact band.
- Verify the marker sheet decodes at the planned scan heights, with the chosen
  camera once it exists, otherwise with a phone camera and the saved
  intrinsics.
- **Exit:** measured board and piece dimensions inside the twin's tolerance; all
  32 pieces assembled and weighed; markers decode at 6+ px per module; a
  measurement/photo receipt per item. Physical jaw fit and calibration stay in
  Phase 17.

## Phase 17 — physical commissioning and measured calibration

**Outcome:** validate the selected fixture, wrist camera and local executor on
real hardware. **Hard prerequisite: 14–15 passed.** Existing LeLab assembly and
teleoperation remain self-reported; earlier bench gates are still required.

- Verify single-arm fixture, stop/hold, leader-disconnect, limits/speeds, power,
  USB topology and cable strain relief. Measure jaw fit/backlash/aperture, camera
  clearance, weighted masses/friction and all required workspace/tray paths.
- Calibrate joints, lens/distortion, hand–eye/tool contact, board plane/origin/
  orientation/height and grasp offsets. A wrist image does not recalibrate joint
  zeros. Compare measured frames to sim; test low-confidence registration refusal.
- Deploy the tested Pi executor/camera service and BC-250 coordinator with explicit
  hardware enable. Validate local stop/watchdog/network loss before task moves.
- **Exit:** saved calibration/versioned physical config, independent measured
  pose/registration checks within the Phase 9/10 error budget, safe fault tests,
  repeated light-object picks and full empty-board path verification. Return
  discrepancies to the simulator instead of compensating with oracle constants.

## Phase 18 — guarded real pick/place and adaptation

**Outcome:** transfer the simulated skills to actual pieces. Depends on 16.

- Begin one piece / empty board at conservative motion settings, then all types,
  offsets, tray captures, dense neighbors and special-move sub-actions. Verify
  lift and placement from real wrist images before committing state.
- Keep real capture sessions separate from synthetic data; audit lens/exposure,
  timestamps, synchronization and domain shift. Fine-tune perception/ACT only if
  failure evidence supports it, using the qualified local training path and
  held-out real sessions. Re-evaluate the exported artifact on BC-250.
- **Exit:** held-out ≥100 physical transactions across all six types and move/
  capture families meeting the Phase 10 target, zero wrong-piece/collateral
  actions and no undetected failure. Every fault stops/reconciles correctly;
  failed gates return to simulation/data refinement, not longer live games.

## Phase 19 — physical complete games and final acceptance

**Outcome:** the final working core chess robot. Depends on 17.

- Repeat the complete human-turn → wrist observation → legal inference →
  Stockfish → verified execution cycle in ≥10 real games across sessions;
  explicitly exercise castling, en passant, assisted promotion and save/resume.
- Verify ≥95% unassisted completion target over a rolling ≥20-game acceptance
  window (the first ten are a commissioning checkpoint), zero undetected state
  divergence, bounded recovery and logs sufficient to reproduce every incident.
  Include long-session thermals, Wi-Fi loss, stale frames and BC-250 sleep tests.
- **Exit:** documented supported board/piece/layout envelope, launch/stop/recover
  workflow, calibration backup, model/runtime versions, game receipts and known
  limitations. No dependence on a second arm, debug overhead camera, Internet
  service or unqualified training stack for normal play.

## Phase 20 — autonomous setup from loose pieces

**Outcome:** inventory, identify, grasp and arrange all 32 pieces. First qualify
software in simulation; physical rollout depends on 18 and the same new-feature
gates as 17–19. This is not required for the human-confirmed-setup core release.

- Start with upright separated, unsorted pieces in a reachable staging area;
  recognize type/color (shape versus visible identifiers is a measured decision),
  assign destinations, plan access-preserving placement order and verify inventory.
- Then explicitly qualify sideways, touching/overlapping and stacked pieces:
  singulation/reorientation and tool feasibility are new work, not silently
  excluded requirements or capabilities implied by the upright increment.
- **Exit:** ≥20 held-out complete setups per claimed supported increment, no
  missing/duplicate/wrong-type placements left undetected, with success rate,
  interventions and unreachable/unknown-object refusal reported.

## Phase 21 — screen avatar and bounded conversation

**Outcome:** game-aware display on an existing screen first, no purchase implied.
Prototype/qualify with simulated game events before hardware rollout.

- Feed trusted coordinator events/evaluation to expressions and dialogue; keep
  chess facts/rules authoritative and generated text out of motor commands.
- **Exit:** event/replay tests for normal play, uncertainty, recovery and game-over;
  display/model failures never stall or override the chess/motion coordinator.

## Phase 22 — safe game-aware arm emotes

**Outcome:** a collision-checked empty-gripper gesture library. Qualify simulation
first; physical rollout requires measured gesture volume and Phase 19 readiness.

- Coordinator arbitrates gestures against task motion; require hands clear,
  empty tool, local limits/watchdog, cooldowns/hysteresis and robot-perspective
  evaluation triggers. Re-observe after moving the wrist camera.
- **Exit:** every gesture passes collision/fault/preemption tests and measured
  physical clearance before activation; no gesture during a pick, uncertain state
  or interrupted transaction. Personality never gains raw servo authority.

## Experiment lessons carried forward

The chronological log is retained unchanged; this index turns its lessons into
future gates. Historical upstream RTX 5090/two-camera results are **not** local
BC-250 or wrist-only qualifications.

- [EXPERIMENTS §4.1–4.6, §5.1–5.5](EXPERIMENTS.md): falling training loss is not
  task success; audit next-state/action-label conventions and command-rate
  interpolation. Narrow instruction success does not prove wide grounding.
  Historical 6,095 demonstrations/70k steps and 47-hour 5090 run are not an ACT
  budget estimate. Carry into 11–12.
- [§4.7–4.10, §5.6–5.7, §6](EXPERIMENTS.md): 227/300 baseline versus 233/300
  correction checkpoint is not a blanket improvement; use paired seeds and
  confidence/failure breakdowns. More captures did not fix blind action chunks;
  transit-only noise omitted grasp/release corrections and success filtering hid
  hard cases. Preserve schedules on resume; record synchronized corrections from
  actual failure states. Carry into 10–12 and 14.
- [§4.11–4.14](EXPERIMENTS.md): SmolVLA ended at 1/48 on the tested recipe;
  wide-distribution MolmoAct2 attempts collapsed/failed to take off. Avoid repeating
  long runs without new grounding/data/runtime evidence. Carry into 11–12.
- [§7–10](EXPERIMENTS.md): preserve preflights, media/data audits, exclusive
  worker ownership, incident archives and classified contact diagnostics. ±7–9 mm
  board shifts caused destination-jaw neighbor collision; only ±5 mm was qualified.
  Tool geometry/clearance failures need geometry changes, not more demonstrations.
- [§11](EXPERIMENTS.md): scripted expert 332/360 is an oracle-driven ceiling on
  prefiltered sparse tasks; printed captures 73% versus default 98% exposes a real
  piece-set gap. Fixed-suite 100% is not full-board/full-game reliability. Phase 10
  must remeasure weighted printed pieces and the single-arm product layout.
- [§12–13](EXPERIMENTS.md): 5/8 registration sessions accepted; top perception
  99.4% answered accuracy and 96% coverage exclude occlusion/rejected sessions.
  Synthetic-occluder rejection was only 22.5%; move coverage 37.1%. Wrist fusion
  reduced it to 31.4%, so extra views need qualified selection/consistency, not
  blind fusion. Carry into 9, 13–14.
- [BC250_LOCAL_TESTS](../training/BC250_LOCAL_TESTS.md) and
  [Phase 5 acceptance](../training/PRIORITY5_ACCEPTANCE.md): paced frozen-VLM
  inference and tiny actual-task GPU optimization pass, but the MLP loses to
  hold-position. Continuous thermals, real ACT/VLA operators/training and action
  accuracy remain open. Shared RAM, measured latency and thermal stability govern
  selection; no stock ROCm/PyTorch support is assumed.
- [GRASP_PRINTED](GRASP_PRINTED.md), [PRINTED_PIECES](PRINTED_PIECES.md),
  [REACH](REACH.md), [PRIVATE_TOOL_PACK](PRIVATE_TOOL_PACK.md) preserve per-piece
  geometry, imported scale, reach assumptions and private CAD provenance. Added
  M6 ballast invalidates the unweighted mass assumptions. Carry into 8, 10, 16.

## Release and evidence rules

Documentation-only refactors do not require runtime rebuild/deploy. For runtime
changes use repository AGENTS.md: local regression, versioned bundle/compose/
checksums, deploy only authorized `robot_arm`, verify exact live version,
cameras/controls and complete crowded-v3 fixed suite. Repeatability claims require
all three bounded seeded suites and per-case failure archives. Future product
qualification additionally requires the wrist-only/single-arm and game gates above;
the legacy crowded suite is necessary, not sufficient.

Public git contains code, docs and compact nonprivate receipts only. Keep private
CAD, raw recordings, credentials and checkpoints out of public commits. The log
and historical acceptance reports preserve their original names/numbers; current
status and phase numbering come from this roadmap, not stale historical prose.
