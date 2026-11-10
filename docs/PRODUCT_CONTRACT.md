# Product digital-twin and acceptance contract

**Historical baseline, not the current runtime mount/tool decision.** See the
[2026-11-03 updated-offset Phase 8 rerun](PHASE8_UPDATED_OFFSETS.md) before
recommending gripper changes. The dated config below is preserved for regression.

**Frozen 2026-11-02. Phase 8.** The machine-readable source is
[`hardware/product_twin.json`](../hardware/product_twin.json); the loader and
validators are in [`chess_sim/product.py`](../chess_sim/product.py). The
qualification evidence is [Phase 8 qualification](PHASE8_QUALIFICATION.md) and
[`reports/phase8_product_qualification.json`](../reports/phase8_product_qualification.json).

This contract is for **one physical follower**. The two-arm scene remains a
debug/regression fixture; it is not evidence that one arm can cover the board.

## Measured scene

| Item | Frozen value | Source |
| --- | ---: | --- |
| Square pitch | 28.0 mm | Brian, IRL 2026-11-02 |
| Playing field | 224 × 224 mm | 8 × pitch |
| Border | 15.0 mm, labelled | Brian, IRL 2026-11-02 |
| Board thickness | 3.2 mm | Brian, IRL 2026-11-02 |
| Board overall | 254 × 254 mm | field + two 15 mm borders |
| Table top | 430.0 mm | `hardware/dimensions.json` |
| Arm riser | 60.0 mm | `hardware/dimensions.json` |
| Board edge to arm base | 80.0 mm | `hardware/dimensions.json` |
| Arm base from board centre | (0, −207) mm | edge + half overall |
| Tray | 4 slots, 40 mm pitch, first at (−160, −100) mm | `hardware/dimensions.json` |
| Control rate | 30 Hz | `ControlConfig` |
| Operational camera | Wrist only | product requirement |

The 15 mm border is assumed equal on all four sides with no additional outer
trim. If the physical board differs, the config and this contract must change
together.

## Tool decision

The frozen product tool is the **stock SO-101 pinch gripper**. The NormaCore
custom PLA parallel jaw is retained as a rejected candidate for this 28 mm
board: its Phase 8 path map failed 11/64 squares and spot executions failed at
`e1`, `c1`, `d1` and `h8`; the stock tool passed the same checks. This is a
mechanical-envelope decision, not a claim that the stock tool is calibrated for
force or friction. Reconsider the custom tool only after a revised jaw passes
the same matrix.

## Units and frames

- **World:** metres, right-handed, +z up. Board centre at `BoardConfig.origin`;
  the arm sits on the −y side.
- **Board transform:** `square_center(square)` is the analytic centre of the
  playing square. `board.top = table_top + thickness`; `board.yaw` rotates the
  board about its centre. Phase 8 fixes origin `(0, 0)`, yaw `0`.
- **Arm joints:** six values in this exact order:
  `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`,
  `gripper`.
- **Actions:** six-element vector, radians. The first five are arm joint
  targets; the sixth is the gripper joint angle, converted to a jaw gap by
  `chess_sim.gripper.gap_to_angle`.
- **Observation state:** six joint positions and six joint velocities, radians;
  `Observation.time` is MuJoCo simulation seconds. Images are `uint8` H×W×3;
  the product operational camera is `wrist`.
- **Dataset export:** `chess_sim.conventions.SO101_DEGREES` (degrees with the
  documented signs and 0/90/90/0/0/0 offsets). Simulator actions stay radians.
- **Camera:** stock wrist pose `(-75, 45, 12)` mm in the gripper frame, target
  `(0, 0, 12)` mm, 75° vertical FOV, 640×480. This is a candidate CAD pose, not
  a physical lens/focus calibration.
- **Timestamps:** the recorder samples at `control_hz` (30 fps); one action per
  control period. No wall-clock timestamp is inferred from a frame index.

## Mass model

Brian measured all six **final assembled** piece types on 2026-11-04: king
5.3 g, queen 4.8 g, knight 4.5 g, rook 4.3 g, pawn 3.8 g, bishop 4.1 g. The
rook measurement supersedes the earlier 4.8 g value. Qualification uses these
measured totals as nominal, plus separate uniform **−0.1 g** and **+0.1 g**
sensitivity strata; those bounds are uncertainty assumptions, not additional
measurements. The unweighted mesh is combined with a point ballast at z=3 mm to
derive COM and inertia, because the actual ballast distribution was not
measured. Do not describe those derived tensors as measured.

## Supported perturbations and evaluator

Phase 8 supports board origin ±5 mm per axis and arm-start ±0.1 rad per joint.
Board yaw, sideways placement, camera-pose variation and unseen board relocation
are deferred to Phase 9. The success rule is `chess_sim.config.ToleranceConfig`,
used by `ChessSimEnv.evaluate` for both the scripted expert and policies:

- move placement error < **11 mm**, piece upright (`z` alignment ≥ 0.85), and no
  other piece moved > **10 mm**;
- capture leaves the playing field, lands within **30 mm** of its tray slot,
  stays upright and disturbs nothing;
- failure reasons remain the published `FailureReason` values. Do not add a
  forgiving success label.

## Interfaces frozen by Phase 8

| Interface | Entry point |
| --- | --- |
| Scripted control | `chess_sim.controller.PickPlaceController.pick_place` |
| Episode API | `ChessSimEnv.reset` / `.execute` / `.evaluate` |
| Observation | `chess_sim.env.Observation` |
| Action | six-element vector in `JOINTS` order |
| Recorder | `chess_sim.recorder.EpisodeRecorder` |
| Board registration | `chess_sim.board_registration` (Phase 9 input) |
| Perception | `chess_sim.perception` (Phase 9/10 input) |
| Move inference | `chess_sim.move_inference` (Phase 9 input) |
| Policies | `chess_sim.policies`, `chess_sim.rollout` (Phase 11/12) |
| Game state | `python-chess` board; coordinator/UI is Phase 13 |

Regression IDs and commands are in [PRODUCT_TEST_MATRIX.md](PRODUCT_TEST_MATRIX.md).
