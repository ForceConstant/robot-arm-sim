# WristScanner live wiring — 2026-11-25

Endpoint: `POST /board/scan` on the live console. It is additive: `/scan`
keeps the marker plan, the aim overlay keeps `TargetDetector`, and no move
legality uses the labels yet.

## What was wired

- `live/wrist_scan_runner.py`: `plan_pose_degrees` (sim plan -> bridge degrees;
  wrist_roll = sim - WRIST_ROLL_ZERO_SIM_DEG, jaw 0..100) and
  `frames_from_captured` (bridge degrees -> scanner radians).
- `live/app.py`: `Live.start_board_scan` / `_board_scan_worker`. Homes first,
  holds `motion` for the session, runs `ScanController` over the wrist plan,
  holds one `/stream` client so the bridge keeps producing frames, runs
  `run_scan(references=None)`, publishes `board_scan`, `board_scan_running`,
  `board_scan_error` in `/state`, homes afterwards.
- `live/scan_controller.py`: keeps accepted frames (`captured`) and retries a
  failed `capture()` instead of aborting the visit.
- `chess_sim/wrist_calibration.py`: the FK limit guard tolerates 1.5 deg past a
  chain limit (the plan can sit on a stop and the servo settles slightly past
  it; 95.8 vs the 95.0 chain limit rejected a valid frame).

## Live runs (arm homed before and after; no gripper command beyond the plan)

| run | result |
|---|---|
| 1 | reached pose 3 then `ValueError: measured joint outside calibrated hard limits` (flex 95.8 vs chain 95.0) |
| 2 | reached pose 10 then `503 no camera frame within 6 s` (no stream client attached) |
| 3 | completed all 12 poses; `[fusion] reject rms=66.961 gate=1.5 views=12` → registration failed, no labels |

The pose sequence, roll conversion (live 0, ±67.5, ±90), capture pairing and
endpoint all work. The remaining blocker is the board-pose fusion on the real
wrist frames: 27 markers across 12 views, RMS 67 px against a 1.5 px gate,
while a single-frame PnP at home gives RMS 1.25 px. That points at the camera
mount/FK used by the fusion or a bad outlier view, not the layout.

Side effect: the plan carries jaw 0.6 rad, which maps to a fully open gripper
(99.7); the arm is left homed with the jaw open.

## Frame capture and the fusion diagnosis (second session)

`/board/scan` now saves every accepted frame plus `manifest.json` under
`live/captures_board_scan/<stamp>/`, and `/state` exposes `board_scan_dir`.
A twelfth run reproduced the failure with the frames saved.

Per-view PnP on those frames (CharucoLayout, face plane 432.65 mm):

- 11 of 12 views solve at **0.7–2.3 px** rms.
- `v12` (2 markers, low view) solves at **50.7 px** — a false/mis-associated
  view that alone took the 12-view fusion from 67 px and blocked it.
- `chess_sim/fiducials.estimate_board_pose` now pre-screens a view by its own
  planar PnP and drops anything above 5 px. Fusion with that filter: 11 views,
  **33 px rms** — still above the 1.5 px gate.

Transforming each view's camera-frame board pose into the arm base frame with
the live calibration gives:

| view (roll) | board x,y,z mm | yaw |
|---|---|---|
| v1 (0.3) | −84, −50, 4.7 | −173.4 |
| v4 (0.0) | −89, −44, 4.4 | −173.3 |
| v8 (0.3) | −39, −77, −10.8 | −172.8 |
| v9 (0.3) | −127, −23, 18.3 | −172.9 |
| v6 (90) | +26, −112, 4.1 | −172.4 |

The yaw is consistent (4° spread) but the position scatters ~48 mm, and the
scatter does not respond to focal-length scaling (0.85–1.1 all ~48 mm). A
single per-view pose is self-consistent, so this is the **FK calibration**
(chain/camera mount) not matching the arm pose-by-pose, not the scan code.
Phase 9's mat-based registration did not expose it.

## Status

Wiring: complete and exercised (12 poses, correct roll sequence, frames saved,
endpoint and state fields). Blocked on: re-deriving/validating the live chain
and camera mount against these frames (hand-eye bundle or a board-position
survey). No labels are published until the fusion gate passes.
