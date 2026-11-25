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
