# Board registration — 2026-11-27

Ran the per-session board registration. Result is usable but **the mat is not back
at the pinned heading yet**: the measurement puts it ~10° off.

## What ran

1. Eight poses (wrist-scan postures, rolls 0/±45, pan ±46), markers 6–12 in view.
2. Ten more on a pan/lift/elbow/flex grid, markers 4–13 in view.
3. `tools/calibrate_board_charuco.py --layout charuco` over all 18 captures.

```
143 sightings over 18 captures
  x_mm     -11.50   spread 27.62     (min -49.4, max 5.8 on the 8-view pass)
  y_mm     -37.72   spread 27.32
  z_mm     423.77   spread 20.77
  yaw_deg  -165.50  spread  2.42
```

Stored `live/board.json`: x −27.62, y −31.63, z 432.65, yaw 184.32.

**Yaw is the reliable number here** — 2.4° spread and identical to two decimal
places across the 8-view and 18-view runs (−165.49 and −165.50). Against the
stored 184.32 that is **10.18° of mat rotation**, so the mat is still ~10° from
the pinned heading (it was ~21° before Brian nudged it back).

## Why the translation spread did not improve with more views

It went 19.2/28.0/18.3 mm on 8 views to 27.6/27.3/20.8 mm on 18 — it did not
shrink, so it is not sampling noise. It is the same per-view pose scatter measured
earlier (board-pose displacement median 16.3 mm, p90 26.6 mm on the 23-view wide
volume): rotations agree between views, translations do not.

The medians are stable across runs to 1–2 mm, so the *median* is a better estimate
than its spread suggests — but a ±25 mm per-view scatter is not a 9 mm picking
frame, and it should not be mistaken for one.

## Recommendation

Do not write this into `live/board.json` yet. Two reasons: the heading is ~10° from
where Brian intends to leave the mat, and pinning a pose whose own spread is 27 mm
would put a soft number behind a hard hash.

Sequence that closes it: mat returned to the intended heading → register (this
takes ~4 minutes with the two batches already scripted) → write `live/board.json`
→ re-pin `rig_state/board.json`, `MANIFEST.json` and the `BOARD_EXPECTED` literal →
`test_frame_pins`.

## One structural question for Brian

`board.json` is currently in the pinned set, but it is a **per-session
measurement** — so every legitimate session re-registration trips the frame-pin
test until it is re-pinned. Either:
(a) keep pinning it and accept a re-pin step each session (current practice), or
(b) drop it from the pinned set and check instead that a registration happened
    this session (mtime/marker), keeping the pin for genuinely constant files
    (`live_config`, `wrist_calibration`, mount calibration).

The invariant "board placement is per-session" argues for (b); the manifest's
stated purpose ("a wrong value moves every commanded square and fails silently")
argues for (a). Worth deciding once rather than per session.

## State

Arm homed before and after both batches; gripper held at its measured value and
never actuated. `live/board.json` unchanged. Captures and the report JSON are on
the BC-250 (`live/captures_boardreg_20261127/`, `reports/board_charuco_20261127.json`).
