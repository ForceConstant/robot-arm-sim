# SO-101 bridge (Phase 11d)

One process owns the follower bus in REMOTE mode and delegates TELEOP to LeLab.
It exists because LeLab 0.6.1 has no joint-target endpoint: `/move-arm` starts
teleoperation from a port pair and a calibration name, nothing else. See
`docs/ROADMAP.md` → Phase 11d for the measured boundary.

## Modes

| Mode | Bus owner | What moves the arm |
| --- | --- | --- |
| `HOLD` | nobody, or bridge holding last goal | nothing |
| `TELEOP` | LeLab (leader + follower) | the leader arm |
| `REMOTE` | bridge (follower only) | bounded joint commands |
| `FAULT` | bridge holding, latched | nothing until `/arm` |

Start-up is `HOLD`. The follower is opened only when entering `REMOTE`.

## Safety rules encoded here

- Joint targets are clamped to the calibrated envelope (per joint, 2° inside the
  mechanical stops) and to a per-cycle step budget; a far target is walked in.
- Commands carry `ttl_ms` (40–400) and a monotonic `seq`; stale targets hold the
  last accepted pose, duplicate/replayed sequences are rejected.
- `speed_scale` is capped at 0.3.
- The worker thread owns all serial I/O; HTTP requests never touch the bus.
- Losing the camera is not a motion fault; a bus exception latches `FAULT`.
- Returning to `TELEOP` requires: follower closed, leader read once, leader
  within `SO101_HANDOVER_DEG` (default 10°) of the follower, then LeLab starts.

## API (LAN only, `x-bridge-token` header required on POSTs)

```
GET  /health            -> {ok, mode, armed}
GET  /state             -> mode, joints (deg), limits, counters, ages
POST /arm               {"confirm": true}
POST /mode              {"mode": "HOLD"|"TELEOP"|"REMOTE"}
POST /command           {"v":1,"seq":n,"ttl_ms":150,
                         "joints":[pan,lift,elbow,flex,roll,gripper],
                         "speed_scale":0.2}
POST /estop
GET  /joints            WebSocket, 20 Hz state frames
GET  /camera.mjpg       MJPEG from /dev/video0 (opened only while watched)
GET  /camera.jpg        last frame
```

Joint order is `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll, gripper`; gripper is 0–100, everything else degrees.

## Run

```
# foreground
SO101_TOKEN_FILE=/home/pi/so101-bridge/token.txt \
  /home/pi/.local/share/uv/tools/lelab/bin/python /home/pi/so101-bridge/app.py

# service (needs sudo)
sudo cp so101-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now so101-bridge
```

Do not `enable` it until the demo is signed off: with LeLab's autostart also
enabled, a reboot leaves TELEOP running and the bridge in `HOLD`, which is safe
but confusing in the GUI. Pick one boot behaviour deliberately.

## Deliberate limitation — motion speed

`MAX_SPEED=35` deg/s with `speed_scale <= 0.3` caps joint motion at **10.5°/s**.
That is the demo default on purpose: a scan pose that overshoots is a collision,
and the arm is slow enough to watch. Raising it toward the servos' real maximum
is a separate qualification (overshoot, settle time, stop distance, re-qualified
scan), tracked in `docs/ROADMAP.md` under the full-speed action phase.

## Tests

`pytest -q live/tests/test_protocol.py` covers parsing, envelope clamping and
the step budget; it does not touch hardware and runs anywhere.
