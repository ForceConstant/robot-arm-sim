# SO-101 bridge (Phase 11d)

One process owns the follower bus in REMOTE mode and the leader bus in TELEOP.
It exists because LeLab 0.6.1 has no joint-target endpoint. This revision is
deliberately two-state: the HOLD trap and the FAULT latch were removed because
the owner secures the arm by switching power off, so a state no command could
leave only made the bridge unusable.

## Modes

| Mode | Bus owner | What moves the arm |
| --- | --- | --- |
| `REMOTE` | bridge (follower) | bounded joint commands from `/command` |
| `TELEOP` | bridge (leader + follower) | the leader arm, mirror-by-joint |

There is no third state. Boot enters `REMOTE` and holds the pose. `POST /mode`
switches; `HOLD` is accepted as an alias for `REMOTE`.

## What is still enforced

- Joint targets are clamped to the calibrated envelope (per joint, 2° inside the
  mechanical stops; wrist_roll at the full ±90° window) and to a per-cycle step
  budget; a far target is walked in. A joint that would lead the measured pose
  by more than `SO101_MAX_LAG` is bounded, so a backlog cannot wind up.
- `speed_scale` is capped at 0.3. TELEOP is capped at `SO101_TELEOP_SPEED`
  (default 35°/s).
- Commands carry `ttl_ms` (40–400). A stale target holds the last accepted pose;
  it never queues and never replays after a reconnect. **A dead client fails
  safe to a hold.**
- The worker thread owns all serial I/O; HTTP requests never touch the bus.
- `x-bridge-token` on every mutating endpoint.
- Losing the camera is not a motion fault.

## What was removed

| Removed | Why | What replaced it |
| --- | --- | --- |
| `HOLD` as a state | A state you had to be let out of | REMOTE with no fresh command holds the pose |
| `FAULT` latch | Only a power cycle (or an `/arm` that needed REMOTE) cleared it | `error` / `estop` reports that still accept commands |
| `/arm` as a precondition | Nothing needs arming; both modes take input | `/arm` kept as a no-op compatibility call |
| Handover alignment gate (`SO101_HANDOVER_DEG`) | One out-of-spec joint refused the whole handover | per-joint click-in (`SO101_CLICK_IN_DEG`) |
| LeLab `/move-arm` delegation + joint proxy WS | LeLab mirrors the instant it connects, defeating click-in | the bridge opens and reads the leader itself |
| `seq must be newer` rule | Permanently wedged any client whose counter fell behind (63M gap, 2026-11-24) | any `seq` accepted; freshness is arrival time |
| E-stop latch | Stopped the arm until a power cycle for no safety gain | `/estop` holds the pose; the next command resumes |

## TELEOP handover: per-joint click-in

Requesting `TELEOP` takes control at once. Every joint within
`SO101_CLICK_IN_DEG` of the follower follows the leader immediately. A joint
further out holds its pose and joins in on the first cycle the leader brings it
within that radius. `/state` reports `teleop_engaged` and `click_in` so the
console can show which joints are still waiting.

## API (LAN only, `x-bridge-token` header required on POSTs)

```
GET  /health            -> {ok, mode, estop, error, camera}
GET  /state             -> mode, joints (deg), limits, counters, click_in, ...
POST /mode              {"mode": "TELEOP"|"REMOTE"|"HOLD"}  (HOLD = REMOTE)
POST /command           {"v":1,"seq":n,"ttl_ms":150,
                         "joints":[pan,lift,elbow,flex,roll,gripper],
                         "speed_scale":0.2}
POST /arm               {"confirm": true}   (compatibility; nothing to arm)
POST /estop             hold the pose; next command resumes
GET  /joints            WebSocket, 20 Hz state frames
GET  /camera.mjpg       MJPEG from the camera (opened only while watched)
GET  /camera.jpg        last frame
```

`POST /mode` waits up to 2.5 s for the transition. A failed transition returns
409 with the reason and the old mode intact; a slow one returns
`{"pending": true}` and the worker finishes it — poll `/state`.

Joint order is `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll, gripper`; gripper is 0–100, everything else degrees.

`seq` is advisory: any value is accepted and a lower one is reported as
`seq_reset: true`. A restarted console just works.

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

## Tests

```
pytest -q live/tests/test_protocol.py        # parsing, envelope, step budget
pytest -q live/tests/test_bridge_redesign.py # loopback: no wedge, click-in,
                                             # stale seq, dead-client TTL
```

Both run anywhere; neither touches hardware.

## Deliberate limitation — motion speed

`MAX_SPEED=35` deg/s with `speed_scale <= 0.3` caps joint motion at **10.5°/s**.
That is the demo default on purpose: a scan pose that overshoots is a collision,
and the arm is slow enough to watch. Raising it toward the servos' real maximum
is a separate qualification, tracked in `docs/ROADMAP.md`.
