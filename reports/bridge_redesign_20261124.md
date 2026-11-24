# Bridge redesign — two states, no latch, no wedge

**Date:** 2026-11-24
**Scope:** `live/pi_bridge/app.py`, `live/pi_bridge/protocol.py`,
`live/pi_bridge/README.md`, `live/pi_bridge/so101-bridge.service`,
new `live/tests/test_bridge_redesign.py`
**Not touched:** `live/app.py` (:8080 console), `live/bridge_client.py`,
`live/scan_controller.py`, the roadmap, the Pi service (still running the old
build), the arm itself.

Owner direction: the arm is not a safety concern — the owner secures it by
switching power off. The old bridge was over-guarded for that reality: a HOLD
state you had to be let out of, a FAULT latch cleared only by a power cycle, an
`/arm` precondition, an alignment gate that refused a whole handover over one
joint, and a `seq` rule that permanently wedged a console whose counter fell
behind.

---

## 1. Design

Exactly two states, both entered on request, both accepting commands:

| Mode | Bus owner | How the follower moves |
| --- | --- | --- |
| `REMOTE` | bridge, follower port | `/command` targets, walked in at the step budget |
| `TELEOP` | bridge, leader + follower ports | leader read 30 Hz, mirrored per joint |

Boot enters `REMOTE` and holds the pose. `POST /mode {"mode":"HOLD"}` is
accepted as an alias for `REMOTE`, so a stale console button can never strand
the arm in a state that refuses motion.

**Conditions are reported, not latched.** Three status fields replace FAULT:

| Field | Meaning | Cleared by |
| --- | --- | --- |
| `error` + `error_source` | the last exception, tagged `worker`/`follower`/`leader`/`transition`/`follower` | that same source succeeding again, or any accepted command / mode request |
| `estop` | "stop driving, hold this pose" | the next command or `/mode` |
| `click_in` | TELEOP joints not yet following the leader | that joint coming within `CLICK_IN_DEG` |

A source-tagged error means a healthy follower read cannot silently erase a
dead-leader report — the operator keeps the clue that matters.

**TELEOP owns the leader itself.** The old code delegated to LeLab's
`/move-arm`, which starts mirroring the instant it connects. That is
irreconcilable with per-joint click-in, so the bridge opens the leader bus and
reads it directly. LeLab's teleop is still *stopped* before any transition
(it may be holding the ports), but nothing in the handover path depends on it.

**Per-joint click-in.** On entering TELEOP every joint within `CLICK_IN_DEG`
(default 10°) of the follower is marked engaged and follows at once. A joint
further out is pinned to its measured pose and joins in on the first cycle the
leader brings it within the radius. One misaligned wrist delays exactly one
joint.

**Sequence numbers are advisory.** Any `int >= 0` is accepted. `last_seq` is
tracked for reporting and `seq_reset` is true when an arriving value is lower,
but nothing is refused. Freshness is `now - command_ts <= ttl_ms`, i.e. arrival
time, so no counter can wedge the bridge and a dead client still fails safe.

**Self-healing buses.** The worker reopens a missing follower (any mode) or
leader (TELEOP) on a 3 s backoff instead of requiring a mode round-trip. A
worker exception sets an error and a 1 s backoff; it never changes the mode.

---

## 2. Removed vs kept

| Guard / behaviour | Old | New | Rationale |
| --- | --- | --- | --- |
| `HOLD` state | separate mode, bus closed | **removed** (alias → REMOTE) | a state you must be let out of |
| `FAULT` latch | latched, needed `/arm` + REMOTE | **removed** | a latch needing a power cycle |
| `/arm` precondition | required before commands and TELEOP | **no-op compatibility call** | nothing needs arming |
| `armed` in `/state` | false after fault | constant `true` | key kept so the console/scan controller are unchanged |
| `fault` in `/state` | reason string when latched | constant `null` | consumers must not read a report as a latch |
| Handover alignment gate | `SO101_HANDOVER_DEG=10`, refused whole TELEOP | **replaced** by per-joint `SO101_CLICK_IN_DEG=10` | one bad joint must not refuse the handover |
| LeLab `/move-arm` + joint proxy WS | mirror via LeLab, `ws://…/ws/joint-data` | **removed** | LeLab mirrors on connect; defeats click-in |
| `seq must be newer` | `409` forever once behind | **removed** — any seq accepted | 63M-gap wedge, 2026-11-24 |
| E-stop latch | `FAULT`, `/arm` to clear | `estop` hold, next command resumes | powering off is the real e-stop |
| Joint-limit clamping (per joint, margin; wrist_roll 0 margin) | kept | **kept** | real protection, costs nothing |
| Per-cycle step budget (`MAX_SPEED`, `speed_scale ≤ 0.3`) | kept | **kept** | a far target is walked in, never jumped |
| Lead guard `MAX_LAG_DEG` | REMOTE only | **kept, now also TELEOP** | caps unexecuted backlog in both modes |
| TTL command expiry (40–400 ms) | kept | **kept** | a dead client fails safe to a hold |
| Token auth on mutating endpoints | kept | **kept** (`/mode` included) | unchanged |
| Camera loss ≠ motion fault | kept | **kept** | separate subsystem |

Also changed: `protocol.py` `limits_from_calibration` gained the
`margin_by_joint` parameter the app already called (the repo copy was behind the
BC-250 copy, so a fresh checkout would have raised `TypeError`).
`so101-bridge.service` replaces `SO101_HANDOVER_DEG` with `SO101_CLICK_IN_DEG`
and corrects the comment about which ports the bridge owns.

---

## 3. Test evidence

`live/tests/test_bridge_redesign.py` — 22 tests, fakes for the serial devices,
plus one fixture that runs the **real** `FastAPI` app under `TestClient` with
the worker thread started, so routing, auth and the `/mode` wait are all
exercised. Run on the BC-250 (`brian@192.168.1.171`,
`/home/brian/robot-arm-sim/.venv`); neither file touches hardware.

```
$ .venv/bin/python -m pytest live/tests/test_bridge_redesign.py live/tests/test_protocol.py -q
.........................................                                [100%]
41 passed in 0.72s
```

22 redesign + 19 protocol. The five original `test_protocol.py` behaviours
(parse, reject bad, limits, clamp, step budget) still pass unchanged.

### (a) No wedged state exists

| Test | Claim |
| --- | --- |
| `test_remote_accepts_a_command_after_any_reported_condition[clean/error/estop/error_and_estop/stale_seq]` | after every reporting condition, the next command is accepted **and** the condition is cleared |
| `test_a_bus_exception_is_reported_and_the_next_command_still_works` | a bus timeout on a read sets `error`, leaves the mode, and the next command drives the arm |
| `test_teleop_can_be_left_for_remote_without_any_precondition` | TELEOP → REMOTE with a leader 80° out of alignment, no armed handshake, leader bus released |
| `test_estop_is_a_hold_not_a_latch` | `/estop` holds the exact pose; the next command moves the arm |
| `test_switch_mode_waiting_on_the_worker_reports_an_honest_failure` | a failed TELEOP entry returns the reason (409) with the old mode intact; the bridge is still usable |
| `test_only_remote_commands_are_refused_and_the_refusal_names_the_fix` | the only refusal names the recovery (`POST /mode {mode:REMOTE}`) |
| `test_a_lost_leader_is_reopened_in_teleop_without_a_mode_round_trip` | a USB drop self-heals |
| `test_http_loopback_two_states_no_wedge` | same claims over HTTP, including `HOLD` → `REMOTE` and `/estop` → command |

### (b) Immediate takeover + per-joint click-in

`test_teleop_starts_immediately_and_follows_every_aligned_joint` — leader
`shoulder_pan` 3° out, `elbow_flex` 3° out, `wrist_roll` **60° out**. On the
first cycle the first two follow (`> POSE`) while `wrist_roll` is sent exactly
at its measured pose; `wrist_roll` is in `click_in`.

`test_the_out_of_spec_joint_clicks_in_once_the_leader_comes_close` — the leader
wrist is brought to 5° of the follower; `wrist_roll` moves into
`teleop_engaged`, leaves `click_in`, and the next cycle sends a wrist target.

`test_http_loopback_teleop_click_in_is_visible_in_state` — the same over HTTP:
`/mode TELEOP` returns `TELEOP` on the first request with `wrist_roll` in
`click_in`, and after the leader moves the joint appears in
`teleop_engaged` with `click_in == []`.

### (c) Stale-sequence recovery

| Test | Claim |
| --- | --- |
| `test_a_client_millions_behind_recovers_on_its_next_command` | `last_seq = 63_000_000`, client sends `seq=3` → accepted, `seq_reset: true`, then `seq=4` accepted |
| `test_duplicate_and_older_sequences_are_not_rejected` | 10, 10, 9, 11 all accepted |
| `test_freshness_comes_from_arrival_time_not_the_counter` | same command: 50 ms old moves, 300 ms old holds, regardless of seq |
| `test_http_loopback_two_states_no_wedge` | 63M gap over HTTP → 200 with `seq_reset: true` |

### (d) Dead client still fails safe

`test_a_dead_client_fails_safe_on_ttl_expiry` — five cycles of a live client
ramp the joint; the client stops and after the TTL the bridge sends only the
hold pose (the joint value stops changing). `ttl_ms=150`, expiry measured from
arrival.

### Full live suite

```
$ .venv/bin/python -m pytest -q live/tests
7 failed, 45 passed, 2 errors in 16.78s
```

The 7 failures and 2 errors are **pre-existing and environmental**, verified by
running the same files against a pristine `HEAD` worktree on the BC-250:

- `test_scan_controller.py` (7) — `ValueError: plan pose 'marker14_r68' has
  implausible joint values for degrees` and `plan_in_degrees` on a plan fixture;
  identical 7 failures with `live/pi_bridge` checked out at `HEAD`.
- `test_app.py`, `test_http_pipeline.py` (2 errors) — `ValueError: printed
  pieces require ROBOT_PIECE_ASSETS with set.json`; identical on `HEAD`
  (and they are `8 skipped` in a clean `git worktree` without the private
  assets).

Neither touches the bridge. Nothing regressed.

---

## 4. What was NOT changed

- `live/app.py` — the :8080 console. It reads `mode`, `armed`,
  `follower_open`, `lelab_active`, `joints`, `limits` from `/state`; every one
  still exists with the same type, so the console keeps working without a code
  change. Its own `command_seq` logic is untouched.
- `live/bridge_client.py` and `live/scan_controller.py` — unchanged. They call
  `/state`, `/command`, `/mode`, `/arm`, `/estop`; all still exist.
- Contract of `/command`: same body, same joint order, same units, same 409
  when not REMOTE.
- No roadmap edit, nothing committed, the Pi service untouched, no motion.

## 5. Blockers / follow-ups for the owner

- **Deployment needs your hands.** The Pi (`pi@192.168.1.167`, unit
  `so101-bridge`, active, enabled) still runs the 2026-11-20 build. Copying
  `app.py`, `protocol.py` and the unit file to `/home/pi/so101-bridge` and
  `sudo systemctl restart so101-bridge` is a live service replacement, so it is
  deliberately not done here. The Pi's own `.bak-*` files suggest you keep that
  habit — no new backup was written from this side.
- **LeLab service**: with the redesign the bridge no longer needs LeLab for
  TELEOP, but it still calls `/teleoperation-status` and `/stop-teleoperation`
  before a transition. If LeLab is removed from the Pi, that call fails open
  (`URLError` → treat as inactive) so nothing breaks; say the word if you want
  the dependency dropped entirely.
- **Speed**: `MAX_SPEED=35`, `speed_scale ≤ 0.3` → 10.5°/s still caps motion.
  Raising it is still the separate qualification the README describes.
