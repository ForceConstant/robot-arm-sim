# Phase 0 — move verification (2026-11-24)

**Question this answers:** commit `9252a5b` closed `_stream_to` on measured
encoder joints. Tracking improved (2.08 → 0.69 deg), but a large relocation was
then *accepted* (perr 1.5 mm), reported a small error, and **the arm never
moved**. `9aa123c` reverted it with one instruction: *do not reapply without a
move-verification test for a large relocation, not just small steps.*

This is that test. **No motion behaviour was changed** — `live/app.py` is
untouched, and nothing here is committed.

---

## 1. What was built

| file | role |
|---|---|
| `tools/move_settle.py` | `settle()` — poll joints until they stop changing. The one measurement primitive; `require_fresh()` guards against frozen readback; `deltas()`, `distance_mm()`, `wait_for_motion()`. |
| `tools/sim_move_bridge.py` | Standalone sim-backed Pi-bridge stand-in over real HTTP, with injectable mechanisms (`--sag`, `--deadband`, `--stall-above`, `--stale-readback`). |
| `tools/test_move_executes.py` | The harness. Commands a large relocation + small steps and **fails unless the measured joints actually change**. Runs `--sim` or `--irl`. |
| `tools/phase0_status_image.py` | Renders the status sheets below (Pillow; no matplotlib on the BC-250 venv). |

### Why `settle()` is the core of it

Every existing check in the tree reads the *console's own reply*. That reply is
the thing that lied in `9252a5b`. Worse, the "measured coupling matrix" in
`tools/sim_jacobian_sweep.py` was later shown to be a **mid-flight sampling
artifact** — the probes read FK 1.6 s after commanding, i.e. sampled a
half-travelled arm and called it coupling. A single read cannot distinguish

- arrived,
- still travelling,
- never started.

`settle()` polls until the joints stop changing for `quiet_s` and reports
whether they *settled* or merely *timed out*. `wait_for_motion()` is the
separate assertion "did it go anywhere at all".

### The gate the harness enforces

A large case passes only when all of:

1. measured joints moved ≥ 1.0 deg within the motion window,
2. they then **settled** (not a timeout),
3. the console did not refuse (409),
4. tool travel ≥ **100 mm**,
5. ≥ **2 arm joints** each moved ≥ 5 deg,
6. ≥ **60 deg** of total arm-joint travel.

### Two gate bugs found and fixed against real data

Both were the harness being wrong, not the arm — worth recording because each
one initially looked like a failure of the thing under test.

- **"two joints each ≥ 25 deg" rejected a good move.** A pan-dominant sweep
  *across the board* is a legitimate large relocation: measured 68.2 deg on
  `shoulder_pan`, 8.6 deg on `shoulder_lift`, **154.4 mm of tip travel**. The
  bar is about distance covered, not about spreading motion evenly across joints.
  Replaced (items 4–6 above).
- **The case measured from "wherever the arm happened to be".** After a probe
  left the arm at the far corner, the same accepted target travelled only
  89.9 mm. The harness now commands a **declared start pose** first, so "large
  relocation" means the same distance every run.

---

## 2. Sim results (BC-250, CachyOS)

Command:
```
.venv/bin/python tools/test_move_executes.py --sim
```
Env: `ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=.private/piece-assets-v5
ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y MUJOCO_GL=egl`

| case | verdict | joint travel | max joint | reported err | tip travel |
|---|---|---|---|---|---|
| `park_at_start` | PASS | 7.5° | 3.5° | 0.54° | 5.9 mm |
| `large_relocation_large` | **PASS** | 112.7° | 44.4° | 0.27° | **122.2 mm** |
| `small_step_pan` | PASS | 5.5° | 5.5° | 0.46° | 9.9 mm |
| `small_step_pan_back` | PASS | 5.5° | 5.5° | 0.46° | 9.9 mm |
| `small_step_wrist` | PASS | 4.8° | 4.9° | 0.15° | 14.6 mm |

`reports/phase0_sim6.json`.

### Controls — the harness must be able to fail

A green suite proves nothing unless the suite can go red. Three injected
mechanisms, all on the sim path:

| control | mechanism | result |
|---|---|---|
| **frozen readback** | `/state.joints` pinned at the start pose while the mechanism moves | **FAIL** — every case `moved: false`, travel 0.0°, while the console still reported `error_deg` 0.15–0.46. **This reproduces the 2026-11-23 symptom** (small reported error, no motion); it does not by itself establish the cause — see Finding 1. `reports/phase0_stale6.json` |
| **sag + deadband** | shoulder_lift −1.5°, elbow_flex +1.2°, ignore steps < 0.25° | **PASS** 5/5 — the harness does not rely on a perfect mechanism, so it is not trivially green |
| stall-above | accept and ignore moves > 25 deg | did **not** trigger: the console streams ≈0.7 deg/tick, so no single command ever exceeds the knob. Documented here rather than removed — it constrains the *shape* of the bug to "frozen or ignored readback", which the frozen control then reproduces. |

`reports/phase0_sag6.json`, `reports/phase0_stale6.json`.

---

## 3. IRL result — one verified relocation

Safety practice followed: dry-run first, `REMOTE` mode, keep-in respected,
**no gripper actuation**, single relocation, bridge returned to `HOLD` after.

Endpoint pair, chosen because **both ends solve on the same wrist_roll branch
(87.56 deg)** — AGENTS.md prefers targets that keep the current roll branch, and
the −x/+y corner solves at −2.44/67.6, i.e. a different branch:

- start/park: `(65, 70, 110)` mm
- relocation: `(-90, 20, 110)` mm

Nominal separation 163 mm, corner to corner rather than centre to corner, so a
~10 mm solved-landing error cannot erode the 100 mm bar.

### Dry run first
`PASS` — solved at perr 4.7 mm / appr 0.50 deg, `max_joint_delta_deg 67.8`.
`reports/phase0_irl_dry3.json`.

### The relocation
`reports/phase0_irl_run2.json` — `passed: true`, 2026-11-24T08:05:34-05:00.

The run happened to capture **both directions**:

| leg | before (deg, 6 joints) | after (deg) | tip before → after (mm) | tip travel | reported err | settled |
|---|---|---|---|---|---|---|
| `park_at_start` | 20.57, −19.03, 19.12, 93.58, 87.65, 43.88 | −47.12, −28.00, 24.13, 94.90, 87.65, 43.88 | (−84.8, 28.1, 97.0) → (65.0, 67.0, 103.0) | **154.9 mm** | 0.48° | yes (1.05 s) |
| **`large_relocation_large`** | −47.12, −28.00, 24.13, 94.90, 87.65, 43.88 | 21.10, −20.00, 18.33, 93.49, 87.65, 43.88 | **(65.0, 67.0, 103.0) → (−87.4, 25.5, 100.7)** | **158.0 mm** | 0.13° | yes (1.50 s) |

Relocation detail: `shoulder_pan` **+68.2°**, `shoulder_lift` +8.0°,
`elbow_flex` −5.8°, `wrist_roll` 87.65 → 87.65 (**branch preserved**),
max joint 68.2°, total arm-joint travel 83.4°, **3 moving joints**.
Console solved the move at perr 4.7 mm.

**The arm really moved and really settled.** Motion was detected 2.33 s in,
while the console was still inside its stream loop — i.e. the movement was
observed independently of the command returning, which is the property the
reverted code lacked.

Readback freshness was asserted before measuring: `joints_age_s` 0.088 s at the
start of the relocation. This matters — see finding 1 below.

---

## 4. Findings that bear on re-landing the measured-joint loop

### Finding 1 — the bridge's joint readback is mode-dependent, and stale in `HOLD`

`live/pi_bridge/app.py` only calls `_read_follower()` in `REMOTE`, and there only
every `READ_EVERY` (=3) ticks; the `HOLD`/`FAULT` branch calls `_hold_tick()`,
which *sends* the last goal but never *reads* the encoders. Measured on the live
bridge 2026-11-24:

```
mode=HOLD    joints_age_s = 49209.5   (~13.7 h), later 150.7 s after a move
mode=REMOTE  joints_age_s = 0.02 .. 0.09
```

**Scope correction — this is NOT proof that `9252a5b` failed this way, and the
first draft of this receipt said it was.** Every move is gated by
`_require_remote()`, and `_enter_remote()` calls `_open_follower()`, which
calls `_read_follower()` unconditionally. So by the time `_stream_to` runs, the
readback *is* fresh — confirmed in the IRL run above (`joints_age_s` 0.088 s at
the start of the relocation). A stale `HOLD` readback therefore cannot by itself
explain the `9252a5b` non-execution.

What the measurement does establish, and why it still matters:

- The readback's freshness depends on a mode transition having happened, not on
  the read path itself. A loop that trusts `state["joints"]` is only as good as
  an invariant enforced somewhere else in the file.
- The frozen-readback control shows the *symptom* exactly — small `error_deg`,
  zero travel — and that symptom is what `9252a5b` produced. Whether the real
  cause was a stale snapshot, a lost command, or an empty `joints` dict
  (`len(joints) == 6` silently skips the update, leaving `measured` at `start`)
  is **still unresolved in the tree** and must be settled before re-landing.
- Note the third candidate in that line: `if len(joints) == 6: measured = ...`
  means a `{}` or partial state leaves `measured` frozen at `start` forever while
  the loop keeps commanding `start + residual`. That is a *code-level* frozen
  readback and it does not need the bridge to misbehave at all.

**Consequence for re-landing:** the loop must (a) treat a missing/short joint
dict as a hard error rather than a no-op, (b) assert readback freshness at move
start, and (c) be told the real cause of the `9252a5b` non-execution before it
is trusted on a large move.

### Finding 2 — the console blocks, and `/move` is `async def` over blocking work

`POST /move` is declared `async def` but calls blocking `live.move()`, which does
the IK solve inline. A cold `solve_tip` for a far target measured **98.7–120.6 s**
on 2026-11-24 (near targets: 2 ms). During that window the whole console is
unresponsive — `/state` and `/keep-in` both time out — and it recovers only once
the solve finishes. This is *not* a crash: it drained and came back twice.

Consequence for the harness: the motion window has to outlast a ~2-minute solve,
so `--move-timeout-s` defaults to 180 s and `watch_motion()` exits early once the
command returns. Consequence for the console: unrelated, and **not** fixed here
(out of scope) — flagged for a separate decision.

### Finding 3 — 100 mm of *tool* travel is a weak bar for "large"

The reverted bug was a **large relocation**. Measured on this arm, a full
corner-to-corner sweep at constant roll is only ~158 mm of tip travel with 83° of
joint travel — so 100 mm is comfortably clearable but not generous, and a
centre-to-corner start (≈120 mm nominal) can be eaten entirely by a ~10 mm solved
landing error. Hence: declared start pose, corner-to-corner endpoints, and the
three-part bar (>100 mm, ≥2 moving joints, ≥60° joint travel) rather than a
single distance.

---

## 5. Acceptance gate for re-landing the measured-joint loop

`9252a5b`'s loop may be reapplied only when **all** of the following hold. This is
the recommendation, not a change already made.

1. **`tools/test_move_executes.py --sim` is green 5/5**, including
   `large_relocation_large` (≥100 mm tip, ≥2 moving joints, ≥60° joint travel).
   The suite must also still go **red** under `--stale-readback` — if the frozen
   control ever passes, the suite has stopped testing anything.
2. **`--irl --case large` passes once**, with the arm observed to move while the
   console is still inside the loop (`motion.before_return: true`), then settle
   (`after_settled: true`), from a declared start pose, at tip travel ≥100 mm.
3. **The stale-readback path is closed**, by one of:
   - the bridge polls the follower in `HOLD` as well, or
   - the console refuses to start a move whose `joints_age_s` exceeds ~1 s
     (`move_settle.require_fresh` already implements this check and is wired to
     `--irl` runs).
   Evidence required: `joints_age_s` at move start recorded in the receipt.
   This is not currently a live bug on the move path — `_require_remote()` plus
   a reading `_open_follower()` makes the move-start readback fresh today
   (measured 0.088 s). It is a gate because the freshness is an incidental
   consequence of a mode transition, not an invariant, so a re-landed loop would
   depend on something nobody is checking.

   **Additionally required:** the loop must treat a missing or short joint dict
   as a hard failure. `9252a5b` wrote `if len(joints) == 6: measured = ...`, so a
   `{}` state silently left `measured` frozen at `start` while the loop kept
   commanding `start + residual` — a code-level frozen readback that needs no
   bridge misbehaviour at all, and the most likely of the three candidates.
4. **Reported error is honest for a large move.** `error_deg` must be quoted
   against *measured* joints, and a move that does not move must be a failure —
   the frozen control shows what silent success looks like (`error_deg` 0.15–0.46
   with 0.0° of travel).
5. **No new `async def` blocking work** in the console: `/move` must not hold the
   event loop for a two-minute solve. Either keep the existing behaviour and
   document the block, or move the solve off the loop — but a re-landed loop that
   polls `state()` every 66 ms through a solve that takes 100 s is a different
   failure mode than the one being fixed.

Until 1–5 hold, `9252a5b`'s loop stays reverted.

---

## 6. Status

```
PASS  sim baseline (5/5)          reports/phase0_sim6.json
PASS  sim sag+deadband (5/5)      reports/phase0_sag6.json
FAIL  sim frozen readback (stops at park, 0/1)
                                  reports/phase0_stale6.json   <- control, red on purpose
PASS  IRL dry run                 reports/phase0_irl_dry3.json
PASS  IRL relocation 158.0 mm     reports/phase0_irl_run2.json
```

Sheet: `reports/phase0_suite.png` (left to right: sim baseline, sag+deadband,
frozen readback, IRL).

**Blocker encountered, no workaround used:** the BC-250 console wedged for
~4 minutes during probing (Finding 2). It was left to drain rather than
restarted, since `live/app.py` was not in scope and a restart would have
discarded an in-flight solve.

## 7. Working tree

Uncommitted, as instructed — no commits, no pushes, no roadmap edits:

```
tools/move_settle.py             new
tools/sim_move_bridge.py         new
tools/test_move_executes.py      new
tools/phase0_status_image.py     new
reports/phase0_*.json            new
reports/phase0_suite.png         new
reports/phase0_move_verification_20261124.md   new (this file)
live/app.py                      UNCHANGED
```
