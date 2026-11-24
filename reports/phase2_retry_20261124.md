# Phase 2 retry — gate blocked by the mechanism; console wedge fixed; recovery landed (2026-11-24)

**Scope this run:** re-attempt the IRL gate after the owner rectified the
mechanism, gate the configuration we ship (mink + 3.0 mm), fix the console wedge
the first retry exposed, and add the stored-home recovery primitive.

**Outcome in one line:** the console defects are fixed and tested, the
mink/3.0 console solves and passes every pre-move check, but **the arm still
cannot execute commanded joint motion under load**, so no relocation was run and
the flip is **not** applied. Recovery is implemented, tested and configured from
a stored pose.

Nothing committed. `live/pi_bridge/*` in-flight work untouched.

---

## 1. Elbow probe — actuates, then freezes

Bridge state at the start of the run: `REMOTE`, `armed`, no fault, readback age
0.1 s, joints `{pan -16.0, lift 51.3, elbow 16.04, flex 94.99, roll 87.74,
gripper 43.88}`, tip **y 132.9 mm — 14.9 mm outside keep-in**.

Single-joint probes through `tools/phase2_ticks.py` (each command acked
`accepted: true`; bridge `commands_accepted` 266, `commands_rejected` 0):

| when | probe | commanded | achieved | note |
|---|---|---|---|---|
| pre-redeploy | elbow, 0.7° lead | −3.0 | **0.26** | one servo step, then still |
| pre-redeploy | elbow, 1.0° lead ×3 s | −2.0 | **0.35** | stall |
| pre-redeploy | elbow, 3.0° lead ×3 s | −4.0 | **2.11** | moves, 1.89° residual |
| pre-redeploy | shoulder_lift, 0.7° lead | −3.0 | **0.09** | stall |
| pre-redeploy | wrist_roll (unloaded) | −3.0 | **2.82** | healthy |
| post-redeploy | elbow, 0.7° lead | −3.0 | **0.18** | still stalls |
| post-redeploy | pan, 3.0° lead | −5.0 | **4.14** | moves, then ~0.9° dead zone |
| post-redeploy | elbow, 4.0° lead | −4.0 | **1.50** | then freezes with 2.5° error |
| post-redeploy | elbow, 4.0° lead | −3.0 | **0.61** | freeze worsens |
| post-redeploy | elbow, 4.0° lead | −2.0 | **0.17** | effectively frozen |

Reading: the bus, the mode and the servo electronics answer (unloaded wrist_roll
tracks 2.82 of 3.0). The loaded joints need 1–3° of commanded error to move at
all and then stop **while the goal stays 2–2.5° away** — the bridge keeps writing
`Goal_Position` at 30 Hz and the encoders do not follow. This is the same family
as the 2026-11-24 morning failure, partially recovered: the elbow now travels
~3.4° in total where it previously travelled 0.0.

The arm was left at `{pan -21.19, lift 54.29, elbow 9.98, flex 97.01, roll 84.84,
gripper 43.88}`, tip **y 124.3 mm — 6.3 mm outside keep-in**, stationary,
`REMOTE`, no fault, gripper untouched. Attempts to walk it the last 6 mm inward
(three elbow nudges) returned 0.17–0.61° each; pushing further would have been
repeatedly driving a joint through its protection, so it stopped there.

## 2. Console wedge — fixed, with a test that fails without the fix

`POST /move` was a bare `async def` around the blocking `Live.move`. A cold h1
solve (~169 s on least_squares) parked the whole event loop: `GET /state` hung
and the console looked dead. That is what stalled the first retry's edge dry-run.

* every route that can spend real time (`/move`, `/jog`, `/sanity`, `/home`,
  `/scan`, `/pose/run`, `/wrist/*`, `/board/derive`, `/capture-view`) now
  dispatches through `run_in_threadpool`; motion semantics, guards, keep-in and
  the stream loop are untouched — the motion lock is still the single-writer
  guard.
* `live/tests/test_console_responsive.py` holds a `/move` in flight and asserts
  `/state` answers in <1 s, then asserts the same route still jogs the arm.
  Proved both ways: **with the fix 2 passed; with `git show HEAD:live/app.py`
  restored the test fails with `TimeoutError` on `/state`.**

## 3. Gating the configuration we ship (mink + 3.0 mm)

Second console on `:8081` (`LIVE_CONFIG=/tmp/live_config_8081.json`,
`ik_solver: mink`, `move_tolerance_mm: 3.0`), live service `:8080` untouched.

| step | result |
|---|---|
| dry run park + large | PASS, targets keep roll 84.75 and gripper 43.88 |
| dry run h1 edge | PASS, elbow target −63.78 (74° of travel), roll 84.75 |
| leg check park (65,70,110) | `clear_for_motion: true` — inside keep-in, perr ≤3, appr ≤8, roll Δ0.09 |
| leg check large (−90,20,110) | `clear_for_motion: true` |
| leg check h1 (98,−98,60) | `clear_for_motion: true` |
| path clearance current→park | **2 samples outside keep-in** — the *starting* pose is 6.3 mm outside; the interpolated path itself never exceeds 123.8 mm |

**Relocation not executed.** The park leg alone needs +17° of elbow and h1 needs
−74°; the elbow cannot deliver 1°, so the move would either stall mid-board or
trip the hardened loop's non-execution raise after 30 s. The gate exists to prove
that a real relocation executes and lands; with the actuator unable to move, the
result would be noise, not evidence.

**Flip NOT applied** — `live/app.py` DEFAULTS stay `least_squares` / 20.0 mm.

### A deployment finding the flip would hit immediately

`MoveSolver.enable_mink()` needs `chess_sim.gripper.CUSTOM`, i.e. the
`ROBOT_GRIPPER=cad_parallel` + `ROBOT_GRIPPER_ASSETS=…` environment. Without it
the mink arm model has no jaw bodies and the first solve raises
`KeyError: "Invalid name 'so101:moving_jaw_so101_v1'"` — verified by starting the
console with no `ROBOT_*` env. The live `:8080` service runs with only
`PYTHONPATH` and `LIVE_CONFIG`, so flipping `ik_solver` to mink there without also
adding the gripper env would break the console at startup. Whatever lands the
flip must land the service environment with it.

## 4. Recovery primitive (owner's rule)

> "If the robot gets stuck, it should always just reset to home position high
> above middle square and start again."

* `recovery_joints` — a **stored joint pose** (never solved at runtime):
  `{pan -12.37, lift -13.02, elbow 4.80, flex 98.34, roll 87.74, gripper 43.88}`.
  FK: tip `(0.17, −0.91, 116.53) mm`, 1.07 mm from `(0, 0, 116)`, jaws 0.12°
  from vertical, roll on the play branch, all six inside the measured limits.
  Derived by `tools/recovery_pose.py`; z is 116 not 130–140 because a jaws-down
  hover over the centre needs `wrist_flex ≈ 102.5` at z=130, past the 98.79 limit.
* **Leader candidate tried first, as asked — a one-time capture only.** No leader
  polling exists in the recovery path and none was added: recovery reads the
  stored pose and nothing else. Read the leader read-only
  (`tools/leader_pose.py`, `Present_Position` only, no register writes, leader bus
  free because the bridge is in REMOTE): `{pan 29.8, lift −102.68, elbow 96.44,
  flex 73.85, roll −29.85, gripper −26.15}`; with `wrist_flex` forced to 0 the tip
  lands at `(−144.6, −24.7, 188.7) mm` with the jaws **96° off vertical** and
  `elbow_flex 96.44` **past its 94.75 limit** → **rejected**, fallback to the
  solved pose, reasons recorded in `reports/recovery_leader_candidate.json`. The
  human's leader posture is not over the middle square; that is information, not
  a failure.
* `recover_to_home()` — direct joint-space move to the stored pose through the
  same stream/step-budget path. Two legs when pan must change: raise with pan
  held, then pan at height, so a folded arm never sweeps its jaws sideways at low
  z. Joint limits are clamped (the pose is a request, not an exemption); the
  **gripper is pinned to its measured value** and never actuated.
* Automatic on `StuckError`: keep-in refusal, unreachable pose, solver exception,
  the hardened loop's stall/non-execution raise, and a move that lands outside
  keep-in. Recover once, retry the move once, then raise with the last reason.
  `recovery_max_attempts` 3/op, `move_retries_after_recovery` 1 — both in config,
  both hard bounds; a test with retries at 9 still cannot loop.
* `POST /recover` (`{"reason": "..."}`) and `recover_first: true` on `/move`.
* Every event appends one JSON line to `live/recovery_log.jsonl`: reason, from
  pose, clamped values, per-leg residuals, landed check, seconds.
* Tests: `live/tests/test_recovery.py` — 7 tests (stored pose, single retry,
  no-loop on second failure, cap, landed-outside trigger, dry-run never recovers,
  `/recover` uses no solver and no gripper). All pass.

## 5. Test suite

`live/tests`: **9 failed / 55 passed**. The failures are pre-existing: the same
two app-importing modules were run against `git show HEAD:live/app.py` and fail
identically (`test_app::test_page_loads_and_scan_runs`,
`test_http_pipeline::test_http_pipeline_returns_board_offset`), and the
`test_scan_controller` failures come from a module that does not import `app.py`
(reported pre-existing since Phase 2).

## 6. State left behind

* Arm: `{pan -21.19, lift 54.29, elbow 9.98, flex 97.01, roll 84.84,
  gripper 43.88}`, tip y 124.3 mm (**6.3 mm outside keep-in**), stationary,
  bridge `REMOTE`, `error: null`, `commands_rejected: 0`.
* `:8081` retry console stopped; `:8080` live service never restarted (still runs
  pre-Phase-2 code).
* Bridge redeploy by the owner is live and healthy (uptime, no fault).

## 7. Working tree (uncommitted)

```
live/app.py                          threadpool dispatch; StuckError; recovery
live/tests/test_console_responsive.py new — /state answers during a long move
live/tests/test_recovery.py          new — trigger/cap/no-loop/recovery pose
tools/phase2_fk_check.py             new — read-only FK + keep-in margins
tools/phase2_retry_status.py         new — milestone sheets
tools/recovery_pose.py               new — derive/verify the stored pose, leader fallback
tools/leader_pose.py                 new — read-only leader read (runs on the Pi)
reports/retry_*.json/.png            new receipts
reports/recovery_leader_candidate.json
```

No commits, no roadmap edits, no gripper actuation. `live/pi_bridge/*` not
modified.

## 8. What remains

1. **Mechanism.** The arm must execute a commanded joint move before the gate can
   mean anything. Evidence points at load-dependent actuation (weak loaded
   joints, healthy unloaded one, zero bus errors) — supply/torque, not code. A
   servo register dump (`Max_Torque_Limit`, `Protection_Current`, `Overload_Torque`,
   `Present_Load/Voltage/Temperature`) needs the bus while the bridge is stopped,
   which is an owner action.
2. **Then** the gate: one relocation + h1 through `--irl` on the mink/3.0 console.
3. **Then** the flip, together with the service environment
   (`ROBOT_GRIPPER=cad_parallel`, `ROBOT_GRIPPER_ASSETS`).
4. The ~19 mm model bias stays Phase 3.
