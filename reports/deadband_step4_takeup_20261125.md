# Deadband Step 4 — a tested backlash take-up primitive

**Date:** 2026-11-25, 08:36–09:10 EST
**P gain / I / D:** unchanged at 32 / 0 / 32 (the settled step-2 state).
**Verdict:** the take-up is implemented, config-gated and tested, and it removes
the direction-dependent settle on the joint that mattered: **loaded elbow
−1.12° → −0.24°** (best run −0.15°, so 4.7–7.4×). It meets the ≤0.3° target on
the loaded direction and on both pan directions, **but not on the elbow's
gravity-assisted direction (−0.02° → −0.73°)**, where the approach from past the
target now leaves the joint further out on the other side. That is reported as
measured, not massaged: the primitive trades a 1.10° direction spread for a
0.48° one and a consistent sign.

---

## 1. What was built, and where

All in `live/app.py` (console). One code path for every move, as required:

| piece | where | what it does |
|---|---|---|
| `backlash_take_up_deg` | `DEFAULTS` (3.5) | the margin. `0` disables; read at request time. |
| `Live._take_up_request` | after `_stream_to` | plans the overshoot pose or refuses with a reason. No motion. |
| `Live._stream_with_take_up` | after that | overshoot leg → final leg, with the fallback and the receipt. |
| `Live._take_up_receipt` | static | the plan as it appears in the move result. |
| `_execute_target` | tail of every `/move` | dry-run reports the plan; the move uses the helper. |
| `jog` | `/jog` | same helper, so joint jogs take up too. |
| `_stream_to` | — | new optional `settle_allow` and `deadline_s`; defaults unchanged. |

Behaviour: for every joint that travels ≥0.2°, the target is overshot by the
margin **in the direction of travel**, streamed, and then the real target is
commanded from there. The gripper is never overshot — it is held at the target
in both legs. `rel` and `aim` modes change only in that they pass through the
same helper.

Guards (all of them run on the **overshoot** pose before any motion):

- joint envelope — refused if the overshoot would violate a calibrated limit
  (or make an existing violation worse);
- wrist window — refused if `wrist_roll` would leave the calibrated window;
- keep-in — refused if the overshoot tip would leave the box;
- gripper-only moves and sub-0.2° moves do nothing;
- the overshoot is a normal `_stream_to`, so the step budget, stall guard,
  REMOTE/armed re-read and fault handling are the existing ones.

Every refusal falls back to a direct move and is recorded in the result under
`take_up: {applied: false, reason: "..."}`. A refusal is never silent, and a
failed overshoot leg also falls back with `fell_back: true`.

Final-leg constants (measured, see §4): stop at 0.3°, settle allowance 1.6°, a
**separate** stall-fault lead of 3.0°, 10 s deadline. The two numbers are
separate on purpose: the return leg lands inside the servo's static band, which
measured 0.6–1.3°, so "resting within the band" must not be scored as "not
actuating". With one shared number the stall guard fired on a healthy 0.80°
settle (allowance 0.8) and again at 1.22° (allowance 1.2) — and each time the
move path's recovery drove the arm home over a servo that was simply resting in
its friction band (`live/recovery_log.jsonl` 08:50:43 and 09:08:57). At 3.0°
the fault test is unambiguous: the leg is only 3.5° long.

## 2. The A/B measurement

`tools/backlash_take_up_test.py` (new). Protocol is the deadband discriminator's,
so the numbers are comparable: command a ±10° step, **hold the exact goal at the
bridge for 1.5 s**, report `residual = goal − Present_Position` (positive = short).

Two deliberate improvements over the step-1/2 protocol:

1. **Every rep starts from the same anchor pose** (`/move mode=joint` back to it),
   which removes the artifact where three same-direction reps walked the elbow
   into its reach corner.
2. **The anchor is chosen for keep-in headroom** (pan −60°, elbow 40°, over the
   board at z 152 mm), because at the old pose two of the four directions had
   their overshoot refused by keep-in and the A/B would have compared OFF with
   OFF. The requested find was worth reporting on its own: a take-up near the
   ±118 mm box edge is silently unavailable, by design.

Condition switch: `"backlash_take_up_deg": 0` in `live/live_config.json` plus a
`robot-live.service` restart for OFF, then the file restored byte-for-byte
(sha256 `4c28d879…`, identical to `rig_state/MANIFEST.json`) and restarted for
ON. The bridge was **not** restarted; the Pi was not touched.

### Medians of 3 reps, degrees (shipped build)

| joint / direction | OFF | ON (3.5°) | change | passes ≤0.3? |
|---|---|---|---|---|
| `shoulder_pan` +10 | +0.24 | **−0.11** | −0.35 | yes |
| `shoulder_pan` −10 | −0.24 | **−0.07** | +0.17 | yes |
| `elbow_flex` +10 (gravity-assisted) | −0.02 | **−0.73** | −0.71 (worse) | **no** |
| `elbow_flex` −10 (loaded) | **−1.12** | **−0.24** | **+0.88 (4.7×)** | yes |

Tail spread 0.000° in all 24 settles. Per-rep residuals are in
`reports/backlash_take_up_{off,on_shipped}_20261125.json`; the elbow −10 values
are −0.07/−0.24/−0.24 (ON) against −1.03/−1.12/−1.21 (OFF). Run-to-run range on
the previous build: elbow −10 −0.15, elbow +10 −0.64…−0.81 — the conclusions do
not depend on which run is quoted.

**Consistency:** the two elbow medians are now −0.73 and −0.24 (spread 0.48)
against −0.02 and −1.12 (spread 1.10); the two pan medians are 0.04 apart
against 0.48. Both directions now land on the **same side** of the goal (past
it), which is the mechanical signature of a known approach direction — but the
magnitude on the elbow's assisted side is outside the 0.3 target.

### What was tried and rejected (all measured, same anchor)

| condition | elbow +10 | elbow −10 | verdict |
|---|---|---|---|
| OFF | −0.02 | −1.12 | baseline |
| ON, margin 3.5° | −0.64 … −0.73 | −0.15 … −0.24 | kept |
| ON, margin 4.5° | −0.63 | −0.24 (one rep refused by keep-in) | no better |
| ON, 3.5°, slower final leg (0.3°/tick) | −0.81 | −0.16 | **worse**, reverted |

The margin is not the lever, and a gentler final ramp is not either: the
remaining elbow offset is not a velocity coast. It is the joint resting inside
its static band on the side it was released from.

### What this rules out, and what it does not

- The loaded-direction term was **history/approach dependent**, not
  load-proportional: approaching from past the target removes 0.97° of it, which
  a load/Kp droop could not do (it would need the load to vanish). So the 1.1°
  on that side is hysteresis/backlash, and the step-2 "droop" reading of the same
  direction was only partly right.
- The remaining 0.6–0.8° on the assisted side behaves the same way (same sign
  both ways), so it is the static band, not a solver or frame term. A frame term
  would move every square coherently and would not be ±10°-direction dependent.

## 3. Tests

- **Unit/behaviour:** `live/tests/test_backlash_take_up.py`, 9 tests, all pass on
  the BC-250: overshoot geometry and direction, per-joint only, gripper never,
  disabled, tiny move, joint-envelope refusal, wrist-window refusal, keep-in
  refusal, command order (max command past the target, last command exactly the
  target), and the refused-plan direct fallback.
  ```bash
  .venv/bin/python -m pytest -q live/tests/test_backlash_take_up.py   # 9 passed
  ```
- **Hardware A/B:** §2.
- **Regression:** `tools/verify_baseline.py` → **7/7 PASS** once the marker plane
  came up (aim: 6 solves, 0.05 s; actuation 3.96°; drift 1.58°), and **6/7** in
  runs where the plane was down (only the aim check fails). Rig-state checksum
  PASS in every run, so `live/board.json`, the mount calibration and the console
  config are unchanged.

## 4. Deployment

| item | value |
|---|---|
| `live/app.py` md5 | `33ce90c30041432d08ac64105ce7561c` (repo and BC-250 identical) |
| console | `robot-live.service` restarted, `active`, REMOTE, no fault |
| bridge | **not restarted** (P=32/I=0/D=32 untouched, so the console's camera client was never invalidated) |
| config | `backlash_take_up_deg` comes from the code default 3.5; `live_config.json` restored byte-for-byte |
| unit tests | 9/9 on the BC-250 |

Console-reported `error_deg` (max joint error at the end of a 10° move) fell from
p50 0.59 / max 1.21 (OFF) to p50 0.20 / max 0.81 (ON) on the same 12 moves, at
~2× the commands (p50 15 → 30) — the cost of the extra leg.

## 5. Blocker hit during step 5 (not caused by this change)

`verify_baseline.py` passed **7/7** once (aim plane 6 solves), so the sweep was
started. It completed **one** square and then stopped itself:

```
a1 tip: ok True tip_err 4.19 aim_err None px None 'None'
a1 aim: plane not usable (aim ray points away from the board); not commanding
b1: plane not usable (board plane is stale: last marker solve 49 s ago (limit 30 s))
c1: ... 65 s ...          d1: ... 81 s ...
board plane unavailable for 3 squares; stopping rather than commanding blind
```

So the one end-to-end number that exists is a1's tip leg — **4.19 mm / 1.04°**
against the baseline's **7.17 mm / 2.47°** at the same square — and the aim
column has no "after" value at all. Receipt:
`reports/aim_squares_out/aim_squares_takeup_20261125.json`.

The cause is the vision path, not the take-up:

- `/aim-point` reports `no board plane: no marker solve yet` at the A/B anchor,
  the recovery pose and the home pose; a direct bridge frame grab decodes
  **zero** ArUco ids with the strict detector, the lenient one, the inverted one
  and a 3× upscale — while the same detector returns **32 ids** on the rendered
  `board_charuco.png`, so the detector and dictionary are fine;
- the live frame is very soft: Laplacian variance **25.0** at home and 184 at the
  pose that points at the arm's own yellow body, against the "sharp close-up is
  several hundred" reference in the camera skill; an isolated vision check of
  one frame described a blurred, partly blown-out marker sheet in the corner and
  a large yellow surface across the centre;
- frames are stable and live (2.3 MB per 12 s, 4.5 fps, pose-dependent), so this
  is not a frozen capture;
- the plane does come up sometimes (6 solves for one `verify_baseline` run, 8
  samples for the sweep start) and then cannot be renewed at working height,
  which is why the tool's own guard is what stopped the run.

That is a camera aiming/focus condition. Refocusing the lens or moving the
camera would be a physical change to the mount, which is explicitly out of scope
here, so I stopped rather than adjusting hardware. Step 5's before/after
board-mm comparison therefore has **no new number**, and none was invented.

## 6. Constraints honoured — and one unintended jaw motion

- **Gripper: two unintended jaw motions, disclosed and ended where they started.**
  At 09:02 I ran `POST /home` to put the arm at the sweep's marker-refresh pose.
  `/home` commands all six joints from `home_joints`, which includes a stored jaw
  value, so the jaws went **26.77° → 37.95°**. Then the A/B tool's
  `--anchor-pose` (its first version sent all six values) commanded 26.77 again
  and they ended at **26.96°** — the starting value. No object was present, no
  grasp was attempted, nothing was obstructed. The tool now sends only the five
  arm joints, so an anchor return holds the jaws at whatever they measure. I did
  not restore the jaws deliberately: that would have been a third actuation.
- `live/board.json`, the mount calibration and the console config are unchanged
  (rig-state check PASS after the config restore).
- P/I/D untouched; the bridge was never restarted.
- All motion: 10° A/B steps with ±3.5° take-up legs, the 58° return to the
  recovery/home poses, and verify_baseline's 4° jog + 10 mm nudge. No sweep.
- No commit; `live/app.py` is modified and the new tools/tests/receipts are
  untracked, for the parent to commit.
- BC-250 `b1`/handeye files untouched.
