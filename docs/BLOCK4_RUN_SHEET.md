# Block 4 — IRL bench run sheet (2026-11-18)

Read this straight through before touching anything. Every step has an exact
command and a pass condition. If a step fails, stop at that step — do not
improvise a workaround at the bench.

**State measured 2026-11-18 09:30 (read-only, before any motion):**

| field | value |
|---|---|
| bridge `mode` | `HOLD` |
| `armed` | `false` |
| `follower_open` | `false` |
| `joints_source` | `none` |
| `lelab_active` | `false` |
| joints | all `0.0` |
| BC-250 live app (8080) | **down** |

Power alone does not mean the servos are readable: `follower_open=false` and
`joints_source=none` mean the bridge has no joint feedback yet.

## Roles and abort authority

- **Brian** — at the arm: power, the physical e-stop, opening the follower at
  the Pi console, and the last word on any "carry on?" question.
- **Agent** — issues the HTTP commands, reads state, records results.
- **Either one calls an abort, immediately and without discussion.** The e-stop
  button beats every software abort. Nobody argues with an abort call mid-run.

Host for every command below: **BC-250, CachyOS**, `brian@192.168.1.171`,
`cd /home/brian/robot-arm-sim`. The Pi bridge is `http://192.168.1.167:8100`;
the live app is `http://127.0.0.1:8080` on the BC-250.

---

## 0. Preconditions (no motion)

```fish
cd /home/brian/robot-arm-sim
git rev-parse --short HEAD          # expect the Block 4 commit
git status --porcelain              # expect only live/ untracked files
grep '"move_tolerance_mm"' live/live_config.json   # expect 20.0
grep '"marker_z_m"' live/live_config.json          # expect 0.4337
~/.local/bin/robot-arm-sim python tools/block4_dryrun.py   # offline, no arm
```

**Pass:** tree clean apart from `live/`, gate 20, `marker_z_m` 0.4337, and the
dry-run harness prints its verdict table (it exits 1 if any square is not `OK` —
that is information, not a bench blocker; read the verdicts).

**Abort:** the gate is not exactly 20.0, or `marker_z_m` is 0.43056. Fix before
going on; a scan with the mat plane cannot pass the fusion RMS gate.

## 0b. The scan plan must match the printed face (pre-bench gate)

```fish
grep -E '"plan"|"layout_kind"' live/live_config.json
python3 -c "import json;d=json.load(open('chess_sim/assets/marker_scan_plan_charuco.json'));print('offset',d['layout_offset_mm'],'ids',sorted({m for p in d['poses'] for m in p['visible']})[:4],'... n=',len({m for p in d['poses'] for m in p['visible']}))"
```

**Pass:** `layout_kind` is `charuco`, `plan` ends in
`marker_scan_plan_charuco.json`, `offset 0.0`, and the ids are exactly 5–36
(32 of them).
**Abort:** `plan` still points at `marker_scan_plan.json`, or the ids print
`[0, 1, 2, 3]`. That is the mat-tag plan (offset 152, tags 40 mm outside the
board edge); with the ChArUco face those poses look *past* the board and
`/scan` comes back with no markers, which reads like a hardware fault. Point
`live_config.json` at `chess_sim/assets/marker_scan_plan_charuco.json` — do not
work around it at the arm.

The ChArUco plan (Block 4b, 11 poses) was validated in sim at nominal and at
±8 mm x/y, ±2.5° yaw: 7/7 placements register, 32 markers per scan, ≤0.21 mm XY
and ≤0.025° yaw. `reports/scan_plan_validation.json` is the baseline.

## 1. Pi and bridge alive

```fish
curl -s --max-time 5 http://192.168.1.167:8100/health
curl -s --max-time 5 http://192.168.1.167:8100/state
```

**Pass:** `/health` answers, `/state` returns `"protocol":1`, `fault":null`,
`error":null`, and the limits below.

| joint | bridge limit (°) |
|---|---|
| shoulder_pan | −108.99 … 121.38 |
| shoulder_lift | −107.05 … 108.64 |
| elbow_flex | −96.68 … 94.75 |
| wrist_flex | −104.59 … 98.79 |
| **wrist_roll** | **−81.82 … 91.05** |
| gripper | 0 … 100 |

**Abort:** `fault` non-null, `error` non-null, or a limit differs from this table.
A different limit invalidates the whole Block 4 sweep; re-run it, do not eyeball.

## 2. Port ownership — stop LeLab/teleop *before* touching the follower

On the **Pi console** (Brian — the bridge has no follower endpoint):

```fish
lsof /dev/ttyACM0 /dev/ttyACM1 2>/dev/null    # whichever the arm enumerates as
```

**Pass:** after stopping LeLab/teleop, the only holder is the bridge process.
Confirm from the BC-250:

```fish
curl -s http://192.168.1.167:8100/state | python3 -m json.tool | grep -E "lelab_active|follower_open|joints_source"
```

**Expect:** `"lelab_active": false`.
**Abort:** `lelab_active: true`. Two processes on one serial port will fight and
one will start commanding the arm. Do not open the follower until it is false.

## 3. Open the follower and verify joint feedback

Brian opens the follower at the Pi console (LeLab/lerobot follower service, as
used on 11-17). Then, from the BC-250:

```fish
curl -s http://192.168.1.167:8100/state | python3 -m json.tool
```

**Pass all three:**
- `"follower_open": true`
- `"joints_source"` is no longer `"none"` (typically `"follower"`)
- `"joints_age_s"` is small (< 1 s) and keeps refreshing on a second call
- joints read near the last parked posture, not all zeros

**Abort:** `follower_open:true` but `joints_source:"none"`, or `joints_age_s`
climbing. That is a dead sensor bus; commanding in that state moves blind.

## 4. Start the live app on the BC-250

```fish
scripts/start-live.sh
curl -s http://127.0.0.1:8080/state | python3 -m json.tool | head -40
```

**Pass:** `/state` returns a `bridge` block whose `mode` is `HOLD`, and no
`"error"` key at the top level.
**Abort:** the app starts but `bridge.error` is set — the token or URL is wrong;
do not continue to arming.

## 5. REMOTE, then arm (first motion authority)

```fish
curl -s -X POST http://127.0.0.1:8080/mode -H 'Content-Type: application/json' -d '{"mode":"REMOTE"}'
curl -s -X POST http://127.0.0.1:8080/arm  -H 'Content-Type: application/json' -d '{"confirm":true}'
curl -s http://127.0.0.1:8080/state | python3 -m json.tool | grep -E '"mode"|"armed"'
```

**Pass:** `"mode": "REMOTE"`, `"armed": true`.
**Abort:** `/arm` returns 409, or `armed` stays false. Do not repeat-loop the arm
call; read `bridge.error` and stop.

*From here the arm is live.* Brian's hand stays near the e-stop for steps 6–12.

## 6. Seed the board frame from the saved pose

```fish
curl -s -X POST http://127.0.0.1:8080/board/assume -H 'Content-Type: application/json' \
  -d '{"x_mm":-13.74,"y_mm":-3.57,"z_mm":409.5,"yaw_deg":7.14}'
curl -s http://127.0.0.1:8080/board | python3 -m json.tool
```

**Pass:** `/board` echoes x −13.74, y −3.57, z 409.5, yaw 7.14.
**Abort:** values do not round-trip — `/board/assume` did not apply, and every
target after this point would be in the wrong frame.

This is a seed, not a measurement. Step 7 replaces it.

## 7. `/scan` — multi-pose ChArUco board calibration

```fish
curl -s -X POST http://127.0.0.1:8080/scan -H 'Content-Type: application/json' -d '{}'
```

Poll until it finishes (it moves the wrist through the saved scan plan):

```fish
curl -s http://127.0.0.1:8080/state | python3 -m json.tool | grep -E 'scan_running|"ok"'
# repeat every 5 s until scan_running is false
```

**Pass:** `scan_running:false` and `last_scan.ok == true`, with
`views_used >= 6` and RMS under the 1.2 px gate.
**Abort:** `ok:false`, or RMS above the gate. The old `marker_z_m=0.43056`
(mat plane) failed here with RMS 1.33–2.56 px; it is now 0.4337, so a repeat
means a real mounting/lighting problem — stop, do not loosen the gate.

## 8. Compare `board.json` against the scan

```fish
cat live/board.json
```

**Pass (all of):**
- |Δx| ≤ 3 mm, |Δy| ≤ 3 mm against −13.74 / −3.57
- |Δyaw| ≤ 1.5° against 7.14°
- z unchanged from 433.7 (the board plane is a *separate* fit, step 9)

**Abort:** anything larger. A large z or yaw change means the board was moved or
replaced: re-fit and re-derive, never nudge the saved numbers to match.

Write the new numbers down — they are the Block 5 baseline.

## 9. Board plane z (multi-marker planar fit)

`/scan` is **x/y/yaw only**; it does not estimate z. Fit z from the marker
sightings in the same captures (Block 3 procedure):

```fish
~/.local/bin/robot-arm-sim python tools/calibrate_board_charuco.py \
  --layout charuco --captures live/captures --out reports/board_charuco_irl.json
```

It prints one line per axis with the spread across captures.

**Pass:** `z_mm` within 0.5 mm of 433.7 and `worst_axis_spread_mm` ≤ 0.5.
**Abort:** either number is worse — the captures are bad (blur, glare, a
single-marker view). Re-scan rather than accept the fit.

Note the semantics from Block 3: on the ChArUco face this PnP median returns
the **playing surface**, the same plane `board_z_m` names, so the two numbers are
comparable. It is a different estimator from `/scan`, so do not average them —
if they disagree by more than 0.5 mm, that disagreement *is* the finding.

## 10. `/sanity` — corners and centre

```fish
curl -s -X POST http://127.0.0.1:8080/sanity -H 'Content-Type: application/json' \
  -d '{"execute":true,"z_mm":120.0,"tolerance_mm":6.0}'
```

**Pass:** every corner and the centre within 6 mm.
**Abort:** any point over 6 mm — the sweep would spend its whole budget
re-learning a calibration error. Stop and re-fit the board.

## 11. Eight-square dry run (no motion)

Offline, on the BC-250 — production solver, bridge limits, no arm traffic:

```fish
~/.local/bin/robot-arm-sim python tools/block4_dryrun.py --squares a1,b1,f1,g1,h1,a2,g2,g3
```

Live equivalent — solves and returns the joint target **without moving**
(requires `mode=REMOTE` **and** `armed=true`; it deliberately does *not* run the
keep-in check, so read the target joints, not just the 200):

```fish
curl -s -X POST http://127.0.0.1:8080/move -H 'Content-Type: application/json' \
  -d '{"mode":"xyz","x_mm":-98.0,"y_mm":-98.0,"z_mm":60.0,"dry_run":true}'
```

Expected reply shape: `{"dry_run":true,"requested":{...},"target_joints":{...},
"max_joint_delta_deg":N}`.

**Pass:** every square is `OK` offline, and each live dry run returns
`target_joints` with no joint past the table in step 1.
**Abort:** any `IK_FAILURE` or `JOINT_LIMIT` verdict. Those squares cannot be
reached on the real arm; the 64/64 target is not available and Block 5 must not
start. `KEEP_IN_REFUSAL` is different: the pose is legal and the live move would
be refused — decide that one with Brian before the sweep.

## 12. Full 64-square sweep

```fish
~/.local/bin/robot-arm-sim python tools/cartesian_squares.py --backend irl \
  --out reports/squares_irl_2026-11-18.json
~/.local/bin/robot-arm-sim python tools/cartesian_squares.py --compare \
  reports/squares_sim_bridge_limits.json reports/squares_irl_2026-11-18.json
```

**Pass:** the IRL count matches the sim real-limit count, and every disagreement
is explained by a physical cause, not a solver tweak.
**Abort:** a guard refusal or a joint-limit refusal is *not* unreachability —
record it and keep going; a sudden multi-square regression on one side of the
board is a frame problem — stop and re-scan.

## 13. Record

- `reports/squares_irl_2026-11-18.json`, the scan result, the new `board.json`.
- Every abort, with the step number and what the state showed.
- Return the arm to `HOLD` when done: `POST /mode {"mode":"HOLD"}`.

---

## Abort table (short form)

| step | condition | call it when | action |
|---|---|---|---|
| 0b | `plan` still the mat plan while layout is `charuco` | before the bench trip | point it at `marker_scan_plan_charuco.json` |
| 1 | `fault`/`error` non-null, limits differ | before anything moves | stop, report |
| 2 | `lelab_active: true` | before opening follower | stop LeLab first |
| 3 | `joints_source:"none"` with follower open | before arming | stop, dead sensor bus |
| 5 | `armed` stays false | at arming | stop, read `bridge.error` |
| 6 | board values do not round-trip | before scan | stop |
| 7 | scan `ok:false` / RMS > 1.2 px | during scan | stop, do not loosen gate |
| 8 | Δx/Δy > 3 mm or Δyaw > 1.5° | after scan | stop, re-fit |
| 9 | plane bias > 0.5 mm or spread > 0.5 mm | after plane fit | re-capture |
| 10 | any `/sanity` point > 6 mm | before sweep | stop, re-fit |
| 11 | any `IK_FAILURE` / `JOINT_LIMIT` | before sweep | no Block 5 |
| 12 | sudden multi-square regression | during sweep | stop, re-scan |

## Do not

- Do not start LeLab or teleop while the bridge owns the port.
- Do not loosen `move_tolerance_mm` (20 mm) or `MAX_RMS_PX` (1.2 px).
- Do not edit `marker_z_m` back to the mat plane.
- Do not treat `/scan` as a 6-DoF pose — z comes from step 9.
- Do not retune the solver to make an IRL square agree with sim.
