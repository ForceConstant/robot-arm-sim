# Deadband Step 2 — restore the factory P gain (16 → 32), measured

**Date:** 2026-11-25, 07:57–08:12 EST
**Owner-approved change:** Brian, 2026-11-25 — STS3215 position proportional gain
16 → 32 (Feetech factory default), nothing else.
**Verdict:** **P=32 helped, and only where the physics says it should — on the
loaded direction, where the droop roughly halved. No revert condition was met.**
The arm is left running at P=32.

---

## 1. What changed

`live/pi_bridge/app.py`, `_open_follower()`:

```python
robot = SO101Follower(SO101FollowerConfig(port=self.follower_port,
                                          id=self.follower_cal,
                                          position_p_coefficient=32,
                                          position_i_coefficient=0,
                                          position_d_coefficient=32))
```

I and D are now spelled out explicitly even though they equal LeRobot's
defaults, so the intent is visible in the code rather than inherited from a
library default that could drift. A comment above the call records the factory
value (32), what LeRobot ships (16), the measured droop at 16, and that this is
a single-variable experiment.

This is the only lever that mattered: `SO101Follower.configure()` writes
P/I/D to all six servos on **every** bus open, so the config — not the servo
EEPROM — is the source of truth (verified against the installed
`lerobot 0.6.1` source on the Pi, see §2).

### Deployment

| item | value |
|---|---|
| Pi backup | `/home/pi/so101-bridge/app.py.bak-20261125-080026` (previous file, 61 335 B) |
| deployed file md5 | `e1a199f141b04a7968450209f79bba3c` (repo and Pi identical) |
| py_compile on Pi | OK (`/home/pi/.local/share/uv/tools/lelab/bin/python`) |
| restart | `sudo systemctl restart so101-bridge`, `active` |
| came up | `mode REMOTE`, `follower_open true`, `joints_source follower`, `torque enabled` |
| fault / error | none |

## 2. The gain actually took — register proof

Read with the bridge **stopped** (`tools/servo_deadzone_dump.py
/dev/ttyACM0 1,2,3,4,5,6`), i.e. straight from the servos after the new config
had been written by `configure()` on the 08:01:16 open. Read-only, no writes.

| servo id | model | P | I | D | CW/CCW dead zone | Operating_Mode |
|---|---|---|---|---|---|---|
| 1 | 777 | **32** | 0 | 32 | 1 / 1 | 0 (position) |
| 2 | 777 | **32** | 0 | 32 | 1 / 1 | 0 |
| 3 | 777 | **32** | 0 | 32 | 1 / 1 | 0 |
| 4 | 777 | **32** | 0 | 32 | 1 / 1 | 0 |
| 5 | 777 | **32** | 0 | 32 | 1 / 1 | 0 |
| 6 | 777 | **32** | 0 | 32 | 1 / 1 | 0 |

All six read P=32 (was 16), I and D unchanged at 0/32, dead zone still at the
factory minimum 1/1. Single variable: P only. Raw dump:
`/tmp/p32_dump_20261125.json` on the Pi (BC-250 copy `/tmp/p32_dump.json`).

## 3. Before / after, same discriminator, same joints, same steps

`tools/deadband_discriminator.py`, ±10° steps, 3 repeats per direction,
`speed_scale 0.25`. Residual = commanded goal − `Present_Position` at settle,
positive means the joint stopped short of the goal.

### shoulder_pan — gravity-neutral

| direction | P=16 reps | median | P=32 reps | median | change |
|---|---|---|---|---|---|
| +10° | +0.33 / +0.33 / +0.59 | **+0.33** | −0.11 / +0.15 / +0.24 | **+0.15** | −0.18° |
| −10° | −0.24 / −0.24 / −0.33 | **−0.24** | −0.15 / −0.24 / −0.16 | **−0.16** | +0.08° |

### elbow_flex — loaded (the joint the hypothesis is about)

| direction | P=16 reps | median | P=32 reps | median | change |
|---|---|---|---|---|---|
| +10° (gravity-assisted) | +0.07 / +0.50 / +6.13* | **+0.50** | +0.06 / +0.51 / +5.34* | **+0.51** | +0.01° (nil) |
| −10° (gravity-loaded) | −1.38 / −1.56 / −2.09 | **−1.56** | −0.68 / −0.77 / −0.86 | **−0.77** | **−0.79° (−51%)** |

\* the third +10° repeat is the travel-limit corner the root-cause report already
flagged: the goal had walked to 76.4° (P=16) / 77.0° (P=32) and the joint
physically stops near 70–72° with the arm extended. It is present in **both**
conditions, slightly smaller at P=32 (6.13 → 5.34°), and is a reach limit, not
droop. Excluding it, the +10° pairs are 0.07/0.50 vs 0.06/0.51 — identical.

Results reproduce the P=16 baseline in §Root-cause (shoulder_pan +0.33/−0.24 vs
+0.41/−0.33; elbow −1.56 vs −1.74; same signs, same magnitudes, same
travel-limit outlier), so the comparison is valid.

### What the pattern says

- **Loaded direction halves, unloaded direction does not move.** −1.56 → −0.77
  is a factor 0.49; doubling Kp predicts 0.5 for `residual = load/Kp`. The +10°
  direction has gravity helping rather than loading the joint, and it is
  unchanged to 0.01°. That is the signature of a proportional-droop fix, not a
  generic "tighter servo" effect — it only helps where there is load to resist.
- **The pan numbers are now at the mechanical floor.** ±0.15–0.24° residual on a
  gravity-neutral joint, with the sign still flipping between directions, is
  hysteresis/backlash plus one encoder count (0.088°), not loop stiffness. More
  P would not buy anything here.
- **The elbow's remaining 0.77° is the droop that I=0 still cannot remove.** With
  P doubled it is under the 1° target, but it is still load-proportional: it will
  grow on a heavier payload.

## 4. Failure signatures — the revert checklist

| signature of too much gain | observed at P=32 | verdict |
|---|---|---|
| tail_spread > 0.2° | **0.000°** in all 12 settle tails | pass |
| oscillation / hunting | 557 state samples over ~6 s on a held pose: p2p **0.000°**, sd 0.000 on both joints | pass |
| a LARGER residual | none; every direction equal or better (elbow −10° halved) | pass |
| rising current / audible buzz | **not directly measured** — the bridge owns the serial port under load and there is no shunt or mic in the loop. Substitute evidence: zero position jitter, unchanged tail spread, no bus errors, no fault, `commands_rejected 0`. | no evidence of either |

No revert condition met → **no revert performed.** The Pi keeps the new
`app.py`; the backup stays in place at
`/home/pi/so101-bridge/app.py.bak-20261125-080026`.

## 5. Regression check

`tools/verify_baseline.py` on the BC-250, twice:

- Run 1: 6/7 — FAIL on "aim point ok on a fresh marker plane" (*plane stale:
  last solve 57 s ago, limit 30 s*). That check only samples while the MJPEG
  stream is being pulled and nothing was pulling it at that moment.
- Run 2: **7/7 PASS** — rig state, console REMOTE/no-fault, mink config and
  3.0 mm gate, fresh marker plane (9 solves, 10.6 s), **actuation jog moved
  the arm 3.78°**, rel-mode 10 mm nudge accepted, arm returned with max drift
  0.97°.

Final pose: `shoulder_pan −70.33, shoulder_lift −17.54, elbow_flex 46.90,
wrist_flex 78.46, wrist_roll 0.00, gripper 26.77`, mode REMOTE, no fault.
Sane, and the same neighbourhood it started in.

## 6. What this implies for the "near 1°" goal

- **P=32 is the right setting to keep.** It is the manufacturer's value, it
  halves the loaded-direction droop, and it costs nothing measurable in
  stability. Reverting to 16 would give back 0.8° on the one joint that matters.
- **The residual is now two separate terms with two separate fixes:**
  1. ~0.77° of load-proportional droop on the loaded elbow. `I=0` is still the
     only reason a constant gravity load cannot be nulled; a modest I (as the
     root-cause report proposes, 8–16, bounded against windup at the travel
     limits) is the next single-variable step. Expect it to remove this term,
     not reduce it by half.
  2. ~0.2° of direction-dependent hysteresis on the gravity-neutral joint, which
     is mechanical. The existing overshoot primitive is the right tool: it drove
     a 1.49° direct-command stall to 0.26° by approaching from the other side.
- **Do not expect P alone to reach a repeatable 1°.** It gets the loaded elbow
  from 1.6° to 0.8°, which is inside the target for this pose, but that number
  scales with payload. The reason to go on to I is that it is the term that
  makes the error payload-independent.

## 7. Constraints honoured

- No commit: `live/pi_bridge/app.py` is modified and uncommitted (plus the two
  JSON captures and this receipt); the parent commits.
- No gripper actuation (gripper held 26.77° throughout), no `live/board.json`,
  no mount-calibration change, no I or D change.
- All motion was 10° steps from the discriminator, plus the 4° jog and 10 mm
  nudge inside `verify_baseline.py`.
- BC-250 `b1`/handeye files untouched.

**Artefacts:** `reports/deadband_step2_before_P16.json`,
`reports/deadband_step2_after_P32.json` (raw per-rep records and summaries).
