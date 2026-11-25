# Deadband Step 3 — a bounded integral gain, measured and rejected

**Date:** 2026-11-25, 08:08–08:31 EST
**Owner-approved change:** Brian, 2026-11-25 — add a *bounded* integral gain to the
STS3215 position loop (P=32, D=32 held, single variable).
**Verdict: the integral was measured, it did exactly what it promises, and it was
REVERTED to I=0.** I=8 removed the load-proportional droop completely
(loaded elbow −0.81° → −0.15°) with no overshoot and no hunting on reachable
poses — but it introduced integral windup against a target the arm cannot reach:
after a 2 s stall the servo ignored reverse commands for **more than 10 s**.
I=4 behaved the same (it needed a **7° reverse command** before the joint moved at
all). I=0 is restored, registers verified, `verify_baseline.py` 7/7.

The droop is *not* gone. It is back exactly where it was, and the reason is now
measured rather than assumed: this servo exposes an integral term with no
anti-windup clamp we own, and a robot arm that silently ignores a 5° correction is
worse than one that stops 0.8° short.

---

## 1. Did the baseline reproduce?

`tools/deadband_discriminator.py`, shoulder_pan + elbow_flex, ±10°, 3 reps,
`speed_scale 0.25`, on the running P=32/I=0 bridge:

| joint | direction | reps (deg) | median |
|---|---|---|---|
| shoulder_pan | +10° | −0.02 / +0.15 / +0.24 | **+0.15** |
| shoulder_pan | −10° | −0.15 / −0.24 / −0.16 | **−0.16** |
| elbow_flex | +10° | −0.02 / +0.59 / +5.26\* | **+0.59** |
| elbow_flex | −10° | −0.60 / −0.68 / −0.94 | **−0.68** |

\* the third +10° rep is the known unreachable-reach corner (goal walked to
76.5°, joint stops near 71°). It appears in every run since the root-cause report
and is excluded from the medians below.

Step 2's loaded-elbow −0.77° sits inside today's per-rep spread (−0.60…−0.94), and
the independent reversal probe put the same direction at **−0.81°**. Baseline
reproduced; the experiment proceeded.

## 2. The measurement built for this step

`tools/deadband_reversal_probe.py` (new). An integrator does not fail by leaving a
residual, so the protocol measures what an integrator breaks:

- settle at A, command B = A ± 10°, **sample at 19.9 Hz** (median; minimum 19.9,
  requirement ≥15) through the move and the hold,
- report **(a)** peak excursion past B, **(b)** settle residual at B,
- report **(c)** sign reversals in the last 500 ms and the tail's peak-to-peak
  spread, with sub-encoder-count dither (one count = 0.088°) excluded so
  quantisation is not scored as oscillation.

Two modes were added after the first results raised a question the short probe
could not answer:

- `--hold-watch` — hold the target for 8 s and print the spread in consecutive
  500 ms windows. A single 500 ms tail cannot distinguish a decaying transient
  from a limit cycle; the window sequence can.
- `--limit-probe` — drive a joint into a goal it cannot reach in 10° steps, hold
  against the stall for 2 s (where an integrator accumulates), then reverse one
  step and watch. This is where integrators bite, and it is the test that decided
  the outcome.

## 3. Register proof — the gain really took

Read with the bridge **stopped** (`tools/servo_deadzone_dump.py /dev/ttyACM0
1,2,3,4,5,6`), i.e. straight from the servos after `configure()` had written them
on open. Read-only, no writes. Raw dumps in `reports/i*_dump_20261125.json`.

| condition | served file (md5) | servos 1–6 P/I/D | dead zone | torque |
|---|---|---|---|---|
| I=8 deployed | `4e18d524…` | **32 / 8 / 32** on all six | 1 / 1 | enabled on reopen |
| I=4 deployed | `4e18d524…` (value edited) | **32 / 4 / 32** on all six | 1 / 1 | enabled on reopen |
| reverted | `455dbffe…` | **32 / 0 / 32** on all six | 1 / 1 | enabled on reopen |

Nothing else moved: P and D stayed 32/32 in every condition. Each restart came up
`mode REMOTE`, `follower_open true`, `joints_source follower`, `fault null`,
`commands_rejected 0`, and the pose was identical before and after each
stop/start cycle on the servos (no sag while torque was off).

## 4. Before / after — settle and overshoot

Reversal probe, 2 reps per direction, 19.9 Hz. Residual = commanded − reached, so
positive means it stopped short. "Peak past B" is the worst excursion beyond the
target in the direction of travel.

| joint / direction | I=0 residual | I=8 residual | I=0 peak past B | I=8 peak past B |
|---|---|---|---|---|
| shoulder_pan +10° | +0.11 | +0.11 | 0.00 | 0.20 |
| shoulder_pan −10° | −0.15 | +0.07 | 0.11 | 0.11 |
| elbow_flex +10° | +0.15 | −0.03 | 0.00 | 0.20 |
| **elbow_flex −10° (loaded)** | **−0.81** | **−0.15** | 0.00 | 0.00 |

The discriminator, same protocol as the baseline, says the same:

| joint / direction | I=0 median | I=8 median | change |
|---|---|---|---|
| shoulder_pan +10° | +0.15 | +0.15 | 0 |
| shoulder_pan −10° | −0.16 | −0.06 | +0.10 |
| elbow_flex +10° | +0.59 | +0.15 | −0.44 |
| **elbow_flex −10°** | **−0.68** | **+0.20** | **+0.88 — sign flipped** |

That is the integrator doing its job: the loaded direction no longer sits short,
it now sits a hair *past* the goal on two of three reps (−0.41 / +0.20 / +0.63),
which is the integral having erased a load error it had accumulated earlier in the
sequence. Residual is no longer payload-proportional; it is payload-independent
noise of ~0.2°.

**Overshoot did not grow materially** (0.11 → 0.20°, i.e. two encoder counts, far
under the 1.5° revert line) and **hunting did not appear on any reachable pose**.

## 5. Hunting: the 2.6 s probe said maybe, the 8 s hold said no

The reversal probe reported `tail_spread 0.44°, 1 reversal` on `shoulder_pan +10°`
at I=8 — over the 0.2° line, which is a revert criterion. That number alone would
have been enough to reject I=8, so it was tested properly instead of argued about.

`--hold-watch`, 8 s at 20 Hz, 3 reps per direction, spread per 500 ms window
(degrees):

| joint / dir | windows 0→8 s | last 1 s | reversals |
|---|---|---|---|
| shoulder_pan +10° (×3) | 2.6 3.6 1.9 0.00 0.00 … | **0.000** | 0 |
| shoulder_pan −10° (×3) | 2.5 3.4 2.1 0.00 0.00 … | **0.000** | 0 |
| elbow_flex +10° (×3) | 2.8 3.4 1.5 0.00 0.00 … | **0.000** | 0 |
| elbow_flex −10° (×3) | 0.1 3.0 2.3 0.97 0.70 0.53 0.44 0.00 … | **0.000** | 0 |

Every run reaches exactly 0.000° within ~1.5–2 s and stays there for the
remaining 6 s, with zero reversals. The 0.44° was a decaying transient caught by
the short window, not a limit cycle. **Hunting: absent at I=8** — the honest
statement is that the 2.6 s criterion is too short to judge an integrating loop,
which is why the 8 s window exists now.

## 6. The failure that decided it — windup at an unreachable goal

The discriminator has been walking joints into the elbow's reach corner since the
root-cause run: the goal goes to 76°, the arm physically stops near 71°, and the
error (≈5°) cannot be reduced. This is the case where a proportional loop just
sits still, and where an integrator accumulates.

Protocol: 10° steps to a goal of 76°, hold the stall 2 s, then reverse 10° to 66°
and sample for **10 s**.

| condition | stalled at | error at stall | reversal to 66° | released after | final |
|---|---|---|---|---|---|
| **I=0 control** (restored mid-experiment) | 71.16 | +4.84 | 66.0 | **0.70 s** | 66.15 (residual −0.15) |
| **I=8** | 71.08–71.16 | +4.84 | 66.0 | **never, in 10 s** | 71.08 (residual −5.08) |
| **I=8 (first run, 2.6 s window)** | 71.16 | +4.84 | 66.0 | never, in 2.6 s | 71.25 (−5.25) |
| **I=4** | 71.96 | +4.04 | 66.0 | 0.45 s then stalled again | 71.34 (−5.34) |
| **I=0 post-revert** | 71.78 | +4.22 | 66.0 | **0.55 s** | 66.24 (−0.24) |

At I=8 the joint simply did not move for the whole 10 s window. A trace at I=4
(shorter hold, then a 1°-per-1.2 s reverse ramp) quantifies what "windup" means
operationally here:

| reverse target | 70.2 | 69.2 | 68.2 | 67.2 | 66.2 | 65.2 | **64.2** |
|---|---|---|---|---|---|---|---|
| joint position | 71.16 | 71.16 | 71.16 | 71.16 | 71.16 | 71.16 | **68.44** |

The joint ignored every correction until the command was **7°** away from where it
was stuck; at 6° it was still frozen. That is not "a slower settle", it is a
position loop that has been taught to distrust small errors — and a 5° correction
being silently dropped is a safety problem on an arm, not a tuning nuisance.

Nothing else was wrong while this happened: `fault null`, `error null`,
`commands_rejected 0`, bridge in REMOTE, the other joints moving normally (pan
jogged 5° on request during one of the frozen windows). The freeze clears
eventually, and a 25° command later pulled the joint free immediately — which is
consistent with the integral term, not with a damaged servo.

**This is outside the revert list but strictly worse than it.** The list's spirit
is "any behaviour that makes the loop less trustworthy than P-only"; a joint that
ignores a 5° reversal for 10 s while reporting a healthy state fails that
decisively, so I=0 was restored rather than shopping for an even smaller I.

## 7. Revert — what was restored and how it was proven

| item | value |
|---|---|
| Pi file restored from | `/home/pi/so101-bridge/app.py.bak-20261125-081140` (md5 `e1a199f1…`, the P=32/I=0 file) |
| then deployed | repo file `live/pi_bridge/app.py` md5 `455dbffe…` — same values, carrying the step-3 finding in the comment |
| py_compile on Pi | OK (`uv` lelab python) |
| restart | `systemctl restart so101-bridge` → `active` |
| came up | REMOTE, follower_open, joints_source follower, torque enabled, fault null |
| registers after revert (bridge stopped) | **I=0, P=32, D=32 on all six**, dead zone 1/1 |
| pose across the stop/start | identical (−64.88 / −15.52 / 64.40 / 77.58 / −1.05 / 26.77) |

Post-revert, the same two probes:

- discriminator: pan +0.16 / −0.07, elbow +0.15 / **−0.77** — the loaded droop is
  back to the step-2 value, exactly as expected from P-only.
- limit probe: released **0.55 s** after a 2 s stall, landed at 66.24. Behaviour
  restored.

## 8. verify_baseline — and an incident worth recording

First attempt failed three checks, and the cause was **not** the gain work:

```
POST /jog  → 409 Conflict                      (actuation: moved 0.00 deg)
POST /move → 200, rel 10 mm                     (used a plane 781 s stale)
result: max drift 58.72 deg
```

Diagnosis, in order:

1. The console's camera client died at 08:11:47 with `ConnectionResetError` — that
   is the moment I restarted the bridge. It never reconnected, so no marker
   solves ran, so the aim plane aged to 921–976 s.
2. With a stale plane the board frame was nonsense (`board.z = 432.6 mm`, yaw
   −175.7°), and the rel-mode 10 mm nudge was solved in that garbage frame. The
   arm swung 58.7° to a different IK branch — `keep-in` still reported `inside
   true`, no fault, no rejected command.
3. The `/jog` 409 was the console's own gate refusing to move on that state, so
   the "actuation" check correctly reported 0.00°.

Recovery: returned the arm to the 2026-11-25 baseline pose in ≤8° bridge-side
legs, every leg moved (no stalls, no faults, gripper untouched at 26.77), then
restarted `robot-live.service` to reattach the camera. Stream delivered 3.6 MB
again, plane went fresh (rms 2.9 px, age <10 s), and:

```
PASS rig state   PASS console REMOTE/no fault   PASS config mink/3.0 mm
PASS aim point ok on a fresh marker plane [9 solves, 1.36 s]
PASS actuation: 4 deg jog moves the arm [moved 3.69 deg]
PASS rel mode accepts a 10 mm board nudge
PASS arm returned to its starting pose [max drift 1.06 deg]
BASELINE OK — the arm is controllable to the 2026-11-25 standard.
```

**Finding for the operator, not fixed here:** restarting `so101-bridge` silently
kills the console's camera client, and a rel-mode `/move` against the resulting
stale plane is accepted and acts on a bad board frame. The stale-plane check gates
`/aim-point` and `aim_point_move`, but the plain `/move` path does not refuse it.
That is pre-existing behaviour, outside this step's scope, and worth its own fix.

Final state: `shoulder_pan −70.51, shoulder_lift −15.78, elbow_flex 47.78,
wrist_flex 78.37, wrist_roll 0.00, gripper 26.77`, REMOTE, no fault, estop false,
`commands_rejected 0` — the baseline neighbourhood it started in.

## 9. Did the integral earn its place?

**No — not in this hardware, not yet.** Summing up honestly:

- It solved the stated problem. The load-proportional droop is gone at I=8
  (−0.81° → −0.15°, and the discriminator's −0.68° → +0.20° with the sign
  flipping), which is the thing P could never do. Overshoot grew by two encoder
  counts; hunting never appeared on a reachable pose, including 8 s holds.
- It broke something worse. With no anti-windup, ~5° of persistent error against
  an unreachable target turns the loop into one that ignores small corrections.
  The elbow's reach corner is not exotic — the discriminator walks into it in
  every run — so this is a normal operating condition, not an edge case.
- I=4 did not help. Same lockup, with a measured ~6–7° dead zone against
  reversal. Smaller is not safer here; the mechanism is unchanged, only scaled.

What would make an integral usable, in the order I would try them:

1. **Anti-windup at the bridge, not the servo.** The servo's I register has no
   clamp we own, but the bridge already has a stall guard. On detected stall,
   drop the unreachable goal and re-target to the measured position instead of
   holding an error the loop cannot clear. That converts the failure mode into a
   no-op. Then I=8 becomes a genuine candidate and should be re-measured with
   this same probe.
2. **Bound the error that reaches the integrator.** Ramp goals in small
   increments (the discriminator's 10° step is what parks the loop at the stop);
   at minimum this keeps the accumulation small, though it does not remove the
   steady state at a hard reach limit.
3. If payload droop becomes a real problem before either exists, get the
   payload-independence back at the application layer: the existing overshoot
   primitive already converted a 1.49° direct-command stall into 0.26° by
   approaching from the other side, and it cannot wind up.

Until one of those exists, **P=32, I=0, D=32 stands** — the manufacturer's
proportional and damping values with no integral, which is what the arm is running
now.

## 10. Constraints honoured

- One variable at a time: I only. P=32 and D=32 in every condition, verified in
  the registers three times.
- No commit. `live/pi_bridge/app.py` is modified and uncommitted (documented
  comment + I reverted to 0); `tools/deadband_reversal_probe.py` and
  `tools/deadband_step3_card.py` are new and uncommitted. The parent commits.
- No gripper actuation (held 26.77° throughout), no `live/board.json`, no mount
  calibration change, BC-250 `b1`/handeye files untouched.
- All probe motion ≤10° steps; the limit probe stayed inside the calibrated elbow
  range (76° against a 94.75° limit) at a reach corner that has been exercised in
  every run since 2026-11-25.
- Compute on the BC-250 only; the Pi only served the bridge.

**Artefacts** (`reports/`): `deadband_step3_{before,i8,revert}_discriminator.txt`,
`deadband_step3_{before,i8}_overshoot.{json,txt}`,
`deadband_step3_i8_holdwatch_{pan,elbow}.json`,
`deadband_step3_limit_{i0_control,i8,i8_short,i4,i0_after}.json`,
`i{8,4}_dump_20261125.json`, `i0_revert_dump_20261125.json`,
`card_{before,i8,final}.png`.
Tools: `tools/deadband_reversal_probe.py`, `tools/deadband_step3_card.py`.
