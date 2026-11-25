# Deadband investigation — measured root cause (2026-11-25)

**Question:** the arm settles 2–4° short of a commanded joint position. Is that a
servo dead-zone setting, a mechanical limit, or something tunable?

**Answer: it is proportional-controller droop under load. The servo's dead-zone
register is already at minimum and has nothing to do with it.**

## The register dump (read-only, bridge stopped, all six servos identical)

| register | value | note |
|---|---|---|
| `CW_Dead_Zone` / `CCW_Dead_Zone` | **1 / 1** | 0.088° — already the minimum. Not the cause. |
| `P_Coefficient` | 16 | LeRobot default |
| `I_Coefficient` | **0** | **no integrator — cannot remove steady-state error** |
| `D_Coefficient` | 32 | LeRobot default |
| `Minimum_Startup_Force` | 16 | low |
| `Angular_Resolution` | 1 | 0.088° per tick |
| `Acceleration` | 254 | |
| `Torque_Limit` | 1000 | 100% |
| supply | 12.1–12.3 V | healthy under load |

Those P/I/D values are exactly `SO101FollowerConfig`'s defaults
(`position_p_coefficient=16`, `position_i_coefficient=0`,
`position_d_coefficient=32`), and `SO101Follower.configure()` **writes all three
on every bus open**. So the gains are not a servo property we would have to flash
— they are imposed by software we own, every time the bridge starts.

## The discriminator (3 repeats per direction, ±10° steps)

Residual = commanded goal − `Present_Position` at settle. Tail spread 0.000° in
every case, so these are settled values, not samples mid-flight.

| joint | load | +10° residual (median) | −10° residual (median) |
|---|---|---|---|
| `shoulder_pan` | gravity-neutral | **+0.41°** | **−0.33°** |
| `elbow_flex` | loaded | +0.41° | **−1.74°** |

Three separate facts:

1. **The servo knows it is short.** `Present_Position` settles 1.7–1.9° away from
   `Goal_Position` on the loaded joint. The error is inside the servo's own loop,
   not in a horn or printed adapter downstream of the sensor — which is what the
   earlier backlash hypothesis predicted and what this measurement rules out as
   the primary cause.
2. **It scales with load.** Unloaded ~0.4°, loaded ~1.8°. That is the signature of
   `residual = load / Kp`: with `I = 0` there is no term that can drive a constant
   gravity load to zero error.
3. **It is directional but only weakly.** The sign flips between +10° and −10°
   (+0.41 vs −1.74), which accounts for the previously measured 3.78° difference
   between approach directions. Some hysteresis is real — call it ~0.7° of
   geartrain backlash on top of the droop — but it is the smaller term.

One outlier worth noting: on the third `elbow_flex` +10° repeat the goal walked up
to 73.96° and the joint reached 69.41° (4.55° short). That pose is near the elbow's
working limit with the arm extended, so it is more likely a reach/gravity corner
than a fourth mechanism.

## What this means for the plan

- **Firmware is not the lever.** `roboninecom/STS32XX-Firmware` advertises tunable
  PID and gravity compensation, but the PID we need to tune is already exposed by
  `SO101FollowerConfig` and re-written on every open. Flashing GD32F130 firmware
  would add bricking, batch-compatibility and protocol risk to change numbers we
  can already change in Python.
- **The fix is a two-line config change plus a proven primitive.** Raise
  `position_i_coefficient` from 0 (modest, e.g. 8–16) so the loop can null a
  constant load, and optionally raise P 16→24. Test on one loaded joint, revert on
  oscillation.
- **Keep the overshoot primitive** for the residual hysteresis: driving
  `wrist_roll` to −1.8° and back to 0 landed at −0.26° where a direct command
  stalled at 1.49°. It is already the right tool for the ~0.7° that remains.


## Vendor register table — and why the plan changed

From the ST3215 register documentation (Mickael-Roger/python-st3215, which
transcribes the Feetech table):

| addr | register | factory default | range | unit | ours | note |
|---|---|---|---|---|---|---|
| 0x15 | Position loop **P** | **32** | 0–254 | — | **16** | **we run at half the factory stiffness** |
| 0x16 | Position loop D | 32 | 0–254 | — | 32 | matches |
| 0x17 | Position loop I | 0 | 0–254 | — | 0 | matches |
| 0x18 | Minimum starting force | 16 | 0–1000 | — | 16 | matches |
| 0x1A | CW insensitive zone | 1 | 0–32 | step (0.088°) | 1 | already minimum |
| 0x1B | CCW insensitive zone | 1 | 0–32 | step (0.088°) | 1 | already minimum |
| 0x29 | Acceleration | 0 | 0–254 | 100 step/s² | 254 | **SRAM, not EEPROM** — resets each power-up; 254 = max |

Two consequences:

1. **The dead-zone registers are at the factory minimum too**, not a mis-set
   value. They were never the mechanism.
2. **`SO101FollowerConfig` ships P=16, which is half the servo's design value of
   32.** With I=0, a lower Kp means a larger steady-state error under the same
   load — exactly the `residual = load/Kp` behaviour measured above. So the
   cheapest correct experiment is not "add an integrator": it is **restore the
   manufacturer's P of 32**, which is a known, bounded, well-understood value
   rather than a new control term.

Integral gain stays as the second step, only if P=32 leaves a residual, because
an integrator adds windup behaviour at the travel limits that proportional gain
does not.

Revised order: P 16 → 32 first (factory default), measure with
`tools/deadband_discriminator.py`; only then consider a small I.

## Open items

- Feetech manual values for `Minimum_Startup_Force` semantics and whether
  `Acceleration=254` means "max" or "unlimited" — needed before claiming safe
  bounds for a gain change.
- Integral windup behaviour at the travel limits, where the error cannot be
  reduced: the integrator must be bounded or it will overshoot on reversal.
- Horn-screw / printed-adapter condition (visual only) to size the hysteresis
  term properly.
