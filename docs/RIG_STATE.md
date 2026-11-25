# Rig state — what makes the code actually run this arm

The repository holds the **code**. It does not, on its own, hold the **rig**. A
fresh clone gets a console that boots cleanly, accepts commands and moves the
arm to the wrong place, because four calibration files are deliberately ignored
by git. That failure is silent, which is why this document exists.

Verified baseline: **2026-11-25**, after the board-delta correction and the mink
solver flip.

## The four files

| file in `rig_state/` | canonical path on the machine | what it decides |
|---|---|---|
| `board.json` | `live/board.json` | board pose in the arm frame (x, y, z, yaw). A wrong value moves every commanded square and fails silently. |
| `live_config.json` | `live/live_config.json` | solver (`mink`), move tolerance (3.0 mm), auth posture, home pose, layout. |
| `wrist_calibration.json` | `live/wrist_calibration.json` | wrist roll zero and the allowed roll window. |
| `mount_calibration.json` | `.private/phase11d/calibration.json` | camera-in-tool mount **and** the arm chain. Every projection, the aim point and every IK solve depend on it. |

`rig_state/MANIFEST.json` records each file's SHA-256, size and purpose.

They are ignored because `live/*.json` is machine state and `.private/` also
holds CAD, meshes and weights. The values are committed here instead, as data,
so the bench can be rebuilt from the repository.

## Restoring

On the machine that owns the arm (the BC-250):

```bash
cd /home/brian/robot-arm-sim
.venv/bin/python tools/rig_state.py verify     # do the committed values match this rig?
.venv/bin/python tools/rig_state.py restore    # write them back (backs up first)
systemctl --user restart robot-live            # the console reads board+config at startup
.venv/bin/python tools/verify_baseline.py      # prove controllability, with motion
```

`restore` verifies every checksum before writing anything — a half-applied rig
is worse than an untouched one. It refuses on mismatch unless `--force` is
passed.

## Re-snapshotting

After any deliberate recalibration:

```bash
.venv/bin/python tools/rig_state.py snapshot --note "what changed and why"
```

Then commit `rig_state/`.

## Home before every task

`AGENTS.md` makes this a standing rule: any work that moves the arm starts by
driving it to the stored home pose.

```bash
.venv/bin/python tools/home_arm.py --check   # report only
.venv/bin/python tools/home_arm.py           # home, then verify
```

Home is `recovery_joints` — a stored joint pose, never an IK target, so it works
even when the solver is broken. The gripper is carried through untouched.

Why it is a rule and not a habit: a task left the arm parked at an experiment
pose, off the board and low, and the next vision run reported no markers and a
soft image. That looks exactly like a camera fault and cost an hour of
diagnosis before anyone checked where the arm was pointing.

## Proving the arm is controllable

`tools/verify_baseline.py` checks seven things and exits non-zero on any
failure. It is not a config read: it jogs a joint and requires the arm to move,
because "commands accepted, arm still" is the exact failure this project spent
2026-11-24 chasing (torque off — 266 commands accepted, 0.00° of travel).

```
PASS  rig state matches rig_state/MANIFEST.json
PASS  console answers, REMOTE, no fault
PASS  config: mink solver + <=3.5 mm gate
PASS  aim point ok on a fresh marker plane
PASS  actuation: 4 deg jog moves the arm
PASS  rel mode accepts a 10 mm board nudge
PASS  arm returned to its starting pose
```

Run it at the start of every new phase. `--no-motion` skips the move checks for
a read-only survey.

## The baseline behaviours, and why each check exists

These are the fixes that made the arm usable; each has a check above so a
regression cannot pass unnoticed.

- **Position gain P=32, the servo's factory value.** LeRobot ships P=16 and
  `configure()` rewrites P/I/D on every bus open, so the bridge sets 32/0/32
  explicitly. With I=0 the loop cannot null a constant load and settles at
  `residual = load / Kp`; at P=16 the loaded elbow left 1.56 deg on the table,
  at P=32 it leaves 0.77. I stays 0 deliberately - an integrator adds windup at
  the travel limits.
- **Torque on every bridge open.** LeRobot's `disconnect()` clears the servo
  torque register. The bridge now re-enables it after `configure()`; without it
  the arm accepts 266 commands and moves 0.00° (`actuation`).
- **Lead accumulates past the deadband.** Servos ignore goals less than 2–4.4°
  ahead, so the move loop advances its *command* rather than re-deriving it from
  the measurement; otherwise nothing ever crosses the deadband and every move
  times out (`actuation`).
- **Mink solves free-roll targets; the chain solves frozen-roll ones.** With the
  roll pinned, the 5-joint arm is over-determined and the QP trades approach
  angle (36° error on a 10 mm nudge). `rel` therefore routes to the chain solver
  (`rel mode`).
- **Aim is the control quantity, not the tip.** The aim ray (jaw origin → tip →
  board plane) is what gets commanded; `TIP_PIXEL` is a mount constant used for
  diagnosis only. See `tip-center.md`.
- **The board delta is applied.** Board pose `x −27.62, y −31.63, z 432.65,
  yaw 184.32`, sign-tested (marker-vs-configured disagreement 68 → 22 px, versus
  170 px inverted). This halved the frame error to 9.0 mm p50.
- **Long moves run off the event loop.** A cold solve used to freeze `/state`
  and wedge the console for the duration.

## Known, accepted limitations

- Landing accuracy is ~9 mm (frame residual) plus the 3–4° servo deadband, so
  small corrections below ~10 mm of tip travel cannot be actuated reliably.
- `auth_required` is `false` on the live console so the browser dashboard can
  POST; the remaining guard is same-origin. Fine on a bench LAN, not on a shared
  network.
- The fixed-focus lens is calibrated near 160 mm; measurements at 60–70 mm hover
  understate true working-distance accuracy.
