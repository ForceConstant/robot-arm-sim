# Tip-frame convention (settled 2026-11-18)

> Tool-side frames (`so101:gripper` → tool base → jaw bodies → fingertips) are in
> [`TOOL_FRAME.md`](TOOL_FRAME.md). This file owns the tip constant only.

Block 1 of the IRL Cartesian plan. The question was which of three numbers in
the tree is the chain→jaw-tip transform. The answer is **none of them as
written**; the MJCF says the tip is on the wrist axis at the fingertip plane.

## The command

`TIP_IN_CHAIN` in `live/tip_frame.py` is the only transform. It maps the
`so101:gripper` body frame (where the calibrated chain ends) to the jaw tip:

```
rotation:  tip +z = chain +z   (back up the fingers; approach is -z)
           tip +y = chain +x   (jaw span)
           tip +x = chain -y
translation: (0, 0, -0.104425) m
```

Every Cartesian target, every `_keep_in_check`, every reported `tip_mm`, the
sim sweep and the regression test use this one constant. `live_config.json`
cannot override it and `tool_tip_mm` is gone.

## Evidence (tools/audit_tip_frame.py)

`robot-arm-sim python tools/audit_tip_frame.py --json reports/tip_frame_stock.json`

The arm model is the stock SO-101 with the contact pads. Eight poses span pan
±40°, lift −60…−20°, elbow 20…90°, wrist flex −50…60°, roll −90…22°, and jaw
opening 10…70°:

| pose | deepest solid point (chain −z) |
|---|---:|
| home (pan 0.1°, grip 38.4°) | 104.425 mm |
| pan −40° | 104.425 mm |
| pan +40° | 104.425 mm |
| lift −20°, elbow 40° | 104.425 mm |
| lift −60°, elbow 90° | 104.425 mm |
| elbow 20°, flex −50° | 104.425 mm |
| flex 60°, grip 10° | 106.132 mm (open moving fingertip corner) |
| roll −90°, grip 70° | 104.425 mm |

- The `gripperframe` site is at `(−7.900, −0.218, −98.127) mm`; its +x is the
  approach axis. Site + `TIP_X` (8 mm) reads 106.127 mm, but the finger mesh
  only reaches 104.425 mm, so the declared `TIP_X` overshoots the plastic.
- Fixed contact pad face: **104.100 mm**, constant over every pose. At the
  reference aperture (`gap_at_zero` 15.8 mm) the moving pad is symmetric about
  the chain axis, so the pinch centre is at x = y = 0.
- Live calibration chain vs MJCF body pose: **0.000 mm / 0.000°** at all eight
  poses, so the chain frame is exactly `so101:gripper`.

The axial offset is 104.425 mm; the pad face is 0.325 mm inside it. The tip
origin stays on the chain axis because the arm cannot hold azimuth: `solve()`
pins position and approach and leaves in-plane rotation weak, and an off-axis
origin turns yaw error into millimetres of tip error.

## The three old numbers

| old value | verdict |
|---|---|
| `jaw_frame` z `−104.827` (live config) | Right depth, wrong lateral origin. Its x/y (9.462, 23.429) are the custom CAD **mount** position, not the jaw. With the free in-plane rotation that produced up to ~25 mm of tip error — the a1 24 mm failure. |
| `jaw_frame` z `−173.027` (app default) | Wrong in both models. It is the CAD mount z (−62.577) plus a hard-coded 110.45 mm in `tools/cartesian_squares.py`; the CAD pack's own TCP site is 100.45 mm from the body. It also only exists under `ROBOT_GRIPPER=cad_parallel`, which the launcher does not set. |
| `tool_tip_mm = 83` | Not an axial offset. It is a vertical reading taken with the wrist at ~39°: 104.4 × cos(39.3°) ≈ 81 mm. Used as `z + tool_tip_mm` it also ignored orientation entirely. |

The two failures were different: the lateral x/y were a frame-composition bug;
the 83 mm was a vertical-only measurement. Both are removed.

## Solver path

One path now, `ArmKinematics.solve_tip`:

- builds the tip pose from the board point and span (`xyz_target`, sign fixed:
  tip +z is up, so the jaws point down);
- converts once with `inv(TIP_IN_CHAIN)`;
- runs the fast path, then the pan-seed × quarter-turn-span escalation;
- calls `solve(..., tool_frame=TIP_IN_CHAIN)`, so the **residual and reported
  `position_error_mm` are measured at the tip**, not at the chain frame.

`Live.move`, `sanity_check`, `tools/cartesian_squares.py` and the regression
all call `solve_tip`. The old inline search in `app.py` is deleted; the old
`xyz_target(x, y, z + tool_tip_mm)` in `sanity_check` is deleted.

## Regression

`live/tests/test_tip_convention.py` — solve a1 from home, then h8 from a1's
posture (mid-sequence), and compare the solver's reported error with
`fk(joints) @ TIP_IN_CHAIN`:

```
a1: solver 3.90 mm, guard tip 3.88 mm, delta 0.025 mm
h8: solver 20.80 mm, guard tip 20.80 mm, delta 0.003 mm
1 passed
```

The gate is agreement, not reach. a1 is comfortably inside the 20 mm gate. h8
is ~330 mm horizontally from the base (base frame at board (0, −217) mm) and
lands 20.8 mm out — a reach-envelope residual, now reported identically by the
solver and the guard. That 0.8 mm overshoot is a block-2 reach/seed question,
not a frame question; do not paper over it by moving the tip again.

## Reproduce

```
robot-arm-sim python tools/audit_tip_frame.py --json reports/tip_frame_stock.json
robot-arm-sim python -m pytest -q live/tests/test_tip_convention.py -s
```

`reports/tip_frame_stock.json` holds the per-pose rows. The CAD pack
(`ROBOT_GRIPPER=cad_parallel`) audits to 100.45 mm, confirming that −173.027
was never the CAD tip either.
