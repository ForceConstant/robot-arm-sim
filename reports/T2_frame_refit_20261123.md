# T2 — FK-vs-image error field refit (2026-11-23)

Escalation step T2: re-fit the existing real captures to decide whether the
~5 mm grasp error is a rigid frame error (one constant) or something pose
dependent. No arm motion, no GPU, no service changes.

**The prediction under test:** the escalated diagnosis expected a ~22° yaw to
explain the measured `[[0.337, -0.138], [0.100, 0.712]]` Jacobian, on the
reasoning that gravity sag cannot rotate axes.

## Data

`.private/real_piece_views/*/manifest.json` — every view records the detected
marker pose *and* the measured joints (`state_joints_deg`), so both sides of the
comparison are recoverable offline:

- image side: PnP camera position in the marker frame (`camera_in_board_mm`)
- model side: FK of the measured joints through `camera_in_tool`, expressed in
  the same frame with the deployed `live/board.json` transform

65 deduped views (by `sha256`); 57 after `rms_px <= 5` and `>= 3` markers;
24 at `rms_px <= 2`. `live/board.json` (2026-11-20) predates every capture, so
the board spec did not change underneath the data.

## Result 1 — it is a translation, not a yaw

Baseline disagreement across 57 views:

```
|err| mean 18.74 mm    mean vector [-16.1, -4.0, 6.5]    sd [3.3, 2.6, 4.6]
```

A rigid fit (yaw + translation) over the 65-view set:

```
yaw +5.16 deg    t [-9.4, -13.5, 7.9] mm    residual mean 3.85 mm
```

**+5.2°, not 22°.** Removing a rotation buys very little: the error was already
nearly constant. Within a single capture set the scatter is ~1 mm
(`adapt1`, n=24: mean `[-17.1, -3.8, 7.2]`, sd `[0.8, 1.3, 1.3]`).

So the 22° axis rotation in the Jacobian is **not** a board yaw error. It has to
come from the command→joint side (IK/`base_xy` resolution), not the frame.

## Result 2 — two candidate owners, indistinguishable from these poses

A constant offset in board coordinates can be a board-origin error or a hand-eye
translation error, and they are fixed in different places. Both are linear fits:

| model | parameter | residual mean | median (robust) |
|---|---|---|---|
| A board.json origin | `[+14.2, +4.7, +6.8]` mm (base) | 4.86 mm | 1.2 mm |
| B hand-eye translation | `[+12.2, +8.1, -7.2]` mm (tool) | 4.94 mm | 1.2 mm |
| B robust (weighted) | `[+14.9, +7.9, -6.7]` mm | 4.10 mm | 1.23 mm |

Robust fit keeps 18/24 views under 6 mm; the rest are two capture sets
(`clean1`, `king_pos`) that sit ~10–12 mm away from every other set, so they
should be excluded — they are internally inconsistent with the rest, not
evidence for either model.

**Why A and B cannot be separated here:** wrist_roll was 67.25° in every single
capture. With the tool orientation barely varying, a tool-frame translation and
a board-frame translation are nearly collinear. Separating them needs captures
whose tool orientation genuinely differs (a roll sweep, or lift extremes).

## Result 3 — the z component is suspicious

Model B's tool translation is `[-6.7, ...]` mm in z, and `live/tip_frame.py`
already warns that the stock and `cad_parallel` tip constants are 110.45 vs
104.425 mm — **6 mm apart**. A 6.7 mm offset in exactly that axis is not a
coincidence worth ignoring; it is the first thing to check.

A first attempt at the discriminating test (a hand-eye shift moves the predicted
tip pixel; a board shift does not) is **inconclusive**: projecting the tip with
the model lands ~120–147 px away from the measured tip-centre pixel (320, 250),
i.e. ~17 mm at that range, and the camera convention needed to be resolved by
hand (the tip sits at `-118.8 mm` on the camera's z axis). That test depends
entirely on (320, 250) being the true tip pixel, so it is a lead, not a verdict.

## What this changes

1. The 22° Jacobian rotation is **not** a frame error. Look at the command→joint
   path (IK, roll-branch continuity, `base_xy` resolution) for that.
2. There **is** a ~17–19 mm systematic offset that the console cannot see,
   because `perr` compares the solver to itself (`live/kinematics.py:266`). This
   is bigger than the jaw's capture tolerance, so square-centre aim cannot be
   trusted until it is removed.
3. Removing it is a 3-number change (board origin *or* hand-eye translation) —
   but which one needs roll-varied captures, i.e. daylight.
4. If the offset were consistent for robot-placed pieces, a pick could still
   work because the bias cancels: place with the model, aim with the model.
   That is a hypothesis for why f4→f6 succeeded while hand-placed e5 failed; it
   is untested.

## Tools

- `tools/t2_frame_fit.py` — the error field, rigid/rigid+scale fit, per-dataset
  yaw, tip cross-check.
- `tools/t2b_offset_owner.py` — board-origin vs hand-eye fit, quality filters,
  robust weighting.
- `tools/t2c_tip_pixel_check.py` — tip-pixel discriminator (inconclusive as
  above).

Read-only: no sockets, no motion, no writes outside `.private/`.
