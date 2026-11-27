# Why the sim wrist render loses markers — 2026-11-27

Investigating the four zero-marker squares (a8, d8, g8, h8) found while adding the
T4 marker column.

**Short version: it is not mainly the field of view. The sim's wrist camera sits
at the wrong place relative to the board — pose-dependently — and the 47° FOV was
fitted to hide that at one pose.** Changing the FOV alone makes coverage worse, not
better.

## What the scene actually carries

```
cameras          external 42.0°, top 24.0°, wrist 47.0°, wrist_landscape 32.1445°
render size      640x480 landscape  (not the portrait 480x640 the comment claims)
wrist fovy used  47.0°  -> f = 552 px
measured lens    32.14° -> f = 832 px   (calibration.json fx 831.897 / fy 833.014)
```

`wrist_landscape` already exists at the measured 32.1445°, and
`ROBOT_WRIST_CAMERA=4c` selects the calibrated mount with that FOV — so the pieces
of a correct camera are present; the default path just does not use them.

## Markers visible: rendered vs geometrically predicted

Projecting the real board through the chain and the calibrated mount, then counting
markers whose four corners land inside a 640x480 frame:

| square | predicted @ f832 | predicted @ f552 | rendered (4c) | rendered (reference) |
| --- | --- | --- | --- | --- |
| a8 | 2 | 6 | 1 | 0 |
| d8 | 1 | 5 | 0 | 0 |
| g8 | 3 | 7 | 0 | 0 |
| h8 | 5 | 9 | 0 | 0 |
| e5 | 7 | 12 | 1 | 6 |
| d4 | 8 | 15 | 2 | 5 |
| a1 | 8 | 19 | 1 | 2 |
| h1 | 10 | 23 | 0 | 1 |

Two independent problems, in this order:

1. **The render finds fewer markers than geometry says are in frame**, in *both*
   camera configurations. Even at a1 with the wide 47° view, 19 markers project
   inside and 2 are detected. Something beyond framing is eating coverage.
2. **The far row is empty under both configurations**, and its geometry is thin to
   begin with (1–5 markers at the measured lens).

## The scale check that identifies the cause

Marker side length in the render against the real camera's 95.8 px at comparable
poses:

```
4c source, plain overhead poses   57.0 – 86.2 px   (1–5 markers)
4c source, board square e5        98.3 px          (1 marker)
reference source, e5              68.5 px          (6 markers)
real camera                       95.8 px          (5–13 markers)
```

A sim marker rendering at 57–86 px against a real 95.8 px means the sim camera is
**30–45% farther from the board than the real one** at those poses — and the amount
varies by pose. That is the same pose-dependent error the wide-volume work has been
chasing all week, now visible through the render rather than through the mount
solve.

The 47° FOV explains itself under this reading: at one pose it made the rendered
marker side match 95.6 px against 95.8 px measured. A camera 1.4× too far, viewed
through a lens 1.5× too wide, produces the right apparent size and the wrong
perspective — which is exactly what a pixel column built on it would disagree with.

## What this means for T4/T6

- The marker column added to `phase2_squares.py` is honest about its own coverage
  (it reports the count), but a pixel comparison built on these renders is not
  measuring the camera — it is measuring the sim camera's pose error. The 48 → 98 px
  anomaly cannot be attributed until the render camera is right.
- With the measured mount and lens (`ROBOT_WRIST_CAMERA=4c`), the scale is correct
  but coverage collapses (1–2 markers), because the FOV stops hiding the distance
  error. Neither configuration is usable as-is for a pixel axis.

## Recommended order, smallest first

1. **Verify the sim camera pose against `camera_in_tool`.** The mount is mapped
   into the gripper body through a reference frame; if that mapping is off, the
   camera sits at the wrong offset in a pose-dependent way. One pose where the real
   camera's marker pattern is recorded, compared against the render's, is enough to
   size the offset. No new calibration, no motion beyond one known pose.
2. **Then set the FOV to the measured lens** (32.1445°, already in the scene as
   `wrist_landscape`) and re-check coverage. Doing this first makes coverage worse,
   as measured above.
3. **Then re-run T4 and rebuild the pixel axis** on the squares that carry enough
   markers, and re-examine the 48 → 98 px column.

Nothing was changed here: no scene edit, no FOV change, no `camera_in_tool` edit.
Probes ran on the BC-250; the numbers above are from
`reports/phase2_squares_20261127.json` plus the two probe scripts.

## Direct readout: the sim camera vs the calibrated mount (no markers, no fitting)

In the simulator the camera is a rigid body, so its pose can be read straight from
MuJoCo (`data.cam_xpos`, `data.cam_xmat`) and compared with
`chain.fk(q) @ camera_in_tool` at the same joint vector. Five very different poses:

```
                     flange delta      camera delta
 pose                dT mm    dR deg    dT mm   dR deg
 [0,-50,45,70,0]      61.9   178.03      16.3    23.40
 [-20,-60,40,80,0]    62.1   178.03      17.6    23.40
 [20,-40,30,60,0]     60.5   178.03      14.3    23.40
 [0,-70,60,90,0]      63.1   178.03      16.5    23.40
 [-12,-13,5,95,0]     64.0   178.03      17.4    23.40
```

Two constants, and the pair settles it:

- **Flange: 178.03° and ~62 mm, identical at every pose.** That is a frame
  convention — the MJCF `so101:wrist` body frame versus the calibration chain's
  flange frame — not an error.
- **Camera: 23.40° and 14–18 mm, also constant.** Once the flange convention is
  factored out, the sim camera and the calibrated mount differ by a **fixed**
  transform.

A constant transform means this is **not** a physics, pose or FOV problem at all.
`chess_sim/scene.py` carries `WRIST_CAMERA_MEASURED` as hand-transcribed numbers
(`pos`, `xyaxes`, `fovy`) rather than building the camera from
`calibration.camera_in_tool`; those numbers disagree with the calibration by
~23° and ~15 mm. The `reference` source (47°) is a second, further reconstruction,
which is why its marker sizes (68 px) and the 4c ones (98 px) bracket the real
95.8 px.

## Answering the original question

Yes — on the real arm, several poses plus marker PnP give the camera-versus-arm
relation, and that is exactly how `handeye-2026-11-18T11-block4c` was made (per-view
PnP 0.70 px). In the simulator the same relation needs no images at all: it is read
from the model, and it disagrees with the calibration by a constant 23.4° / 15 mm.

**Therefore the fix is one line of construction, not a calibration:** build the
sim wrist camera from `camera_in_tool` and the measured fovy (32.1445°), keeping
`scene.py`'s hand-written numbers only as a documented fallback. After that the
marker size and coverage should be re-measured before any pixel axis is rebuilt —
the earlier geometric projection suggests the far row may still be thin even with
the correct mount, which would be a separate question about the sim board's extent.

## Fix applied: the sim camera now derives from the IRL calibration (41aa8ed → this commit)

Brian: the sim must take its offset **from** the IRL calibration, never the other
way round. `chess_sim/scene.py` now reads `camera_in_tool` and the measured
intrinsics at build time (`measured_mount_from_calibration()`) and uses that mount
for `ROBOT_WRIST_CAMERA=4c`; the hand-copied `WRIST_CAMERA_MEASURED` stays only as a
documented fallback when the file cannot be read. Nothing writes back to the
calibration.

Convention found by measurement, not by fitting: MuJoCo `xyaxes` are the
**columns** of `camera_in_tool`'s rotation. Rows give a constant 17.64° error;
columns give **0.00°** at every pose.

Verification after the change, five poses:

```
camera delta vs camera_in_tool   5.0 mm / 0.00 deg  at every pose   (was 23.40 deg)
marker side in render            95.5 – 100.7 px    (real camera 95.8 px; was 57–86)
```

So the lens and pose now agree with the bench camera. The residual is a constant
**5 mm lateral** offset (4.9 mm along camera +x), reported rather than absorbed.

## What is still wrong, and it is not the camera

Coverage did **not** improve: still 0–2 markers on the sampled squares, against
5–13 on the real camera at the same scale and the same field of view. With lens,
scale and pose now matching, a different marker footprint means the **sim board
face does not cover the same area as the printed mat** — a board-content question,
independent of the camera. Candidates to check next: the rendered ChArUco face's
extent against the 8×8 × 28 mm printed mat, and `AppearanceConfig(board_markers=False)`.

Suite after the change: 298 collected, 274 passed, 22 skipped, 2 failed — the two
known pre-existing failures, nothing new.

## Jaw-vs-camera check (Brian: rotate the jaw, not the camera)

Measured in the model at one pose:

```
camera vs camera_in_tool          0.00 deg (unchanged, every pose)
jaw span vs gripper-site +x       90.0 deg
jaw span vs gripper-site +y       70.0 deg
jaw span vs gripper-site +z       20.0 deg     <- +z is the documented jaw-span axis
jaw bodies midpoint, site frame   (-56.0, 0.0, 0.0) mm
jaw span in the camera frame      (-0.728, 0.497, 0.473) -> 43.3 deg off image x
```

So the CAD jaws sit **20° off the axis their own documentation names as the jaw
span**, and the span lands diagonally in the camera frame. That is a jaw-versus-
camera error of the kind Brian described, but the magnitude in this measurement is
20°, not 90°.

Consequence for where the fix belongs: **not** the camera. The camera now matches
the bench calibration exactly and must stay that way. The correction belongs in the
tool model — a yaw about the approach axis applied to the CAD tool mount
(`tool_frames.MOUNT_QUAT`, or the jaw bodies' placement), set from the real images
rather than from the CAD.

What is missing to set it: the *sign and size* of the jaw yaw that the real images
require. I cannot read that off the render myself — review images are delivery only
in this setup — so the panel (`reports/sim_roll_options_20261127.jpg`) is the
measurement channel: whichever rotation of the render matches the real fingertips
gives the number, and it then goes into the tool mount as a constant, with the
camera untouched.
