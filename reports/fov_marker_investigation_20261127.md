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

## Root cause found: the CAD tool is pitched 90° about the jaw-span axis

Brian: "if you have the jaws open like IRL you should see the 2 fingers, not the one
side." Measured in the model, at one pose:

```
chain flange vs so101:gripper          dT  5.0 mm   dR   0.00 deg   <- camera_in_tool's frame
chain flange vs so101:wrist            dT 61.9 mm   dR 178.03 deg
chain flange vs so101:cad:parallel_base dT 70.3 mm  dR  90.00 deg   <- the CAD tool
CAD base relative to the flange: 90.00 deg about flange -z, translation (14.0, 23.6, -64.7) mm
```

And where the jaws actually sit in the calibrated camera frame:

```
jaw left    camera mm ( 64.9, -27.7, -95.3)   depth 95.3 mm
jaw right   camera mm (-14.1,  26.2, -43.9)   depth 43.9 mm
```

The two fingers are **51 mm apart in depth** and on opposite sides laterally. One is
in front of the other, which is exactly why the render shows one finger side-on
instead of two jaws side by side. That is not an image roll — rotating the render
(0/90/180) cannot fix it, which is why none of the four options looked close.

The cause is a **90.00° pitch of the CAD tool mount about the jaw-span axis**
relative to the calibrated flange. The camera is correct (0.00° against
`camera_in_tool`, and that mount lives in the gripper body frame, which the flap
above confirms is the calibration's frame to 5 mm / 0.00°). **The tool model is
wrong, exactly as Brian described.**

### Where the fix goes, and its blast radius

The correction is in the CAD mount — `tool_frames.MOUNT_QUAT`
(`(0.707107, 0.0, 0.0, -0.707107)`, itself a 90° rotation) or the jaw placement in
`cad_tool.install`. A −90° correction about the jaw-span axis should put the jaws
side by side at equal depth.

This is not a safe one-liner to slip in. Both are pinned:

- `tests/test_frame_pins.py` pins `MOUNT_POS_MM`, `MOUNT_QUAT`, `TIP_M` and
  `TIP_DEPTH_MM` (110.450 mm measured over every jaw vertex).
- Rotating the tool by 90° about the jaw-span axis will move the deepest jaw vertex,
  so `TIP_DEPTH_MM` and every grasp/pinch test that leans on it will change.
- `chess_sim/tool_frames.py` documents the chain with "what the chain has to
  produce" bboxes, so the rotation has to be reconciled with that description, not
  simply overwritten.

Recommended: apply the −90°, re-measure the jaw depths (expect them equal) and the
new tip depth, then re-pin deliberately with `tools/repin_rig_state.py` semantics
and run the suite to see which grasp tests move. That needs Brian's go-ahead because
it changes a pinned physical constant, not because it is technically uncertain.

## Orientation and scale after the revert: it is the render configuration

Brian: in the side-by-side, IRL has the fingers on the right and the sim has them at
the bottom, and the aspect/orientation look changed. Measured, same pose:

```
IRL capture shape            640x480 (raw bridge frame)
env.render("wrist") shape    640x480
marker side, Renderer 640x480     46.60 px
marker side, Renderer 480x640     57.70 px   (portrait)
marker side, env.render()        67-80 px   (control.image_size)
real camera                      95.80 px
board-origin depth   IRL 233 / 289 / 240 mm
                     SIM 300 / 343 / 341 mm   (65-100 mm farther)
```

So three different render configurations give three different marker scales, and
none of them lands on 95.8 px. The 47° reference FOV was fitted at *one* specific
size and orientation (`reports/sim_vs_irl_fovfit.json`); away from that exact
configuration the geometry drifts, and the rendered image sits at a different
orientation from the raw sensor frame — the sensor is mounted 90° to the model
(`scene.py`, "Portrait 480x640 is the render orientation that matches the sensor").

That is what Brian is seeing: a portrait-intent camera drawn into a landscape frame.
Nothing about it changed in the revert — the revert restored the arrangement that
was already there; the mismatch predates today's work and was masked because the
comparisons were only ever qualitative.

Consequence: no pixel column, marker count or image comparison is meaningful until
the render configuration (size + orientation) is pinned to the one the FOV was
fitted at. That is a different question from the mount, which the calibration
states correctly, and from the CAD tool, which is 90° pitched.

## The comparison camera already existed (tip-center.md §3) — I was using the wrong one

Brian pointed at `tip-center.md`. That document, confirmed by eye on 2026-11-24 and
marked "do not change without asking", states the conventions:

1. **Compare in landscape** — IRL frames as captured, 640×480, no rotation.
2. **The sim comparison camera is rolled −90°** about its optical axis, because the
   real C270 is mounted 90° to the model (i.e. the sensor is portrait, 480×640).
3. **Apply `sim_wrist_roll()`** — sim roll = live roll + **−87.21°**.
4. **Render at the measured lens**, fovy 32.144°; `WRIST_REFERENCE_FOVY_DEG = 47.0`
   is a scale fudge and "must not be used when comparing pixels".

`chess_sim/scene.py` already implements (2) as the **`wrist_landscape`** camera —
reference mount rolled −90°, `tool_reference.fovy_deg()`, added explicitly "for
sim-vs-IRL pixel comparison". And `tools/full_parity_sample_images.py` already uses
it by default.

Every comparison I produced today rendered **`wrist`** — the 47° perception camera
without the roll. That is the whole explanation for "fingers at the bottom instead
of the right" and for the 25–35% scale error. Not a new bug, not the revert: I used
the wrong camera.

Measured with the documented camera and `sim_wrist_roll` applied:

```
pose   IRL markers / side      SIM markers / side
 a      9 / 68.58 px            7 / 76.39 px     (+11 % size)
 f     10 / 55.92 px            8 / 59.63 px     (+7 %)
 h     12 / 60.21 px            3 / 73.01 px     (+21 %, coverage poor)
```

Within-pose marker size now agrees to 7–21 % instead of 25–35 %, and counts are
close at a and f. Pose h still loses most markers — the remaining coverage question,
unchanged by any camera choice.

Panel: `reports/sim_vs_irl_landscape_20261127.jpg` (IRL above SIM, three poses).

## Pose h explained: the parked pieces occlude the mat

Segmentation at pose h (`wrist_landscape`, `sim_wrist_roll` applied), sampling the
centre of every marker whose corners project inside the 640×480 frame:

```
17 markers project inside the frame: 6,7,8,11,12,14,15,16,19,20,22,23,24,27,28,31,32
0 of them show background at their centre
all 17 are covered by body id 2 or 3
body 2 = white_pawn1, body 3 = white_pawn2
```

The pieces absent from the position are parked in a grid 60 mm off the board's right
edge (`BoardConfig.graveyard_gap = 0.06`) and measured at x = 0.18–0.29 m against a
board half-width of ~0.11 m. At pose h that parked strip sits **between the lens and
the mat**, so most markers are physically behind a pawn.

The bench captures were taken on a clear mat, so the comparison was never
like-for-like. This is the whole of "pose h loses markers", and likely most of the
earlier far-row losses too — not the camera, not the board texture.

### An attempt that did not work, recorded rather than hidden

Re-running with `graveyard_gap=0.55` (pieces parked 0.55 m away) returned **2 markers
at every pose with an identical 66.8 px side**, which is implausible for three
different joint vectors and contradicts the earlier 7/8/3 result on the same camera.
I do not understand that result and am not presenting it as evidence. The next step
is a fair comparison scene with the pieces genuinely gone (hidden geoms, not merely
parked), verified by a segmentation check that the mat is unobstructed before any
frame is used for a pixel comparison.

## Corrections from Brian's two notes (2026-11-27, 14:50)

**Note 1 — there are no pieces on the board in either view. Correct, and my
occlusion claim was wrong.** Retracted.

Two errors in that probe:

- MuJoCo segmentation returns **geom** ids, not body ids. I read id 2/3 and looked up
  *body* 2/3, which happened to be `white_pawn1`/`white_pawn2`. Looking up the geoms:
  geom 2 is `table`, geom 3 is `board`. Every sampled marker centre was landing on
  the mat, exactly as it should.
- The projection also carried a y-sign slip (`v = fy*(+y)/depth` instead of the
  project convention `v = fy*(-y)/depth`). With the sign corrected, the same 21
  markers still resolve to table/board — i.e. **nothing is occluded**.

So "parked pieces between lens and mat" is false, and the accompanying commit
(`d7180db`) and its figure should be read as retracted.

**Note 2 — the IRL looks rotated ~15° CCW by eye. Measured: it is the mat heading,
not a camera roll.**

```
sim BOARD_TURN            +180.00 deg
session live/board.json   -165.50 deg
difference                 +14.50 deg
```

The mat was physically rotated this session (the earlier scan measured +21.1° from
the stored value, then it was nudged back). The simulator keeps a fixed
`BOARD_TURN`, so the IRL board appears rotated against the sim board by exactly that
difference. A fair comparison needs the sim board yaw set to the session's
registered yaw; it is not a lens or mount property.

**What is still unexplained, and where it now points:** at pose h, 21 markers project
inside the frame with nothing in front of them, yet the render detects 3 — and those
3 are all in one row (ids 30/31/32 at board y = +70 mm). At pose a the 7 detected ids
are likewise a subset. That pattern points at the **sim board's marker texture
covering only part of the mat**, while the layout model assumes the full 5–36. That
is the next thing to check, and it is an asset/texture question, not a camera one.

## Pose-level comparison: the sim camera is ~100 mm too close, and 10–15° off

Counting markers kept giving different answers, so this compares the *board pose*
from PnP on each image at the same joint vector (sim rendered through
`wrist_landscape` with `sim_wrist_roll` applied, IRL at the recorded joints):

```
pose   board orientation  sim vs irl     board depth  IRL        SIM
 h            15.62 deg                    240.3 mm      163.6 mm
 a             9.60 deg                    233.3 mm      143.7 mm
 f             9.39 deg                    288.9 mm      184.0 mm
```

Two numbers, both real:

1. **The ~15° Brian sees is there** — 9.4–15.6° of board orientation between the two
   images at the same joints.
2. **The bigger error is depth: the sim camera sits 90–105 mm closer to the board.**
   At a fixed FOV that shrinks the visible board by roughly 40 %, which is the
   marker-coverage gap far better than any of the explanations I tried before.

Controls run:

- `sim_wrist_roll` is doing its job: without it the orientation error is **93–97°**,
  with it 9–16°. So the −87.21° offset is correct in kind and leaves a residual.
- Matching the mat heading (session yaw instead of `BOARD_TURN`) changed counts by
  one or two markers, not by the gap: pose h 3→4, a 7→5, f 8→8. So heading is
  not the coverage cause either.
- The board texture draws a marker in every light square (`FIRST_ID` upward, 32 of
  64 squares), so the "texture covers only part of the mat" theory is dead too.

What this points at: the tool frame the camera hangs from. The CAD tool is measured
at a 90.00° pitch about the jaw-span axis, and a 90° error there is exactly the kind
of thing that leaves a 10–15° residual after a −87.21° joint offset and shifts the
lens ~100 mm along the approach axis. Fixing the tool mount and then re-deriving
`sim_wrist_roll` and the mount is the ordered path; nudging the camera is not.
