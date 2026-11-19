# Tool frame chain (settled 2026-11-19)

Block 1 of the 2026-11-19 plan. This is the **single** written-down description of
how the private CAD parallel tool is placed on `so101:gripper`, and what the built
model must measure. The tip constant itself lives in
[`TIP_FRAME_CONVENTION.md`](TIP_FRAME_CONVENTION.md); this file does not restate it.

The numbers here are mirrored in code as `chess_sim/tool_frames.py`, which
`tests/test_tool_orientation.py` enforces against the compiled model. If the pack
changes, both the table and that module change together.

## The chain

```
so101:gripper                body, parent = so101:wrist          <- the calibrated chain frame
  |  mount                   pos (9.462, 23.429, -62.577) mm
  v                          quat (0.707107, 0, 0, -0.707107)  [Rz(-90 deg)]
so101:cad:parallel_base      body, the TOOL BASE                  <- jaws hang off this
  |  base_geom               pos (23.4292, -9.46249, 75.8269) mm
  v                          quat (0, 0.707107, 0, -0.707107)
base part frame              the frame Complete_Fixed_Jaw.stl is authored in
  |  jaw_body + jaw_geom
  v
jaw part frame               the frame gripper_jaw_1__solid1.stl is authored in
```

| # | child | parent | transform (pos mm, quat wxyz) | mesh authored in |
|---|---|---|---|---|
| 1 | `cad:parallel_base` | `so101:gripper` | (9.462, 23.429, -62.577), (0.707107, 0, 0, -0.707107) | — (holds the base visual) |
| 2 | base visual geom | `cad:parallel_base` | (23.4292, -9.46249, 75.8269), (0, 0.707107, 0, -0.707107) | base part frame |
| 3 | `cad:parallel_jaw_left` | `cad:parallel_base` | (4.879, -41.462, 8.127), (0.5, -0.5, 0.5, 0.5) | — (holds the jaw visual) |
| 4 | `cad:parallel_jaw_right` | `cad:parallel_base` | (41.979, 22.537, 8.127), (0.5, 0.5, 0.5, -0.5) | — |
| 5 | jaw visual geom (both) | its jaw body | (22.0, -18.7, 19.0), (0, -0.707107, 0, 0.707107) | jaw part frame |
| 6 | `cad:parallel_pinion` | `cad:parallel_base` | (23.4292, -9.46249, 17.3269), (0.999507, 0, 0, 0.031411) | — |

Links 1 and 2 are what `chess_sim/cad_tool.py` installs (`profile['mount_position']`
/ `mount_rotation`, then the pack's own `model.xml` geoms). Links 3–6 are the pack's
`model.xml` verbatim; nothing in the sim re-expresses them.

**The jaw bodies are already in the tool-base frame.** Their `pos`/`quat` are
children of `cad:parallel_base`, so they compose once, against the base, and the
base composes once against the wrist. There is no second placement to reconcile.

## Collapsed: gripper ← base part frame

Links 1 and 2 compose to

```
g_x = -p_y        g_y = p_z        g_z = 13.25 - p_x
```

i.e. `R = [[0,-1,0],[0,0,1],[-1,0,0]]`, `t = (0, 0, 13.25) mm`, det +1.
The base part origin therefore sits 13.25 mm below the gripper body origin.

That is why the base's own numbers read the way they do:

| base part frame | gripper frame |
|---|---|
| x 13.25 … 69.20 | z −55.95 … 0 |
| y ±32.00 | x ∓32.00 |
| z −35.25 … 23.45 | y −35.25 … 23.45 |

So the base top face lands on `y = 23.45 mm` in the gripper frame, against the
mount plane at `y = 23.429 mm` (0.02 mm). The tool hangs on one side of the wrist,
which is why "identity on `so101:gripper`" is 90°/60 mm wrong.

## Jaw placement, in the tool-base frame

`pos`/`quat` are links 3–5 above. What they produce, measured in `so101:gripper`
at zero jaw slide:

| quantity | value (mm) |
|---|---|
| left jaw bbox | x −16.045 … 35.000, y −30.746 … 30.446, z −110.450 … −45.950 |
| right jaw bbox | x −35.000 … 16.045, y −30.446 … 30.746, z −110.450 … −45.950 |
| deepest point, left / right | (0.00, −8.15, −110.45) / (0.00, 8.15, −110.45) |
| tip-face centroid, left / right | (−1.25, −0.15, −110.45) / (1.25, 0.15, −110.45) |
| tip gap at zero slide | 2.52 |
| left mirrored through (x,y) → right | max nearest-vertex error **0.000** |

The two jaws are one part turned 180° about the approach axis — a half-turn, not
a mirror plane — so the pair is 2-fold symmetric and the tips converge on the axis.
Jaw slide is along gripper **x** (each body's joint `0 0 1` maps to ±x), 0 … −19 mm
per jaw, i.e. a 0 … 38 mm aperture.

Each jaw body sits at `y = ∓32` in the base part frame — exactly the base's ±32
faces, where the sliders are. Rotating the pair 90° about the approach axis would
put a jaw at part `z = +32`, outside the base (which ends at 23.45). **The pose
above is the only orientation that fits the base's rails**, and
`test_jaws_stay_under_the_base_footprint` is the check that says so.

## The assembly-root occurrences do not compose

The Onshape assembly occurrences are

| part | pos (mm) | quat |
|---|---|---|
| `Gripper_Base:1__Solid1` | (9.462, 23.429, −62.577) | (0.707107, 0, 0, 0.707107) |
| `Gripper_Jaw:1__Solid1 <1>` | (23.279, 13.914, −13.873) | (0.707107, 0, 0, −0.707107) |
| `Gripper_Jaw:1__Solid1 <2>` | (22.979, −32.839, −13.873) | (0.707107, 0, 0, 0.707107) |

All four plausible readings of "jaw = E ∘ inv(base occurrence) ∘ jaw occurrence"
(base and jaw rotation each taken as-is or transposed) were evaluated against the
built model. Every one lands the jaw **off the tool**: the jaw mesh ends up spanning
y 14.7 … 79.2 mm, or z 19.7 … 70.8 mm — nowhere near the −110.45 mm tip. There is
no constant assembly-root ↔ tool-base offset that rescues them, because the base
occurrence *is not* the profile mount: its rotation is the transpose of
`mount_rotation` (Rz(+90) vs Rz(−90)) while the translation is identical. The
occurrence pair describes the untrimmed assembly part frame; the merged part that
ships in the pack is authored 13.25 mm and 90° away from it.

**Do not place jaws from the occurrences.** This was the 5y scatter.

## CAD cross-checks

Measured from the compiled model (`tools/b1_chain.py`), against the values the
2026-11-19 plan requires:

| check | target | built | delta | verdict |
|---|---|---|---|---|
| base x range (part frame) | 13.25 … 69.20 | 13.25 … 69.20 | 0.00 | pass |
| base z min (part frame) | −35.25 | −35.25 | 0.00 | pass |
| fingertip depth (gripper −z) | 110.450 | 110.450 | **0.00** | pass |
| gear origin in gripper frame | (0, 0, 12.750) | (0.000, 0.000, 12.750) | 0.00 | pass |
| jaw body centres | (−0.300, ∓27.828, −76.600) | see below | — | **FINDING** |

### The finding — jaw body centres

The 2026-11-19 plan withdrew the "finger span ±27.83" framing (it was never a
span across the roots) and asked instead for the pre-5y **jaw body-centre**
placement, `(−0.300, ∓27.828, −76.600)` in the gripper frame. The built model
does not reproduce it under any definition:

| definition | left (mm) | right (mm) |
|---|---|---|
| body origin | (−32.000, 18.550, −54.450) | (32.000, −18.550, −54.450) |
| vertex centroid | (−5.786, 5.268, −56.579) | (5.786, −5.269, −56.579) |
| mesh bbox mid | (9.477, −0.150, −78.200) | (−9.478, 0.150, −78.200) |
| quoted CAD centre | (−0.300, −27.828, −76.600) | (−0.300, +27.828, −76.600) |

The quoted pair is mirrored about **x only**, i.e. it spans gripper **y**; the
built pair is a half-turn pair (mirrored in x and y) spanning **x**, and the jaw
bodies sit on the base's ±32 rails at part y ∓32. A 90° yaw cannot reconcile
them either — it throws one jaw to part z +32, outside the base. So this is the
same frame question the span check was, not a placement defect: the built pose
is the one that fits the base, and the quoted centres belong to a pre-5y
arrangement that no longer exists in this pack.

**No placement change was made.** The test pins the built centres so they cannot
drift while the question is open.

## Regression

```
ROBOT_GRIPPER=cad_parallel \
ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \
PYTHONPATH=. ~/.local/bin/robot-arm-sim python -m pytest tests/test_tool_orientation.py -q
```

Twelve checks over the compiled model: mount pose, base-part composition, base
extents in its own frame, jaw bodies in the tool-base frame, tip depth, fingers
along the approach axis, jaws under the base footprint, jaw mirror symmetry, tip
centring, gear origin, base on the mount plane, and no camera-housing proxy.

Measurement receipts: `tools/b1_chain.py` (every number above),
`tools/b1_occ.py` (the occurrence sweep), `tools/b1_sweep.py` (yaw and aperture
scores).

`test_pack_is_not_silent_stock` covers the trap Block 2 found: `gripper.py`
keys off `ROBOT_GRIPPER` alone, and `scripts/start-live.sh` exports neither
`ROBOT_GRIPPER` nor `ROBOT_GRIPPER_ASSETS` (the `gripper`/`gripper_assets` keys
in `live/live_config.json` are read by nothing), so a launch from that script
compiles the **stock** tool. A receipt produced that way is ambiguous; the test
fails instead of letting one be written.

`tools/b1_mutate_check.py` proves the checks bite: an identity placement is caught
by 6 of them, a 90° yaw of the pair by 2, a single-jaw 90° error by 4.

## Renders

`tools/b1_sheet.py` (the visual-review gate), at the `marker14` pose from
`reports/block5m_frames.json`:

- `reports/b1_orientation_sheet.png` — the contact sheet: front / side / top
  full arm, close-up with dimension callouts, wrist camera, IRL frame, plus the
  provenance strip and the three checks
- `reports/b1_orientation_receipt.json` — the same provenance, machine-readable

`tools/b1_render.py` renders the older plain set:

- `reports/b1_orientation_ortho.png` — front / side / top / iso + wrist
- `reports/b1_orientation_wrist.png` — 4c wrist camera
- `reports/b1_orientation_marker14.png` — wide view
- `reports/b1_orientation_vs_irl.png` — sim wrist beside `live/captures/scan_5m_marker14.jpg`

The wrist tile is usable and matches the IRL frame in structure: the board fills
the left two thirds and the right jaw cuts in from the right edge, which is what
`scan_5m_marker14.jpg` shows. The 4c mount still sits at
`(67.66, −2.92, −10.88)` mm in the gripper and Block 3 re-solves it; this block
does not touch it, and the tool was not adjusted to make the wrist view work.

## Frozen

`TIP_IN_CHAIN` (104.425 mm), the 20 mm gate, the scan plan and every calibration
file are untouched by this block.
