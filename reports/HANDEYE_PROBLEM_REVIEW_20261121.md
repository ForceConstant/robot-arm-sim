# Hand-eye problem review — everything measured so far

**Status: paused for review.** No transform is published. The rig is idle in HOLD.
Last updated 2026-11-21 ~21:00 (BC-250 clock).

---

## 1. The problem

The wrist camera's pose is derived two ways and they disagree:

- **PnP**: marker corners in the image → camera pose relative to the board.
- **FK**: measured joints → forward kinematics → camera pose via the stored
  camera-in-tool mount.

The two disagree by **12.5–34 mm** depending on the view, and the disagreement
moves with the view (not just with the wrist roll). The practical symptom: the
detector names a marker 1–2 squares off, so the arm aims at the wrong square.

This is the named defect left over from Step 2 and it blocks IRL piece pickup.

## 2. The goal

Re-derive the **camera-to-wrist transform** (the camera-in-tool mount `X`,
including the wrist-roll zero) from marker observations, branch-aware, and
publish it only if it collapses the PnP-vs-FK disagreement and validates on
held-out views.

Acceptance (from `docs/CHARUCO_HANDEYE.md`): reprojection RMS ≈ 1 px and a board
pose within a couple of millimetres of where the board physically sits.

## 3. Setup

- **Arm**: SO-101 follower, Pi 4 bridge at `192.168.1.167:8100`, live console at
  `192.168.1.171:8080` (BC-250), camera on the wrist, `cad_parallel` gripper
  (`.private/gripper-assets-5y`).
- **Board**: 244 mm printed ChArUco face, 28 mm squares, 20 mm markers, ids 5–36
  on the white squares. Measured placement `live/board.json`:
  `x −34.92, y −18.13, z 431.35 mm, yaw 181.92°`.
- **Lens**: `WristCalibration` from `.private/phase11d/calibration.json`:
  fx 831.9, fy 833.0, cx 320, cy 240 (fovy 32.1°), distortion
  k1 −0.1023, k2 0.6857.
  **Cross-check:** measured marker edges are 84–108 px for 20 mm markers at
  ~160 mm → f ≈ 832. The calibration is consistent with the images.
- **Tool tip**: gripper origin → tip 110.45 mm (`live/tool_tip.json`).
- **Current plan**: `chess_sim/assets/marker_scan_plan_charuco.json` (updated —
  see §8).

## 4. Verified facts

### 4.1 Already settled in Step 2 (`reports/step2_verdict_20261121.md`)

- **Half turn ruled out**: 12–30 mm measured against 133–193 mm predictions.
- **Print/board ruled out**: three markers agree to 2.4–5.8 px = 0.45–1.08 mm at
  5.38 px/mm; a print offset of ~1 mm cannot produce a 33–51 mm naming error.
- **Blur/lens ruled out**: b2 decoded 3 markers with per-marker IPPE 0.04–0.58 px
  at 14–18 px/module.
- **Roll-zero hypothesis dead**: azimuth difference PnP−FK was +1.80° to +22.60°
  (mean +13.73, std 9.26) and, at the *same* roll 69.01, b2 gave +22.50° while e5
  gave +1.80°. The error tracks board position, not roll alone.
- **Single constant offset dead**: best offset (14.4, 9.2, −7.3) mm took the mean
  disagreement 22.4 → 13.1 mm (only ~40 %), leaving a view-dependent 6.4–19.6 mm.
- **Marker count per view is the measured bottleneck**: PnP radius varied
  62.3–85.4 mm while FK held 68.8–71.2 mm.

### 4.2 Arm reach limits the standoff (measured 2026-11-21)

Plan candidates above the board at 0.26 / 0.32 m: the solver reached 0.196–0.229 m
camera height but stayed **55–112 mm short** horizontally. At 0.18–0.20 m the
shortfall was 7–43 mm. The current plan's camera height is
`0.5937 − 0.4337 = 0.160 m` above the board.

**Conclusion: a larger standoff is not available.** The wrist camera cannot be
backed off; the sim's planned standoff is already at the practical limit.

### 4.3 Jaw is not a detection lever (measured)

One pose, jaw commanded 0 / 25 / 50 / 75 / 100 (Pi bridge, speed 0.3):
**the same two markers decoded at every setting** ([14, 19]). The fingers do not
occlude the decodes. Jaw authority is now standing, but it buys no markers.

### 4.4 Camera tone is not the limiter; the settled *exposure* was

Tone sweep at one pose (exposure 705, gain 64), markers decoded:

| brightness/contrast | mean | clip % | markers |
|---|---|---|---|
| 222/32 | 149.4 | 2.03 | 4 |
| 192/32 | 121.0 | 1.39 | 4 |
| 160/32 | 89.4 | 0.10 | 4 |
| 128/32 | 57.9 | 0.00 | 4 |
| 96/32 | 37.5 | 0.00 (dark 28 %) | 4 |
| 128/64 | 45.8 | 1.43 | 3 |
| 160/64 | 69.2 | 5.15 | 4 |
| 192/64 | 105.0 | 11.00 | 4 |
| 222/64 | 151.4 | 17.45 | 4 |

Brightness is not the limiter. The **first capture pass settled exposure to
2600/96 at the board edge** and reused it at centre-board poses, where it blew
the frame (clip 9–14 %). Re-running the same poses at 705/64, brightness 160,
contrast 32 **doubled the marker sightings** (§5).

*Caveat:* the jaw test and the tone sweep reported different frame means at the
same nominal controls (≈200 vs ≈149). The cause was not isolated (camera reopen /
an auto control). Absolute brightness comparisons across sessions are not exact.

### 4.5 PnP-vs-FK camera disagreement, base frame (authoritative)

Computed directly in the arm-base frame from the pass-1 captures (stored mount):

| view | markers | disagreement |
|---|---|---|
| marker14 | 1 | 24.6 mm |
| marker9 | 2 | 27.2 mm |
| marker22 | 2 | 24.8 mm |
| marker10 | 1 | 34.1 mm |
| marker16 | 2 | 20.7 mm |
| marker8 | 2 | 23.9 mm |
| marker21 | 1 | 30.0 mm |
| marker13 | 1 | 30.5 mm |

mean **26.97 mm**, max **34.07 mm**; azimuth difference PnP−FK mean **+17.93°**,
std 5.51°.

> **Code caveat:** the evaluation script also computes this in the *board* frame
> and reports 312–328 mm. That variant is inconsistent with the base-frame
> computation and is not trustworthy — a frame-convention bug in that one
> comparison. Use the base-frame numbers.

## 5. Capture passes (the data)

Three passes over the plan poses, all 13/13 or 8/8 reached, roll held within 0.5°.

### Pass 1 — roll 21.7, exposure 2600/96, jaw held 17.97

| pose | markers decoded |
|---|---|
| marker14 | 17 |
| marker26 | — |
| marker25 | — |
| marker9 | 18, 22 |
| marker22 | 5, 9 |
| marker10 | 29 |
| marker16 | 11, 6 |
| marker8 | 23, 19 |
| marker24 | — |
| marker15 | — |
| marker23 | — |
| marker21 | 6 |
| marker13 | 14 |

**12 sightings, 8 views, 5 empty, 0 views with ≥3.**

### Pass 2 — roll 21.7, exposure 705/64, brightness 160, contrast 32, plan jaw (0)

| pose | markers decoded |
|---|---|
| marker14 | 17, 13 |
| marker26 | 9 |
| marker25 | 10, 9 |
| marker9 | 18, 22, 21, 25 |
| marker22 | 5, 13, 9 |
| marker10 | 29 |
| marker16 | 19, 11, 6 |
| marker8 | 23, 27, 19 |
| marker24 | — |
| marker15 | — |
| marker23 | — |
| marker21 | 6, 14 |
| marker13 | 14 |

**22 sightings, 10 views, 3 empty, 4 views with ≥3.** Per-view clip 0.0–14.0 %
(the tone is still not uniform across poses).

### Pass 3 — roll 68.0 (same poses +46.3°), same tone

| pose | markers decoded |
|---|---|
| marker14 | 17, 9 |
| marker26 | — |
| marker9 | 18, 13, 17 |
| marker22 | 9 |
| marker16 | 11, 14 |
| marker8 | 19, 11, 22 |
| marker21 | 6, 10 |
| marker13 | 18, 14, 9, 6, 5 |

**18 sightings, 7 views with markers (8 captured), 1 empty.**
Roll reached 67.60–68.04 and held — a genuine second branch.

**Combined: 40 sightings, 17 PnP views, rolls 21.7 and 68.0.**

## 6. Solve results (staged + bootstrap, measured lens)

| set | views / sightings | RMS | mount change vs stored | board solved |
|---|---|---|---|---|
| pass 1 only | 7 / 11 | 12.50 px | 16.40 mm, 158.6° | (−16.0, −10.7, 420.6) yaw −178.1 |
| pass 2 only | 9 / 21 | 12.07 px | 23.24 mm, 160.3° | (−21.6, −8.1, 417.3) yaw −178.1 |
| **combined (both rolls)** | **14 / 35** | **16.43 px** | **21.40 mm, 164.2°** | (−29.2, −27.2, 418.6) yaw −175.2 |

Stored mount translation: `[67.7, −2.9, −10.9] mm`.
Measured board: `(−34.9, −18.1, 431.4) mm, yaw 181.9°`.

Cross-view board-pose deviation from the view median (in-sample):

| set | before (stored mount) | after (solved mount) |
|---|---|---|
| pass 1 | 62.7 mm mean, 135.0 max | 4.72 mm mean, 10.8 max |
| pass 2 | 68.0 mm mean, 132.5 max | 5.22 mm mean, 8.8 max |
| combined | 95.2 mm mean, 168.5 max | 7.42 mm mean, 12.1 max |

By branch (combined): roll 21.7 → 8.04 mm mean; roll 68.0 → 6.72 mm mean.

Leave-one-out held-out prediction (combined): **mean 7.12 mm, max 13.06 mm**, but
the training RMS on every fold is 15.3–18.1 px.

### Interpretation

- The in-sample board agreement improves 10–20×, but that is 17 views fitting 12
  unknowns (6-DoF mount + 6-DoF board) with 1–5 markers per view.
- **RMS 16.4 px against a ~1 px bar.** Adding the second branch made the fit
  *worse* (12.1 → 16.4 px), which is consistent with the residual being more than
  one rigid mount — but at this sighting density a roll-dependent term cannot be
  separated from PnP noise.
- The mount rotation comes back as a **158–164° change** in every run. With one
  or two roll branches and few markers, that rotation component is unconstrained;
  it is not a measurement.
- The board pose is **19 mm off** the measured placement.

**Therefore: no transform is published.** The data cannot support one.

## 7. Hypotheses — state

| hypothesis | state |
|---|---|
| Half turn | dead (Step 2) |
| Print / marker layout offset | dead (Step 2) |
| Blur / focus | dead (Step 2) |
| Constant roll-zero error | dead (Step 2: azimuth varies at identical roll) |
| Single constant camera offset | dead (Step 2: removes only ~40 %) |
| Lens/focal error | **not the cause** — measured edges 84–108 px at ~160 mm match f ≈ 832 |
| Roll-dependent mount term | **open, leading suspect** — cannot be separated from PnP noise at 1–5 markers/view |
| Marker count per view | **confirmed bottleneck** |
| Standoff too small | confirmed as a contributor, **not fixable** (reach limit) |

## 8. Plan update (done 2026-11-21)

`chess_sim/assets/marker_scan_plan_charuco.json` now has **17 poses**:

- 10 at roll 21.7° that decoded markers on the rig,
- 7 at roll 68.0° (same targets, +46.3°),
- **dropped** `marker24`, `marker15`, `marker23` (0 decodes at either roll),
- `visible` / `edge_px` taken from the **real captures**, not the simulator.

Views per marker (from the real captures):

```
 5:2  6:4  7:0  8:0  9:6 10:2 11:3 12:0 13:3 14:4 15:0 16:0
17:3 18:3 19:3 20:0 21:1 22:2 23:1 24:0 25:1 26:0 27:1 28:0
29:1 30:0 31:0 32:0 33:0 34:0 35:0 36:0
```

17 of 32 markers still have no view. `robot-live.service` was restarted so the
console serves the new plan.

## 9. What is still unknown

1. **Is the residual a roll-dependent mount term, a board/layout term, or PnP
   noise?** With 1–5 markers per view the three cannot be separated.
2. **Why do some poses aim off the board entirely?** marker24, marker15,
   marker23 (and marker26 at roll 68) show the bench/board edge. The measured
   mount offset is ~24 mm in board x (~1 square), which explains a one-square
   miss but not a view that misses the board.
3. **What is the true mount rotation?** Every solve returns ~160°, which is the
   unconstrained direction; the true rotation error is unknown.
4. **Does the print match the layout marker-for-marker at 244 mm?** Step 2 says
   the *joint* residual bounds it at ~1 mm for three markers at b2, but the full
   32-marker face was never compared corner-for-corner against a rectified
   top-down capture (that check was proposed in Step 2 and not run).

## 10. Options for the next step (not started)

1. **Bootstrap from the best view.** marker13 at roll 68 saw 5 markers. Take
   `X_est = FK(q)⁻¹ · board_measured · PnP_cam⁻¹` from that view, re-aim the plan
   with `X_est`, capture again. This is the procedure's "iterate once" step and
   is the cheapest path to more markers per view.
2. **Visual servoing for capture** — aim, detect, nudge to centre, capture. Breaks
   the circularity directly; a new capture behaviour.
3. **Rectified print audit** (Step 2's check #1) — top-down capture of the empty
   print compared corner-for-corner with `board_charuco.png`. Rules the print out
   definitively with no camera pose in the loop. Cheap, and would remove one of
   the three unknowns in §9.1.

## 11. Constraints in force

- Do **not** edit `board.json`, the print, the lens, the saved camera profile or
  the calibration. The plan file may be edited (Brian, 2026-11-21).
- Every robot-arm calculation runs on the BC-250, never on the Unraid host.
- Motion is pre-approved while the Pi bridge is powered and reachable; print each
  leg, run the clearance check, stop on 409 or an unexpected branch change.
- Gripper actuation is standing-authorised while the Pi is powered (Brian,
  2026-11-21).
- Exposure is settled per task by numbers and recorded with every capture.

## 12. Artifacts

**Reports**
- `reports/step2_verdict_20261121.md` — the prior verdict (pre-existing)
- `reports/handeye_transform_20261121.md` — first attempt, no transform
- `reports/handeye_transform_20261121_phase2.md` — second attempt, plan update
- `reports/handeye_capture_20261121.json` / `_b.json` / `_c.json` — per-pose receipts

**Captures** (BC-250 `live/`)
- `captures_handeye_20261121/` — pass 1 (13 jpg+json)
- `captures_handeye_20261121b/` — pass 2 (13)
- `captures_handeye_20261121c/` — pass 3, roll 68 (8)
- `captures_handeye_all/` — pass 2 + 3 merged (21) used for the combined solve

**Code changed (both trees)**
- `tools/plan_marker_scan.py` — added `--rolls`, `--target-offset-mm`, a
  calibration-limit guard around `calibration.camera()`.
- `tools/plan_wrist_scan.py` — `solve_pose(..., roll_deg=)` with a soft roll term
  (0.5 rad ≈ a 10 mm position error).
- `chess_sim/assets/marker_scan_plan_charuco.json` — the updated 17-pose plan.

**Scratch scripts on the BC-250** (`/tmp`, ephemeral — say if they should be
kept): `p4_board_clear_check.py`, `p4_square_stats.py`, `p4_rectified_view.py`,
`p4_cam_debug.py`, `p4_scan_preflight.py`, `p4_jaw_test.py`, `p4_tone_sweep.py`,
`p4_handeye_capture.py`, `p4_handeye_capture2.py`, `p4_handeye_capture3.py`,
`p4_handeye_eval.py`, `p4_final_metrics.py`, `p4_view_axis.py`,
`p4_update_plan.py`.

## 13. Bugs and oddities found along the way

1. **Console `/move` 409s after a scan.** The live console's command counter
   starts at process start; a scan advances the bridge's `last_seq` past it, so
   every later `/move` is rejected ("seq … is not newer than …"). A dashboard
   restart clears it. Not fixed in code — `command_seq` should be seeded from the
   bridge state.
2. **`p4_vision_receipt.py` settle grid tops out at exposure 300** and its
   adequacy rule accepted 180 on a dark frame (mean 85). The grid needs a wider
   range and a board-region metric, not whole-frame dark%.
3. **Old plan's jaw did not match its scoring jaw.** The committed plan carried
   jaw −0.799 rad (command 0) while the planner scored visibility at 0.785 rad
   (command 50). The new plan keeps the measured behaviour; the jaw was shown not
   to matter for decodes.
4. **Scene fovy print ≠ calibration fovy.** The planner prints "resolved fovy
   47.0" (cad_parallel pack) and the p4 tools "36.0", while the calibration and
   the measured marker sizes give 32.1° / f ≈ 832. The print is a render-model
   value; the detector and the solve use the calibration. Worth tidying so the
   prints stop contradicting the measurement.
5. **Evaluation script board-frame metric is inconsistent** (312–328 mm) with the
   base-frame metric (27 mm) for the same views. Do not quote the board-frame
   variant.

## 15. Tip-centre marker (2026-11-22) — POSE-SPECIFIC, NOT a fixed hand-eye transform

A **Tip set** dashboard tab lets the user jog the arm and click the camera image
at the midpoint between the finger tips. The pixel is stored in
`live/tip_center.json` and served at `GET/POST /tip-center` (+ `/clear`).
The detection overlay reads that pixel and draws the cyan "tip centre" cross
there, replacing the old FK-tip-through-PnP projection (invalid: it mixed the
FK tool frame with the uncalibrated camera PnP frame). This is a **display
marker only** — it does not command motion.

Saved by eye 2026-11-22: **pixel (320, 152)** in the raw 640x480 frame, chosen
at wrist roll ≈ −16.4° and the camera height/standoff of that pose.

**Brian's explicit caveat (must not be forgotten):** the chosen tip pixel is
valid only at the specific camera **height, angle and distance from the board**
at which it was set. A fixed image pixel locates the tips *in the image*, but a
jaw tip stands above the board plane; converting that pixel to a board-space
jaw target depends on camera height/angle/standoff because of parallax between
the tip depth and the board plane. Therefore:

- Do **not** claim the pixel "holds at any pose" or treat it as a solved
  camera-to-jaw (hand-eye) transform. It is a per-pose annotation.
- Before using a tip pixel to aim a grasp, re-set it at (or transform it to)
  the actual grasp pose's camera height/angle/standoff, or solve the real
  camera-to-jaw rigid transform (the still-unsolved hand-eye problem).
- This is exactly why the three grasp attempts missed: the camera crosshair
  and the jaw tip differ by tens of mm at the grasp pose (e.g. cyan jaw at board
  (6.4, 49.1) vs piece at (7.6, 6.9)), and that offset changes with pose.

### 15a. Measured structure of the pixel (2026-11-23) — u fixed, v tracks tilt

Brian's reading, confirmed by measurement: **the horizontal pixel is a property
of the mount and does not move with wrist pitch; only the along-view coordinate
follows the camera's tilt to the board.**

Jaw-difference test at fixed roll (67.7 deg), three wrist pitches:

| wrist_flex | midpoint u | midpoint v |
|---|---|---|
| 73.5 deg | 503.0 | 313.8 |
| 84.5 deg | 502.8 | 397.9 |
| 62.7 deg | 501.1 | 387.1 |

u varies by 2 px over 22 deg of pitch; v varies by ~80 px. (That test's blob
pairing was later found to pick non-tip components, so treat its absolute values
as suspect — the **u-invariance** is the part that held up, and it is the part
that matters.)

Ground-truth clicks by Brian agree: **u = 320.0 at roll -16.4 deg** (2026-11-22)
and **u = 323.7 at roll +67.7 deg, wrist_flex 72.5 deg** (2026-11-23), with v
moving 152.0 -> 240.6 between those poses.

Consequences:

- One click per mount fixes **u** permanently; only **v** needs relating to the
  camera's tilt, so the marker is no longer "wrong at every other pose".
- Automatic blob-difference measurement is **not** trustworthy for this: the two
  largest moving components are not reliably left-finger + right-finger, and
  produced a 180 px u error when tried.
- `POST /tip-center` now stores `joints_deg` with the pixel, so a stored pixel
  carries the pose it is valid at instead of looking pose-free.

### 15b. Sim verdict (2026-11-23): the tip pixel is a MOUNT CONSTANT in both axes

Brian's suggestion was to settle this in sim by casting from the tips and
projecting through the model camera. Done with the CAD gripper and its camera
mount (`ROBOT_GRIPPER=cad_parallel`), projecting the `so101:gripperframe` site
(which sits on the fingertip plane):

| wrist_flex | projected uv | jaw axis off vertical | ray hit vs tip y |
|---|---|---|---|
| 45° | (317.8, 290.2) | 30.0° | 303.5 vs 14.9 mm |
| 60° | (317.8, 290.2) | 15.0° | 102.9 vs −26.5 mm |
| 75° | (317.8, 290.2) | 0.0° | −70.9 vs −70.9 mm |
| 90° | (317.8, 290.2) | 15.0° | −244.7 vs −115.3 mm |
| 105° | (317.8, 290.2) | 30.0° | −445.3 vs −156.7 mm |

Roll sweep at fixed pitch: identical pixel at −45°, 0°, +45°.

**Both u and v are invariant (0.0 px) across 60° of pitch and 90° of roll.** The
camera and the fingertip plane ride the same rigid body, so the pixel is a
property of the mount, not of the pose. The earlier "u fixed, v tracks tilt"
reading was an artefact of comparing clicks made at different times — v cannot
move kinematically.

Side benefit, and it is Brian's ray idea working: at wrist_flex 75° the jaw axis
is exactly vertical and the ray from the tips lands on the tip XY to 0.1 mm,
while at 15° off vertical it misses by ~130 mm. That is a cheap check that a
grasp pose is square to the board.

**Adopted (Brian, 2026-11-23): tip centre = (320, 250).** x is pinned to the
frame centre, y from the latest click. The jaws are 2–3× the piece size, so the
few-pixel residue does not matter for pickup. The overlay now draws this
constant and no longer projects the FK tip: a projection inherits the 6.6° FK
error and moves the marker exactly when the pose is weakest.

## 14. Rig state at pause

- Bridge: **HOLD**, armed, no fault.
- Joints: shoulder_pan −7.91, shoulder_lift −59.91, elbow 78.90, wrist_flex
  40.84, wrist_roll 67.96, gripper 17.88 (last roll-68 pose).
- Camera controls: exposure 705, gain 64, brightness 222, contrast 32
  (the saved profile values; the 160-brightness setting measured better and is
  not persisted).
- Board: clear at e5; no piece on the playing area in any capture.
