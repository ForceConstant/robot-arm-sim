# Tip centre projection and aim — the reference

Date started: 2026-11-24 (Brian + Vera). Last updated: 2026-11-24 14:45.

**Read this before deriving any camera or aim number.** Most of the confusion in
this area has come from re-deriving things that were already settled, or from
using one word for two different points.

---

## 1. Two different points, never interchange them

| Name | Definition | Pose dependent? |
|---|---|---|
| `TIP_GRIPPER` | the physical jaw tip, `(0, 0, -0.1106)` m in `so101:gripper` | no |
| `TIP_PIXEL` | `TIP_GRIPPER` projected into the camera | **no** — a constant. Diagnostic only |
| `AIM_WORLD(q)` | ray from the **jaw origin through the tip**, extended to the board plane | yes |
| `AIM_PIXEL(q)` | `AIM_WORLD` projected into the camera | **yes** — this is what you aim with |

Brian's rulings (2026-11-24):

1. The aim ray is **jaw origin → tip centre → straight out**. It is the tool
   axis, *not* world-vertical. The two coincide only when the tool is vertical.
2. The camera mount is **rigidly fixed to the jaw and rotates with the wrist**.
   No compliance term. `TIP_PIXEL` invariance is therefore an identity.

### Why `TIP_PIXEL` constancy is not a result

Camera and tip are both rigid in `so101:gripper`. The tip's projection is
invariant under *every* joint configuration — roll, flex, arm pose. That is
algebra, not measurement. A simulation cannot falsify it, so a sweep proving it
proves nothing. Only `AIM_PIXEL` may be described as moving with Z, and only it
may be used to aim.

This exact mistake was made on 2026-11-24: a roll sweep returned 0.000 px spread
and it was reported as a finding. It was a tautology dressed as evidence.

`AIM_PIXEL` also **cannot be moved by rotating the image**. Rolling the wrist
rotates the tool about its own axis; the jaw origin and tip both lie on that
axis, so the ray does not move. If an aim is in the wrong quadrant, the geometry
is wrong — rotation will not fix it.

### `AIM_PIXEL` moves along one image axis only (confirmed 2026-11-24)

The camera sits offset from the jaw axis in a single plane that contains that
axis. The aim ray therefore projects displaced along **one** image direction and
cannot move in the perpendicular one.

In the confirmed landscape + camera-roll −90° setup, the aim moves **left/right
only**; its vertical position is fixed to within ~1 px. Measured across 10 live
poses spanning 0…40° of wrist roll and varied pan:

| pose | roll | aim | tip | aim − tip |
|---|---|---|---|---|
| 1 | −0.3 | (299, 243) | (395, 243) | (−96, −1) |
| 3 | −0.3 | (270, 243) | (395, 243) | (−125, −1) |
| 4 | −0.3 | (251, 242) | (395, 243) | (−144, −1) |
| 7 | +5.0 | (276, 243) | (395, 243) | (−119, −1) |
| 8 | +10.3 | (271, 243) | (395, 243) | (−124, −1) |
| 10 | +39.6 | (264, 242) | (395, 243) | (−131, −1) |

`TIP_PIXEL` stayed at (395, 243) in every pose — the rigid identity, exact.

So an expectation of a diagonal ("up and left") aim is geometrically impossible
here; the offset is one-dimensional. Earlier notes in this session expecting a
vertical component were mistaken.

---

## 2. Source of truth — where the numbers actually live

| What | Where |
|---|---|
| **Accepted camera mount (B5 reference, commit 37e7232)** | `chess_sim/tool_reference.py` → `CAMERA`, `camera_in_gripper()` |
| Tool transform chain | `chess_sim/tool_frames.py`, explained by `docs/TOOL_FRAME.md` |
| Calibrated bench mount (block 4c, older tool) | `chess_sim/scene.py` → `WRIST_CAMERA_MEASURED` |
| Established portrait projection + roll convention | `tools/b5b_evaluate.py` (§50–120) |
| Hand-eye **and** board-pose solver | `tools/handeye_bundle.py` |
| Live wrist-roll ↔ sim roll conversion | `chess_sim/config.py` → `sim_wrist_roll()`, `WRIST_ROLL_ZERO_SIM_DEG` |
| Orientation regression | `tests/test_tool_orientation.py` |
| Side-view evidence renders | `reports/b5_camera*_4x.png` |

**Accepted camera in `so101:gripper`:** position `(0.41, 60.34, -23.74)` mm,
optical axis `(0, -0.4945, -0.8692)` = **29.64° off −z**, fovy `32.144°`,
fx 831.897 / fy 833.014 / cx 320 / cy 240 at 640×480.

### The trap that keeps biting

In `so101:gripper` the camera's **lateral offset is along +Y** and its tilt is in
the **Y–Z plane**. Putting that offset along X — the natural reading of a CAD
sketch drawn in X/Z — silently yields a mount rotated 90° about the approach
axis. The wrist view still looks plausible; the camera is simply on the wrong
side.

Brian's CAD is quoted **Z-down with the lateral in X**. To compare:

- CAD lateral X → gripper **Y**
- CAD Z (down) → gripper **−z**

---

## 3. Confirmed conventions for sim-vs-IRL comparison (2026-11-24)

Brian confirmed these by eye. Do not change them without asking.

1. **Compare in landscape.** IRL frames are used as captured (640×480), no
   rotation.
2. **The sim wrist camera is rolled −90°** about its optical axis so its view
   matches the real C270. Implemented by offsetting the camera's x/y axes:
   `R_cam = camera_in_gripper()[1] @ Rz(-90°)`, then `add_camera(..., xyaxes=...)`
   on the `so101:gripper` body. Roll +90° does **not** match.
3. **Apply `sim_wrist_roll()` to the wrist_roll joint.** The sim roll is
   `live_roll + WRIST_ROLL_ZERO_SIM_DEG`. The offset was **−87.21°**; re-measured
   2026-11-27 against eight bench poses as a median of −7.5° further, so it is now
   **−94.71°** (Brian approved; see `reports/fov_marker_investigation_20261127.md`).
   The same change shifted `chess_sim/assets/wrist_scan_plan.json` by −7.5° so those
   poses stay physically identical. A 3.7–10.5° pose-dependent residual remains and
   is *not* to be absorbed by moving this constant again.
   Skipping this rotates the rendered pose ~87° and wrecks the comparison.
4. **Render at the measured lens, not the render default.** Use
   `fovy = 2*atan(240 / 833.014) = 32.144°` so sim pixels correspond to real
   pixels. The project's `WRIST_REFERENCE_FOVY_DEG = 47.0` is a scale fudge for
   matching marker size and must **not** be used when comparing pixels.

Modifying the camera via `spec.cameras[...]` and setting `.quat` **silently did
not take effect** in this MuJoCo version. Add a fresh camera to the gripper body
with explicit `xyaxes` instead — that works and is verifiable (rendered image
hash changes with roll).

---

## 4. What has been validated

### 4a. Projection maths — validated in sim against real geometry

A real thin cylinder was placed along the aim ray, with a dot at the board
intersection, both positioned each pose from the tip's world position. The dot
found by colour mask was compared with `AIM_PIXEL` from the formula:

| standoff | predicted | rendered | error |
|---|---|---|---|
| 20 mm | (318.1, 238.2) | (317.6, 235.3) | 3.0 px |
| 60 mm | (318.5, 169.3) | (318.1, 168.0) | 1.4 px |
| 120 mm | (303.8, 110.2) | (303.3, 108.9) | 1.5 px |
| 200 mm | (104.5, 90.3) | (106.2, 94.0) | 4.0 px |

Residual is the marker's own visible hemisphere, not the model (shrinking the
marker made the error *worse*). The chain tip → ray → board → camera →
intrinsics is correct.

### 4b. Behaviour with standoff

With the tool vertical, the aim starts near frame centre and sweeps up-frame as
the arm rises, staying in frame past 200 mm — consistent with Brian's CAD
sketch. At reach limit (~200 mm up) the arm cannot hold the tool vertical and
the ray visibly tilts, which is the "top row at some Z" case.

### 4c. First real-frame comparison (pose 142257)

With the confirmed landscape + camera roll −90 conventions, on live pose
`pan -12.84 lift -10.59 elbow 6.73 flex 97.54 roll 0.35`:

- `AIM_PIXEL` = **(140.8, 241.8)**
- `TIP_PIXEL` = **(395.0, 243.3)**
- aim is **254 px left of the tip, level with it**

Left-hand side matches what Brian sees by eye. The remaining discrepancy is the
**vertical** component: level here, expected raised. Most likely cause is the
board plane being intersected — the sim board sits at z = 0.4332 m and the real
board is at a different height, which moves the aim along the ray.

---

## 5. Open items

1. **Use the real board plane.** The aim must be intersected against the board
   the robot is actually over, recovered from the ArUco markers, not the sim's
   board. `tools/handeye_bundle.py` jointly solves camera mount and board pose
   from marker sightings and is the intended tool.
2. **Confirm the mount on real data.** Everything so far uses the B5 reference
   mount. If the physical mount differs (the CAD sketch implies ~6 mm more axial
   offset: 29.733 vs 23.74 mm), the bundle solve will show it.
3. **Capture a pose over the board.** The first live capture was taken with the
   arm off the board; a pose over the board gives a meaningful aim comparison.
4. **Board orientation.** The sim board's yaw is fixed by `BOARD_TURN`; it was
   matched to IRL previously. Re-verify once a real pose over the board exists.

---

## 6. Tooling notes

- Everything here runs on the **BC-250** (`brian@192.168.1.171`,
  `/home/brian/robot-arm-sim`), never the Unraid host:
  `ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.../gripper-assets-5y
   PYTHONPATH=/home/brian/robot-arm-sim ~/.local/bin/robot-arm-sim python <script>`
- Live app API is on `127.0.0.1:8080` on the BC-250; the Pi bridge is
  `192.168.1.167:8100` with token in `live/bridge_token.txt`.
- `/capture` is a read-only GET and is safe to poll; `/move` is not, and the
  arm may be in use by another agent.
- The live app's `command_seq` is seeded once at startup and `+= 1` per command.
  After a few days up it falls behind a time-based client and every `/move`
  returns `409 seq is not newer than …`. A restart re-seeds it.
