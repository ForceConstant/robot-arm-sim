# 2026-11-19 — Finalize geometry, camera, and sim↔IRL parity

**Owner/planner:** librarian session (this session)
**Workers:** one agent per block, own session, report back
**Status legend:** `[ ]` pending · `[~]` in progress · `[x]` done · `[!]` blocked

## Today's goal (Brian, 2026-11-19 06:42)

1. **Sim and IRL both do every board position via Cartesian control**, and agree.
2. **Camera calibration and board calibration work** — on the real arm, not just in sim.
3. **Sim matches IRL completely.**
4. **Finalize the model orientation in sim** — yesterday's remaining defect.

**Hard constraint: the Raspberry Pi 4 / arm cannot be powered up until ~08:00.**
Everything before that is sim, CAD and software only. No IRL block may start
early, and no block may "warm up" the bridge to test a theory.

## Starting state (measured, 2026-11-19 06:47)

### The planner checkout is **stale** — read this first

| tree | head | note |
|---|---|---|
| planner `/home/node/.openclaw/workspace/robot-arm-sim` | **`02c9a4e`** | dirty: `config.py`, `scene.py`, `fiducials.py`, `assets.py`, `chess_so101.xml`, `.gitignore`, `ROADMAP.md` |
| BC-250 `/home/brian/robot-arm-sim` | **`941a8b8`** | 13+ commits ahead, plus untracked `reports/b5_*` and `hardware/onshape-camera-mount-2026-11-18/` |

Nothing from yesterday's Blocks 4b/4c/5/5x/5y was pushed. **All of it exists only
on the BC-250.** Planning or reviewing against the planner tree right now would
be reviewing week-old code. Block 0 fixes this before anything else.

### What yesterday established (carried forward)

- **Cartesian path is sound in sim:** 64/64 with the real bridge joint limits,
  worst 14.9 mm (f8), zero limit violations, tightest margin +2.93° (g8).
- **Board calibration is sound in sim:** ChArUco multi-pose `/scan` recovers
  ±8 mm / ±2.5° placements to 0.22 mm / 0.048°; the named procedure is
  **`/scan` fusion for x/y/yaw + a multi-marker planar fit for z**.
- **The lens is measured, not assumed:** fx 831.9 / fy 833.0, fovy 32.14°,
  zero distortion, RMS 1.067 px over 106 ChArUco frames. The old fx 312.77 was
  2.66× wrong.
- **The 4c hand-eye mount is real:** `camera_in_tool` translation
  **(67.66, −2.92, −10.88) mm**, sim-solved = IRL-solved to the last digit,
  1.7 px median over 264 corners.
- **But 4c was solved on the OLD tool.** Applied to today's fingers it aims
  through them: 0 % board, 100 % occlusion. The lens is 99.7 mm above the tips
  and 124.0 mm above the board, where a 60 mm finger subtends ≈33° against a
  32.14° frame. **No aim fixes that** — solved exactly as specified, both tips
  land symmetric at 87 % frame height and the board is still invisible.
- **The tip measurement moved:** functional tips at `(−0.30, ∓16.00, −110.27)`
  mm in `so101:gripper`, extremum −110.60. Committed `TIP_IN_CHAIN` is 104.425.
- **Jaw meshes scatter** when placed from the assembly-root occurrences: the
  sim jaw bodies live under the tool base, the occurrences are in the assembly
  root. This is the orientation defect Brian wants closed.
- **Merged base + camera mount is correctly attached** at the profile mount
  (`pos 0.009462, 0.023429, −0.062577`, `quat (0.707107, 0, 0, 0.707107)`),
  and identity placement is 90°/60 mm wrong — that experiment is settled.

### Un-applied change waiting in the tree

`MAX_RMS_PX` is still **1.2** in the planner `chess_sim/fiducials.py`; Brian
approved **1.5** and it was never applied. Resolve its location (the constant
does not appear in the BC-250 tree under that name) in Block 0 and apply it in
exactly one place.

---

## Visual review gates — Brian signs off on geometry (standing rule)

Yesterday's confusion came from pictures nobody could attribute. A render made
from a different pack, a different tip constant or a different camera is worse
than no render: it looks like evidence. So every block that touches geometry
produces a render set and **stops for human review before it commits or hands
off**.

**Blocks that carry a gate:** Block 1 (orientation), Block 3 (sim camera mount),
Block 5b (pixel-parity overlay), Block 6 (board registration). Block 8 adds a
post-sweep contact sheet for the record but does not gate.

**Every render in a gate must carry provenance**, printed onto the image or in
an adjacent caption block — not left to memory:

```
git head        the exact commit the render was made from
gripper pack    resolved gripper.KIND + asset root (stock vs cad_parallel, which pack)
tip constant    the resolved TIP_IN_CHAIN z used by the solver that produced it
board pose      x/y/z/yaw, and whether it is the sim twin or the live board
camera          intrinsics source + mount source (solved / assumed / placeholder)
pose            the joint vector or marker14, and the hover height
```

**Required render set per gate** (one contact sheet, tiles labelled, numbers
printed on the sheet, not only in prose):

1. External orthogonal views — front, side, top — full arm, board visible.
2. Close-up of the jaw/base seam and the fingertip plane, with a dimension
   callout drawn in millimetres.
3. The wrist-camera view, as the camera actually sees it.
4. Side-by-side with the corresponding IRL photo where one exists.

**The gate:** the block posts the sheet, states what the reviewer should check,
and waits. A "looks right" without a named check is not an approval. If the
render disagrees with the numbers, the render wins until the discrepancy is
explained — pictures have already caught two real defects this week that the
numbers called clean.

**Attribution rule:** a render may only be cited from the same commit it was
made from. Re-running after a change invalidates the old sheet; re-render, do
not re-use.

---

## Block 0 — Reconcile the trees (sim, 15 min)

- [ ] Fetch the BC-250 commits into the planner checkout (bundle or `git fetch`
      over ssh — do **not** push, do **not** rewrite BC-250 history)
- [ ] Reconcile the planner's dirty files against `941a8b8`. The BC-250 is
      authoritative for `config.py`, `scene.py`, `fiducials.py`, `assets.py`,
      `chess_so101.xml`; the planner's `MAX_RMS_PX` intent is the only thing
      worth porting forward
- [ ] Confirm on **both** trees: `TIP_IN_CHAIN` = `(0, 0, −0.104425)`,
      `move_tolerance_mm` = 20, no `jaw_frame` / `tool_tip_mm` anywhere
- [ ] Confirm `live/live_config.json` on the BC-250 still has
      `marker_z_m = 0.4337` and `plan = marker_scan_plan_charuco.json`
- [ ] Record both heads in this doc

**Exit gate:** planner and BC-250 at the same commit, both trees clean or
dirt-documented, no `live/` file mirrored. Nothing else changes.

---

## Block 1 — Finalize the sim model orientation (sim, 60–75 min) ← Brian's ask

**This is the day's gating block.** Nothing downstream can be trusted until the
tool's frame convention is one thing, written down, and machine-checked.

The defect: the base and fingers are now in the right orientation visually
(Brian confirmed), but there are **three competing ways** the tool is placed —
the assembly-root occurrences, the profile `mount_position/rotation`, and an
identity mapping onto `so101:gripper` — and the jaw bodies are still placed by
one while sitting under another. That is why 5y scattered.

- [ ] **Write the transform chain in one place**, as a single documented
      composition: `so101:gripper` → tool base → jaw bodies → fingertips. State
      the parent of each part, the transform, and the frame each part's mesh is
      authored in (CAD base frame vs mesh frame).
- [ ] **Resolve jaw placement**: express the jaw body transforms **in the
      tool-base frame** (the frame the profile mount already speaks) rather than
      the assembly root, so the occurrences compose once. If an extra
      assembly-root ↔ tool-base offset is needed, compute it and write it down —
      do not guess one.
- [ ] **Verify against the CAD**, not against a feeling: the placed jaw meshes
      must reproduce the base's own x range (13.25…69.20 mm), the fingertip
      depth (**110.27 mm** measured, 110.60 extremum), and the finger span
      (±27.83 mm at the roots).
- [ ] **Add an orientation regression test** (`tests/` or `live/tests/`) that
      asserts, from the built model: base axes, finger direction, tip depth,
      jaw symmetry, and the camera mount position. It must fail if anyone
      re-introduces an identity map or a 90° error. Eyeballing renders is what
      let this survive a day.
- [ ] Render orthogonal + wrist views at marker14 and compare to the IRL photos;
      save under `reports/b1_orientation_*`.
- [ ] **HUMAN REVIEW GATE — do not commit until Brian approves.** Post the
      contact sheet the standing rule requires (external ortho pair, jaw/base
      seam close-up with mm callouts, wrist view, side-by-side with the IRL
      photo) plus provenance. Name the three checks he should make: the base sits
      under the wrist with no gap; both fingers symmetric and on the same plane;
      fingertips at the stated depth. Record his approval in the receipt before
      committing.
- [ ] Confirm no floating proxy remains (the `camera_collision` housing proxy
      was deleted in 5y; check for the servo/gear box that survived).

**Exit gate:** one documented transform chain, zero scattered geometry, the
orientation test green, and the three CAD cross-checks matching to <1 mm.
`TIP_IN_CHAIN`, the gate, the plan and any calibration file **unchanged**.

---

## Block 2 — Tip constant: like-for-like (sim, 20 min, parallel with Block 1)

Report only. Brian rules.

The worker reported a **67.89 mm** delta against `TIP_IN_CHAIN`. That figure is
a **datum mismatch**, not a discovery:
`docs/TIP_FRAME_CONVENTION.md` defines `TIP_IN_CHAIN` as **104.425 mm along the
`so101:gripper` body −z axis** — measured from the *gripper body origin*. The
worker's 172.31 mm is measured from the **wrist body origin**, which sits
61.1 mm along −y and 18.1 mm along +z from the gripper origin.

- [ ] Produce one table with **all** candidate datums (gripper body origin,
      wrist body origin, wrist-roll joint anchor, `gripperframe` site) and the
      same tip point measured from each
- [ ] State the **like-for-like** delta: measured drop below the gripper origin
      is **110.27 mm** (extremum 110.60) vs committed **104.425** →
      **+5.85 mm** (+6.18 at the extremum). Confirm this arithmetic
- [ ] Confirm whether the 5.85 mm comes from real geometry change or from the
      measurement picking a different vertex
- [ ] **Do not change `TIP_IN_CHAIN`.** Present the value Brian would set and
      what it would move

**Exit gate:** one number, one datum, traced to the doc. No edits.

---

## Block 3 — Camera: re-solve the mount against today's tool (sim, 60–90 min)

**The root cause is now known and it is not a CAD problem yet:** the 4c mount
was solved against the **older finger geometry**. Re-solve it against the tool
that actually exists. That is the cheapest path to "camera calibration works",
and it either clears the CAD question or proves a standoff is genuinely needed.

- [ ] Using the **measured intrinsics** (fx 831.9 / fy 833.0, cx 320, cy 240,
      zero distortion, fovy 32.1445°) re-solve `camera_in_tool` on the current
      model, from the same 37 `cal` joint vectors and the ChArUco board
      geometry, with the OpenGL/OpenCV convention handled explicitly
- [ ] Report the new translation, look direction, view count, sightings and RMS
      — plus the delta from 4c `(67.66, −2.92, −10.88)`
- [ ] **Render the wrist view from the new mount** with the full scene, board
      and markers. Required measures: board coverage %, markers visible, tool
      occlusion %, and both fingertips' image coordinates
- [ ] Compare against Brian's reference photo. If the board is still invisible,
      report the **minimum standoff along the mount direction that yields
      ≥60 % board coverage** as a single number — that is the CAD decision,
      stated once, instead of another pose hunt
- [ ] Only then apply, and only if the render shows the board
- [ ] **HUMAN REVIEW GATE** — post the wrist render with board coverage %,
      markers visible, occlusion %, both fingertip image coordinates, and the
      full provenance block. Brian confirms the framing before this mount is
      written anywhere the live path can read.

**Exit gate:** a solved mount whose render shows the board, with the numbers to
prove it, **or** one precise standoff requirement in millimetres. Either is a
pass; another ambiguous pose is not. The mount this block produces is the input
to Block 5b, so record it in the receipt with full precision.

---

## Block 3b — Re-run the 64-square sim sweep on the settled geometry (sim, 45 min)

**Added 2026-11-19 after Blocks 1 and 2 landed.** The 64/64 sim result was flown
with the old jaw placement and `TIP_IN_CHAIN = 104.425`. Both have changed, so
that receipt is now stale and cannot be used as the sim baseline for the IRL
comparison in Block 9.

- [ ] Confirm the compiled tool first: resolved `gripper.KIND`, asset root, and
the tip constant the solver is actually using (from `/state` or the stamp)
- [ ] Re-run the full sweep with the bridge joint limits, 20 mm gate, same
solver and targets as the IRL run
- [ ] Record per square: target, IK error, achieved tip, joint vector, guard
status
- [ ] Report the worst square and how much margin it has left against the gate

**Exit gate:** 64/64 at 20 mm with the new geometry and the derived constant, or
a named physical cause for any failure. Movement of individual residuals is
expected and is not a failure — a *new* square crossing 20 mm is.

**Known tension to watch:** the convention regression moved a1 3.90 → 4.80 mm and
h8 12.30 → 17.10 mm when the constant changed. h8 is now 2.9 mm inside the gate
instead of 7.7. If any square lands over 20 mm, that is a reach/limit finding to
report, not a reason to touch the constant back.

---

## Block 4 — Bench bring-up (IRL, from 08:00, 20 min)

- [ ] Pi 4 powered at 08:00; `GET http://192.168.1.167:8100/state` answers
- [ ] Port ownership: LeLab / teleop stopped before the follower port is opened
- [ ] Open the follower and read joint feedback **before** any command. Power
      alone is not readability — expect `joints_source=none` until it is opened
- [ ] REMOTE mode, arm enabled; **change modes once and leave it** (HOLD↔REMOTE
      cycling adds ~3°/cycle — a known, unfixed defect)
- [ ] Record the bridge's real joint limits and compare to the 2026-11-18 set
      (pan −108.99…121.38, lift −107.05…108.64, elbow −96.68…94.75,
      wrist_flex −104.59…98.79, **wrist_roll −81.82…91.05**, gripper 0…100)

**Exit gate:** bridge reporting real joint positions, arm holding still, `armed`
and `follower_open` as expected, limits recorded. **No motion beyond that.**

---

## Block 5 — IRL camera mount re-solve (45–60 min) ← "camera calibration works"

This is the IRL twin of Block 3, and it must run **before** `/scan`, because a
mount that aims through the new fingers cannot register the board at all.

- [ ] Capture the ChArUco frames at the 37 documented `cal` joint vectors, raw
      wrist frames only — never a photo of a monitor or a phone screen
- [ ] Detect at the measured intrinsics; require the valid-view gate
      (≥2 markers, no tag clipped by the frame border)
- [ ] Solve `camera_in_tool`; report translation, look direction, views,
      sightings, RMS, and the **sim-vs-IRL mount agreement** (the whole point is
      that these match)
- [ ] Compare to the sim solve from Block 3. A large split means the sim model
      is still wrong — stop and report, do not average them
- [ ] Apply only if the view-count and RMS gates pass; back up the previous
      file first

**Exit gate:** mount solved with its view-count and RMS gates passed, and the
IRL mount agrees with Block 3's sim mount. **This gate does not prove the two
cameras match** — see Block 5b, which does. A large split means the sim model is
still wrong: stop and report, do not average.

---

## Block 5b — Pixel-parity gate: the two cameras must be the same camera (30 min)

**Why this block exists, in one sentence:** a mount delta can be fooled by a
compensating pair of errors — if the sim's finger geometry is wrong by 3 mm, the
IRL solve bends the lens 3 mm the other way and both calibrations agree
perfectly while both are wrong about the hardware. Copying the IRL mount into
the sim would then launder that error into the model permanently.

So the binding test is **image space**, not parameter space: put the arm at the
same joint vectors in both worlds, against the same board, and compare where
the ChArUco corners land in the frame.

- [ ] Move the real arm to each of the documented `cal` joint vectors and capture
      the raw wrist frame (raw frames only — never a photo of a monitor/phone)
- [ ] Undistort with the measured intrinsics; detect ChArUco corners
- [ ] Render the sim view at the **same** joint vectors with the sim board pose,
      with Block 3's solved mount and the same intrinsics; project the same corner
      set through the sim camera
- [ ] Report per view: matched corner count, **median and max pixel error**, and
      the per-corner residual plot
- [ ] Apply the valid-view rule from Block 4c: ≥2 detected markers and no tag
      clipped by the frame border (`edge_px/6 + 1 px`). Excluded frames get
      tabulated with their border distance and an `_excluded` overlay — never
      quietly dropped

**Thresholds (unchanged from the 2026-11-18 Block 4c gate):**

```
valid-view set     median ≤ 2.0 px     max ≤ 4.0 px
reference (4c)     median 1.252 px     max 3.382 px     12 views / 152 corners
```

**Exit gate:** thresholds met on the valid-view set. **Failing means the sim
camera or the tool geometry is wrong — stop, do not bank the mount and do not
widen the threshold.** A pass here is the only evidence that satisfies "sim
matches IRL completely" for the camera.

- [ ] **HUMAN REVIEW GATE** — post the corner-overlay images (sim corners drawn
      on the real frame, residual per corner) next to the numbers, so the pass is
      visible rather than asserted.

**If it fails, the diagnosis order is fixed:** tool geometry (Block 1's
cross-checks) → intrinsics → mount → board pose. Fail the earliest one that
cannot hold, and report it. Do not re-fit the mount to close a pixel residual
that came from a misplaced finger.

---

## Block 6 — IRL board calibration (45 min)

- [ ] `/scan` with `plan = marker_scan_plan_charuco.json`, `marker_z_m = 0.4337`
- [ ] `/scan` is **x/y/yaw only** — run the multi-marker plane fit for z
- [ ] Compare to the saved board pose **as relative offsets**, never absolute z
      against the sim twin's 433.7 (`board.json` x/y are absolute in the arm
      frame with the base link at y = −217; `/scan` offsets are base-relative)
- [ ] A large z or yaw change means stop and re-fit, not fudge

**Exit gate:** board registered, offsets inside 2 mm / 1°, plane fit stable
across views, and a written record of every offset changed.

- [ ] **HUMAN REVIEW GATE** — post the detected-marker overlay from `/scan` and
      the board pose before/after, with provenance. A board in the wrong place
      still produces a clean-looking scan, so the picture is the check.

---

## Block 7 — `/sanity` and the 8-square dry run (30 min)

- [ ] `/sanity`: four corners + centre. **Any corner over 6 mm stops the sweep**
- [ ] 8-square dry run (a1, b1, f1, g1, h1, a2, g2, g3) with `dry_run` only
- [ ] Confirm every square is `OK` / `RESCUED_BY_SEED`, and that no refusal is a
      keep-in refusal mistaken for unreachability

**Exit gate:** sanity inside 6 mm at all five points, all 8 squares dry-run
clean. Otherwise stop and report which square, which axis.

---

## Block 8 — Full 64-square IRL sweep (60–90 min)

- [ ] Same solver, same targets, same 20 mm gate as sim — **speed 0.3, pose
      timeout 45 s, keep-in on**
- [ ] Record per square: target, IK error, achieved tip, joint vector, guard
      status, marker count
- [ ] No full sweep to discover a single defect: dry-run the known-failing
      squares first (f8 at 14.9 mm is the worst sim case; c8–f8 are ≤15 mm
      fast-path accepts and are not failures)

**Exit gate:** 64/64 at 20 mm, zero guard refusals reported as unreachable.
A disagreement is a finding to record, not a number to tune.

---

## Block 9 — Square-by-square sim-vs-IRL comparison (45 min)

Camera parity is already settled by Block 5b; this block is the **tip-space**
half of "sim matches IRL".

- [ ] Join the two receipts by square; report per-square tip delta and the
      worst case
- [ ] Classify every disagreement: solver seed, joint limit, calibration, or
      hardware
- [ ] Update `docs/ROADMAP.md` and write the day's receipts

**Exit gate:** a per-square agreement table, every delta over 5 mm explained,
and the day's goal answered explicitly: does sim now match IRL completely?

---

## Decisions waiting on Brian (block the day if unanswered)

1. **Camera standoff.** If Block 3's re-solve still cannot see the board, the
   lens needs to move along the mount. Yesterday's number was **+25 to +35 mm**.
   Block 3 will replace that estimate with one solved figure — Brian rules on it.
2. **`TIP_IN_CHAIN`.** Committed 104.425, measured 110.27. Like-for-like delta
   **+5.85 mm**. Worker proposes ~110.27 (from the gripper origin); Brian has
   not ruled.
3. **a8 wrist_roll.** Solved at 160.8° in one branch, 0.00° from the extended
   branch; bridge limit is 91.05°. Block 4 records the real range; if the bridge
   range is genuinely ±91°, confirm the extended branch is what runs IRL.

## Rules for the day

- **The Pi does not power up before 08:00.** No IRL block, no bridge probe, no
  "just checking" before then.
- **Geometry freezes when Block 1 passes.** No orientation tweak mid-sweep; a
  new orientation finding goes in a new block.
- **Orientation is machine-checked.** If it can only be seen in a render, it is
  not verified.
- No gate loosening. No calibration constant changed without a measurement that
  predicts it. No retuning the solver to hide an IRL disagreement.
- No full 64-square sweep to discover one defect.
- Every sim run, calibration solve and sweep runs on the **BC-250 in CachyOS**
  (`brian@192.168.1.171`), never the Unraid/Docker host.
- Workers commit locally and **do not push**; the planner pushes.
- Sync only `live/kinematics.py`, `live/app.py`, `tools/cartesian_squares.py`
  by hand. **Never mirror `live/`** — the BC-250 owns `live_config.json`,
  `board.json`, `poses.json`, `bridge_token.txt` and `captures/`.
- Three failed attempts on one defect: stop and report the precise
  inconsistency, not a fourth theory.
- Never calibrate against a photo of a monitor or a phone image. Raw wrist
  frames or existing artefacts only.

---

# What we learned — 2026-11-19 / 20 (added to the plan as required reading)

Everything below is measured, with the receipt named. It supersedes earlier
numbers where they conflict.

## 1. Wrist-roll zero

- **Operator `wrist_roll` 0° = sim `so101:wrist_roll` −87.21°.** Implemented as
  `chess_sim/config.py: WRIST_ROLL_ZERO_SIM_DEG` + `sim_wrist_roll(deg)`.
- Method: rendered a roll sweep, nulled on the base's own edge (−87.21° made the
  base `+y` edge horizontal in the front view; −90.00 was Brian's pick within
  1.7° of it).
- **Range correction:** the sim joint allows −157.2…+162.8°; the *bridge* limit
  is **−81.82…+91.05°**. Do not plan a sim pose outside the bridge range.

## 2. Camera-side conventions

- The real C270 is mounted **90° rotated** relative to the model. Renders that
  match the real frame are **portrait 480×640**; the rotated vertical FOV is the
  old horizontal FOV, **42.0283°** (fx 831.897 / fy 833.014 differ 0.13 %;
  MuJoCo carries one `fovy`, so both cannot be exact).
- **`fovy 47.0` is baked but PROVISIONAL** (`e4dba38`, env `ROBOT_WRIST_FOVY`).
  It matches the *average gravity sag* at the reference pose, not a stable
  measurement. Do not treat it as a solved constant; see §5.
- Rotation cannot be validated by marker detection — the detector is
  rotation-invariant (same ids, same side length at 0/90/180/270°).

## 3. Board pose (applied)

`live/board.json` (gitignored; record in `reports/board_pose_applied.json`):

| | before | after |
|---|---|---|
| `yaw_deg` | 5.67 | **1.92** |
| `x_mm` | −9.92 | **−34.92** |
| `y_mm` | −3.13 | **−18.13** |
| `z_mm` | 431.35 | 431.35 |

Sim/IRL markers then match to **3.9 px mean** over ids 27/28/31.

## 4. Nine bench checks — all within ±3 mm

| check | measured | model |
|---|---|---|
| base mesh top above table | 102 | 102 |
| board stack (slab) | 3.20 | 3.20 |
| marker pitch | 28.00 | 28.00 |
| tool span (tip → camera point) | 105.0 | 105.2 |
| hand (tip → wrist-roll axis) | 112 | 110.45 (+1.55) |
| upper arm (horn→horn) | 116.00 | 116.00 |
| forearm (horn→horn) | 135.00 | 135.00 |
| table → `shoulder_lift` axis | 152 | 149.0 |
| wrist section | 170 | 172.50 |

**Conclusion: the geometry is sound. The residual is mechanical, not dimensional.**

## 5. Arm slack — the number that governs everything

- Shoulder deadband measured two ways, both agreeing: commanded downward moves
  stopped **4.19°** and **4.44°** short *in the direction of travel*.
- Directional test, 3 repeats each: approached from above settled **34.24°**,
  from below **30.46°** — difference **3.78° ≈ 14 mm at the tip** (3.7 mm/deg),
  with **0.00° spread within a direction**.
- Hand test: ~**20 mm** of slack at extension (gravity).
- **Deterministic, not noise** — a one-directional approach lands on the same
  side of the slack every time. **Usable, if the move is built that way.**
- **Withdrawn claim:** the earlier "22.1 mm / 13 % camera range short" is inside
  this slack and must not be cited as geometry.
- **Gate consequence:** Block 5b's median ≤ 2.0 px ≈ **2 mm** at 175 mm range,
  while tip repeatability is **±16–20 mm**. The 2 px gate is an order of
  magnitude below what the hardware can promise from bridge-reported joints.
  Any replacement gate must be set from measured repeatability (3–5× looser),
  or be a camera-**orientation** test, which the deadband does not move.

## 6. Live solver defect — unruled

`TIP_IN_CHAIN` is **104.425 mm**; CAD and tape both say **112 mm**. That is
**7.6 mm short of the physical tip**, so every live Cartesian move believes the
fingertip is 7.6 mm higher than it is. No calibration file changed pending a
ruling.

## 7. Sim vs bridge joint ranges — unruled

The sim declares narrower ranges than the arm can reach (e.g. `shoulder_lift`
±100 vs bridge −107.05/+108.64; `wrist_flex` ±95 vs −104.59/+98.79). Even with
perfect geometry the sim would refuse reachable poses and mis-screen reach.

## 8. Design rules that follow (binding)

1. **The sim plans; the camera closes.** Planning, IK seeds and clearance checks
   live in the sim. Final positioning is vision-closed — no joint-space plan can
   absorb a load-dependent 20 mm sag.
2. **One direction, never reverse.** Fine-approach moves descend only; a
   reversal pays the full deadband twice.
3. **Never trust a calculated contact.** Contact is verified with the wrist
   camera, not inferred from a commanded position.
4. **Vision budget:** at 175 mm and fx 832, **1 px ≈ 0.21 mm** — the camera
   resolves ~100× finer than the deadband. That is the closing mechanism.
5. **Scripted-move floor:** sim-check every leg; refuse any whose minimum
   fingertip height over the board plane is below ~25 mm.
6. **Stale joint reads are dangerous:** a `/state` with `joints_age_s` in the
   thousands looks plausible and is ~40 minutes old. Always check the age.
7. **No model vision.** Images are for Brian; every conclusion must be a printed
   number.

---

# Brian's rulings — 2026-11-20

1. **Tip constant: use the real CAD measurement.** Verified already in force:
   `live/tip_frame.py` resolves `TIP_IN_CHAIN = -110.45 mm` from
   `live/tool_tip.json` (pack `cad_parallel`, stamp `31483e2`, `ok: true`,
   `cross_check_delta_m = 0.0`). The 104.425 figure quoted in an earlier report
   was the **stock** SO-101 literal printed in that module's docstring, not the
   resolved value. **No code change was needed and the claimed 7.6 mm live
   grasp-height error does not exist.** Correction recorded so the error is not
   re-chased.

2. **Joint limits: use the real limits with a 3° soft margin.**
   Applied to `chess_sim/assets/so101/so101.xml` (the MJCF `build_scene()`
   actually loads — note `assets/scene/chess_so101.xml` is an unused export and
   was left untouched). Both `range=` and `ctrlrange=` updated; the jaw keeps its
   full 0–100 command range because it is an aperture, not a swing joint.

   | joint | bridge limit | sim range now (3° inside) |
   |---|---|---|
   | shoulder_pan | −108.99 … +121.38 | **−105.99 … +118.38** |
   | shoulder_lift | −107.05 … +108.64 | **−104.05 … +105.64** |
   | elbow_flex | −96.68 … +94.75 | **−93.68 … +91.75** |
   | wrist_flex | −104.59 … +98.79 | **−101.59 … +95.79** |
   | wrist_roll | −81.82 … +91.05 | **−78.82 … +88.05** |

   Two consequences worth stating: the sim can now command the reach the real arm
   has (lift and flex were up to 9.6°/8.8° narrower), and `wrist_roll` is now
   **narrower** than it was (−157/+163 → −79/+88), which removes a class of
   sim-only poses the arm cannot reach.

3. **The picking loop replaces the 2 px pixel-parity gate.** The 2 px threshold
   (~2 mm at 175 mm) is an order of magnitude below the arm's ±16–20 mm
   joint-derived repeatability, so it was measuring servo slack, not the camera.
   Parity is judged instead by the picking loop: pre-grasp from the observed
   pose, one-directional vision-closed approach, camera-verified grasp. A
   camera-orientation/scale check (which the deadband does not move) may be kept
   as a companion, but not as a positional gate on bridge-reported joints.

---

# Piece-first camera calibration (Brian, 2026-11-20 15:14)

**Why this exists.** Auto-exposure optimises for the brightest thing in frame — the
printed ChArUco board and its white squares — so a black piece on a dark square is
crushed to the same level as the print and the detector sees a fragment (measured:
**94 mm² against a 150 mm² gate**, while the same scene at a brighter moment gave
736–967 mm²). Marker registration was never the problem: PnP held at **1.0–1.5 px
RMS** throughout. The camera must be tuned for *pieces*, not for markers.

## The pipeline (add as a block before any picking run)

1. **Fix the frame first.** Before tuning, confirm frame stability: 6 consecutive
   frames with frame-mean spread < 0.5 levels and a constant marker count. Unstable
   frames invalidate every tuning result — that is what produced today's earlier
   contradictory sweeps (means 83.6 / 40.9 / 39.6, markers 8 / 4 / 2). Root cause
   found and fixed: the bridge closed the camera when idle, and every re-open
   restarted auto-exposure **and** re-applied an env fallback `contrast=64,
   sharpness=64`. The applied set now persists in `camera_controls.json` and is
   re-applied in `_open()`.
2. **Put the hardest case in view.** A dark piece on a dark square, not a pawn on a
   white square and never the bare board. That is the worst case the loop will meet.
3. **Measure separation, not brightness.** For the piece's square and for a bare
   square of the same parity, report: detected component area (mm²), the square's
   NCC against the printed-face reference, and the p5/p95 spread inside the square.
   The pass condition is the *piece* metric, not how the image looks.
4. **Sweep with the metric.** Exposure (manual), gain, brightness and contrast, one
   at a time, with everything else pinned. Manual exposure and fixed gain are
   mandatory — auto-exposure is the thing that destroys dark-on-dark contrast.
5. **Lock and persist.** Save the winning set with **Save as startup** so it survives
   the camera closing and re-opening; re-read the device to confirm it took.
6. **Gate the result:** 5 consecutive frames, piece area within ±10 %, `status =
   target` on all five, and marker PnP RMS < 2 px. Record the winning values and the
   five frames in a receipt.
7. **Re-run the sweep** whenever the lens, the lighting, the board or the exposure
   mode changes — and always after a board rotation.

## Board orientation — the arm always plays black

The arm plays **black**. The board was found rotated so that black was not on the
arm's side, which invalidates both the square naming and the target coordinates for
the current fit.

- Before any picking run, confirm the black side faces the arm.
- **Any physical board rotation invalidates `live/board.json`.** Its `x_mm`,
  `y_mm` and `yaw_deg` were fitted to one orientation; after a rotation every
  reported square is wrong by that rotation. Re-fit the board pose, then verify
  with a known piece: state its square, and the detector must name the same one.
- Treat a mismatch between a stated square and the detected one as a **pose
  problem first**, not a detector problem — today it read `c6/d5` at
  (−26.8, 25.3) mm for a queen Brian states is on `e5`, a 42 mm disagreement that
  is consistent with a stale board frame.

---

# Board registration findings — 2026-11-20 (evening)

**1. `board.json` does not affect the displayed square.** The overlay is built from
the calibration + the printed-face reference only (`live/app.py:217`); `board.json`
is consumed for **arm motion**, not for naming. So a `board.json` re-fit can never
change a piece's label — that hypothesis was wrong and is recorded here so it is
not retried.

**2. A `board.json` re-fit from this view does not fit.** Eight frames, camera held
fixed (joints + calibrated chain), only the 16–32 detected marker corners as
evidence: **fit RMS 8.4–9.6 px** against the same corners the marker solve fits at
2.3 px, and it wants to move the board surface **76 mm** (z 392 instead of 431).
That is not a board offset — it is the joint-derived camera and the marker geometry
not being in one convention. **Not applied.** The file is byte-identical.

**3. The real fault is registration coverage, not naming or the board pose.** With
only **2–4 markers clustered in the b1–d2 corner**, the far side of the board is
undetermined and the frame is free to rotate — which is exactly a one-square error
that wanders between frames (observed: d4, c5, d5, c6, a4, b5, a1). The corroborating
number: **`light_ncc_min` 0.00–0.03** — the rectified frame does not match the printed
reference anywhere away from the marker corner. So the homography extrapolates badly,
and every downstream square inherits the error.

**Consequence for the picking loop:** it cannot start until a frame resolves the
board across its full extent. Three routes, in order of cost:

1. **Nothing moves:** make registration robust to clustered markers — use **all**
   detected corners with RANSAC rather than 3–4 corner markers, and re-check
   `light_ncc_min` across the whole board as the acceptance number (target: > 0.5
   everywhere, not 0.00–0.03 near one corner).
2. **Pose the wrist over the board centre** so 6+ markers spread across the board
   are visible — the acceptance test is the same `light_ncc_min` figure plus PnP
   RMS < 2 px.
3. **`POST /sanity`**, which visits a1/h1/a8/h8 and reports landing error.

Only route 1 needs no arm motion, and it is the one that makes routes 2 and 3
meaningful — a better view of a bad homography is still a bad homography.

**Process note.** Three separate hypotheses were raised and withdrawn this evening
(stale board yaw, parallax, `board.json` position). Each was killed by a measured
number, which is the correct outcome — but the pattern is worth naming: when a
square is off by ~1, check *how much of the board the registration actually sees*
before touching any frame constant.
