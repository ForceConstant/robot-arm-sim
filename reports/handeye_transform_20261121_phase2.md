# Camera-to-wrist transform — second attempt (2026-11-21, 20:05–21:00)

## Outcome

**Detection improved 12 → 40 marker sightings; still no qualified transform.**
The scan plan is updated (17 poses, two wrist-roll branches). The hand-eye solve
now has 35 sightings across 14 views on both branches and still fails the
acceptance bar: **reprojection RMS 16.4 px** (the procedure accepts ~1 px).

## What changed, and the measurements behind it

### 1. The jaw is not a detection lever (measured)
Same pose, jaw commanded 0 / 25 / 50 / 75 / 100: **the same markers decoded at
every setting**. The fingers do not occlude the decodes. Jaw authority is now
standing, but it does not buy markers.

### 2. The first pass's exposure was the real problem
The first pass settled exposure at pose 1 (board edge) to **2600/96** and used it
for all 13 poses; at centre-board poses that blew the frame (clip 9–14 %). Re-run
with **705/64, brightness 160, contrast 32**: same 13 poses gave

| pass | sightings | views with a solve | views with 0 | views with ≥3 |
|---|---|---|---|---|
| 1 (2600/96) | 12 | 8 | 5 | 0 |
| 2 (705/64, b160) | **22** | 10 | 3 | 4 |

A tone sweep at a fixed pose (brightness 96–222) decoded **4 markers at every
setting**, so brightness alone is not the limiter either — the settled *exposure*
was.

### 3. The standoff cannot be increased (reach limit)
Plan candidates at 0.26 / 0.32 m above the board came back **55–112 mm short**;
the arm tops out at a camera height of about **0.165 m** above the board (the
current plan uses 0.156 m). A larger standoff — the fix I proposed last time —
is not available on this arm.

### 4. Second roll branch captured
The 8 poses that decoded markers were re-run with **wrist roll + 46.3°** (21.7 →
68.0). All 8 reached, roll held 67.6–68.04, and **18 markers decoded over 8 views**
(one empty). Combined set: **40 sightings, 18 PnP views, rolls 21.7 and 68.0**.

## The updated plan

`chess_sim/assets/marker_scan_plan_charuco.json` (both trees), 17 poses:

- 10 poses at roll 21.7 that decoded markers on the rig,
- 7 poses at roll 68.0 (same targets, +46.3° roll),
- `marker24`, `marker15`, `marker23` **dropped** — zero markers at either roll,
- `visible` / `edge_px` now come from the **real captures**, not the simulator.

The dashboard was restarted so `robot-live.service` serves the new plan.

## Solve, per branch (combined set)

Staged + bootstrap, measured lens, 244 mm board:

| | value |
|---|---|
| views / sightings used | 14 / 35 |
| reprojection RMS | **16.43 px** |
| mount change vs stored | 21.40 mm, **164.2°** |
| board solved | (−29.2, −27.2, 418.6) yaw −175.2° vs measured (−34.9, −18.1, 431.4) yaw 181.9° |
| held-out (leave-one-out) | mean 7.12 mm, max 13.06 mm |

Cross-view board-pose deviation (in-sample): before 95.2 mm → after 7.4 mm;
branch 21.7 mean 8.04 mm, branch 68.0 mean 6.72 mm. In-sample only — 17 views
fitting 12 unknowns.

## Why it still is not a transform

- **RMS 16.4 px** against a ~1 px bar. One to five markers per view cannot pin a
  6-DoF mount plus a 6-DoF board; the single-marker PnP carries a mirror
  ambiguity.
- The mount rotation is still returned as a **164° change** — an unconstrained
  direction, not a measurement. Adding the second branch made the fit *worse*
  (12.1 → 16.4 px), which is consistent with the residual being more than one
  rigid mount, but with this sighting density it cannot separate a roll-dependent
  mount term from PnP noise.
- The board pose is **19 mm off** the measured placement, where the procedure
  accepts a couple of millimetres.

## What still needs Brian

1. **More markers per view is the only remaining lever.** The best single view in
   the whole set is 5 markers (marker13 at roll 68). Two options:
   - seed the mount from the 5-marker view (`--init-from-records`), re-aim the
     plan with that mount, and capture again — the documented "iterate once" step;
   - or accept visual servoing (aim, detect, nudge, capture) for the capture pass.
   Both are a scope call: the first re-aims the plan again, the second adds a new
   capture behaviour.
2. **Console sequence bug (found, not fixed):** after a scan, the live console's
   `/move` returns 409 (`seq ... is not newer than ...`) because its command
   counter starts at process start and my scan advanced the bridge past it. A
   dashboard restart clears it. Worth re-seeding `command_seq` from the bridge
   state.

No layout, board, print, lens, saved camera profile or calibration was edited.
Arm left in **HOLD**, armed, no fault, at the last roll-68 pose, roll 67.96.
