# Camera-to-wrist transform — re-derivation attempt (2026-11-21, 17:20–17:45)

## Outcome

**No transform is published.** The qualified `/scan` plan executed cleanly, but the
data it produced cannot determine the mount: it is a single wrist-roll branch with
1–2 markers per view. The best solve leaves the PnP-vs-FK disagreement at
**22.4 mm mean** (from 27.0 mm) and returns a **158.6° rotation change** on an axis
this data cannot observe. That is not a correction; it is an unconstrained fit.

## What was run

- Board check first: the rectified wrist view of the tip neighbourhood (c2–f6) was
  empty printed ChArUco; the detector rejected the only large component
  (12,421 mm²) as too big to be a piece. The console overlay had briefly named a
  1,080 mm² dark component at a6, outside that view; at the scan poses the
  detector returned no target, and no fault or stall occurred on the passes over
  the a-file, so that component is bench/edge rather than a board piece.
  **e5 is clear.**
- `/scan` plan `marker_scan_plan_charuco.json`, 13 poses, run through the
  qualified `ScanController._goto` path. **13/13 poses reached**, max arrival
  error 1.19°, roll held at **21.63–21.98°** (spread 0.35°).
- Gripper **not actuated**: held at 17.97 in every command (plan asks for 0.0).
- Exposure settled at pose 1 by numbers: 1000→3400 ISO-less steps at gain 64,
  plus 2600/96; clip 0.00% at every setting; **settled 2600/96** (markers 1).
  Restored to 705/64 afterwards.
- 13 captures saved to `live/captures_handeye_20261121/` (jpg + json with
  joints, drift 0.00°, roll, exposure). Receipt:
  `reports/handeye_capture_20261121.json`.

## The data

| | |
|---|---|
| views captured | 13 |
| views with a PnP solve | 8 |
| marker sightings | **12** |
| views with ≥3 markers | **0** |
| views with 2 markers | 4 |
| views with 1 marker | 4 |
| views with 0 markers | 5 |

Five poses (marker26, marker25, marker24, marker15, marker23) framed the board
edge and table — pose 2 shows the printed border labels and the bench, not board
interior. At a ~160 mm standoff the frame covers ~5 squares, so the ~27 mm mount
error (≈1 square) is enough to push the intended markers out of frame.

## Solve attempt (documented routine, `tools/handeye_bundle.py`, staged + bootstrap)

- 7 views / 11 sightings used; **reprojection RMS 12.5 px** (acceptance ~1 px).
- Mount: stored `[67.7, -2.9, -10.9] mm` → solved `[59.9, 7.4, -21.0] mm`
  (16.4 mm translation, **158.6° rotation** about ≈ tool −x).
- Board: solved `(-16.0, -10.7, 420.6)` yaw −178.1°, against the measured
  placement `(-34.9, -18.1, 431.4)` yaw 181.9° — **19 mm off in x**, where the
  procedure accepts a couple of millimetres.

## Residuals, before → after, per roll branch

PnP-vs-FK camera disagreement (the Step 2 metric), same 8 views:

| branch | n | before mean | before max | after mean | after max |
|---|---|---|---|---|---|
| 21.70 | 7 | 27.31 mm | 34.07 mm | 22.84 mm | 26.21 mm |
| 22.00 | 1 | 24.61 mm | 24.61 mm | 19.32 mm | 19.32 mm |
| all | 8 | **26.97 mm** | 34.07 mm | **22.40 mm** | 26.21 mm |

Azimuth difference PnP−FK: **+17.93° mean, std 5.51** before → **+10.57° mean,
std 5.49** after. The spread does not move.

A cross-view board-pose fit does look better in-sample (62.7 → 4.7 mm mean), but
that is 8 views fitting 12 unknowns; the held-out PnP-vs-FK number above is the
honest one and it does not collapse.

## Why the data cannot support a solve

1. **One roll branch.** All 13 plan poses command wrist roll 21.7°; the measured
   roll never left 21.63–21.98. The mount's rotation about the view/roll axis is
   therefore unobservable, and the solver spent that freedom: a 158.6° rotation
   change at 16.4 mm translation is the signature of an unconstrained axis, not a
   measured mount.
2. **1–2 markers per view.** A one-marker PnP is a planar 4-point fit with a
   mirror ambiguity; 11 sightings cannot pin a 6-DoF mount plus a 6-DoF board.
   RMS 12.5 px confirms it.
3. **The aim is already the defect.** The plan's aims were generated from the
   camera model under test, so on the real rig they land ~1 square off — exactly
   the error being measured. Using those poses to fix that error is circular at
   this standoff.

## What still needs Brian

1. **Gripper actuation for the scan.** The plan expects the jaw at 0.0; I held it
   at 17.97 because actuation is not authorised. The fingers are in frame (pose 1
   shows them over the right side) and cost marker decodes. Approving a closed jaw
   for the capture pass may recover several markers per view.
2. **A second roll branch.** Either a capture pass that commands a different roll
   (e.g. ≈68° or −20.75°) or a re-generated plan with 2–3 roll values. Without
   that the roll component stays unobservable no matter how many views are added.
3. **A larger standoff (~250–300 mm).** At 160 mm a 27 mm aim error costs 1–2
   squares and most of the frame; at 280 mm the same error is a small fraction and
   4–8 markers stay in view. That is the procedure's "several standoffs" step, and
   it needs the plan's standoff changed.

No layout, board, print, lens, camera profile, calibration or plan file was edited.
Arm left in **HOLD**, armed, no fault, at the last scan pose
`[-7.91, -59.91, 78.90, 40.84, 21.71, 17.88]`, roll 21.71.
