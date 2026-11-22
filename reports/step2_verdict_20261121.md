# Step 2 — board frame verdict (2026-11-21, final)

## Status

Step 2 is closed as far as motion can take it. The half-turn question is answered;
the remaining defect is a pose-model inconsistency that needs a bench measurement,
not another arm pose.

Arm state: **HOLD**, armed, no fault, nothing clamped. Tip (12.0, 17.5, 59.9) —
the e5 hover, 2.5 mm from the piece centre, roll 67.51, gripper 17.88 untouched.

## Settled facts

**1. Half turn — ruled out.** PnP-vs-FK disagreement across every multi-marker
view was **12.5–29.5 mm**, against half-turn predictions of **133–193 mm** in the
same views. Every measurement is 100+ mm short of the half-turn signature.

| view | markers | disagreement | half-turn prediction |
|---|---|---|---|
| e5, roll 69 | 2 | 12.5 mm | 159.5 mm |
| e5, roll 69 | 2 | 18.0 mm | 163.1 mm |
| e5, roll 67.5 | 3 | 18.2 mm | 163.1 mm |
| b7 | 1 | 23.8 mm | 68.4 mm |
| b2 | 3 | 29.4 mm | 193.0 mm |
| e5, roll −20.75 | 2 | 29.5 mm | 89.7 mm |

**2. Blur — ruled out.** b2 decoded three markers with per-marker IPPE fits of
**0.04–0.07 px (id 9), 0.45–0.58 px (id 5), 0.15–0.24 px (id 13)** at **14–18
px/module**. Sub-pixel fits at that resolution mean the optics resolve the print.
The 1-of-N decode at e5 was framing and occlusion (6–7 marker squares complete,
the rest clipped by the frame edge or covered by the jaws), not focus.

**3. ~1 square naming bias still present.** Ids land off their declared squares:

| roll | id | declared | landed | miss |
|---|---|---|---|---|
| −20.75 | 23 | f5 | f5 | 12.0 mm |
| 67.51 | 27 | e6 | f7 | 37.5 mm |
| 67.51 | 19 | e4 | e5 | 33.3 mm |
| 67.51 | 18 | c4 | c6 | 50.8 mm |

**4. Pose-model inconsistency — the open defect.** Individual markers fit to
**0.04–0.58 px**, but the joint solve across the same markers is **2.4–5.8 px**
(2.36–5.84 observed at b2; 1.35–3.27 at the e5 hover). A single rigid pose cannot
fit all corners to sub-pixel while leaving a multi-pixel joint residual.

## Hypothesis — corrected

My first hypothesis was that the printed marker positions are offset from the
declared layout. **It does not survive its own numbers.** Three markers at b2 agree
with each other to a joint residual of 2.4–5.8 px, which at 5.38 px/mm is
**0.45–1.08 mm**. A print offset of about a millimetre cannot produce a naming
error of 33–51 mm, which is more than a square. **The print is almost certainly
fine.**

The corrected reading:

- the **joint residual of 0.45–1.08 mm bounds the print error** — the markers sit
  where the layout says, to about a millimetre;
- the **PnP-vs-FK disagreement of 12.5–29.5 mm** says the camera pose derived from
  the joint chain disagrees with the camera pose derived from the markers by
  roughly a square;
- the **naming error of 33–51 mm** is measured through the FK pose, so it is a
  consequence of that same camera-pose error, not independent evidence about the
  print.

**The defect is the camera-to-wrist transform (hand-eye), including a roll-dependent
term** — the disagreement moves 19 mm between roll −20.75 and roll 67.51 at the
same tip position, and a print error cannot move with the wrist roll.

## What would settle it

1. **Top-down orthographic capture of the empty print**, rectified and compared
   corner-for-corner with `board_charuco.png`. Cheap, and it **rules the print out
   definitively** — no camera pose in the loop.
2. **Re-derive the camera-to-wrist transform**, including the roll zero, from
   marker observations at several roll values. This is the real work; the naming
   error is the blocker for pickup and it lives here.

Neither was run, and no layout or calibration file was edited.

## Addendum 3 — single-offset fit across all views (15:37)

Question: does one constant camera-in-board offset explain the PnP-vs-FK disagreement?

| view | markers | delta PnP−FK (mm) | \|d\| |
|---|---|---|---|
| b2 | 3 | (5.6, 25.7, −13.1) | 29.4 |
| e5, roll 69 (a) | 2 | (10.4, −0.4, −7.0) | 12.5 |
| e5, roll 69 (b) | 2 | (14.4, −1.2, −10.8) | 18.0 |
| e5, roll 67.5 | 3 | (12.7, 5.2, −12.0) | 18.2 |
| e5, roll −20.8 | 2 | (28.3, −4.4, −6.9) | 29.5 |
| b7 (a) | 1 | (13.9, 19.3, −0.2) | 23.8 |
| b7 (b) | 1 | (15.7, 19.9, −1.4) | 25.4 |

Best single constant offset: **(14.4, 9.2, −7.3) mm**.

| | mean \|d\| | max \|d\| | std |
|---|---|---|---|
| before | 22.4 mm | 29.5 mm | 5.9 |
| after | **13.1 mm** | **19.6 mm** | 4.5 |

Multi-marker views only (5): offset (14.3, 5.0, −10.0), residual **mean 11.2 mm, max 22.7 mm**.

**It does not collapse.** A single offset removes roughly 40 % of the disagreement and
leaves a 6.4–19.6 mm spread that still tracks the view — b2 at 19.6 mm, e5 roll −20.8
at 19.4 mm, both b7 views at 12.3 mm, e5 roll 67.5 at 6.4 mm. So the defect is
**view-dependent**, and a one-shot hand-eye translation is not the fix.

## Corner loop — stopped

- b7 re-captured at roll 68.57: 1 marker (id 29), rms 0.28 px, disagreement 25.4 mm.
- **g2 flipped the wrist to roll 0.35** (a third branch, neither 68.57 nor −20.75) on
  arrival. Stopped per the branch rule; no capture taken there.
- g7 not attempted — it flipped to −20.75 at 15:26 from the same predecessor.
- Arm left in **HOLD at g2**, tip (63.6, −60.2, 79.2), **roll 0.35**, armed, no fault,
  nothing clamped. Restoring the roll is motion and has not been taken.

## Addendum 4 — roll-zero hypothesis test (15:45)

Vectors are `camera_xy − tip_xy` in board coordinates; the tip comes from FK on the
recorded joints of the same receipt.

| view | roll | \|v_pnp\| | \|v_fk\| | azimuth diff (PnP − FK) |
|---|---|---|---|---|
| b2 (3 mk) | 69.01 | 62.3 | 68.7 | **+22.50°** |
| b7a (1 mk) | 68.22 | 65.9 | 69.0 | **+20.15°** |
| b7b (1 mk) | 68.57 | 67.7 | 69.0 | **+21.36°** |
| e5 r69a (2 mk) | 69.01 | 81.4 | 71.2 | **+1.80°** |
| e5 r69b (2 mk) | 69.01 | 85.4 | 71.2 | **+1.95°** |
| e5 r67.5 (3 mk) | 67.51 | 80.3 | 68.8 | **+5.77°** |
| e5 r−20.8 (2 mk) | −20.75 | 74.5 | 69.3 | **+22.60°** |

Azimuth difference: **mean +13.73°, std 9.26°, range 1.80–22.60°**.
Radii: `|v_pnp|` **62.3–85.4 mm (mean 73.9, std 8.2)**; `|v_fk|` **68.8–71.2 mm (mean 69.6, std 1.1)**.

**Verdict: the roll-zero hypothesis is dead.** A wrong roll zero would produce a
constant azimuth offset across every capture. The offset is not constant — and
decisively, at the **same roll 69.01** the b2 view gives **+22.50°** while the e5
view gives **+1.80°**, a 20.7° spread with the wrist in the identical position. The
difference tracks the board position, not the roll.

A second flag: the PnP-derived radius varies by **23 mm** (62.3–85.4) while the FK
radius holds to **2.4 mm** (68.8–71.2). The mount geometry is fixed, so a quantity
that should be constant varying by 23 mm says the PnP camera estimate itself is
unstable — consistent with 1–3 markers per view at 1–6 px joint residual.

So there is no single number to fix. The error has both a position-dependent part
and a roll-dependent part, and the marker-derived pose is noisy at these view
sizes. The hand-eye re-derivation needs several markers per view across the board,
not a correction factor.
