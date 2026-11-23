# First IRL grasp attempt — result (2026-11-22)

**Verdict: FAILED (no pickup).** All commanded steps executed cleanly, but the
piece remained on e5.

## What ran

`tools/p4_grasp_attempt.py`, gripper actuation explicitly authorized by Brian.
Every leg dry-run first (all OK), then executed:

| step | command | result |
|---|---|---|
| open jaws | jaw 100 | gap fully open (visually confirmed) |
| hover | xyz tip (14,14,45) | perr 3.6mm, roll 67.95 |
| descend | xyz tip (14,14,16) | perr 1.4mm |
| close jaws | jaw 30 | executed (final 30.6) |
| lift | xyz tip (14,14,70) | perr 2.4mm |

No 409, no fault, roll held (68.04→67.95→68.04). Bridge restored to HOLD at the
end. Gripper back near 31. Video: `reports/grasp_attempt.mp4` (only 6 frames / 1.2 s
— the recorder pulled ~1 frame/s from the MJPEG stream, so it under-samples the
motion; still shows the sequence).

## Why it failed (from frames + geometry)

- The tip was aimed at the **square centre** (14,14) board mm. The piece's actual
  base is offset from that centre — our earlier mount-free detection put the
  standing centroid around (12.5,12.2) with a projection bias, and the jaw tip
  targets the *square centre*, not the measured piece base.
- At the close frame the piece is beside/below the finger tips, not between them;
  after lift the piece is visibly still on e5.
- Jaw value 30 (from 100 open) may not have closed far enough / at the right
  height for a pawn. The pad band and tip clearance mean the grasp z=16 mm is
  also uncertain for this real piece.

## What is established

- Gripper direction confirmed on hardware: **0 = closed, 100 = open**.
- FK-based tip targeting (`/move xyz`) reaches e5 with ~1–4 mm residual and holds
  the roll branch — this path does not use the suspect camera mount, so the
  earlier "1–2 square naming error" does not block aiming the tip.
- The closed-loop gaze servo and tip-target approach are safe and repeatable.

## Next attempt needs

1. Localize the **piece base** in board mm with the mount-free PnP detector
   (not the square centre, not the standing centroid), and aim the tip there.
2. Determine the correct grasp height and close value for a real pawn — likely
   descend lower and close further (jaw ~10–20), verified by a dry run.
3. Improve video capture (the frame recorder under-sampled); use a dedicated
   stream dump or higher-rate grab.

No grasp claim made. Receipt: `reports/grasp_attempt_14.0_14.0.json`.
