
## Interpretation

1. **Aiming at the ray roughly halves the aim-point error.** Same session, same
   board, same height: tip-commanded p50 **9.59 mm** (p95 14.74, max 18.29, n 61)
   versus aim_point p50 **5.46 mm** (p95 11.93, max 12.48, n 60). The share of
   squares inside 5 mm doubles, 12/61 -> 26/60; inside 10 mm 34/61 -> 43/60.
   Where the loop closes (the 17 squares that needed no refinement) it lands at
   p50 **3.55 mm**, i.e. inside the 3-5 mm design tolerance.
2. **It wins where it can actuate, and stalls where it cannot.** The first
   aim_point step was p50 10.2 mm and moved the aim p50 0.72 mm; on 23 of 51
   squares it moved 0.5 mm or less. That is the servos' 3-4 deg directional
   deadband: a 10 mm correction at this reach is ~2.9 deg of joint travel, below
   the threshold at which the arm moves at all. The paired median improvement is
   only 0.19 mm while the mean is 2.24 mm - the distribution is bimodal (big
   wins where the correction clears the deadband, nothing elsewhere), not a
   uniform gain.
3. **The remaining absolute error is not the aim mode - it is the board frame.**
   Projecting each square centre through the marker solve and through the
   console's configured frame (same image, same instant) puts them a median
   **84 px** apart, up to 191 px. At this 60-130 mm viewing range a ~10 mm board
   frame / FK translation projects exactly like that: perspective amplifies it
   across the board, so a constant error looks like a gradient. The aim mode
   moves the ray to the *commanded* coordinate, so it improves in board mm but
   not in the image; on the 37 squares where both columns have a marker grid it
   is 16 px worse (56.5 vs 40.2 px) because the tip mode falls part-way back
   toward the true board as the arm lands short in the direction of the frame
   error. This is the same ~8.6 mm residual the phase-3 alternating solve named
   and deliberately left unapplied (reports/phase3_constant_20261124.md 5, 7).
4. **Consequence for grasp targeting.** Commanding the ray is the right
   primitive and is measurably better in the frame the console commands in, but
   it cannot put the jaws on a visible square centre until the board pose (and
   the FK/hand-eye residual behind it) is corrected. Applying the phase-3 board
   delta (+7.3, -13.5 mm, +2.4 deg yaw) is the next worthwhile step; it is worth
   more than any further aim-loop tuning.
5. **Servo-level floor.** Sub-5 mm centring needs corrections the servos do not
   execute. The deadband-compensated rel refinement (command residual + estimated
   shortfall) recovered 18 squares, degraded 11 and was refused on the six
   rank-1 squares already at the reach limit, so it is reported as an experiment
   rather than as the protocol. A dither / approach-from-one-side move primitive
   in the console is the clean fix.

## Failures, with reasons

- h1 tip: `no reachable pose: off by 0.27 mm / 8.742 deg` (approach tilt limit);
  aim skipped because the arm stayed at the recovery pose.
- c8, d8 tip: `no reachable pose: off by 6.65 / 6.07 mm / 6.17 / 6.38 deg`;
  aim skipped for the same reason.
- b8: `no usable board plane` (marker-poor view; the arm was not commanded).
- a1-f1, h2 refinement: `no reachable pose: off by 5.9-7.8 mm / ~0.5 deg` - the
  correction solve cannot translate further at the near edge with roll frozen.
