# Grasp attempt 2 — result (2026-11-22)

**Verdict: FAILED (no pickup).** Piece remained on e5.

## Change from attempt 1

Attempt 1 closed the jaws to value 30 (gap 11.4 mm) over a pawn ~9 mm wide —
the jaws never clamped. Attempt 2 closed to jaw 18 (gap 6.8 mm) and descended
to tip z=14 mm (the pawn's measured grasp height 14.76 mm).

## Execution

All legs dry-ran OK; no 409, no fault, roll held (67.96→67.87→67.96), bridge
restored to HOLD. 33 video frames recorded. Video/GIF: `reports/grasp2.gif`.

## Root cause now identified

The tip-target readback is accurate: commanded tip (14,14,16) landed at
(13.4,15.8,17.5) board mm (perr 1.6 mm). So FK tip targeting is fine.

The problem is **where the piece actually sits**. In the close/lift frames the
commanded aim crosshair (e5 centre, board mm 14,14) lands on the **right edge**
of the white piece — the piece's base is offset several mm to the left of the
square centre. Commanding the tip to the square centre therefore puts the jaws
beside the piece, not around it. This is consistent with the camera-to-jaw-tip
parallax measured separately (image centre ≈ 10 mm x, 24 mm y from the jaw tip).

## What is established

- Tip targeting via `/move xyz` is accurate to ~1–3 mm in board coordinates.
- Jaw gap at value 18 (6.8 mm) is appropriate for a 9 mm pawn base; jaw 30 was
  too wide.
- The remaining error is **piece localization**: aiming at the nominal square
  centre is not good enough; the real piece sits offset within the square.
- The mount-free detector goes blind during close (1–2 markers as the jaws and
  piece occlude the face), so it cannot correct the aim at the last moment.

## Next (attempt 3)

1. Locate the piece's actual base in board mm from a clean frame with the jaws
   lifted and ≥3 markers (measure the deviation blob centre, not the square
   centre).
2. Command the tip to that measured base, not to (14,14).
3. Descend to z≈14 mm and close to jaw 18; verify the blob is centred between
   the fingers before closing.

No grasp claim made. Receipt: `reports/grasp_marked.json` (from this run).
