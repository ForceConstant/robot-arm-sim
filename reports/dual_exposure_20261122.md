# Dual-exposure hypothesis (Brian, 2026-11-22)

**Question:** piece observation and marker decoding want conflicting tonal ranges,
so should the wrist camera have a second auto-exposure dedicated to markers?

## Offline test on the 21 existing captures

Applied image-side tone transforms (approximating camera exposure/tone outcomes)
and re-ran the SAME ArUco detector. Tools: `tools/p4_tone_offline.py`,
`tools/p4_tone_validate.py`. No arm motion.

Total marker sightings across all 21 frames:

| tone | sightings | vs baseline |
|---|---|---|
| baseline (705/64, b160, c32) | 40 | — |
| gamma 1.6 (darken highlights) | 41 | +1 |
| CLAHE local contrast | 41 | +1 |
| dark + CLAHE | 41 | +1 |
| gamma 0.6 / 2.2 / stretch | 40 | 0 |

Frames where a tone added a decode:
- marker15, marker26 (0-marker frames): CLAHE/dark-CLAHE recovers id 32 / 17,
  but **no baseline markers in those frames to validate against** — could be
  false positives.
- marker8 (3→4): new id 15 reprojects to **1.40 px** against the pose fit from
  the 3 baseline markers — genuine.
- marker13 (5→6): new id 10 reprojects to **1.15 px** against the 5 baseline
  markers — genuine.

## Interpretation

- A marker-tuned tone adds at most **+1 sighting per frame**, and only **+1
  overall** on frames where the result can be verified. The two 0-marker frames
  that "gain" a marker cannot be validated (no baseline markers to fit a pose),
  so they do not establish a real gain.
- The earlier pixel analysis found the 0-marker frames are **over-bright** at the
  current settled exposure (mean luma 139-164 vs 114 on the 5-marker frame;
  marker count vs luma r = -0.56). This is consistent with a *darker* marker
  exposure helping those specific board-edge poses.
- But the dominant limit is still **field of view**: at 160 mm standoff the
  camera sees ~123 x 92 mm (~4.4 x 3.3 squares), so most poses simply do not
  contain enough markers regardless of tone. Tone can recover a marginal decode;
  it cannot create markers outside the frame.

## Verdict

Brian's premise is sound — piece-optimized and marker-optimized exposure are in
conflict, and a second profile is the right structural answer for a system that
must do both jobs. But for the **hand-eye calibration bottleneck specifically**,
the offline data shows a marker-only exposure would add ~1 sighting per frame at
best, not the jump needed to reach the ~1 px solve bar.

A decisive on-rig test is still cheap and worth doing before ruling it out:
re-shoot the 0-marker board-edge poses (marker24/15/23/26) at a darker marker
tone (exposure ~400-500, gain 64, brightness 96-128) and see whether they decode
2+ markers with sub-pixel reprojection. That requires arm motion, which is
standing-authorised only while the Pi bridge is reachable — confirm before run.

No transform published; no config changed.
