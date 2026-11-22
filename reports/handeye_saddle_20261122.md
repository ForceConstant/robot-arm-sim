# ChArUco saddle-corner diagnostic (2026-11-22)

**Question:** the external review proposed extracting ChArUco saddle corners
(`cv2.aruco.CharucoDetector`) to multiply geometric constraints per view and bring
the hand-eye solve from ~12–16 px RMS toward ~1 px. Run offline on the 21 existing
captures; no arm motion, no calibration/print change.

## Correction (Brian, 2026-11-22)

An earlier version of this report claimed the print has markers on dark squares.
**That was wrong** — it came from reading a downscaled preview. Pixel samples of
the generated face show marker squares have a white background (value 246) with
a black marker; dark squares are ~26. The print is a standard ChArUco face.

## Finding 1 — CharucoDetector works on a clean render with legacyPattern=True

On OpenCV 4.13, `cv2.aruco.CharucoDetector` with:
- board ids in image row-major order matching our print (`[33,34,35,36, 29..32, ...,
  5,6,7,8]`), and
- `cboard.setLegacyPattern(True)` (the print uses the legacy chessboard polarity)

returns **49/49 saddle corners** on a clean render of our print. With the default
`legacyPattern=False` it returns 0. So the expert's mechanism is valid in
principle, and the id/pattern setup is now understood.

## Finding 2 — It fails on the actual wrist-camera captures

On the 21 real captures (640×480, blurred, 1–5 ArUco markers per frame):

- CharucoDetector finds only **1–3 saddle corners per frame** (vs up to 49 on a
  clean render), and 10 of 21 frames return none at all.
- More damning: for the frames where it does return saddles, projecting the
  known grid vertex from the **ArUco-derived camera pose** puts the detected
  saddle **166–198 px away** from where CharucoDetector placed it. These are
  false detections, not the printed grid vertices.

Tool: `tools/p4_charuco_consistency.py` prints the per-frame check.

Feeding those saddle observations into the staged solve made the fit markedly
worse: ArUco-only **16.73 px** (board z 430.2 mm) vs ArUco+saddle **34.80 px**
(board z 332.4 mm, mount translation 112 mm). The saddle corners are noise.

## Finding 3 — Homography-derived saddle corners add little (earlier attempt)

A substitute was also tried: derive grid-vertex corners from a homography fitted
to the detected markers and refine with `cornerSubPix`. Same solver, 17 views:
ArUco-only 160 points / 18.31 px vs +133 saddle points / 17.05 px. The ~7% gain
is because with 1–4 markers per view the derived vertices are predictions of the
same geometry, not independent observations. Tool:
`tools/handeye_20261121/p4_saddle_solve.py`.

## Conclusion

The expert's mechanism is real but **does not survive contact with the actual
captures**: CharucoDetector's saddle detections on these frames are sparse and
geometrically false (~180 px off). No trustworthy gain over the ArUco-only solve.
**No transform is published.** The bottleneck remains decoded markers per view,
capped by the ~123×92 mm FoV at a 160 mm standoff that arm reach will not let us
enlarge.

## Evidence

- `tools/p4_charuco_consistency.py` — per-frame saddle vs ArUco consistency check.
- `tools/handeye_20261121/p4_saddle_solve.py` — homography/saddle alternative.
- `reports/handeye_saddle_20261122.json` — machine-readable result of the first
  (homography) attempt.
- Probe scripts `p4_charuco_probe*.py`, `p4_charuco_validate*.py`,
  `p4_real_charuco_solve*.py`, `p4_polarity_check.py` were run on the BC-250 only.
