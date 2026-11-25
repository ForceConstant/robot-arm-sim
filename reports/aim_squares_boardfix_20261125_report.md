# Board-frame delta applied and the 64-square aim sweep re-run (2026-11-25)

**Outcome in one line:** the queued Phase-3 board delta is the **right sign and the
right size** — the marker-vs-configured frame error halves, from **17.9 → 9.0 mm p50**
(pixel p50 89.9 → 43.7), landing exactly on the 8.6 mm residual the Phase-3
alternating solve predicted would remain. The **≤ 5 mm target is still not met**
(best leg 29/60), because what is left is the documented board-depth/mount valley
(≈ 9 mm) plus the 3–4° servo deadband, and neither is a board translation.

Session 2026-11-25T06:37 → 07:00. All sim/analysis ran on the BC-250
(`cachyos`, `brian@192.168.1.171:/home/brian/robot-arm-sim`). Nothing committed.
No gripper actuation (`gripper` joint constant at 26.77° in every record). The
Raspberry Pi bridge was up (`/health ok`, camera frame age 0.39 s, mode REMOTE,
armed), so arm motion was covered by the standing approval.

---

## 1. Sign test — three probes, same six squares

`tools/aim_squares.py --squares d4,e4,d5,e5,c4,f5` at z 70 mm, separate `--out`
per probe. Metric: **median |square_uv − square_uv_cfg|** — the same instant's
marker-projected square centre versus the projection through the console's
configured board frame, i.e. directly the error the delta is meant to remove.
Neither projection touches the other's inputs.

| probe | board (x, y, z, yaw) | pairs n | pooled p50 | tip p50 (n) | aim p50 (n) | max |
|---|---|---|---|---|---|---|
| A — as stored | −34.92, −18.13, 431.35, 181.92 | 7 | **68.39 px** | 79.32 (5) | 58.41 (2) | 100.73 |
| B — plus delta (+7.3, −13.5, +1.3 mm, +2.4°) | −27.62, −31.63, 432.65, 184.32 | 8 | **21.96 px** | 20.08 (5) | 23.83 (3) | 32.02 |
| C — inverted delta | −42.22, −4.63, 430.05, 179.52 | 10 | **170.30 px** | 165.38 (6) | 180.46 (4) | 214.67 |

The six squares that produced a pair in **all three** probes (paired, same pose
sequence each time) leave no ambiguity:

| square/leg | A stored | B plus delta | C inverted |
|---|---|---|---|
| c4/tip | 48.4 | **26.2** | 111.3 |
| c4/aim | 48.5 | **24.2** | 111.5 |
| d4/tip | 68.4 | **6.2** | 137.9 |
| d5/tip | 79.3 | **10.0** | 154.2 |
| e4/tip | 92.4 | **20.1** | 179.4 |
| e5/tip | 100.7 | **32.0** | 176.5 |
| **paired p50** | **73.86** | **22.15** | **146.08** |

Every one of the six improved with +delta and worsened with −delta. The metric is
also monotone and roughly linear in the delta, so the stored board was on the
wrong side, not merely mis-scaled. The inverted probe was run as a control even
though it was not required — if the metric had been insensitive it would have
looked the same, and it does not.

**Kept: probe B, board `x −27.62, y −31.63, z 432.65, yaw 184.32`.** Applied via
`POST /board` with `x-bridge-token`, which persists `live/board.json` and applies
to the live kinematics at runtime (no service restart). `GET /board` returned
`{"x_mm":-27.6,"y_mm":-31.6,"z_mm":432.6,"yaw_deg":-175.68}` (same angle,
normalised). Backup: **`live/board.json.bak-20261125-before-boardfix`**
(byte copy of the original, x −34.92, y −18.13, z 431.35, yaw 181.92; also
copied to `reports/board_before_boardfix_20261125.json`).

---

## 2. Full re-run — conditions

`tools/aim_squares.py --out reports/aim_squares_boardfix_20261125.json`, 64
squares, z 70.0 mm, tolerance 4.0 mm, iterations 3, frames in
`reports/aim_squares_frames_boardfix_20261125/`. Same tool, same parameters as
the committed baseline. 64/64 squares processed; 60 aim legs and 61 tip legs
produced a measurement.

Plane discipline was unchanged and held: `/stream` was pulled for the whole run,
every command was preceded by a fresh-solve check, and the one square without a
usable plane (b8) was **not** commanded — `no usable board plane` is recorded.
No square was ever driven on a stale plane.

Integrity checks: `readout_spread_mm` p50 0.0 (max 0.3) over the three-sample
readouts — the arm was stationary when measured; the gripper joint never moved
(26.77° throughout). Marker-solve quality: 73 of 82 views projected at a PnP RMS
≤ 4 px, 9 above it (max 5.44 px, b4/aim and c4/tip). The baseline had 1 of 88 above
4 px (max 4.40 px), so today's solves are slightly noisier; that is a few pixels of
scatter, not the systematic gradient the discrepancy shows.

## 3. Full sweep, before vs after

Identical sweep geometry both days, same lens, same 70 mm tip height.

### Board millimetres (the aim readout against the commanded centre)

| metric | before (2026-11-24) | after (2026-11-25) | change |
|---|---|---|---|
| tip p50 / p95 / max (n 61) | 9.59 / 14.74 / 18.29 | 9.25 / 15.80 / 16.53 | p50 −0.34, p95 +1.06 |
| tip ≤5 mm / ≤10 mm | 12 / 34 | 12 / 32 | — |
| aim_point p50 / p95 / max (n 60) | 5.46 / 11.93 / 12.48 | 5.57 / 11.58 / 11.82 | p50 +0.11, p95 −0.35 |
| aim_point ≤5 mm / ≤10 mm | 26 / 43 | **29** / 45 | **+3** |
| aim_final p50 / p95 / max (n 60) | 7.78 / 11.23 / 12.40 | 7.44 / 11.60 / 12.01 | p50 −0.34 |
| aim_final ≤5 mm / ≤10 mm | 23 / 46 | **26** / 47 | **+3** |

### Pixels against the marker-projected centre (`pixel_delta`, independent of the configured frame)

| metric | before | after |
|---|---|---|
| tip p50 / p95 / max (n 49 → 47) | 48.05 / 106.23 / 113.79 | 56.25 / 101.48 / 115.40 |
| aim p50 / p95 / max (n 39 → 35) | 56.47 / 132.74 / 140.82 | **48.42** / 102.37 / 129.44 |
| paired aim (25 squares both days) | — | median −13.2 px, better 14 / worse 11 |
| paired tip (39 squares both days) | — | median +9.2 px, better 18 / worse 21 |

### Pixels against the configured centre (`pixel_delta_cfg`, chain consistency)

| metric | before | after |
|---|---|---|
| tip p50 (n 61) | 56.90 | 59.82 |
| aim p50 (n 60) | 57.23 | **52.92** |

### The discrepancy itself — the metric the delta targets

| | before | after |
|---|---|---|
| p50 px | 89.94 | **43.67** |
| p95 px | 173.22 | 98.29 |
| max px | 191.44 | 104.04 |
| p50 **mm** (px ÷ local px/mm at that pose, 4.89 px/mm p50) | **17.9** | **9.0** |
| p95 **mm** | 33.3 | 19.0 |
| max **mm** | 35.6 | 21.9 |
| paired p50 px (64 views both days) | 84.58 | 41.55 |

The millimetre row is the one to read: it converts each view's pixel error through
that pose's own scale (28 mm board step projected through the same frame), so it is
the error in the units the arm is commanded in. **17.9 → 9.0 mm p50.**

---

## 4. Failures, verbatim

```
b1 aim/refine: no reachable pose: off by 5.1 mm / 0.44 deg
f1 aim/refine: no reachable pose: off by 6.9 mm / 0.6 deg
g1 aim/refine: no reachable pose: off by 7.3 mm / 0.58 deg
h1 aim/refine: no reachable pose: off by 8.6 mm / 0.65 deg
b8 aim: plane before aim_point: no usable board plane
b8 plane: no usable board plane
c8 tip: no reachable pose: off by 15.77 mm / 3.471 deg
c8 aim: arm is not over the square after the tip leg (tip [4.7, -11.5, 109.3] vs [-42.0, 98.0, 70.0]); skipped
d8 tip: no reachable pose: off by 15.39 mm / 10.427 deg
d8 aim: arm is not over the square after the tip leg (tip [4.7, -11.5, 109.3] vs [-14.0, 98.0, 70.0]); skipped
e8 tip: no reachable pose: off by 14.29 mm / 2.737 deg
e8 aim: arm is not over the square after the tip leg (tip [4.7, -11.5, 109.3] vs [14.0, 98.0, 70.0]); skipped
```

Same failure families as the baseline: the rank-1 near edge cannot make the small
refinement translation with roll frozen (b1, f1, g1, h1 — baseline had a1–f1, h2),
b8 still has a marker-poor view, and rank 8 at the far edge is now refused on three
squares (c8, d8, e8; baseline refused c8 and d8). No new failure mode.

`aim_converged` 23 → 26, `aim_refined` 36 → 39.

## 5. What actually improved, stated honestly

1. **The frame is fixed.** Marker-vs-configured disagreement halved in both pixels
   and millimetres, and the residual (9.0 mm) is the number Phase-3 predicted would
   be left (8.6 mm). That is the specific defect the aim sweep named — "the marker
   centre and the console's configured centre disagree by p50 84 px" — and it is now
   ~44 px.
2. **Aim-mode absolute accuracy improved**: pixel p50 56.5 → 48.4, paired median
   −13.2 px, and the ≤5 mm count rose 26 → 29 (aim_point) / 23 → 26 (aim_final).
3. **The board-mm readouts barely moved** (tip 9.59 → 9.25, aim_point 5.46 → 5.57).
   They are computed in the configured frame against a target in the same frame, so
   a rotation/translation of that frame does not change them — they measure the
   controller's convergence (dominated by deadband), not absolute accuracy.
4. **Tip-mode pixels got worse** (48.1 → 56.3). That is the expected signature of a
   compensating error being removed: the 2026-11-24 report showed tip mode looked
   good in pixels only because the arm fell part-way back toward the true board as
   it landed short in the direction of the frame error. With the frame corrected
   there is nothing left to cancel, so tip mode now shows its real (deadband-limited)
   miss. It did not physically degrade; the earlier number was flattered.

## 6. What remains (measured, not applied)

Read-only diagnostics on the same receipt, to say what the residual is:

* **A free board re-fit to these marker projections** wants
  `dx −5.7, dy +12.5, dz +33.1 mm, dyaw +2.6°` and would take the discrepancy
  p50 to **26.2 px** (translation-only: `dz +41.9 mm`, 25.0 px). A +33…42 mm board
  height is physically implausible — the optimiser is walking down the depth valley
  Phase-3 documented (board z 428.5…434.7 at equal residual). This is not an
  available correction, it is a measure of how shallow the valley is.
* **Mount standoff / focal scans** move the metric by less than 1 px:
  mount z ±6 mm → 42.8–47.8 px (best −7 mm: 41.4 px with y +1), focal scale
  0.94–1.06 → 43.0–48.3 px. So the residual is not a lens or standoff term either.
* The residual field is a smooth gradient across the board (du −35 → +86 px,
  dv +89 → −57 px from file a→h, rank 1→7), with a 10.7 px scatter about that
  gradient. A gradient of that shape is what a small board/perspective height
  mismatch produces at a 60–130 mm hover — exactly the geometry the Phase-3 report
  flagged as biased for the 64-square sweep.

**Conclusion for the next authorised step:** the board delta is spent. The next
millimetres are not in the board pose; they are in (a) a second standoff or a
known-size cue to break the depth valley, and (b) the servo deadband (a
dither / approach-from-one-side primitive), as the 2026-11-24 report already said.
Until then the arm's honest absolute floor at this hover is ~9 mm, not 5 mm.

---

## Files

* Board applied: `live/board.json` → x −27.62, y −31.63, z 432.65, yaw 184.32.
  Backup `live/board.json.bak-20261125-before-boardfix`; original copy in
  `reports/board_before_boardfix_20261125.json`.
* Probes: `reports/aim_squares_probeA_20261125.json` (stored),
  `reports/aim_squares_probeB_20261125.json` (+delta, kept),
  `reports/aim_squares_probeC_20261125.json` (inverted control), logs beside them.
* Full run: `reports/aim_squares_boardfix_20261125.json` (+ `.log`, auto `.md`,
  frames in `reports/aim_squares_frames_boardfix_20261125/`).
* Heatmap: `reports/aim_squares_out/aim_squares_boardfix_heatmap.png`.
* Run GIF: `reports/aim_squares_out/aim_squares_boardfix.gif` (from the boardfix frames).
* Status cards: `reports/aim_boardfix_sign_20261125.png`,
  `reports/aim_boardfix_half_20261125.png`, `reports/aim_boardfix_compare_20261125.png`.
* Diagnostics (read-only, uncommitted): `/tmp/boardfit.py` (free board re-fit),
  `/tmp/resid_scan.py` (mount standoff / focal scan), `/tmp/mm_err.py`
  (px → mm via local scale), `/tmp/disc.py` (discrepancy per probe).

No roadmap edits, no commits, no gripper actuation, no live-service reconfiguration
beyond the board POST. BC-250's uncommitted b1/hand-eye files were not touched.
