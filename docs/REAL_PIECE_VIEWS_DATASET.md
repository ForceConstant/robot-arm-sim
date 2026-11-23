# Real multi-view piece dataset (2026-11-22) — contract

**Decision (Brian 2026-11-22):** capture REAL wrist-camera views of each piece
at multiple angles/distances to learn (a) piece type and (b) the **base centre**
on the board, view-invariantly. Motivation: sim-trained type/pose probes
(`phase11c_type_pose_probe.py`) reached only 0.47–0.66 type accuracy with poor
E2E acceptance; the live failure is that brightness blobs shift with real
lighting/view. Real captured data is required to learn the real camera/lighting.

**Scope:** dataset generation + capture tooling first; training and deployment
are separate follow-on steps. No physical gate is implied by simulator data.

## Per-sample contract
- `sample_id`, `piece_type`, `square`, `view_index`
- `image` (raw 640×480 wrist frame, JPEG) + SHA256
- `camera_controls` snapshot (exposure/gain/brightness/contrast/WB)
- `joints_deg`, `keep_in.tip_mm`, `pose.tool_mm`
- `pnp`: marker ids, reprojection RMS, `camera_in_board_mm`, R/tvec, intrinsics
- `look_at` request: azimuth, pitch, distance, target (x,y,z)
- `label_base_xy_mm`: board x/y of the piece's base contact centre
- `label_source`: how base xy was obtained (see below)

## Base-centre labeling (critical, view-invariance)
The pawn is ~23 mm tall; projecting its bright top to z=0 shifts with view.
Base centre must be labeled at the **board plane** via top-down rectification:
- rectify each frame to a top-down board image using the marker PnP
  (`Board.rectified`), then locate the piece silhouette in rectified space and
  take its centroid; OR
- a one-time human click on the rectified top-down mosaic for a placed piece,
  producing a fixed board (x,y) reused across all that piece's views.
A labeled sample stores the same `label_base_xy_mm` for every view of that
placement; the model learns to predict it from the varying raw views.

## Capture geometry
Views chosen from reachable `look_at` poses around the target square:
azimuth 180/200/225°, pitch 30/45/60°, distance 160–240 mm (all dry-run
reachable). Jaws open throughout; no grasp. Arm returns to a safe HOLD.

## Splits / evidence
Group by placement (all views of one placement in one split) to avoid leakage.
Record generation script/config hash, counts per type/view, train/val/test
split manifest, camera-control snapshot hashes, and held-out metrics
(type accuracy, base XY error in mm, abstention). Compute: BC-250 CachyOS CPU
(EGL rendering, not GPU training). Local only; no cloud.

## Placement map for the full capture (Brian places before the run)

Config: `tools/piece_placements.json`. One piece per square, six spread across
the board centre/left/right (far corners a8/h8 avoided - near the reach limit):

| piece | square |
|-------|--------|
| pawn  | e5     |
| rook  | d4     |
| knight| f6     |
| bishop| c3     |
| queen | g5     |
| king  | e3     |

Place each roughly centred (small offsets are fine and useful - the model
learns in-square position). Note the knight's facing direction if visible.
Keep those six squares free of other pieces. Leave e5 / the camera approach
area clear while the capture runs.

## Capture commands (ready)
```
cd /home/brian/robot-arm-sim
PYTHONPATH=. .venv/bin/python tools/p4_capture_scenes.py \
  --placements-file tools/piece_placements.json --label full1
```
Uses 5 viewpoint offsets from the known-good e5 base pose; saves raw frames +
PnP/pose/tip/controls metadata + a rectified top-down mosaic per placement to
`.private/real_piece_views/<label>/`. Jaws open, no grasp, arm ends HOLD.

## Status (2026-11-22 17:10)
Contract, capture script (`p4_capture_scenes.py`), and placement map are ready
and synced to BC-250. Verified view poses (`p4_verify_view_poses.py`):
base = 3 markers RMS 2.4-3.6, lift+4 = 3 markers RMS 5.4, lift-8 = 2 markers
(marginal); pan-4 / wrist-flex±4 failed. Script now uses only base/lift+4/lift-8,
forces REMOTE with a 409-retry, and builds the rectified mosaic.

POC capture on the e5 pawn is still UNRELIABLE: marker detection at these poses
is flaky run-to-run (base gave 3 markers in one run, 0 in the next); some runs
capture the base view, others get no PnP frame. `look_at` ring was unreachable.
No model trained or claimed. Arm safe HOLD, jaws open.

**Next work before the full run:** make marker detection stable (exposure/tone
per view, or lower the >=3-marker gate with a confidence check), then re-run
POC until it reliably captures 3 views. Only then run the full placement set.

## Capture complete (2026-11-22 18:05)

First REAL dataset captured: `.private/real_piece_views/clean2/` —
**12 clean (overlay-free) views across all 6 pieces**, 2 views each (base +
lift−8), plus a rectified top-down mosaic per piece (7.1 MB total).
Captured via `tools/p4_capture_scenes_clean.py` using POST `/capture-view`
(raw frame BEFORE the HUD overlay) + locally computed PnP. PnP RMS 1.5–3.7 px.

Per-piece views in frame: pawn e5 ✓✓, rook d4 ✓✓, knight f6 ✓✓,
bishop c3 ✓✓, queen g5 OFF-FRAME both views (square beyond camera view at
these poses; needs a dedicated viewpoint), king e3 ✓✓.

Key fixes along the way:
- `/stream` bakes in the detection HUD — unusable for training. `/capture-view`
  saves clean raw frames (this is the correct source).
- `capture-view` rejects frames while the arm is still moving (moved_px>~2):
  settle 2.5 s + retry.
- Joint offsets must be ADDED to the base pose, not replace it.
- rectified()'s valid mask is 3-channel; mosaic collapses to 2D.
- Restored locked tone: auto_exposure=1, exposure≈705, gain=64, brightness=160,
  contrast=32 (it had drifted to gain=1/exposure=334, causing grainy frames).

Next: label base centres (click on the mosaics), capture queen g5 from a
dedicated viewpoint, then generate more placements/rounds and train.

### Queen g5 re-capture attempt (18:03) — BLOCKED
Two pose searches (`p4_find_g5_pose.py`, `p4_find_g5_fine.py`) could not frame
g5 (board 70,14) cleanly. Base view puts g5 at pixel x≈−107 (off left edge);
pan−4 brings it to x≈−23 (still just off), but pan≤−4.5 loses PnP entirely
(<2 markers / bad solve). Other offsets (lift, wrist_flex, elbow) move it
further away or break PnP. Conclusion: g5 is outside the usable camera view
for the current mount/pose family. Needs a different pose strategy (e.g. a
gaze-servo from a reachable stance, or relocating the queen to a framed square)
before it can be added. Remaining 5 pieces have 2 clean views each.

### Queen resolved (18:14) — moved to c5
Brian relocated the queen from g5 to c5. Re-captured cleanly:
`.private/real_piece_views/queen_c5/` — 2 overlay-free views (base + lift−8),
both in frame, PnP RMS 2.8/3.8, plus mosaic. Updated placement: queen c5.
All 6 piece types now have clean in-frame views (5 in clean2, queen in queen_c5).

### Base-centre labeling (18:28, p4_label_base_centres.py)
First-pass labels in `.private/real_piece_views/clean2/labels.json`. Method:
detect piece top blob in each clean view, unproject at the piece's KNOWN height
(pawn 23 … king 46.5 mm) to get the base xy. View-to-view spread across the
2 views: knight 0.28/0.25 mm and king 0.73/0.5 mm (good), pawn 3.5 mm and
rook 3.7/2.6 mm (marginal), bishop/queen only one usable view (blob detector
failed in the other). Unprojecting at piece height IS more view-invariant than
z=0 (z=0 points differ by 10–50 mm between views), so the approach is sound,
but blob detection needs work for robust labels on every view.
NOTE: the queen g5 entry in clean2 is stale (piece moved to c5); use the
queen_c5 dataset for the queen.

### CRITICAL data-quality finding (18:30): views not actually distinct
Camera-pose separation between the 2 views per placement: pawn 13.6mm,
rook 14.2mm, queen(g5) 12.9mm — but knight 1.2mm, bishop 0.4mm, king 0.6mm,
queen(c5) 2.7mm. The latter are effectively the SAME view (identical images),
so the "view-invariance" spreads for those pieces were meaningless. Root cause:
the lift−8 joint offset often doesn't produce a real pose change (IK lands near
the base, or the capture happens before motion settles).

Impact: the current 14-image seed has genuine multi-view coverage for only
pawn/rook. It is NOT sufficient for training a view-invariant type+base model.
Required before training:
  1. A capture tool that VERIFYs camera separation between views (reject
     captures with <5mm separation, re-attempt with a different offset).
  2. Find a larger set of genuinely distinct, reachable, piece-framing poses.
  3. Re-capture all 6 pieces with verified-distinct views, plus more rounds.
Do not train on the current seed and claim view-invariance.

## Adaptive capture + relabel (18:48)

`p4_capture_adaptive.py` fixed the view-distinctness problem: it measures the
actual camera pose via PnP after each move and only accepts captures with
camera separation >=8mm. Result: **24 verified-distinct clean views**, 4 per
piece (pawn e5, rook d4, knight f6, bishop c3, queen c5, king e3), stored in
`.private/real_piece_views/adapt1/`. PnP rms 1.8–4.9 px. This is the first
dataset with genuinely different viewpoints.

Relabeling (`p4_label_base_centres.py`, unproject top blob at known piece
height) on these real views exposes the REAL bottleneck — the blob detector:

| piece | labeled views | base-spread (mm) |
|-------|---------------|------------------|
| pawn  | 4/4 | 2.08, 2.60 |
| rook  | 3/4 | 15.2, 5.0 |
| knight| 4/4 | 1.0, 5.9 |
| bishop| 2/4 | 15.4, 9.1 |
| queen | 4/4 | 7.1, 6.5 |
| king  | 2/4 | 3.5, 4.4 |

The bright/threshold blob is not a reliable piece silhouette: it grabs marker
faces, neighbouring pieces, and highlight patches, and fails entirely in some
views. Sparse coverage + multi-mm spread means these labels are NOT good enough
to train a base-centre regressor yet.

### Training plan (updated, honest)
The captured images are clean and genuinely multi-view — good. The gap is
LABELING, not capture. Options, in order:
  1. **Human labels**: click the base centre on each of the 24 rectified views
     (or 6 mosaics) — reliable, ~24 clicks, gives a clean seed training set.
  2. **Better auto-label**: segment the piece using its known mesh/colour with
     a proper detector (not brightness threshold), validated against human
     labels on a small set.
  3. Only then train; require held-out base-XY error < ~3mm and type accuracy
     measured, with abstention, before any grasp use.
Do not train on the current threshold labels and claim a qualified model.

### Masked labeler comparison (18:55, p4_label_masked.py)
Masking ChArUco quads before blob detection reduces marker contamination.
Comparison (unmasked -> masked):
  pawn   4/4 spread 2.1/2.6  ->  3/4 spread 2.0/1.7   (better precision)
  knight 4/4 spread 1.0/5.9  ->  4/4 spread 2.0/2.8   (better)
  queen  4/4 spread 7.1/6.5  ->  4/4 spread 4.5/6.5   (slightly better)
  rook   3/4 spread 15/5.0   ->  1/4 (no spread)      (worse coverage)
  bishop 2/4 spread 15/9.1   ->  2/4 spread 4.3/8.0   (better precision)
  king   2/4 spread 3.5/4.4  ->  2/4 spread 3.9/4.4   (same)
Conclusion: heuristics (threshold + marker mask) cannot reliably segment all
6 piece types across views. Best spreads (~2mm) are only on pawn/knight.
This CONFIRMS the plan: use human-click labels on the 24 rectified views (or
24 clean images) as the training seed, then optionally train a learned
segmenter from those. Do not train on heuristic labels. labels_masked.json
saved for comparison.

### Labeling UI (18:58)
`tools/label_server.py` serves a click-to-label UI for all 24 clean adapt1
views. Running on BC-250 at http://192.168.1.171:8123/ (port 8123, PID 7753).
Operator clicks the piece BASE CENTRE on each image; clicks are saved to
`.private/real_piece_views/adapt1/labels_human.json` (resumable, progress shown).
This is the reliable seed-label source that unblocks training.

### Training script prepared (19:05, train_real_piece_model.py)
Consumes flat `labels_human.json` ({image_key:[u,v]}) + clean images + PnP.
Features: 8x8 thumbnail + colour stats + gradient around clicked base. Does
leave-one-piece-out type classification + label QA (clicked-base spread), and
emits a receipt with an explicit data-sufficiency verdict (seed vs qualified).
Dry-run on heuristic labels (16 flat rows): pipeline works end-to-end; LOPO
accuracy 0/16 as expected (bad labels, tiny data). NOT a qualified model.
Ready to run on the 24 human labels the moment they exist.

### Labeler access fixed (2026-11-23 07:01)
Port 8123 on BC-250 was blocked by UFW (only 8080 reachable). Brian authorized
disabling the firewall on this local-only dev machine: UFW stopped+disabled,
nftables flushed, iptables cleared. `curl http://192.168.1.171:8123/` now
returns 200 from the OpenClaw container. Labeler runs directly on BC-250
(port 8123, labels_human.json there). Portal/tunnel workarounds removed.
Direct link: http://192.168.1.171:8123/

## adapt2 consolidation (2026-11-23 08:30)
Brian labeled 14/24 of adapt1, skipping views where pieces were cut off. Built
`.private/real_piece_views/adapt2/`: skipped cases removed, valid human labels
preserved, and 2 new well-framed clean bishop views added (deduped). Now 16
views, 14 prefilled labels; only 2 bishop c3 views need clicking.
Found and fixed a white-balance drift (WB temp 5230 -> 2920); note the captured
images still look tinted vs the stream, needs checking before trusting color.
Labeler now serves adapt2 at http://192.168.1.171:8123/ (14/16 done).
Framing poses for it/rook/king also found in reports/framing_poses_20261123.json.

## Click audit (2026-11-23 08:38)
p4_audit_clicks.py compared each click to the PnP-projected target square:
13/14 clicks land 9-33px from their target square (correct pieces), including
all 3 queen c5 clicks. Only rook pan_m4 is bad (122px). So Brian's existing
labels are valid; the camera DOES frame c5 etc. correctly.

BUT the 2 new bishop views are genuinely bad: c3 projects to y=46-55 (top
edge), so the bishop is cut off even though margin math looked ok on the base
point. Wrist tilting cannot fully frame distant c3 from the e5-centered camera.
These 2 views should be dropped, not labeled. Need a different camera POSITION
(not just wrist tilt) to frame c3/king/rook far squares.

## Camera-position probe (2026-11-23 08:45) - SOLUTION for far squares
Moving the CAMERA (absolute xyz tip position) frames far squares that wrist tilt
could not. Results (tip target -> where square projects):
- tip c3 (-42,-42,60): c3 at pixel (177,231) - centered, whole piece frameable
- tip e3 (14,-42,60): e3 at pixel (177,287) - centered, frameable
- tip e5 (14,14,60): c3 at (412,40) top edge (confirms old views were limited)
- d4/c4/d3 positions gave no PnP at z=60 (IK/fault); c5 tip frames c3 off-edge.
So to capture bishop/king, position the jaw tip over c3/e3 at z~60, then take
clean views with small pose variation. Build a position-based capture script.

## Merged dataset + first real model (2026-11-23 08:55)
`.private/real_piece_views/merged1/`: 19 human-labeled clean views, all 6 piece
types, labels sub-2mm consistent (pawn 0.7, rook 0.4, knight 0.8, bishop 0.4,
queen 1.8, king 0.9mm). Dropped 2 bad clicks (rook pan_m4, old king adapt1).
Training (train_real_piece_model.py, compact patch features, nearest-class-mean,
leave-one-view-out): TYPE ACCURACY 15/19 = 78.9% vs 21% majority baseline.
NOTE: earlier 0% was a BUG in eval (leave-one-PIECE-out removes the class to
predict) - fixed to leave-one-VIEW-out. Still a SEED model, not qualified;
needs more data for held-out confidence + base-XY regression + abstention.

## XY regression + abstention eval (2026-11-23 09:00)
On merged1 (19 views): base-XY leave-one-view-out nearest-neighbour: median
2.1mm, mean 5.7mm (p90 11, max 40) - large errors when NN picks a different
piece. Class-conditioned (predict type first): median 1.5mm, 14/19 within 3mm,
but a misclassification causes an 88mm outlier. TYPE abstention (confidence
margin): margin>=1.0 gives 100% precision at 63% coverage; >=2.0 gives 100%
at 32% coverage. Takeaway: type + base XY are learnable from real views, but
reliability hinges on abstaining when type confidence is low. Still a seed
model, not qualified. Receipt: reports/xy_abstain_eval_merged1.json.

## Round 2 capture + auto-advance labeler (2026-11-23 09:10)
Generated 18 new position-based clean views, ~3-4 per piece across all 6, all
target squares centered in frame, in `.private/real_piece_views/round2/`.
Labeler (http://192.168.1.171:8123/) now serves round2 with AUTO-ADVANCE: after
each click it jumps to the next unlabeled view, so Brian can click straight
through 18 cases. Verified in browser (write + auto-advance code present).

## Round2 retrain result + DIAGNOSIS (2026-11-23 09:25)
Merged dataset now 37 human-labeled views (all 6 pieces). Retraining gave only
11/37 = 29.7% (near 21.6% baseline) - WORSE than the 19-view 78.9%. Diagnosis:
pieces were at DIFFERENT board positions between capture sessions (pawn y~26
in round1 vs y~12 in round2; queen (-1,35) vs (-25,6); knight y~55 vs ~45).
The patch features encode POSITION, so mixing placements confuses the classifier.
Controlled eval: round2-only (consistent positions) = 15/17 = 88.2%; round1-only
= 60%; merged = 29.7%. So the model learns where pieces sit, not their shape.
FIX for next iteration: position-invariant features (patch normalized to square
frame, or include square-relative offset with square-centred patches) and/or
re-capture all pieces at consistent positions. Do NOT claim 30% is a capability
regression - it's a confound from mixed placements.

## Position-invariant feature experiments (2026-11-23 09:30)
Goal: beat the placement confound (merged 29.7%). Three feature sets, LOOV type
accuracy (merged / round1 / round2):
- patch stats (baseline):         0.30 / 0.60 / 0.88
- background-subtracted residual: 0.43 / 0.85 / 0.71
- board-rectified top-down:       0.51 / 0.45 / 0.71
Observations: invariance helps merged (0.30->0.51) but no single set wins all
splits; round1 vs round2 differ (appearance/placement drift between sessions).
With only 3-8 views per type, class means are unstable. Honest status: real
signal exists (subgroups hit 0.85-0.88) but NOT a robust/qualified model.
Next: more views per piece at CONSISTENT positions + a proper train/test split
by placement (not random views) to measure real generalisation.

## Brian's correction + z=0 fix (2026-11-23 09:35)
Brian pointed out pieces never physically moved, so the 'placement confound'
was MY error: I unprojected base clicks at piece HEIGHT, adding view-dependent
parallax. Verified: unprojecting at z=0 gives per-piece positions consistent
across sessions within 1-3mm (pawn (10,11) both, bishop (-44,-41) both, etc.).
So the data IS consistent; the confound was computational.

Square-relative features (top-down patch centered on known square centre, z=0
base offset) on merged 37: 18/37 = 48.6% (up from 29.7%); round2 82.4%,
round1 50%. Confusion: bishop 6/6, knight 5/7; pawn<->queen/king confusion
tracks capture session (appearance/tone drift), not position.
Honest: real signal, but not a robust model. Remaining gap = appearance drift
between sessions (lighting/WB/tone changed). Fix = consistent capture settings
and/or per-session normalisation + more views. NOT a placement problem.

## Focus vs distance (2026-11-23 09:45, Brian's question)
Brian noted captures look soft and asked if we need to be closer. Swept tip z
with clean frames, measured Laplacian sharpness RAW and on board-rectified
(fixed 6px/mm) images to remove magnification. Results (tip_z -> raw/rectified):
  39mm -> 34.9/16.5 ; 45 -> 49.7/26.6 ; 50 -> 65/30 ; 58 -> 95/38 ;
  60 -> 79/32 ; 73 -> 136/36.
Both raw AND scale-invariant rectified sharpness are LOWEST when closest and
rise with distance, plateauing around tip z 55-75mm. So the captures are NOT
too far - if anything moving CLOSER makes it softer (fewer pixels per feature
and likely near-field/lens limits). The earlier z~60 position (rectified 38,
best of the low range) is a reasonable capture height. The 'optimal focus
distance' from block5c (focus score 144 then) was a RAW score at a closer board
distance; raw scores are magnification-dependent, so it is not directly
comparable. Conclusion: softness is not solved by moving closer; likely lens
aperture / lighting / exposure. Block5c also found brightness-vs-sharpness
corr -0.84 (glare), suggesting tone/lighting, not focus distance.

## Best model so far: MLP on square-relative features (2026-11-23 09:55)
train_mlp_real.py: 2-layer MLP on square-relative top-down features (z=0 base,
square-centred patch), leave-one-view-out on merged 37:
  28/37 = 75.7% vs 21.6% majority baseline.
Consistent across sessions (round1 14/20, round2 14/17) - fixes the earlier
session-inconsistency of nearest-mean methods. Per type: queen 6/6, pawn 7/8,
knight 6/7, bishop 6/7, rook 2/3, KING 1/6 (confused with bishop - both tall).
Color normalization had no effect; focus-distance change not the issue.
Still a small seed dataset - not a qualified model - but a real, consistent
signal. Next: more king/bishop views from distinct angles to separate them,
then a held-out-by-placement split and base-XY regression on these features.

## Base-XY regression on square-relative features (2026-11-23 10:00)
train_xy_regression.py, z=0 labels, leave-one-view-out:
- nearest-neighbour: median 1.2mm, p90 56mm, mean 22.6mm; 22/37 within 3mm.
  When NN finds a same-piece neighbour the base is accurate to ~1mm; large
  errors are cross-piece matches (same story as type misclassification).
- linear ridge: median 35.9mm (fails - mapping is non-linear in these features).
Takeaway: base XY is learnable to ~1mm when type/neighbour is correct; overall
grasp reliability is gated on the type model's accuracy + abstention, not on XY
regression alone. Consistent with the 75.7% MLP + abstention results.

## Round 3: targeted king/bishop cases (2026-11-23 10:10)
To break the king/bishop confusion, captured 9 new position-based views with
more angle diversity (added roll ±8, pan/lift combos), tone locked at WB2920:
king e3 7 views (incl. roll variants), bishop c3 2 views (many poses lost PnP).
Auto-advance labeler serves round3 (0/9). Arm HOLD.

## Held-out-by-session generalization (2026-11-23 10:15)
eval_grouped.py: MLP on square-relative features, train on one capture session
and test on the OTHER (no near-duplicate leakage):
- train round2 -> hold round1: 12/20 = 60%
- train round1 -> hold round2: 7/17 = 41%
- overall grouped: 19/37 = 51.4%
vs random leave-one-view-out 75.7%. So the honest cross-session generalization
is ~51%, meaningfully below the optimistic random split. Session appearance
drift (tone/framing between capture rounds) still hurts. This is the real bar
to beat with more consistent captures / more data. NOT a qualified model.

## Round3 merged (46 views) retrain - HONEST: no improvement (2026-11-23 10:20)
Added 9 king/bishop views (46 total: king 13, bishop 9). Results:
- MLP random LOOV: 32/46 = 69.6% (DOWN from 75.7% at 37 views)
- grouped cross-session: 24/46 = 52.2% (~flat vs 51.4%)
- confusion: king 8/13 (5 -> bishop), bishop 6/9 (3 -> king) - confusion is now
  SYMMETRIC and persists despite more data; queen dropped to 4/6 (2 -> rook).
Conclusion: more views of the SAME two hard pieces does not fix king/bishop -
they are genuinely hard to distinguish from this camera angle, and round3 is a
third appearance session that slightly hurts random splits. More of the same
data is not the lever. Need either: (a) a viewpoint that reveals a distinguishing
feature (king cross vs bishop slit), (b) higher image quality/resolution, or
(c) accept type-known context from the game state (the move being played tells
you the piece type) and use vision only for base XY. Do NOT keep grinding
identical captures.

## TYPE-KNOWN base-XY eval (2026-11-23 10:25) - STRONG
When piece TYPE is given (from game/move context), vision-only base XY via
true-type-conditioned nearest neighbour, leave-one-view-out on 46:
  median 0.6mm, mean 1.0mm, p90 1.7mm, max 4.7mm; 43/46 within 3mm (93%),
  46/46 within 5mm. Per-type medians: bishop 0.2, king 0.4, knight 0.6, pawn
  1.1, rook 1.2, queen 1.5mm. The earlier type misclassification (king/bishop)
  was the ONLY thing hurting XY - with type known, position is excellent.
This strongly supports option 3 for the grasp goal: get type from game context,
use this vision model purely for base XY. Report: reports/typeknown_xy_eval.json.

## Type-known base-XY: leakage fixed + deployable model (2026-11-23 10:35)
Brian decided: type from game context, vision only for base centre. IMPORTANT
CORRECTION: the earlier 0.6mm result was TARGET LEAKAGE - square-relative
features included the click-derived offset (the target itself). Rebuilt with
LEAKAGE-FREE features (image-only: top-down square-centred residual thumbnail):
  honest type-known base-XY LOOV on 46: median 1.68mm, mean 2.45mm,
  30/46 within 3mm (65%), 40/46 within 5mm, max 5.8mm.
Model artifact live/base_xy_model.json (sha256 c71da8d6b2c2); inference tool
tools/infer_base_xy.py predict_base_xy(frame, piece, square) -> board xy.
Verified end-to-end on a captured frame (2.6mm on an in-training frame).
Not a qualified model, but a real, honest signal usable for targeting when type
is known. Directly comparable to the ~13mm raw tip error the loop already fixes.

## Integrated grasp pipeline dry-run (2026-11-23 10:45)
tools/p4_grasp_context.py: type-from-context -> predict_base_xy (leakage-free
model) -> validated tip-correction loop -> descend -> close -> lift. Dry-run from
a known e5 pose: predicted pawn base (9.7,13.9) mm, servo target reachable
(poserr 3.0mm), no motion/jaws executed. Requires --execute (explicit approval)
for a real grasp. Gripper actuation still needs Brian's go-ahead.

## First integrated grasp attempt (2026-11-23 10:15) - ABORTED SAFELY
Ran p4_grasp_with_video.py (pawn e5). Vision predicted base (9.7,13.9)mm.
Tip-correction loop aborted on leg 1: cmd=cmd+err accumulated the FULL 67.8mm
initial error, commanding z=126.5mm which failed the dry-run. No descent, no
jaws closed, no fault; arm safe HOLD. GIF of attempt: reports/grasp_final.gif.
FIX: clamp each correction step to +/-12mm, z to [2,200], steps 6->14. Synced.
Ready to retry with the clamped loop on Brian's go-ahead.
