# Phase 10: piece-specific contact-band calibration

## Status

Development evidence, not Phase 10 completion or physical qualification. The
unchanged baseline matrix remains 247/300 complete, with no scored wrong-piece
or collateral moves. A new held-out qualification is still required after
grasp, perception and settling changes are finalized. Do not update the roadmap
from these diagnostics.

Brian identified that the king has two collars about 13 mm apart, while the
recorded fingers reached only the upper one. Contact geometry supports that
diagnosis. The fix changes the fingertip depth, not the finger meshes, material,
friction, commanded jaw closure or abort/success thresholds.

## Measured contact evidence

All heights below are measured from the piece base in the existing private
printed mesh, not new physical metrology.

| Piece | Baseline tip height | Relevant collars | Evidence / candidate |
| --- | ---: | --- | --- |
| King | 32.70 mm | about 26 and 39.5 mm | Baseline contacts only 35–40 mm. Lower tip 8 mm to 24.70 mm. |
| Queen | 24.17 mm | about 23.3 and 33.5 mm | Contact loads are unbalanced. Lowering 1–4 mm has not yet rescued the representative failed carry; no queen offset adopted. |
| Bishop | 18.01 mm | about 18 and 27.2 mm | Baseline already loads both bands throughout the measured carry; retain its current height. |

### King matched replay

Seed 20261110, case 10: the original king e1→e2 staging primitive aborted as
`carry_unstable`. Lowering only its grasp/place height by 8 mm:

- maintains contacts on both collars for all 39 lift and 118 carry samples;
- keeps peak base-reference drift below 0.85 mm (unchanged 4 mm abort limit);
- completes the primitive, released/supported/settled, with 0.386 mm placement
  error and no collateral;
- allows the next rook primitive to complete. The full castle still abstains
  at the final king localization (reported sigma 4.754 mm versus a 4 mm gate).
  That transaction is **not** scored complete.

A 10 mm-lower diagnostic also completed the same primitive, but 8 mm is the
smaller validated candidate. In a five-case paired development pilot
(2, 10, 12, 16, 74), complete transactions improve from 2/5 to 4/5: two prior
passes remain passes, two failed castles become complete, and case 10 remains
an honest later-stage abstention. No wrong-piece/collateral event was recorded.
Ordinary execution with the candidate pack exactly reproduces case 74's
three primitive placement errors from the override-based diagnostic.

### Queen and bishop controls

Queen case 111: the baseline fails carry. A 4 mm-lower tip increases lower-ring
load but unloads the upper ring and still fails carry. The 1 mm-lower variant
also fails carry; 2/3 mm-lower variants fail the guarded grasp descent. Retain
those failed controls; this is not evidence for a universal lowering rule.

A coupled diagnostic at 2 mm lower / +20° jaw span completes case 111 and
maintains both bands through all 118 carry samples. Neither 2 mm lower alone
nor +20° alone completes it. This is a successful case-specific correction,
not a global queen default. The opposite −20° span is rejected: the queen
falls after the slip abort and displaces a pawn. The trace shows queen/pawn
contact only after `carry_unstable` latched, not during jaw closure. Denying
game-state commit is not proof that a post-abort fall is contained. Preserve
this failed control and trace; do not relax guards or select that orientation.

Bishop case 0: baseline and 3 mm-lower variants both complete without
collateral. Baseline loads both collars for all 102 carry samples. Lowering
shifts load toward the lower collar and is not adopted without a demonstrated
benefit. Contact count alone does not establish stable torque support.

## Candidate representation and reproduction

Printed-pack records may optionally specify `grasp_tip_offset_m` (default zero).
`PieceGeometry.grasp_tip_offset` scales with the piece. The custom parallel
controller applies it to the tip height while preserving the existing
width/closure calibration; the full finger band is still checked for opening
clearance. The stock controller is unchanged.

The private candidate `.private/piece-assets-v6-king-candidate` sets only the
king offset to −0.008 m. The original v5 manifest and all original meshes are
preserved. This is opt-in; neither the live service nor its assets were changed.

Evidence is under `.private/phase10/`:

- `diagnostic-case10-king-lower8-seed20261110.json` and its full control trace;
- matched king cases 2/12/16/74 and ordinary `candidate-pack-case74` receipt;
- `diagnostic-case111-queen-{baseline,lower1,lower2,lower3,lower4}*`;
- `diagnostic-case0-bishop-{baseline,lower3}*`;
- separate Phase 8 candidate receipts, without overwriting prior reports.

The public synthetic two-collar regression checks that the lower tip includes
both collars, clears the board, opens around the full shape, and preserves the
original hold gap. Fresh Phase 8 receipts and all source-provenance audits must
pass before release. Simulation preload, friction and ballast distribution
remain modeling assumptions rather than measured hardware force calibration.

The king-only candidate's fresh CAD Phase 8 receipt passes: 64/64 first-pass
source paths and 4/4 distinct tray destinations; crowded execution remains
23/25, with the two queen cases still open. This is bounded foundation evidence,
not the Phase 10 ≥99% reliability gate. Full regression runs initially report
146 CAD passes / 140 stock passes plus 6 stock skips and two old-receipt
source-hash failures in each; those audits require newly measured receipts,
not changed expectations. After archiving the prior receipts and installing
the completed source-matched measurements, all three receipt audits pass.
Fresh full reruns are green: **148 CAD tests passed; 142 stock tests passed,
6 skipped**. No test expectation or success threshold was weakened.

## Piece-shape variants are permitted design candidates

Brian also confirmed that simple piece-shape changes can be considered. Prefer
small variants that preserve board footprint and overall height, such as a
straight grip band, matched collar diameters or moving a collar into a reachable
grasp band. Preserve the current set as the control; qualify new variants with
real geometry/contact, approach clearance, release and the same success gates.
Report changes in feasible approach, contact stability and placement separately
so every shape candidate is measured against the same task.

A changed mesh requires revised mass, center of mass and inertia. The existing
assembled weights remain measured facts only for the unchanged pieces. Estimate
new shell mass plus the explicitly assumed existing ballast for simulation and
label it as an estimate; do not silently reuse the old measured totals as new
measurements. Physical repeatability and any new-piece weigh-in remain separate
from the simulation gate.

## Broader development checks (continuation, 2026-11-04)

The lower king tip is a candidate, not a blanket reliability fix. The matched
66-king-case bank is being measured in `.private/phase10/ring-candidate-bank/`.
It has already exposed regressions on previously passing cases: case 19 fails
release/settling, and case 57 fails the executed lift waypoint. Keep these
controls in every proposed repair bank; do not count a repaired carry as a
completed transaction.

The queen −2 mm/+20° diagnostic finished on cases 4/7/9/11/111/212/258/262:
**5/8 complete versus 4/8 baseline**, zero collateral. It rescues 111 and 262,
but regresses the previously passing case 4 to `gripper_neighbor_contact`;
258 also refuses neighbor contact, and 212 still abstains on localization.
This is not adopted as a global queen height/span setting.

The saved-state case-19 trace shows stable two-collar carry (maximum base drift
0.475 mm), jaws clear at the end of opening, then renewed king/jaw contact
while withdrawing. The IK-driven retreat changes posture beside the king and
ends with the piece still touching a jaw. A private experiment reverses the
**measured placement joint trajectory with open jaws**, then continues the
normal high retreat. It completes case 19 with release/support/settling and
zero collateral, but a broader regression bank is still required before source
adoption. It neither repositions a piece directly nor changes verdicts.

A separate saved-state knight case-1 diagnostic distinguishes this from the
king's original pivot: the base drifts 4.289 mm and a material point anchored at
the initial pinch patch drifts 4.279 mm relative to the tool. Relative rotation
reaches 16.85°. Increasing only the knight's existing 9 mm closure allowance to
11 mm still aborts `carry_unstable`, with zero collateral. The separate pack
`piece-assets-v6-knight-squeeze11-diagnostic` is a rejected control, not a new
hardware-force calibration or runtime default. The original pack is unchanged.

### Completed development banks and current source candidate

The 8 mm lower king-tip bank is complete: **60/66 transactions**, compared with
38/66 on the prior controller/profile, with **24 rescues, two regressions, zero
wrong-piece final picks and zero collateral**. The six remaining cases are 10
(vision), 19 (release/settling), 57 (lift waypoint), 130 (grasp descent), 146
(placement descent), and 247 (the captured victim's carry, not the moving king).
These reused cases are development evidence, not a new held-out qualification.

Lowering only 6 mm rescued several reach cases, but regressed previously passing
case 2 with one displaced pawn (duplicated final/transient count 2). It is **not**
a safe global replacement for the 8 mm profile. Separately, case 57 completes
at the original 8 mm lower tip with only the span changed to 0°; continue
reach/span planning rather than hardcoding that angle or changing every tip.
The 0.8 s fixed post-motion wait completed 11/12 selected cases, but case 17
still failed settling. No global timing change was adopted from that pilot.

The retry-aware reverse-placement retreat prototype passes 7/7 selected cases,
including 19, 17 and 165, and preserves its passing controls. The corresponding
source change is now in `controller.py`: the custom parallel tool records the
latest measured placement descent, resets it on a descent retry, and commands
that joint path backwards with open jaws before the high retreat. All steps
still use servo physics and neighboring-piece contact guards. Stock retreat
is unchanged. Ordinary benchmark runs of source cases 19/17/165/2 all complete
with zero collateral. Fresh Phase 8 and regression receipts are being measured;
older receipts do not qualify this new source, and nothing has been deployed.

A separate pixel-ray centroid-height fit is rejected: valid completed diagnostic
receipts score 1/2, with a collateral regression on prior pass 2; cases 10/212
also hit a diagnostic serialization error and are not physical failure counts.
It is not in the source detector. Preserve these controls rather than treating
an experimental lower uncertainty estimate as a qualified localization fix.

### Private straight-grip-zone knight shape candidate

The knight trace's loaded contacts migrate from roughly z11–21 mm after closing
to z16–19 mm at carry abort, narrowing as the piece slides through its tapered
waist. A separate **13.4 mm diameter straight zone at z11–22 mm**, blended over
z9–24 mm, was created from the original mesh. Height and overall footprint are
unchanged, and both the original and failed shape-tool attempt are preserved.
The valid new pack is `.private/piece-assets-v8-knight-grip-zone-candidate`;
**do not use** the incomplete v7 directory. New closed, positive-volume meshes,
72 collision slabs and controller envelopes were regenerated. This is a design
candidate, not a fabricated or weighed part.

At the original effective-density estimate, the shell changes from 1.55854 g to
2.11612 g. Keeping the original *estimated* ballast of 2.94146 g at the explicit
3 mm assumption yields **5.05758 g estimated assembled nominal**, with newly
computed COM/inertia. The source product twin still correctly records Brian's
**4.5 g measured original knight**. Shape diagnostics override the new body's
mass properties explicitly and label them estimated; they do not reuse the old
measured total for changed geometry. The candidate's ±0.1 g regimes are modeling
sensitivity about that estimate, not new weight measurements.

Matched shape checks: **10/12 knight cases complete**, plus **4/4 non-knight
controls**, zero collateral. Both incomplete cases (22, a previous pass, and
105, an old carry failure) meet placement/release/support/upright but fail the
instantaneous settling check. The exact original carry case 1 now completes at
1.119 mm placement error. A separate bounded sustained-settling diagnostic
requires the unchanged physical gates continuously for at least 0.2 s, with
about 1 s maximum additional observation and a fail-closed timeout. Neither
that verification policy nor the shape is adopted into the runtime profile yet.

The reverse-retreat source candidate has now completed fresh foundation checks:
CAD 64/64 first-pass source paths, 23/25 crowded cases, 4/4 tray destinations;
stock comparator 51/64 paths, 7/25 crowded, 3/4 trays. Prior official receipts
are archived under `.private/phase10/phase8-before-reverse-retreat/`; current
CAD/stock receipts were installed only after verifying every source and manifest
hash. Full current-source regressions pass: **148 CAD; 142 stock, 6 skips**.
`git diff --check` is clean; roadmap is unchanged and nothing is deployed.

The shape's 1-second sustained-window banks score 15/17 distinct cases, zero
collateral; 15 and 17 do not sustain the window despite instantaneous passes.
A separately preserved 2-second-budget confirmation scores 6/7, still failing
previously passing case 15. Other unchanged checks remain the 10/10 earlier
confirmation, not a newly rerun two-second bank. The case-15 saved trace and
velocity series show repeated settling spikes (linear speed up to 0.0192 m/s,
angular speed up to 0.878 rad/s), while release/support/placement remain valid.
Do not adopt the shape or declare the stronger settling policy qualified.
Investigate the persistent motion/contact model against unchanged controls
before another timing change or a hardware recommendation.

A fine-angle fallback tested only after cardinal planning returns no span still
refuses bishop case 191; its passing bishop control 0 remains passing, with no
collateral. This fallback is not adopted. These diagnostics do not close the
fresh >=300-case >=99% gate, queued wrist self-occlusion work, or fault suite.

## Brian’s measured parallel-sided knight (2026-11-05)

Brian supplied a fabricated replacement STL with parallel sides and measured
assembled mass **4.7 g**. This supersedes the radial grip-zone prototype as the
next design to evaluate, not the preserved historical control. Import uses
native millimetres at scale 1.0: height 32.044 mm and footprint 18.792 mm across.
It must not receive the old source set’s 0.5 scale again. Sampled X widths are
about 8.5 mm throughout the upper gripping region; Y remains head-shaped.

The private candidate retains the lower-eight king calibration and all other
piece meshes. It has 52 horizontal convex collision slabs, a closed
positive-volume visual mesh, and zero uncovered points above 10 micrometres in
a seeded 5,000-point surface sample. This is sampled coverage, not an exact
approximation bound; slab convexification still fills horizontal cavities.

The development wrapper binds the new knight’s total mass to 4.7 g with the
approved ±0.1 g envelope. Estimated effective-density shell mass is 1.99656 g;
the residual 2.70344 g is modeled at 3 mm above the bottom. Only the assembled
total was measured; COM/inertia and ballast distribution remain estimates.
The historical product twin and original knight remain intact during this
controlled comparison. The first bank retains prior grip height/closure and
unchanged slip/contact/vision/completion gates, with matched current-source
original-design controls. No material, jaw geometry, controller, dwell or
threshold change is bundled with this import. No deployment or Phase 10
completion claim follows from this development bank.

Completed matched current-source bank (seed 20261110, reused development cases):
**11/12 complete versus 4/12 original**, zero final wrong picks, zero collateral,
and zero transient wrong-piece/collateral events. All four passing controls
15/22/30/47 remain complete. Seven original carry failures
1/21/54/105/261/265/288 become complete with commit authorized. Case 13 changes
from carry instability to `gripper_neighbor_contact`, remains incomplete, and
correctly denies commit with no displaced neighbors recorded. Do not count the
refusal as a completed transaction. Case 1 also completes in separate nominal
4.7 g and high-bound 4.8 g runs, alongside its matrix low-bound 4.6 g run.
Receipts: private `brian-knight-{first-bank,original-controls}` and
`brian-knight-comparison-summary.json`. The geometry and measured total changed
together, so this is a product-design comparison, not an isolated shape-only
causal estimate. This candidate is the leading knight design for further work;
12 reused cases do not establish the fresh >=300-case >=99% exit gate.

## Knight thin-face orientation and aperture diagnosis (2026-11-05)

Brian identified that wrist rotation should present the jaws to the replacement
knight’s parallel thin faces. Restoring case 13’s saved approach confirms the
actual jaw span was 89.36 degrees in world XY, versus the thin-face normal at
121.9999 degrees (the equivalent opposite normal is -58 degrees): approximately
32.65 degrees of 3-D misalignment. It was not a full 90-degree face error.

Preserved ablations: a fixed world-X span fails all six selected cases on
neighbor contact. Oracle-assisted piece-axis alignment imposed on both source
and destination completes only 2/6. Source-only axis alignment retains the
ordinary destination planner but also completes 2/6: case 13 refuses
`grasp_no_clear_span`; cases 15/30 contact a neighbor, and 47 has no clear span.
Neither is adopted. An earlier wrapper import error is tooling failure, not a
physical trial, and is excluded from these scores.

The next explicitly diagnostic candidate aligns only the source span with the
piece’s local X parallel-face normal, opens to **16.5 mm** (8.5 mm face width
plus 4 mm each side), and retains the old commanded hold gap. It replaces the
stock swinging-jaw 20 mm clearance-room heuristic for this knight with a
symmetric translating-jaw estimate of 12.75 mm per side: 8.25 mm half-aperture,
2.5 mm finger thickness, and 2 mm margin. The existing 8 mm heuristic tolerance
is unchanged. Actual robot/neighbor contact, slip, IK and transaction-commit
guards remain active; no source controller/config/vision code is modified.
This is a combined orientation/aperture/planning-heuristic experiment, not an
orientation-only causal estimate or proof that the old clearance threshold was
physically impossible. Case 13’s selected source span is 121.99996 degrees;
it completes and authorizes commit with no wrong picks/collateral events.

**Orientation in this feasibility experiment comes from simulator body pose.**
That is intentionally oracle-assisted diagnostic information, not wrist-only
inference or a production control policy. A real fix must estimate the piece’s
thin-face orientation from the wrist observations, account for its uncertainty,
select a reachable equivalent jaw direction, size the opening from the oriented
geometry, and qualify that full path without using simulator pose as input.

The combined thin-axis/narrow-opening development bank completes **6/6**
(cases 13/1/15/22/30/47), all commits authorized, zero wrong picks, zero
collateral and zero transient wrong-piece/collateral events. It preserves all
five selected passes from the preceding replacement-knight bank while rescuing
13. Receipts: private `brian-knight-thin-axis-opening` and
`brian-knight-orientation-comparison.json`. This is a six-case oracle-assisted
feasibility result only. Do not claim wrist-only qualification, deploy it, or
close Phase 10 from this result.

## Camera-derived orientation continuation (2026-11-05)

An experimental, not-default `chess_sim/piece_orientation.py` now accepts only
calibrated RGB views, known-arm exclusion masks, the registered source prior,
and an authored mesh/texture. It renders an isolated appearance reference and
fits a common yaw across views with bounded pixel translation. It does not
accept a simulator environment, body poses or simulation camera extrinsics.
Fit separation, independent-view support and leave-one-view-out disagreement
produce a conservative acceptance gate; these score-derived uncertainty values
are not yet physically calibrated confidence intervals.

Preserved failed image-only controls: whole-piece convex silhouettes and
foreground precision are dominated by the round foot/neighbor components;
upper-head edge matching alone also gives a wrong estimate in case 13. An
appearance fit on case 13's six saved 640x480 views estimates a 125-degree axis,
versus 122 degrees scoring truth, with four supporting views. This isolated
fit does not establish reliable execution.

- Initial current-resolution camera-driven bank: **0/6**. Five orientation
  uncertainty refusals, one neighbor-contact abort; zero collateral. Original
  accepted localization/contact/slip/commit gates remain intact.
- Image-fitted XY alone at 640x480 still refuses case 13 on contact.
- A separately declared **1280x960 simulated sensor-render envelope**, not an
  upscale of old RGB, uses exactly doubled calibration K/resolution and doubled
  pixel uncertainty/area floor. It completes **2/6** (1 and 30), with three
  uncertainty refusals and case 13 contact. This is not current-resolution or
  physical-camera qualification. No runtime resolution was changed.
- Lighting-invariant masked NCC at 1280x960 also completes **2/6**. Padding the
  search window to retain partially edge-visible views, while excluding invalid
  image pixels, still refuses the three selected uncertain cases 15/22/47.
  These remain failed controls, not successes with softened gates.
- At 1280x960 case 13's image-fitted XY is 0.049 mm from scoring truth and yaw
  is 120 degrees versus 122 degrees truth, yet motion still hits the neighbor.
  Scoring truth is used only after inference, never as its input.

The precise first contact is the **left parallel jaw against the neighboring
white rook**, during approach. The uncertainty-budgeted opening is 14.5 mm;
actual opening at contact is 14.539 mm, ruling out substantial jaw-opening lag.
Tool yaw is 116.798 degrees for a 120-degree command, but its approach axis is
about 33.8 degrees from vertical. The finger sweeps the rook before the intended
pinch. A smaller opening alone therefore does not fix it. Higher upright-IK
weight is being evaluated separately; do not adopt a change from one replay.

Private receipts are `brian-knight-camera-{axis,axis1280,ncc1280}`,
`brian-knight-camera-pose640-v2`, `brian-knight-camera-budget1280`, and
`camera-contact13-cached-baseline-detail.json`. The contact-detail cached run
replays previously image-derived observations and is explicitly **not fresh
perception evidence**. Initial wrapper import/initialization errors and an IK
cost-setter tooling error are excluded from physical counts. Source snapshots
preserve the initial orientation implementation with its receipt-matching hash.
No production grasp/vision defaults, deployment or roadmap completion changed.

### 2026-11-05 — broader camera selector and scan-dynamics audit

The 640x480 image-derived selector (16.5 mm opening, upright IK cost 5)
completed its six-case pilot, but the complete reused 42-knight development bank
was **34/42**, versus **38/42** for the broad-grasp control. Two rescues (13,246)
were outweighed by six regressions (83,105,136,137,165,288). That policy is
rejected for adoption. No wrong-piece, collateral or transient disturbance events
were recorded; unsuccessful moves denied commit. Uncertain image orientation
rejects the thin-axis candidate, not the pre-existing broad strategy.

Pure kinematic diagnostics identified untrackable pregrasp targets under the
stronger upright objective. The private v3 selector checks the existing 4 mm
final grasp tolerance, 15-degree tilt limit and 10 mm nonprecise pregrasp
tolerance before moving. It may choose the original broad plan after a
pre-motion planning refusal, never after a physical contact/slip abort. The
first clean-scan focused bank completes **7/10** (13,83,105,136,137,165,288);
134 still aborts on carry instability, 206 and 246 on neighboring-piece contact.
All deny commit on failure, with zero scan contacts or collateral.

**Benchmark audit:** the inherited camera scan helper zeroed all simulator
velocities after each view. This was removed without changing its servo-step
counts. Scan motion now hard-latches robot/active-piece contact or excessive
active-piece displacement, prevents subsequent scan/control execution, and
records scan contact/displacement evidence in compound receipts. Earlier banks
with that reset are development comparisons only, not clean exit receipts.

On the corrected scan dynamics, separate containment development receipts pass
**14/14** missing/uncertain localization, inconsistent/missing reobservation,
capture interruption and promotion cases; second-prefix castle interruption
passes **4/4** (both colors, both sides). Zero scan contacts or collateral.
These intentional aborts are not successful transaction-rate cases. They do not
replace fresh held-out qualification, full prefix coverage or perception gates.

A clean-scan full 42-knight paired comparison of the preflight v3 selector and
broad control is underway. Current integration remains private diagnostic
wrappers; the normal application has not adopted the image-axis policy or the
new knight default. Phase 10 remains active; no deployment or roadmap completion.

Clean-scan castling interruption coverage subsequently completed **10/10**:
all nonfinal prefixes of white/black kingside and queenside plans, no commit,
no scan contact or collateral. Private summary: `clean-castle-interruption-summary.json`.

A separate approach-preflight v4 now checks the controller-selected directed
span and actual tool-axis pregrasp, rather than an arbitrary axis sign and a
vertical pregrasp. Initial case13 remains successful; broader results pending.
An isolated v5 re-scoring IK multistart branches with configured task costs
(instead of the solver's module constants) regressed case13 to neighboring-piece
contact. It was rejected and remaining v5 development cases intentionally
cancelled; preserved completed failure is not erased or counted as success.
No such solver scoring change was applied to production source.

Approach-preflight v4 completes **1/3** focused cases:13passes,206/246 still
abort on neighboring-piece contact after their thin-axis candidates fail
preflight and the existing broad alternative is executed. Zero collateral; no
commit on failure. It corrects a diagnostic mismatch but does not itself solve
the two remaining approaches. A private adaptive-upright experiment evaluates
5/2/1 task weights against the same position/tilt preflight checks before any
physical attempt; no verdict thresholds are changed.

## Observation-return safety repair (2026-11-06, development only)

Exact seed 20261113 cases 65 and 73 (`g8f6`, board friction ×0.75) reproduce
candidate7's collateral failures. Full control traces locate the **first**
robot/pawn contact in the return from wrist re-observation, not in jaw closure.
The return inherited the `vision_reobserve_park` diagnostic phase and used an
8-step joint interpolation. The measured TCP dipped below the nominal high
waypoint through the pawn rank; that phase was absent from the contact guard.
Case 65 aborted before any `close` phase. Case 73 subsequently closed on air
and aborted during lift. The scorer's `picked=g7` means the pawn had the largest
final XY displacement; it does **not** establish that the pawn was gripped.
The original reported wrong-piece/collateral counts remain failed qualification
evidence; no historical receipt or success definition was rewritten.

The repair labels the return as `transit`, uses the same 22-step budget as the
initial transit, checks contacts during observation parking/transit/closure,
and stops joint parking on its first neighboring-piece contact. It changes no
mesh, friction, localization gate, contact physics or outcome tolerance.

Exact replays now complete both cases: placement errors 0.654/0.656 mm,
`picked=g8`, released/supported/settled, zero wrong-piece/collateral/transient
events. Receipts and integration traces are
`.private/phase10/{repro,trace}-resume-{65,73}-{baseline,return22}.json`.
Focused controller/abort tests: **23 passed**. The full pre-repair baseline was
138 passed, 6 skipped, 4 failed: two stale source-hash Phase 8 receipt audits
and two slip tests still using the superseded 4 mm guard. The slip tests now
exercise the configured measured guard; receipt audits require fresh runs,
not altered hashes or relaxed assertions.

The prior 42-case development bank and eight remaining candidate7 failures are
being rerun. These cases are now development evidence, not fresh held-out
qualification. Phase 10 and its roadmap entry remain open pending all exit and
release gates, including paired fresh cases, compound families, wrist
self-occlusion turn resolution and source-matched Phase 8 receipts.

The remaining-failure rerun is now complete: the same return repair also
rescues 47 (`place_descend`), 114 (`grasp_descend`), 140
(`lift_waypoint_miss`) and 72 (`gripper_neighbor_contact`). Together with
65/73 that is **6/10 prior failures rescued**, zero collateral in all ten
replays. The remaining four are 37 (`carry_unstable`), 102/156 (localization
uncertainty), and 104 (no clear span). An isolated 15-degree span fallback
still refuses 104, preserving all gates; it is not promoted.

Case 37's saved trace distinguishes a pivot from translational slip: maximum
base drift 6.329 mm, fixed material-patch drift 0.632 mm and relative rotation
11.53 degrees (including post-abort hold). Before the abort the carry solver
reports 39.36 mm position residual and 32.94 degrees tilt. The default
130 mm transit is therefore suspect for this near-edge queen, rather than
insufficient pad friction. A private queen-only 90 mm transit probe preserves
the contact, slip, placement and release gates; it is not a production default.

Post-repair full regression run: **143 passed, 6 skipped, 2 failed**, both stale
Phase 8 source-provenance audits. Additional boundary cases separately verify
4.9 mm drift remains allowed and 5.1 mm aborts in both lift and carry; no
controller tolerance changed in this repair. The frozen runtime manifest is
`.private/phase10/observation-return22-freeze.json`. Do not mix later source
changes into the running development bank or label its partial result an exit
qualification.

## Release-anchored withdrawal and exit-gate audit — 2026-11-06

The production queen transit calibration is now pack-owned (`transit_m=0.11`),
not a global grasp-height change. Six queen sentinels passed. The subsequent
production development bank completed **42/42 transactions**, with zero
wrong-piece, collateral or transient events (`.private/phase10/prod-final-42/summary-verified.json`).
That is development evidence, not an exit qualification.

The next fresh candidate bank stopped after **13 valid receipts: 12 complete,
one failure**. Seed 20261114/index 10 (`a8b8`) displaced the b7 pawn by
10.948 mm during retreat; commit was denied. The final collateral and transient
counts describe the same pawn, not two different pieces. Preserve
`.private/phase10/fresh-candidate/case-20261114-010.json` as failed qualification
evidence. Its pool is now consumed development data.

The saved replay localizes first contact to the first withdrawal command after
successful placement/release: reversing the sampled descent from its stale
endpoint dips the jaws about 5 mm and shifts them about 6 mm. A private probe
anchors the descent's joint deltas to the measured release posture, using
ordinary servo dynamics and unchanged contact guards. It rescues the exact
rook case and passes **8/8 contrasting sentinels** (1,13,134,206,246,298,111,258),
zero collateral/transient events. This correction is ported into the production
controller; focused retreat/waypoint/compound checks **17 passed**. Production
replay and wider source-matched qualification remain required. Diagnostic
sentinel provenance predates a benchmark-only inference-hash expansion; do not
present these as a frozen qualification bank.

The preceding-controller CAD Phase 8 rerun completed **64/64 first-pass paths,
25/25 crowded cases and 4/4 distinct tray slots**, `phase8_pass=true`;
`.private/phase10/phase8_final2_cad.json`. The comparator stock tool scored
51/64 feasible paths, 8/25 crowded and 3/4 trays (`phase8_pass=false`). Both
receipts are historical once the controller changes and must not have their
hashes edited or be installed as current evidence.

Independent audit confirms exit gaps: the previous selected 300 omitted
castling/en-passant, had no paired control receipts, and contained 296 distinct
FEN/UCI combinations; the compound generator was not seed-dependent. At that audit, turn
inference lacked the queued joint/calibration-projected self-model mask, and
fault receipts predated the current runtime and omitted physical en-passant
interruptions. Subsequent corrections and measurements are recorded below. Phase 10 remains **active and
unqualified**; no roadmap completion or deployment is claimed.

## Interrupted-run recovery and physical self-contact diagnostics — 2026-11-06

The recovered full regression suite is **179 passed, 6 skipped**. Castling
receipt fixtures now use the actual three/four-primitive plan; historical
Phase 8 receipt tests verify their immutable publishing revision rather than
misrepresenting those receipts as qualification of today's modified runtime.

A private replay of frozen seed 20261207/index 17 (`d1d4`, sensitivity-high)
adds the physical gripper-base/camera-mount hulls to shoulder/upper-arm planning
clearance, retaining the existing limits. Both primitives complete; placement
error is 2.759 mm, with no wrong-piece or collateral events and zero robot
self-contact samples across 798 recorded control steps. This is a bounded
single-case rescue, not substep contact-free proof or production qualification.

The six retained sentinel identities finish **4/6 complete** (9, 35, 37, 258).
Case 2 is a king move and case 37 a pawn move: neither exercises the queen-only
candidate. Case 111 refuses uncertain localization before the candidate activates.
Their historical baselines differ in controller, scanning and benchmark source;
current-source matched controls are required before attributing regressions.

The case-2 trace identifies an independent recovery defect. Shoulder/base contact
starts during approach, the intended king is gripped, and its lift falls 37.255 mm
short. The ensuing `abort_return` sweeps that held king into the e2 pawn. Final
`picked=e2` is a largest-displacement classification, **not proof the pawn was
initially gripped**. The final and transient collateral counters refer to the same
pawn. Existing recovery phases lack the neighboring-piece guards used in normal
lift/carry. Private self-contact containment and king-clearance probes are prepared;
no recovery fix or broadened clearance policy is yet adopted.

In frozen castling index 20, placement-only self-contact containment denies commit
without wrong-piece or collateral events. Adding rook planning clearance completes
the first three primitives but refuses final king localization; 105 robot
self-contact records across 25 traced control samples remain. Neither variant completes the castle or qualifies a
contact-free plan. All private probe execution exits are terminal; a successful
process exit is not a successful chess transaction.

The twelve missing development transactions and interrupted paired crowded-board
tests are now recovered in new artifact paths; runtime source remains frozen
while the queued matched controls and king probes finish.
The reserved paired >=300-case matrix is not consumed by these diagnostics.
Phase 10 remains active; its roadmap completion and release gates are unchanged.


### Recovered paired self-occlusion measurement

Exact seed 9312, eight crowded sessions and 25 turns each, using the current
v9 printed assets: **195/200 resolved (97.5%) with joint-derived masking**,
versus **189/200 (94.5%) without masking**. Both runs have **zero false
confirmations**, **29/29 safe invalid-case rejections**, and **8/8 registrations
within 2 mm / 1 degree**. This passes the stated 95% crowded perception target
for this frozen stimulus; it is not manipulation qualification or a statistical
claim of general improvement. Receipts: `.private/phase10/recovery-20261106/`
`crowded-{masked,control}.json`, audited together in `banks-summary.json`.

Recovered development manipulation is **58/60 complete (96.667%)**, with
58 commits and zero wrong-piece, collateral, transient, unsafe-commit or
unverified-completion events; localization p95 3.606 mm, max 4.683 mm. The
queen17 and castle20 failures remain. This is unpaired consumed development
evidence, not the reserved >=300-case qualification matrix.

The unadopted private safety candidate has **82 CAD tests passed**, and
**81 stock tests passed / 1 CAD-only skip**. These include physical pair scope,
entry/step abort latching, recovery-neighbor protection and bounded placement
bias. No candidate physical execution or production safety claim follows from
these unit/kinematic tests. Initial interpolation and scan motion remain
separate coverage gaps; exact-case replay and independent review are pending.


### Scan self-contact confirmed; expanded private candidate

The command-preserving exact castle20 observation replay captures **1,062
robot self-contact records across nine scan motions**, all at v4. Maximum
normal force is **16.724 N**, penetration **2.413 mm**, and final joint error
**0.45447 rad**. The isolated authored-geometry check finds v4's commanded
camera-mount endpoint intersecting the shoulder by **12.743 mm** (other five
rigid base/arm endpoints are beyond the 30 mm query cap). These are distinct
static and dynamic measurements, not interchangeable penetration estimates.
Evidence: `castle-scan-contact-audit.json`, `scan-endpoint-base-clearance.json`
and `castle-final-observations/` in the private recovery directory. The former
uncertain scan-collision hypothesis is now directly evidenced.

Thus the earlier paired perception score is a dated accuracy sub-gate, **not
collision-free scan qualification**. The scan plan needs physical screening
and containment, and perception must be remeasured after that change.

The expanded, still-unadopted candidate screens rigid scan endpoints using
the unchanged 5 mm robot-clearance margin (not image confidence), monitors
physical scan contacts, validates nonlinear IK endpoint clearance, treats
CAD reach-query infeasibility as unavailable and retries only the existing
current/flipped branch choices independently. It passes **117 CAD tests**
and **114 stock tests / 3 CAD-only skips**. Exact king/queen/castle replay is
active. No production source edit, completion promotion or deployment yet.

Matched frozen probes are terminal: nominal queen17 completes with zero
recorded self-contact/collateral; current king2 control reproduces recovery
collateral, containment safely refuses with zero collateral, and king-only
base-clearance completes with zero recorded self-contact/collateral. Queen111
control refuses localization before motion. These are bounded development
comparisons, not the combined-candidate result.

### Knight wrist-axis fit and controlled yaw sweep — 2026-11-07

The capability smoke found one safe failure on a crowded black-knight capture
at c7-b5: the jaw contacted the adjacent black queen during approach, before
the knight was picked. The victim-removal primitive had completed; no piece was
misidentified or displaced. Trace:
`.private/phase10/capability-capture-c7b5-trace.json`.

With the wrist-view mesh-axis fit enabled, a controlled 0/45/90/135-degree yaw
sweep on that same crowded capture stimulus completed **4/4**. Every case
removed the victim and placed the intended knight, with release/support/settle
true, zero wrong-piece picks, zero collateral and 0.19–0.63 mm knight placement
error. The fitted axis matched the simulator-only audit yaw within 0.0004° for
these rendered, grid-aligned test poses; fits had 5° sigma, confidence 0.99 and
three or four supporting views. Evaluation-only post-settle knight yaw was
within **0.10° of a board axis** for all starts. Receipts:
`.private/phase10/c7b5-yaw-gated-v9-{0,45,90,135}.json`.

This establishes a narrow controlled-scene orientation integration check, not
general recognition accuracy or physical-camera performance. All cases share
one source square, destination, black knight, crowded layout and source hash
(`tools/benchmark_phase10_compound.py` SHA-256
`8a9d147bcff50ae6dca74da7f2d7b0ec872fb898d72df545cadfebd0f9ed12b8`). The
earlier randomized candidate/control pair both completed, so it is not a
fixed-pose causal ablation. Sol review found the benchmark's hint gate looser
than the estimator contract; this is corrected to call
`AxisEstimate.acceptable()` (sigma <=15°, confidence >=0.7, >=3 views). The
refreshed receipts use this strict gate and the audited cad_parallel v3 /
printed v9 runtime, and the focused orientation tests pass 6/6. The estimator
returns an axis modulo 180°, not knight head/tail direction. Straight final yaw
does not prove the held piece avoided angular slip: the current slip guard only
checks translation. The controller's placement span switch, missing neighbour
guard in `place_approach`, relative-yaw/tilt monitoring and swept-path checks
remain open. Expand over both colors, measured hard/far source and destination
squares, deliberate sparse/crowded boards, non-grid yaw and uncertain/occluded
images. No roadmap completion or deployment follows from this result.

### Explicit-square manipulation pilot

The legal transaction generator cannot guarantee all actor/color/source and
destination squares or balanced density. A separate spatial-only fixture now
generates 1,536 combinations (six piece types, both colors, all 64 board squares
as both source and destination, sparse and crowded); the transfer pairing is
adjacent and does not assume physical board symmetry. Fixture tests check the
full matrix, inventory bounds and square pairing.

On the center c3 subset, all **24/24** cases passed (0.19–1.62 mm placement,
zero wrong-piece/collateral). At four crowded corners, **46/48** passed
(0.39–6.20 mm placement, zero wrong-piece/collateral). The two safe aborts
occurred on retreat: white knight h1→h2 touched black rook h3; black king
h8→g8 touched white knight g7. Contact traces are stored at
`.private/phase10/capability-square-corner-failures-trace.json`.

For those exact failures, calibrated narrow opening/release without an axis
hint enabled the h8 king move, but h1 knight release/support validation still
failed and final yaw was 21.38°. Oracle span hints enabled both commits, though
they also narrow the aperture; the h1 final yaw varied from 0.38° to 10.08°.
This points to coupled retreat-clearance and orientation/span requirements,
not a proven single-factor fix. These are development-only receipts; retain
contact guards and keep the roadmap gate open while the full square matrix and
perception/fault/release gates are developed.

The first matrix version exposed a test-fixture flaw: its “sparse” label counted
pieces but did not keep source/destination neighbor rings clear. The v1 sparse
black-pawn e1 case had a filler on adjacent d2 and aborted; the same move passed
at 0.22 mm after v2 enforced clear endpoint neighborhoods. V2 now deliberately
occupies both endpoint neighborhoods for crowded layouts and leaves both clear
for sparse layouts; generator tests enforce the distinction. V1 receipts are
preserved but excluded from v2 rates. V2 sparse pawn coverage is complete at
**128/128** across both colors and all 64 squares. Sparse white knights are
**64/64** successful pick/place commits, but the separately measured final
board-axis residual exceeds the provisional 5° straightness indicator at h1,
b5, e3 and f3 (maximum 12.43°). Move commit does not enforce the requested
straight-knight result; residual-yaw needs calibration and its own pass gate.
