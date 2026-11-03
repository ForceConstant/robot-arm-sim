# Board/base layout optimization — 2026-11-03

## Selected current simulator layout

- **90 mm outer board edge to arm base-frame origin; 30 mm riser/mount-datum height.**
- Board underside/table at 430 mm; arm datum at 460 mm; playing surface at 433.2 mm.
- Base frame position: white `(0, -217, 460)` mm; optional two-arm debug fixture mirrored `(0, 217, 460)` mm.
- The gap is NOT measured from the front edge of the plastic foot or the shoulder-pan axis. The CAD bottom-band front-foot clearance is approximately **25.4 mm**. Verify against the physical mount before printing a jig.
- Current custom jaw: `4dca0986`, aperture 0–38 mm. Board: measured 28 mm pitch, 15 mm border, 3.2 mm slab, 254 mm overall.
- Runtime defaults and `hardware/current_layout.json` describe this layout. `hardware/product_twin.json` and its Phase 8 report retain the historical stock-tool baseline, not a newly qualified custom-tool product contract.

## Method

Execution, not IK alone: each candidate runs eight independent single-follower physical/contact tasks. Close kings at e1 and d1; close rooks a1→h1 and reverse; far rooks a8→h8 and reverse; diagonal rooks a1→h8 and reverse. Cases are physical probes and need not be legal chess moves. Same measured board, printed geometry, evaluator, CAD revision and controller on every candidate. Imported assets stay private; report hashes pin the piece manifest and controller.

Rank by passed cases, then worst placement error, then mean error; lower riser/gap breaks ties. The coarse grid is gaps 60/80/100 mm × heights 0/20/40/60 mm. Refinement is gaps 90/100/110 mm × heights 30/40/50 mm. This gives **20 distinct layouts and 168 critical-case executions** (100/40 repeated for consistency), not a global continuous optimum. QP-infeasible candidates remain named failures, not exclusions.

The custom-tool point cost is 30 rather than 1 (tilt remains 0.5): distances are in metres, whereas alignment errors are unitless. Earlier weighting stopped 28.6 mm short of A8 to stay vertical. The corrected execution can tilt within the existing 30° grasp cone. Joint limits, velocity limits, self-collision constraints, physical contacts, friction and success thresholds are unchanged. Failed pickup descents now retreat open instead of closing near the wrong square.

## Nominal search results

| Gap mm | Riser mm | Passed | Worst error mm | Mean error mm |
| ---: | ---: | ---: | ---: | ---: |
| 60 | 0 | 6/8 | 23.07 | 5.66 |
| 60 | 20 | 6/8 | 28.02 | 8.26 |
| 60 | 40 | 6/8 | 28.00 | 8.41 |
| 60 | 60 | 5/8 | 113.45 | 21.86 |
| 80 | 0 | 6/8 | 27.33 | 8.82 |
| 80 | 20 | 6/8 | 28.00 | 8.69 |
| 80 | 40 | 5/8 | 48.98 | 16.14 |
| 80 | 60 | 6/8 | 22.46 | 6.76 |
| 90 | 30 | 8/8 | 4.90 | 2.93 |
| 90 | 40 | 8/8 | 8.71 | 4.22 |
| 90 | 50 | 7/8 | 15.23 | 7.60 |
| 100 | 0 | 6/8 | 15.77 | 5.97 |
| 100 | 20 | 7/8 | 27.54 | 6.42 |
| 100 | 30 | 8/8 | 9.81 | 5.28 |
| 100 | 40 | 8/8 | 9.99 | 5.89 |
| 100 | 50 | 4/8 | 264.89 | 62.64 |
| 100 | 60 | 6/8 | 266.52 | 59.52 |
| 110 | 30 | 8/8 | 10.22 | 5.03 |
| 110 | 40 | 5/8 | 28.00 | 9.06 |
| 110 | 50 | 7/8 | 11.42 | 4.06 |

## Qualification and limits

- Selected layout: **8/8 critical cases**, worst 4.90 mm, mean 2.93 mm.
- Every source square: **64/64 adjacent transfers**, alternating rook/king, worst 7.02 mm, mean 1.84 mm. This is not all six piece types on every square.
- Four board-offset corners (±2 mm in x and y): **29/32**. All failures are retained below; nominal success does not imply repeatability under placement changes.
- Only the rook has a measured assembled mass (4.8 g including its M6 nut). Other masses and inertial models remain assumptions. The live printed scene uses the same product mass model for all 32 slots, including parked pieces.
- No hardware movement, servo access, purchases, attachment tricks, teleportation or widened evaluator tolerances.

- Offset [2.0, -2.0] mm: `near-king-d` failed, 18.24 mm error, placement error.
- Offset [-2.0, -2.0] mm: `near-king-e` failed, 19.11 mm error, placement error.
- Offset [-2.0, 2.0] mm: `near-rook-left` failed, 227.02 mm error, placement error.

## Repeat the tests

Supply private packs through `ROBOT_GRIPPER=cad_parallel`, `ROBOT_GRIPPER_ASSETS` and `ROBOT_PIECE_ASSETS`. Run:

```bash
MUJOCO_GL=disabled .venv/bin/python scripts/optimize_layout.py --output reports/layout_search_coarse.json
MUJOCO_GL=disabled .venv/bin/python scripts/optimize_layout.py --gaps 90,100,110 --heights 30,40,50 --output reports/layout_search_refined.json
MUJOCO_GL=disabled .venv/bin/python scripts/optimize_layout.py --gaps 90 --heights 30 --full-board --output reports/layout_full_board.json
```

The deployed UI has **Layout: 16 cases**: the eight white-piece probes plus mirrored black-piece probes, all using the same physical follower, with real incident recordings and per-case verdicts. `GET /api/suite?name=layout` lists them; `POST /api/control` with `{ "action":"suite", "suite":"layout" }` starts them. The original 25-case `crowded-v3` suite remains separate. Status exposes current mount geometry; `scripts/verify_layout.py` verifies it against the live build.

Reports: [coarse](../reports/layout_search_coarse.json), [refined](../reports/layout_search_refined.json), [all squares](../reports/layout_full_board.json), [offset sensitivity](../reports/layout_robustness.json).

## Regression scope

The original six-move sequential crowded-opening regression is pinned to its original 120 mm gap / 60 mm riser / 12 mm board fixture. Changing a default must not silently redefine that historical test. At the new layout, the sequential `f1c4` bishop transfer slipped during preflight; empty-board reach is not evidence of crowded-game reliability. This failure is retained here, and the separate 25-case crowded suite is reported independently of layout-v1.

The release also retains a valid primary IK solution when an alternate wrist-roll seed has an infeasible collision-avoidance QP; it does not remove constraints or relabel the infeasible alternate as a solution. Search receipts pin the controller/solver versions used for those measurements.

## Runtime scene alignment

The deployment now sets `ROBOT_ARM_COUNT=1`: one actual model, six joints, one wrist camera. Both piece colours are manipulated by that same follower. The old two-arm model remains selectable as a debug fixture by omitting the flag. Its parked opposite arm obstructed far-board probes, so dual-arm collisions are not used to grade the single-follower mount. No robot is merely hidden visually or exempted from contacts: the single-follower scene contains exactly one arm.

Local runtime preflight on the actual single-follower adapter: **16/16 layout-v1 cases passed**. Stock public regression: **102 passed, 5 skipped**; custom-pack regression: **107 passed**. Live receipts are recorded separately.

## Final live verification — 1.12.1

- Deployed commit `b6437d2`, NVIDIA GTX 1060 renderer; one actual follower, six joints and one operational wrist camera. Live status confirms 90 mm setback / 30 mm riser / 3.2 mm board.
- **Layout button: 16/16 passed**, with clean full-board and post-parking checks. Tested by clicking the delivered browser control, not merely inspecting the handler.
- **Crowded-v3: 21/25 physical actions passed**. White queen, white king, white queen-far and white king-far disturb neighbours. This remains a clearance/grasp-reliability limitation, not solved by nominal reach optimization.
- Browser checks passed: actual joints, Cartesian jog, wrist/gripper keys, start/pause/resume/stop/reset, installed camera JPEGs, wrist-only sensor contract, CSRF and mobile overflow.
- Local tests: custom **107 passed**; public/stock **102 passed, 5 skipped**; single-adapter follow-up **14 passed**.
- The first deployment smoke test found a missing full-board verifier on the single adapter; 1.12.1 restores the same verifier used by the debug fixture. Final live layout cases include it and post-parking checks. The 1.12.0 release is superseded.

Sanitized live receipt: [layout_live_verification.json](../reports/layout_live_verification.json). No physical arm was moved; no unrelated stack was changed.
