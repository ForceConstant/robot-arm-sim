# FULL-PARITY-01 — first pass (2026-11-25)

**Result: BLOCKED — T1–T7 run; T8/T9 not run. Not a pass.**

Host: bc250-cachy (CachyOS), MuJoCo physics on CPU.
Head: T1–T4 at `805dec0a`; T5 at `c13dae4`/`d208482` (runner only; `run_sim` unchanged).
Rig state: `MATCHES the manifest` before and after; zero drift lines.

## Ran

| Step | Result |
| --- | --- |
| T1 source-tree consistency | **PASS** — board exact (−27.62/−31.63/432.65/184.32°), `ik_solver` mink, tolerance 3.0 mm, `TIP_IN_CHAIN` (0,0,−0.11045) |
| T2 full suite (stock gripper) | **11 failed / 221 passed / 42 skipped** — exactly the documented §4b set |
| T2b frame pins + companions (5y pack) | **44/47** — 4/5 literal pins; the 3 fails are the documented ones |
| T3 model parity | **PASS** — 5292 samples, worst Δ 0.000000 mm (tol 0.1), rot 3.4e−6° |
| T4 sim 64-square (bridge limits) | **64/64 solved**; tip err p50 4.21 / p95 10.29 / max 12.83 mm (prior sim run 3.70 / 12.30 / 14.92) |
| T4b phase2_squares (MoveSolver) | 20 mm gate mink 61/64, 3 mm gate 61/64; the 3 are the far corners over the 8° approach gate |
| T5 IRL 64-square (3 passes, merged) | **60/64 solved**; tip err p50 4.68 / p95 8.19 / max 9.28 mm; settle p50 1.09°; roll delta 0.0°; 4 not solved (c8, d8, e8 refusals; f8 bridge timeout after moving) |
| T6 paired compare | **60/64 agreement**; mismatches c8, d8, e8, f8 (sim solved, IRL refused/unconfirmed) |
| T7 rig re-verify | **PASS** after motion; arm homed, tip (6.6, −12.3, 111.7) mm = 14.0 mm from centre |

## Not run (blockers)

- **T8** — needs external front/side/top photographs of the real arm; the rig has no external camera (wrist camera + Pi bridge only).
- **T9** — needs gripper actuation, which is still a specific ask under the standing motion approval.

## Plan gaps found on the first run

1. **Gripper pack is undocumented.** T2b/T3 require `ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y`. The launcher default (`.private/gripper-assets`) is the pre-5y pack and produces **6 false tool-orientation failures** with a systematic 10 mm tip / 8 mm jaw offset — exactly the kind of "clean evidence from the wrong pack" the plan warns about.
2. **T3 does not run on the stock gripper** — `KeyError: so101:moving_jaw_so101_v1`. It needs `ROBOT_GRIPPER=cad_parallel` + 5y.
3. **Printed-piece tests** need `ROBOT_PIECE_SET=printed` + `ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight`; otherwise one raises `ValueError` and another sees `grasp_squeeze=None`.
4. **§4b's stale rig-state pin is described backwards.** The test constant expects `4c28d879…`, while `MANIFEST.json` and the committed snapshot are both `8c8303a1…`. The manifest is fine; the test literal is stale.
5. **T4's "solver gate 3.0 mm" is not enforced** by `cartesian_squares` — it solves 64/64 with residuals up to 12.83 mm. The gate is a config value, not a pass criterion in that tool.
6. **pytest's final totals line is missing** from both suite logs even on exit 1; counts were taken from `--junitxml`.
7. **Stop-on-409 conflicts with the T5 baseline.** The recorded 61/64 run includes 3 refusals, which requires continuing after a refusal; §0.6 says stop on a 409. Decision needed.
8. **The local pre-check is advisory.** c8 was predicted at 0.74 mm / 5.3° and the app refused at 1.83 mm / 12.284°; the app's gate is authoritative.
9. **The Pi bridge can time out mid-move.** f8 returned HTTP 500 (`/command` read timed out) after the arm had reached the target (observed tip 53.6, 95.3, 59.8 vs 42, 98, 60). The app stayed healthy; the sweep stopped; the arm was re-homed. Treat a 5xx as *unconfirmed*, not a refusal.

Artifacts: `reports/full_parity_20261125_firstpass.json`, `reports/phase1_parity_fullparity01.json`,
`reports/squares_sim_fullparity01.json`, `reports/phase2_squares_fullparity01.json`,
`reports/squares_irl_fullparity01.json` (merged 64 rows), `_partial/_rest/_tail.json`,
`squares_irl_fullparity01.log`.
