# Phase 8 product test matrix

Current-mount rerun and gripper recommendation:
[PHASE8_UPDATED_OFFSETS.md](PHASE8_UPDATED_OFFSETS.md). Use
`scripts/requalify_phase8.py` for actual distinct tray destinations; the old
runner's reset loop repeated slot zero. The dated matrix below is historical.

**Frozen 2026-11-02.** Commands assume the repository root and the project venv.
`ROBOT_PIECE_ASSETS` points at the private printed pack; geometry is not
redistributed. The stock gripper is selected (see
[PRODUCT_CONTRACT.md](PRODUCT_CONTRACT.md)).

| ID | Check | Command | Pass criterion |
| --- | --- | --- | --- |
| P8-01 | Product config validates against code | `pytest -q tests/test_product_twin.py` | All contract tests pass |
| P8-02 | Measured board | `pytest -q tests/test_product_twin.py::test_measured_board_and_single_follower` | 28 mm pitch, 3.2 mm slab, 15 mm border, 254 mm overall, one follower |
| P8-03 | Wrist-only observation and evaluator | `pytest -q tests/test_product_twin.py::test_product_env_is_wrist_only_and_uses_frozen_evaluator` | Operational camera `wrist`; 11/30/10 mm and 0.85 upright thresholds |
| P8-04 | Rook measurement and unknown ranges | `pytest -q tests/test_product_twin.py::test_rook_measurement_and_unknown_piece_ranges` | Rook 4.8 g incl. nut; other pieces explicit ranges |
| P8-05 | Tool decision | `pytest -q tests/test_product_twin.py::test_tool_selection_is_explicit` | Stock selected; custom tool rejection recorded |
| P8-06 | Full path/clearance map | `ROBOT_PIECE_ASSETS=... MUJOCO_GL=egl .venv/bin/python scripts/qualify_phase8.py` | 64 squares × 6 types and 4 tray slots mapped; no silent square exclusion |
| P8-07 | Every square executes | same command | 64/64 squares have a successful pick/place with at least one piece type |
| P8-08 | Tray destinations execute | same command | 4/4 tray slots succeed |
| P8-09 | Crowded-start neighbour clearance | same command | Jaw-span clearance and executable-move inventory recorded; failures named |
| P8-10 | Report completeness | `pytest -q tests/test_product_twin.py::test_committed_qualification_report_is_complete_when_present` | 64 square keys, 64 execution rows, 4 tray rows, `phase8_pass: true` |
| P8-11 | Simulation regression | `MUJOCO_GL=egl .venv/bin/python -m pytest -q tests` | Existing suite passes; no loosened thresholds |
| P8-12 | Docs/source consistency | `git diff --check` and the above | No whitespace errors; roadmap/contract/report agree |

## Downstream inputs (not Phase 8 passes)

- **Phase 9:** wrist-only board registration, full-board observation, opponent
  move inference; current fixed-overhead/37.1% coverage does not pass.
- **Phase 10:** pawn-grasp reliability at `g1`, `a4`, `g4`, `e5`, `c6`, `e8`
  (the six first-pass failures in the execution sweep; those squares pass with
  king/rook, so they are grasp-reliability cases, not reach exclusions).
- **Phase 11/12:** ACT runtime/training; no learned-policy claim here.
- **Phase 17/18:** physical fixture, camera/arm calibration, guarded real
  manipulation. This phase does not touch the real arm.
