# Phase 8 rerun at the updated offsets — 2026-11-03

## Recommendation

**Keep the current NormaCore parallel gripper geometry. Do not redesign or
reprint fingers on the basis of the old Phase 8 reach failures. Improve grasp
span selection first.** The updated mount and current controller remove the
custom jaw's old board-coverage objection. Crowded queen/king failures can be
fixed by a parameter-only experiment with the identical tool/contact model.

This is simulation evidence, not physical qualification or proof of robust
operation under board offsets, unknown piece masses or arbitrary game states.
The live v1.12.1 controller and its 21/25 crowded receipt are unchanged.

## Matched inputs and method

- One white-side follower handles both colours. Board: 28 mm pitch, 15 mm
  border, 3.2 mm slab, 254 mm overall. Table datum 430 mm.
- Mount: **90 mm board-outer-edge to base-frame origin, 30 mm riser**;
  base frame `(0, -217, 460)` mm. This is not a 90 mm plastic-foot clearance.
- Private printed pack v5 and current CAD gripper pack v3; source microversion
  `4dca0986f24304c301118aa4`. Reports record manifests and controller hashes.
- Same Phase 8 nominal mass model for both tools: rook measured 4.8 g including
  its M6 nut, other pieces unmeasured nominal PLA. No fabricated assembled masses.
- Same evaluator: 11 mm placement, 30 mm tray, 10 mm neighbour displacement,
  upright alignment above 0.85. Real contacts, no attachment/teleport shortcuts.
- Repeat the original empty/crowded 64-square × six-type advisory path scans;
  execute the original deterministic mixed-type source-square matrix, then
  retry every failure with king and rook. A 64/64 path result is **not** all six
  pieces physically succeeding on every square.
- Add the 25 independent crowded-v3 physical tasks with CRC32 case seeds.
  Geometry scans remain advisory; actual execution is the Phase 8 path gate.
- Compare each tool with its currently implemented controller calibration:
  position cost **1 for stock, 30 for custom**, tilt cost 0.5 for both. These
  are configured-system comparisons, not an isolated proof of mechanical
  superiority. A stock controller retuning experiment was not performed.

## Board execution

| Tool/controller | First pass | Paths after king/rook retries |
| --- | ---: | ---: |
| Current CAD parallel jaw | 62/64 | **64/64** |
| Stock SO-101, native controller | 58/64 | 60/64 |

Both tools pass **4/4 actual tray destinations**, at `(-160, -100)`,
`(-160, -60)`, `(-160, -20)`, `(-160, 20)` mm. Custom tray errors are
1.96, 1.17, 0.64 and 2.48 mm. **Custom passes the bounded Phase 8 execution
gate; stock with its native controller does not at this mount.**

Default crowded execution is **21/25 custom**, **18/25 stock**. The four
custom failures are the white queen/king short/far cases. The direct runner's
far-king result is a placement failure, while the saved live receipt labelled
it neighbour disturbance; preserve both receipts rather than inferring the
historical outcome from a fresh run.

Advisory scans still have residuals: custom empty/crowded squares **51/64 and
49/64**, tray **4/4** in each; stock **48/64 and 51/64**, tray **3/4** in each.
Every scan row requires all six types to pass; this stricter advisory count is
not the mixed-type execution gate. Named residuals remain in the JSON reports,
and the successful non-cardinal spans below demonstrate one heuristic limit.

Custom first-attempt failures: `c1` knight and `f2` rook. Each square passes with
a king retry; the `f2` rook retry still fails. These remain grasp/repeatability
issues, not board exclusions. Stock's unrecovered squares are `a7`, `h7`, `a8`,
`h8`; this does not establish an immutable hardware limit.

## Crowded failures: parameter change, not new fingers

The 14-run diagnostic sweeps use fresh identical nominal starts (seed 0), not
replay of historical incident states. For `d1d3` and `e1e3`, baseline, grasp
height ±4 mm, and place-y ±2 mm fail; forcing jaw spans at either **−20° or
+20° absolute world XY angle from +X** passes without disturbed neighbours.
These are absolute span settings, not rotations relative to an automatically
selected angle.

The full candidate rerun uses exactly the baseline's 25 FENs and CRC32 seeds.
Only `white-queen`, `white-queen-far`, `white-king`, `white-king-far` receive a
forced +20° span; all other cases retain default parameters. **25/25 pass**.

| Corrected case | Placement error | Disturbed neighbours |
| --- | ---: | --- |
| Queen d1→d3 | 1.17 mm | None |
| Queen d1→h5 | 1.64 mm | None |
| King e1→e3 | 1.28 mm | None |
| King e1→f5 | 2.34 mm | None |

The existing span selector considers only four cardinal directions and uses a
conservative 2-D clearance model. `grasp_span() == None` is therefore not proof
that no physical span fits. Successful intermediate-angle contact executions
disprove that conclusion for these nominal cases. Do not turn this experiment
into a global +20° override: a general planner must validate candidate spans
against the actual piece/neighbour envelope and retain collision/abort guards.

## Reproduce and inspect evidence

Regression checks: public/stock **102 passed, 5 skipped**; current private
custom/printed pack **107 passed**; new receipt audits **3 passed**, including
distinct tray coordinates, complete square coverage and matched candidate
seeds/overrides. An initial custom test invocation accidentally used the default
piece set and failed the printed-knight profile assertion; rerunning with
`ROBOT_PIECE_SET=printed` passed all 107 tests. No test or tolerance was relaxed.

For each tool, set `ROBOT_GRIPPER` to `stock` or `cad_parallel`, and supply
`ROBOT_GRIPPER_ASSETS` and `ROBOT_PIECE_ASSETS` pointing at the private v3/v5
packs. Run from the repository root:

```bash
scripts/ensure_venv.sh
MUJOCO_GL=egl .venv/bin/python scripts/requalify_phase8.py
# The following two experiments require ROBOT_GRIPPER=cad_parallel:
MUJOCO_GL=egl .venv/bin/python scripts/diagnose_phase8.py
MUJOCO_GL=egl .venv/bin/python scripts/verify_phase8_spans.py
```

- [Custom jaw matrix](../reports/phase8_updated_cad_parallel.json)
- [Stock matrix](../reports/phase8_updated_stock.json)
- [14-run parameter sweep](../reports/phase8_updated_parameter_sweep.json)
- [Full crowded candidate verification](../reports/phase8_updated_span_verification.json)

Reports record the base Git revision plus hashes of the working experiment
scripts and controller. The original dated product twin remains untouched.
No runtime release, deployment, hardware access or CAD edit is part of this
qualification-only task. Phase 9 remains next; generalized grasp/reliability
work belongs to Phase 10.
