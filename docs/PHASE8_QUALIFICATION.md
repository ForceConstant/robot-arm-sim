# Phase 8 — product twin qualification

**Historical 80/60 stock-tool report.** The [updated-offset rerun](PHASE8_UPDATED_OFFSETS.md)
supersedes this report for the current 90/30 mount. Audit correction: the old
`tray_execution` reset the capture list each time and therefore repeated slot
zero despite labelling rows 0–3. Its 4/4 is not evidence of four distinct actual
destinations. The new rerun asserts and records each actual destination.

Generated 2026-11-02T23:49:40.861813+00:00 from `887a7a60ba94`.

**Phase 8 verdict: PASS.** The product twin is the frozen
single-follower scene, not a learned policy or a physical arm.

## Frozen inputs

- Board: 28.0 mm squares, 3.2 mm thick, 254.0 mm overall.
- Arm: one SO-101 follower, base at [0.0, -207.0, 490.0] mm, 30 Hz control.
- Tool: Stock SO-101 pinch gripper.
- Operational camera: wrist.
- Evaluator: env.evaluate: destination error < placement, upright above upright_min, no other piece moved > disturbance; captures must leave the field, land within tray tolerance and stay upright

## Actual execution — the Phase 8 gate

A real pick/place was executed from every square with a deterministic
piece type, then a failure was retried with a king and a rook to separate
a square/path limit from a piece-grasp limit.

| Check | Result |
| --- | ---: |
| Squares with a successful first attempt | 59/64 |
| Squares with a feasible path (after retry) | 64/64 |
| Tray destinations executed | 4/4 |

First-attempt failures that passed the same square with king/rook:

- `a1` (pawn): placement error; retry: king=pass; rook=pass
- `c3` (pawn): placement error; retry: king=pass; rook=pass
- `a7` (pawn): placement error; retry: king=pass; rook=pass
- `g7` (pawn): placement error; retry: king=pass; rook=pass
- `e8` (pawn): placement error; retry: king=pass; rook=pass

No square was excluded: every square has at least one successful piece path.

## Geometry scan (advisory)

The scan checks approach, grasp, lift, carry and descent IK plus jaw
clearance for all six piece types. Grasp/descent use the controller's
10 mm precise gate; non-precise transit legs use 30 mm / 60°. A transit
residual at an extreme corner is not a hidden square exclusion; the
actual execution above is the gate, and named residuals remain Phase 10
clearance inputs.

| State | Squares | Tray slots |
| --- | ---: | ---: |
| Empty board | 60/64 | 4/4 |
| Crowded start | 58/64 | 4/4 |

Executable quiet moves in the standard start: **2**; executable captures: **32**.

## Layout comparison

| Layout | Gap | Riser | Reachable squares | First missing |
| --- | ---: | ---: | ---: | --- |
| baseline_80mm_gap_60mm_riser | 80 mm | 60 mm | 64/64 | — |
| low_riser_0mm | 80 mm | 0 mm | 64/64 | — |
| close_60mm_gap | 60 mm | 60 mm | 60/64 | c1, d1, e1, f1 |
| far_100mm_gap | 100 mm | 60 mm | 62/64 | a8, h8 |

The selected 80 mm gap / 60 mm riser is the only tested layout with
64/64 reach; 60 mm loses the near rank and 100 mm loses the far corners.

## Mass model

The rook is the only measured assembled piece: 4.8 g including one M6
nut. Its COM/inertia are a bounded bottom-point-mass model, not a
measured tensor. All other assembled masses remain explicit ranges;
the nominal scene uses unweighted PLA geometry and the heavy bound is
a sensitivity case, not a fact.

| Piece | Nominal g | Heavy-bound g | Status |
| --- | ---: | ---: | --- |
| pawn | 0.626 | 5.626 | unmeasured range |
| rook | 4.800 | 4.800 | measured |
| knight | 1.559 | 6.559 | unmeasured range |
| bishop | 1.075 | 6.075 | unmeasured range |
| queen | 1.937 | 6.937 | unmeasured range |
| king | 2.553 | 7.553 | unmeasured range |

## Named geometry residuals

- empty a8: source pawn: IK leg; destination pawn: IK leg; source rook: IK leg; destination rook: IK leg; source bishop: IK leg; destination bishop: IK leg; source queen: IK leg; destination queen: IK leg; source king: IK leg; destination king: IK leg
- empty b8: source king: IK leg; destination king: IK leg
- empty g8: source king: IK leg; destination king: IK leg
- empty h8: source pawn: IK leg; destination pawn: IK leg; source rook: IK leg; destination rook: IK leg; source bishop: IK leg; destination bishop: IK leg; source queen: IK leg; destination queen: IK leg; source king: IK leg; destination king: IK leg
- crowded d1: source bishop: IK leg
- crowded f1: source pawn: IK leg; source rook: IK leg; source bishop: IK leg; source queen: IK leg
- crowded a8: source pawn: IK leg; source rook: IK leg; source bishop: IK leg; source queen: IK leg; source king: IK leg
- crowded b8: source king: IK leg
- crowded g8: source king: IK leg
- crowded h8: source pawn: IK leg; source rook: IK leg; source bishop: IK leg; source queen: IK leg; source king: IK leg

## Scope

Phase 8 freezes geometry, units, interfaces and the success contract. It
does not prove wrist-only perception, grasp localization, learned-policy
performance or physical manipulation. Those are Phases 9–18.
