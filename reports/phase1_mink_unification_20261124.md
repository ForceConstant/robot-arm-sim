# Phase 1 — mink unification (2026-11-24)

**Question this answers:** Phase 0 proved a move can be *executed* and measured.
Phase 1 asks the model question behind it: the console's hand-rolled
`scipy.optimize.least_squares` solver in `live/kinematics.py` and the sim's
`mink` QP stack in `chess_sim/ik.py` are **two solvers over two models of one
arm**. Can the mink stack become *the* solver behind `/move`?

**Answer: yes, and the model question is settled — the mink model is bit-exact
to the live chain.** The flag is wired, default OFF, and proven inert when off.
The gate proposal is deliberately *not* applied: it needs owner approval and it
is blocked on a measurement the console cannot make.

Nothing is committed. `live/app.py` gains a flag plus a solver adapter; no motion
behaviour changes with the flag off.

---

## 1. What was built

| file | role |
|---|---|
| `live/mink_arm.py` | Builds an arm-only MuJoCo model **from the WristCalibration chain JSON + `TIP_IN_CHAIN`** — the same two sources `live/kinematics.py` uses — and wraps `So101Ik` over it. The site frame is defined so mink's "approach = site +x, span = site +z" means the same as the console's tip frame. |
| `live/solver.py` | `MoveSolver`: `ArmKinematics`'s solve API, backed by either solver. Flag off → delegates to the chain unchanged; flag on → the same chain-frame goal is passed through `TIP_IN_CHAIN` to the mink model. Escalation policy is *copied* from `solve_tip` so the comparison isolates the solver, not the search. |
| `tools/phase1_parity.py` | FK(chain) vs FK(mink model) over a joint grid. **This is the gate for everything else.** |
| `tools/phase1_model_diff.py` | The *other* model: sim-scene arm vs chain, and the mount offset. |
| `tools/phase1_squares.py` | The identical 64-square sweep, both solvers, gate verdict + timing per square. |
| `tools/phase1_gate.py` | Honest landing decomposition → proposed `move_tolerance_mm`. |
| `tools/phase1_wireup.py` | Proves flag-off is identical to the chain and flag-on is the mink stack, guards included. |
| `tools/phase1_status_image.py` | The three milestone sheets (parity, squares, gate). |
| `tools/phase1_summary_image.py` | The one-page summary sheet. |

### `live/app.py` changes (all behind the flag)

- `DEFAULTS["ik_solver"] = "least_squares"`. `"mink"`/`"qp"`/`"so101ik"` turns it on.
- `self.solver = MoveSolver(...)` built next to `self.kinematics`.
- Every `self.kinematics.solve*` call in the solve path (`move` xyz/look_at/aim/
  rel, `sanity_check`, the reach probe) became `self.solver.solve*`.
- `_refresh_limits` calls `self.solver.sync_limits(limits)` so the mink model
  follows the bridge's measured ranges exactly as the chain does.

**Unchanged by design:** bridge streaming, TTL/seq, speed caps, fault latching,
keep-in, joint-limit violations, the wrist-roll window, dry-run, `_stream_to`.

---

## 2. Parity — the finding that makes the rest possible

`tools/phase1_parity.py`, BC-250, 5292 joint-grid samples (20° step, rolls
0/±90), jaw tip through `TIP_IN_CHAIN`:

```
samples           5292
references (rad)  [0, 0, 0, 0, 0]  (all zero: True)
worst position Δ  0.000000 mm      (gate 0.1 mm)
worst rotation Δ  3.42e-06 deg
limit mismatch    none
PASS
```

The mink model is **bit-exact** to the live chain's FK. The 3e-6° rotation term
is float64 round-off through two different quaternion paths; it is not a model
difference. This is the whole premise: the QP stack now solves *the live arm*,
not a scene twin.

Why it works: `ArmKinematics.fk` applies a joint as
`T(anchor − R·anchor, R)` *inside* the link frame. A MuJoCo hinge with
`pos=anchor, axis=axis` applies exactly that transform. Building the model link
for link from the chain therefore reproduces the FK one-for-one, and the test
proves it rather than asserting it.

### The *other* model, and where it differs

`tools/phase1_model_diff.py` measures the sim-scene arm (`scene.build_arm`)
against the chain over 1764 poses:

```
chain vs live model (parity)   worst 0.000 mm
chain vs sim-scene model       worst 5.000 mm, median 5.000, p95 5.000
mount offset                   [0, +5.0, 0] mm
```

The sim scene differs from the live arm by a **constant 5.0 mm in +y** — exactly
the difference between `BoardConfig.arm_base` (a scene constant,
`[0, −0.212, 0.4624]`) and the chain's measured mount (`[0, −0.217, 0.4624]`).
That is the mount, not a kinematic disagreement, and it is why Phase 1 mounts
the model at the chain's own first link rather than borrowing the scene's.

---

## 3. 64-square comparison — both solvers, identical targets

`tools/phase1_squares.py`, BC-250, hover 60 mm, board frame from `live/board.json`,
both solvers seeded from the same posture (the sweep snaps to the **incumbent's**
solution after each square, so mink gets no warm-start advantage).

| metric | least_squares | mink |
|---|---|---|
| solved, gate 20 mm / 8° | **64/64** | **64/64** |
| regressions (LS pass → mink fail) | — | **0** |
| solver residual p50 | 4.40 mm | **0.34 mm** |
| solver residual p95 | 13.15 mm | **1.34 mm** |
| solver residual max | 16.00 mm | **1.49 mm** |
| solve time p50 | **1867 ms** | **3 ms** |
| solve time max | **169 504 ms (169 s)** | **4 ms** |
| total over 64 squares | **417 s** | **0.2 s** |

`reports/phase1_squares.json`, sheet `reports/phase1_squares.png`.

Two things to read carefully:

- **No square regressed.** Both are 64/64 on the live gate. On the *position*
  residual mink is better on all 64 squares: the smallest improvement is 1.65 mm
  (g5), the largest 15.55 mm (h1). There is no square where LS beats mink on
  position. (Approach angle is a separate story — see below.)
- **Speed.** LS's *median* is 1.9 s, but `g1` took **127 s** and `h1` **169 s** —
  these are the far-corner escalations the brief flagged ("solve_tip escalates to
  ~4 s today"; on this run the slow cases were far worse). Mink solved every
  square in 3–4 ms. That is the difference between a `/move` that blocks the
  whole console for three minutes and one that returns immediately — Phase 0
  Finding 2 in `reports/phase0_move_verification_20261124.md`.

### A trade-off to know about: approach angle at the reach edge

Mink buys position accuracy partly *by giving up approach angle* at the extremes.
On four squares it lands with a larger approach error than LS, though every one
stays inside the 8° gate:

| square | LS approach | mink approach |
|---|---|---|
| h1 | 3.08° | **7.35°** |
| c8 | 0.47° | 4.55° |
| g1 | 2.03° | 4.42° |
| d8 | 0.46° | 3.98° |

`h1` is the near +x corner at the edge of reach, and 7.35° is only 0.65° inside
the 8° gate. This is the `POSITION_COST = 30` position-priority working as
designed — it will tilt the approach to land the point — but it means the
approach gate is doing real work on the mink path where LS coasted well clear of
it. **Phase 2 should not loosen the 8° approach gate**, and an IRL move at h1
should be in the verification set precisely because it sits closest to that edge.

### Why mink is better here, stated plainly

Least-squares folds rotation error into the position residual with a fixed
`ROT_SCALE_M = 0.25` and a `CONTINUITY` pull toward the seed. On a downward
approach at the board's far edge that soft balance stalls: the solver holds
position within a few mm but cannot also meet the approach, and the fast path
hands back a 12–16 mm landing. The mink stack makes joint/step/collision limits
*hard* QP constraints and uses position-priority, so it converges to the true
pose instead of trading millimetres for approach. Same arm, same targets — the
QP formulation is simply stronger.

---

## 4. `move_tolerance_mm` — proposal, not a change

The gate is compared against the solver's *own* residual, and both sides go
through the same FK. So it is blind, by construction, to:

| term | size | who can see it |
|---|---|---|
| solver residual | mink p50 **0.34**, p99 1.47, max 1.49 mm | this gate |
| **model bias** | **18.7 mm** (T2 refit, 57 views) | a physical check only |
| hardware scatter | ~8.2 mm p95 at 0.7° settle tolerance | the settle tolerance |

`tools/phase1_gate.py`, `reports/phase1_landing.json`, sheet `reports/phase1_gate.png`:

```
honest physical landing  p50 18.7 mm   p95 20.5 mm   <- today's 20 mm is essentially this
```

**That is the laundering.** The 20 mm gate is not a solver tolerance at all; it
is a hair above the physical landing error the console cannot see. It passes a
move that lands 19 mm off the board point, and it is so loose it would also pass
a genuinely bad solve.

**Proposal: `move_tolerance_mm` = 3.0 mm**, a bound on the *solver*
(max(p99, max) × 2 headroom, rounded up), **not on physical landing**. With the
mink residual distribution above, every one of the 64 squares passes 3.0 mm and a
bad solve does not. On the least_squares path 3.0 mm would pass only 12/64 — but
that is not an argument against 3.0 mm; it is an argument that the gate and the
solver must move together.

**It is blocked.** Two things must happen before this gate is usable:

1. **The mink solver must be the one running.** At 3.0 mm the incumbent LS path
   fails 52/64 squares it currently passes. A 3.0 mm gate on the LS path refuses
   moves that are physically fine. So the gate and the solver flip together.
2. **Physical landing still needs its own check.** No FK-internal number can see
   the 18.7 mm bias. That needs a marker read-back (Phase 2), and until it
   exists, the gate bounds solver quality only — it does not certify a landing.

**Do not apply this without owner approval.** The proposal above is the number
to put in front of him: **3.0 mm gate, together with the mink flag flip, plus a
physical check for the bias.** Tightening the gate *alone* (flag still off) would
refuse 52 of 64 currently-fine moves.

---

## 5. Wire-up proof

`tools/phase1_wireup.py` asserts the two properties that make the flag safe:

1. **Flag off is identical.** `MoveSolver(use_mink=False).solve_tip` returns the
   same joints, `position_error_mm`, `approach_error_deg`, `starts` **and goal**
   as `ArmKinematics.solve_tip`, exactly, on every square checked. The adapter is
   a pure pass-through when off, so turning it off is a real rollback. (The
   sampled squares cover the four corners and the far-edge escalations, which is
   where any frame or limit difference would surface.)
2. **Flag on is the mink stack, and it is trustworthy.** On the 8 squares that
   include the corners and the far-edge escalations:
   - deterministic (same answer twice),
   - self-consistent (the reported `position_error_mm` equals re-FK of its own
     joints against the goal, to <0.05 mm),
   - never worse than the chain with the *same* search,
   - every answer passes the 20 mm / 8° gate and the keep-in envelope,
   - no joint-limit violations.

`reports/phase1_wireup.json` — `passed: true`, `mismatches: none`.

A separate smoke test from the real `home_joints` posture (the seed `/move`
actually uses), via the live board frame:

```
 e4: mink   0.04 mm / 0.00°   LS   2.50 mm / 0.33°
 a1: mink   0.16 mm / 2.08°   LS  14.40 mm / 2.52°
 h8: mink   0.50 mm / 0.01°   LS   4.30 mm / 0.38°
 h1: mink   0.32 mm / 7.38°   LS  14.50 mm / 2.26°
 a8: mink   0.36 mm / 0.00°   LS   4.20 mm / 0.11°
```

Both flag states build and solve from a real seed; mink is ~10–90× closer on
position, and h1 shows the approach trade-off below.

Note on the initial comparison this tool made: it first compared flag-on against
a *single* bare `MinkArm.solve` and failed a8/h8. That was the test being wrong
— `solve_tip` deliberately searches, while a single start does not, and the
fast probe is run *without* a span task. Comparing a search to a single start
tests the search, not the wire-up. The assertion was corrected to the honest
claim: same search, mink never worse than chain. Both were ≥1 mm short of the
20 mm gate either way, so nothing about the conclusion moved.

---

## 6. Go / no-go for the Phase 2 flag flip

**Conditional GO**, with three conditions:

1. **GO on the model.** Parity is exact (0.000000 mm). The mink stack solves the
   live arm; there is no second model left in the live path.
2. **GO on the solver.** 64/64, zero regressions, ~600× faster median, and it
   removes the multi-minute `solve_tip` stalls that block the whole console.
3. **NO-GO on flipping without these:**
   - the `move_tolerance_mm` change must land **with** the flag, not before it
     (3.0 mm gate on the LS path refuses 52/64 valid moves);
   - a physical landing check (marker read-back) must exist before the gate is
     read as accuracy, because 18.7 mm of model bias is invisible to it;
   - an IRL move behind dry-run + REMOTE + keep-in, measured with the Phase 0
     `tools/test_move_executes.py --irl` harness, showing the mink-solved joints
     actually arrive — the sim column cannot prove the servo accepts them.

Nothing in this phase flips the flag. `ik_solver` stays `"least_squares"`.

---

## 7. Blockers and scope notes

- **BC-250 checkout is behind origin/main** (HEAD `b03c392`) with in-flight
  b1/handeye work. All Phase 1 files were written into the working tree beside
  it; none of the in-flight files were touched, stashed or committed.
  - The working-tree `live/app.py` there was byte-identical to `origin/main`
    (which is why it showed as modified against HEAD). Verified by arithmetic:
    `b03c392→origin/main` is +338/−11 and `origin/main→phase1` is +31/−6, and
    `b03c392→working tree` is +369/−17 — the exact sum, and `git diff origin/main`
    of that file contains only the Phase 1 lines. Nothing in-flight was lost.
  - The 4 routes `/label*` seen on the running server come from
    `label_routes.install(app)`, which both versions call — not extra in-flight
    work.
- **The mink model carries no collision geometry from the sim scene.** It has
  the installed tool's `ik_boxes` (the pack `tip_frame` already requires) for
  `So101Ik`'s self-collision limit, and nominal link masses for MuJoCo's
  compile-time check. FK parity is unaffected; the QPs are kinematic.
- **The `analyze` note:** `chess_sim/ik.py`'s `POSITION_COST = 30.0 if
  gripper.CUSTOM else 1.0` is honoured — the `cad_parallel` tool gets the
  position-priority weighting the sim uses.
