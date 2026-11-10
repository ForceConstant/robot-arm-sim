# Phase 11c A1 analysis notes (draft, 2026-11-10)

## Scope
Subtask (a): opponent-move change detection robust to occlusion and shadows. Candidate path evaluated: **C1 frozen/geometry self-occlusion-aware perception/fusion + existing legal-successor move inference** (A1 path), measured from paired C1 receipts.

## Evidence base
- Controlled paired receipts (seed 9300, 8×25=200, 6-view oblique, markers, identical source/calibration/plan/model hashes): `heldout-oblique-selfocc-s8t25.json` vs `heldout-oblique-noselfocc-s8t25.json`.
- Reference: Phase 9 `heldout-oblique.json` (contextual; different case/rejection composition).

## Key outcomes
- **Safety:** 0 false confirmations in all compared receipts (paired and baseline).
- **Resolution/abstention (paired):** 190/200 → **200/200** resolved; abstentions 11 → **1**. No-self had 10 `insufficient` (scans=3) + 1 `ambiguous`; self-occ had **0 `insufficient`**, 1 `ambiguous` (safe).
- **Observability:** unknown-square burden reduced sharply (mean 3.067→0.753, ≥5 73→5, ≥10 6→3, zero 38→182). Rescans eliminated (scans 190/0/10 → 200/0/0).
- **Registration:** unchanged and within budget (8/8 sessions ≤2mm/1°, identical paired reg stats).

## Crowded vs sparse move slices
| Slice | Paired no self-occ | Paired self-occ |
|---|---:|---:|
| Crowded confirmed/total | 95/100 (5 insufficient) | 100/100 (0) |
| Crowded unknown mean (≥5/≥10) | 0.91 (0/0) | 0.00 (0/0) |
| Sparse confirmed/total | 94/100 (5 insufficient, 1 ambiguous) | 99/100 (1 ambiguous) |
| Sparse unknown mean (≥5/≥10) | 4.94 (64/2) | 0.64 (0/0) |

Interpretation: self-occlusion masking removed insufficient/coverage-driven abstentions in this paired sample across both slices and substantially reduced unknown squares; benefit is consistent with excluding arm-induced vote contamination.

## Residual abstention (self-occ)
- Single non-confirmed move: **ambiguous**, scans=1, unknown=0, answered 64/64, category `promotion`, expected `a2a1b` (session 3, sparse). This is a **move-inference ambiguity** (promotion piece type not observable in occupancy/colour state) rather than occlusion/coverage failure — treated as safe/correct abstention by benchmark semantics.

## Robustness considerations / gaps
- **Shadows/lighting:** paired protocol used randomized scene lighting as configured in benchmark; receipts do not isolate cast-shadow-from-pieces vs arm-shadow stress. Dedicated shadow/low-angle/arm-over-changed-square stress characterization is still needed for full (a) robustness claim.
- **Unknown-cause attribution:** unknowns dropped near-zero in this sample; further stress runs should attribute any remaining unknowns (arm mask, occlusion, perception uncertainty) if they appear.
- **Composition caveat:** baseline-vs-paired comparisons are contextual due to rejection/case-composition differences (31 vs 23 rejection records); controlled paired toggle is primary evidence.

## Preliminary A1 judgment (not exit-closed)
- Frozen/geometry A1 path (with C1 self-occlusion) shows **strong measured improvement and safety** for change detection/move resolution on the paired 200-turn protocol, with no observed insufficient/occlusion-driven abstentions in this sample.
- **Exit still requires:** documented robustness characterization under occlusion/shadow/crowding stress (or measured limitation), R evidence (partially collected), and exit-compliant closure (accept with budgets or scoped A2 training probe / measured rejection+fallback if budgets/robustness unmet). No false confirmations observed.
