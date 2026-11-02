# Priority 7 acceptance — infer move/state changes from observations

**Status (2026-11-02): complete for the qualified fixed-overhead-rig envelope
with settled observations, sparse positions and a clear arm park pose.** This
closes the move/state-inference *task*, not physical play and not full-game
rules. The previous confirmed position is the only source of history; an image
supplies only per-square `empty` / `white` / `black` / `unknown` labels.

## What was built

- `chess_sim/move_inference.py` — legal-successor comparison with explicit
  `confirmed`, `unchanged`, `ambiguous`, `insufficient` and `unexplained`
  outcomes. A move is returned only when exactly one legal successor is
  verified and no other legal move remains possible under unobserved squares.
  The returned board is the previous board with the move pushed, so python-chess
  preserves castling rights, en-passant rights and the half-move clock that an
  image cannot recover.
- `chess_sim.perception.recover_reference_empty()` — opt-in, conservative
  recovery of *empty* squares that match the session's empty-board reference.
  It never guesses white or black. This recovers shadowed empty squares without
  treating a shadowed piece as a known colour.
- `chess_sim.move_inference.merge_observations()` — top/wrist fusion; a square
  known in one view and unknown in another keeps the known label, while two
  views that confidently disagree become `unknown`.
- `tools/benchmark_move_inference.py` — held-out rendered evaluation with
  simple moves, captures, castling, en passant, promotion, unchanged boards,
  illegal board changes and hidden changed squares.
- `tests/test_move_inference.py` — 17 tests covering every status, history/rights
  preservation, no mutation of the previous board, promotion ambiguity, hidden
  squares, weak evidence, illegal changes, fusion and input validation.
- `tests/test_perception.py` — reference-empty recovery contract and opt-in
  behavior.

## Decision rule

For every legal successor, the module computes the squares whose expected label
changes. A hypothesis is **verified** only when every changed square is observed
with at least `min_support` confidence, no changed square conflicts, and every
confident deviation elsewhere is explained. It is **possible** when it has no
conflict but some changed square is hidden. Confirmation requires exactly one
verified hypothesis and no other possible hypothesis. Otherwise:

- several verified successors → `ambiguous` (promotion is the common case, and
  the result sets `promotion_required`);
- hidden/weak changed squares or an unruled-out alternative → `insufficient`;
- confident deviations no legal move explains → `unexplained`.

The previous board is never mutated. A promotion whose piece type is invisible
is never guessed as a queen.

## Qualified envelope

- Published overhead camera rig; calibrated board registration and the session's
  empty-board reference.
- Sparse practice positions: two kings plus 2–8 extra pieces.
- Settled observation after the move; the arm is parked clear at shoulder-pan
  `-1.0 rad` for the observation. The published rest pose reaches over the board
  and hides roughly a third of the squares, so it is not a valid observation
  pose for inference.
- Top view. Wrist fusion is implemented but did not improve this envelope: the
  parked wrist sees only about 25 usable squares and 22 are arm-occluded.
- `min_support = 0.5`, `deviation_confidence = 0.5`; reference-empty recovery
  uses `ref_abs_delta_luma <= 0.02` and `ref_fraction_changed <= 0.05`.

## Measured result

Held-out seed 7001, 3 sessions, 35 rendered cases, top view, all sessions
registered. The benchmark uses simulator truth only to build a scene and score
the outcome; inference receives only perception labels.

| Metric | Result |
| --- | --- |
| Accepted moves (`confirmed` / `unchanged`) | 13 |
| Accepted correct | 13 / 13 (100%) |
| False confirmations | **0** |
| Coverage (accepted / all cases) | 37.1% |
| Abstentions | 22 / 35 (62.9%) |
| Expected-abstain cases honoured | 15 / 15 |
| Status accuracy | 68.6% |

By category:

| Category | Accepted correct | Note |
| --- | --- | --- |
| Simple quiet move | 6 / 9 | 2 hidden changed squares, 1 unexplained colour conflict |
| Capture | 2 / 2 | |
| Castling | 2 / 3 | one hidden changed square |
| En passant | 0 / 3 | one changed square was `unknown`; correctly abstained |
| Promotion | 0 / 3 | 1 explicit ambiguous/ask; 2 hidden changed squares |
| Unchanged | 3 / 3 | |
| Hidden changed square (occluded) | 0 / 6 | all 6 abstained |
| Illegal board change | 0 / 6 | all 6 returned `unexplained` |

Fused top+wrist: 11 accepted, 11 correct, **0 false confirmations**, 31.4%
coverage. It was not better than top-only, so the qualified result uses top.

**No false confirmation occurred in either run.** Every accepted move was
correct. The low coverage is not permissiveness: the hard arm shadow leaves a
changed square unknown in many cases, and the module abstains rather than
guessing.

## Limits

- The overhead view's hard arm shadow makes roughly 20 empty squares `unknown`.
  Reference-empty recovery fixes most of them, but a shadowed *piece* can still
  change apparent colour; the module keeps those `unknown` and may abort a move
  that touches one.
- En passant and promotion require every changed square to be visible. Promotion
  is intentionally `ambiguous` (ask which piece); en passant abstained in this
  run because the captured-pawn square was not confidently observed.
- Sparse positions only; no full-game castling/en-passant/promotion history
  recovery, no crowded-board evaluation, no physical capture, no move execution,
  and no live web-UI integration. Those are separate gates.
- Camera calibration is simulated, not independently measured. The benchmark's
  clear park pose is a benchmark observation pose, not a changed simulator
  default or a hardware claim.

## Reproduction

```bash
MUJOCO_GL=egl PYTHONPATH=. .venv/bin/python tools/benchmark_move_inference.py \
  --sessions 3 --seed 7001 --views top \
  --out .private/move-inference/heldout-top.json

MUJOCO_GL=egl .venv/bin/python -m pytest -q tests
```

Regression: **86 passed, 4 skipped**.

Receipts:

- `.private/move-inference/heldout-top.json` —
  `a7c784ea33c86574d2be95300f0cebbf053a0a55365d162883644e8f0da4f005`
- `.private/move-inference/heldout-fused.json` —
  `778a53eeb7ac0529eebcb4e4dccbdf89d649fc226bb82f071020684c790cdf0a`
- model SHA-256 `318b5ae36c23db99263206a6054fd1a819e1ae222978229197f2a63744a81306`
