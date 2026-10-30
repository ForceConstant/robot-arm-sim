# Priority 6 acceptance — board registration and occupancy/colour perception

**Status (2026-10-30): complete for the qualified fixed-overhead-rig envelope.**
This closes the perception *task* for a calibrated camera, a known board size
and surface height, board placement within the stated envelope, and a session's
own empty-board reference. It does not qualify a physical camera, arbitrary
camera placement, full-game recovery, or move/state inference.

## What was built

- `chess_sim/board_registration.py` — calibrated planar registration from one
  overhead frame. A multi-scale checker-saddle response supplies a coarse pose;
  an edge-consistency term resolves the checker lattice's one-square phase; a
  local Nelder–Mead refinement returns origin, yaw and square size. The caller
  receives `score`, `coverage` and `edge_score`, and the qualified helper
  (`perception_data.register_for_perception`) **abstains** on a low-score fit.
  No piece pose, FEN or simulator transform is read.
- `chess_sim/perception.py` — per-square rectified patches, same-colour
  neighbour background estimates, and a small NumPy MLP over 34 colour, texture,
  gradient and empty-reference difference features. Output is one of
  `empty`, `white`, `black` or `unknown`; a blank/near-black frame and
  out-of-frame squares also abstain.
- `tools/train_perception.py` — generates sessions, captures each session's
  empty-board reference, samples top and wrist views, and trains the model.
  Training/validation splits whole sessions, never frames.
- `tools/benchmark_perception.py` — held-out sessions with a different seed
  stream; compares predictions with simulator truth only after prediction.
- `tests/test_board_registration.py`, `tests/test_perception.py` — geometry,
  feature contract, model artifact, named-piece colour, dark-frame abstention
  and threshold calibration.

## Qualified envelope

- Published overhead camera rig (`PUBLISHED_RIG_CAMERA`), camera intrinsics and
  extrinsics from the calibrated camera model.
- Board square and surface height known from the board; board placement within
  approximately ±12 mm and ±2.5° of the calibrated nominal pose. Registration
  searches ±16 mm / ±4.5° around the nominal pose.
- Sparse practice positions: two kings plus 2–8 extra pieces.
- One empty-board reference frame per session at the same board/camera pose.
  The board and camera stay fixed for all positions in that session.
- Top and wrist views. The wrist view uses the top-view board pose plus the
  calibrated camera-to-arm chain; only squares the wrist resolves contribute.

## Measured result

Training: 10 sessions × 12 positions, 11,220 rows (7,680 top, 3,540 wrist).
Labels: 8,016 empty, 417 white, 459 black, 2,328 unknown. Two whole sessions
were held out: coverage 81.2%, answered accuracy 98.1%, confident-error rate
1.58%, threshold 0.40.

Held-out evaluation (`seed 9001`, 8 sessions × 5 positions, 5 sessions
registered, 3 rejected by the registration gate):

| Metric | Result |
| --- | --- |
| Registration | 1.42 mm median / 1.92 mm max origin error; 0.49° median / 1.43° max yaw; 80% within 5 mm and 1° |
| Top, unoccluded | 96.0% coverage, 99.4% answered accuracy, 95.8% occupied accuracy, 99.9% empty accuracy |
| Top, confident errors (unoccluded) | 6 / 1,100 (0.55%) |
| Top, natural arm occlusion | 90.6% abstention; 10% answered |
| Wrist | 99.9% coverage, 97.8% answered accuracy, 88.2% occupied accuracy, 98.9% empty accuracy |
| Shadow stress | 98.4% answered accuracy |
| Synthetic opaque occluder stress | 22.5% abstention — **weak** |

The model never receives simulator truth, a FEN or a piece pose at inference.
Truth is used only to label training data and to score the held-out run.

## Limits

- **Registration acceptance is 5/8 held-out sessions.** Rejected sessions are
  reported as abstentions and excluded from perception metrics, not silently
  used. Wider boards/edges of the calibrated frame and some appearance draws
  still fail the gate; improving that is future work.
- **Synthetic opaque occluders are not reliably `unknown`** (22.5% abstention),
  although the natural robot-arm occlusion is (90.6%). Hands/foreign objects
  need a stronger occlusion detector.
- The empty reference is a calibration observation. It is valid only while the
  board and camera stay fixed; moving either requires a new reference.
- Camera calibration is simulated (the MuJoCo camera model), not an independent
  physical calibration; physical capture sessions and camera-to-tool mounts
  remain Priority 10.
- Sparse positions only. Crowded boards and full legal-game recovery remain
  Priority 7/8 and deferred scope. Occupancy classification is not move
  understanding.
- The wrist view is evaluated at the parked pose; a moving wrist camera and
  motion blur are not qualified.

## Receipts and reproduction

Private receipts (gitignored):

- `.private/perception/training-v8.json` — training rows, split, history,
  operating point.
- `.private/perception/heldout-v8.json` — per-square records, per-session
  table, registration errors, stress results.
- `.private/perception/dataset-v8.npz` — 11,220-row feature dataset.
- Model artifact: `chess_sim/assets/perception/model.json`
  SHA-256 `318b5ae36c23db99263206a6054fd1a819e1ae222978229197f2a63744a81306`.

Reproduce:

```sh
MUJOCO_GL=egl PYTHONPATH=. .venv/bin/python tools/train_perception.py \
  --sessions 10 --positions 12 --seed 61
MUJOCO_GL=egl PYTHONPATH=. .venv/bin/python tools/benchmark_perception.py \
  --sessions 8 --positions 5 --seed 9001
MUJOCO_GL=egl .venv/bin/python -m pytest -q tests
```

The full regression at closure: **67 passed, 4 skipped**.
