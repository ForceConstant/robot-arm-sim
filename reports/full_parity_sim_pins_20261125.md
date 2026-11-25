# FULL-PARITY-01 — sim-side implementation receipt (2026-11-25)

Status: **partial — sim-only**. IRL portions blocked while work is underway on
the real arm. This is a test-plan implementation, not a full test pass.

## Changes

- `docs/TESTS.md` — full paired sim/IRL plan, frame/rotation pin step T2b,
  pain-point map §3b, human gates T8/T9, drift decision rule.
- `tests/test_frame_pins.py` — new, 5 tests, pure config/static reads:
  1. `test_board_json_exact` — board pose incl. yaw 184.32°.
  2. `test_rig_state_snapshot_matches_manifest` — SHA-256 of all 4 snapshots
     and the machine-local canonical copies against the manifest.
  3. `test_board_facing_config` — `ARM_PLAYS=black`, `BOARD_TURN=pi`.
  4. `test_tool_chain_constants` — tool_frames mount pos/quat, tip depth,
     base bbox, mirrored jaw origins.
  5. `test_tip_constant_stamp` — resolved tip = 0.11045 m, pack cad_parallel.

## Host / tree provenance

- Host: `cachyos-x8664`, `PRETTY_NAME="CachyOS"` (BC-250, 192.168.1.171).
- Local source head: `51a403bba532139e8132cfe9f9c3551fb50555a1` (`main`).
- BC-250 head at run: `0d33f8c`, **17 commits behind** local, with uncommitted
  IRL work (`live/app.py`, `live/solver.py`, `live/pi_bridge/*`, etc.).
- Only `tests/test_frame_pins.py` was copied to the BC-250; nothing was synced
  over the in-progress IRL tree and no arm motion was commanded.

## Results (BC-250)

```
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  .venv/bin/python -m pytest -q tests/test_frame_pins.py
.....                                                                    [100%]
5 passed in 0.75s

# companion guards (same host)
pytest -q tests/test_board_facing.py tests/test_aim_geometry.py \
         tests/test_vision_geometry.py tests/test_product_twin.py
...........................                                              [100%]
27 passed in 4.69s
```

## Not green / not settled

With the private gripper pack and hand-eye guards on the **stale BC-250 tree**:

- `test_tool_orientation.py::test_base_extents_in_its_own_part_frame` —
  no matching base mesh in built model.
- `test_hand_eye.py::test_refinement_recovers_a_perturbed_mount` — detected
  marker 14 not in `MarkerLayout._CENTRE_SIGNS`.

The hand-eye code is byte-identical on both trees, so the cause is not proven.
These failures predate this test and are **not** logged as drift findings or
waived. Re-run on a synced tree after the IRL work lands. T2b's pass is the 5
literal pins only; it does not claim these two guards green.

## Remaining / blocked

- T1 source-tree consistency — cannot certify while the BC-250 is behind and
  dirty; no sync attempted.
- T3 model parity, T4 sim sweep — not run: an IRL `aim_squares` sweep and
  `live/app.py` were active; avoided competing load. Re-run after IRL work.
- T5 IRL sweep, T6 paired comparison, T7 rig re-verify — blocked by in-progress
  arm work.
- T8/T9 human gates — pending; require pictures and Brian's sign-off.

Owner: Vera. Resume condition: IRL work complete and BC-250 tree synced to
`51a403b` (or newer), then re-run all of the above and collect both gates.
