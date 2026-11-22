# Close-then-servo — sim closed loop (2026-11-22)

## Result: converged, mount-free

`tools/p4_sim_servo2.py` runs the full loop in the BC-250 simulator:

1. one pawn on e5; coarse observation pose then a ~25 mm camera offset;
2. each step, **both** the camera board-plane position (`camera_in_board_mm`
   from `Board.solve`) and the piece centroid come from **marker PnP only** —
   no FK, no `camera_in_tool`, no `board.json`;
3. error = piece_board_mm − camera_board_mm; command a damped board-frame
   camera translation (stand-in for live `rel`); re-measure and repeat.

Convergence on e5 (gain 0.5, max step 20 mm):

| step | error mm |
|---|---|
| 0 | 76.32 |
| 1 | 45.56 |
| 2 | 17.28 |
| 3 | 25.10 |
| 4 | 6.49 |
| 5 | 3.23 |
| 6 | 20.52 (spurious detection) |
| 7 | 11.55 |
| 8 | 4.84 |
| 9 | **2.69 → converged** |

The loop recovered from a bad frame at step 6 — closed feedback absorbs both
perception noise and the ~2× kinematic response of the motion primitive. With
gain 1.0 it settled into a ±16 mm two-cycle (overshoot); gain 0.5 is stable.
This is precisely the benefit of servoing over a one-shot mount-based move.

Receipt: `reports/visual_servo_sim_e5.json`.

## What this does and does not prove

Proves: a mount-free error signal drives a closed loop to a few mm in sim with
realistic detection (8–11 markers, 0.26–2 px PnP RMS), and tolerates a bad frame.
Does not prove: live hardware convergence. The live motion primitive is `/move
rel` (board-axis translate, 120 mm cap, speed-capped streaming with settle),
which has real gear play and latency; and IRL has only 1–5 markers per view.

## Honest caveats for the IRL attempt

- IRL detection is much weaker (13/21 calibration frames had a stable solve;
  marker count 1–5, often 2). The sim had 8–11 markers/frame.
- The area/compactness gates need tuning for the real piece at e5.
- The coarse observation pose must put e5 and ≥2 markers in the ~123×92 mm FoV.
- Each `rel` move must be dry-run/clearance-checked; stop on 409 or a branch flip;
  no gripper actuation.
