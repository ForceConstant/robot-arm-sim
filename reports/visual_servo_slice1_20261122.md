# Close-then-servo prototype — slice 1 (2026-11-22)

**Decision (Brian):** pivot from the hand-eye re-derivation to option 2 —
close-then-servo pickup, which should bypass the suspect camera-in-tool mount.

## Decisive finding: square naming is already mount-free and correct

On a real 4-marker frame (`T202508832_marker9`, 0.90 px PnP RMS), the same marker
(18) lands two different places:

- **Pure marker PnP** (`Board.solve`, what the overlay uses to name squares):
  declared centre `(−42, −14)` mm — correct by construction from the markers.
- **FK + camera_in_tool + board.json** (the aim/command path): `(−24, +21)` mm —
  ~35 mm (more than one square) off.

`Board.solve()` uses only marker corners + intrinsics; it never touches FK,
`camera_in_tool`, or `board.json`. So:
- the overlay's square IDs and the piece centroid (both derived from PnP
  rectification) are **correct and independent of the mount**;
- the 1–2 square "naming" error in the review is actually entirely in the
  **FK/mount path used to aim the arm**, not in perception.

This validates the premise of close-then-servo: perceive the square/piece with
PnP (already correct), then close the loop in image/board coordinates, never
commanding an open-loop move that depends on the bad mount.

Evidence: `tools/p4_naming_check.py`, `tools/p4_detector_signal.py`.

## Servo core (written and unit-tested)

`tools/p4_visual_servo_proto.py` — a damped position-based servo:
- error = target_mm − piece_mm (both from mount-free PnP detection),
- step = clip(gain · error, ±max_step), commanded as a board-axis `rel` move,
- stop at tolerance, abstain if no piece/board pose, cap steps, detect stalls.

Unit test with a synthetic noisy detector (σ=0.8 mm) converges in 2–3 steps
to within 3 mm tolerance. No gripper actuation.

## Detector-signal validation on real frames

Running `Board.solve` + `grid` on the 21 calibration captures:
- 13/21 give a stable board solve (RMS 0.6–1.1 px, one at 3.12 px) with a
  sensible nearest-square name;
- 8 fail (single/zero-marker board-edge poses from the calibration scan —
  expected, since those frames point at edges, not the board centre).

## Status / what remains

Done: diagnosis, mount-free servo core, signal validation. All run on the
BC-250; nothing committed; no arm motion; no gripper actuation.

Remaining for end-to-end closed loop (needs a decision):
1. **Sim harness** — wire the servo core to `SingleFollowerEnv` wrist renders +
   `TargetDetector`, place a piece, run the loop to convergence. Larger build.
2. **Live harness** — wire the core to the real wrist stream + `/move rel`, with
   a piece on the board and the bridge in REMOTE. Needs board setup and a fresh
   motion authorisation for this new loop.

The arm is currently HOLD, armed, no fault, board clear (calibration state).
