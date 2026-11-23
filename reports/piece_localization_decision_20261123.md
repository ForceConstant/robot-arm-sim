# Piece localisation — decision: square-centre aim, learned keypoint parked (2026-11-23)

## Decision

Grasping targets the **nominal square centre**. The learned base-centre keypoint
model is parked, not deleted. Revisit only if a real grasp fails on offset rather
than on something else.

Brian's reasoning (2026-11-23): the robot places the pieces itself, so a piece is
near the centre of its square by construction and the residual is the arm's own
placement repeatability. The jaws are 2–3× the piece width, so a few millimetres
of offset is inside the mechanical tolerance.

## Measured basis

| approach | error on the 46 real labelled frames |
|---|---|
| square centre aim | **4.06 mm** median |
| sim-trained keypoint, no fine-tune | 12.98 mm |
| sim-trained keypoint + real fine-tune | 7.59 mm (folds 5.88–10.83) |

The sim-only keypoint model reaches **1.13 mm median on unseen squares in
simulation** (all 64 squares, uniform 0.61–1.89 mm per square), with p95 18.4 mm
ungated and 2.7 mm at 24% acceptance. None of that survives the transfer to the
real camera: the appearance gap costs more than the geometry gains.

## What is parked, and where

- `tools/sim_keypoint_corpus.py` — free-camera corpus, 192 px crops, exact
  base-contact pixel labels, randomised light and camera inside the operating
  envelope. 2304 samples in `.private/sim_corpus/kp_v1/`.
- `.private/sim_corpus/kp_results.npz`, `real_kp.npz`, `real_kp_pred.npz` —
  predictions, confidence peaks and scores behind the numbers above.
- The model is a 3-conv heatmap net; it was trained in-process and is not saved
  as a checkpoint. Rebuilding it is one run of the scripts above.

## What would unpark it

A few hundred real labelled frames (the `tools/label_server.py` click-the-base
UI), spread across squares, lighting and piece types, then the same grouped
holdout. Gate stays at **< 3 mm median on real frames**, and a confidence
threshold validated on real data before it may drive motion.

## Related findings from the same session

- A residual measured against the *printed face asset* is worthless: the sim's
  `board_texture.png` and the reference `board_charuco.png` are different images
  (corr −0.676). Subtract an **empty render of the same scene** instead —
  residual centroids then track the offset at 0.999 correlation.
- The sim's board-body frame is 180° + ~5 mm from the marker-layout frame; place
  and crop in one frame only.
- GPU hard locks are compositor-dependent: with KDE Wayland running, sustained
  compute killed the board three times; with `multi-user.target` it ran three for
  three. Filed as MastaG/linux-cachyos-bc250#15.
