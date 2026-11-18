# Wrist camera: hand-eye and board calibration

Two routines, both driven by marker sightings from the wrist camera. They run in
the simulator today and are written to run unchanged on the real arm once the
printed ChArUco board is complete.

## Why the old estimator was wrong

`cv2.calibrateHandEye` solves `A X = X B` from *relative* motions. On this arm
every relative rotation is small (pan sweeps, small wrist tilts), so the
translation block is poorly conditioned: rotations solve cleanly while
translations scatter. Real data showed exactly that — 0.05–0.09 rotation RMS
beside ~100 mm translation spread.

Neither routine here uses relative motion. Both fit one consistent model to
absolute observations, so there is no differencing to amplify noise.

## Routine A — board pose (`tools/calibrate_board_charuco.py`)

PnP per marker against board-frame corners, composed with forward kinematics to
give one board-pose estimate per marker; reported as median plus 1-sigma spread.

```
robot-arm-sim python tools/calibrate_board_charuco.py \
    --layout charuco --captures live/captures --out reports/board_charuco.json
```

The spread is the honest error. If it is tens of millimetres, the input
calibration is bad and routine B must run first.

## Routine B — hand-eye mount + board pose together (`tools/handeye_bundle.py`)

Closed-form alternating solve: given the current mount, average the board pose
over views; given the board pose, average the mount over views. Each half is a
rigid-transform mean (SVD-averaged rotation), so there is no optimiser to fall
into a basin — a nonlinear bundle was tried first and converged to the same
wrong point from every start.

```
# simulator
robot-arm-sim python tools/handeye_bundle.py --method staged --bootstrap \
    --captures sim/captures_wrist --records sim/captures_wrist/records.json \
    --out reports/handeye.json
```

`--bootstrap` derives the mount from the **nominal board placement** — the one
thing a real setup always knows approximately — so it needs no good prior.

### Simulator results (ground truth known)

| init | mount error | rotation | reprojection RMS |
|---|---|---|---|
| one known camera pose | **2.68 mm** | 0.37 deg | 1.00 px |
| bootstrap (no prior) | **19.25 mm** | 1.67 deg | 11.1 px |
| stored calibration (old) | 209 mm | 102 deg | 746 px |

For comparison, the real captures scatter ~100 mm. Bootstrap at 19 mm is a
five-fold improvement before any refinement, and it is the path available IRL.

### Failure modes already found and fixed

- **`fiducials.MARKER_IDS` gated ids to the mat set (0–4)** and silently dropped
  every ChArUco tag. Detection looked fine, results looked empty.
- **MuJoCo cameras look down -z, OpenCV down +z.** Without the 180 deg
  conversion the solve appears broken; with it, 10 of 12 views agreed to <1 mm.
- **A single-view "correction" (rank flip) was a marker-subset coincidence.**
  It improved one view to 2.4 mm and made every later measurement wrong. The
  sim board matches the print marker-for-marker; do not flip the mapping.
- **One occluded view inflated RMS from 1 px to 65 px.** Robust view pruning
  (median + 3 MAD) is required, not optional.

## View collection (`tools/collect_wrist_views.py`)

Commands the arm in the same vocabulary as the live app — Cartesian targets over
the board, which the look-at helper converts into camera aims — then records
joints, frame and the simulator's true camera pose.

```
robot-arm-sim python tools/collect_wrist_views.py --out sim/captures_wrist --count 12
```

Diversity matters more than count: translation error falls with the spread of
board points and standoffs, not with the number of nearly identical views.

## IRL procedure

1. **Print the ChArUco target** (`charuco_28mm_final.zip`), two halves, face
   down, lap glued. Measure a square: 28.0 mm, and the field: 224.0 mm.
2. **Place it on the board, rotated 180 degrees** so the arm plays black.
   Confirm the notation reads correctly from the arm's side and that a tag
   decodes in the app.
3. **Capture 12–20 views** with the arm over different board points, using the
   dashboard's x/y/z jog and look-at panel; keep several standoffs. Two frames
   300 ms apart must agree, which the app already enforces.
4. **Run routine B** with `--bootstrap` against `live/captures`.
5. **Read the RMS and the board spread, not just the mount.** RMS above ~2 px
   with a clean capture set means bad correspondences; a large board spread
   means the mount is still wrong.
6. **Iterate once**: re-run with the refined mount as the init. Each pass pulls
   the outlier views out.

Accept a result only when the reprojection RMS is around 1 px and the board pose
is within a couple of millimetres of where the board physically sits.

## Verification discipline

Every claim here came from a measurement that could fail:

- chain FK against simulator camera poses: **0.002 mm spread** over 12 views
- per-marker PnP: **0.04 px**
- solver validated by recovering a *known* mount, not by converging
