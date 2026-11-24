#!/usr/bin/env python3
"""Sim task 1 - IK / Jacobian sweep, and the truncated-path hypothesis.

Two questions, in order:

A) Is the solver itself coupled? For a grid of board targets at observation and
   grasp height, solve the tip, then build the local command->achieved Jacobian
   from the model. If the model gives a clean identity everywhere, the measured
   [[0.337,-0.138],[0.100,0.712]] is NOT the solver's doing.

B) Does a truncated joint-space path look like that matrix? The real measurement
   read FK 1.6 s after the command. If the servos were still travelling, the
   sampled point sits part-way along a curved cartesian path, which produces
   both an undershoot and an apparent axis rotation. Sweep the travelled
   fraction and see which one, if any, reproduces the measured J.

Pure kinematics: no MuJoCo, no rendering, no GPU. CPU-only on the BC-250.
Reads live/board.json and the calibration; writes nothing outside .private/.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from kinematics import JOINTS, SQUARE_MM, ArmKinematics, square_xy  # noqa: E402
from tip_frame import TIP_IN_CHAIN  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402

MEASURED = np.array([[0.337, -0.138], [0.100, 0.712]])
STEP_MM = 8.0


def frame_at(x_mm, y_mm, z_mm, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    m = np.eye(4)
    m[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = [x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0]
    return m


def tip_of(kin, joints_deg, jaw):
    """Jaw tip in board millimetres for a joint vector."""
    tip = (kin.fk(joints_deg) @ jaw)[:3, 3]
    return kin.to_board(tip) * 1000.0


def jacobian_at(kin, jaw, seed, xy, z, step=STEP_MM):
    """Model Jacobian: solve the base once, then one solve per axis."""
    s0, _ = kin.solve_tip(kin.board_point(xy[0], xy[1], z), seed, jaw)
    if s0["position_error_mm"] > 20.0:
        return None, None, None
    j0 = [float(s0["joints_deg"][n]) for n in JOINTS]
    a0 = tip_of(kin, j0, jaw)
    cols, resid = [], []
    for axis in (0, 1):
        moved = list(xy)
        moved[axis] += step
        s1, _ = kin.solve_tip(kin.board_point(moved[0], moved[1], z), seed, jaw)
        if s1["position_error_mm"] > 20.0:
            return None, None, None
        a1 = tip_of(kin, [float(s1["joints_deg"][n]) for n in JOINTS], jaw)
        cols.append(np.array([a1[0] - a0[0], a1[1] - a0[1]]) / step)
        resid.append(float(s1["position_error_mm"]))
    return np.column_stack(cols), resid, j0


def main():
    calib = WristCalibration.load(str(ROOT / ".private/phase11d/calibration.json"))
    spec = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(spec.get("x_mm", 0), spec.get("y_mm", 0),
                     spec.get("z_mm", 0), spec.get("yaw_deg", 0))
    kin = ArmKinematics(calib, {}, board)
    jaw = TIP_IN_CHAIN

    print("deployed board:", {k: round(float(v), 2) for k, v in spec.items()}, flush=True)
    print("board frame yaw from matrix:",
          round(math.degrees(math.atan2(kin.board[1, 0], kin.board[0, 0])), 2), "deg", flush=True)

    seed0, _ = kin.solve_tip(kin.board_point(0.0, 0.0, 70.0), np.zeros(6), jaw)
    seed = [float(seed0["joints_deg"][n]) for n in JOINTS]
    print(f"seed pose residual {seed0['position_error_mm']} mm "
          f"approach {seed0['approach_error_deg']} deg\n", flush=True)

    # ---- A: model Jacobians over the board -------------------------------
    print("A) MODEL JACOBIAN (solve +8 mm, FK the solution)")
    print(f"   {'z':>4} {'grid':>6} {'det':>7} {'yaw':>7} {'sig':>14} {'resid':>7}")
    for z in (70.0,):
        for spacing in (60.0,):
            mats, resids, t0 = [], [], None
            warm = seed
            for gx in np.arange(-60.0, 60.1, spacing):
                for gy in np.arange(-60.0, 60.1, spacing):
                    J, res, j0 = jacobian_at(kin, jaw, warm, (gx, gy), z)
                    if J is None:
                        continue
                    mats.append(J)
                    resids.extend(r for r in res)
                    warm = j0                      # warm start the next pose
            if not mats:
                continue
            arr = np.array(mats)
            dets = np.linalg.det(arr)
            yaws = np.degrees(np.arctan2(arr[:, 1, 0], arr[:, 0, 0]))
            sig = np.linalg.svd(arr, compute_uv=False)
            print(f"   {z:4.0f} {len(mats):6d} {np.median(dets):7.3f} "
                  f"{np.median(yaws):+7.2f}      "
                  f"{np.median(sig[:, 0]):.3f}/{np.median(sig[:, 1]):.3f} "
                  f"{np.median(resids):7.2f}")
    print("   (det 1.0 and yaw 0 mean the model moves exactly where told)\n")

    # ---- B: truncated joint-space path ----------------------------------
    # The real probe read FK 1.6 s after the command. Interpolate in joint space
    # from the start pose and sample at a fraction f of the way; compare the
    # resulting xy direction with the measured matrix.
    print("B) TRUNCATED JOINT-SPACE PATH vs the measured matrix")
    print("   measured J ==")
    for row in MEASURED:
        print(f"     {row[0]:+.3f} {row[1]:+.3f}")
    print(f"   det {np.linalg.det(MEASURED):.3f}  "
          f"yaw {math.degrees(math.atan2(MEASURED[1, 0], MEASURED[0, 0])):+.2f} deg")
    print(f"\n   {'f':>5} {'J00':>7} {'J01':>7} {'J10':>7} {'J11':>7} {'yaw':>7} {'gap':>7}")
    best = None
    start_xy, z = (10.0, 10.0), 70.0
    s0, _ = kin.solve_tip(kin.board_point(start_xy[0], start_xy[1], z), seed, jaw)
    j_start = np.array([float(s0["joints_deg"][n]) for n in JOINTS])
    for axis in (0, 1):
        pass
    targets = []
    for axis in (0, 1):
        moved = list(start_xy)
        moved[axis] += STEP_MM
        s1, _ = kin.solve_tip(kin.board_point(moved[0], moved[1], z), seed, jaw)
        targets.append(np.array([float(s1["joints_deg"][n]) for n in JOINTS]))
    for f in np.arange(0.05, 1.001, 0.05):
        cols = []
        for j_target in targets:
            j_f = j_start + f * (j_target - j_start)
            a = tip_of(kin, j_start, jaw)
            b = tip_of(kin, j_f, jaw)
            cols.append(np.array([b[0] - a[0], b[1] - a[1]]) / STEP_MM)
        J = np.column_stack(cols)
        gap = float(np.abs(J - MEASURED).max())
        yaw = math.degrees(math.atan2(J[1, 0], J[0, 0]))
        print(f"   {f:5.2f} {J[0,0]:+7.3f} {J[0,1]:+7.3f} {J[1,0]:+7.3f} "
              f"{J[1,1]:+7.3f} {yaw:+7.2f} {gap:7.3f}")
        if best is None or gap < best[1]:
            best = (f, gap, J)
    if best:
        print(f"\n   closest fraction f={best[0]:.2f}  max|J-Jmeasured| {best[1]:.3f}")
        print(f"   det {np.linalg.det(best[2]):.3f}  "
              f"yaw {math.degrees(math.atan2(best[2][1, 0], best[2][0, 0])):+.2f} deg")

    # ---- C: per-joint servo lag ------------------------------------------
    # A single travelled fraction scales both axes together (Part B). If the
    # joints lag by different amounts, a +x command can arrive rotated and
    # anisotropic - which is what the measured matrix looks like. First-order
    # lag per joint, sampled 1.6 s after the command.
    print("\nC) PER-JOINT LAG MODEL, sampled 1.6 s after the command", flush=True)
    t_sample = 1.6
    j_start, targets = None, []
    s0, _ = kin.solve_tip(kin.board_point(10.0, 10.0, 70.0), seed, jaw)
    j_start = np.array([float(s0["joints_deg"][n]) for n in JOINTS])
    for axis in (0, 1):
        moved = [10.0, 10.0]
        moved[axis] += STEP_MM
        s1, _ = kin.solve_tip(kin.board_point(moved[0], moved[1], 70.0), seed, jaw)
        targets.append(np.array([float(s1["joints_deg"][n]) for n in JOINTS]))
    a0 = tip_of(kin, j_start, jaw)
    taus = (0.1, 0.3, 0.6, 1.2, 2.5, 5.0, 12.0)
    best_c = None
    for tau_pan in taus:
        for tau_lift in taus:
            for tau_elbow in taus:
                fr = np.array([1 - math.exp(-t_sample / tau_pan),
                               1 - math.exp(-t_sample / tau_lift),
                               1 - math.exp(-t_sample / tau_elbow),
                               1 - math.exp(-t_sample / 0.3),
                               1 - math.exp(-t_sample / 0.3),
                               1.0])
                cols = []
                for j_target in targets:
                    j_f = j_start + fr * (j_target - j_start)
                    b = tip_of(kin, j_f, jaw)
                    cols.append(np.array([b[0] - a0[0], b[1] - a0[1]]) / STEP_MM)
                J = np.column_stack(cols)
                gap = float(np.abs(J - MEASURED).max())
                if best_c is None or gap < best_c[0]:
                    best_c = (gap, tau_pan, tau_lift, tau_elbow, J)
    gap, tp, tl, te, J = best_c
    print(f"   best tau(pan/lift/elbow) {tp}/{tl}/{te} s   max|J-Jmeasured| {gap:.3f}")
    print(f"   J        {J[0,0]:+.3f} {J[0,1]:+.3f} / {J[1,0]:+.3f} {J[1,1]:+.3f}")
    print(f"   measured {MEASURED[0,0]:+.3f} {MEASURED[0,1]:+.3f} / "
          f"{MEASURED[1,0]:+.3f} {MEASURED[1,1]:+.3f}")
    print(f"   yaw {math.degrees(math.atan2(J[1, 0], J[0, 0])):+.2f} deg  "
          f"det {np.linalg.det(J):.3f}")


if __name__ == "__main__":
    main()
