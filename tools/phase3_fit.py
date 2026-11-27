#!/usr/bin/env python3
"""Phase 3 fit: separate the board-origin error from the camera-mount error.

Three models, all on the same marker observations:

  A  board pose fixed at live/board.json, solve camera_in_tool (6 DoF)
  B  camera_in_tool fixed at the stored calibration, solve board pose (6 DoF)
  C  both free (12 DoF)

and the discriminator that matters: leave-one-roll-group-out.  Every capture in
this sweep has a known wrist_roll; the groups are the planned roll values.  A
model that is the right decomposition predicts a roll group it never saw; a
model that confuses the two translations only does well when the tool
orientation barely varies (which is why every earlier capture at roll ~67 deg
could not tell them apart).

Residuals are real reprojection errors of the printed marker corners, through
the measured joints and the stored mount.  The stored mount is MuJoCo
convention (local -Z is the view axis); OpenCV's projector needs +Z forward, so
the camera frame is composed with ``FLIP`` before projecting.

    .venv/bin/python tools/phase3_fit.py --captures live/captures_phase3 \\
        --out reports/phase3_fit.json
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from chess_sim.projection import FLIP as CAM_FLIP, pnp_tool_frame  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.kinematics import ArmKinematics, frame_at  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
FLIP = np.diag([1.0, -1.0, -1.0, 1.0])
LAYOUT = CharucoLayout(square_mm=28.0, marker_mm=20.0)


def pose(params):
    m = np.eye(4)
    m[:3, :3] = cv2.Rodrigues(np.asarray(params[:3], float))[0]
    m[:3, 3] = np.asarray(params[3:6], float)
    return m


def params_of(T):
    rvec = cv2.Rodrigues(np.asarray(T)[:3, :3])[0].ravel()
    return np.r_[rvec, np.asarray(T)[:3, 3]]


def corners3d(marker_id):
    xy = np.asarray(LAYOUT.corners_mm(marker_id), float) / 1000.0
    return np.column_stack([xy, np.zeros(4)])


def load_views(dirs, cal):
    fx, fy, cx, cy = cal.intrinsics
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    views = []
    for d in dirs:
        for path in sorted(Path(d).glob("*.json")):
            rec = json.loads(path.read_text())
            joints = rec.get("joints_deg") or rec.get("state_joints_deg")
            if not joints or len(joints) != 6:
                continue
            img = cv2.imread(str(path.with_suffix(".jpg")))
            if img is None:
                continue
            dets = [d for d in fiducials.detect_markers(
                cv2.cvtColor(img, cv2.COLOR_BGR2RGB)) if 5 <= d.marker_id <= 36]
            if not dets:
                continue
            obs = {int(d.marker_id): np.asarray(d.corners, float).reshape(-1, 2)
                   for d in dets}
            views.append({
                "name": path.stem,
                "dir": str(d),
                "joints": [float(joints[n]) for n in NAMES],
                "obs": obs,
                "roll": float(joints["wrist_roll"]),
            })
    return views, K, dist


def reproj_for(view, base_cam_cv, board):
    """Predicted pixels and residuals for one view."""
    T = np.linalg.inv(base_cam_cv) @ board
    rvec = cv2.Rodrigues(T[:3, :3])[0]
    tvec = T[:3, 3].reshape(3, 1)
    out = {}
    for marker_id, uv in view["obs"].items():
        proj, _ = cv2.projectPoints(corners3d(marker_id), rvec, tvec, VIEW_K,
                                    VIEW_DIST)
        out[marker_id] = proj.reshape(-1, 2)
    return out


def residuals(views, base_cams_cv, board):
    res = []
    for view, cam in zip(views, base_cams_cv):
        pred = reproj_for(view, cam, board)
        for marker_id, uv in view["obs"].items():
            res.append((pred[marker_id] - uv).ravel())
    return np.concatenate(res) if res else np.zeros(0)


def camera_positions(views, X_mj):
    """FK camera positions in base, metres."""
    return np.array([(kin.camera_pose(v["joints"])[:3, 3].copy())
                     for v in views])


def fk_camera_cv(views, X_mj, kin):
    """Base->camera (OpenCV axes) for each view: FK(flange) @ X @ FLIP."""
    return np.array([v["_fk"] @ X_mj @ FLIP for v in views])


def pnp_camera_pos(view, board):
    """PnP camera position per marker, in base metres (mean over markers)."""
    lens = {"intrinsics": (VIEW_K[0, 0], VIEW_K[1, 1], VIEW_K[0, 2], VIEW_K[1, 2]),
            "distortion": np.asarray(VIEW_DIST, float).ravel()}
    positions = []
    for marker_id, uv in view["obs"].items():
        solved = pnp_tool_frame(corners3d(marker_id), uv, lens)
        if solved is None:
            continue
        # The helper returns the flipped frame; convert to the OpenCV frame
        # this function works in before taking the camera position.
        R = CAM_FLIP @ solved[0][:3, :3]
        tvec = CAM_FLIP @ solved[0][:3, 3]
        cam_in_board = -R.T @ tvec
        positions.append((board[:3, :3] @ cam_in_board) + board[:3, 3])
    return np.mean(positions, axis=0) if positions else None


def report_metrics(tag, views, X_mj, board, kin, verbose=True):
    cams = fk_camera_cv(views, X_mj, kin)
    res = residuals(views, cams, board)
    rms = float(np.sqrt(np.mean(res ** 2))) if res.size else float("nan")
    errs, per_marker = [], []
    for view, cam in zip(views, cams):
        pnp = pnp_camera_pos(view, board)
        if pnp is None:
            continue
        errs.append(np.linalg.norm(cam[:3, 3] - pnp) * 1000.0)
    errs = np.array(errs)
    out = {"tag": tag, "views": len(views), "reproj_rms_px": round(rms, 3),
           "fk_vs_image_mean_mm": round(float(errs.mean()), 3) if errs.size else None,
           "fk_vs_image_p50_mm": round(float(np.median(errs)), 3) if errs.size else None,
           "fk_vs_image_max_mm": round(float(errs.max()), 3) if errs.size else None}
    if verbose:
        print(json.dumps(out))
    return out, errs


def fit_model(views, kin, X0, B0, fit_x, fit_b, fixed_b=None):
    """Least squares over the selected parameter blocks."""
    X_start = X0.copy() if fixed_b is None else fixed_b[0]
    B_start = B0.copy() if fixed_b is None else fixed_b[1]

    def unpack(p):
        i = 0
        X = pose(p[0:6]) if fit_x else X_start
        i = 6 if fit_x else 0
        B = pose(p[i:i + 6]) if fit_b else B_start
        return X, B

    def residual(p):
        X, B = unpack(p)
        return residuals(views, fk_camera_cv(views, X, kin), B)

    p0 = []
    if fit_x:
        p0 += list(params_of(X_start))
    if fit_b:
        p0 += list(params_of(B_start))
    result = least_squares(residual, np.asarray(p0, float), method="trf",
                           x_scale="jac", xtol=1e-13, ftol=1e-13, gtol=1e-13,
                           max_nfev=4000)
    X, B = unpack(result.x)
    return X, B, result


def cond_number(result, fit_x, fit_b):
    """Condition number of the residual Jacobian, in degrees and millimetres."""
    J = np.asarray(result.jac, float)
    scale = []
    if fit_x:
        scale += [180.0 / math.pi] * 3 + [1000.0] * 3
    if fit_b:
        scale += [180.0 / math.pi] * 3 + [1000.0] * 3
    J = J * np.asarray(scale)[None, :]
    sv = np.linalg.svd(J, compute_uv=False)
    return float(sv[0] / max(sv[-1], 1e-18)), sv.tolist()


def roll_groups(views, rolls):
    out = {}
    for v in views:
        nearest = min(rolls, key=lambda r: abs(v["roll"] - r))
        out.setdefault(nearest, []).append(v)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", nargs="+",
                    default=[str(ROOT / "live" / "captures_phase3")])
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_fit.json"))
    ap.add_argument("--rolls", default="0,40,-40,68,87")
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    board_json = json.loads((ROOT / "live/board.json").read_text())
    board_frame = frame_at(board_json["x_mm"], board_json["y_mm"],
                           board_json["z_mm"], board_json["yaw_deg"])
    kin = ArmKinematics(cal, {}, board_frame)
    X0 = np.asarray(cal.camera_in_tool, float)

    global VIEW_K, VIEW_DIST
    fx, fy, cx, cy = cal.intrinsics
    VIEW_K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    VIEW_DIST = np.asarray(cal.distortion, float).reshape(-1, 1)

    views, _, _ = load_views(args.captures, cal)
    if not views:
        print("no usable views")
        return 2
    rolls = [float(v) for v in args.rolls.split(",")]
    for v in views:
        # flange pose only: camera_in_tool is the parameter being fitted (or
        # held fixed), so it must not already be baked in here.
        v["_fk"] = kin.fk(v["joints"])
    groups = roll_groups(views, rolls)
    print(f"{len(views)} views; roll groups "
          f"{ {r: len(v) for r, v in sorted(groups.items())} }")

    out = {"schema": "phase3-fit/1", "captures": args.captures,
           "views": len(views),
           "roll_groups": {str(k): [v["name"] for v in g]
                           for k, g in sorted(groups.items())}}

    print("\n== baseline (stored X, stored board) ==")
    base_metrics, base_err = report_metrics("baseline", views, X0, board_frame, kin)
    out["baseline"] = base_metrics

    print("\n== model A: board fixed, camera_in_tool free ==")
    XA, BA, rA = fit_model(views, kin, X0, board_frame, True, False)
    mA, _ = report_metrics("A", views, XA, board_frame, kin)
    condA, svA = cond_number(rA, True, False)
    out["A"] = {"metrics": mA, "cond": condA, "sv": svA,
                "X_delta_translation_mm": (np.asarray(XA)[:3, 3] - X0[:3, 3]) * 1000,
                "X": np.asarray(XA).tolist()}

    print("\n== model B: camera_in_tool fixed, board free ==")
    XB, BB, rB = fit_model(views, kin, X0, board_frame, False, True)
    mB, _ = report_metrics("B", views, X0, BB, kin)
    condB, svB = cond_number(rB, False, True)
    out["B"] = {"metrics": mB, "cond": condB, "sv": svB,
                "board_delta_mm": (np.asarray(BB)[:3, 3] - board_frame[:3, 3]) * 1000,
                "B": np.asarray(BB).tolist()}

    print("\n== model C: both free ==")
    XC, BC, rC = fit_model(views, kin, XA, BB, True, True)
    mC, _ = report_metrics("C", views, XC, BC, kin)
    condC, svC = cond_number(rC, True, True)
    out["C"] = {"metrics": mC, "cond": condC, "sv": svC,
                "X": np.asarray(XC).tolist(), "B": np.asarray(BC).tolist()}

    print("\n== leave-one-roll-group-out (held-out reprojection RMS px) ==")
    loo = {}
    for held in sorted(groups):
        train = [v for r, g in groups.items() if r != held for v in g]
        test = groups[held]
        if len(train) < 10 or not test:
            continue
        row = {"n_train": len(train), "n_test": len(test)}
        for tag in ("A", "B", "C"):
            if tag == "A":
                X, B, _ = fit_model(train, kin, X0, board_frame, True, False)
            elif tag == "B":
                X, B, _ = fit_model(train, kin, X0, board_frame, False, True)
            else:
                X, B, _ = fit_model(train, kin, XA, BB, True, True)
            held_cams = fk_camera_cv(test, X, kin)
            res = residuals(test, held_cams, B)
            rms = float(np.sqrt(np.mean(res ** 2))) if res.size else float("nan")
            errs = []
            for v, cam in zip(test, held_cams):
                pnp = pnp_camera_pos(v, B)
                if pnp is not None:
                    errs.append(np.linalg.norm(cam[:3, 3] - pnp) * 1000.0)
            row[tag] = {"rms_px": round(rms, 3),
                        "fk_vs_image_mean_mm": round(float(np.mean(errs)), 2)}
        loo[str(held)] = row
        print(f"  held roll {held:5.0f} ({row['n_test']} views): "
              f"A {row['A']}  B {row['B']}  C {row['C']}")

    out["leave_one_roll_out_mm"] = loo
    if loo:
        for tag in ("A", "B", "C"):
            vals = [row[tag]["fk_vs_image_mean_mm"] for row in loo.values()]
            out.setdefault("loo_summary", {})[tag] = {
                "mean_heldout_mm": round(float(np.mean(vals)), 2),
                "max_heldout_mm": round(float(np.max(vals)), 2)}
        print("\nheld-out summary:", json.dumps(out["loo_summary"]))

    # per-roll residual for the full fits, so a roll-dependent miss is visible
    per_roll = {}
    for roll, vs in sorted(groups.items()):
        per_roll[str(roll)] = {}
        for tag, (X, B) in {"baseline": (X0, board_frame), "A": (XA, board_frame),
                            "B": (X0, BB), "C": (XC, BC)}.items():
            _, errs = report_metrics(tag, vs, X, B, kin, verbose=False)
            per_roll[str(roll)][tag] = round(float(errs.mean()), 2) if errs.size else None
    out["per_roll_fk_vs_image_mm"] = per_roll
    print("\nper-roll FK-vs-image mean (mm):", json.dumps(per_roll, indent=1))

    def enc(o):
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(type(o))

    Path(args.out).write_text(json.dumps(out, indent=2, default=enc) + "\n")
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
