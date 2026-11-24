#!/usr/bin/env python3
"""Alternating rigid solve for (mount, board) from absolute per-view PnP poses.

Each view gives T_cam_board.  With the board fixed, every view implies a mount;
with the mount fixed, every view implies a board.  Alternating the two rigid
means is the documented staged solve (docs/CHARUCO_HANDEYE.md), and with a roll
sweep the two are no longer collinear, so this converges to a pair that is
testable: fit on four roll groups, predict the fifth.

Reports the pair, the change from the stored values, the conditioning of the
2-block normal equations, and held-out camera-position error.

    .venv/bin/python tools/phase3_pair.py --captures live/captures_phase3
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.kinematics import ArmKinematics, frame_at  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
LAYOUT = CharucoLayout(28.0, 20.0)
FLIP = np.diag([1.0, -1.0, -1.0, 1.0])


def rigid_mean(Ts, weights=None):
    w = np.ones(len(Ts)) if weights is None else np.asarray(weights, float)
    R = np.zeros((3, 3))
    for T, wi in zip(Ts, w):
        R += wi * T[:3, :3]
    u, _, vt = np.linalg.svd(R)
    Rn = u @ vt
    if np.linalg.det(Rn) < 0:
        u[:, -1] *= -1
        Rn = u @ vt
    t = np.average(np.array([T[:3, 3] for T in Ts]), axis=0, weights=w)
    out = np.eye(4)
    out[:3, :3] = Rn
    out[:3, 3] = t
    return out


def load(dirs, cal, min_markers=2, rms_cap=6.0):
    fx, fy, cx, cy = cal.intrinsics
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    views = []
    for d in dirs:
        for path in sorted(Path(d).glob("*.json")):
            rec = json.loads(path.read_text())
            joints = rec.get("joints_deg")
            if not joints or len(joints) != 6:
                continue
            img = cv2.imread(str(path.with_suffix(".jpg")))
            if img is None:
                continue
            dets = [x for x in fiducials.detect_markers(
                cv2.cvtColor(img, cv2.COLOR_BGR2RGB)) if 5 <= x.marker_id <= 36]
            if len(dets) < min_markers:
                continue
            obj, pix = [], []
            for x in dets:
                xy = np.asarray(LAYOUT.corners_mm(x.marker_id), float) / 1000.0
                obj.append(np.column_stack([xy, np.zeros(4)]))
                pix.append(np.asarray(x.corners, float).reshape(-1, 2))
            obj = np.vstack(obj).reshape(-1, 1, 3)
            pix = np.vstack(pix).reshape(-1, 1, 2)
            ok, rvec, tvec = cv2.solvePnP(obj, pix, K, dist,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
            rms = float(np.sqrt(np.mean(np.sum(
                (proj.reshape(-1, 2) - pix.reshape(-1, 2)) ** 2, axis=1))))
            if rms > rms_cap:
                continue
            T = np.eye(4)
            T[:3, :3] = cv2.Rodrigues(rvec)[0]
            T[:3, 3] = np.ravel(tvec)
            views.append({"name": path.stem, "joints": [float(joints[n]) for n in NAMES],
                          "roll": float(joints["wrist_roll"]), "markers": len(dets),
                          "rms_px": rms, "T": T, "fk": None,
                          "obs": {int(x.marker_id): np.asarray(x.corners, float).reshape(-1, 2)
                                  for x in dets}})
    return views


def alternate(views, kin, B0, X0, iters=12):
    B = np.asarray(B0, float)
    X = np.asarray(X0, float)
    hist = []
    for _ in range(iters):
        Xs = []
        for v in views:
            base_cam_from_board = B @ np.linalg.inv(v["T"])
            Xs.append(np.linalg.inv(v["fk"]) @ base_cam_from_board @ FLIP)
        X = rigid_mean(Xs)
        Bs = [v["fk"] @ X @ FLIP @ v["T"] for v in views]
        B = rigid_mean(Bs)
        hist.append((np.linalg.norm(X[:3, 3]), np.linalg.norm(B[:3, 3])))
    return X, B, hist


def cam_error(views, X, B):
    """Mean |FK@X - PnP camera| in mm, with the board pose B applied to PnP."""
    errs = []
    for v in views:
        cam = v["fk"] @ X
        pnp = (B[:3, :3] @ (-v["T"][:3, :3].T @ v["T"][:3, 3])) + B[:3, 3]
        errs.append(np.linalg.norm(cam[:3, 3] - pnp) * 1000.0)
    return np.array(errs)


def err_vec(views, X, B):
    out = []
    for v in views:
        cam = v["fk"] @ X
        pnp = (B[:3, :3] @ (-v["T"][:3, :3].T @ v["T"][:3, 3])) + B[:3, 3]
        out.append((cam[:3, 3] - pnp) * 1000.0)
    return np.concatenate(out)


def cond_of(views, X, B, fracs=(1, 0)):
    """Condition number of the joint (mount, board) normal matrix.

    ``fracs`` selects which blocks are free: (1,0) mount only, (0,1) board only,
    (1,1) both.  Rotations are perturbed in milliradians, translations in mm.
    """
    free_mount, free_board = fracs
    base = err_vec(views, X, B)
    cols = []
    for block in ("X", "B"):
        if (block == "X" and not free_mount) or (block == "B" and not free_board):
            continue
        for axis in range(3):
            p = np.zeros(6); p[axis] = 1e-3
            Xp, Bp = X.copy(), B.copy()
            dR = Rotation.from_rotvec(p[:3]).as_matrix()
            if block == "X":
                Xp[:3, :3] = X[:3, :3] @ dR.T
            else:
                Bp[:3, :3] = B[:3, :3] @ dR.T
            cols.append((err_vec(views, Xp, Bp) - base) / 1e-3)
        for axis in range(3):
            Xp, Bp = X.copy(), B.copy()
            t = np.zeros(3); t[axis] = 1.0
            if block == "X":
                Xp[:3, 3] = X[:3, 3] + t / 1000.0
            else:
                Bp[:3, 3] = B[:3, 3] + t / 1000.0
            cols.append((err_vec(views, Xp, Bp) - base) / 1.0)
    J = np.column_stack(cols)
    sv = np.linalg.svd(J, compute_uv=False)
    return float(sv[0] / max(sv[-1], 1e-15)), sv.tolist()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", nargs="+", default=[str(ROOT / "live" / "captures_phase3")])
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_pair.json"))
    ap.add_argument("--min-markers", type=int, default=2)
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    b = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])
    kin = ArmKinematics(cal, {}, board)
    X0 = np.asarray(cal.camera_in_tool, float)
    views = load(args.captures, cal, args.min_markers)
    for v in views:
        v["fk"] = kin.fk(v["joints"])
    print(f"{len(views)} usable views")
    if len(views) < 8:
        return 2

    e0 = cam_error(views, X0, board)
    print(f"baseline camera-position error: mean {e0.mean():.2f}  "
          f"p50 {np.median(e0):.2f}  max {e0.max():.2f} mm")

    X, B, hist = alternate(views, kin, board, X0, iters=25)
    e = cam_error(views, X, B)
    print(f"\nalternating solve: mean {e.mean():.2f}  p50 {np.median(e):.2f}  "
          f"max {e.max():.2f} mm")
    print(f"mount  t {np.round(X[:3,3]*1000,2).tolist()} mm  "
          f"euler {np.round(Rotation.from_matrix(X[:3,:3]).as_euler('xyz', degrees=True),1).tolist()} "
          f"(stored {np.round(X0[:3,3]*1000,2).tolist()}, "
          f"{np.round(Rotation.from_matrix(X0[:3,:3]).as_euler('xyz', degrees=True),1).tolist()})")
    print(f"delta  t {np.round((X[:3,3]-X0[:3,3])*1000,2).tolist()} mm  "
          f"rotation {np.degrees(np.linalg.norm(Rotation.from_matrix(X0[:3,:3].T @ X[:3,:3]).as_rotvec())):.2f} deg")
    print(f"board  t {np.round(B[:3,3]*1000,2).tolist()} mm  "
          f"yaw {math.degrees(math.atan2(B[1,0], B[0,0])):.2f} "
          f"(stored {[b['x_mm'], b['y_mm'], b['z_mm']]}, yaw {b['yaw_deg']})")
    print(f"delta  t {np.round((B[:3,3]-board[:3,3])*1000,2).tolist()} mm  "
          f"yaw {((math.degrees(math.atan2(B[1,0], B[0,0])) - b['yaw_deg'] + 180) % 360) - 180:.2f} deg")

    cX, _ = cond_of(views, X, B, (1, 0))
    cB, _ = cond_of(views, X, B, (0, 1))
    cJ, sv = cond_of(views, X, B, (1, 1))
    print(f"conditioning (Jacobian of the camera-position residual): "
          f"mount-only {cX:.1f}, board-only {cB:.1f}, joint {cJ:.1f}")

    groups = {}
    for v in views:
        groups.setdefault(round(v["roll"]), []).append(v)
    loo = {}
    print("\nleave-one-roll-out, held-out camera-position error (mm):")
    for held in sorted(groups):
        train = [v for r, g in groups.items() if r != held for v in g]
        test = groups[held]
        if len(train) < 8:
            continue
        Xt, Bt, _ = alternate(train, kin, board, X0, iters=25)
        e_mount = np.mean(cam_error(test, Xt, board))
        e_board = np.mean(cam_error(test, X0, Bt))
        e_both = np.mean(cam_error(test, Xt, Bt))
        loo[str(held)] = {"n": len(test), "mount_only": round(float(e_mount), 2),
                          "board_only": round(float(e_board), 2),
                          "both": round(float(e_both), 2)}
        print(f"  held roll {held:5d} ({len(test):2d}): mount {e_mount:5.2f}  "
              f"board {e_board:5.2f}  both {e_both:5.2f}")
    if loo:
        for k in ("mount_only", "board_only", "both"):
            print(f"  mean {k:11s}: {np.mean([r[k] for r in loo.values()]):5.2f} mm")

    Path(args.out).write_text(json.dumps({
        "schema": "phase3-pair/1", "views": len(views),
        "baseline_mean_mm": round(float(e0.mean()), 3),
        "solved_mean_mm": round(float(e.mean()), 3),
        "mount": X.tolist(), "board": B.tolist(),
        "mount_delta_t_mm": np.round((X[:3, 3] - X0[:3, 3]) * 1000, 3).tolist(),
        "board_delta_t_mm": np.round((B[:3, 3] - board[:3, 3]) * 1000, 3).tolist(),
        "cond_mount": cX, "cond_board": cB, "cond_joint": cJ,
        "singular_values": sv,
        "leave_one_roll_out": loo,
    }, indent=2) + "\n")
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
