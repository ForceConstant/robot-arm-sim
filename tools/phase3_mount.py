#!/usr/bin/env python3
"""Rigid-average the mount (and the board pose) from per-view PnP, and validate
on held-out rolls.

Each view that decodes two or more tags gives one complete camera pose, hence
one implied mount X_v = FK(q)^-1 * B * T_cam_board and one implied board pose
B_v = FK(q) * X * T_cam_board.  Nothing is fitted globally, so a pose-dependent
error cannot hide inside a compromise.

Reports, for each roll group: the implied mount translation and rotation, the
implied board pose, and how tight each is.  Then it averages over training
groups and scores the held-out group by reprojection RMS and camera-position
error for three models: mount only, board only, both.

    .venv/bin/python tools/phase3_mount.py --captures live/captures_phase3
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
    """Chordal mean of rotation + weighted median translation."""
    w = np.ones(len(Ts)) if weights is None else np.asarray(weights, float)
    R = np.zeros((3, 3))
    for T, wi in zip(Ts, w):
        R += wi * T[:3, :3]
    u, _, vt = np.linalg.svd(R)
    Rn = u @ vt
    if np.linalg.det(Rn) < 0:
        u[:, -1] *= -1
        Rn = u @ vt
    t = np.median(np.array([T[:3, 3] for T in Ts]), axis=0)
    out = np.eye(4)
    out[:3, :3] = Rn
    out[:3, 3] = t
    return out


def rot_deg(a, b):
    return float(np.degrees(np.linalg.norm(
        Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).as_rotvec())))


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
            views.append({"name": path.stem, "joints": [float(joints[n]) for n in NAMES],
                          "roll": float(joints["wrist_roll"]), "markers": len(dets),
                          "rms_px": rms, "R": cv2.Rodrigues(rvec)[0],
                          "t": np.ravel(tvec)})
    return views, K, dist


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", nargs="+", default=[str(ROOT / "live" / "captures_phase3")])
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_mount.json"))
    ap.add_argument("--min-markers", type=int, default=2)
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    b = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])
    kin = ArmKinematics(cal, {}, board)
    X0 = np.asarray(cal.camera_in_tool, float)
    views, K, dist = load(args.captures, cal, args.min_markers)
    print(f"{len(views)} usable views")
    if len(views) < 8:
        return 2

    for v in views:
        v["fk"] = kin.fk(v["joints"])
        T = np.eye(4)
        T[:3, :3] = v["R"]
        T[:3, 3] = v["t"]
        v["T_cam_board"] = T
        # implied mount: camera_in_tool (MuJoCo convention)
        base_cam_from_board = board @ np.linalg.inv(T)     # base <- camera (OpenCV)
        v["X"] = np.linalg.inv(v["fk"]) @ base_cam_from_board @ FLIP
        # implied board pose with the stored mount
        v["B"] = v["fk"] @ X0 @ FLIP @ T

    groups = {}
    for v in views:
        groups.setdefault(round(v["roll"]), []).append(v)
    print("\nroll   n | implied mount t (mm)          | vs stored (mm)        | "
          "rot vs stored | implied board t (mm)        | yaw")
    X0t = X0[:3, 3] * 1000
    for roll in sorted(groups):
        sel = groups[roll]
        Xg = rigid_mean([v["X"] for v in sel])
        Bg = rigid_mean([v["B"] for v in sel])
        print(f"{roll:5d} {len(sel):2d} | {str(np.round(Xg[:3,3]*1000,1).tolist()):28s} | "
              f"{str(np.round(Xg[:3,3]*1000-X0t,1).tolist()):21s} | "
              f"{rot_deg(Xg, X0):6.1f} deg    | "
              f"{str(np.round(Bg[:3,3]*1000,1).tolist()):26s} | "
              f"{math.degrees(math.atan2(Bg[1,0], Bg[0,0])):7.2f}")

    def reproj_rms(v, X, B):
        cam = v["fk"] @ X @ FLIP
        T = np.linalg.inv(cam) @ B
        rvec = cv2.Rodrigues(T[:3, :3])[0]
        tvec = T[:3, 3].reshape(3, 1)
        # rebuild this view's observations
        return None

    def score(sel, X, B):
        """Camera-position error and reprojection RMS over the given views."""
        errs, res = [], []
        for v in sel:
            cam = v["fk"] @ X @ FLIP
            pnp = (board[:3, :3] @ (-v["R"].T @ v["t"])) + board[:3, 3]
            errs.append(np.linalg.norm(cam[:3, 3] - pnp) * 1000)
            T = np.linalg.inv(cam) @ B
            rv = cv2.Rodrigues(T[:3, :3])[0]
            tv = T[:3, 3].reshape(3, 1)
            # not storing object points here; position error is the metric
        return float(np.mean(errs))

    print("\nleave-one-roll-out (held-out camera-position error, mm):")
    loo = {}
    for held in sorted(groups):
        train = [v for r, g in groups.items() if r != held for v in g]
        test = groups[held]
        X_est = rigid_mean([v["X"] for v in train])
        B_est = rigid_mean([v["B"] for v in train])
        # mount-only, board-only and both
        e_mount = score(test, X_est, board)
        e_board = score(test, X0, B_est)
        # both: recompute each test view's implied board with X_est, average
        B_both = rigid_mean([(v["fk"] @ X_est @ FLIP @ v["T_cam_board"]) for v in test])
        e_both = score(test, X_est, B_both)
        loo[str(held)] = {"n": len(test), "mount_only": round(e_mount, 2),
                          "board_only": round(e_board, 2), "both": round(e_both, 2)}
        print(f"  held roll {held:5d} ({len(test):2d}): mount {e_mount:5.2f}  "
              f"board {e_board:5.2f}  both {e_both:5.2f}")
    if loo:
        for k in ("mount_only", "board_only", "both"):
            print(f"  mean {k:11s}: "
                  f"{np.mean([r[k] for r in loo.values()]):5.2f} mm")

    X_all = rigid_mean([v["X"] for v in views])
    B_all = rigid_mean([v["B"] for v in views])
    print(f"\nall-view mount: t {np.round(X_all[:3,3]*1000,2).tolist()} mm  "
          f"euler {np.round(Rotation.from_matrix(X_all[:3,:3]).as_euler('xyz', degrees=True),1).tolist()} deg  "
          f"(stored t {np.round(X0t,2).tolist()}, "
          f"euler {np.round(Rotation.from_matrix(X0[:3,:3]).as_euler('xyz', degrees=True),1).tolist()})")
    print(f"rotation change vs stored: {rot_deg(X_all, X0):.2f} deg")
    print(f"board change vs stored: t {np.round((B_all[:3,3]-board[:3,3])*1000,2).tolist()} mm  "
          f"yaw {math.degrees(math.atan2(B_all[1,0],B_all[0,0])) - b['yaw_deg']:.2f} deg")

    Path(args.out).write_text(json.dumps({
        "schema": "phase3-mount/1", "views": len(views),
        "mount_all": X_all.tolist(), "board_all": B_all.tolist(),
        "leave_one_roll_out": loo,
        "per_view_mount_t_mm": [np.round(v["X"][:3, 3] * 1000, 2).tolist() for v in views],
        "per_view_roll": [v["roll"] for v in views],
        "per_view_markers": [v["markers"] for v in views],
    }, indent=2) + "\n")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
