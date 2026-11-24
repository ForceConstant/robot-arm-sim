#!/usr/bin/env python3
"""Identifiability in the domain that defines the 18.74 mm error.

The error being chased is a *camera-position* disagreement: PnP says the camera
is here, forward kinematics through the stored mount says it is there.  Written
per view v:

    r_v = p_fk(q_v) - p_pnp(v)                        (3-vector, base frame)

Two rigid explanations:

    A  board origin shifted by d      ->  r_v = -d                + noise
    B  camera mount shifted by delta  ->  r_v = R_tool(q_v) delta + noise

where R_tool is the tool rotation in base.  With wrist_roll held at one value
(as in every capture before today) R_tool barely varies, so A and B are
collinear.  With a roll sweep they are separable: a mount error rotates with
the tool, a board error does not.

The combined design [R_tool | -I] gives both, and its condition number is the
honest statement about how well this dataset can tell them apart.  Leave one
roll group out and predict it: the model with the smaller held-out error is the
one the data supports.

    .venv/bin/python tools/phase3_linear.py --captures live/captures_phase3
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

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


def load_views(dirs, cal, min_markers=2):
    fx, fy, cx, cy = cal.intrinsics
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    out = []
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
            out.append({
                "name": path.stem, "dir": str(d),
                "joints": [float(joints[n]) for n in NAMES],
                "roll": float(joints["wrist_roll"]),
                "markers": len(dets), "rms_px": rms,
                "R_pnp": cv2.Rodrigues(rvec)[0], "t_pnp": np.ravel(tvec),
            })
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", nargs="+",
                    default=[str(ROOT / "live" / "captures_phase3")])
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_linear.json"))
    ap.add_argument("--min-markers", type=int, default=2)
    ap.add_argument("--rms-cap", type=float, default=6.0)
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    b = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])
    kin = ArmKinematics(cal, {}, board)
    X0 = np.asarray(cal.camera_in_tool, float)

    views = load_views(args.captures, cal, args.min_markers)
    views = [v for v in views if v["rms_px"] <= args.rms_cap]
    if len(views) < 8:
        print(f"only {len(views)} usable views; not enough")
        return 2
    print(f"{len(views)} usable views (>= {args.min_markers} markers, "
          f"rms <= {args.rms_cap}px)")

    for v in views:
        fk = kin.fk(v["joints"])
        v["R_tool"] = fk[:3, :3]
        cam_in_board = np.eye(4)
        cam_in_board[:3, :3] = v["R_pnp"]
        cam_in_board[:3, 3] = v["t_pnp"]
        v["p_pnp"] = (board[:3, :3] @ (-v["R_pnp"].T @ v["t_pnp"])) + board[:3, 3]
        v["p_fk"] = (fk @ X0)[:3, 3]
        v["r"] = v["p_fk"] - v["p_pnp"]

    R = np.array([v["r"] for v in views]) * 1000.0
    print(f"baseline: |r| mean {np.linalg.norm(R, axis=1).mean():.2f} mm  "
          f"mean vec {np.round(R.mean(0), 1).tolist()}  "
          f"sd {np.round(R.std(0), 1).tolist()}")

    def design(views, mode):
        rows = []
        for v in views:
            Rt = v["R_tool"]
            if mode == "A":
                rows.append(-np.eye(3))
            elif mode == "B":
                rows.append(Rt)
            else:
                rows.append(np.hstack([Rt, -np.eye(3)]))
        return np.vstack(rows)

    def solve(views, mode):
        A = design(views, mode)
        y = np.concatenate([v["r"] for v in views]) * 1000.0
        x, *_ = np.linalg.lstsq(A, y, rcond=None)
        res = (y - A @ x).reshape(-1, 3)
        return x, res

    names = {"A": "board origin d (base)", "B": "mount delta (tool)",
             "C": "both"}
    fits = {}
    for mode in ("A", "B", "C"):
        x, res = solve(views, mode)
        n = np.linalg.norm(res, axis=1)
        A = design(views, mode)
        sv = np.linalg.svd(A, compute_uv=False)
        fits[mode] = {"x_mm": np.round(x * 1000, 2).tolist(),
                      "resid_mean_mm": round(float(n.mean()), 2),
                      "resid_p50_mm": round(float(np.median(n)), 2),
                      "resid_max_mm": round(float(n.max()), 2),
                      "cond": round(float(sv[0] / max(sv[-1], 1e-12)), 2)}
        print(f"model {mode} ({names[mode]}): x {np.round(x*1000,2).tolist()} mm  "
              f"residual mean {n.mean():5.2f} p50 {np.median(n):5.2f} "
              f"max {n.max():5.2f}  cond {fits[mode]['cond']}")

    # LOO by roll group
    groups = {}
    for v in views:
        groups.setdefault(round(v["roll"]), []).append(v)
    loo = {}
    print("\nleave-one-roll-group-out (held-out |r| mean mm, and held-out "
          "residual of the fitted model):")
    for held in sorted(groups):
        train = [v for r, g in groups.items() if r != held for v in g]
        test = groups[held]
        if len(train) < 8:
            continue
        row = {"n_test": len(test)}
        for mode in ("A", "B", "C"):
            x, _ = solve(train, mode)
            A_test = design(test, mode)
            y_test = np.concatenate([v["r"] for v in test]) * 1000.0
            res = (y_test - A_test @ x).reshape(-1, 3)
            row[mode] = {"heldout_resid_mean_mm": round(float(np.linalg.norm(res, axis=1).mean()), 2),
                         "x_mm": np.round(x * 1000, 1).tolist()}
        loo[str(held)] = row
        print(f"  held roll {held:5d} ({len(test)} views): " + "  ".join(
            f"{m} {row[m]['heldout_resid_mean_mm']:5.2f}mm" for m in "ABC"))

    out = {"schema": "phase3-linear/1", "captures": args.captures,
           "views": len(views), "baseline_mean_mm":
           round(float(np.linalg.norm(R, axis=1).mean()), 3),
           "fits": fits, "leave_one_roll_out": loo,
           "per_view": [{"name": v["name"], "roll": v["roll"],
                         "markers": v["markers"], "rms_px": round(v["rms_px"], 2),
                         "r_mm": np.round(v["r"] * 1000, 2).tolist()} for v in views]}
    Path(args.out).write_text(json.dumps(out, indent=2) + "\n")
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
