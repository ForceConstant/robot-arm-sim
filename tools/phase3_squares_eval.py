#!/usr/bin/env python3
"""Before/after FK-vs-image on identical 64-square frames.

Reads the sweep receipt (joints + marker corners per square) and evaluates the
camera-position disagreement twice: once through the mount that was in force
before Phase 3 (the .bak file) and once through the current one.  Same images,
same board pose, only the mount differs.

    .venv/bin/python tools/phase3_squares_eval.py --old <backup calibration>
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

from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.kinematics import ArmKinematics, frame_at  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
LAYOUT = CharucoLayout(28.0, 20.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--receipt", default=str(ROOT / "reports" / "phase3_squares_irl.json"))
    ap.add_argument("--old", required=True)
    ap.add_argument("--new", default=None)
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_squares_eval.json"))
    ap.add_argument("--min-markers", type=int, default=2)
    ap.add_argument("--rms-cap", type=float, default=4.0)
    ap.add_argument("--board", default=None,
                    help="optional board pose json to try instead of live/board.json")
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    new_path = args.new or str(ROOT / cfg["calibration"])
    cal_new = WristCalibration.load(new_path)
    cal_old = WristCalibration.load(args.old)
    bpath = args.board or str(ROOT / "live" / "board.json")
    b = json.loads(open(bpath).read())
    board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])

    receipt = json.loads(Path(args.receipt).read_text())

    def evaluate(cal, X):
        fx, fy, cx, cy = cal.intrinsics
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        dist = np.asarray(cal.distortion, float).reshape(-1, 1)
        kin = ArmKinematics(cal, {}, board)
        errs, rows = [], []
        for rec in receipt["squares"]:
            if not rec.get("markers"):
                continue
            joints = [float(rec["joints_deg"][n]) for n in NAMES]
            fk = kin.fk(joints)
            cam = fk @ X
            obj, pix = [], []
            for m in rec["markers"]:
                xy = np.asarray(LAYOUT.corners_mm(m["id"]), float) / 1000.0
                obj.append(np.column_stack([xy, np.zeros(4)]))
                pix.append(np.asarray(m["corners"], float).reshape(-1, 2))
            o = np.vstack(obj).reshape(-1, 1, 3)
            p = np.vstack(pix).reshape(-1, 1, 2)
            ok, rvec, tvec = cv2.solvePnP(o, p, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(o, rvec, tvec, K, dist)
            rms = float(np.sqrt(np.mean(np.sum(
                (proj.reshape(-1, 2) - p.reshape(-1, 2)) ** 2, axis=1))))
            if rms > args.rms_cap or len(rec["markers"]) < args.min_markers:
                continue
            pnp = (board[:3, :3] @ (-cv2.Rodrigues(rvec)[0].T @ np.ravel(tvec))) + board[:3, 3]
            e = float(np.linalg.norm(cam[:3, 3] - pnp) * 1000)
            errs.append(e)
            rows.append({"square": rec["square"], "markers": len(rec["markers"]),
                         "err_mm": round(e, 2)})
        return np.array(errs), rows

    e_old, rows_old = evaluate(cal_old, np.asarray(cal_old.camera_in_tool, float))
    e_new, rows_new = evaluate(cal_new, np.asarray(cal_new.camera_in_tool, float))
    print(f"frames with markers: {len(e_old)}")
    for tag, e in (("before", e_old), ("after", e_new)):
        if not e.size:
            continue
        print(f"  {tag:6s}: mean {e.mean():6.2f}  p50 {np.median(e):6.2f}  "
              f"p90 {np.percentile(e, 90):6.2f}  max {e.max():6.2f} mm")
    out = {"schema": "phase3-squares-eval/1", "frames": int(len(e_old)),
           "before": {"mean": round(float(e_old.mean()), 3) if e_old.size else None,
                      "p50": round(float(np.median(e_old)), 3) if e_old.size else None,
                      "p90": round(float(np.percentile(e_old, 90)), 3) if e_old.size else None,
                      "max": round(float(e_old.max()), 3) if e_old.size else None},
           "after": {"mean": round(float(e_new.mean()), 3) if e_new.size else None,
                     "p50": round(float(np.median(e_new)), 3) if e_new.size else None,
                     "p90": round(float(np.percentile(e_new, 90)), 3) if e_new.size else None,
                     "max": round(float(e_new.max()), 3) if e_new.size else None},
           "per_square": [{"square": a["square"], "markers": a["markers"],
                           "before_mm": a["err_mm"], "after_mm": c["err_mm"]}
                          for a, c in zip(rows_old, rows_new)]}
    Path(args.out).write_text(json.dumps(out, indent=2) + "\n")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
