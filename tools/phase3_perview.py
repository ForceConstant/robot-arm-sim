#!/usr/bin/env python3
"""Per-view hand-eye candidates: what does each capture say the mount (and the
board pose) is, with no global fit to hide a pose-dependent error?

For each view:
  T_cam_board  = multi-marker PnP over every decoded tag in that frame
  X_v          = FK(q)^-1 @ B_stored @ T_cam_board     (mount implied by the view)
  B_v          = FK(q) @ X_stored @ T_cam_board        (board implied by the view)

If the stored mount is right, B_v is constant and X_v scatters only with PnP
noise.  If the board pose is right, X_v is constant and B_v scatters.  A
systematic trend of X_v with wrist_roll is neither: it is a kinematic or
mount-orientation error that no single rigid transform can absorb.

    .venv/bin/python tools/phase3_perview.py --captures live/captures_phase3
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", nargs="+",
                    default=[str(ROOT / "live" / "captures_phase3")])
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_perview.json"))
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    b = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])
    kin = ArmKinematics(cal, {}, board)
    X0 = np.asarray(cal.camera_in_tool, float)
    fx, fy, cx, cy = cal.intrinsics
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.asarray(cal.distortion, float).reshape(-1, 1)
    layout = CharucoLayout(28.0, 20.0)

    rows = []
    for d in args.captures:
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
            if len(dets) < 2:
                continue
            obj, pix = [], []
            for x in dets:
                xy = np.asarray(layout.corners_mm(x.marker_id), float) / 1000.0
                obj.append(np.column_stack([xy, np.zeros(4)]))
                pix.append(np.asarray(x.corners, float).reshape(-1, 2))
            obj = np.vstack(obj).reshape(-1, 1, 3)
            pix = np.vstack(pix).reshape(-1, 1, 2)
            ok, rvec, tvec = cv2.solvePnP(obj, pix, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            R = cv2.Rodrigues(rvec)[0]
            cam_in_board = np.eye(4)
            cam_in_board[:3, :3] = R
            cam_in_board[:3, 3] = np.ravel(tvec)
            # reprojection rms of the multi-marker solve
            proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
            rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - pix.reshape(-1, 2)) ** 2, axis=1))))
            fk = kin.fk([float(joints[n]) for n in NAMES])
            # PnP is OpenCV convention (+Z view); the stored mount is MuJoCo
            # (-Z view).  FLIP is that 180 deg about local X and is its own
            # inverse.
            flip = np.diag([1.0, -1.0, -1.0, 1.0])
            base_cam_cv = fk @ X0 @ flip
            Bv = base_cam_cv @ cam_in_board         # board pose implied by the stored mount
            base_cam_from_board = board @ np.linalg.inv(cam_in_board)
            Xv = np.linalg.inv(fk) @ base_cam_from_board @ flip   # implied mount
            rows.append({
                "name": path.stem, "roll": float(joints["wrist_roll"]),
                "markers": len(dets), "rms_px": round(rms, 2),
                "X_t_mm": (np.linalg.inv(Xv)[:3, 3] * 1000).tolist(),
                "X_rot_deg": Rotation.from_matrix(np.linalg.inv(Xv)[:3, :3]).as_euler("xyz", degrees=True).tolist(),
                "B_t_mm": (Bv[:3, 3] * 1000).tolist(),
                "B_yaw_deg": float(math.degrees(math.atan2(Bv[1, 0], Bv[0, 0]))),
            })

    if not rows:
        print("no views with >=3 markers")
        return 2

    X0t = np.asarray(X0)[:3, 3] * 1000
    R0 = Rotation.from_matrix(np.asarray(X0)[:3, :3]).as_euler("xyz", degrees=True)
    print(f"{len(rows)} views with >=2 markers")
    print(f"stored X: t {np.round(X0t,1).tolist()} mm  euler {np.round(R0,1).tolist()} deg")
    print(f"stored B: t {[b['x_mm'], b['y_mm'], b['z_mm']]} yaw {b['yaw_deg']}\n")
    print("roll   n  |  X t (mm)                |  dX vs stored          |  B t (mm)              |  B yaw")
    for roll in sorted({round(r["roll"]) for r in rows}):
        sel = [r for r in rows if round(r["roll"]) == roll]
        Xt = np.array([r["X_t_mm"] for r in sel])
        Bt = np.array([r["B_t_mm"] for r in sel])
        By = np.array([r["B_yaw_deg"] for r in sel])
        med = np.median(Xt, axis=0)
        print(f"{roll:5d} {len(sel):2d}  | {str(np.round(med,1).tolist()):24s} | "
              f"{str(np.round(med - X0t,1).tolist()):22s} | "
              f"{str(np.round(np.median(Bt,0),1).tolist()):22s} | {np.median(By):7.2f}")
    sel = rows
    Xt = np.array([r["X_t_mm"] for r in sel])
    print(f"\nall: X t median {np.round(np.median(Xt,0),1).tolist()} "
          f"sd {np.round(Xt.std(0),1).tolist()}")
    Path(args.out).write_text(json.dumps({"stored_X_t_mm": X0t.round(2).tolist(),
                                          "stored_X_euler_deg": np.round(R0, 2).tolist(),
                                          "views": rows}, indent=2) + "\n")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
