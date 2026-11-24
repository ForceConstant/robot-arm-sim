#!/usr/bin/env python3
"""T2c - separate a hand-eye translation error from a board-origin error.

T2/T2b found a ~18 mm constant offset between the FK/hand-eye model and the
markers, and both candidate owners (board.json origin, camera_in_tool
translation) fit it equally well from these poses. They are not equally
testable, though:

  * a hand-eye translation error moves the CAMERA, so it moves where the tip
    is predicted to appear in the image;
  * a board-origin error moves the coordinate frame, so the camera's predicted
    pixel for the tip does not change at all.

So: predict the tip-centre pixel from the model with and without the fitted
hand-eye shift, and compare against the measured tip pixel. Whichever model
lands the tip on the measured pixel owns the offset.

Read-only: reads manifests and images, prints numbers.
"""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))
import cv2  # noqa: E402
from chess_sim.board_target import Board  # noqa: E402
from tip_frame import TIP_IN_CHAIN  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
         "wrist_roll", "gripper")


def project(C, point_m):
    """Pixel of a base-frame point for a 4x4 camera pose in base coordinates.

    Convention mirrors Board.unproject: camera +z is the viewing direction and
    +y runs down the image, so u = cx + fx*x/z, v = cy + fy*y/z.
    """
    p_cam = C[:3, :3].T @ (np.asarray(point_m) - C[:3, 3])
    z = p_cam[2]
    if z <= 1e-6:
        return None
    cx, cy, fx, fy = 320.0, 240.0, 831.897, 833.014
    return np.array([cx + fx * p_cam[0] / z, cy + fy * p_cam[1] / z])


def main():
    calib = json.loads((ROOT / ".private/phase11d/calibration.json").read_text())
    board = Board(calib)
    tip_px = np.array([320.0, 250.0])

    # The hand-eye shift fitted in T2b (tool frame, mm), robust variant.
    shifts = {
        "none": np.zeros(3),
        "t2b_robust": np.array([14.9, 7.9, -6.7]),
    }

    dirs = sorted(p.parent for p in
                  (ROOT / ".private/real_piece_views").glob("*/manifest.json"))
    seen, rows = set(), []
    for d in dirs:
        manifest = json.loads((d / "manifest.json").read_text())
        label = manifest.get("label") or d.name
        for placement in manifest.get("placements", []):
            for view in placement.get("views", []):
                sha = view.get("sha256")
                if sha and sha in seen:
                    continue
                if sha:
                    seen.add(sha)
                meta = view.get("capture_meta") or {}
                joints = meta.get("state_joints_deg") or meta.get("joints_deg")
                if not joints:
                    continue
                name = f"{placement.get('piece')}_{placement.get('square')}_{label}_{view.get('view')}.jpg"
                path = d / name
                if not path.exists():
                    cands = sorted(d.glob(f"{placement.get('piece')}_{placement.get('square')}_{label}_{view.get('view')}*"))
                    if not cands:
                        continue
                    path = cands[0]
                raw = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if raw is None:
                    continue
                sol, dets = board.solve(board.rectify(raw))
                if sol is None or float(sol["rms_px"]) > 2.0 or len(dets) < 3:
                    continue
                rows.append({"dataset": d.name,
                             "ordered": [float(joints[n]) for n in NAMES]})

    print(f"views (rms<=2 px, >=3 markers) {len(rows)}")
    for tag, shift in shifts.items():
        ctc = np.asarray(calib["camera_in_tool"], float).copy()
        shift = np.asarray(shift, float) / 1000.0
        ctc[:3, 3] = ctc[:3, 3] + shift
        local = copy.deepcopy(calib)
        local["camera_in_tool"] = ctc.tolist()
        b = Board(local)
        errs = []
        for r in rows:
            cam = b.camera_from_joints(r["ordered"])          # base frame camera
            flange = cam @ np.linalg.inv(b.camera_in_tool)     # drop hand-eye
            tip = (flange @ TIP_IN_CHAIN)[:3, 3]
            uv = project(cam, tip)
            if uv is None:
                continue
            errs.append(uv - tip_px)
        if not errs:
            print(f"  {tag:<12} no projections")
            continue
        e = np.asarray(errs)
        n = np.linalg.norm(e, axis=1)
        print(f"  {tag:<12} tip-pixel err mean {n.mean():6.2f} px  median {np.median(n):6.2f}"
              f"  max {n.max():7.2f}   mean vec {e.mean(0).round(1)}")


if __name__ == "__main__":
    main()
