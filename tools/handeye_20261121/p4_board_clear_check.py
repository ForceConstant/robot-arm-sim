#!/usr/bin/env python3
"""Read-only: settle exposure on a high-exposure grid, then measure square occupancy.

Runs on the BC-250.  No file writes, no saved camera controls, no motion.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.board_target import TargetDetector  # noqa: E402
from p4_auto_exposure import (api, base_metrics, grab, roi_edges,  # noqa: E402
                              solve_any, square_quad)
from p4_vision_receipt import otsu_blob  # noqa: E402
from p4_board_frame_check import JOINTS  # noqa: E402

GRID = [(400, 64), (500, 64), (600, 64), (705, 64), (850, 64)]
SQUARES = ["e5", "e6", "e4", "d5", "f5", "d4", "f4", "a8"]


def fk_cam(solver, joints):
    board = json.loads((ROOT / "live/board.json").read_text())
    yaw = math.radians(board["yaw_deg"])
    frame_in_arm = np.array([
        [math.cos(yaw), -math.sin(yaw), 0, board["x_mm"] / 1000],
        [math.sin(yaw), math.cos(yaw), 0, board["y_mm"] / 1000],
        [0, 0, 1, board["z_mm"] / 1000], [0, 0, 0, 1]])
    return (np.linalg.inv(frame_in_arm)
            @ solver.camera_from_joints(joints))[:3, 3] * 1000.0


def main() -> int:
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    det = TargetDetector(str(ROOT / cfg["calibration"]),
                         str(ROOT / "chess_sim/assets/scene/generated/board_charuco.png"))
    solver = det.board

    rows = []
    for e, g in GRID:
        api("/camera/controls", {"controls": {"exposure_time_absolute": e,
                                              "gain": g}, "save": False})
        time.sleep(0.8)
        rects = grab(solver, 3)
        if not rects:
            print(f"{e}/{g}: no frames")
            continue
        bm = [base_metrics(r) for r in rects]
        agg = {k: float(np.mean([b[k] for b in bm])) for k in bm[0]}
        counts, rmss, multi = [], [], 0
        for r in rects:
            sol, dets = solve_any(solver, r)
            counts.append(len(dets))
            if sol is not None:
                rmss.append(round(float(sol["rms_px"]), 2))
                if not sol.get("single_marker"):
                    multi += 1
        rows.append({"exp": e, "gain": g, "mean": agg["mean"], "clip": agg["clip"],
                     "dark": agg["dark"], "markers": counts, "rms": rmss, "multi": multi})
        print(f"{e}/{g}: mean {agg['mean']:.1f} clip {agg['clip']:.2f}% "
              f"dark {agg['dark']:.2f}% markers {counts} rms {rmss} multi {multi}/3")

    pool = [r for r in rows if r["clip"] <= 1.5]
    if not pool:
        print("no exposure met the clipping budget")
        return 1
    pick = max(pool, key=lambda r: (r["multi"], max(r["markers"] or [0]), r["mean"]))
    print(f"pick {pick['exp']}/{pick['gain']}")
    api("/camera/controls", {"controls": {"exposure_time_absolute": pick["exp"],
                                          "gain": pick["gain"]}, "save": False})
    time.sleep(0.8)

    rects = grab(solver, 5)
    state = api("/state")
    joints = [float(state["bridge"]["joints"][n]) for n in JOINTS]
    fk = fk_cam(solver, joints)
    print(f"roll {state['bridge']['joints']['wrist_roll']} joints {joints}")
    print(f"fk_cam_mm {np.round(fk, 1).tolist()}")

    sols = []
    for r in rects:
        sol, dets = solve_any(solver, r)
        if sol is not None and not sol.get("single_marker"):
            sols.append(sol)
    if not sols:
        print("no multi-marker solution in 5 frames")
        return 2
    sol = sols[0]
    cam = np.asarray(sol["camera_in_board_mm"], float)
    print(f"ids {sol.get('ids')} rms {sol['rms_px']:.2f} "
          f"cam {np.round(cam, 1).tolist()} "
          f"disagreement_mm {np.linalg.norm(cam - fk):.1f}")

    gray = cv2.cvtColor(rects[0], cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    for sq in SQUARES:
        try:
            quad, centre = square_quad(solver, sol, sq)
        except Exception as exc:  # noqa: BLE001
            print(f"{sq}: quad failed {exc}")
            continue
        if (not np.all(np.isfinite(quad)) or quad[:, 0].min() < 0
                or quad[:, 1].min() < 0 or quad[:, 0].max() > w - 1
                or quad[:, 1].max() > h - 1):
            print(f"{sq}: outside frame {np.round(quad, 0).tolist()}")
            continue
        e, _, _ = roi_edges(gray, quad)
        blob = otsu_blob(gray, quad)
        area_px = float(abs(cv2.contourArea(quad.astype(np.float32))))
        fill = blob.get("blob_area_px", 0) / max(1.0, area_px)
        print(f"{sq}: centre {np.round(centre, 1).tolist()} "
              f"edge_density {e['edge_density']:.2f} contrast {e['roi_contrast']:.2f} "
              f"blob_px {blob.get('blob_area_px')} "
              f"blob_fill_of_square {fill:.3f} "
              f"blob_wh {(blob.get('blob_w'), blob.get('blob_h'))} "
              f"square_px {area_px:.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
