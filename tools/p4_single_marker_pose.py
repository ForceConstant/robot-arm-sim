#!/usr/bin/env python3
"""Read-only single-marker audit when the view is too tight for multi-marker PnP.

For every detected ChArUco id it solves PnP from that one marker's declared
corners (camera pose in the *print* frame), and separately unprojects the
marker centroid with the FK camera pose in the *board.json* frame.  Uses the
codebase camera convention from Board.solve_joints (camera looks along -z,
v flipped), not cv2's projectPoints.

    PYTHONPATH=. .venv/bin/python tools/p4_single_marker_pose.py --frames 5
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.board_target import TargetDetector, JOINTS  # noqa: E402
from chess_sim.charuco_face import CharucoLayout, square_of  # noqa: E402
from p4_board_frame_check import stream_frames  # noqa: E402

D = np.diag([1.0, -1.0, -1.0])


def square_name(x_mm: float, y_mm: float) -> str:
    col = int(math.floor((x_mm + 112.0) / 28.0))
    rank = int(math.floor((y_mm + 112.0) / 28.0))
    if not (0 <= col < 8 and 0 <= rank < 8):
        return f"off-board(col={col},rank={rank})"
    return f"{chr(97 + col)}{rank + 1}"


def unproject(K, R_bc, C_bc, uv, z_mm=0.0):
    """Pixel -> board mm on the plane z=z_mm, codebase camera convention."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d_code = np.array([(uv[0] - cx) / fx, -(uv[1] - cy) / fy, -1.0])
    d_world = R_bc @ d_code
    if abs(d_world[2]) < 1e-12:
        return None
    t = (z_mm / 1000.0 - C_bc[2]) / d_world[2]
    return (C_bc + t * d_world)[:2] * 1000.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--config", default="live/live_config.json")
    ap.add_argument("--image", default="")
    args = ap.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    board = json.loads((ROOT / "live/board.json").read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board
    layout = CharucoLayout(square_mm=28.0)
    K = solver.K

    state = json.loads(urllib.request.urlopen(args.url + "/state", timeout=10).read())
    joints = [float(state["bridge"]["joints"][name]) for name in JOINTS]
    print(f"bridge mode={state['bridge']['mode']} armed={state['bridge']['armed']} "
          f"fault={state['bridge']['fault']} open={state['bridge']['follower_open']} "
          f"age={state['bridge']['joints_age_s']:.0f}s")
    print(f"joints {[round(v, 2) for v in joints]}")

    yaw = math.radians(board["yaw_deg"])
    frame_in_arm = np.array([
        [math.cos(yaw), -math.sin(yaw), 0, board["x_mm"] / 1000],
        [math.sin(yaw), math.cos(yaw), 0, board["y_mm"] / 1000],
        [0, 0, 1, board["z_mm"] / 1000], [0, 0, 0, 1]])
    T_arm_cam = solver.camera_from_joints(joints)
    T_board_cam = np.linalg.inv(frame_in_arm) @ T_arm_cam
    fk_cam = T_board_cam[:3, 3] * 1000.0
    R_bc = T_board_cam[:3, :3]
    print(f"FK camera in board.json frame mm {np.round(fk_cam, 1)}")

    if args.image:
        raw = cv2.imread(str(ROOT / args.image), cv2.IMREAD_COLOR)
        frames = [raw]
        pre = True
    else:
        frames = stream_frames(args.url, args.frames, 10.0)
        pre = False
    print(f"frames {len(frames)}")

    for index, portrait in enumerate(frames):
        land = portrait if pre else cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rect = solver.rectify(land)
        detections = fiducials.detect_markers(cv2.cvtColor(rect, cv2.COLOR_BGR2RGB))
        print(f"\nframe {index}: {len(detections)} charuco markers "
              f"{[d.marker_id for d in detections]}")
        for det in detections:
            mid = det.marker_id
            sq = square_of(mid)
            quad2 = np.asarray(layout.corners_mm(mid), float).reshape(4, 2)
            obj = np.hstack([quad2, np.zeros((4, 1))]).reshape(4, 1, 3)
            img = det.corners.astype(np.float64).reshape(4, 1, 2)
            count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
            sols = []
            for k in range(count):
                R_cv, _ = cv2.Rodrigues(rvecs[k])
                C = (-R_cv.T @ tvecs[k]).ravel()
                proj, _ = cv2.projectPoints(obj, rvecs[k], tvecs[k], K, None)
                rms = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).mean()))
                sols.append((C, rms))
            decl = quad2.mean(axis=0)
            decl_name = f"{chr(97 + sq[0])}{sq[1] + 1}"
            uv = det.corners.mean(axis=0)
            landed = unproject(K, R_bc, fk_cam / 1000.0, uv)
            landed_name = square_name(*landed) if landed is not None else "n/a"
            dist = float(np.linalg.norm(landed - decl)) if landed is not None else float("nan")
            print(f"  id {mid} declared {decl_name} centre {np.round(decl,1)} mm")
            for i, (C, rms) in enumerate(sols):
                dlt = C - fk_cam
                print(f"    IPPE {i}: camera in print frame {np.round(C,1)} mm "
                      f"rms {rms:.2f} px  |delta vs FK| {np.linalg.norm(dlt):.1f} mm")
            # half-turn prediction: FK position rotated 180 deg in the print frame
            flip = np.array([-fk_cam[0], -fk_cam[1], fk_cam[2]])
            print(f"    half-turn prediction {np.round(flip,1)} mm "
                  f"(distance from IPPE0 {np.linalg.norm(sols[0][0]-flip):.1f} mm, "
                  f"from FK {np.linalg.norm(sols[0][0]-fk_cam):.1f} mm)")
            print(f"    centroid px {np.round(uv,1)} -> FK-unprojected {np.round(landed,1)} mm "
                  f"= {landed_name} ({dist:.1f} mm from declared {decl_name})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
