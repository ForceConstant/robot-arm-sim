#!/usr/bin/env python3
"""Read-only: print FK / half-turn projections of the declared marker centres."""
from __future__ import annotations

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

from chess_sim.board_target import TargetDetector, JOINTS  # noqa: E402
from chess_sim.charuco_face import CharucoLayout, square_of  # noqa: E402

LAYOUT = CharucoLayout(square_mm=28.0)
config = json.loads((ROOT / "live/live_config.json").read_text())
board = json.loads((ROOT / "live/board.json").read_text())
detector = TargetDetector(str(ROOT / config["calibration"]),
                          str(ROOT / "chess_sim/assets/scene/generated/board_charuco.png"))
solver = detector.board
K = solver.K
state = json.loads(urllib.request.urlopen("http://127.0.0.1:8080/state", timeout=10).read())
joints = [float(state["bridge"]["joints"][n]) for n in JOINTS]
T_arm_cam = solver.camera_from_joints(joints)
print("T_arm_cam\n", np.round(T_arm_cam, 4))


def fa(yaw_extra=0.0):
    yaw = math.radians(board["yaw_deg"] + yaw_extra)
    return np.array([[math.cos(yaw), -math.sin(yaw), 0, board["x_mm"] / 1000],
                     [math.sin(yaw), math.cos(yaw), 0, board["y_mm"] / 1000],
                     [0, 0, 1, board["z_mm"] / 1000], [0, 0, 0, 1]])


def project_code(K, R_bc, C_bc, xy_mm):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    c = (np.array([xy_mm[0] / 1000.0, xy_mm[1] / 1000.0, 0.0]) - C_bc) @ R_bc
    depth = -c[2]
    if depth <= 1e-6:
        return None
    return cx + fx * c[0] / depth, cy - fy * c[1] / depth


for name, extra in (("board.json", 0.0), ("+180", 180.0)):
    T = np.linalg.inv(fa(extra)) @ T_arm_cam
    R, C = T[:3, :3], T[:3, 3]
    print(f"\n=== {name}: camera in board mm {np.round(C*1000,1)}")
    inside = 0
    for mid in range(5, 37):
        c = np.asarray(LAYOUT.corners_mm(mid), float).mean(axis=0)
        uv = project_code(K, R, C, c)
        if uv is None:
            print(f"  id{mid:2d} behind camera")
            continue
        u, v = uv
        ok = 0 <= u < 640 and 0 <= v < 480
        inside += ok
        sq = square_of(mid)
        print(f"  id{mid:2d} {chr(97+sq[0])}{sq[1]+1} mm({c[0]:6.1f},{c[1]:6.1f}) "
              f"-> px({u:7.1f},{v:7.1f}) {'IN' if ok else 'out'}")
    print(f"  inside {inside}/32")
