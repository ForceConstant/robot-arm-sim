#!/usr/bin/env python3
"""Read-only: project the declared marker layout three ways onto a live frame.

Uses the codebase camera convention from `Board.solve_joints` (camera looks
along -z, v is flipped), not cv2's projectPoints, because the calibrated
`camera_in_tool` pose is expressed in that convention.

Green = declared print frame via a single-marker PnP (anchor marker).
Red   = FK pose through live/board.json (yaw as stored).
Blue  = FK pose with board.json yaw + 180 deg (the half-turn hypothesis).

Whichever labels sit on the printed markers they name is the frame the print is
actually in.  Nothing moves; only reports/ is written.

    PYTHONPATH=. .venv/bin/python tools/p4_layout_overlay.py --frames 10
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
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from p4_board_frame_check import stream_frames  # noqa: E402

LAYOUT = CharucoLayout(square_mm=28.0)


def board_frame(board: dict, extra_yaw_deg: float = 0.0) -> np.ndarray:
    yaw = math.radians(board["yaw_deg"] + extra_yaw_deg)
    return np.array([
        [math.cos(yaw), -math.sin(yaw), 0, board["x_mm"] / 1000],
        [math.sin(yaw), math.cos(yaw), 0, board["y_mm"] / 1000],
        [0, 0, 1, board["z_mm"] / 1000], [0, 0, 0, 1]])


def project_code(K, R_bc, C_bc, xy_mm):
    """Board mm -> pixels in the codebase camera convention (see solve_joints)."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    c = (np.array([xy_mm[0] / 1000.0, xy_mm[1] / 1000.0, 0.0]) - C_bc) @ R_bc
    depth = -c[2]
    if depth <= 1e-6:
        return None
    return cx + fx * c[0] / depth, cy - fy * c[1] / depth


def draw(img, K, R_bc, C_bc, colour, tag):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    for mid in range(5, 37):
        c = np.asarray(LAYOUT.corners_mm(mid), float).mean(axis=0)
        uv = project_code(K, R_bc, C_bc, c)
        if uv is None:
            continue
        u, v = uv
        if not (0 <= u < img.shape[1] and 0 <= v < img.shape[0]):
            continue
        cv2.circle(img, (int(round(u)), int(round(v))), 3, colour, -1)
        cv2.putText(img, f"{tag}{mid}", (int(round(u)) + 4, int(round(v)) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, colour, 1, cv2.LINE_AA)
    return img


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--config", default="live/live_config.json")
    ap.add_argument("--anchor", type=int, default=26)
    ap.add_argument("--image", default="",
                    help="analyse a saved portrait frame instead of the stream")
    args = ap.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    board = json.loads((ROOT / "live/board.json").read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board
    K = solver.K
    state = json.loads(urllib.request.urlopen(args.url + "/state", timeout=10).read())
    joints = [float(state["bridge"]["joints"][name]) for name in JOINTS]
    T_arm_cam = solver.camera_from_joints(joints)

    if args.image:
        raw = cv2.imread(str(ROOT / args.image), cv2.IMREAD_COLOR)
        frames = [raw]  # already landscape (p4_capture_frame saves rotated)
        pre_rotated = True
        print(f"image {args.image} shape {raw.shape}")
    else:
        frames = stream_frames(args.url, args.frames, args.seconds)
        pre_rotated = False
    print(f"frames {len(frames)}")
    best, anchor_pose = None, None
    for portrait in frames:
        land = portrait if pre_rotated else cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rect = solver.rectify(land)
        dets = fiducials.detect_markers(cv2.cvtColor(rect, cv2.COLOR_BGR2RGB))
        for d in dets:
            if d.marker_id == args.anchor:
                best, det = rect, d
    if best is None:
        print(f"anchor id {args.anchor} not detected in {len(frames)} frames")
        best = solver.rectify(frames[0] if pre_rotated else
                              cv2.rotate(frames[0], cv2.ROTATE_90_COUNTERCLOCKWISE))
    else:
        quad2 = np.asarray(LAYOUT.corners_mm(args.anchor), float).reshape(4, 2)
        obj = np.hstack([quad2, np.zeros((4, 1))]).reshape(4, 1, 3)
        img_pts = det.corners.astype(np.float64).reshape(4, 1, 2)
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img_pts, K, None,
                                                     flags=cv2.SOLVEPNP_IPPE)
        sols = []
        for k in range(count):
            R_cv, _ = cv2.Rodrigues(rvecs[k])
            # object points are millimetres, so tvec is millimetres
            C = (-R_cv.T @ tvecs[k]).ravel()
            sols.append((C, R_cv.T))
        for i, (C, R) in enumerate(sols):
            print(f"anchor PnP {i}: camera in print frame {np.round(C,1)} mm")
        above = [s for s in sols if s[0][2] > 0]
        anchor_pose = (above or sols)[0]
        print(f"chosen anchor camera {np.round(anchor_pose[0],1)} mm")

    T_board_cam = np.linalg.inv(board_frame(board)) @ T_arm_cam
    T_flip_cam = np.linalg.inv(board_frame(board, 180.0)) @ T_arm_cam
    print(f"FK camera in board.json frame {np.round(T_board_cam[:3,3]*1000,1)} mm")

    out = best.copy()
    if anchor_pose is not None:
        draw(out, K, anchor_pose[1], anchor_pose[0] / 1000.0, (0, 200, 0), "P")
    draw(out, K, T_board_cam[:3, :3], T_board_cam[:3, 3], (0, 0, 255), "B")
    draw(out, K, T_flip_cam[:3, :3], T_flip_cam[:3, 3], (255, 0, 0), "F")
    path = ROOT / "reports/p4_layout_overlay.png"
    cv2.imwrite(str(path), out)
    print(f"saved {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
