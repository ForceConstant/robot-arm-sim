"""Grid-based wrist calibration: validate the method in simulation first.

The single-marker method needs two markers per frame, which a 46 deg lens
almost never gives at wrist working distance.  The board's own 8x8 checker
grid supplies 49 corners per view instead, which conditions both the
intrinsics fit and every board pose.

This tool renders sim views, runs the identical live pipeline (chessboard
detection -> intrinsics -> per-view PnP -> hand-eye) and reports how close it
gets to the simulator's known camera mount.  Nothing here touches hardware.

    PYTHONPATH=. robot-arm-sim python tools/solve_hand_eye_grid.py --views 15
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import mujoco
import numpy as np

from chess_sim import fiducials
from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.product import layout_env_config
from scipy.spatial.transform import Rotation

from chess_sim.wrist_calibration import WristCalibration, _transform

SQUARE_M = 0.028          # printed reference board
GRID = (7, 7)             # inner corners of an 8x8 board


def flange_pose(calibration: WristCalibration, joints) -> tuple[np.ndarray, np.ndarray]:
    joints = np.asarray(joints, dtype=float)
    world = np.eye(4)
    for link in calibration.chain:
        world = world @ _transform(link["position"], link["rotation"])
        if "joint_index" not in link:
            continue
        angle = joints[link["joint_index"]]
        anchor = np.asarray(link["anchor"])
        rotation = Rotation.from_rotvec(
            np.asarray(link["axis"]) * (angle - link["reference"])).as_matrix()
        world = world @ _transform(anchor - rotation @ anchor, rotation)
    return world[:3, :3].copy(), world[:3, 3].copy()


def grid_points() -> np.ndarray:
    cols, rows = GRID
    xs, ys = np.meshgrid(np.arange(cols) * SQUARE_M, np.arange(rows) * SQUARE_M)
    return np.stack([xs.ravel(), ys.ravel(), np.zeros(cols * rows)], 1)


def render_views(env, calibration, count: int, seed: int):
    """Random wrist poses that keep the whole grid in frame."""
    rng = np.random.default_rng(seed)
    points = grid_points()
    out = []
    attempts = 0
    while len(out) < count and attempts < count * 60:
        attempts += 1
        joints = np.radians([rng.uniform(-45, 45), rng.uniform(5, 45),
                             rng.uniform(-70, -20), rng.uniform(45, 85),
                             rng.uniform(-40, 40), 1.0])
        env.data.qpos[env._joint_qpos] = joints
        mujoco.mj_forward(env.model, env.data)
        image = env.render(Camera.WRIST).copy()
        found, corners = cv2.findChessboardCorners(
            cv2.cvtColor(image, cv2.COLOR_RGB2GRAY), GRID,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            continue
        out.append((joints.copy(), image, corners.reshape(-1, 2).astype(np.float64)))
    return out, attempts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", default=".private/phase11d/calibration.json")
    parser.add_argument("--views", type=int, default=15)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--out", default="reports/hand_eye_grid_sim.json")
    args = parser.parse_args()

    calibration = WristCalibration.load(args.calibration)
    truth = np.asarray(calibration.camera_in_tool, dtype=float)
    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset("8/8/8/8/8/8/8/8")
        views, attempts = render_views(env, calibration, args.views, args.seed)
        print(f"accepted {len(views)} of {attempts} attempts")
        if len(views) < 6:
            raise SystemExit("not enough full-board views; widen the pose sampler")
        height, width = views[0][1].shape[:2]
        points = grid_points()

        obj = [np.ascontiguousarray(points, np.float32) for _ in views]
        img = [np.ascontiguousarray(v[2], np.float32) for v in views]
        flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
        guess = np.array([[width * 1.6, 0, width / 2],
                          [0, width * 1.6, height / 2], [0, 0, 1]], np.float64)
        rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, (width, height), guess,
                                                np.zeros((1, 5)), flags=flags)
        print(f"intrinsics rms {rms:.3f} px  fx {K[0,0]:.1f} fy {K[1,1]:.1f} "
              f"cx {K[0,2]:.1f} cy {K[1,2]:.1f}")

        Rg, tg, Rt, tt = [], [], [], []
        for joints, _, corners in views:
            ok, rvec, tvec = cv2.solvePnP(
                np.ascontiguousarray(points, np.float64),
                np.ascontiguousarray(corners, np.float64), K, dist,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            rotation_g, translation_g = flange_pose(calibration, joints)
            Rg.append(rotation_g)
            tg.append(translation_g.reshape(3, 1))
            Rt.append(rotation)
            tt.append(np.asarray(tvec).reshape(3, 1))
        if len(Rg) < 5:
            raise SystemExit("too few posed views")

        Rx, tx = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=cv2.CALIB_HAND_EYE_PARK)
        angle = math.degrees(math.acos(max(-1.0, min(1.0,
            (np.trace(Rx.T @ truth[:3, :3]) - 1.0) / 2.0))))
        tmm = float(np.linalg.norm(tx.ravel() - truth[:3, 3]) * 1000)
        print(f"recovered mount: {angle:.2f} deg / {tmm:.1f} mm from truth "
              f"(views {len(Rg)})")

        # Consistency check: the board is static, so rebuilt poses must agree.
        boards = []
        for i in range(len(Rg)):
            T = np.eye(4)
            T[:3, :3] = Rg[i]
            T[:3, 3] = tg[i].ravel()
            X = np.eye(4)
            X[:3, :3] = Rx
            X[:3, 3] = tx.ravel()
            C = np.eye(4)
            C[:3, :3] = Rt[i]
            C[:3, 3] = tt[i].ravel()
            boards.append(T @ X @ C)
        pos = np.array([b[:3, 3] for b in boards])
        spread = float(np.linalg.norm(pos - pos.mean(0), axis=1).max() * 1000)
        print(f"board spread {spread:.1f} mm")

        payload = {
            "schema": "hand-eye-grid-sim/1",
            "views": len(Rg),
            "intrinsics": {"fx": float(K[0, 0]), "fy": float(K[1, 1]),
                           "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                           "rms_px": round(float(rms), 4)},
            "mount_error": {"rotation_deg": round(angle, 4), "translation_mm": round(tmm, 3)},
            "board_spread_mm": round(spread, 3),
            "passed": angle <= 1.0 and tmm <= 3.0,
        }
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(payload, indent=2) + "\n")
        print("passed" if payload["passed"] else "FAILED", "->", args.out)
        return 0 if payload["passed"] else 1
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
