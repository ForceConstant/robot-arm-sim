#!/usr/bin/env python3
"""Collect wrist-camera views in the simulator by commanding look-at targets.

Views are requested as "aim the camera at this board coordinate from this
standoff and this tilt" -- the same vocabulary the live app exposes -- and the
sim is driven to those poses through the calibrated chain, so the collected
data exercises the real command path rather than arbitrary joint sweeps.

Each record carries the measured joints, the rendered frame, and the simulator's
*true* camera pose, which is what lets a later hand-eye solve be scored against
ground truth instead of against itself.

    robot-arm-sim python tools/collect_wrist_views.py --out sim/captures_wrist
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import chess
import cv2
import mujoco
import numpy as np

from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
from chess_sim.env import ChessSimEnv
from chess_sim.wrist_calibration import WristCalibration
from kinematics import ArmKinematics, JOINTS


def _pose(matrix) -> tuple[np.ndarray, np.ndarray]:
    return matrix[:3, 3].copy(), matrix[:3, :3].copy()


def _transform(position, rotation) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation, dtype=float)
    out[:3, 3] = np.asarray(position, dtype=float)
    return out


def _body_of_camera(model, camera_name: str) -> int:
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        raise SystemExit(f"camera {camera_name!r} not found")
    return int(model.cam_bodyid[cam_id]), cam_id


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="sim/captures_wrist")
    parser.add_argument("--calibration", default=".private/phase11d/calibration.json")
    parser.add_argument("--camera", default="wrist")
    parser.add_argument("--count", type=int, default=18)
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    board_cfg = BoardConfig(face="charuco", yaw=math.pi)
    env = ChessSimEnv(EnvConfig(board=board_cfg,
                                appearance=AppearanceConfig(board_markers=False)))
    env.reset(position=chess.Board(None))          # empty board: no piece occlusion
    model, data = env.model, env.data
    mujoco.mj_forward(model, data)

    tool_body, cam_id = _body_of_camera(model, args.camera)
    tool = _transform(data.body(tool_body).xpos, data.body(tool_body).xmat.reshape(3, 3))
    camera = _transform(data.cam(cam_id).xpos, data.cam(cam_id).xmat.reshape(3, 3))
    x_sim = np.linalg.inv(tool) @ camera            # camera in tool, from the model

    cal = WristCalibration.load(args.calibration)
    limits = {name: (-180.0, 180.0) for name in JOINTS}
    limits.update({"shoulder_pan": (-109, 121), "shoulder_lift": (-107, 109),
                   "elbow_flex": (-97, 95), "wrist_flex": (-105, 99),
                   "wrist_roll": (-83, 61), "gripper": (0, 100)})
    kinematics = ArmKinematics(cal, limits)

    # Chain frame -> world offset, so board coordinates can be expressed in the
    # same frame the IK works in.  Measured once from the solved pose.
    joints_now = np.array([float(data.qpos[env._joint_qpos[i]]) for i in range(6)])
    chain_tool = kinematics.fk(joints_now)
    chain_to_world = tool @ np.linalg.inv(chain_tool)

    board_center = _transform([board_cfg.origin[0], board_cfg.origin[1],
                               board_cfg.table_top + board_cfg.thickness],
                              np.array([[math.cos(board_cfg.yaw), -math.sin(board_cfg.yaw), 0.0],
                                        [math.sin(board_cfg.yaw), math.cos(board_cfg.yaw), 0.0],
                                        [0.0, 0.0, 1.0]]))
    # The calibrated chain already reports in the simulator's world frame (its
    # base link sits at the mount), so the board goes in unchanged.  Applying a
    # chain-to-world offset here would shift every target by the difference
    # between the chain's tool frame and the camera's parent body -- 124 mm and
    # 39 degrees, which showed up as a constant ~25 degree approach error.
    kinematics.board = board_center

    x_chain = np.linalg.inv(chain_tool) @ camera    # camera in chain tool frame
    # Aim with the simulator's own mount.  Using the stored calibration here
    # would shift every target by whatever error that calibration carries --
    # which is exactly the quantity this exercise is meant to measure.
    kinematics.camera_in_tool = x_chain

    records = []
    # Cartesian sweep over the board: the simulator's own IK places the tool
    # point above each square, so commands use the same position vocabulary as
    # the app and no frame conversion is involved.
    points = [(0, 0), (56, 56), (-56, 56), (56, -56), (-56, -56),
              (0, 56), (0, -56), (56, 0), (-56, 0), (28, 28), (-28, -28),
              (84, 0), (-84, 0)]
    heights = [0.10, 0.12, 0.14, 0.16]
    yaw = board_cfg.yaw
    rot = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                    [math.sin(yaw), math.cos(yaw), 0.0],
                    [0.0, 0.0, 1.0]])
    attempts = 0
    for index in range(args.count):
        x_mm, y_mm = points[index % len(points)]
        height = heights[(index // len(points)) % len(heights)]
        attempts += 1
        local = np.array([x_mm / 1000.0, y_mm / 1000.0, 0.0])
        target = np.array([board_center[0, 3], board_center[1, 3],
                           board_cfg.top]) + rot @ local
        target = target + np.array([0.0, 0.0, height])
        result = env.ik.solve(data, target)
        if result.position_error > 0.006 or result.tilt > 0.10:
            print(f"  skip view {index}: ik {result.position_error*1000:.1f} mm "
                  f"/ {math.degrees(result.tilt):.1f} deg")
            continue
        data.qpos[env._joint_qpos[:5]] = result.q
        mujoco.mj_forward(model, data)
        joints = np.degrees(np.array([float(data.qpos[env._joint_qpos[i]])
                                      for i in range(6)]))
        image = env.render(args.camera)
        name = f"sim_{index:02d}"
        cv2.imwrite(str(out / f"{name}.jpg"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        true_cam = _transform(data.cam(cam_id).xpos, data.cam(cam_id).xmat.reshape(3, 3))
        records.append({
            "name": name,
            "label": "sim_cartesian",
            "joints_deg": {n: round(float(v), 3) for n, v in zip(JOINTS, joints)},
            "target_mm": [round(float(v) * 1000, 1) for v in target],
            "true_camera_position": [round(float(v), 6) for v in true_cam[:3, 3]],
            "true_camera_rotation": [round(float(v), 6) for v in true_cam[:3, :3].ravel()],
        })
        if len(records) >= args.count:
            break

    np.save(out / "sim_truth.npy", {"x_sim": x_sim, "chain_to_world": chain_to_world,
                                    "board_center": board_center,
                                    "camera_in_chain_tool": x_chain}, allow_pickle=True)
    (out / "records.json").write_text(json.dumps(records, indent=2) + "\n")
    print(f"collected {len(records)} views from {attempts} attempts -> {out}")
    env.close()


if __name__ == "__main__":
    main()
