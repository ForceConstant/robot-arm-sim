"""Export the wrist hand-eye chain for the current layout and tool.

The saved `WristCalibration` is installation geometry: every link transform from
the world to the wrist camera, plus intrinsics.  It has to be regenerated
whenever the mount, riser or installed gripper changes, otherwise the recovered
board pose carries a fixed bias and the marker solver's residual gate starts
rejecting good frames.

This writes `robot-arm-wrist-calibration/1` from the live scene and proves it by
comparing forward kinematics against MuJoCo's own camera pose at several joint
configurations.  No board truth is used.

    MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
    PYTHONPATH=. robot-arm-sim python tools/export_wrist_calibration.py \
      --out .private/phase11d/calibration.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from chess_sim import gripper
from chess_sim.config import JOINTS, Camera
from chess_sim.env import ChessSimEnv
from chess_sim.product import current_layout, layout_env_config
from chess_sim.wrist_calibration import WristCalibration


def build_chain(model, camera_id: int) -> list[dict]:
    chain: list[dict] = []
    body_id = int(model.cam_bodyid[camera_id])
    while body_id:
        body = model.body(body_id)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, body.quat)
        link = {"name": body.name,
                "position": body.pos.tolist(),
                "rotation": rotation.reshape(3, 3).tolist()}
        for joint in range(int(model.body_jntadr[body_id]),
                           int(model.body_jntadr[body_id]) + int(model.body_jntnum[body_id])):
            name = model.joint(joint).name.split(":")[-1]
            if name not in list(map(str, JOINTS)):
                continue
            link.update(joint_index=list(map(str, JOINTS)).index(name),
                        anchor=model.jnt_pos[joint].tolist(),
                        axis=model.jnt_axis[joint].tolist(),
                        reference=float(model.qpos0[model.jnt_qposadr[joint]]),
                        limits=model.jnt_range[joint].tolist())
        chain.insert(0, link)
        body_id = int(model.body_parentid[body_id])
    return chain


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=".private/phase11d/calibration.json")
    parser.add_argument("--samples", type=int, default=5)
    args = parser.parse_args()

    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset("8/8/8/8/8/8/8/8")
        model, data = env.model, env.data
        camera_id = model.camera(str(Camera.WRIST)).id
        chain = build_chain(model, camera_id)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, model.cam_quat[camera_id])
        camera_in_tool = np.eye(4)
        camera_in_tool[:3, :3] = rotation.reshape(3, 3)
        camera_in_tool[:3, 3] = model.cam_pos[camera_id]
        fy = 480 / (2 * math.tan(math.radians(model.cam_fovy[camera_id]) / 2))
        layout = current_layout()
        profile = getattr(gripper, "PROFILE", {}) or {}
        tool = profile.get("model_file", "stock-jaw")
        identifier = (f"sim-{layout['version']}-{tool}-"
                      f"{layout['mount']['white_arm_base_frame_mm'][2]:.1f}mm-2026-11-15")
        payload = {
            "schema": "robot-arm-wrist-calibration/1",
            "width": 640, "height": 480,
            "intrinsics": [fy, fy, 320.0, 240.0],
            "distortion": [0.0] * 5,
            "chain": chain,
            "camera_in_tool": camera_in_tool.tolist(),
            "calibration_id": identifier,
        }
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, indent=2) + "\n")

        calibration = WristCalibration.load(out)
        rng = np.random.default_rng(7)
        worst = 0.0
        for _ in range(args.samples):
            joints = np.array([rng.uniform(lo, hi) for lo, hi in
                               [(l[0], l[1]) for l in
                                [link["limits"] for link in chain if "joint_index" in link]]])
            joints = np.r_[joints[:5], 0.0]
            data.qpos[env._joint_qpos] = joints
            mujoco.mj_forward(model, data)
            camera = calibration.camera(joints)
            position_error = float(np.linalg.norm(
                np.asarray(camera.camera_position) - data.cam_xpos[camera_id]))
            rotation_error = float(np.abs(
                np.asarray(camera.camera_rotation).reshape(3, 3)
                - data.cam_xmat[camera_id].reshape(3, 3)).max())
            worst = max(worst, position_error)
            if position_error > 1e-9 or rotation_error > 1e-9:
                raise SystemExit(
                    f"calibration does not reproduce the scene: "
                    f"{position_error*1000:.4f} mm, {rotation_error:.2e}")
        print(f"calibration {identifier}: {len(chain)} links, "
              f"max position error {worst*1000:.6f} mm -> {out}")
        return 0
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
