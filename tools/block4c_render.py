#!/usr/bin/env python3
"""Block 4c: render the sim wrist view at real cal joint vectors.

Proves the scene camera now sits where the measurement says: FK(q) @
camera_in_tool must equal MuJoCo's own ``cam_xpos``/``cam_xmat`` at every cal
pose, and the rendered frame is saved beside the real capture for a visual
check.

    MUJOCO_GL=egl PYTHONPATH=. robot-arm-sim python tools/block4c_render.py \
        --out reports/block4c_overlays
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.product import layout_env_config
from chess_sim.wrist_calibration import WristCalibration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handeye_bundle import chain_fk  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--mount", default="reports/block4c_mount.json")
    parser.add_argument("--out", default="reports/block4c_overlays")
    parser.add_argument("--names", default="20261117T120833103_cal,20261117T120940124_cal")
    args = parser.parse_args()

    camera_gl = np.asarray(json.loads(Path(args.mount).read_text())["camera_in_tool_gl"], float)
    calibration = WristCalibration.load(".private/phase11d/calibration.json")

    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset()
        model, data = env.model, env.data
        camera_id = model.camera(str(Camera.WRIST)).id
        renderer = mujoco.Renderer(model, 480, 640)
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        worst = 0.0
        for name in args.names.split(","):
            record = json.loads((Path(args.captures) / f"{name}.json").read_text())
            joints_deg = [float(record["joints_deg"][n]) for n in JOINTS]
            data.qpos[env._joint_qpos] = np.radians(joints_deg)
            mujoco.mj_forward(model, data)
            predicted = chain_fk(calibration, joints_deg) @ camera_gl
            position = float(np.linalg.norm(predicted[:3, 3] - data.cam_xpos[camera_id]))
            rotation = float(np.abs(predicted[:3, :3] - data.cam_xmat[camera_id].reshape(3, 3)).max())
            worst = max(worst, position)
            print(f"{name}: FK vs MuJoCo {position*1000:.4f} mm, rotation {rotation:.2e}")
            renderer.update_scene(data, camera=str(Camera.WRIST))
            frame = renderer.render()
            cv2.imwrite(str(out / f"sim_{name}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            real = cv2.imread(str(Path(args.captures) / f"{name}.jpg"))
            if real is not None:
                cv2.imwrite(str(out / f"real_{name}.png"), real)
        print(f"worst position error {worst*1000:.4f} mm")
        renderer.close()
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
