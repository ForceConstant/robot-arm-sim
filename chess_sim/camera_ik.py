"""Solve arm joints for a desired *camera* pose, using the simulator's own FK.

Why not the calibrated chain: its final frame sits ~39 degrees off the real
jaw/camera axis, so targets expressed through it land in position but point the
camera elsewhere -- on the bench that showed up as the camera staring at the sky
while position residuals read 2-4 mm. Position residuals cannot confirm a pose
move; the aim has to be part of the target.

Here the camera pose comes from MuJoCo directly, so "put the camera here looking
there" is solved in the frame the verification also measures, and the two cannot
disagree.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ARM_JOINTS = 5


def rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Rotation vector taking ``current`` orientation onto ``target``."""
    delta = np.asarray(target) @ np.asarray(current).T
    return Rotation.from_matrix(delta).as_rotvec()


class CameraIK:
    """Numerical IK on the wrist camera pose, straight off the simulator."""

    def __init__(self, env, camera: str = "wrist",
                 limits_deg=None):
        self.env = env
        self.model = env.model
        self.data = env.data
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if self.camera_id < 0:
            raise ValueError(f"camera {camera!r} not in the model")
        self.qpos_index = env._joint_qpos[:ARM_JOINTS]
        default = np.array([[-109, 121], [-107, 109], [-96.7, 94.8],
                            [-104.6, 98.8], [-83, 61]], float)
        self.limits = np.radians(default if limits_deg is None else np.asarray(limits_deg))
        # Position error per radian of aim error. Too high and the solver buys
        # orientation with position -- which reads as a solved pose whose camera
        # is nowhere near the target.
        self.rot_weight = 0.06

    def camera_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.data.qpos[self.qpos_index] = q
        mujoco.mj_forward(self.model, self.data)
        return (self.data.cam(self.camera_id).xpos.copy(),
                self.data.cam(self.camera_id).xmat.reshape(3, 3).copy())

    def solve(self, position, rotation, seed=None, max_nfev: int = 200):
        target_p = np.asarray(position, float)
        target_r = np.asarray(rotation, float)
        start = np.clip(np.asarray(seed, float) if seed is not None else
                        (self.limits[:, 0] + self.limits[:, 1]) / 2.0,
                        self.limits[:, 0] + 1e-3, self.limits[:, 1] - 1e-3)

        target_forward = -np.asarray(target_r, float)[:, 2]
        target_forward /= np.linalg.norm(target_forward)

        def residual(q):
            pos, rot = self.camera_pose(q)
            forward = -rot[:, 2]
            forward = forward / max(1e-9, np.linalg.norm(forward))
            # Aim is constrained as a DIRECTION, not a full rotation: five joints
            # can hold position plus where the camera points, leaving the roll
            # about the view axis free.  Asking for a specific roll made the
            # solver trade position for an orientation the arm cannot reach.
            return np.concatenate([(pos - target_p),
                                   np.cross(forward, target_forward) * self.rot_weight])

        result = least_squares(residual, start, bounds=(self.limits[:, 0], self.limits[:, 1]),
                               x_scale="jac", max_nfev=max_nfev, xtol=1e-12, ftol=1e-12)
        pos, rot = self.camera_pose(result.x)
        forward = -rot[:, 2]
        forward /= max(1e-9, np.linalg.norm(forward))
        aim = math.degrees(math.acos(max(-1.0, min(1.0, float(forward @ target_forward)))))
        return {"q": result.x,
                "position_error_mm": float(np.linalg.norm(pos - target_p) * 1000.0),
                "aim_error_deg": float(aim),
                "cost": float(np.linalg.norm(residual(result.x)))}

    def aim_at(self, point, distance_mm: float, pitch_deg: float = 20.0,
               azimuth_deg: float = 0.0):
        """Camera pose that stares at ``point`` from ``distance_mm`` away.

        ``pitch`` tilts the viewpoint off vertical; the camera's optical axis is
        always pointed straight at the target, which is the property the marker
        read-back verifies.
        """
        point = np.asarray(point, float)
        tilt = (Rotation.from_euler("z", azimuth_deg, degrees=True).as_matrix()
                @ Rotation.from_euler("x", pitch_deg, degrees=True).as_matrix())
        up = np.array([0.0, 0.0, 1.0])
        forward = tilt @ (-up)                    # camera -> target, down-ish
        forward /= np.linalg.norm(forward)
        position = point - forward * (distance_mm / 1000.0)
        # MuJoCo camera frames look along -z (OpenGL convention), so the camera's
        # z axis is the BACKWARD direction.  Building it with +z put every target
        # 180 degrees out and the solver answered with ~115 degree aim errors.
        z = -forward
        up_hint = tilt @ np.array([0.0, 1.0, 0.0])
        y = up_hint - z * float(up_hint @ z)
        if np.linalg.norm(y) < 1e-6:
            y = np.array([0.0, 1.0, 0.0]) - z * float(z[1])
        y /= np.linalg.norm(y)
        x = np.cross(y, z)
        return position, np.column_stack([x, y, z])
