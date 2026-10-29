"""Calibrated pinhole geometry for mapping image detections onto the board.

This module intentionally accepts image pixels, not simulator piece poses. A
detector may propose a pixel for a piece base; the camera pose/intrinsics and
board-plane calibration map that point to a world XY target. No piece identity,
square occupancy, or simulator oracle is consulted here.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    camera_position: tuple[float, float, float]
    camera_rotation: tuple[float, ...]  # camera-local axes expressed in world coordinates

    @classmethod
    def from_mujoco(cls, model: mujoco.MjModel, data: mujoco.MjData,
                    camera: str, width: int, height: int) -> "CameraCalibration":
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if cid < 0:
            raise ValueError(f"unknown camera: {camera}")
        fovy = np.deg2rad(float(model.cam_fovy[cid]))
        fy = height / (2.0 * np.tan(fovy / 2.0))
        # MuJoCo's camera fovy is vertical; square pixels are assumed.
        fx = fy
        return cls(width, height, fx, fy, width / 2.0, height / 2.0,
                   tuple(map(float, data.cam_xpos[cid])),
                   tuple(map(float, data.cam_xmat[cid].reshape(3, 3).ravel())))

    def project(self, world_xyz) -> tuple[float, float] | None:
        """Project a world point to pixel coordinates; None means behind lens."""
        rotation = np.asarray(self.camera_rotation).reshape(3, 3)
        camera_xyz = rotation.T @ (np.asarray(world_xyz, dtype=float) - self.camera_position)
        depth = -float(camera_xyz[2])  # MuJoCo cameras look down local -Z.
        if depth <= 0:
            return None
        return (self.cx + self.fx * camera_xyz[0] / depth,
                self.cy - self.fy * camera_xyz[1] / depth)

    def unproject_to_plane(self, pixel_xy, plane_z: float) -> tuple[float, float] | None:
        """Intersect a pixel ray with a horizontal board plane; reject invalid rays."""
        u, v = map(float, pixel_xy)
        if not (0 <= u < self.width and 0 <= v < self.height):
            return None
        ray_camera = np.array([(u - self.cx) / self.fx,
                               -(v - self.cy) / self.fy, -1.0], dtype=float)
        rotation = np.asarray(self.camera_rotation).reshape(3, 3)
        ray_world = rotation @ ray_camera
        if abs(ray_world[2]) < 1e-9:
            return None
        t = (plane_z - self.camera_position[2]) / ray_world[2]
        if t <= 0:
            return None
        point = np.asarray(self.camera_position) + t * ray_world
        return float(point[0]), float(point[1])


@dataclass(frozen=True)
class Localization:
    """A detected piece-base pixel mapped to board coordinates."""
    world_xy: tuple[float, float]
    pixel_xy: tuple[float, float]
    confidence: float
    sigma_m: float

    def acceptable(self, max_sigma_m: float = 0.004, min_confidence: float = 0.7) -> bool:
        return self.confidence >= min_confidence and self.sigma_m <= max_sigma_m


def localize_piece_base(calibration: CameraCalibration, pixel_xy,
                        board_z: float, confidence: float,
                        pixel_sigma: float = 1.5,
                        height_sigma_m: float = 0.0) -> Localization | None:
    """Map a detector's base pixel to the calibrated board plane.

    `sigma_m` is a first-order conservative uncertainty estimate obtained by
    propagating a pixel-radius through the plane intersection. Low confidence
    remains representable so callers can abstain rather than force a grasp.
    """
    xy = calibration.unproject_to_plane(pixel_xy, board_z)
    if xy is None:
        return None
    u, v = map(float, pixel_xy)
    samples = [calibration.unproject_to_plane((u + dx, v + dy), board_z)
               for dx, dy in ((pixel_sigma, 0), (-pixel_sigma, 0),
                              (0, pixel_sigma), (0, -pixel_sigma))]
    valid = [np.asarray(p) for p in samples if p is not None]
    sigma = max((float(np.linalg.norm(p - xy)) for p in valid), default=float("inf"))
    if height_sigma_m < 0 or not np.isfinite(height_sigma_m):
        raise ValueError("height_sigma_m must be finite and non-negative")
    if height_sigma_m:
        height_samples = [calibration.unproject_to_plane(pixel_xy, board_z + dz)
                          for dz in (-height_sigma_m, height_sigma_m)]
        sigma = max(sigma, *(float(np.linalg.norm(np.asarray(p) - xy))
                             for p in height_samples if p is not None))
    return Localization(xy, (u, v), float(confidence), sigma)
