"""Scene-level aim laser: red jaw-origin -> tip -> board ray, yellow impact.

Baked into the sim environment (when ``AppearanceConfig.aim_laser`` is on) so it
appears in *every* render — external, top, wrist, offscreen — without any render
tool drawing on pixels. The geometry follows the arm: ``update`` is called after
each physics step and before observations.

Definitions come from tip-center.md §1: the ray starts at the jaw origin (the
``so101:gripper`` body origin / wrist axis), passes through the tip centre
110.45 mm down that axis, and continues to the board plane. No IRL data, no
oracle board state: only the sim model's own body pose and board surface.

The pieces are visual-only (contype/conaffinity 0, no collisions) and carry no
mass, so physics and perception are unchanged apart from the visible marks.
"""
from __future__ import annotations

import numpy as np

import mujoco

from .config import BoardConfig
from .scene import ARM_PREFIX

RAY = "aim_laser_ray"               # thin red capsule along the ray
IMPACT = "aim_laser_impact"        # yellow impact marker
RAY_RADIUS_M = 0.0009             # 0.9 mm, reads as a thin laser line
IMPACT_RADIUS_M = 0.0045
IMPACT_LIFT_M = 0.003             # sit above the board so it is not z-fighting/occluded
TIP_DEPTH_M = 0.11045             # so101:gripper -> tip centre, measured
MAX_RAY_M = 1.5                   # clamp when the ray misses the board


def _quat_from_to(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """MuJoCo quat (w,x,y,z) rotating +z onto b-a."""
    direction = np.asarray(b, float) - np.asarray(a, float)
    direction /= max(1e-12, float(np.linalg.norm(direction)))
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(direction @ up)) > 0.999:       # vertical: use +x as reference
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(up, direction)
    right /= np.linalg.norm(right)
    real_up = np.cross(direction, right)
    R = np.column_stack([right, real_up, direction])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R.reshape(-1))
    return quat


def add_aim_laser(spec: mujoco.MjSpec, board: BoardConfig, *, tip_depth_m: float = TIP_DEPTH_M):
    """Add the laser bodies/geoms to a scene spec (no-op if already present).

    Returns ``tip_depth_m`` so the caller can store it for ``update``.
    """
    if any(g.name == RAY for g in spec.worldbody.geoms):
        return tip_depth_m
    spec.add_material(name="mat_aim_ray", rgba=[1.0, 0.0, 0.0, 1.0],
                      specular=0.0, shininess=0.0)
    spec.add_material(name="mat_aim_impact", rgba=[1.0, 0.85, 0.0, 1.0],
                      specular=0.0, shininess=0.0)
    # Mocap bodies, like the board: update() writes mocap_pos/mocap_quat.
    # A welded body's xpos is recomputed from qpos by mj_forward and would
    # snap back to the origin; mocap is the supported way to move a body at
    # runtime without joints or collisions.
    ray = spec.worldbody.add_body(name="aim_laser_ray_body", pos=[0, 0, 0], mocap=True)
    ray.add_geom(name=RAY, type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                 size=[RAY_RADIUS_M, 0.1, 0], material="mat_aim_ray",
                 contype=0, conaffinity=0, density=0)
    impact = spec.worldbody.add_body(name="aim_laser_impact_body", pos=[0, 0, 0], mocap=True)
    impact.add_geom(name=IMPACT, type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[IMPACT_RADIUS_M, 0, 0], material="mat_aim_impact",
                    contype=0, conaffinity=0, density=0)
    return tip_depth_m


def update(model: mujoco.MjModel, data: mujoco.MjData, *,
           board_z: float, tip_depth_m: float = TIP_DEPTH_M):
    """Move the laser geoms to the current aim ray. Cheap; called every step.

    Returns ``(impact_mm, on_board)`` for diagnostics. No-op if the geoms were
    not added (e.g. the feature is disabled).
    """
    ray_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, RAY)
    hit_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, IMPACT)
    if ray_id < 0 or hit_id < 0:
        return None, False

    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ARM_PREFIX + "gripper")
    if gid < 0:
        return None, False
    origin = np.asarray(data.xpos[gid], float)
    R = np.asarray(data.xmat[gid], float).reshape(3, 3)
    axis = -R[:, 2]                          # tool +z points back up the jaws
    tip = origin + tip_depth_m * axis
    denom = float(axis[2])
    distance = (board_z - origin[2]) / denom if abs(denom) > 1e-6 else MAX_RAY_M
    distance = float(np.clip(distance, 0.0, MAX_RAY_M))
    hit = origin + distance * axis

    # Capsule: geom frame +z spans its half-length, so centre the capsule
    # between the two endpoints and align +z with the ray direction.
    centre = (origin + hit) / 2.0
    quat = _quat_from_to(origin, hit)
    ray_bid = int(model.geom_bodyid[ray_id])
    hit_bid = int(model.geom_bodyid[hit_id])
    r_mocap = int(model.body_mocapid[ray_bid])
    h_mocap = int(model.body_mocapid[hit_bid])
    data.mocap_pos[r_mocap] = centre
    data.mocap_quat[r_mocap] = quat
    data.mocap_pos[h_mocap] = hit + np.array([0.0, 0.0, IMPACT_LIFT_M])
    data.mocap_quat[h_mocap] = [1.0, 0.0, 0.0, 0.0]
    half = max(1e-4, float(np.linalg.norm(hit - origin)) / 2.0)
    model.geom_size[ray_id] = [RAY_RADIUS_M, half, 0.0]
    # Mocap bodies are positioned by mj_forward from mocap_pos/quat; call it
    # so a render immediately after a joint write sees the updated laser.
    mujoco.mj_forward(model, data)

    on_board = abs(float(hit[0])) < 0.5 * board_side(board_z) and abs(float(hit[1])) < 0.5 * board_side(board_z)
    return (hit * 1000.0).round(2), on_board


def board_side(board_z: float) -> float:
    """Board width in metres (procedural 8x8 board, square 28 mm + 10 mm border)."""
    return 8 * 0.028 + 2 * 0.010
