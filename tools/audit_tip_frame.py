#!/usr/bin/env python3
"""Print chain-frame -> physical-tip from the MJCF, across poses.

The live solver has three competing "tip" numbers in the tree.  This probe
measures the transform the model actually encodes: for a set of joint poses it
puts every jaw geom vertex in the ``so101:gripper`` body frame and reports the
deepest point along the approach axis (body -z), which is the fingertip plane.
A convention is only usable if that transform is constant over poses.

    robot-arm-sim python tools/audit_tip_frame.py --json reports/tip_frame.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402

from chess_sim import gripper, scene  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")

# Eight poses covering the working range.  Degrees, as the live app speaks.
POSES = {
    "home":        (0.1, -55.1, 72.0, 39.3, 22.4, 38.4),
    "pan_left":    (-40.0, -55.0, 72.0, 39.0, 22.4, 38.4),
    "pan_right":   (40.0, -55.0, 72.0, 39.0, 22.4, 38.4),
    "lift_low":    (0.0, -20.0, 40.0, 10.0, 0.0, 38.4),
    "lift_high":   (0.0, -60.0, 90.0, 30.0, 0.0, 38.4),
    "elbow_in":    (0.0, -55.0, 20.0, -50.0, 0.0, 38.4),
    "flex_open":   (0.0, -55.0, 55.0, 60.0, 0.0, 10.0),
    "roll_swing":  (0.0, -55.0, 55.0, 20.0, -90.0, 70.0),
}


def _rotation(quat) -> np.ndarray:
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.asarray(quat, dtype=float))
    return out.reshape(3, 3)


def geom_vertices(model, gid: int) -> np.ndarray:
    """Vertices of a geom in its own frame (mesh verts or box corners)."""
    kind = model.geom_type[gid]
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mid = model.geom_dataid[gid]
        adr = int(model.mesh_vertadr[mid])
        num = int(model.mesh_vertnum[mid])
        return np.asarray(model.mesh_vert[adr:adr + num], dtype=float).reshape(-1, 3)
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        s = model.geom_size[gid]
        return np.array([[i * s[0], j * s[1], k * s[2]]
                         for i in (-1.0, 1.0) for j in (-1.0, 1.0)
                         for k in (-1.0, 1.0)])
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        r = float(model.geom_size[gid][0])
        return np.array([[0.0, 0.0, r], [0.0, 0.0, -r]])
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        r, h = model.geom_size[gid][:2]
        return np.array([[0, 0, h], [0, 0, -h], [r, 0, 0], [-r, 0, 0]])
    return np.zeros((0, 3))


def descendants(model, root_body: int) -> list[int]:
    bodies, frontier = [], [root_body]
    while frontier:
        bid = frontier.pop()
        bodies.append(bid)
        frontier.extend([int(b) for b in range(model.nbody)
                         if int(model.body_parentid[b]) == bid])
    return bodies


def measure(model, data, pose_deg) -> dict:
    rad = np.radians(np.asarray(pose_deg, dtype=float))
    for index, name in enumerate(JOINTS):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "so101:" + name)
        if jid < 0:
            raise SystemExit(f"joint {name} missing from the arm model")
        data.qpos[model.jnt_qposadr[jid]] = rad[index]
    mujoco.mj_forward(model, data)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "so101:gripper")
    position = np.asarray(data.body(body_id).xpos, dtype=float)
    rotation = np.asarray(data.body(body_id).xmat, dtype=float).reshape(3, 3)

    bodies = set(descendants(model, body_id))
    deepest, which = -1e9, None
    pads: dict[str, float] = {}
    for gid in range(model.ngeom):
        if int(model.geom_bodyid[gid]) not in bodies:
            continue
        verts = geom_vertices(model, gid)
        if not len(verts):
            continue
        world = (np.asarray(data.geom_xpos[gid], dtype=float)
                 + verts @ np.asarray(data.geom_xmat[gid], dtype=float).reshape(3, 3).T)
        local = (world - position) @ rotation          # body frame
        depth = float(np.max(-local[:, 2]))
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.endswith("_pad"):
            pads[name.split(":")[-1]] = depth * 1000.0
        if depth > deepest:
            deepest = depth
            deepest_index = int(np.argmin(local[:, 2]))
            deepest_point = local[deepest_index]
            which = (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
                     or f"geom{gid}")

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "so101:gripperframe")
    site_pos = np.asarray(model.site_pos[site_id], dtype=float)
    site_rot = _rotation(model.site_quat[site_id])
    # gripper.py: +x approach, +z jaw gap; TIP_X is measured from the pocket.
    tip_from_site = site_pos + site_rot @ np.array([gripper.TIP_X, 0.0, 0.0])

    return {"pose": list(pose_deg),
            "body_position": position.tolist(),
            "deepest_mm": deepest * 1000.0,
            "deepest_point_mm": (deepest_point * 1000.0).tolist(),
            "geom": which,
            "pads_mm": pads,
            "site_pos_mm": (site_pos * 1000.0).tolist(),
            "site_z_mm": float(site_pos[2] * 1000.0),
            "tip_from_site_mm": (tip_from_site * 1000.0).tolist(),
            "tip_from_site_z_mm": float(tip_from_site[2] * 1000.0)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", default="")
    args = parser.parse_args()

    spec = scene.build_arm()
    model = spec.compile()
    data = mujoco.MjData(model)
    rows = [measure(model, data, pose) for pose in POSES.values()]

    depths = np.array([r["deepest_mm"] for r in rows])
    site_z = np.array([r["site_z_mm"] for r in rows])
    tip_z = np.array([r["tip_from_site_z_mm"] for r in rows])
    print(f"model: stock={not gripper.CUSTOM} tip_x={gripper.TIP_X * 1000:.3f} mm")
    print(f"{'pose':>11} {'deepest geom':>24} {'deep mm':>9} {'site z':>9} {'site+tip z':>11}")
    for r in rows:
        print(f"{'':>0}{r['pose'][0]:>11.1f} {r['geom']:>24} {r['deepest_mm']:>9.3f} "
              f"{r['site_z_mm']:>9.3f} {r['tip_from_site_z_mm']:>11.3f}")
    print()
    print(f"deepest point in chain frame: mean -z {depths.mean():.3f} mm, "
          f"spread {depths.max() - depths.min():.4f} mm")
    print(f"site in chain frame:          mean -z {-site_z.mean():.3f} mm, "
          f"spread {site_z.max() - site_z.min():.4f} mm")
    print(f"site + TIP_X:                 mean -z {-tip_z.mean():.3f} mm, "
          f"spread {tip_z.max() - tip_z.min():.4f} mm")
    for key in ("gripper_pad", "moving_jaw_so101_v1_pad"):
        values = [r["pads_mm"].get(key) for r in rows if key in r["pads_mm"]]
        if values:
            print(f"{key:>24}: {min(values):.3f}..{max(values):.3f} mm over poses")
    x = np.array([r["deepest_point_mm"][0] for r in rows])
    y = np.array([r["deepest_point_mm"][1] for r in rows])
    print(f"deepest point lateral: x {x.mean():.3f} +/- {x.std():.3f} mm, "
          f"y {y.mean():.3f} +/- {y.std():.3f} mm")

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(
            {"poses": rows,
             "deepest_mean_mm": float(depths.mean()),
             "deepest_spread_mm": float(depths.max() - depths.min()),
             "site_mean_mm": float(-site_z.mean()),
             "tip_from_site_mean_mm": float(-tip_z.mean()),
             "custom_gripper": gripper.CUSTOM,
             "tip_x_mm": gripper.TIP_X * 1000.0}, indent=2) + "\n")


if __name__ == "__main__":
    main()
