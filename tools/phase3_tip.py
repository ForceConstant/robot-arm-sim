#!/usr/bin/env python3
"""Where does the 110.45 mm jaw-tip constant actually come from?

``tools/derive_tool_tip.py`` takes ``max(-z)`` over every vertex of every geom on
the jaw bodies, in the ``so101:gripper`` frame.  That rule is only right if the
deepest vertex is a real, broad contact surface.  If the last few millimetres
come from a thin lip, chamfer or a handful of outlier vertices, the effective
tip plane is shallower -- and 6 mm is exactly the stock-vs-cad difference that
T2 flagged.

This prints the depth profile: how many vertices live in each 1 mm slice of the
deepest 15 mm, in total and within 8 mm of the jaw axis, plus the deepest few
vertices' radial distance.  A broad face shows hundreds of vertices per slice;
a spike shows a handful.

    .venv/bin/python tools/phase3_tip.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


def vertices_of(model, gid):
    import mujoco
    kind = model.geom_type[gid]
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mid = int(model.geom_dataid[gid])
        adr = int(model.mesh_vertadr[mid])
        count = int(model.mesh_vertnum[mid])
        return np.asarray(model.mesh_vert[adr:adr + count], dtype=float).reshape(-1, 3)
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        size = np.asarray(model.geom_size[gid], float)
        return np.array([[i * size[0], j * size[1], k * size[2]]
                         for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        size = np.asarray(model.geom_size[gid], float)
        ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        ring = np.array([[size[0] * math.cos(a), size[0] * math.sin(a)]
                         for a in ang])
        return np.array([[x, y, z * size[1]] for z in (-1, 1) for x, y in ring])
    return np.zeros((0, 3))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", default="cad_parallel")
    ap.add_argument("--assets", default=str(ROOT / ".private/gripper-assets-5y"))
    ap.add_argument("--aperture", type=float, default=38.4)
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_tip.json"))
    args = ap.parse_args()

    os.environ["ROBOT_GRIPPER"] = args.pack
    os.environ["ROBOT_GRIPPER_ASSETS"] = args.assets
    os.environ.setdefault("MUJOCO_GL", "egl")
    sys.path.insert(0, str(ROOT))

    import mujoco
    from chess_sim import gripper, scene

    if not gripper.CUSTOM:
        raise SystemExit("pack did not select")
    model = scene.build_arm().compile()
    data = mujoco.MjData(model)
    home = {"shoulder_pan": 0.1, "shoulder_lift": -55.1, "elbow_flex": 72.0,
            "wrist_flex": 39.3, "wrist_roll": 22.4, "gripper": args.aperture}
    for index, name in enumerate(JOINTS):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "so101:" + name)
        data.qpos[model.jnt_qposadr[jid]] = math.radians(float(home[name]))
    mujoco.mj_forward(model, data)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "so101:gripper")
    rot = np.asarray(data.body(body).xmat, float).reshape(3, 3)
    org = np.asarray(data.body(body).xpos, float)

    per_body = {}
    all_pts = []
    for gid in range(model.ngeom):
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                  int(model.geom_bodyid[gid])) or ""
        if "jaw" not in bname:              # same rule derive_tool_tip uses
            continue
        pts = vertices_of(model, gid)
        if not len(pts):
            continue
        # mesh vertices are in the geom frame; put them in the body frame
        world = (np.asarray(data.geom_xpos[gid], float)
                 + pts @ np.asarray(data.geom_xmat[gid], float).reshape(3, 3).T)
        local = (world - org) @ rot
        per_body.setdefault(bname, []).append(local)
    for bname, blocks in sorted(per_body.items()):
        pts = np.vstack(blocks)
        all_pts.append(pts)
        depth_mm = -pts[:, 2] * 1000.0
        radial_mm = np.hypot(pts[:, 0], pts[:, 1]) * 1000.0
        print(f"\n{bname}: {len(pts)} vertices, depth range "
              f"{depth_mm.min():.2f}..{depth_mm.max():.2f} mm, "
              f"radial {radial_mm.min():.1f}..{radial_mm.max():.1f} mm")
        for lo in range(int(depth_mm.max()) - 15, int(depth_mm.max()) + 1):
            sel = (depth_mm >= lo) & (depth_mm < lo + 1)
            near = sel & (radial_mm < 8.0)
            if sel.sum():
                print(f"  depth {lo:6.2f}-{lo+1:6.2f} mm: {int(sel.sum()):6d} verts "
                      f"({int(near.sum())} within 8 mm of axis)")
        idx = np.argsort(-depth_mm)[:8]
        print("  deepest:", [(round(float(depth_mm[i]), 2),
                              round(float(radial_mm[i]), 2)) for i in idx])

    pts = np.vstack(all_pts)
    depth = -pts[:, 2] * 1000.0
    radial = np.hypot(pts[:, 0], pts[:, 1]) * 1000.0
    # A contact plane needs vertices spread over the jaw face, not a spike:
    # report the deepest depth that still has >= 20 vertices within 2 mm, and
    # the deepest depth among vertices within 8 mm of the axis.
    near_axis = depth[radial < 8.0]
    result = {"pack": args.pack, "aperture": args.aperture,
              "vertices": int(len(pts)),
              "deepest_mm": round(float(depth.max()), 3),
              "deepest_near_axis_mm": round(float(near_axis.max()), 3),
              "deepest_with_20_verts_2mm_mm": None}
    counts = []
    for d in np.arange(depth.max(), depth.max() - 20, -0.1):
        counts.append((float(d), int((depth >= d).sum())))
    for d, n in counts:
        if n >= 20:
            result["deepest_with_20_verts_2mm_mm"] = round(d, 3)
            break
    print("\nsummary:", json.dumps(result, indent=1))
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
