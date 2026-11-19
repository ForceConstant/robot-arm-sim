#!/usr/bin/env python3
"""A2 evidence: measure the compiled full sim against the frozen B5 reference.

Reports, in the gripper body frame G (the sim's calibrated chain frame):
  * base / jaw L / jaw R / gear world bboxes vs the reference (mapped into G)
  * tip depth
  * compiled wrist camera pose vs the reference camera
  * jaw-in-frustum share with the compiled camera
  * aspect: fovy, and fov_x back-computed from the rendered aspect

The reference's tool_mount frame W is mapped by W = Rz(-90 deg) * G, derived
from the compiled base placement (residual printed).

Usage:
  ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-5y \
  ~/.local/bin/robot-arm-sim python tools/a2_reference_check.py [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from chess_sim import gripper, scene, tool_reference as ref  # noqa: E402

JAW_ANGLE = ref.REFERENCE_GRIPPER_RAD


def cloud(m, d, body, mesh_hint):
    b = m.body(body).id
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi] != b or m.geom_group[gi] != 2:
            continue
        if m.geom_type[gi] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        nm = str(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[gi]))
        if mesh_hint not in nm:
            continue
        mid = m.geom_dataid[gi]
        v = np.array(m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]])
        return (np.array(d.geom_xmat[gi]).reshape(3, 3) @ v.T).T + np.array(d.geom_xpos[gi])
    raise KeyError((body, mesh_hint))


def to_g(pts, Rg, pg):
    return (Rg.T @ (np.asarray(pts) - pg).T).T


def bbox(pts):
    lo, hi = pts.min(0) * 1000.0, pts.max(0) * 1000.0
    return lo, hi


def ref_bbox_in_g(key):
    """The reference's measured bbox, mapped from W into G (corner-wise)."""
    lo, hi = np.array(ref.MEASURED[key + "_bbox_mm"], dtype=float)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    g = (ref.rz(90.0) @ corners.T).T
    return g.min(0), g.max(0)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None)
    args = ap.parse_args(argv)

    m = scene.build_scene().compile()
    d = mujoco.MjData(m)
    jid = m.joint("so101:gripper").id
    d.qpos[m.jnt_qposadr[jid]] = JAW_ANGLE
    gripper.seed_coupled(m, d, "so101:")
    mujoco.mj_forward(m, d)

    gid = m.body("so101:gripper").id
    Rg = np.array(d.xmat[gid]).reshape(3, 3)
    pg = np.array(d.xpos[gid])

    parts = {
        "base": ("so101:cad:parallel_base", "complete_fixed_jaw"),
        "jaw_left": ("so101:cad:parallel_jaw_left", "jaw_1__solid1"),
        "jaw_right": ("so101:cad:parallel_jaw_right", "jaw_1__solid1"),
        "gear": ("so101:cad:parallel_pinion", "gear_1__solid1"),
    }
    out = {"git_head": os.popen("git -C %s rev-parse --short HEAD"
                                % os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                                ).read().strip(),
           "camera_source": scene.CAMERA_SOURCE,
           "gripper_angle_rad": JAW_ANGLE,
           "pack": {"kind": gripper.KIND, "root": str(gripper.ASSET_ROOT)},
           "bboxes_mm": {}, "deltas_mm": {}}
    clouds = {}
    for key, (body, hint) in parts.items():
        pts = to_g(cloud(m, d, body, hint), Rg, pg)
        clouds[key] = pts
        lo, hi = bbox(pts)
        out["bboxes_mm"][key] = {"min": [round(float(x), 4) for x in lo],
                                 "max": [round(float(x), 4) for x in hi]}
        mref = np.array(ref_bbox_in_g(key))
        out["deltas_mm"][key] = {
            "min": [round(float(a - b), 4) for a, b in zip(lo, mref[0])],
            "max": [round(float(a - b), 4) for a, b in zip(hi, mref[1])],
            "max_abs": round(float(max(np.abs(lo - mref[0]).max(),
                                       np.abs(hi - mref[1]).max())), 4)}

    jaws = np.vstack([clouds["jaw_left"], clouds["jaw_right"]])
    out["tip_depth_mm"] = round(float(-jaws[:, 2].min()) * 1000.0, 4)
    out["tip_depth_reference_mm"] = ref.MEASURED["tip_depth_mm"]

    # --- camera: local pose in the gripper body ---------------------------
    want_pos, want_R = ref.camera_in_gripper()
    cid = m.camera("wrist").id
    cpos = np.array(m.cam_pos[cid])
    q = np.array(m.cam_quat[cid])
    Rc = np.zeros(9)
    mujoco.mju_quat2Mat(Rc, q)
    Rc = Rc.reshape(3, 3)
    dpos = (cpos - want_pos) * 1000.0
    dR = Rc.T @ want_R
    dang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
    out["camera"] = {
        "compiled_pos_mm": [round(float(x) * 1000, 6) for x in cpos],
        "reference_pos_mm_in_G": [round(float(x) * 1000, 6) for x in want_pos],
        "delta_mm": [round(float(x), 6) for x in dpos],
        "delta_norm_mm": round(float(np.linalg.norm(dpos)), 6),
        "delta_rot_deg": round(float(dang), 6),
        "fovy_deg": round(float(m.cam_fovy[cid]), 6),
        "reference_fovy_deg": round(ref.fovy_deg(), 6),
    }

    # --- jaw in frustum, accepted convention ------------------------------
    i = ref.CAMERA["intrinsics"]
    x = (Rc.T @ (jaws - cpos).T).T
    z = -x[:, 2]
    ok = z > 1e-9
    u = np.full(len(x), np.nan)
    v = np.full(len(x), np.nan)
    u[ok] = i["cx"] + i["fx"] * x[ok, 0] / z[ok]
    v[ok] = i["cy"] - i["fy"] * x[ok, 1] / z[ok]
    inside = ok & (u >= 0) & (u < i["width"]) & (v >= 0) & (v < i["height"])
    out["jaw_in_frustum"] = {"pct": round(100.0 * inside.sum() / len(x), 4),
                             "points": int(len(x)), "inside": int(inside.sum()),
                             "reference_pct": ref.MEASURED["jaw_in_frustum_pct"]}

    # --- aspect -----------------------------------------------------------
    out["aspect"] = {
        "render": {"width": int(m.vis.global_.offwidth), "height": int(m.vis.global_.offheight)},
        "intrinsics": {"width": i["width"], "height": i["height"]},
        "fovy_deg": round(ref.fovy_deg(), 6),
        "fovx_from_aspect_640x480_deg": round(ref.fovx_from_aspect(640, 480, ref.fovy_deg()), 6),
        "fovx_from_intrinsics_deg": ref.ASPECT["fov_x_deg"],
        "residual_deg": round(ref.fovx_from_aspect(640, 480, ref.fovy_deg())
                              - ref.ASPECT["fov_x_deg"], 6),
    }

    text = json.dumps(out, indent=2)
    print(text)
    if args.json:
        open(args.json, "w").write(text + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
