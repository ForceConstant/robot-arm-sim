#!/usr/bin/env python3
"""B5 reference scene — the accepted tool + camera arrangement, reproducible.

Self-contained port of the throwaway render script that produced the accepted
sheet (``/tmp/b5_final_4up.png`` on the BC-250, copied into the repo as
``reports/b5_reference_4up.png``).  No ``/tmp`` paths, no imports from the
simulator, no scratch state: the mesh files plus the constants below are the
whole input.

Modes
    sheet   three axis views + the camera's own view   (the accepted image)
    axis    the three axis views only
    camera  the camera's own view only
    config  no render; export the machine-readable configuration

Every number comes from the code that actually rendered:

    tool poses    TARGET below: the CAD pose of each mesh inside its body.
                  ``resolve_frames`` re-measures those poses against the mesh
                  files with a Kabsch/ICP pass and prints the residual; it is
                  a verification pass, not a fit (the source settled at 0).
    camera pose   reports/block4c_calibration.json -> camera_in_tool,
                  calibration id handeye-2026-11-18T11-block4c.
    intrinsics    same calibration: fx 831.897, fy 833.014, cx 320, cy 240
                  at 640x480 (rendered through MuJoCo with fovy from fy/cy).

Camera construction, exactly as accepted: start from the calibrated pose,
swing the optical axis into the x-z plane (drop its y component), zero the
roll, apply ``--roll`` degrees about the optical axis (the accepted image
used 180), then move 15 mm along the *calibrated* axis.

Frames
    tool_mount  scene world. Origin on the tool mount face, +z away from the
                arm; the mount face was moved +13.25 mm so the base's face
                plane lands on z = 0.  Jaw tips sit at z = -110.600 mm.
    body        each geom's parent body frame (``parallel_base`` hosts both
                jaws and the gear).

Usage
    robot-arm-sim python tools/b5_reference_scene.py
    robot-arm-sim python tools/b5_reference_scene.py --mode axis
    robot-arm-sim python tools/b5_reference_scene.py --mode camera --roll 0
    robot-arm-sim python tools/b5_reference_scene.py --mode config

The STL parser reads binary STL only (same as the source script).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
from datetime import datetime, timezone

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

# --------------------------------------------------------------------------- #
# Inputs
# --------------------------------------------------------------------------- #

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MESH_FILES = {
    "base_mesh": os.path.join(REPO, ".private/complete_fixed_jaw_bin.stl"),
    "finger_mesh": os.path.join(REPO, ".private/finger_jaw_ui_bin.stl"),
    "gear_mesh": os.path.join(REPO, ".private/gripper-assets/assets/gripper_gear_1__solid1.stl"),
}

GROUND_Z = -0.130          # m, scene ground plane (presentation only)
MOUNT_FACE_OFFSET_MM = 13.25   # base mesh shifted +z so its mount face sits on z = 0
TIP_Z = -0.1106            # m, tip plane drawn in the sheet (accepted value)

# The tool pack's own axes, expressed as the rotation each mesh needs inside
# its body.  Taken verbatim from the accepted render script.
R_BASE = np.array([[0.0, 0.0, 1.0],
                   [0.0, 1.0, 0.0],
                   [-1.0, 0.0, 0.0]])
R_GEAR = np.array([[0.0, -0.386309, -0.922370],
                   [0.0, -0.922370, 0.386309],
                   [-1.0, 0.0, 0.0]])

TARGET = {
    "base_visual": ("base_mesh", R_BASE, np.array([0.0, 0.0, 0.01325])),
    "jaw_left_visual": ("finger_mesh", None, np.array([-0.0003, -0.027828, -0.0766])),
    "jaw_right_visual": ("finger_mesh", None, np.array([-0.0003, 0.027828, -0.0766])),
    "gear_visual": ("gear_mesh", R_GEAR, np.array([0.0, 0.0, 0.01275])),
}
PARENT = {
    "base_visual": None,
    "jaw_left_visual": "parallel_base",
    "jaw_right_visual": "parallel_base",
    "gear_visual": "parallel_base",
}

# Camera: calibrated pose (tool frame) + intrinsics, provenance in the docstring.
CAM_T_CAL = np.array([0.067656, -0.002916, -0.010885])
CAM_R_CAL = np.array([[-0.845366, -0.218268, 0.487560],
                      [0.155227, -0.973702, -0.166759],
                      [0.511136, -0.065290, 0.857016]])
FX, FY, CX, CY = 831.897, 833.014, 320.0, 240.0
RES_W, RES_H = 640, 480
CAM_SHIFT_M = 0.015        # moved along the calibrated axis

VIEWS = [("VIEW ALONG -X   (camera at +X)", 0.0, 0.0),
         ("VIEW ALONG -Y   (camera at +Y)", 90.0, 0.0),
         ("VIEW ALONG -Z   (camera above, +Z)", 0.0, 88.0)]
DIST, LOOK = 0.31, np.array([0.025, 0.0, -0.049])   # axis-view orbit


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #

def Rz(d):
    a = math.radians(d)
    return np.array([[math.cos(a), -math.sin(a), 0.0],
                     [math.sin(a), math.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])


def _rot(target):
    """Rotation for a target entry; jaws are the same finger mesh mirrored by yaw."""
    gname, (mesh, R_cad, p_cad) = target
    if R_cad is not None:
        return R_cad
    if gname == "jaw_left_visual":
        return Rz(-90)
    if gname == "jaw_right_visual":
        return Rz(90)
    raise KeyError(gname)


def raw_ordered(path):
    """Unique STL vertices in first-seen order (binary STL, 50-byte records)."""
    b = open(path, "rb").read()
    n = int(np.frombuffer(b[80:84], dtype=np.uint32)[0])
    rec = np.frombuffer(b[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    v = rec[:, 12:48].copy().view(np.float32).reshape(n, 3, 3).astype(np.float64).reshape(-1, 3)
    seen, order = {}, []
    for q in map(tuple, np.round(v, 9)):
        if q not in seen:
            seen[q] = len(order)
            order.append(q)
    return np.array(order)


def q_from_R(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    w = math.sqrt(max(0.0, 1.0 + t)) / 2.0
    if w < 1e-6:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        v = [0.0, 0.0, 0.0]
        v[i] = math.sqrt(max(0.0, 1 + R[i, i] - R[(i + 1) % 3, (i + 1) % 3]
                             - R[(i + 2) % 3, (i + 2) % 3])) / 2
        return [0.0] + v
    return [w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
            (R[1, 0] - R[0, 1]) / (4 * w)]


def kabsch(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cd - R @ cs


def world_verts(m, d, gname):
    i = m.geom(gname).id
    mid = m.geom_dataid[i]
    v = np.array(m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]])
    return (np.array(d.geom_xmat[i]).reshape(3, 3) @ v.T).T + np.array(d.geom_xpos[i])


# --------------------------------------------------------------------------- #
# Camera
# --------------------------------------------------------------------------- #

def camera_pose(roll_deg):
    """Calibrated pose -> axis swung into x-z, zero roll, then ``roll_deg``."""
    f_old = -CAM_R_CAL[:, 2]
    f_new = np.array([f_old[0], 0.0, f_old[2]])
    f_new /= np.linalg.norm(f_new)
    z = -f_new
    right = np.cross(np.array([0.0, 0.0, 1.0]), z)
    right /= np.linalg.norm(right)
    up = np.cross(z, right)
    R = np.column_stack([right, up, z])
    Rz_roll = Rz(roll_deg)
    Rz_roll[np.abs(Rz_roll) < 1e-12] = 0.0
    R = R @ Rz_roll
    t = CAM_T_CAL + CAM_SHIFT_M * f_old
    return t, R


def camera_meta(t, R, roll_deg):
    half_v = math.atan(CY / FY)
    half_h = math.atan(CX / FX)
    return {
        "position_m": [float(x) for x in t],
        "position_mm": [round(float(x) * 1000.0, 4) for x in t],
        "optical_axis": [float(x) for x in -R[:, 2]],
        "right": [float(x) for x in R[:, 0]],
        "up": [float(x) for x in R[:, 1]],
        "rotation": [[float(v) for v in row] for row in R],
        "roll_deg": float(roll_deg),
        "calibrated_pose": {
            "position_m": [float(x) for x in CAM_T_CAL],
            "rotation": [[float(v) for v in row] for row in CAM_R_CAL],
            "shift_along_axis_m": CAM_SHIFT_M,
        },
        "intrinsics": {"fx": FX, "fy": FY, "cx": CX, "cy": CY,
                       "width": RES_W, "height": RES_H},
        "fov_y_deg": 2.0 * math.degrees(half_v),
        "fov_x_deg": 2.0 * math.degrees(half_h),
    }


def frustum_pct(points_world, cam_t, cam_R):
    """Share of points inside the 640x480 image, MuJoCo pixel convention."""
    half_v, half_h = math.atan(CY / FY), math.atan(CX / FX)
    x = (cam_R.T @ (np.asarray(points_world) - cam_t).T).T   # camera coords
    z = -x[:, 2]
    ok = z > 1e-9
    u = np.full(len(x), np.nan)
    v = np.full(len(x), np.nan)
    u[ok] = CX + FX * x[ok, 0] / z[ok]
    v[ok] = CY - FY * x[ok, 1] / z[ok]
    inside = ok & (u >= 0) & (u < RES_W) & (v >= 0) & (v < RES_H)
    return float(100.0 * inside.sum() / len(x)), {
        "points": int(len(x)), "inside": int(inside.sum()),
        "half_angle_h_deg": math.degrees(half_h),
        "half_angle_v_deg": math.degrees(half_v),
    }


# --------------------------------------------------------------------------- #
# Scene
# --------------------------------------------------------------------------- #

def build(frames, markers=True, cam_markers=True, width=1880, height=1880,
          cam_t=None, cam_R=None, roll_deg=180.0):
    if cam_t is None or cam_R is None:
        cam_t, cam_R = camera_pose(roll_deg)
    half_v = math.atan(CY / FY)
    half_h = math.atan(CX / FX)
    fovy = 2.0 * math.degrees(half_v)
    cam_view = -cam_R[:, 2]

    spec = mujoco.MjSpec()
    for nm, path in MESH_FILES.items():
        spec.add_mesh(name=nm, file=path)
    spec.visual.global_.offwidth = width
    spec.visual.global_.offheight = height
    for nm, col in (("mat_base", [0.72, 0.73, 0.76, 1]), ("mat_finger", [0.55, 0.56, 0.60, 1]),
                    ("mat_gear", [0.35, 0.36, 0.40, 1]), ("mat_ground", [0.32, 0.32, 0.34, 1]),
                    ("mk_vol", [1.0, 0.15, 0.15, 0.22]), ("mk_org", [1.0, 0.0, 0.0, 1]),
                    ("mk_cross", [1.0, 1.0, 1.0, 1]), ("mk_face", [1.0, 0.62, 0.0, 0.75]),
                    ("mk_tip", [0.2, 1.0, 0.4, 0.6]), ("cam_c", [1.0, 0.30, 0.90, 1.0]),
                    ("cam_f", [1.0, 0.55, 0.95, 0.85])):
        spec.add_material(name=nm, rgba=col, specular=0.15, shininess=0.2)

    w = spec.worldbody
    w.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0.4, 0.4, 0.1], pos=[0, 0, GROUND_Z],
               material="mat_ground", name="ground")
    w.add_light(name="key", type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, dir=[-0.4, 0.3, -1.0],
                diffuse=[0.9, 0.9, 0.9], ambient=[0.35, 0.35, 0.38], castshadow=False)
    w.add_light(name="fill", type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, dir=[0.6, -0.4, -0.5],
                diffuse=[0.45, 0.45, 0.5], castshadow=False)

    wp = {}
    for gname, target in TARGET.items():
        mesh, _, p_cad = target
        R_cad = _rot((gname, target))
        Rc, tc = frames[mesh]
        wp[gname] = (Rc @ R_cad, Rc @ p_cad + tc)

    for gname in ("base_visual", "jaw_left_visual", "jaw_right_visual", "gear_visual"):
        R_w, p_w = wp[gname]
        if PARENT[gname] is None:
            body = w.add_body(name="parallel_base", pos=list(p_w), quat=list(q_from_R(R_w)))
        else:
            Rp, pp = wp["base_visual"]
            body = w.add_body(name=gname.replace("_visual", ""),
                              pos=list(Rp.T @ (p_w - pp)),
                              quat=list(q_from_R(Rp.T @ R_w)))
        mat = {"base_mesh": "mat_base", "finger_mesh": "mat_finger",
               "gear_mesh": "mat_gear"}[TARGET[gname][0]]
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=TARGET[gname][0],
                      material=mat, name=gname)

    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    bb = world_verts(m, d, "base_visual")
    bmin, bmax = bb.min(0), bb.max(0)
    ctr, half = (bmin + bmax) / 2, (bmax - bmin) / 2

    if markers:
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=list(half), pos=list(ctr),
                   material="mk_vol", name="mk_volume", contype=0, conaffinity=0)
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[half[0], half[1], 0.0004],
                   pos=[ctr[0], ctr[1], float(bmax[2])], material="mk_face", name="mk_face",
                   contype=0, conaffinity=0)
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.03, 0.03, 0.0004], pos=[0, 0, TIP_Z],
                   material="mk_tip", name="mk_tipplane", contype=0, conaffinity=0)
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.0025, 0, 0], pos=[0, 0, 0],
                   material="mk_org", name="mk_origin", contype=0, conaffinity=0)
        QY = [0.7071068, 0.0, 0.7071068, 0.0]
        QX = [0.7071068, 0.7071068, 0.0, 0.0]
        for ax, quat in ((0, QY), (1, QX), (2, [1, 0, 0, 0])):
            for sgn in (+1, -1):
                p = [0.0, 0.0, 0.0]
                p[ax] = sgn * 0.012
                w.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.0004, 0.004, 0],
                           pos=p, quat=quat, material="mk_cross",
                           name="mk_cross_%d%d" % (ax, sgn), contype=0, conaffinity=0)

    if cam_markers:
        def frame_from_dir(dvec, side=(1.0, 0.0, 0.0)):
            z = -np.asarray(dvec, float)
            z /= np.linalg.norm(z)
            side = np.asarray(side, float)
            if abs(float(side @ z)) > 0.9:
                side = np.array([0.0, 1.0, 0.0]) if abs(float(z[1])) < 0.9 else np.array([0.0, 0.0, 1.0])
            x = side - float(side @ z) * z
            x /= np.linalg.norm(x)
            return np.column_stack([x, np.cross(z, x), z])

        def rodc(a, b, radius, mat, name):
            a = np.asarray(a, float)
            b = np.asarray(b, float)
            dd = b - a
            L = float(np.linalg.norm(dd))
            if L < 1e-6:
                return
            w.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[radius, L / 2, 0],
                       pos=list((a + b) / 2), quat=list(q_from_R(frame_from_dir(dd))),
                       material=mat, name=name, contype=0, conaffinity=0)

        w.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.0022, 0, 0], pos=list(cam_t),
                   material="cam_c", name="cam_dot", contype=0, conaffinity=0)
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.004, 0.004, 0.005],
                   pos=list(cam_t + cam_view * 0.005), quat=list(q_from_R(cam_R)),
                   material="cam_c", name="cam_box", contype=0, conaffinity=0)
        rodc(cam_t, cam_t + cam_view * 0.045, 0.0006, "cam_c", "cam_axis")
        D = 0.16
        corners = []
        for sx in (+1, -1):
            for sy in (+1, -1):
                dv = (cam_view
                      + cam_R[:, 0] * math.tan(half_h) * sx
                      + cam_R[:, 1] * math.tan(half_v) * sy)
                dv /= np.linalg.norm(dv)
                P = cam_t + dv * D
                corners.append(P)
                rodc(cam_t, P, 0.00030, "cam_f", "cam_ray_%d%d" % (sx, sy))
        for i in range(4):
            rodc(corners[i], corners[(i + 1) % 4], 0.00030, "cam_f", "cam_rect_%d" % i)

    w.add_camera(name="wrist", pos=list(cam_t), quat=list(q_from_R(cam_R)), fovy=fovy)
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return spec, m, d


def resolve_frames(raw, verbose=True, tol_mm=5e-4):
    """ICP verification of the CAD poses.  Settles at identity for this pack."""
    frames = {k: (np.eye(3), np.zeros(3)) for k in MESH_FILES}
    ref = {"base_mesh": "base_visual", "finger_mesh": "jaw_left_visual",
           "gear_mesh": "gear_visual"}
    err = 0.0
    for it in range(4):
        _, m, d = build(frames, markers=False, cam_markers=False)
        new = dict(frames)
        for mesh, gname in ref.items():
            target = TARGET[gname]
            R_cad = _rot((gname, target))
            p_cad = target[2]
            R_star, t_star = kabsch(world_verts(m, d, gname), raw[mesh] @ R_cad.T + p_cad)
            Rc, tc = frames[mesh]
            new[mesh] = (R_star @ Rc, R_star @ tc + t_star)
        frames = new
        _, m, d = build(frames, markers=False, cam_markers=False)
        err = 0.0
        for gname, target in TARGET.items():
            mesh, _, p_cad = target
            R_cad = _rot((gname, target))
            err = max(err, float(np.abs(world_verts(m, d, gname)
                                        - (raw[mesh] @ R_cad.T + p_cad)).max()))
        if verbose:
            print("iter %d placement error %.6f mm" % (it, err * 1000))
        if err * 1000 < tol_mm:
            break
    return frames, err * 1000.0


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #

def font(px):
    for p in ("/usr/share/fonts/TTF/DejaVuSans.ttf",
              "/usr/share/fonts/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(p, px)
        except Exception:
            pass
    try:
        return ImageFont.load_default(size=px)
    except Exception:
        return ImageFont.load_default()


def render_axis_tiles(m, d, S):
    W = H = 470 * S
    F, FS = font(13 * S), font(10 * S)
    px_per_mm = (H / 2) / (DIST * math.tan(math.radians(22.5))) / 1000.0
    tiles = []
    for label, az, el in VIEWS:
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.lookat[:] = LOOK
        cam.distance = DIST
        cam.azimuth = az
        cam.elevation = el
        r = mujoco.Renderer(m, H, W)
        r.update_scene(d, cam)
        im = Image.fromarray(np.asarray(r.render()))
        r.close()
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, W - 1, 46 * S], fill=(0, 0, 0))
        dr.text((10 * S, 8 * S), label, font=F, fill=(255, 255, 255))
        dr.text((10 * S, 28 * S), "tips 110.600 | base face on origin | camera frustum (pink)",
                font=FS, fill=(200, 220, 255))
        dr.line([14 * S, H - 22 * S, 14 * S + 20 * px_per_mm, H - 22 * S],
                fill=(255, 255, 255), width=2 * S)
        dr.text((14 * S, H - 40 * S), "20 mm", font=FS, fill=(255, 255, 255))
        tiles.append(im)
    return tiles, W, H


def render_camera_view(frames, cam_t, cam_R, S, roll_deg):
    W, H = 1880, 1410
    spec2, m2, d2 = build(frames, markers=False, cam_markers=False,
                          width=W, height=H, cam_t=cam_t, cam_R=cam_R, roll_deg=roll_deg)
    r = mujoco.Renderer(m2, H, W)
    r.update_scene(d2, camera=m2.camera("wrist").id)
    img = Image.fromarray(np.asarray(r.render()))
    r.close()
    drc = ImageDraw.Draw(img)
    F, FS = font(13 * S), font(10 * S)
    half_v = math.atan(CY / FY)
    drc.rectangle([0, 0, W - 1, 46 * S], fill=(0, 0, 0))
    drc.text((10 * S, 8 * S), "CAMERA VIEW  (this camera, own intrinsics)", font=F,
             fill=(255, 255, 255))
    drc.text((10 * S, 28 * S),
             "640x480 @ fx %.1f fy %.1f -> %.2f x %.2f deg | %dx%d render"
             % (FX, FY, 2 * math.degrees(math.atan(CX / FX)),
                2.0 * math.degrees(half_v), W, H),
             font=FS, fill=(255, 180, 240))
    drc.rectangle([0, 0, W - 1, H - 1], outline=(255, 77, 230), width=2 * S)
    return img


def compose_sheet(tiles, camimg, W, H, S, cam_t, roll_deg):
    F, FS = font(13 * S), font(10 * S)
    leg = Image.new("RGB", (W * 4, 44 * S), (18, 18, 18))
    dl = ImageDraw.Draw(leg)
    cam_label = "camera (%.2f,%.2f,%.2f) mm, %g deg roll" % (
        cam_t[0] * 1000.0, cam_t[1] * 1000.0, cam_t[2] * 1000.0, roll_deg)
    items = [((230, 0, 0), "base origin / volume"),
             ((120, 255, 160), "tip plane -110.600"),
             ((255, 77, 230), cam_label),
             ((255, 140, 242), "frustum 160 mm"),
             ((200, 200, 200), "camera view = this camera, own intrinsics")]
    x = 14 * S
    for col, txt in items:
        dl.rectangle([x, 12 * S, x + 14 * S, 12 * S + 14 * S], fill=col, outline=(120, 120, 120))
        dl.text((x + 20 * S, 12 * S), txt, font=FS, fill=(235, 235, 235))
        x += 20 * S + dl.textlength(txt, font=FS) + 22 * S

    sheet = Image.new("RGB", (W * 4, H + 44 * S), (18, 18, 18))
    for i, t in enumerate(tiles):
        sheet.paste(t, (i * W, 0))
    sheet.paste(camimg, (3 * W, 0))
    sheet.paste(leg, (0, H))
    return sheet


# --------------------------------------------------------------------------- #
# Configuration export
# --------------------------------------------------------------------------- #

def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _bbox(pts):
    lo, hi = pts.min(0), pts.max(0)
    return {
        "min_mm": [round(float(x) * 1000.0, 4) for x in lo],
        "max_mm": [round(float(x) * 1000.0, 4) for x in hi],
        "size_mm": [round(float(x) * 1000.0, 4) for x in (hi - lo)],
        "center_mm": [round(float(x) * 1000.0, 4) for x in (lo + hi) / 2],
    }


def git_rev():
    try:
        return subprocess.run(["git", "-C", REPO, "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None


def export_config(frames, residual_mm, cam_t, cam_R, roll_deg, raw, m, d):
    meta = camera_meta(cam_t, cam_R, roll_deg)

    placements = {}
    bodies = {"base_visual": "parallel_base",
              "jaw_left_visual": "jaw_left",
              "jaw_right_visual": "jaw_right",
              "gear_visual": "gear"}
    for gname, body_name in bodies.items():
        mesh, _, p_cad = TARGET[gname]
        R_cad = _rot((gname, TARGET[gname]))
        gid = m.geom(gname).id
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
        R_world = np.array(d.geom_xmat[gid]).reshape(3, 3)
        placements[gname.replace("_visual", "")] = {
            "mesh": mesh,
            "body": body_name,
            "body_pos_tool_mm": [round(float(x) * 1000.0, 4) for x in d.xpos[bid]],
            "body_quat_wxyz": [float(x) for x in d.xquat[bid]],
            "rotation_tool": [[round(float(v), 9) for v in row] for row in R_world],
            "body_pos_in_parent_mm": [round(float(x) * 1000.0, 4) for x in m.body_pos[bid]],
            "body_quat_in_parent_wxyz": [float(x) for x in m.body_quat[bid]],
            "cad_rotation": [[round(float(v), 9) for v in row] for row in R_cad],
            "cad_offset_mm": [round(float(x) * 1000.0, 4) for x in p_cad],
        }

    base_bb = _bbox(world_verts(m, d, "base_visual"))
    jl = world_verts(m, d, "jaw_left_visual")
    jr = world_verts(m, d, "jaw_right_visual")
    gear = world_verts(m, d, "gear_visual")
    jaw_all = np.vstack([jl, jr])
    pct, pct_detail = frustum_pct(jaw_all, cam_t, cam_R)

    return {
        "schema": "b5_reference_config/1",
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "generated_by": "tools/b5_reference_scene.py",
        "git_rev": git_rev(),
        "units": "metres for *_m, millimetres for *_mm, degrees for *_deg",
        "frames": {
            "tool_mount": ("Scene world. Origin on the tool mount face, +z away from the arm; "
                           "the base mount face was moved +%.2f mm onto z = 0. "
                           "Jaw tips at z = %.3f mm." % (MOUNT_FACE_OFFSET_MM, TIP_Z * 1000.0)),
            "body": "Parent body frame; parallel_base hosts both jaws and the gear.",
            "cad": "The mesh's own STL vertex frame.",
        },
        "mount_face_offset_mm": MOUNT_FACE_OFFSET_MM,
        "meshes": {k: {"path": os.path.relpath(v, REPO), "md5": md5(v),
                       "unique_vertices": int(len(raw[k]))}
                   for k, v in MESH_FILES.items()},
        "placements": placements,
        "tip_depth_mm": {
            "accepted_tip_plane_z_mm": round(TIP_Z * 1000.0, 4),
            "measured_jaw_min_z_mm": round(float(jaw_all[:, 2].min()) * 1000.0, 4),
            "measured_jaw_tip_depth_mm": round(float(-jaw_all[:, 2].min()) * 1000.0, 4),
        },
        "camera": meta,
        "measured": {
            "base_bbox": base_bb,
            "jaw_left_bbox": _bbox(jl),
            "jaw_right_bbox": _bbox(jr),
            "jaw_combined_bbox": _bbox(jaw_all),
            "gear_bbox": _bbox(gear),
            "tip_plane_z_mm": round(TIP_Z * 1000.0, 4),
        },
        "verification": {
            "placement_residual_mm": round(residual_mm, 9),
            "jaw_in_frustum_pct": round(pct, 4),
            "jaw_in_frustum_detail": pct_detail,
        },
    }


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("sheet", "axis", "camera", "config"), default="sheet")
    ap.add_argument("--roll", type=float, default=180.0,
                    help="roll about the optical axis, degrees (accepted image: 180)")
    ap.add_argument("--scale", type=int, default=4, help="pixel scale factor S (default 4)")
    ap.add_argument("--out", default=None, help="PNG path (default reports/b5_reference_<mode>.png)")
    ap.add_argument("--json-out", default=os.path.join(REPO, "reports/b5_reference_config.json"))
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    missing = [p for p in MESH_FILES.values() if not os.path.isfile(p)]
    if missing:
        raise SystemExit("missing mesh file(s): " + ", ".join(missing))

    S = args.scale
    raw = {k: raw_ordered(v) for k, v in MESH_FILES.items()}
    frames, residual_mm = resolve_frames(raw, verbose=not args.quiet)
    cam_t, cam_R = camera_pose(args.roll)

    if not args.quiet:
        print("camera pos mm %s" % np.round(cam_t * 1000.0, 4))
        print("camera axis  %s  right %s  up %s" % tuple(
            np.round(v, 6) for v in (-cam_R[:, 2], cam_R[:, 0], cam_R[:, 1])))

    if args.mode != "config":
        _, m, d = build(frames, cam_t=cam_t, cam_R=cam_R, roll_deg=args.roll)
        default_name = {"sheet": "4up", "axis": "axis_3up", "camera": "camera_view"}[args.mode]
        out = args.out or os.path.join(REPO, "reports/b5_reference_%s.png" % default_name)
        if args.mode == "axis":
            tiles, W, H = render_axis_tiles(m, d, S)
            sheet = Image.new("RGB", (W * 3, H), (18, 18, 18))
            for i, t in enumerate(tiles):
                sheet.paste(t, (i * W, 0))
            sheet.save(out)
            print("wrote %s %s" % (out, sheet.size))
        elif args.mode == "camera":
            img = render_camera_view(frames, cam_t, cam_R, S, args.roll)
            img.save(out)
            print("wrote %s %s" % (out, img.size))
        else:
            tiles, W, H = render_axis_tiles(m, d, S)
            camimg = render_camera_view(frames, cam_t, cam_R, S, args.roll)
            sheet = compose_sheet(tiles, camimg, W, H, S, cam_t, args.roll)
            sheet.save(out)
            print("wrote %s %s" % (out, sheet.size))
    else:
        _, m, d = build(frames, markers=False, cam_markers=False,
                        cam_t=cam_t, cam_R=cam_R, roll_deg=args.roll)

    if args.json_out:
        cfg = export_config(frames, residual_mm, cam_t, cam_R, args.roll, raw, m, d)
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
        with open(args.json_out, "w") as fh:
            json.dump(cfg, fh, indent=2, sort_keys=False)
            fh.write("\n")
        print("wrote %s" % args.json_out)
        print("jaw_in_frustum_pct %.4f  base bbox %s .. %s mm"
              % (cfg["verification"]["jaw_in_frustum_pct"],
                 cfg["measured"]["base_bbox"]["min_mm"],
                 cfg["measured"]["base_bbox"]["max_mm"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
