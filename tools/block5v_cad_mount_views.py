#!/usr/bin/env python3
"""Block 5v: add the CAD camera mount as a visual geom and render external views."""
import json, math, sys
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
import mujoco
import trimesh

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "live"))
from chess_sim.product import layout_env_config
from chess_sim.env import ChessSimEnv
from chess_sim.wrist_calibration import WristCalibration

SRC = ROOT / "hardware/onshape-camera-mount-2026-11-18/Camera_Mount_v3.stl"
BIN = ROOT / ".private/camera_mount_v3_bin.stl"
POS = [-0.0000208, -0.0094625, 0.0514519]
QUAT = [0.0, 0.707107, 0.0, -0.707107]   # (w,x,y,z)

m = trimesh.load(str(SRC), process=False)
m.export(str(BIN), file_type="stl")
print("mesh", len(m.vertices), "verts", len(m.faces), "faces ->", BIN)

frames = json.loads((ROOT / "reports/block5m_frames.json").read_text())
rec = next(r for r in frames if r["pose"] == "marker14")
NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
arm = [math.radians(rec["joints_deg"][n]) for n in NAMES]

cfg = layout_env_config("printed", cameras=())
cfg = cfg.model_copy(update={"board": cfg.board.model_copy(update={"face": "charuco"})})
env = ChessSimEnv(cfg)
env.reset("8/8/8/8/8/8/8/8")
env.data.qpos[env._joint_qpos] = np.asarray(arm)
mujoco.mj_forward(env.model, env.data)
q = env.data.qpos.copy()

# add the CAD camera mount as a VISUAL geom on the gripper body
mesh_asset = env.spec.add_mesh(name="camera_mount_v3", file=str(BIN))
body = env.spec.body("so101:gripper")
body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="camera_mount_v3",
              pos=POS, quat=QUAT, contype=0, conaffinity=0, group=2,
              rgba=[0.12, 0.12, 0.14, 1.0], name="cad_camera_mount")
env.model = env.spec.compile()
env.data = mujoco.MjData(env.model)
env.data.qpos[:] = q
mujoco.mj_forward(env.model, env.data)

gid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "so101:gripper")
tool = env.data.xpos[gid].copy()
gid_geom = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "cad_camera_mount")
cam_box = env.data.geom_xpos[gid_geom].copy()
cal = WristCalibration.load(str(ROOT / ".private/phase11d/calibration.json"))
wrist_cam = np.asarray(cal.camera(tuple(arm)).camera_position)
print("gripper body mm:", np.round(tool * 1000, 1))
print("CAD mount geom origin mm:", np.round(cam_box * 1000, 1))
print("sim wrist camera mm:", np.round(wrist_cam * 1000, 1))
print("CAD-geom -> sim camera offset mm:", np.round((wrist_cam - cam_box) * 1000, 1),
      "|d| =", round(float(np.linalg.norm(wrist_cam - cam_box)) * 1000, 1))
camid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
print("mj wrist camera mm:", np.round(env.data.cam_xpos[camid] * 1000, 1),
      " vs calibration:", round(float(np.linalg.norm(env.data.cam_xpos[camid] - wrist_cam)) * 1000, 2), "mm apart")

renderer = mujoco.Renderer(env.model, 360, 480)
views = [("side", 0.0, 12.0), ("front", 90.0, 12.0), ("other", 180.0, 12.0), ("lower", 200.0, -35.0)]
tiles = []
for name, az, el in views:
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = tool
    cam.distance = 0.32
    cam.azimuth = az
    cam.elevation = el
    renderer.update_scene(env.data, cam)
    tiles.append((name, Image.fromarray(renderer.render())))
renderer.update_scene(env.data, camid)
tiles.append(("wrist (4c lens)", Image.fromarray(renderer.render())))
renderer.close()

W, H = 480, 360
sheet = Image.new("RGB", (W * 3, H * 2), (10, 10, 10))
d = ImageDraw.Draw(sheet)
for i, (name, img) in enumerate(tiles):
    x, y = (i % 3) * W, (i // 3) * H
    sheet.paste(img.resize((W, H)), (x, y))
    d.rectangle([x, y, x + W - 1, y + 16], fill=(0, 0, 0))
    d.text((x + 4, y + 2), name, fill=(255, 255, 255))
out = ROOT / "reports/tool_external_views_updated.png"
sheet.save(out)
print("wrote", out)
env.close()
