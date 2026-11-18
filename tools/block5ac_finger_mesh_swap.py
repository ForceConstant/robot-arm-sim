#!/usr/bin/env python3
"""Block 5y: UI finger as the moving jaws + merged base, at the assembly transforms."""
import json, math, shutil, sys
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

FINGER = ROOT / "hardware/onshape-fingers-2026-11-18/Finger_Jaw_UI_m.stl"
FINGER_BIN = ROOT / ".private/finger_jaw_ui_bin.stl"
BASE = ROOT / "hardware/onshape-fixed-jaw-2026-11-18/Complete_Fixed_Jaw.stl"
BASE_BIN = ROOT / ".private/complete_fixed_jaw_bin.stl"
PACK = ROOT / ".private/gripper-assets-5y"

for src, dst in ((FINGER, FINGER_BIN), (BASE, BASE_BIN)):
    m = trimesh.load(str(src), process=False)
    m.export(str(dst), file_type="stl")
    print("mesh", src.name, len(m.vertices), "verts", len(m.faces), "faces")

if not PACK.exists():
    shutil.copytree(ROOT / ".private/gripper-assets-v3", PACK)
shutil.copy2(FINGER_BIN, PACK / "assets/gripper_jaw_1__solid1.stl")

LEFT = dict(pos=[0.023376, 0.00015, -0.07645], quat=[1.0, 0, 0, 0])
RIGHT = dict(pos=[-0.023377, 0.00045, -0.07645], quat=[0.0, 0, 0, 1.0])

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

spec = env.spec
# merged base + camera mount at the tool mount
spec.add_mesh(name="complete_fixed_jaw", file=str(BASE_BIN))
spec.body("so101:gripper").add_geom(
    type=mujoco.mjtGeom.mjGEOM_MESH, meshname="complete_fixed_jaw",
    pos=[0.009462492872, 0.023429161181, -0.06257690676747059],
    quat=[0.707107, 0, 0, 0.707107], contype=0, conaffinity=0, group=2,
    rgba=[0.15, 0.15, 0.17, 1.0], name="cad_complete_fixed_jaw")
# jaws: unchanged pack poses; the mesh file is the only geometry change
# hide the old camera housing proxy and the pack's camera-mount geoms
try:
    spec.delete(spec.body("so101:camera_housing"))
    print("camera_housing body removed")
except Exception as exc:  # noqa: BLE001
    print("housing body not removed:", exc)
removed = []
try:
    base_body = spec.body("so101:cad:parallel_base")
    for geom in list(base_body.geoms):
        removed.append(geom.name or "unnamed-base-geom")
        spec.delete(geom)
except Exception as exc:  # noqa: BLE001
    print("parallel_base not pruned:", exc)
for geom in list(spec.geoms):
    gname = geom.name or ""
    if gname == "so101:camera_collision" or "camera_mount" in gname:
        removed.append(gname)
        spec.delete(geom)
print("camera geoms removed:", removed)

env.model = spec.compile()
env.data = mujoco.MjData(env.model)
env.data.qpos[:] = q
mujoco.mj_forward(env.model, env.data)
print("nbody", env.model.nbody, "ngeom", env.model.ngeom)

gid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "so101:gripper")
tool = env.data.xpos[gid].copy()
cal = WristCalibration.load(str(ROOT / ".private/phase11d/calibration.json"))
wrist_cam = np.asarray(cal.camera(tuple(arm)).camera_position)
print("gripper mm", np.round(tool*1000, 1), "lens mm", np.round(wrist_cam*1000, 1))

renderer = mujoco.Renderer(env.model, 360, 480)
tiles = []
for name, az, el in (("side", 0.0, 12.0), ("front", 90.0, 12.0),
                     ("other", 180.0, 12.0), ("lower", 200.0, -35.0)):
    cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = tool; cam.distance = 0.32; cam.azimuth = az; cam.elevation = el
    renderer.update_scene(env.data, cam)
    tiles.append((name, Image.fromarray(renderer.render())))
camid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
renderer.update_scene(env.data, camid)
tiles.append(("wrist (4c lens)", Image.fromarray(renderer.render())))
renderer.close()

W, H = 480, 360
sheet = Image.new("RGB", (W*3, H*2), (10, 10, 10))
d = ImageDraw.Draw(sheet)
for i, (name, img) in enumerate(tiles):
    x, y = (i % 3)*W, (i // 3)*H
    sheet.paste(img.resize((W, H)), (x, y))
    d.rectangle([x, y, x+W-1, y+16], fill=(0, 0, 0))
    d.text((x+4, y+2), name, fill=(255, 255, 255))
out = ROOT / "reports/tool_external_views_5z3.png"
sheet.save(out)
print("wrote", out)
env.close()
