"""Block 1: orthogonal + wrist views of the tool at marker14, beside the IRL frame."""
import sys, json
from pathlib import Path
import numpy as np, mujoco
from PIL import Image, ImageDraw

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT))
from chess_sim.config import EnvConfig, Camera
from chess_sim.env import ChessSimEnv

rec = next(r for r in json.loads((ROOT/'reports/block5m_frames.json').read_text())
           if r['pose'] == 'marker14')
J = rec['joints_deg']
env = ChessSimEnv(EnvConfig())
m, d = env.model, env.data
d.qpos[env._joint_qpos[:5]] = np.radians([J[k] for k in
    ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')])
d.qpos[env._joint_qpos[5]] = np.radians(J['gripper'])
mujoco.mj_forward(m, d)

gid = m.body('so101:gripper').id
lookat = d.xpos[gid] + d.xmat[gid].reshape(3, 3) @ np.array([0, 0, -0.070])
DIST = 0.22

views = [('front  az 0   el 0', 0, 0), ('side  az 90  el 0', 90, 0),
         ('top   az 0   el -89', 0, -89), ('iso   az 35  el -25', 35, -25)]
tiles = [(lab, env.render(orbit={'lookat': lookat, 'distance': DIST,
                                 'azimuth': az, 'elevation': el})) for lab, az, el in views]
wrist = env.render(camera=Camera.WRIST)
irl = Image.open(ROOT / 'live/captures/scan_5m_marker14.jpg').convert('RGB')
w, h = tiles[0][1].shape[1], tiles[0][1].shape[0]
irl = irl.resize((w, h))

sheet = Image.new('RGB', (w * 3, h * 2), (16, 16, 18))
dr = ImageDraw.Draw(sheet)
cells = tiles + [('wrist camera (4c mount)', wrist)]
for i, (lab, im) in enumerate(cells):
    x, y = (i % 3) * w, (i // 3) * h
    sheet.paste(Image.fromarray(im), (x, y))
    dr.text((x + 8, y + 8), lab, fill=(255, 255, 120))
sheet.paste(irl, (2 * w, h))
dr.text((2 * w + 8, h + 8), 'IRL wrist - scan_5m_marker14.jpg', fill=(255, 255, 120))
sheet.save(ROOT / 'reports/b1_orientation_ortho.png')

Image.fromarray(wrist).save(ROOT / 'reports/b1_orientation_wrist.png')
wide = env.render(orbit={'lookat': lookat, 'distance': 0.45, 'azimuth': 35, 'elevation': -20})
Image.fromarray(wide).save(ROOT / 'reports/b1_orientation_marker14.png')

pair = Image.new('RGB', (w * 2, h), (16, 16, 18))
pair.paste(Image.fromarray(wrist), (0, 0))
pair.paste(irl, (w, 0))
d2 = ImageDraw.Draw(pair)
d2.text((8, 8), 'sim wrist (4c mount)', fill=(255, 255, 120))
d2.text((w + 8, 8), 'IRL wrist - scan_5m_marker14.jpg', fill=(255, 255, 120))
pair.save(ROOT / 'reports/b1_orientation_vs_irl.png')
print('wrote reports/b1_orientation_{ortho,wrist,marker14,vs_irl}.png')
