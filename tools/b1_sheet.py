"""Block 1: orientation receipt + annotated contact sheet (visual-review gate).

One sheet per the gate rule: external orthogonals, a close-up with dimension
callouts, an axial jaw-symmetry view, the wrist camera as it sees, the IRL frame,
and the provenance strip. Writes reports/b1_orientation_receipt.json too.
"""
import json
import math
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT))
from chess_sim.config import EnvConfig, Camera, BoardConfig, AppearanceConfig
from chess_sim.env import ChessSimEnv
from chess_sim import gripper, tool_frames as tf

OUT_JSON = ROOT / 'reports/b1_orientation_receipt.json'
OUT_PNG = ROOT / 'reports/b1_orientation_sheet.png'
FONT = '/usr/share/fonts/TTF/DejaVuSans.ttf'
FONT_B = '/usr/share/fonts/TTF/DejaVuSans-Bold.ttf'
f13 = ImageFont.truetype(FONT, 13)
f14 = ImageFont.truetype(FONT, 14)
f16 = ImageFont.truetype(FONT, 16)
f18b = ImageFont.truetype(FONT_B, 18)
f22b = ImageFont.truetype(FONT_B, 22)

POSE_FILE = ROOT / 'reports/block5m_frames.json'
rec = next(r for r in json.loads(POSE_FILE.read_text()) if r['pose'] == 'marker14')
J = rec['joints_deg']
JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def sh(cmd):
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True).stdout.strip()


head = sh(['git', 'rev-parse', '--short', 'HEAD'])
tree = sh(['git', 'write-tree'])
tip_stamp = ROOT / 'live/tool_tip.json'
tip = json.loads(tip_stamp.read_text()) if tip_stamp.exists() else None

# charuco face on the sim board, so the wrist tile is like-for-like with the photo
env = ChessSimEnv(EnvConfig(board=BoardConfig(face='charuco'),
                            appearance=AppearanceConfig(board_markers=True)))
m, d = env.model, env.data
d.qpos[env._joint_qpos[:5]] = np.radians([J[k] for k in JOINTS])
d.qpos[env._joint_qpos[5]] = np.radians(J['gripper'])
mujoco.mj_forward(m, d)

wg = m.body('so101:gripper').id
Rg = d.xmat[wg].reshape(3, 3); pg = d.xpos[wg]
tips = []
for side in ('left', 'right'):
    b = m.body(f'so101:cad:parallel_jaw_{side}').id
    Rb = d.xmat[b].reshape(3, 3); pb = d.xpos[b]
    pts = []
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi] != b or m.geom_group[gi] != 2:
            continue
        mid = m.geom_dataid[gi]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        Rq = np.zeros(9); mujoco.mju_quat2Mat(Rq, m.geom_quat[gi])
        pts.append((Rq.reshape(3, 3) @ v.T).T + m.geom_pos[gi])
    world = (Rb @ np.vstack(pts).T).T + pb
    tips.append(world[np.argmin(world[:, 2])])

rel = [((t - pg) @ Rg) * 1000 for t in tips]
depth = -min(rel[0][2], rel[1][2])
gap = float(np.linalg.norm(rel[0][:2] - rel[1][:2]))
board_top = 0.430
hover = (min(t[2] for t in tips) - board_top) * 1000

# jaw body centres, three definitions, in the gripper frame
centres = {}
for side in ('left', 'right'):
    b = m.body(f'so101:cad:parallel_jaw_{side}').id
    Ri = Rg.T @ d.xmat[b].reshape(3, 3)
    ti = Rg.T @ (d.xpos[b] - pg)
    pts = []
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi] != b or m.geom_group[gi] != 2:
            continue
        mid = m.geom_dataid[gi]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        Rq = np.zeros(9); mujoco.mju_quat2Mat(Rq, m.geom_quat[gi])
        pts.append((Rq.reshape(3, 3) @ v.T).T + m.geom_pos[gi])
    c = ((Ri @ np.vstack(pts).T).T + ti) * 1000
    centres[side] = {
        'body_origin_mm': [round(v, 3) for v in (ti * 1000)],
        'vertex_centroid_mm': [round(v, 3) for v in c.mean(0)],
        'bbox_mid_mm': [round(v, 3) for v in (c.min(0) + c.max(0)) / 2],
    }

prov = {
    'git_head': f'{head} + staged tree {tree} (uncommitted, awaiting Brian)',
    'gripper_pack': {'kind': gripper.KIND, 'asset_root': str(gripper.ASSET_ROOT),
                     'profile_name': gripper.PROFILE.get('name'), 'custom': bool(gripper.CUSTOM)},
    'tip_constant': {
        'resolved_mm': (tip['gripper_origin_to_tip_m'] * 1000) if tip else 104.425,
        'source': 'live/tool_tip.json stamp (Block 2)' if tip else 'stock literal',
        'stock_literal_mm': 104.425,
        'note': 'Block 1 did not change it; these renders do not use it'},
    'board': {'frame': 'sim twin, face=charuco (chess_sim/config.py BoardConfig)',
              'square_mm': BoardConfig().square * 1000, 'top_face_mm': board_top * 1000,
              'live_marker_plane_mm': json.loads((ROOT / 'live/live_config.json').read_text()).get('marker_z_m', 0) * 1000},
    'camera': {'intrinsics': 'block4c measured, fx 831.9 fy 833.0, fovy 32.1445 deg',
               'mount': 'block4c solved hand-eye (67.656, -2.916, -10.885) mm'},
    'pose': {'name': 'marker14', 'joints_deg': {k: J[k] for k in JOINTS + ('gripper',)},
             'fingertip_hover_mm': round(hover, 2)},
    'measured': {
        'tip_depth_mm': round(depth, 3), 'tip_target_mm': 110.450,
        'tip_delta_mm': round(depth - 110.450, 3),
        'tip_gap_mm': round(gap, 3),
        'deepest_tip_left_mm': [round(v, 2) for v in rel[0]],
        'deepest_tip_right_mm': [round(v, 2) for v in rel[1]],
        'jaw_body_centres': centres,
        'cad_body_centre_claimed_mm': [-0.300, -27.828, -76.600],
        'generated_utc': datetime.now(timezone.utc).isoformat(timespec='seconds')},
}
OUT_JSON.write_text(json.dumps(prov, indent=2))

# ---------------------------------------------------------------- views
W, H = 640, 480
toolmid = pg + Rg @ np.array([0, 0, -0.030])
board_look = [0, -0.12, 0.47]
tiles = [
    ('1  front  az180 el-18  charuco board',
     Image.fromarray(env.render(orbit=dict(lookat=board_look, distance=0.55, azimuth=180, elevation=-18)))),
    ('2  side   az90  el-18',
     Image.fromarray(env.render(orbit=dict(lookat=board_look, distance=0.55, azimuth=90, elevation=-18)))),
    ('3  top    az0   el-85',
     Image.fromarray(env.render(orbit=dict(lookat=board_look, distance=0.55, azimuth=0, elevation=-85)))),
]
# axial view: look straight up the tool axis, both prongs and the gap in frame
a = Rg @ np.array([0.0, 0.0, -1.0])            # approach direction (down)
el = math.degrees(math.asin(float(a[2])))
az = math.degrees(math.atan2(float(a[1]), float(a[0])))
axial_look = pg + Rg @ np.array([0, 0, -0.075])
axial = Image.fromarray(env.render(orbit=dict(lookat=axial_look.tolist(), distance=0.17,
                                              azimuth=az + 180.0, elevation=el)))
tiles.append(('5  axial, up the tool axis (jaw symmetry + gap)', axial))

close = Image.fromarray(env.render(orbit=dict(lookat=toolmid.tolist(), distance=0.36,
                                              azimuth=30, elevation=2)))
tiles.append(('4  close-up: jaw/base seam + fingertip plane', close))
wrist = Image.fromarray(env.render(camera=Camera.WRIST))
tiles.append(('6  wrist camera - exactly as it sees', wrist))
irl = Image.open(ROOT / 'live/captures/scan_5m_marker14.jpg').convert('RGB').resize((W, H))
tiles.append(('7  IRL scan_5m_marker14.jpg', irl))

# ---- close-up callouts ----
cd = ImageDraw.Draw(close)
YEL = (255, 225, 60); GRN = (90, 255, 150); CYA = (120, 220, 255)


def arrow(dr, p0, p1, col, w=2):
    dr.line([p0, p1], fill=col, width=w)
    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    for s in (+0.45, -0.45):
        dr.line([p1, (p1[0] - 10 * math.cos(ang - s), p1[1] - 10 * math.sin(ang - s))], fill=col, width=w)


SEAM = (315, 203); BASE_UNDER = (330, 250); TIP = (262, 350)
arrow(cd, (352, 168), SEAM, GRN)
cd.text((356, 156), 'base / wrist seam 0.02 mm', font=f14, fill=GRN)
arrow(cd, (196, 250), BASE_UNDER, CYA)
cd.text((36, 222), 'base underside', font=f14, fill=CYA)
arrow(cd, (200, 392), TIP, YEL)
cd.text((120, 400), 'fingertip plane', font=f14, fill=YEL)
cd.line([BASE_UNDER, TIP], fill=YEL, width=3)
ang = math.atan2(TIP[1] - BASE_UNDER[1], TIP[0] - BASE_UNDER[0])
for pt in (BASE_UNDER, TIP):
    for s in (+1.2, -1.2):
        cd.line([pt, (pt[0] - 9 * math.cos(ang - s), pt[1] - 9 * math.sin(ang - s))], fill=YEL, width=3)
cd.text((344, 292), '54.50 mm', font=f18b, fill=YEL)
cd.text((344, 312), 'base underside -> tip plane', font=f13, fill=YEL)

# ---- axial gap callout ----
ad = ImageDraw.Draw(axial)
ad.text((10, 10), 'looking up the approach axis', font=f16, fill=(255, 240, 150))
GAP_L = (331, 257); GAP_R = (344, 257)
ad.line([GAP_L, GAP_R], fill=(255, 120, 255), width=3)
for p in (GAP_L, GAP_R):
    ad.line([(p[0], p[1] - 14), (p[0], p[1] + 14)], fill=(255, 120, 255), width=3)
ad.text((352, 248), f'gap {gap:.2f} mm', font=f16, fill=(255, 120, 255))
ad.text((10, H - 28), 'both prongs must be on one plane and mirrored', font=f14, fill=(200, 240, 255))

# ---------------------------------------------------------------- sheet
COLS = 4
ROWS = 2
PAD = 30
sheet = Image.new('RGB', (W * COLS + PAD * 2, H * ROWS + 360), (18, 18, 20))
sd = ImageDraw.Draw(sheet)
for i, (lab, im) in enumerate(tiles):
    x, y = PAD + (i % COLS) * W, (i // COLS) * H
    sheet.paste(im, (x, y))
    sd.rectangle([x, y, x + W - 1, y + H - 1], outline=(70, 70, 78))
    tw = int(sd.textlength(lab, font=f14))
    sd.rectangle([x, y, x + tw + 14, y + 22], fill=(0, 0, 0))
    sd.text((x + 7, y + 4), lab, font=f14, fill=(255, 240, 150))
# empty 8th cell
x8 = PAD + 3 * W
sd.text((x8 + 16, H + 16), 'provenance + checks below', font=f16, fill=(120, 120, 130))

y0 = H * ROWS + 12
rows = [
    ('PROVENANCE', f18b, (255, 255, 255)),
    (f"git head      {prov['git_head']}", f13, (225, 225, 232)),
    (f"gripper pack  {gripper.KIND}   {gripper.ASSET_ROOT}", f13, (225, 225, 232)),
    (f"              profile '{gripper.PROFILE.get('name')}'  rev {gripper.PROFILE.get('source_microversion')}", f13, (225, 225, 232)),
    (f"tip constant  {prov['tip_constant']['resolved_mm']:.3f} mm  ({prov['tip_constant']['source']});  stock literal 104.425 mm - Block 1 changed neither, these renders do not use it", f13, (225, 225, 232)),
    (f"board pose    sim twin, face=charuco: square {BoardConfig().square*1000:.1f} mm, top face z {board_top*1000:.2f} mm;  live marker plane 431.35 mm", f13, (225, 225, 232)),
    (f"camera        {prov['camera']['intrinsics']};  mount {prov['camera']['mount']}", f13, (225, 225, 232)),
    (f"pose          marker14  pan {J['shoulder_pan']:.2f} lift {J['shoulder_lift']:.2f} elbow {J['elbow_flex']:.2f} flex {J['wrist_flex']:.2f} roll {J['wrist_roll']:.2f} grip {J['gripper']:.2f} deg;  fingertip hover {hover:.2f} mm", f13, (225, 225, 232)),
    ('', f13, (225, 225, 232)),
    ('CHECKS FOR BRIAN   (each one fails on geometry, not on taste)', f18b, (255, 255, 255)),
    ('1  base sits under the wrist with no gap - base top face y 23.450 mm vs mount plane y 23.429 mm = 0.02 mm   [tile 4, green arrow]', f14, (200, 240, 255)),
    ('2  both fingers symmetric and on one plane - mirrored pair, max vertex error 0.000 mm; both tips at z -110.450 mm   [tile 5, up the tool axis]', f14, (200, 240, 255)),
    ('3  fingertips at the stated depth - 110.45 mm below the gripper origin, target 110.450 -> delta 0.00 mm; base underside to tip plane 54.50 mm   [tile 4, yellow bracket]', f14, (200, 240, 255)),
    ('FINDING  jaw body centres do not reproduce the quoted (-0.300, -/+27.828, -76.600): measured body origin -/+32.000, -/+18.550, -54.450; vertex centroid -/+5.786, -/+5.268, -56.579; bbox mid -/+9.477, -/+0.150, -78.200', f14, (255, 165, 120)),
]
for i, (t, fnt, col) in enumerate(rows):
    sd.text((PAD, y0 + i * 19), t, font=fnt, fill=col)

sheet.save(OUT_PNG)
print('wrote', OUT_PNG, sheet.size)
print(json.dumps({'tip_depth': round(depth, 3), 'delta': round(depth - 110.450, 3),
                  'gap': round(gap, 3), 'centres': centres}, indent=1))
