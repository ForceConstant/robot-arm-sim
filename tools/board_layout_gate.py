"""Board-layout visual gate (Step 1.4).

Renders the 244 mm printed board twin with the arm at reset, plus the wrist
camera, and prints provenance onto the sheet: git revision, board/layout
numbers, base-mesh front datum, tool pack, tip constant and camera model.

Read-only: builds the sim, resets an empty board, renders. No controller runs,
no bridge, no IRL.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from chess_sim.config import EnvConfig, BoardConfig, AppearanceConfig, Camera
from chess_sim.env import ChessSimEnv
from chess_sim import gripper, scene

OUT = ROOT / 'reports/board_layout_gate'
OUT.mkdir(parents=True, exist_ok=True)
FONT = '/usr/share/fonts/TTF/DejaVuSans.ttf'
FONT_B = '/usr/share/fonts/TTF/DejaVuSans-Bold.ttf'
f13 = ImageFont.truetype(FONT, 13)
f14 = ImageFont.truetype(FONT, 14)
f16 = ImageFont.truetype(FONT, 16)
f18b = ImageFont.truetype(FONT_B, 18)


def sh(cmd):
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True).stdout.strip()


head = sh(['git', 'rev-parse', '--short', 'HEAD'])
dirty = [ln for ln in sh(['git', 'status', '--porcelain']).splitlines() if ln.strip()]
modified = [ln for ln in dirty if not ln.startswith('??')]
untracked = [ln for ln in dirty if ln.startswith('??')]
tip = json.loads((ROOT / 'live/tool_tip.json').read_text()) if (ROOT / 'live/tool_tip.json').exists() else None

board = BoardConfig(face='charuco')
env = ChessSimEnv(EnvConfig(board=board, appearance=AppearanceConfig(board_markers=True)))
env.reset('8/8/8/8/8/8/8/8')
m, d = env.model, env.data
mujoco.mj_forward(m, d)

# ---- measured board/mount numbers from the built model -------------------
base_id = m.body('so101:base').id
Rb = d.xmat[base_id].reshape(3, 3)
pb = d.xpos[base_id]
pts = []
for gi in range(m.ngeom):
    if m.geom_bodyid[gi] != base_id or m.geom_group[gi] != 2:
        continue
    mid = m.geom_dataid[gi]
    if mid < 0:
        continue
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    Rq = np.zeros(9)
    mujoco.mju_quat2Mat(Rq, m.geom_quat[gi])
    pts.append((Rq.reshape(3, 3) @ v.T).T + m.geom_pos[gi])
P = np.vstack(pts)
world = (Rb @ P.T).T + pb

arm_base_y = float(pb[1]) * 1000.0
board_edge_y = (env.board.origin[1] - env.board.width / 2) * 1000.0
mesh_front_y = float(world[:, 1].max()) * 1000.0
mesh_gap_mm = board_edge_y - mesh_front_y

cam_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, str(Camera.WRIST))
cam_fovy = float(m.cam_fovy[cam_id])
cam_pos_world = [round(float(v) * 1000, 3) for v in d.cam_xpos[cam_id]]
cam_quat = [round(float(v), 6) for v in m.cam_quat[cam_id]]

prov = {
    'generated_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
    'git_head': head,
    'git_modified_files': modified,
    'git_untracked_count': len(untracked),
    'board': {
        'face': board.face,
        'square_mm': board.square * 1000,
        'border_mm': board.border * 1000,
        'field_mm': board.field * 1000,
        'overall_mm': board.width * 1000,
        'half_mm': board.width * 500,
        'thickness_mm': board.thickness * 1000,
        'top_z_mm': round(env.board.top * 1000, 3),
    },
    'mount': {
        'arm_base_frame_mm': [round(v * 1000, 3) for v in env.board.arm_base],
        'board_edge_to_base_frame_mm': board.arm_gap * 1000,
        'base_mesh_front_past_origin_mm': round(mesh_front_y - arm_base_y, 3),
        'board_edge_y_mm': round(board_edge_y, 3),
        'base_mesh_front_y_mm': round(mesh_front_y, 3),
        'mesh_front_to_board_edge_mm': round(mesh_gap_mm, 3),
    },
    'tool': {
        'kind': gripper.KIND,
        'asset_root': str(gripper.ASSET_ROOT),
        'profile_name': gripper.PROFILE.get('name'),
        'source_microversion': gripper.PROFILE.get('source_microversion'),
    },
    'tip_constant': {
        'stamp_pack': tip.get('pack') if tip else None,
        'stamp_asset_root': tip.get('asset_root') if tip else None,
        'stamp_tip_mm': round(tip['gripper_origin_to_tip_m'] * 1000, 3) if tip else None,
        'stamp_git_head': tip.get('git_head') if tip else None,
        'stamp_asset_root_exists': bool(tip and Path(tip.get('asset_root', '')).exists()),
    },
    'camera': {
        'source': scene.CAMERA_SOURCE,
        'name': str(Camera.WRIST),
        'fovy_deg_actual': cam_fovy,
        'world_pos_mm': cam_pos_world,
        'quat': cam_quat,
        'import_banner': f"resolved fovy {scene.wrist_camera_fovy():.4f} deg (banner and model share one function)",
    },
    'pose': 'reset, empty board 8/8/8/8/8/8/8/8 (no controller run)',
}
(OUT / 'board_layout_provenance.json').write_text(json.dumps(prov, indent=2))

# ---- render ---------------------------------------------------------------
W, H = 640, 480
fovy_vis = float(m.vis.global_.fovy)


def scale_px_per_mm(distance_m: float) -> float:
    return (H / (2.0 * distance_m * math.tan(math.radians(fovy_vis / 2.0)))) / 1000.0


tiles = []

top_dist = 0.75
top = env.render(orbit=dict(lookat=[0.0, 0.0, env.board.top], distance=top_dist,
                            azimuth=0, elevation=-88))
tiles.append(('A  top (overhead)  board 244 mm, field 224 mm', Image.fromarray(top), top_dist))

front_dist = 0.70
front = env.render(orbit=dict(lookat=[0.0, -0.10, 0.46], distance=front_dist,
                              azimuth=180, elevation=-10))
tiles.append(('B  front (from the board side)  gap 90 mm', Image.fromarray(front), front_dist))

side = env.render(orbit=dict(lookat=[0.0, -0.10, 0.46], distance=front_dist,
                             azimuth=90, elevation=-10))
tiles.append(('C  side  base on the 430 mm plane', Image.fromarray(side), front_dist))

wrist = env.render(camera=Camera.WRIST)
tiles.append((f'D  wrist camera ({gripper.KIND} mount, fovy {cam_fovy:.2f} deg)', Image.fromarray(wrist), None))

# scale bars + labels on the first three
for label, img, dist in tiles:
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, W - 1, 22], fill=(0, 0, 0))
    dr.text((8, 4), label, font=f14, fill=(255, 240, 150))
    if dist is not None:
        px = scale_px_per_mm(dist) * 100.0
        x0, y0 = 20, H - 30
        dr.line([(x0, y0), (x0 + px, y0)], fill=(255, 255, 255), width=3)
        for xx in (x0, x0 + px):
            dr.line([(xx, y0 - 8), (xx, y0 + 8)], fill=(255, 255, 255), width=3)
        dr.text((x0, y0 - 26), f'100 mm  ({px:.0f} px @ lookat plane, approx)',
                font=f13, fill=(255, 255, 255))

# ---- sheet ----------------------------------------------------------------
COLS, ROWS, PAD = 2, 2, 16
TEXT_H = 360
sheet = Image.new('RGB', (W * COLS + PAD * 3, H * ROWS + PAD * 3 + TEXT_H), (18, 18, 20))
sd = ImageDraw.Draw(sheet)
for i, (label, img, _dist) in enumerate(tiles):
    c, r = i % COLS, i // COLS
    sheet.paste(img, (PAD + c * (W + PAD), PAD + r * (H + PAD)))

y = PAD + ROWS * (H + PAD) + 4
lines = [
    (f"PROVENANCE  git {head}   modified {len(modified)}   untracked {len(untracked)}"
     + ("" if not modified else "   mod: " + ", ".join(x[3:] for x in modified[:4]) + (" ..." if len(modified) > 4 else "")),
     f18b, (255, 225, 120)),
    (f"BOARD  square {board.square*1000:.0f}  border {board.border*1000:.0f}  field {board.field*1000:.0f}"
     f"  overall {board.width*1000:.0f} mm  half {board.width*500:.0f}  top z {env.board.top*1000:.1f} mm",
     f16, (200, 240, 255)),
    (f"MOUNT  base frame y {arm_base_y:.1f} mm   board edge y {board_edge_y:.1f} mm   edge-to-base {board.arm_gap*1000:.0f} mm",
     f16, (200, 240, 255)),
    (f"DATUM  base mesh front y {mesh_front_y:.3f} mm  = {mesh_front_y-arm_base_y:.3f} mm past origin;"
     f"  mesh front -> board edge {mesh_gap_mm:.3f} mm  (=25.4 mm nominal)",
     f16, (150, 255, 180)),
    (f"TOOL  {gripper.KIND}  profile {gripper.PROFILE.get('name') or 'stock'}  rev {gripper.PROFILE.get('source_microversion') or '-'}",
     f14, (255, 255, 255)),
    (f"      render pack root {gripper.ASSET_ROOT}", f13, (220, 220, 220)),
    (f"TIP   stamp {prov['tip_constant']['stamp_tip_mm']} mm from {prov['tip_constant']['stamp_pack']}"
     f" (asset_root exists={prov['tip_constant']['stamp_asset_root_exists']})  git {prov['tip_constant']['stamp_git_head']}",
     f14, (255, 255, 255)),
    (f"CAMERA  source {scene.CAMERA_SOURCE}  wrist fovy(actual) {cam_fovy:.2f} deg  pos {cam_pos_world} mm",
     f14, (255, 255, 255)),
    (f"POSE  {prov['pose']}   generated {prov['generated_utc']}", f13, (190, 190, 190)),
]
max_w = sheet.width - 2 * (PAD + 4)
for text, font, col in lines:
    cur = ''
    for word in text.split(' '):
        trial = (cur + ' ' + word).strip()
        if sd.textlength(trial, font=font) <= max_w:
            cur = trial
        else:
            sd.text((PAD + 4, y), cur, font=font, fill=col)
            y += 28 if font in (f18b, f16) else 24
            cur = word
    if cur:
        sd.text((PAD + 4, y), cur, font=font, fill=col)
        y += 28 if font in (f18b, f16) else 24

out_png = OUT / 'board_layout_gate.png'
sheet.save(out_png)
for i, (label, img, _d) in enumerate(tiles):
    img.save(OUT / f'tile_{chr(65+i)}.png')
print('wrote', out_png)
print(json.dumps(prov['mount'], indent=2))
print('camera', prov['camera'])
