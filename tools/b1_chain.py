"""Block 1: transform chain, measured from the built model."""
import sys, json
import numpy as np, mujoco, trimesh
from pathlib import Path
sys.path.insert(0, '.')
import chess_sim.gripper as g
from chess_sim import scene

root = g.ASSET_ROOT

def Rq(q):
    R = np.zeros(9); mujoco.mju_quat2Mat(R, np.asarray(q, float)); return R.reshape(3, 3)

def T(R, t):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M

def inv(M):
    R = M[:3, :3]; return T(R.T, -R.T @ M[:3, 3])

def fmt(M):
    return f'R=\n{np.round(M[:3,:3], 5)}\nt(mm)={np.round(M[:3,3]*1000, 4)}'

spec = scene.build_arm(); m = mujoco.MjModel.from_xml_string(spec.to_xml())
d = mujoco.MjData(m); mujoco.mj_forward(m, d)

def grip_from(bname):
    wg = m.body('so101:gripper').id
    Tw = T(d.xmat[wg].reshape(3,3), d.xpos[wg])
    b = m.body(bname).id
    return inv(Tw) @ T(d.xmat[b].reshape(3,3), d.xpos[b])

T_mount = grip_from('so101:cad:parallel_base')
print('T_gripper_from_base_body:\n', fmt(T_mount))

# base part-file -> base body: XML geom (unchanged by MuJoCo recentering composition)
T_body_file = T(Rq([0, 0.707107, 0, -0.707107]), np.array([0.0234292, -0.00946249, 0.0758269]))
E = T_mount @ T_body_file
print('\nT_gripper_from_basepart (E, file coords):\n', fmt(E), '\ndet', round(np.linalg.det(E[:3,:3]), 6))

def bbox(M, path):
    v = np.asarray(trimesh.load(path, force='mesh').vertices)
    w = (M[:3,:3] @ v.T).T + M[:3,3]
    return w*1000

for label, f in (('base', 'gripper_base_1__solid1'), ('camera_mount', 'camera_mount')):
    w = bbox(E, root/'assets'/f'{f}.stl')
    print(f'{label:12s} gripper-frame min={np.round(w.min(0),3)} max={np.round(w.max(0),3)}')

print('\n--- jaws (part file -> gripper) ---')
for key, bn in (('jaw_left','so101:cad:parallel_jaw_left'), ('jaw_right','so101:cad:parallel_jaw_right')):
    C = grip_from(bn) @ T(Rq([0, -0.707107, 0, 0.707107]), np.array([0.022, -0.0187, 0.019]))
    w = bbox(C, root/'assets/gripper_jaw_1__solid1.stl')
    print(f'{key}: min={np.round(w.min(0),3)} max={np.round(w.max(0),3)}')
    iz = int(np.argmin(w[:,2]))
    print(f'   deepest vertex (gripper)={np.round(w[iz],3)}')
    # vertices within 1mm of the deepest z
    deep = w[w[:,2] < w[:,2].min() + 1.0]
    print(f'   n within 1mm of deepest: {len(deep)}  y range of those: {np.round(deep[:,1].min(),3)}..{np.round(deep[:,1].max(),3)}')
    # roots: vertices near the top face z max
    print(f'   top (root) z max={w[:,2].max():.3f}, x range at z>zmax-12: {np.round(w[w[:,2]>w[:,2].max()-12,0].min(),2)}..{np.round(w[w[:,2]>w[:,2].max()-12,0].max(),2)}')

print('\n--- pinion / gear ---')
bp = grip_from('so101:cad:parallel_pinion')
print('pinion body in gripper:\n', fmt(bp))
print('pinion body in base-part:\n', fmt(inv(E) @ bp))
gear_file_bbox = bbox(np.eye(4), root/'assets/gripper_gear_1__solid1.stl')
print('gear file bbox mm:', np.round(gear_file_bbox.min(0),3), np.round(gear_file_bbox.max(0),3))
Cg = bp @ T(Rq([0.5,0.5,0.5,-0.5]), np.array([0,0,0.058]))
wg = bbox(Cg, root/'assets/gripper_gear_1__solid1.stl')
print('gear in gripper: min', np.round(wg.min(0),3), 'max', np.round(wg.max(0),3))
print('gear in base-part: min', np.round(bbox(inv(E) @ Cg, root/'assets/gripper_gear_1__solid1.stl').min(0),3))
wp = bbox(inv(E) @ Cg, root/'assets/gripper_gear_1__solid1.stl')
print('gear part-frame bbox:', np.round(wp.min(0),3), np.round(wp.max(0),3))

print('\n--- other tool bodies in gripper ---')
for n in ('so101:tool_servo', 'so101:camera_housing'):
    if m.body(n).id >= 0:
        Tb = grip_from(n)
        print(n, 'pos', np.round(Tb[:3,3]*1000,3))
