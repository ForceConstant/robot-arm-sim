"""Block 1: test assembly-root occurrence composition against the CAD cross-checks."""
import sys, json, itertools
import numpy as np, mujoco, trimesh
from pathlib import Path
ROOT = Path('/home/brian/robot-arm-sim'); sys.path.insert(0, str(ROOT))
import chess_sim.gripper as g
from chess_sim import scene

def Rq(q):
    R = np.zeros(9); mujoco.mju_quat2Mat(R, np.asarray(q, float)); return R.reshape(3,3)
def T(R, t):
    M = np.eye(4); M[:3,:3] = R; M[:3,3] = t; return M
def inv(M):
    return T(M[:3,:3].T, -M[:3,:3].T @ M[:3,3])

spec = scene.build_arm(); m = mujoco.MjModel.from_xml_string(spec.to_xml())
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
wg = m.body('so101:gripper').id
Tw = T(d.xmat[wg].reshape(3,3), d.xpos[wg])
def grip(name):
    b = m.body(name).id
    return inv(Tw) @ T(d.xmat[b].reshape(3,3), d.xpos[b])

E = grip('so101:cad:parallel_base') @ T(Rq([0,0.707107,0,-0.707107]),
                                        np.array([0.0234292,-0.00946249,0.0758269]))
print('E:', np.round(E[:3,:3],4).tolist(), np.round(E[:3,3]*1000,3))
V = np.asarray(trimesh.load(g.ASSET_ROOT/'assets/gripper_jaw_1__solid1.stl', force='mesh').vertices)

occ = {'L': (np.array([0.023279,0.013914,-0.013873]), [0.707107,0,0,-0.707107]),
       'R': (np.array([0.022979,-0.032839,-0.013873]), [0.707107,0,0,0.707107])}
Tbo = T(Rq([0.707107,0,0,0.707107]), np.array([0.009462,0.023429,-0.062577]))

def stats(Mx):
    w = ((Mx[:3,:3] @ V.T).T + Mx[:3,3])*1000
    zmin, zmax = w[:,2].min(), w[:,2].max()
    tip = w[w[:,2] < zmin+0.5]; root = w[w[:,2] > zmax-0.5]
    return (f'z {zmin:.2f}..{zmax:.2f} x {w[:,0].min():.1f}..{w[:,0].max():.1f} '
            f'y {w[:,1].min():.1f}..{w[:,1].max():.1f} | tip c {np.round(tip.mean(0),2)} '
            f'root c {np.round(root.mean(0),2)}')

for base_variant, name in ((inv(Tbo), 'inv(Tbo)'), (inv(Tbo).T, 'inv(Tbo).T')):
    for jaw_key, (t, q) in occ.items():
        for qv, qlab in ((Rq(q), 'q'), (Rq(q).T, 'qT')):
            Mx = E @ base_variant @ T(qv, t)
            print(f'{name:12s} {jaw_key} {qlab}: {stats(Mx)}')

print('\ncurrent build:')
for key, bn in (('L','so101:cad:parallel_jaw_left'), ('R','so101:cad:parallel_jaw_right')):
    Mx = grip(bn) @ T(Rq([0,-0.707107,0,0.707107]), np.array([0.022,-0.0187,0.019]))
    print(f'  {key}: {stats(Mx)}')
