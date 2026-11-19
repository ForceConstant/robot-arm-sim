"""Block 1: score jaw placements against the CAD cross-checks."""
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
def Rz(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

spec = scene.build_arm(); m = mujoco.MjModel.from_xml_string(spec.to_xml())
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
wg = m.body('so101:gripper').id
Tw = T(d.xmat[wg].reshape(3,3), d.xpos[wg])
def grip(name):
    b = m.body(name).id
    return inv(Tw) @ T(d.xmat[b].reshape(3,3), d.xpos[b])

V = np.asarray(trimesh.load(g.ASSET_ROOT/'assets/gripper_jaw_1__solid1.stl', force='mesh').vertices)
GEOM = T(Rq([0,-0.707107,0,0.707107]), np.array([0.022,-0.0187,0.019]))

def place(bname, extra=None):
    Mx = grip(bname) @ GEOM
    return Mx if extra is None else extra @ Mx

def measure(ML, MR):
    def pts(M):
        return ((M[:3,:3] @ V.T).T + M[:3,3])*1000
    L, R = pts(ML), pts(MR)
    tipL = L[L[:,2] < L[:,2].min()+0.5]; tipR = R[R[:,2] < R[:,2].min()+0.5]
    rootL = L[L[:,2] > L[:,2].max()-0.5]; rootR = R[R[:,2] > R[:,2].max()-0.5]
    return dict(depth=-L[:,2].min(), tip_sep=np.linalg.norm(tipL.mean(0)-tipR.mean(0)),
                root_sep=np.linalg.norm(rootL.mean(0)-rootR.mean(0)),
                tip_c=np.round((tipL.mean(0)+tipR.mean(0))/2, 2),
                tipL=tipL.mean(0), tipR=tipR.mean(0))

base = measure(place('so101:cad:parallel_jaw_left'), place('so101:cad:parallel_jaw_right'))
print('as built:', {k: (np.round(v,3) if isinstance(v,(int,float)) else v) for k,v in base.items()})
print('  tipL', np.round(base['tipL'],2), 'tipR', np.round(base['tipR'],2))

print('\nrotation about gripper z, at the current aperture:')
for rz in (0, 30, 45, 60, 90, 135, 180, 270):
    R = T(Rz(rz), np.zeros(3))
    s = measure(R @ place('so101:cad:parallel_jaw_left'), R @ place('so101:cad:parallel_jaw_right'))
    print(f'  rz={rz:3d}: depth {s["depth"]:.2f} tip_sep {s["tip_sep"]:.2f} root_sep {s["root_sep"]:.2f} '
          f'tipL {np.round(s["tipL"],1)} tipR {np.round(s["tipR"],1)}')

print('\naperture sweep (joint slide) at rz=0:')
for qmm in (0, -5, -10, -11.78, -15, -19):
    a = qmm/1000.0
    ML = place('so101:cad:parallel_jaw_left', T(np.eye(3), np.array([a,0,0])))
    MR = place('so101:cad:parallel_jaw_right', T(np.eye(3), np.array([-a,0,0])))
    s = measure(ML, MR)
    print(f'  slide {qmm:6.2f} mm: tip_sep {s["tip_sep"]:.2f} root_sep {s["root_sep"]:.2f} '
          f'tipL {np.round(s["tipL"],2)}')
