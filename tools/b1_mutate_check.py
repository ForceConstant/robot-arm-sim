"""Prove the orientation regression actually catches the two failure modes.

Runs the real test functions against a deliberately broken model. Exit code 0
means: every check passes untouched, and the identity / 90 deg mutations are
each caught by at least one check.
"""
import sys
import numpy as np
import mujoco
from pathlib import Path

ROOT = Path('/home/brian/robot-arm-sim')
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tests'))

from chess_sim import scene
import test_tool_orientation as T

import inspect

CHECKS = [n for n in dir(T)
          if n.startswith('test_')
          and n != 'test_no_floating_camera_box'
          and len(inspect.signature(getattr(T, n)).parameters) >= 1]


def fresh():
    m = scene.build_arm().compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def run(m, d):
    bad = []
    for name in CHECKS:
        try:
            getattr(T, name)((m, d))
        except AssertionError:
            bad.append(name)
        except Exception as exc:            # a crash is also a failure
            bad.append(f'{name}({type(exc).__name__})')
    return bad


def rot_z(deg):
    a = np.radians(deg)
    M = np.eye(4)
    M[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return M


def compose_quat(q_body, R_extra):
    R0 = np.zeros(9)
    mujoco.mju_quat2Mat(R0, np.asarray(q_body, float))
    R = R_extra[:3, :3] @ R0.reshape(3, 3)
    out = np.zeros(4)
    mujoco.mju_mat2Quat(out, R.reshape(-1))
    return out


m, d = fresh()
bad = run(m, d)
print(f'baseline: {len(CHECKS) - len(bad)}/{len(CHECKS)} pass', 'FAILURES: ' + ', '.join(bad) if bad else '')
assert not bad, bad

# 1. identity placement of both jaw bodies on the tool base
m, d = fresh()
for side in ('left', 'right'):
    b = m.body(f'so101:cad:parallel_jaw_{side}').id
    m.body_pos[b] = [0, 0, 0]
    m.body_quat[b] = [1, 0, 0, 0]
mujoco.mj_forward(m, d)
bad = run(m, d)
print(f'identity jaw bodies caught by: {bad}')
assert bad, 'identity placement was NOT caught'

# 2. both jaws yawed 90 deg about the approach axis
m, d = fresh()
Rz = rot_z(90)
for side in ('left', 'right'):
    b = m.body(f'so101:cad:parallel_jaw_{side}').id
    q = m.body_quat[b].copy()
    m.body_quat[b] = compose_quat(q, Rz)
mujoco.mj_forward(m, d)
bad = run(m, d)
print(f'90 deg yaw caught by: {bad}')
assert bad, '90 deg yaw was NOT caught'

# 3. one jaw only rotated 90 deg (asymmetric error)
m, d = fresh()
b = m.body('so101:cad:parallel_jaw_left').id
m.body_quat[b] = compose_quat(m.body_quat[b].copy(), Rz)
mujoco.mj_forward(m, d)
bad = run(m, d)
print(f'single-jaw 90 deg caught by: {bad}')
assert bad, 'single-jaw yaw was NOT caught'

print('OK')
