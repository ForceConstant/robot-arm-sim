"""Private, bounded physics incident recordings. MuJoCo access: worker thread only.

Playback renders saved states; it never claims to re-simulate physics. Re-run
restores the saved integration state and executes the controller independently.
"""
from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import re
import shutil
import uuid
import zipfile

import mujoco
import numpy as np

STATE_SPEC = mujoco.mjtState.mjSTATE_INTEGRATION
MAX_FRAMES = 4000
MAX_RECORDINGS = 40
MAX_BYTES = 512 * 1024 * 1024
LABELS = ('unreviewed', 'path_collision', 'gripper_geometry', 'missed_grasp',
          'slip_drop', 'release_retreat', 'tracking', 'other')


def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False))
    tmp.replace(path)


def model_signature(env):
    """Hash of the compiled model: enough to replay a saved state honestly."""
    binary = np.empty(mujoco.mj_sizeModel(env.model), dtype=np.uint8)
    mujoco.mj_saveModel(env.model, None, binary)
    return hashlib.sha256(binary.tobytes()).hexdigest()


def runtime_signature(model_hash):
    """Model plus controller/runtime source. Required to re-execute, not to view."""
    h = hashlib.sha256(str(model_hash).encode())
    for folder in ('chess_sim', 'robot_web'):
        for p in sorted((Path(__file__).resolve().parents[1] / folder).glob('*.py')):
            h.update(p.name.encode()); h.update(p.read_bytes())
    return h.hexdigest()


DIAGNOSTIC_DEFAULTS = dict(diagnostic_phase="idle", diagnostic_target=None,
                           diagnostic_gap=None, diagnostic_ik=None, diagnostic_stall=None)


def reset_diagnostics(env):
    """Blank the per-action diagnostics before a recording starts.

    Without this the first sample of a recording carries the previous action's
    phase and stall flag, so a clean action can be labelled with someone else's
    failure (and a successful case can report a stall).
    """
    for key, value in DIAGNOSTIC_DEFAULTS.items():
        setattr(env, key, value)


def physics_state(env):
    state = np.empty(mujoco.mj_stateSize(env.model, STATE_SPEC))
    mujoco.mj_getState(env.model, env.data, state, STATE_SPEC)
    return state


def bookkeeping(env):
    return dict(position=env.position.fen(), active_side=env.active_side,
                square_slots={str(k): s.body for k, s in env._square_slot.items()},
                captured=[s.body for s in env._captured], capture_count=env._capture_count.copy(),
                step_count=env._step_count, noise_rng=env._noise_rng.bit_generator.state,
                noise_from=env._noise_from.tolist(), noise_to=env._noise_to.tolist(),
                noise_suppressed=env.noise_suppressed)


def restore(env, state, meta):
    import chess
    slots = {s.body: s for s in env.slots}
    env.select_arm(meta['active_side'])
    env.position = chess.Board(meta['position'])
    env._square_slot = {int(k): slots[v] for k, v in meta['square_slots'].items()}
    env._captured = [slots[n] for n in meta['captured']]
    env._capture_count = meta['capture_count'].copy()
    env._step_count = meta['step_count']
    env._noise_rng.bit_generator.state = meta['noise_rng']
    env._noise_from[:] = meta['noise_from']; env._noise_to[:] = meta['noise_to']
    env.noise_suppressed = meta['noise_suppressed']
    mujoco.mj_setState(env.model, env.data, state, STATE_SPEC)
    # Recompute derived fields without losing the saved solver warm start.
    warm = env.data.qacc_warmstart.copy()
    mujoco.mj_forward(env.model, env.data)
    env.data.qacc_warmstart[:] = warm


class Store:
    def __init__(self, root):
        self.root = Path(root) / 'incidents'
        self.root.mkdir(parents=True, exist_ok=True)

    def path(self, ident):
        if not re.fullmatch(r'[0-9a-f]{32}', ident):
            raise ValueError('Invalid recording ID')
        path = self.root / ident
        if not (path / 'manifest.json').is_file():
            raise ValueError('Recording not found')
        return path

    def rows(self):
        rows = []
        for p in sorted(self.root.glob('*/manifest.json'), key=lambda p:p.stat().st_mtime, reverse=True):
            m = json.loads(p.read_text())
            rows.append({k:m.get(k) for k in ('id','created','label','action','success','categories',
                                              'review_label','frames','error_mm','case_id','message')})
        return rows

    def read(self, ident):
        return json.loads((self.path(ident) / 'manifest.json').read_text())

    def annotate(self, ident, label, notes):
        if label not in LABELS:
            raise ValueError('Unknown review category')
        path = self.path(ident) / 'manifest.json'
        m = json.loads(path.read_text()); m.update(review_label=label, review_notes=notes)
        atomic_json(path, m)

    def load(self, ident, expected_signature):
        path = self.path(ident); m = self.read(ident)
        # Schema 1 stored one combined model+source hash, so it cannot be compared
        # to a model-only hash. Those recordings are viewable (playback renders the
        # states they contain) but never re-runnable: re-run needs a runtime hash.
        if m.get("schema", 1) >= 2:
            if m["model_signature"] != expected_signature:
                raise ValueError("This recording was made with a different model; "
                                 "download it and replay it on that build")
            if m["mujoco_version"] != mujoco.__version__:
                raise ValueError(f"Recording needs MuJoCo {m['mujoco_version']}, "
                                 f"this build has {mujoco.__version__}")
        with np.load(path / 'states.npz', allow_pickle=False) as z:
            states = z['states'].copy()
        return m, states, json.loads((path / 'trace.json').read_text())

    def download(self, ident):
        path = self.path(ident); buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
            for p in sorted(path.rglob('*')):
                if p.is_file():
                    z.write(p, str(p.relative_to(path)))
        return buf.getvalue()

    def prune(self):
        # Explicit retention applies to this recorder's own completed recordings only.
        folders = sorted((p.parent for p in self.root.glob('*/manifest.json')),
                         key=lambda p:(p/'manifest.json').stat().st_mtime, reverse=True)
        used = 0
        for i, p in enumerate(folders):
            used += sum(f.stat().st_size for f in p.rglob('*') if f.is_file())
            if i >= MAX_RECORDINGS or used > MAX_BYTES:
                shutil.rmtree(p)


class Recording:
    def __init__(self, store, env, model_hash, code_hash, metadata, frames):
        self.store, self.env = store, env
        self.id = uuid.uuid4().hex
        self.path = store.root / ('.' + self.id)
        self.path.mkdir()
        self.meta = dict(id=self.id, created=datetime.now(timezone.utc).isoformat(),
                         schema=2, model_signature=model_hash, runtime_signature=code_hash,
                         mujoco_version=mujoco.__version__, state_spec=int(STATE_SPEC),
                         config=env.config.model_dump(mode='json'), initial=bookkeeping(env),
                         review_label='unreviewed', review_notes='', **metadata)
        self.states, self.trace, self.contacts = [], [], {}
        self.target_body = None
        self.start_positions = {s.body: env.piece_position(s) for s in env.active_slots()}
        self.images('before', frames)
        self.sample()

    def images(self, prefix, frames):
        for name, data in frames.items():
            (self.path / (prefix + '-' + name + '.jpg')).write_bytes(data)

    def contact_sample(self):
        env = self.env
        phase = getattr(env, 'diagnostic_phase', 'unknown')
        for c in env.data.contact:
            b = [int(env.model.geom_bodyid[g]) for g in (c.geom1, c.geom2)]
            names = [env.model.body(i).name or 'world' for i in b]
            robots = [n.startswith(('so101:', 'black:')) for n in names]
            if not any(robots) or c.dist > .0001:
                continue
            # Retain robot/object and cross-arm contacts, not noisy internal guides.
            if all(robots) and names[0].split(':')[0] == names[1].split(':')[0]:
                continue
            key = (phase, int(c.geom1), int(c.geom2))
            force = np.zeros(6)
            # Contact index belongs to the active data; do not infer force from depth.
            idx = next((i for i in range(env.data.ncon) if env.data.contact[i].geom1 == c.geom1
                        and env.data.contact[i].geom2 == c.geom2), None)
            if idx is not None:
                mujoco.mj_contactForce(env.model, env.data, idx, force)
            other = names[1] if robots[0] else names[0]
            other_id = b[1] if robots[0] else b[0]
            # Only a piece counts as an unexpected grasp-phase contact. The board
            # and table are surfaces the jaws legitimately approach; keeping them
            # in the same bucket flagged every clean pick as suspicious.
            is_piece = other_id in env._slot_body
            expected = is_piece and other == self.target_body and phase in (
                'close', 'lift', 'carry', 'place', 'place_descend', 'release')
            if key not in self.contacts and len(self.contacts) < 4096:
                self.contacts[key] = dict(phase=phase, bodies=names, kind='piece' if is_piece else 'surface',
                    geoms=[env.model.geom(int(g)).name for g in (c.geom1,c.geom2)],
                    first_time=float(env.data.time), frame=max(0,len(self.states)-1),
                    target_contact=expected, peak_force_N=float(np.linalg.norm(force[:3])),
                    minimum_distance_m=float(c.dist))
            elif key in self.contacts:
                event=self.contacts[key]
                event['peak_force_N']=max(event['peak_force_N'],float(np.linalg.norm(force[:3])))
                event['minimum_distance_m']=min(event['minimum_distance_m'],float(c.dist))

    def sample(self, action=None):
        if len(self.states) >= MAX_FRAMES:
            raise RuntimeError('Recording reached its 4000-frame limit; action stopped and partial trace retained')
        env=self.env
        self.states.append(physics_state(env))
        tip,_=env.ik.tool_pose(env.data,np.asarray(getattr(env,'diagnostic_offset',[0,0,0])))
        actual=env.arm_joint_positions()
        shifts={name:float(np.linalg.norm(env.data.xpos[env._slot_body[name]][:2]-p[:2]))
                for name,p in self.start_positions.items() if name != self.target_body}
        ik = getattr(env, 'diagnostic_ik', None) or {}
        self.trace.append(dict(time=float(env.data.time),phase=getattr(env,'diagnostic_phase','initial'),
            target_tcp=getattr(env,'diagnostic_target',None),actual_tcp=tip.tolist(),
            commanded_gap_m=getattr(env,'diagnostic_gap',None),
            actual_gap_m=float(__import__('chess_sim.gripper',fromlist=['angle_to_gap']).angle_to_gap(actual[5])),
            action=None if action is None else np.asarray(action).tolist(),joints=actual.tolist(),
            # what the solver thought and how much joint room was left: a stalled
            # waypoint and a slow one look the same in TCP positions alone
            ik_residual_mm=None if 'residual_m' not in ik else 1000*ik['residual_m'],
            ik_tilt_deg=None if 'tilt_rad' not in ik else float(np.degrees(ik['tilt_rad'])),
            span_deg=ik.get('span_deg'),
            offset_mm=None if 'offset' not in ik else [1000*float(v) for v in ik['offset']],
            joint_margin_rad=self._joint_margin(env),
            stall=getattr(env, 'diagnostic_stall', None),
            max_neighbour_shift_m=max(shifts.values(),default=0)))

    @staticmethod
    def _joint_margin(env):
        """Smallest distance from the active arm's joints to their limits."""
        try:
            from chess_sim.env import JOINTS
            prefix = 'black:' if getattr(env, 'active_side', 'white') == 'black' else 'so101:'
            margins = []
            for name in JOINTS:
                jid = env.model.joint(prefix + str(name)).id
                lo, hi = env.model.jnt_range[jid]
                q = float(env.data.qpos[env.model.jnt_qposadr[jid]])
                margins.append(min(q - lo, hi - q))
            return min(margins)
        except Exception:
            return None

    def finish(self, success, result, message, frames):
        self.images('after',frames)
        categories=[]
        if result:
            if result.get('disturbed'):categories.append('neighbour_disturbance')
            if not result.get('upright',True):categories.append('toppled')
            if float(result.get('error_mm') or 0) > self.env.config.tolerances.placement*1000:
                categories.append('placement_error')
            if not result.get('waypoints_precise',True):categories.append('tracking_error')
        unexpected=[c for c in self.contacts.values()
                    if c.get('kind') == 'piece' and not c['target_contact']]
        if unexpected:categories.append('unexpected_contact_candidate')
        stalls=[step.get('stall') for step in self.trace if step.get('stall')]
        if stalls:categories.append('stalled_descent')
        if not success and not categories:categories.append('interrupted' if message=='Interrupted' else 'execution_error')
        self.meta.update(success=bool(success),frames=len(self.states),categories=categories or ['success'],
                         stalled_phase=(stalls[0] if stalls else None),
                         error_mm=(result or {}).get('error_mm'),result=result,message=message,
                         contact_sampling='every physics substep; frame states every control step',
                         playback='recorded state visualization, not a physical re-simulation',
                         contact_note='Unexpected contacts are candidates, not established root causes')
        np.savez_compressed(self.path/'states.npz',states=np.asarray(self.states))
        atomic_json(self.path/'trace.json',self.trace)
        atomic_json(self.path/'contacts.json',list(self.contacts.values()))
        atomic_json(self.path/'manifest.json',self.meta)
        self.path.rename(self.store.root/self.id)
        self.store.prune()
        return self.meta
