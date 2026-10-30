"""Data-boundary regressions: leakage and corrupted action labels fail closed."""
import json

import numpy as np
import pytest

from chess_sim import JOINTS, ROBOT_CAMERAS
from tools.audit_training_data import audit


def recording(root, name, offset=0):
    path = root / name
    path.mkdir()
    states = np.arange(18, dtype=np.float32).reshape(3, 6) / 100 + offset
    np.savez(path / 'data.npz', observation_state=states, action=states + .01)
    meta = dict(success=True, instruction='move e2 to e4', steps=3, fps=30,
                joints=[str(j) for j in JOINTS], cameras=[str(c) for c in ROBOT_CAMERAS])
    (path / 'meta.json').write_text(json.dumps(meta))
    for cam in ROBOT_CAMERAS:
        (path / f'{cam}.mp4').write_bytes(b'test media')
    return dict(path=name, origin='simulator', session=name, layout_group=name, split='train')


def run(root, entries):
    manifest = root / 'manifest.json'
    manifest.write_text(json.dumps(dict(schema=1, episodes=entries)))
    return audit(manifest, check_video=False)


def test_independent_splits_pass(tmp_path):
    a, b = recording(tmp_path, 'a'), recording(tmp_path, 'b', .1)
    b['split'] = 'test'
    assert run(tmp_path, [a, b])['passed']


@pytest.mark.parametrize('key', ['session', 'layout_group'])
def test_whole_group_must_stay_in_one_split(tmp_path, key):
    a, b = recording(tmp_path, 'a'), recording(tmp_path, 'b', .1)
    b.update(split='test', **{key: a[key]})
    assert not run(tmp_path, [a, b])['passed']


def test_renamed_duplicate_is_rejected(tmp_path):
    a, b = recording(tmp_path, 'a'), recording(tmp_path, 'b')
    b['split'] = 'test'
    assert 'duplicate' in run(tmp_path, [a, b])['errors'][0]['error']


def test_sim_and_real_must_not_mix(tmp_path):
    a, b = recording(tmp_path, 'a'), recording(tmp_path, 'b', .1)
    b['origin'] = 'real_camera'
    assert not run(tmp_path, [a, b])['passed']


def test_nan_action_is_rejected(tmp_path):
    a = recording(tmp_path, 'a')
    np.savez(tmp_path / 'a/data.npz', observation_state=np.zeros((3, 6)),
             action=np.full((3, 6), np.nan))
    assert 'nonfinite' in run(tmp_path, [a])['errors'][0]['error']
