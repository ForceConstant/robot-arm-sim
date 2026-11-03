"""Audit and export wrist RGB + observed radians + instructed board-frame goals.

No oracle poses/FEN enter the policy. Entire episodes/sessions/layouts remain
split. This small historical fixture tests ingestion, not current-layout quality.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.audit_training_data import audit


def goal(move):
    if len(move) != 4 or any(move[i] not in 'abcdefgh' for i in (0, 2)) or any(move[i] not in '12345678' for i in (1, 3)):
        raise ValueError('need explicit source/destination squares')
    return [(ord(move[i])-ord('a')-3.5)*0.028 if j == 0 else
            (int(move[i+1])-1-3.5)*0.028 if j == 1 else 0.0 for i in (0, 2) for j in range(3)]


def export(manifest_path, out):
    report = audit(manifest_path)
    if not report['passed']:
        raise ValueError(f'Failed audit: {report["errors"]}')
    manifest = json.loads(manifest_path.read_text())
    rows = []
    for entry in manifest['episodes']:
        if entry['split'] not in ('train', 'test'):
            raise ValueError('pilot requires train/test splits')
        path = manifest_path.parent/entry['path']
        meta = json.loads((path/'meta.json').read_text())
        if meta['fps'] != 30:
            raise ValueError('pinned 30Hz recording -> 10Hz action contract')
        with np.load(path/'data.npz', allow_pickle=False) as data:
            states, actions = data['observation_state'], data['action']
        reader = imageio.get_reader(path/'wrist.mp4')
        try:
            for index in np.linspace(0, len(states)-1, 16, dtype=int):
                indices = index + np.arange(30)*3
                pad = indices >= len(states)
                chunk = actions[np.minimum(indices, len(states)-1)]
                frame = np.asarray(Image.fromarray(reader.get_data(int(index))).resize((224,224), Image.Resampling.BOX))
                rows.append(dict(image=frame, state=states[index], action=chunk, pad=pad,
                                 goal=goal(meta['move']), split=entry['split'], episode=entry['path'], frame=index))
        finally:
            reader.close()
    arrays = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
    for key in ('state', 'action', 'goal'):
        arrays[key] = arrays[key].astype(np.float32)
        if not np.isfinite(arrays[key]).all():
            raise ValueError('nonfinite export')
    if set(arrays['split']) != {'train', 'test'}:
        raise ValueError('need both splits')
    out.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(out/'dataset.npz', **arrays)
    receipt = dict(schema=1, audit=report, samples=len(rows), train=int(sum(arrays['split']=='train')),
                   test=int(sum(arrays['split']=='test')), image_shape=[224,224,3], chunk_size=30,
                   action_hz=10, action='absolute_joint_targets_radians', goal='source_xyz_destination_xyz_metres_board_frame_z0',
                   frame_alignment='recording-index aligned; physical capture synchronization NOT qualified',
                   scope='historical three-episode platform pilot; not current product-layout training',
                   dataset_sha256=hashlib.sha256((out/'dataset.npz').read_bytes()).hexdigest())
    (out/'manifest.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.manifest, args.out)))
