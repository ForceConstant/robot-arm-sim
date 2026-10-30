"""Export a tiny audited, wrist-only action-data pilot; never a production dataset.

Inputs contain only pixels, observed joints and instructed squares. Simulator
truth (FEN, piece poses, success labels) is not included in model observations.
"""
from __future__ import annotations

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

CONTRACT = "wrist_rgb8x8_joint_radians_task_squares_v1"


def features(frame, state, move):
    if len(move) != 4 or any(move[i] not in "abcdefgh" for i in (0, 2)) or any(
        move[i] not in "12345678" for i in (1, 3)
    ):
        raise ValueError("pilot supports explicit four-character move instructions only")
    state = np.asarray(state, dtype=float)
    if state.shape != (6,) or not np.isfinite(state).all():
        raise ValueError("need six finite observed joints")
    frame = np.asarray(frame)
    if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("need an RGB uint8 wrist frame")
    pixels = np.asarray(Image.fromarray(frame).resize((8, 8), Image.Resampling.BOX), dtype=float)
    squares = ["abcdefgh".index(move[0])/7, (int(move[1])-1)/7,
               "abcdefgh".index(move[2])/7, (int(move[3])-1)/7]
    return (pixels.flatten()/127.5-1).tolist() + (state/np.pi).tolist() + squares


def export(manifest_path, out):
    # Full media and provenance audit must pass before any observations export.
    report = audit(manifest_path)
    if not report["passed"]:
        raise ValueError(f"data audit failed: {report['errors']}")
    manifest = json.loads(manifest_path.read_text())
    result = dict(schema=1, input_contract=CONTRACT, audit=report, train=[], test=[])
    for entry in manifest["episodes"]:
        if entry["split"] not in ("train", "test"):
            raise ValueError("pilot requires explicit train/test splits")
        path = manifest_path.parent / entry["path"]
        meta = json.loads((path/"meta.json").read_text())
        with np.load(path/"data.npz", allow_pickle=False) as data:
            states, actions = data["observation_state"], data["action"]
        reader = imageio.get_reader(path/"wrist.mp4")
        try:
            # Select evenly across each episode, not adjacent-frame random splits.
            for index in np.linspace(0, len(states)-1, 16, dtype=int):
                result[entry["split"]].append(dict(
                    episode=entry["path"], frame=int(index),
                    x=features(reader.get_data(int(index)), states[index], meta["move"]),
                    y=(actions[index].astype(float)-states[index]).tolist()))
        finally:
            reader.close()
    if not result["train"] or not result["test"]:
        raise ValueError("need nonempty independent train and test episodes")
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("x") as handle:
        json.dump(result, handle, allow_nan=False)
    return dict(train_frames=len(result["train"]), test_frames=len(result["test"]),
                sha256=hashlib.sha256(out.read_bytes()).hexdigest(), audit_passed=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.manifest, args.out)))
