"""Export the Phase 11b corpus as the explicit-goal ACT contract.

Input: the shards written by tools/collect_phase11b.py. Only verified successes
are used; the split is by whole episode (every fourth episode of each bucket is
held out), never by adjacent frames. Observations are wrist RGB, observed joint
radians and the task's explicit source/destination in board metres. Oracle FEN,
piece poses and verdicts are never model inputs.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image

CHUNK = 30
STRIDE = 3          # 30 Hz recording -> 10 Hz action contract
TEST_EVERY = 4      # every fourth episode of a bucket is held out


def sha(path: Path) -> str:
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def export(root: Path, out_dir: Path, frames: int, size: int) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = {key: [] for key in ("image", "state", "action", "goal", "pad", "split", "episode", "bucket")}
    episodes = []
    for shard in sorted(root.glob("shard_*")):
        bucket_meta = json.loads((shard / "bucket.json").read_text())
        bucket = bucket_meta["bucket"]
        for episode in sorted(shard.glob("episode_*")):
            meta = json.loads((episode / "meta.json").read_text())
            if not meta.get("success") or not meta.get("goal_xy_m"):
                continue
            if meta["fps"] != 30:
                raise ValueError(f"{episode}: recording is not 30 Hz")
            with np.load(episode / "data.npz", allow_pickle=False) as data:
                states = data["observation_state"].astype(np.float32)
                actions = data["action"].astype(np.float32)
            if states.shape[1] != 6 or actions.shape[1] != 6:
                raise ValueError(f"{episode}: needs six joints")
            index = int(meta["episode"])
            split = "test" if index % TEST_EVERY == TEST_EVERY - 1 else "train"
            goal = np.asarray([[*meta["goal_xy_m"][0], 0.0], [*meta["goal_xy_m"][1], 0.0]],
                              dtype=np.float32).reshape(6)
            wanted = np.linspace(0, len(states) - 1, frames, dtype=int)
            positions = {int(v): i for i, v in enumerate(wanted)}
            frames_out = {}
            reader = imageio.get_reader(episode / "wrist.mp4")
            try:
                for frame_index, frame in enumerate(reader):
                    slot = positions.get(frame_index)
                    if slot is None:
                        continue
                    frames_out[slot] = np.asarray(
                        Image.fromarray(np.asarray(frame)).resize((size, size), Image.Resampling.BOX),
                        dtype=np.uint8)
                    if len(frames_out) == len(positions):
                        break
            finally:
                reader.close()
            if len(frames_out) != len(positions):
                raise ValueError(f"{episode}: missing requested wrist frames")
            for slot, start in enumerate(wanted):
                indices = start + np.arange(CHUNK) * STRIDE
                pad = indices >= len(states)
                chunk = actions[np.minimum(indices, len(states) - 1)]
                rows["image"].append(frames_out[slot])
                rows["state"].append(states[start])
                rows["action"].append(chunk)
                rows["goal"].append(goal)
                rows["pad"].append(pad)
                rows["split"].append(split)
                rows["episode"].append(f"{shard.name}/{episode.name}")
                rows["bucket"].append(bucket)
            episodes.append(dict(shard=shard.name, episode=episode.name, split=split,
                                 bucket=bucket, frames=len(states), samples=frames))
    if not episodes:
        raise ValueError("no verified successful episodes")
    for key in ("state", "action", "goal"):
        rows[key] = np.asarray(rows[key], dtype=np.float32)
        if not np.isfinite(rows[key]).all():
            raise ValueError(f"non-finite {key}")
    arrays = dict(
        image=np.asarray(rows["image"], dtype=np.uint8),
        state=rows["state"], action=rows["action"], goal=rows["goal"],
        pad=np.asarray(rows["pad"], dtype=bool),
        split=np.asarray(rows["split"]), episode=np.asarray(rows["episode"]),
        bucket=np.asarray(rows["bucket"]))
    if arrays["image"].shape[1:] != (size, size, 3):
        raise ValueError("image contract differs")
    train = arrays["split"] == "train"
    test = arrays["split"] == "test"
    if not train.any() or not test.any():
        raise ValueError("need both splits")
    if set(arrays["episode"][train]) & set(arrays["episode"][test]):
        raise ValueError("episode leaked across the split")
    dataset = out_dir / f"phase11b_{size}.npz"
    np.savez(dataset, **arrays)
    summary = dict(schema="phase11b-export/1", dataset=dataset.name,
                   dataset_sha256=sha(dataset), image_shape=list(arrays["image"].shape),
                   chunk_size=CHUNK, action_hz=10, frames_per_episode=frames,
                   episodes=len(episodes), train_episodes=len(set(arrays["episode"][train])),
                   test_episodes=len(set(arrays["episode"][test])),
                   train_samples=int(train.sum()), test_samples=int(test.sum()),
                   buckets=sorted(set(arrays["bucket"].tolist())),
                   heldout_rule=f"every {TEST_EVERY}th episode of each bucket",
                   episodes_detail=episodes,
                   scope="simulated verified successes; model inputs are wrist RGB, joints and explicit goal only")
    (out_dir / "manifest.json").write_text(json.dumps(summary, indent=2))
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--size", type=int, default=224)
    args = parser.parse_args()
    print(json.dumps({k: v for k, v in export(args.root, args.out, args.frames, args.size).items()
                      if k not in ("episodes_detail", "buckets")}))
