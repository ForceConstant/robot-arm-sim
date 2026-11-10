"""Phase 11b pipeline contracts: bucket positions, goal labels and split rule."""
from __future__ import annotations

import json
import random
from pathlib import Path

import chess
import imageio.v2 as imageio
import numpy as np

from chess_sim import TaskFamily
from tools.collect_phase11b import BucketSampler
from tools.export_phase11b import export


def test_king_bucket_keeps_one_king_per_side():
    sampler = BucketSampler("king", "white", TaskFamily.MOVE, "sparse")
    for seed in range(5):
        board, source = sampler._position(random.Random(seed))
        kings = [square for square in chess.SQUARES
                 if board.piece_at(square) and board.piece_at(square).piece_type == chess.KING]
        assert len(kings) == 2
        assert board.piece_at(source).piece_type == chess.KING
        assert board.piece_at(source).color == chess.WHITE
        assert board.is_valid()


def test_non_king_bucket_places_the_requested_piece():
    sampler = BucketSampler("knight", "black", TaskFamily.CAPTURE, "crowded")
    board, source = sampler._position(random.Random(7))
    piece = board.piece_at(source)
    assert piece.piece_type == chess.KNIGHT and piece.color == chess.BLACK
    assert board.is_valid()


def _episode(root: Path, index: int) -> None:
    episode = root / f"episode_{index:04d}"
    episode.mkdir()
    (episode / "meta.json").write_text(json.dumps(dict(
        episode=index, success=True, fps=30,
        goal_xy_m=[[0.10, 0.20], [0.30, 0.40]])))
    states = np.zeros((40, 6), dtype=np.float32)
    actions = np.zeros((40, 6), dtype=np.float32)
    actions[:, 0] = np.linspace(0.0, 0.4, 40)
    np.savez(episode / "data.npz", observation_state=states, action=actions)
    imageio.mimsave(episode / "wrist.mp4", [np.full((8, 8, 3), value, np.uint8)
                                            for value in range(40)], fps=30)


def test_export_splits_by_episode_and_keeps_the_explicit_goal(tmp_path):
    shard = tmp_path / "shard_00"
    shard.mkdir()
    (shard / "bucket.json").write_text(json.dumps(dict(bucket="pawn/white/move/sparse")))
    for index in range(4):
        _episode(shard, index)
    summary = export(tmp_path, tmp_path / "out", frames=2, size=8)
    assert summary["episodes"] == 4
    assert summary["train_episodes"] == 3 and summary["test_episodes"] == 1
    assert summary["heldout_rule"] == "every 4th episode of each bucket"
    with np.load(tmp_path / "out" / "phase11b_8.npz", allow_pickle=False) as data:
        assert data["image"].shape[1:] == (8, 8, 3)
        assert set(data["split"]) == {"train", "test"}
        assert set(data["episode"][data["split"] == "train"]) & set(
            data["episode"][data["split"] == "test"]) == set()
        assert np.allclose(data["goal"][0], [0.10, 0.20, 0.0, 0.30, 0.40, 0.0])
        assert data["action"].shape[1:] == (30, 6)
