"""Collect the Phase 11b control-policy corpus from the deployed product build.

One process per (piece, colour, task family, crowding) bucket, so a bucket's
whole session stays together for the train/test split. Records wrist-only RGB,
observed joints, executed actions and the explicit source/destination goal in
the board frame. Success verdicts stay in meta.json; the exporter keeps only
verified successes.

    ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-v5-tipthin \
    ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight \
    MUJOCO_GL=egl .venv/bin/python tools/collect_phase11b.py --out .private/phase11b/data --episodes 12 --workers 4
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import random
import time

import chess
import numpy as np

from chess_sim import (AppearanceConfig, Camera, CaptureTask, ChessSimEnv, ControlConfig,
                       EnvConfig, EpisodeRecorder, MoveTask, RandomizationConfig, RecorderConfig,
                       TaskFamily, demonstrate)
from chess_sim.config import PER_EPISODE_PLACEMENT, square_at
from chess_sim.positions import PROMOTION_RANKS, spare_pieces

PIECES = {"pawn": chess.PAWN, "knight": chess.KNIGHT, "bishop": chess.BISHOP,
          "rook": chess.ROOK, "queen": chess.QUEEN, "king": chess.KING}
COLORS = {"white": chess.WHITE, "black": chess.BLACK}
CROWDING = {"sparse": (1, 2), "crowded": (7, 8)}
SHARD = "shard_{:02d}"


class BucketSampler:
    """Draws a position that contains the requested piece and yields a task on it."""

    def __init__(self, piece: str, color: str, family: TaskFamily, crowding: str):
        self.piece = piece
        self.piece_type = PIECES[piece]
        self.color = COLORS[color]
        self.family = family
        self.crowding = crowding
        self.extras = CROWDING[crowding]
        self.bucket = f"{piece}/{color}/{family.value}/{crowding}"

    def _position(self, rng: random.Random) -> tuple[chess.Board, int]:
        for _ in range(2000):
            extra = rng.randint(*self.extras)
            board = chess.Board(None)
            if self.piece_type == chess.KING:
                squares = rng.sample(chess.SQUARES, 2 + extra)
                source = squares[0]
                board.set_piece_at(source, chess.Piece(chess.KING, self.color))
                board.set_piece_at(squares[1], chess.Piece(chess.KING, not self.color))
                spare_squares = squares[2:]
            else:
                squares = rng.sample(chess.SQUARES, 3 + extra)
                source = squares[0]
                if chess.square_rank(source) in PROMOTION_RANKS and self.piece_type == chess.PAWN:
                    continue
                board.set_piece_at(source, chess.Piece(self.piece_type, self.color))
                board.set_piece_at(squares[1], chess.Piece(chess.KING, chess.WHITE))
                board.set_piece_at(squares[2], chess.Piece(chess.KING, chess.BLACK))
                spare_squares = squares[3:]
            spare = spare_pieces(board, source)
            rng.shuffle(spare)
            placed = True
            for square in spare_squares:
                for index, piece in enumerate(spare):
                    if piece.piece_type == chess.PAWN and chess.square_rank(square) in PROMOTION_RANKS:
                        continue
                    board.set_piece_at(square, piece)
                    spare.pop(index)
                    break
                else:
                    placed = False
                    break
            if not placed:
                continue
            board.turn = self.color
            if board.is_valid():
                return board, source
        raise RuntimeError(f"could not build a valid position for {self.bucket}")

    def sample(self, env: ChessSimEnv, rng: random.Random, index: int = 0):
        for _ in range(200):
            board, source = self._position(rng)
            env.reset(board.fen(), rng=rng)
            if self.family is TaskFamily.MOVE:
                moves = [m for m in env.executable_moves() if m.from_square == source]
                if moves:
                    return MoveTask.from_move(rng.choice(moves))
            elif source in env.executable_captures():
                return CaptureTask(square=square_at(source))
        raise RuntimeError(f"no executable {self.family} for {self.bucket}")


def collect(job: tuple) -> dict:
    out, index, piece, color, family, crowding, episodes, seed, image_size = job
    rng = random.Random(seed)
    looks = np.random.default_rng(seed)
    root = os.path.join(out, SHARD.format(index))
    os.makedirs(root, exist_ok=True)
    config = EnvConfig(
        appearance=AppearanceConfig.sample(looks, piece_sets=("printed",)),
        randomization=RandomizationConfig(**PER_EPISODE_PLACEMENT),
        control=ControlConfig(image_size=image_size, cameras=(Camera.WRIST,)))
    env = ChessSimEnv(config)
    recorder = EpisodeRecorder(env, RecorderConfig(root=root, cameras=(Camera.WRIST,)))
    sampler = BucketSampler(piece, color, family, crowding)

    def new_look(step: int) -> None:
        if step:
            env.recolor(AppearanceConfig.sample(looks, piece_sets=("printed",)))

    started = time.time()
    results = demonstrate(env, sampler, episodes, rng, recorder=recorder,
                          record_appearance=True, on_episode_start=new_look)
    summary = dict(shard=index, bucket=sampler.bucket, piece=piece, color=color,
                   family=family.value, crowding=crowding, episodes=episodes,
                   successes=sum(r.success for r in results),
                   reasons=[r.reason for r in results], seconds=round(time.time() - started, 1))
    env.close()
    with open(os.path.join(root, "bucket.json"), "w") as handle:
        json.dump(summary, handle, indent=2)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--episodes", type=int, default=12, help="per bucket")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=20261110)
    parser.add_argument("--image-size", type=int, nargs=2, default=(224, 224))
    parser.add_argument("--resume", action="store_true", help="skip buckets that already have bucket.json")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    buckets = [(piece, color, family, crowding)
               for piece in PIECES for color in COLORS
               for family in (TaskFamily.MOVE, TaskFamily.CAPTURE) for crowding in CROWDING]
    jobs = [(args.out, index, *bucket, args.episodes, args.seed + 1000 * index,
             tuple(args.image_size)) for index, bucket in enumerate(buckets)]
    if args.resume:
        import shutil
        keep = []
        for job in jobs:
            shard = os.path.join(args.out, SHARD.format(job[1]))
            if os.path.isfile(os.path.join(shard, "bucket.json")):
                continue
            if os.path.isdir(shard):
                shutil.rmtree(shard)
            keep.append(job)
        jobs = keep
        print(f"resume: {len(jobs)} buckets remaining", flush=True)
    started = time.time()
    results = []
    with mp.get_context("spawn").Pool(args.workers, maxtasksperchild=1) as pool:
        pending = [(job, pool.apply_async(collect, (job,))) for job in jobs]
        for job, handle in pending:
            try:
                results.append(handle.get())
            except Exception as exc:                       # one bad bucket must not stop the corpus
                print(f"FAILED shard {job[1]} {job[2]}/{job[3]}/{job[4]}: {exc!r}", flush=True)
                results.append(dict(shard=job[1], bucket=f"{job[2]}/{job[3]}/{job[4].value}/{job[5]}",
                                    episodes=job[6], successes=0, error=repr(exc)))
    results.sort(key=lambda row: row["shard"])
    report_path = os.path.join(args.out, "collection.json")
    previous = []
    if args.resume and os.path.isfile(report_path):
        previous = [row for row in json.loads(open(report_path).read()).get("buckets", [])
                    if row["shard"] not in {r["shard"] for r in results}]
    results = sorted(previous + results, key=lambda row: row["shard"])
    total = sum(row["episodes"] for row in results)
    successes = sum(row["successes"] for row in results)
    report = dict(schema="phase11b-corpus/1", buckets=results, episodes=total,
                  successes=successes, workers=args.workers, seed=args.seed,
                  image_size=list(args.image_size), seconds=round(time.time() - started, 1),
                  scope="simulated expert demonstrations; not physical data")
    with open(report_path, "w") as handle:
        json.dump(report, handle, indent=2)
    print(json.dumps({k: report[k] for k in ("episodes", "successes", "seconds")}))


if __name__ == "__main__":
    main()
