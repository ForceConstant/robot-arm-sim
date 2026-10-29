"""Measure the scripted expert - the control baseline the roadmap asks for.

    report = run_benchmark(ExpertBenchmarkConfig(moves=32, captures=16))
    print(report.markdown())

The scripted expert plans against simulator state by construction, so this is a
truth-driven control baseline, not a perception result. Every verdict still
comes from `env.evaluate`, the same rule a learned policy is scored by; the
task metadata recorded alongside an episode (which piece stood on the source
square, where the board and arm started) exists only to slice the report. Do
not hand it to a camera-driven controller or policy under test - in a policy
evaluation this module's `describe_task` must not be reachable from the policy.

Placements are randomized the way the qualified envelope allows: fresh positions
from `PositionConfig`, board origin within +/-5 mm per axis and arm start within
+/-0.1 rad, optionally the wider turn/side envelope. Piece sets are built into
the scene, so each set gets its own environment.
"""
from __future__ import annotations

import random
import statistics
import time
from typing import Callable, Iterable

import chess
import numpy as np

from .assets import PRINTED_SET, available_piece_sets
from .config import (AppearanceConfig, Config, ControlConfig, EnvConfig,
                     RandomizationConfig)
from .env import ChessSimEnv, Record
from .positions import PositionConfig
from .rollout import wilson_interval
from .tasks import CaptureSampler, MoveSampler, TaskFamily


class ExpertBenchmarkConfig(Config):
    """How much of the expert to measure, and over what envelope."""

    moves: int = 32                      # move episodes per seed per piece set
    captures: int = 16                   # capture episodes per seed per piece set
    seeds: tuple[int, ...] = (271828, 314159, 161803)
    piece_sets: tuple[str, ...] = ()     # empty: every verified set, plus printed when available
    extra_pieces: tuple[int, int] = (2, 8)   # kings plus this many pieces, as the collector draws
    board_shift: float = 0.005           # m per axis, the supported envelope
    arm_joint_jitter: float = 0.10       # rad per joint, the supported envelope
    board_yaw_deg: float = 0.0           # optional wider envelope: board turned
    board_side: float = 0.0              # optional wider envelope: board off the arm's centre line
    cameras: tuple = ()                  # observations are not rendered for a truth-driven expert


class Summary(Record):
    """One slice of the baseline: counts, rate, interval and failure modes."""

    episodes: int
    successes: int
    rate: float
    interval: tuple[float, float]
    median_error_mm: float
    right_piece: int
    disturbed: int
    reasons: dict[str, int]

    @classmethod
    def of(cls, records: Iterable["EpisodeRecord"]) -> "Summary":
        rows = list(records)
        successes = sum(r.success for r in rows)
        errors = [r.placement_error_mm for r in rows]
        reasons: dict[str, int] = {}
        for row in rows:
            if row.reason:
                reasons[row.reason] = reasons.get(row.reason, 0) + 1
        return cls(episodes=len(rows), successes=successes,
                   rate=successes / len(rows) if rows else 0.0,
                   interval=wilson_interval(successes, len(rows)),
                   median_error_mm=statistics.median(errors) if errors else 0.0,
                   right_piece=sum(r.right_piece for r in rows),
                   disturbed=sum(bool(r.disturbed) for r in rows),
                   reasons=dict(sorted(reasons.items())))


class EpisodeRecord(Record):
    """One expert attempt, with the reporting-only metadata used to slice it."""

    piece_set: str
    seed: int
    family: str
    task: str                          # e2e4 / xg7
    source: str
    target: str | None                 # None for captures
    piece_type: str                    # pawn ... king, read from the sampled position
    piece_color: str                   # white / black
    success: bool
    reason: str | None
    placement_error_mm: float
    right_piece: bool
    upright: bool
    disturbed: tuple[str, ...]
    steps: int
    seconds: float
    board_origin: tuple[float, float]
    arm_start: tuple[float, ...]


class BenchmarkReport(Record):
    """Every episode the run recorded, plus the slices the roadmap asks for."""

    config: ExpertBenchmarkConfig
    started_at: str
    seconds: float
    episodes: tuple[EpisodeRecord, ...]

    # -- slices --------------------------------------------------------------

    def overall(self) -> Summary:
        return Summary.of(self.episodes)

    def by_seed(self) -> dict[int, Summary]:
        return {seed: Summary.of(self._where(seed=seed)) for seed in self._keys("seed")}

    def by_piece_set(self) -> dict[str, Summary]:
        return {name: Summary.of(self._where(piece_set=name)) for name in self._keys("piece_set")}

    def by_family(self) -> dict[str, Summary]:
        return {name: Summary.of(self._where(family=name)) for name in self._keys("family")}

    def by_piece_type(self) -> dict[str, Summary]:
        return {name: Summary.of(self._where(piece_type=name)) for name in self._keys("piece_type")}

    def by_seed_family(self) -> dict[tuple[int, str], Summary]:
        return {pair: Summary.of(self._where(seed=pair[0], family=pair[1]))
                for pair in self._pairs("seed", "family")}

    def by_piece_set_family(self) -> dict[tuple[str, str], Summary]:
        return {pair: Summary.of(self._where(piece_set=pair[0], family=pair[1]))
                for pair in self._pairs("piece_set", "family")}

    def by_piece_set_piece_type(self) -> dict[tuple[str, str], Summary]:
        return {pair: Summary.of(self._where(piece_set=pair[0], piece_type=pair[1]))
                for pair in self._pairs("piece_set", "piece_type")}

    def by_task(self) -> dict[str, Summary]:
        return {name: Summary.of(self._where(task=name)) for name in self._keys("task")}

    # -- writing -------------------------------------------------------------

    def to_json(self) -> str:
        return self.model_dump_json(indent=2)

    def markdown(self) -> str:
        config, overall = self.config, self.overall()
        low, high = overall.interval
        lines = [
            "# Scripted-expert baseline",
            "",
            f"- Generated: {self.started_at} ({self.seconds / 60:.1f} min)",
            f"- Seeds: {', '.join(str(s) for s in self.config.seeds)}",
            f"- Piece sets: {', '.join(self._keys('piece_set'))}",
            f"- Episodes per seed per set: {self.config.moves} moves, {self.config.captures} captures",
            f"- Envelope: board +/-{self.config.board_shift * 1000:.1f} mm per axis, "
            f"arm +/-{self.config.arm_joint_jitter:.2f} rad"
            + (f", board yaw +/-{self.config.board_yaw_deg:.1f} deg"
               if self.config.board_yaw_deg else "")
            + (f", board side +/-{self.config.board_side * 1000:.1f} mm"
               if self.config.board_side else ""),
            "",
            f"**Overall: {overall.successes}/{overall.episodes} "
            f"({overall.rate * 100:.1f}%, 95% CI {low * 100:.1f}-{high * 100:.1f}%); "
            f"median placement error {overall.median_error_mm:.1f} mm; "
            f"named piece engaged {overall.right_piece}/{overall.episodes}; "
            f"{overall.disturbed} episodes disturbed a neighbour.**",
            "",
        ]
        lines += self._section("By task family", self.by_family())
        lines += self._section("By piece type", self.by_piece_type())
        lines += self._section("By seed", self.by_seed())
        lines += self._section("By piece set", self.by_piece_set())
        lines += self._pair_section("By piece set and family", self.by_piece_set_family())
        lines += self._pair_section("By piece set and piece type", self.by_piece_set_piece_type())
        lines += self._pair_section("By seed and family", self.by_seed_family())
        tasks = self.by_task()
        lines += self._section("By task (source-target)", tasks)
        reasons = self.overall().reasons
        if reasons:
            lines += ["## Failure reasons", ""]
            lines += [f"- {reason}: {count}" for reason, count in reasons.items()]
            lines.append("")
        return "\n".join(lines)

    def _section(self, title: str, summaries: dict) -> list[str]:
        lines = [f"## {title}", ""]
        for name, summary in summaries.items():
            lines.append(f"- {name}: {self._line(summary)}")
        lines.append("")
        return lines

    def _pair_section(self, title: str, summaries: dict) -> list[str]:
        lines = [f"## {title}", ""]
        for (first, second), summary in summaries.items():
            lines.append(f"- {first} / {second}: {self._line(summary)}")
        lines.append("")
        return lines

    @staticmethod
    def _line(summary: Summary) -> str:
        low, high = summary.interval
        return (f"{summary.successes}/{summary.episodes} "
                f"({summary.rate * 100:.0f}%, {low * 100:.0f}-{high * 100:.0f}%), "
                f"median {summary.median_error_mm:.1f} mm; "
                f"named piece {summary.right_piece}/{summary.episodes}; "
                f"disturbed {summary.disturbed}"
                + (f"; {summary.reasons}" if summary.reasons else ""))

    def _where(self, **match) -> list[EpisodeRecord]:
        return [row for row in self.episodes
                if all(getattr(row, key) == value for key, value in match.items())]

    def _keys(self, field: str) -> list:
        seen = []
        for row in self.episodes:
            value = getattr(row, field)
            if value not in seen:
                seen.append(value)
        return seen

    def _pairs(self, first: str, second: str) -> list[tuple]:
        """The (first, second) combinations that actually occurred, in order."""
        seen = []
        for row in self.episodes:
            pair = (getattr(row, first), getattr(row, second))
            if pair not in seen:
                seen.append(pair)
        return seen


def describe_task(task, env: ChessSimEnv) -> dict:
    """Reporting metadata only: which piece the task names, and where it was.

    This reads simulator truth. It must never be reachable from a camera-driven
    controller or policy under test - those see observations, not this.
    """
    square = chess.parse_square(str(task.source))
    piece = env.position.piece_at(square)
    target = str(task.to_square) if task.family is TaskFamily.MOVE else None
    return dict(family=str(task.family), task=task.label, source=str(task.source),
                target=target, piece_type=chess.piece_name(piece.piece_type) if piece else "unknown",
                piece_color="white" if piece and piece.color else "black")


def resolve_piece_sets(config: ExpertBenchmarkConfig) -> list[str]:
    """The sets to measure: explicit, else verified sets plus the printed pack."""
    if config.piece_sets:
        return list(config.piece_sets)
    sets = available_piece_sets(verified_only=True)
    if PRINTED_SET not in sets:
        # the printed pack is only listed with ROBOT_PIECE_ASSETS set; asking for it
        # here keeps that requirement explicit instead of silently dropping it
        import os
        if os.environ.get("ROBOT_PIECE_ASSETS"):
            sets.append(PRINTED_SET)
    return sets


def build_env(config: ExpertBenchmarkConfig, piece_set: str) -> ChessSimEnv:
    return ChessSimEnv(EnvConfig(
        appearance=AppearanceConfig(piece_set=piece_set),
        control=ControlConfig(cameras=tuple(config.cameras)),
        randomization=RandomizationConfig(
            board_shift=config.board_shift,
            arm_joint_jitter=config.arm_joint_jitter,
            board_yaw=float(np.radians(config.board_yaw_deg)),
            board_side=config.board_side),
        ))


def run_benchmark(config: ExpertBenchmarkConfig = ExpertBenchmarkConfig(),
                  on_episode: Callable[[EpisodeRecord], None] | None = None,
                  ) -> BenchmarkReport:
    """Score the scripted expert over every seed and piece set in `config`."""
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    started = time.perf_counter()
    position = PositionConfig(min_extra_pieces=config.extra_pieces[0],
                              max_extra_pieces=config.extra_pieces[1])
    records: list[EpisodeRecord] = []
    for piece_set in resolve_piece_sets(config):
        env = build_env(config, piece_set)
        try:
            for seed in config.seeds:
                rng = random.Random(seed)
                for family, sampler, count in (
                        (TaskFamily.MOVE, MoveSampler(position=position), config.moves),
                        (TaskFamily.CAPTURE, CaptureSampler(position=position), config.captures)):
                    for index in range(count):
                        task = sampler.sample(env, rng, index)
                        meta = describe_task(task, env)
                        origin = tuple(env.board.origin)
                        arm_start = tuple(float(v) for v in env.layout.arm_start.to_array())
                        clock = time.perf_counter()
                        result = env.execute(task)
                        elapsed = time.perf_counter() - clock
                        record = EpisodeRecord(
                            piece_set=piece_set, seed=seed, success=result.success,
                            reason=str(result.reason) if result.reason else None,
                            placement_error_mm=result.placement_error * 1000,
                            right_piece=result.right_piece, upright=result.upright,
                            disturbed=tuple(str(s) for s in result.disturbed),
                            steps=result.steps, seconds=round(elapsed, 3),
                            board_origin=origin, arm_start=arm_start, **meta)
                        records.append(record)
                        if on_episode is not None:
                            on_episode(record)
        finally:
            env.close()
    return BenchmarkReport(config=config, started_at=started_at,
                           seconds=round(time.perf_counter() - started, 1),
                           episodes=tuple(records))
