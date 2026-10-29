"""The expert-baseline report: counts, slices and the numbers it prints.

These exercise the report itself, not the simulator: every episode record is
built by hand so a wrong field name or a dropped slice fails here in
milliseconds instead of twenty minutes into a baseline run.
"""
import os

import pytest

from chess_sim.benchmark import (BenchmarkReport, EpisodeRecord, ExpertBenchmarkConfig,
                                 Summary, resolve_piece_sets)
from chess_sim.rollout import wilson_interval


def episode(**overrides) -> EpisodeRecord:
    values = dict(
        piece_set="printed", seed=1, family="move", task="e2e4", source="e2",
        target="e4", piece_type="pawn", piece_color="white", success=True, reason=None,
        placement_error_mm=2.0, right_piece=True, upright=True, disturbed=(), steps=90,
        seconds=2.0, board_origin=(0.0, 0.0), arm_start=(0.0,) * 6)
    values.update(overrides)
    return EpisodeRecord(**values)


def report(*episodes) -> BenchmarkReport:
    return BenchmarkReport(config=ExpertBenchmarkConfig(), started_at="2026-10-29T00:00:00",
                           seconds=1.0, episodes=tuple(episodes))


def test_summary_counts_rate_median_and_reasons():
    summary = Summary.of([
        episode(placement_error_mm=1.0),
        episode(placement_error_mm=5.0),
        episode(success=False, placement_error_mm=20.0, upright=False,
                reason="piece fell", right_piece=False, disturbed=("f7",)),
    ])
    assert summary.episodes == 3
    assert summary.successes == 2
    assert summary.rate == pytest.approx(2 / 3)
    assert summary.interval == wilson_interval(2, 3)
    assert summary.median_error_mm == pytest.approx(5.0)
    assert summary.right_piece == 2
    assert summary.disturbed == 1
    assert summary.reasons == {"piece fell": 1}


def test_summary_of_nothing_is_empty_not_an_error():
    summary = Summary.of([])
    assert (summary.episodes, summary.successes, summary.rate) == (0, 0, 0.0)
    assert summary.median_error_mm == 0.0


def test_report_slices_by_seed_family_piece_and_set():
    rows = [episode(seed=1), episode(seed=2, success=False, reason="placement error"),
            episode(seed=1, family="capture", task="xe5", target=None, piece_type="knight",
                    piece_set="default")]
    data = report(*rows)
    assert set(data.by_seed()) == {1, 2}
    assert set(data.by_family()) == {"move", "capture"}
    assert set(data.by_piece_type()) == {"pawn", "knight"}
    assert set(data.by_piece_set()) == {"printed", "default"}
    assert set(data.by_seed_family()) == {(1, "move"), (1, "capture"), (2, "move")}
    assert set(data.by_piece_set_family()) == {("printed", "move"), ("default", "capture")}
    assert data.by_seed()[1].episodes == 2


def test_markdown_reports_every_slice_and_seed():
    text = report(episode(seed=271828), episode(seed=314159, success=False,
                                               reason="placement error")).markdown()
    assert "Overall: 1/2" in text
    for heading in ("By task family", "By piece type", "By seed", "By piece set",
                    "By piece set and family", "By piece set and piece type",
                    "By seed and family", "Failure reasons"):
        assert heading in text
    assert "271828" in text and "314159" in text


def test_json_round_trips_the_report():
    data = report(episode()).model_dump_json()
    assert '"piece_set":"printed"' in data


def test_resolve_piece_sets_adds_printed_only_when_assets_are_set(monkeypatch):
    monkeypatch.delenv("ROBOT_PIECE_ASSETS", raising=False)
    assert resolve_piece_sets(ExpertBenchmarkConfig()) == ["default"]
    monkeypatch.setenv("ROBOT_PIECE_ASSETS", "/nonexistent")
    assert resolve_piece_sets(ExpertBenchmarkConfig()) == ["default", "printed"]
    explicit = ExpertBenchmarkConfig(piece_sets=("default",))
    assert resolve_piece_sets(explicit) == ["default"]
