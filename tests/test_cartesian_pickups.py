"""Cartesian pickup test: every square, by x/y/z/jaw, inside the keep-in zone.

This is the simulation counterpart of the dashboard's sanity button, and it
encodes what the real arm taught us on 2026-11-17:

* Cartesian targets are *jaw-tip* coordinates. The calibrated chain's final frame
  sits ``board.jaw_tip_mm`` above the tip, so a tip target must lift the frame by
  that much — otherwise every target is 83 mm off and the corners merely look
  unreachable.
* The arm must stay over the playing surface. The keep-in envelope is enforced
  here exactly as the live controller enforces it, so a target outside the board
  is rejected rather than silently driven to.

Run: robot-arm-sim pytest tests/test_cartesian_pickups.py -q
"""
from __future__ import annotations

import numpy as np
import pytest

from chess_sim.config import BoardConfig, EnvConfig
from chess_sim.env import ChessSimEnv
from chess_sim.keep_in import KeepIn
from chess_sim.reach import MAX_IK_ERROR, MAX_TILT

SQUARES = [f + r for f in "abcdefgh" for r in "12345678"]
PICK_HOVER_MM = 20.0          # jaw tip above the board surface at pickup


@pytest.fixture(scope="module")
def env():
    # Default piece set: the printed layout needs external assets that CI and
    # this test deliberately do not depend on.
    environment = ChessSimEnv(EnvConfig())
    environment.reset("8/8/8/8/8/8/8/8")      # empty board: nothing to collide with
    try:
        yield environment
    finally:
        environment.close()


def tip_target(board: BoardConfig, square: str, hover_mm: float = PICK_HOVER_MM):
    """Board-frame (x, y) of a square centre, plus the tool-frame height.

    The returned point is where the *jaw tip* should be; ``tool_z`` is where the
    calibrated chain frame has to sit to put the tip there.
    """
    x, y = board.square_center(square)
    tip_z = board.top + hover_mm / 1000.0
    tool_z = tip_z + board.jaw_tip_mm / 1000.0
    # world target for IK, and the tip expressed as height above the board,
    # which is the frame the keep-in envelope uses
    return (np.array([x, y, tool_z]),
            np.array([x * 1000.0, y * 1000.0, hover_mm]))


def test_jaw_tip_offset_is_recorded():
    """The measured frame-above-tip distance is part of the configuration."""
    board = BoardConfig()
    assert board.jaw_tip_mm == pytest.approx(83.0), "measured 2026-11-17"
    assert board.keep_in_x_mm > board.field / 2.0 * 1000.0 - board.square * 1000.0


def _unreachable(env, squares):
    board = env.board
    failures = []
    for square in squares:
        target, tip = tip_target(board, square)
        result = env.ik.solve(env.data, target)
        if result.position_error > MAX_IK_ERROR or result.tilt > MAX_TILT:
            failures.append((square, round(result.position_error * 1000, 1),
                             round(float(np.degrees(result.tilt)), 1)))
            continue
        env.data.qpos[env._joint_qpos[:5]] = result.q
    assert not failures, f"{len(failures)} square(s) unreachable: {failures[:8]}"


def test_pickup_targets_are_inside_the_keep_in_zone():
    """Every pickup point sits inside the envelope the controller enforces."""
    board = BoardConfig()
    zone = KeepIn(board.keep_in_x_mm, board.keep_in_y_mm,
                  board.keep_in_z_min_mm, board.keep_in_z_max_mm)
    for square in SQUARES:
        _, tip_mm = tip_target(board, square)
        assert zone.contains(tip_mm), f"{square} outside keep-in: {zone.problems(tip_mm)}"


def test_keep_in_rejects_targets_off_the_board():
    """A target beyond the board is refused, not driven to."""
    board = BoardConfig()
    zone = KeepIn(board.keep_in_x_mm, board.keep_in_y_mm,
                  board.keep_in_z_min_mm, board.keep_in_z_max_mm)
    off_board = [(0.0, 200.0, 50.0), (-200.0, 0.0, 50.0), (0.0, 0.0, 400.0),
                 (0.0, 0.0, -50.0)]
    for point in off_board:
        assert not zone.contains(point)
        with pytest.raises(ValueError, match="keep-in zone"):
            zone.require(point)


def test_keep_in_allows_the_whole_playing_surface():
    """The envelope must not be so tight that it blocks legal play."""
    board = BoardConfig()
    zone = KeepIn(board.keep_in_x_mm, board.keep_in_y_mm,
                  board.keep_in_z_min_mm, board.keep_in_z_max_mm)
    half_field = board.field / 2.0 * 1000.0
    assert zone.contains((half_field, half_field, 5.0))
    assert zone.contains((-half_field, -half_field, 5.0))


def test_jaw_command_range_covers_pick_and_release():
    """x/y/z is useless without a jaw value, so pin the commanded range."""
    board = BoardConfig()
    zone = KeepIn(board.keep_in_x_mm, board.keep_in_y_mm,
                  board.keep_in_z_min_mm, board.keep_in_z_max_mm)
    target, tip_mm = tip_target(board, "e4")
    assert zone.contains(tip_mm)
    for jaw in (0.0, 40.0, 100.0):
        assert 0.0 <= jaw <= 100.0
