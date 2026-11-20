"""The board's facing convention, pinned as numbers.

Brian, 2026-11-20: the arm always plays black. The sim had the board set down
for white, so rank 1 - white's back rank - was the rank nearest the arm, every
square read one half-turn from where it should, and nothing in the suite
noticed: the tests that looked at board geometry read it back through the same
`square_center` they were checking. These tests are the ones that would have
caught it, because they compare the board's frame against the arm's position.
"""
import math
import random

import chess
import numpy as np
import pytest

from chess_sim import charuco_face
from chess_sim.config import ARM_PLAYS, BOARD_TURN, BoardConfig, SQUARES
from chess_sim.positions import ARM_COLOR, PositionConfig


@pytest.fixture(scope="module")
def board():
    return BoardConfig()


def test_the_arm_plays_black(board):
    assert ARM_PLAYS == "black"
    assert board.facing == "black"
    assert math.isclose(BOARD_TURN, math.pi)


def test_arm_base_falls_on_the_black_half(board):
    """Black's back rank is the one nearest the arm, by distance, not by name."""
    _, arm_y, _ = board.arm_base
    near_y = board.square_center("a8")[1]
    far_y = board.square_center("a1")[1]

    def distance(y):
        return abs(y - arm_y)

    assert distance(near_y) < distance(far_y), (near_y, far_y, arm_y)
    # rank 8 sits between the arm and the board centre; rank 1 is beyond it
    assert np.sign(near_y - board.origin[1]) == np.sign(arm_y - board.origin[1])
    assert np.sign(far_y - board.origin[1]) != np.sign(arm_y - board.origin[1])
    # metres -> millimetres: 114 mm to rank 8 and 310 mm to rank 1 (border 10 mm print)
    assert abs(near_y - arm_y) * 1000.0 == pytest.approx(114.0, abs=1.0)
    assert abs(far_y - arm_y) * 1000.0 == pytest.approx(310.0, abs=1.0)


def test_every_square_turned_half_a_turn_maps_to_its_opposite(board):
    """a1<->h8 for all 64: a pure 180 degrees about the board centre."""
    for square in SQUARES:
        name = str(square)
        opposite = f"{chr(ord('h') - (ord(name[0]) - ord('a')))}{9 - int(name[1])}"
        x, y = board.square_center(name)
        ox, oy = board.square_center(opposite)
        assert x == pytest.approx(2 * board.origin[0] - ox, abs=1e-9)
        assert y == pytest.approx(2 * board.origin[1] - oy, abs=1e-9)


def test_sampled_boards_stay_face_up_for_the_arm():
    """A random board is set down turned, never turned back to white."""
    for seed in range(25):
        sampled = BoardConfig.sample(np.random.default_rng(seed))
        assert sampled.facing == "black", (seed, math.degrees(sampled.yaw))
        assert abs(math.degrees(sampled.yaw) - 180.0) <= 12.0 + 1e-6


def test_marker_ids_run_from_b1_up_the_ranks():
    """Id order is part of the print: 5 is b1, the first white square on rank 1."""
    assert charuco_face.square_of(5) == (1, 0)
    assert charuco_face.square_of(36) == (6, 7)
    ids = [mid for mid in range(5, 37)]
    squares = [charuco_face.square_of(mid) for mid in ids]
    assert squares == sorted(squares, key=lambda cr: (cr[1], cr[0]))
    # every marker sits on a light square, where the print puts it
    assert all((col + rank) % 2 == 1 for col, rank in squares)


def test_practice_positions_are_the_arm_to_move():
    assert ARM_COLOR == chess.BLACK
    config = PositionConfig()
    for seed in range(10):
        position = config.sample(random.Random(seed))
        assert position.turn == ARM_COLOR
        move = next(m for m in position.legal_moves if not position.is_castling(m))
        with_move = config.sample_with_move(random.Random(seed), move)
        if with_move is not None:
            assert with_move.turn == ARM_COLOR
