"""A failed placement descent must put the piece back, never release mid-air."""
import chess
import numpy as np
import pytest

from chess_sim import AppearanceConfig, ControlConfig, EnvConfig, MoveTask
from chess_sim import controller as C
from chess_sim.dual import DualArmEnv


@pytest.fixture(scope="module")
def env():
    e = DualArmEnv(EnvConfig(control=ControlConfig(cameras=()),
                            appearance=AppearanceConfig(camera_mast=False)))
    e.reset(chess.STARTING_FEN)
    yield e
    e.close()


def test_failed_place_descent_returns_piece_to_source(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    original = C.PickPlaceController._descend

    def failing(self, target, gap, offset, approach, on_step):
        if self.env.diagnostic_phase == "place_descend":
            return False                     # arrive nowhere near the square
        return original(self, target, gap, offset, approach, on_step)

    monkeypatch.setattr(C.PickPlaceController, "_descend", failing)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"))

    assert result.success is False
    assert result.waypoints_precise is False
    assert env.diagnostic_stall == "place_descend"
    # the piece is back on its own square, standing, and e4 is still empty
    slot = env._square_slot.get(chess.E2)
    assert slot is not None
    centre = np.array(env.board.square_center(chess.E2))
    piece = env.piece_position(slot)
    assert np.linalg.norm(piece[:2] - centre) < 0.03, (piece, centre)
    assert env.piece_upright(slot) > env.config.tolerances.upright_min
    assert chess.E4 not in env._square_slot


def test_clean_move_still_releases_normally(env):
    env.reset(chess.STARTING_FEN)
    env.diagnostic_stall = None
    result = env.execute(MoveTask(from_square="e2", to_square="e4"))
    assert result.success is True
    assert result.waypoints_precise is True
    assert env.diagnostic_stall is None
    assert chess.E4 in env._square_slot
