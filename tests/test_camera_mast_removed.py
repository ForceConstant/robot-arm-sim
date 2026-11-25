"""The old yellow/black overhead camera mast must never be built again.

Brian, 2026-11-25: baseline sim images showed a yellow/black camera mast that
does not belong in the scene any more.  The fix was to remove the ``camera_mast``
option entirely, not just default it to False, so no caller can turn it back on.
These tests pin both halves of that contract: the field is gone, and the
compiled default scene contains no mast geometry while the lens stays put.
"""
import chess
import mujoco
import pytest

from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
from chess_sim.env import ChessSimEnv
from chess_sim.scene import build_scene, mast_lens

MAST_GEOMS = ("mast_plate", "mast_lower", "mast_upper", "mast_camera")


def test_camera_mast_option_is_gone():
    assert "camera_mast" not in set(AppearanceConfig.model_fields)
    with pytest.raises(Exception):
        AppearanceConfig(camera_mast=True)


def test_default_scene_has_no_mast_geometry():
    spec = build_scene(BoardConfig(), AppearanceConfig())
    model = spec.compile()
    names = {model.geom(i).name for i in range(model.ngeom)}
    assert not (set(MAST_GEOMS) & names), sorted(set(MAST_GEOMS) & names)


def test_default_env_has_no_mast_and_keeps_the_lens():
    env = ChessSimEnv(EnvConfig(control=type(EnvConfig().control)(cameras=())))
    try:
        names = {env.model.geom(i).name for i in range(env.model.ngeom)}
        assert not (set(MAST_GEOMS) & names), sorted(set(MAST_GEOMS) & names)
        expected = mast_lens(env.board)
        assert env.model.camera("top").pos == pytest.approx(expected, abs=1e-6)
    finally:
        env.close()
