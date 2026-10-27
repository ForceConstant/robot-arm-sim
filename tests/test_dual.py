import chess
import pytest

from chess_sim import AppearanceConfig, ControlConfig, EnvConfig
from chess_sim import gripper
from chess_sim.dual import DualArmEnv


@pytest.fixture(scope="module")
def env():
    e = DualArmEnv(EnvConfig(control=ControlConfig(cameras=()),
                            appearance=AppearanceConfig(camera_mast=False)))
    yield e
    e.close()


def test_two_arms_and_collision_free_home(env):
    env.reset()
    assert env.model.nu == 12
    assert len(env._square_slot) == 32
    white, black = env._contexts["white"], env._contexts["black"]
    assert not set(white["qpos"]) & set(black["qpos"])
    assert white["board"].arm_base[1] == -black["board"].arm_base[1]
    for c in env.data.contact:
        names = [env.model.body(int(env.model.geom_bodyid[g])).name for g in [c.geom1, c.geom2]]
        cross = any(n.startswith("so101:") for n in names) and any(n.startswith("black:") for n in names)
        assert not (cross and c.dist < -0.001), names


@pytest.mark.skipif(gripper.CUSTOM, reason='Stock-tool manipulation regression; new CAD tool grasp calibration is explicitly unqualified')
def test_stock_both_arms_physically_lift_and_place(env):
    env.reset()
    game = chess.Board()
    for side, uci in [("white", "e2e4"), ("black", "e7e5")]:
        env.select_arm(side)
        slot = env.slot_at(uci[:2])
        heights = []
        result = env.move(uci[:2], uci[2:], on_step=lambda _: heights.append(env.piece_position(slot)[2]))
        assert result.success, result
        assert max(heights) - min(heights) > 0.04, "Piece must actually lift, not slide/teleport"
        game.push_uci(uci)
        env.position = game.copy()
        env.park()
        assert env.full_board_error(game) is None
    assert env.slot_at("e2") is None and env.slot_at("e7") is None


def test_black_mount_has_no_white_empirical_calibration(env):
    assert not env._contexts["black"]["board"].calibrated
