import chess
import os
import pytest

from chess_sim import AppearanceConfig, ControlConfig, EnvConfig, gripper
from chess_sim.dual import DualArmEnv


@pytest.fixture(scope="module")
def env():
    e = DualArmEnv(EnvConfig(control=ControlConfig(cameras=()),
                            appearance=AppearanceConfig(camera_mast=False,
                                piece_set=os.environ.get("ROBOT_PIECE_SET", "default"))))
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


@pytest.mark.skipif(not gripper.CUSTOM, reason="crowded manipulation regression is for the deployed CAD tool")
def test_both_arms_physically_lift_and_place(env):
    env.reset()
    game = chess.Board()
    # Opening pawn pair plus real-contact transfers of each other piece class.
    # These six moves exercise both followers after the board has become more
    # crowded; every case keeps the shared physical success/neighbor criteria.
    cases = [("white", "e2e4"), ("black", "e7e5"),
             ("white", "g1f3"), ("black", "g8f6"),
             ("white", "f1c4"), ("black", "f8c5")]
    for side, uci in cases:
        env.select_arm(side)
        slot = env.slot_at(uci[:2])
        assert slot is not None, f"missing source for {side} {uci}"
        heights = []
        result = env.move(uci[:2], uci[2:], on_step=lambda _: heights.append(env.piece_position(slot)[2]))
        assert result.success, f"{side} {uci}: {result}"
        assert max(heights) - min(heights) > 0.04, "Piece must actually lift, not slide/teleport"
        assert not result.disturbed, f"{side} {uci} disturbed neighbors: {result.disturbed}"
        game.push_uci(uci)
        env.position = game.copy()
        env.park()
        assert env.full_board_error(game) is None
    assert env.slot_at("e2") is None and env.slot_at("e7") is None
    assert env.slot_at("g1") is None and env.slot_at("g8") is None
    assert env.slot_at("f1") is None and env.slot_at("f8") is None


@pytest.mark.skipif(not gripper.CUSTOM, reason="CAD printed-knight regression needs the private tool pack")
def test_black_corner_knight_profile_closure_regression(env):
    board = chess.Board()
    board.push_uci("e2e4")
    board.push_uci("e7e5")
    board.turn = chess.BLACK
    env.reset(board.fen())
    env.select_arm("black")
    env.position = board.copy()
    slot = env.slot_at("b8")
    assert slot is not None
    assert slot.geometry.grasp_squeeze == pytest.approx(0.009)
    result = env.move("b8", "c6")
    assert result.success, result
    assert str(result.picked) == "b8"
    assert result.upright and not result.disturbed
    expected = board.copy()
    expected.push_uci("b8c6")
    env.position = expected
    assert env.full_board_error(expected) is None


def test_randomized_layout_is_reproducible_and_bounded(env):
    import random
    nominal = env.config.board.origin
    rest = env.home_pose()
    env.reset(rng=random.Random(77123))
    first_origin = env.layout.board_origin
    first_start = env.layout.arm_start.to_array().copy()
    env.reset(rng=random.Random(77123))
    assert env.layout.board_origin == first_origin
    assert (env.layout.arm_start.to_array() == first_start).all()
    assert all(abs(a - b) <= 0.0050001 for a, b in zip(first_origin, nominal))
    assert max(abs(first_start - rest)) <= 0.1000001
    for side in ("white", "black"):
        env.select_arm(side)
        joints = env.arm_joint_positions()
        # The servo settles within its measured sub-milliradian step response.
        assert max(abs(joints - first_start)) < 0.001


def test_black_mount_has_no_white_empirical_calibration(env):
    assert not env._contexts["black"]["board"].calibrated
