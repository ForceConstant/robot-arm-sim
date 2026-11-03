from chess_sim.single import SingleFollowerEnv
from chess_sim import EnvConfig,ControlConfig,AppearanceConfig
import chess


def test_one_follower_handles_both_colours_without_a_hidden_second_arm():
    env=SingleFollowerEnv(EnvConfig(control=ControlConfig(cameras=()),
                                  appearance=AppearanceConfig(camera_mast=False)))
    try:
        assert env.model.nu==6 and list(env._contexts)==['white']
        assert not any(env.model.body(i).name.startswith('black:') for i in range(env.model.nbody))
        env.select_arm('black')
        assert env.active_side=='white'
        assert env.ik is env._contexts['white']['ik']
        assert env.home_pose()[0]==0
        env.reset()
        assert len(env._square_slot)==32 and env._capture_count=={'white':0}
        assert env.full_board_error(chess.Board()) is None
        assert env.full_board_error(chess.Board(None)) is not None
    finally:env.close()
