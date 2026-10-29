import numpy as np

from chess_sim.config import Camera, EnvConfig
from chess_sim.env import ChessSimEnv
from chess_sim.vision_geometry import CameraCalibration, localize_piece_base


def test_mujoco_camera_round_trip_maps_pixel_to_board_plane():
    env = ChessSimEnv(EnvConfig())
    try:
        env.reset("8/8/8/8/4P3/8/8/4K2k w - - 0 1")
        import mujoco
        mujoco.mj_forward(env.model, env.data)
        calibration = CameraCalibration.from_mujoco(
            env.model, env.data, str(Camera.TOP), 640, 480)
        # Exercise board registration across the full usable plane, not only
        # one point at the optical axis.
        for square in ("a1", "h1", "a8", "h8", "e4"):
            xy = env.board.square_center(square)
            pixel = calibration.project((*xy, env.board.top))
            assert pixel is not None
            detection = localize_piece_base(calibration, pixel, env.board.top,
                                            confidence=0.99, pixel_sigma=0.25)
            assert detection is not None
            assert np.linalg.norm(np.asarray(detection.world_xy) - xy) < 1e-8
            assert detection.acceptable(max_sigma_m=0.004)
    finally:
        env.close()


def test_localization_rejects_out_of_frame_and_exposes_low_confidence():
    calibration = CameraCalibration(640, 480, 500, 500, 320, 240,
                                    (0.0, 0.0, 1.0),
                                    (-1.0, 0.0, 0.0, 0.0, -1.0, 0.0,
                                     0.0, 0.0, 1.0))
    assert localize_piece_base(calibration, (-1, 10), 0.0, 0.99) is None
    localization = localize_piece_base(calibration, (320, 240), 0.0,
                                       confidence=0.2)
    assert localization is not None
    assert not localization.acceptable()
