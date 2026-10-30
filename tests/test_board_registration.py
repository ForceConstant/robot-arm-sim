import math
import random

import numpy as np

from chess_sim.board_registration import checker_corner_response, register_board
from chess_sim.config import Camera, EnvConfig, RandomizationConfig
from chess_sim.env import ChessSimEnv
from chess_sim.vision_geometry import CameraCalibration


def _register(env, camera=Camera.TOP):
    frame = env.render(camera).copy()
    calibration = CameraCalibration.from_mujoco(
        env.model, env.data, str(camera), frame.shape[1], frame.shape[0])
    return register_board(frame, calibration, env.board.top), frame, calibration


def test_registration_recovers_the_nominal_board():
    env = ChessSimEnv(EnvConfig())
    try:
        env.reset("8/8/8/8/8/8/8/8 w - - 0 1")
        registration, _, _ = _register(env)
        assert registration is not None
        origin_error = math.hypot(registration.origin[0] - env.board.origin[0],
                                  registration.origin[1] - env.board.origin[1])
        assert origin_error < 0.005
        assert abs(registration.yaw - env.board.yaw) < math.radians(1.0)
        assert abs(registration.square - env.board.square) < 0.0006
        assert registration.score > 0.5
        assert registration.edge_score > 0.4
        assert registration.coverage == 1.0
    finally:
        env.close()


def test_registration_tracks_a_shifted_turned_board_and_random_camera():
    config = EnvConfig(randomization=RandomizationConfig(
        board_shift=0.008, board_yaw=math.radians(5.0), board_side=0.020,
        arm_joint_jitter=0.05, camera=True))
    env = ChessSimEnv(config)
    try:
        env.reset("8/8/8/8/8/8/8/8 w - - 0 1", rng=random.Random(123))
        registration, _, _ = _register(env)
        assert registration is not None
        # The board moved off the arm's centre line and the camera was redrawn;
        # registration must follow the image, not the nominal config.
        assert math.hypot(registration.origin[0] - env.board.origin[0],
                          registration.origin[1] - env.board.origin[1]) < 0.008
        assert abs(registration.yaw - env.board.yaw) < math.radians(1.5)
        assert abs(registration.square - env.board.square) < 0.001
    finally:
        env.close()


def test_registration_rejects_a_frame_without_a_checkerboard():
    env = ChessSimEnv(EnvConfig())
    try:
        env.reset("8/8/8/8/8/8/8/8 w - - 0 1")
        frame = env.render(Camera.TOP).copy()
        calibration = CameraCalibration.from_mujoco(
            env.model, env.data, str(Camera.TOP), frame.shape[1], frame.shape[0])
        blank = np.zeros_like(frame)
        assert register_board(blank, calibration, env.board.top) is None
    finally:
        env.close()


def test_corner_response_is_high_at_a_saddle_and_zero_on_a_flat_patch():
    flat = np.full((64, 64), 120, dtype=np.float32)
    assert checker_corner_response(flat).max() < 1e-6
    yy, xx = np.indices((64, 64))
    checker = (((yy >= 32) ^ (xx >= 32)).astype(np.float32) * 180 + 40)
    response = checker_corner_response(checker)
    assert response[32, 32] > 0.8
