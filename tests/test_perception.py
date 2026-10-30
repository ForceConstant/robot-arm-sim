import math

import numpy as np
import pytest

from chess_sim.board_registration import register_board
from chess_sim.config import Camera, EnvConfig
from chess_sim.env import ChessSimEnv
from chess_sim.perception import (CLASSES, FEATURE_COUNT, FEATURE_NAMES, MODEL_PATH,
                                  PerceptionModel, board_patches, calibrate_threshold,
                                  expected_background, predict_board,
                                  square_features)
from chess_sim.vision_geometry import CameraCalibration


def _calibrate(env, frame):
    calibration = CameraCalibration.from_mujoco(
        env.model, env.data, str(Camera.TOP), frame.shape[1], frame.shape[0])
    registration = register_board(frame, calibration, env.board.top)
    assert registration is not None
    return calibration, registration


def _empty_reference(env):
    env.reset("8/8/8/8/8/8/8/8 w - - 0 1")
    frame = env.render(Camera.TOP).copy()
    calibration, registration = _calibrate(env, frame)
    return frame, calibration, registration


@pytest.mark.skipif(not MODEL_PATH.exists(), reason="perception model not trained")
def test_shipped_model_contract():
    model = PerceptionModel.load(MODEL_PATH)
    assert model.feature_count == FEATURE_COUNT == len(FEATURE_NAMES)
    assert model.classes == CLASSES
    assert 0.4 <= model.confidence_threshold <= 0.99
    probabilities = model.probabilities(np.zeros((2, FEATURE_COUNT), dtype=np.float32))
    assert probabilities.shape == (2, 4)
    assert np.allclose(probabilities.sum(axis=1), 1.0)


def test_features_have_the_documented_shape_and_stay_finite():
    env = ChessSimEnv(EnvConfig())
    try:
        reference, calibration, registration = _empty_reference(env)
        env.reset("8/8/8/8/4P3/8/8/8 w - - 0 1")
        frame = env.render(Camera.TOP).copy()
        patches, valid = board_patches(frame, calibration, registration)
        reference_patches, _ = board_patches(reference, calibration, registration)
        assert patches.shape == (64, 16, 16, 3)
        assert valid.shape == (64,)
        expected = expected_background(patches, valid)
        features = square_features(patches, valid, expected=expected,
                                   reference=reference_patches)
        assert features.shape == (64, FEATURE_COUNT)
        assert np.isfinite(features).all()
        assert np.all(valid > 0.9)
    finally:
        env.close()


@pytest.mark.skipif(not MODEL_PATH.exists(), reason="perception model not trained")
def test_empty_board_is_mostly_answered_empty():
    env = ChessSimEnv(EnvConfig())
    try:
        reference, calibration, registration = _empty_reference(env)
        model = PerceptionModel.load(MODEL_PATH)
        predictions = predict_board(reference, calibration, registration, model,
                                    reference_frame=reference)
        answered = [p for p in predictions if p.label != "unknown"]
        assert len(answered) / 64 > 0.5
        assert sum(p.label == "empty" for p in answered) / len(answered) > 0.9
    finally:
        env.close()


@pytest.mark.skipif(not MODEL_PATH.exists(), reason="perception model not trained")
def test_named_piece_colours_are_recovered():
    env = ChessSimEnv(EnvConfig())
    try:
        reference, calibration, registration = _empty_reference(env)
        env.reset("8/8/8/8/4P3/8/8/8 w - - 0 1")
        frame = env.render(Camera.TOP).copy()
        model = PerceptionModel.load(MODEL_PATH)
        predictions = predict_board(frame, calibration, registration, model,
                                    reference_frame=reference)
        by_square = {p.square: p for p in predictions}
        assert by_square[28].label == "white"          # e4
        assert by_square[28].confidence >= model.confidence_threshold
        assert by_square[36].label in ("empty", "unknown")  # e5
    finally:
        env.close()


@pytest.mark.skipif(not MODEL_PATH.exists(), reason="perception model not trained")
def test_black_piece_is_not_called_white():
    env = ChessSimEnv(EnvConfig())
    try:
        reference, calibration, registration = _empty_reference(env)
        env.reset("8/8/8/8/4p3/8/8/8 w - - 0 1")
        frame = env.render(Camera.TOP).copy()
        model = PerceptionModel.load(MODEL_PATH)
        predictions = predict_board(frame, calibration, registration, model,
                                    reference_frame=reference)
        by_square = {p.square: p for p in predictions}
        assert by_square[28].label in ("black", "unknown")
        assert by_square[28].label != "white"
    finally:
        env.close()


@pytest.mark.skipif(not MODEL_PATH.exists(), reason="perception model not trained")
def test_a_fully_dark_observation_abstains_rather_than_naming_pieces():
    env = ChessSimEnv(EnvConfig())
    try:
        reference, calibration, registration = _empty_reference(env)
        model = PerceptionModel.load(MODEL_PATH)
        dark = np.zeros((calibration.height, calibration.width, 3), dtype=np.uint8)
        predictions = predict_board(dark, calibration, registration, model,
                                    reference_frame=reference)
        assert sum(p.label == "unknown" for p in predictions) >= 48
        assert not any(p.label in ("white", "black") and p.confidence > 0.9
                       for p in predictions)
    finally:
        env.close()


def test_confidence_threshold_calibration_honours_an_error_budget():
    probabilities = np.array([[0.9, 0.05, 0.04, 0.01],
                              [0.5, 0.3, 0.15, 0.05],
                              [0.2, 0.6, 0.1, 0.1],
                              [0.05, 0.05, 0.85, 0.05]], dtype=float)
    labels = np.array([0, 0, 1, 3])
    threshold, operating = calibrate_threshold(probabilities, labels,
                                               max_confident_error=0.0)
    assert 0.4 <= threshold <= 0.99
    assert operating["confident_errors"] == 0
    assert operating["coverage"] > 0
