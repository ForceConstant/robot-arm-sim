"""tests/test_hand_eye.py — pin the solver's ability to recover a perturbed mount."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.product import layout_env_config  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from solve_hand_eye import (MARKER_LAYOUT, _compare, _view_observations,  # noqa: E402
                            calibration_views, refine_hand_eye)

CALIBRATION = Path(__file__).resolve().parents[1] / ".private/phase11d/calibration.json"


@pytest.mark.skipif(not CALIBRATION.exists(), reason="no current-layout calibration")
def test_refinement_recovers_a_perturbed_mount():
    calibration = WristCalibration.load(CALIBRATION)
    truth = np.asarray(calibration.camera_in_tool, dtype=float)
    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset("8/8/8/8/8/8/8/8")
        board = env.board
        marker_z = board.table_top + fiducials.marker_height_above_table(board.mat)
        views = calibration_views(env, calibration, board_origin=board.origin,
                                  board_yaw=board.yaw, marker_z=marker_z, count=8)
        observations = _view_observations(
            views, calibration, board_origin=board.origin,
            board_yaw=board.yaw, marker_z=marker_z)
        assert len(observations) >= 6

        prior = truth.copy()
        prior[:3, :3] = Rotation.from_rotvec(
            [math.radians(10.0), 0.0, 0.0]).as_matrix() @ prior[:3, :3]
        prior[:3, 3] = prior[:3, 3] + np.array([0.0, 0.015, 0.0])

        recovered = refine_hand_eye(observations, prior)
        error = _compare(recovered[:3, :3], recovered[:3, 3], truth)
        assert error["translation_mm"] <= 1.0, error
        assert error["rotation_deg"] <= 0.5, error
    finally:
        env.close()
