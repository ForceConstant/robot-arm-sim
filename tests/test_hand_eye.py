"""tests/test_hand_eye.py — pin the solver's ability to recover a perturbed mount."""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.config import Camera  # noqa: E402
from chess_sim.product import layout_env_config  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from solve_hand_eye import (_compare, _view_observations,  # noqa: E402
                            calibration_views, face_origin,
                            marker_face_z, marker_layout, refine_hand_eye)

# The sim scene is rendered by the model-exported camera (fx 738.64). The bench
# hand-eye mount (fx 312.77) projects that scene to zero markers, so the mount
# this test perturbs and recovers must be the model's own.
CALIBRATION = (Path(__file__).resolve().parents[1]
               / "reports/block3_sim_calibration.json")


def _printed_pack() -> Path | None:
    """The printed piece pack this scene needs, from the env or .private."""
    env = os.environ.get("ROBOT_PIECE_ASSETS")
    if env and (Path(env) / "set.json").exists():
        return Path(env)
    root = Path(__file__).resolve().parents[1] / ".private"
    for candidate in sorted(root.glob("piece-assets-*")):
        if (candidate / "set.json").exists():
            return candidate
    return None


@pytest.fixture(autouse=True)
def _piece_assets(monkeypatch):
    """``layout_env_config("printed")`` needs a printed pack on the path."""
    pack = _printed_pack()
    if pack is None:
        pytest.skip("no printed piece pack (ROBOT_PIECE_ASSETS)")
    monkeypatch.setenv("ROBOT_PIECE_ASSETS", str(pack))


@pytest.mark.skipif(not CALIBRATION.exists(), reason="no current-layout calibration")
def test_refinement_recovers_a_perturbed_mount():
    calibration = WristCalibration.load(CALIBRATION)
    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset("8/8/8/8/8/8/8/8")
        board, model, data = env.board, env.model, env.data
        # Truth is the camera the scene actually carries, not the calibration
        # file: the block3 file's mount is 44.8 mm / 22 deg away from the
        # compiled one (measured 2026-11-25), so comparing to it pins the wrong
        # transform.  The solver returns T_flange_cam, so express the model
        # camera in the chain's flange frame.
        cam_id = model.camera(str(Camera.WRIST)).id
        grip_id = model.body("so101:gripper").id
        grip = np.eye(4)
        grip[:3, :3] = data.xmat[grip_id].reshape(3, 3)
        grip[:3, 3] = data.xpos[grip_id]
        cam = np.eye(4)
        cam[:3, :3] = data.cam_xmat[cam_id].reshape(3, 3)
        cam[:3, 3] = data.cam_xpos[cam_id]
        # The compiled gripper body is the frame the solve returns: comparing
        # against the chain frame adds a fixed 5 mm link offset.
        truth = np.linalg.inv(grip) @ cam
        fovy = float(model.cam_fovy[cam_id])
        height = 480
        fy = (height / 2.0) / math.tan(math.radians(fovy) / 2.0)
        intrinsics = (fy, fy, 320.0, 240.0)
        layout = marker_layout(board)
        marker_z = marker_face_z(board)
        origin = face_origin(board)
        views = calibration_views(env, calibration, board_origin=origin,
                                  board_yaw=board.yaw, marker_z=marker_z, count=8,
                                  layout=layout)
        observations = _view_observations(
            views, calibration, board_origin=origin,
            board_yaw=board.yaw, marker_z=marker_z, layout=layout,
            intrinsics=intrinsics)
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
