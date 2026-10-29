import numpy as np
import mujoco

from chess_sim.config import Camera, EnvConfig
from chess_sim.env import ChessSimEnv
from chess_sim.vision import detect_piece_base
from chess_sim.vision_geometry import CameraCalibration, Localization
from chess_sim.tasks import MoveTask


def test_foreground_detector_localizes_off_center_piece_and_abstains_when_empty():
    env = ChessSimEnv(EnvConfig())
    try:
        env.reset("8/8/8/8/8/8/8/8 w - - 0 1")
        reference = env.render(Camera.TOP).copy()
        env.reset("8/8/8/8/4P3/8/8/8 w - - 0 1")
        slot = env.slot_at("e4")
        nominal = np.asarray(env.board.square_center("e4"))
        offset = np.asarray((0.004, -0.003))
        yaw = np.deg2rad(47.0)
        env.set_piece_pose(
            slot,
            [nominal[0] + offset[0] - slot.geometry.center[0],
             nominal[1] + offset[1] - slot.geometry.center[1],
             env.board.top - slot.geometry.bottom_offset + 0.0015],
            [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)],
            zero_velocity=True)
        mujoco.mj_forward(env.model, env.data)
        frame = env.render(Camera.TOP).copy()
        calibration = CameraCalibration.from_mujoco(
            env.model, env.data, str(Camera.TOP), frame.shape[1], frame.shape[0])

        detected = detect_piece_base(
            frame, reference, calibration, tuple(nominal), env.board.top,
            slot.geometry.height, search_radius_m=0.75 * env.board.square)
        assert detected is not None
        assert np.linalg.norm(np.asarray(detected.world_xy) - (nominal + offset)) < 0.002
        assert detected.acceptable(max_sigma_m=0.004, min_confidence=0.7)

        # A renderer-derived arm mask can remove robot geometry that changed
        # between reference and observation, while preserving RGB localization.
        arm_geometry_ids = []
        for geom_id in range(env.model.ngeom):
            body_id = int(env.model.geom_bodyid[geom_id])
            body_name = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name and body_name.startswith("so101:"):
                arm_geometry_ids.append(geom_id)
        arm_mask = np.isin(env.render_segmentation(Camera.TOP)[:, :, 0],
                           arm_geometry_ids)
        fully_occluded = detect_piece_base(
            frame, reference, calibration, tuple(nominal), env.board.top,
            slot.geometry.height, search_radius_m=0.75 * env.board.square,
            exclusion_mask=np.ones(frame.shape[:2], dtype=bool))
        assert fully_occluded is None
        assert arm_mask.shape == frame.shape[:2]
        assert 0 < int(arm_mask.sum()) < arm_mask.size

        absent = detect_piece_base(
            reference, reference, calibration, tuple(nominal), env.board.top,
            slot.geometry.height, search_radius_m=0.75 * env.board.square)
        assert absent is None
    finally:
        env.close()


def test_localized_execution_reobserves_and_safely_abstains_on_lost_target():
    env = ChessSimEnv(EnvConfig())
    try:
        env.reset("8/8/8/8/4P3/8/8/8 w - - 0 1")
        slot = env.slot_at("e4")
        source = np.asarray(env.board.square_center("e4"))
        calls = []

        def lost_target():
            calls.append(True)
            return None

        result = env.execute_localized(
            MoveTask(from_square="e4", to_square="e5"),
            Localization(tuple(source), (320.0, 240.0), 0.99, 0.001),
            reobserve_source=lost_target)
        assert calls == [True]
        assert not result.success
        assert env.diagnostic_stall == "vision_reobserve_abstain"
        assert np.linalg.norm(env.piece_position(slot)[:2] - source) < 0.002
    finally:
        env.close()


def test_localized_execution_abstains_when_reobservations_disagree():
    env = ChessSimEnv(EnvConfig())
    try:
        env.reset("8/8/8/8/4P3/8/8/8 w - - 0 1")
        slot = env.slot_at("e4")
        source = np.asarray(env.board.square_center("e4"))
        initial = Localization(tuple(source), (320.0, 240.0), 0.99, 0.001)
        inconsistent = Localization(tuple(source + np.array([0.010, 0.0])),
                                    (330.0, 240.0), 0.99, 0.001)
        result = env.execute_localized(
            MoveTask(from_square="e4", to_square="e5"), initial,
            reobserve_source=lambda: inconsistent)
        assert not result.success
        assert env.diagnostic_stall == "vision_reobserve_inconsistent"
        assert np.linalg.norm(env.piece_position(slot)[:2] - source) < 0.002
    finally:
        env.close()
