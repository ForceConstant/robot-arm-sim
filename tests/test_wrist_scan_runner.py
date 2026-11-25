"""Adapter test: sim captures -> live wrist-scan runner -> 64 predictions.

The fixture is a real product scene rendered at two planned scan poses with one
fixed piece position, so this exercises the same registration, self-occlusion
and classification code the console would call.  It skips (never fakes) when
the product scene assets or the wrist model are absent.

Two fixture details are deliberate:

* the wrist calibration is derived from the scene the frames come from, exactly
  as ``tools/export_wrist_calibration.py`` exports it, because a rig
  calibration's intrinsics and mount do not describe a simulator camera;
* the committed plan predates the tightened ``wrist_roll`` range, so planned
  roll is clamped into the calibrated limits instead of raising.  Pose names,
  target heights and view geometry are unchanged.
"""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from chess_sim.charuco_face import CharucoLayout
from chess_sim.config import Camera
from chess_sim.perception import CLASSES, PerceptionModel
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.wrist_scan import WRIST_MODEL_PATH, ScanPose, load_plan

POSITION = "4k3/8/8/3p4/8/2N5/8/4K3 w - - 0 1"


def _scene():
    """The product scene: printed pieces with the parallel-jaw tool.

    Both are part of the configuration the wrist model is trained for; a
    different tool moves the wrist camera and a different piece pack changes
    what a piece looks like, so the fixture refuses to pretend.
    """
    from chess_sim import assets, gripper
    from chess_sim.env import ChessSimEnv
    from chess_sim.product import layout_env_config

    if not gripper.CUSTOM:
        pytest.skip("product tool unavailable: set ROBOT_GRIPPER=cad_parallel "
                    "with ROBOT_GRIPPER_ASSETS")
    try:
        return ChessSimEnv(layout_env_config(assets.PRINTED_SET,
                                             cameras=(Camera.WRIST,)))
    except (ValueError, FileNotFoundError) as exc:
        pytest.skip(f"product piece pack unavailable: {exc}")


def _sim_calibration(env) -> WristCalibration:
    """The current scene's own wrist calibration (same maths as the exporter)."""
    from tools.export_wrist_calibration import build_chain

    model = env.model
    camera_id = model.camera(str(Camera.WRIST)).id
    chain = build_chain(model, camera_id)
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, model.cam_quat[camera_id])
    camera_in_tool = np.eye(4)
    camera_in_tool[:3, :3] = rotation.reshape(3, 3)
    camera_in_tool[:3, 3] = model.cam_pos[camera_id]
    fy = 480.0 / (2.0 * math.tan(math.radians(model.cam_fovy[camera_id]) / 2.0))
    return WristCalibration(640, 480, (fy, fy, 320.0, 240.0), (0.0,) * 5,
                            tuple(chain), camera_in_tool.tolist(), "test-sim")


def _limited_poses(poses: list[ScanPose], calibration: WristCalibration) -> list[ScanPose]:
    """Planned poses with every joint clamped into the installed hard limits."""
    from chess_sim import gripper

    limits = [link["limits"] for link in calibration.chain if "joint_index" in link]
    adjusted = []
    for pose in poses:
        joints = list(pose.joints)
        for index, (low, high) in enumerate(limits[:5]):
            joints[index] = min(max(joints[index], float(low)), float(high))
        joints[5] = min(max(joints[5], float(gripper.JOINT_MIN)),
                        float(gripper.JOINT_MAX))
        adjusted.append(ScanPose(pose.name, tuple(joints)))
    return adjusted


def _capture(env, pose: ScanPose) -> dict:
    env.data.qpos[env._joint_qpos] = np.asarray(pose.joints, dtype=float)
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)
    return {"pose": pose.name,
            "frame": env.render(Camera.WRIST).copy(),
            "joints": tuple(env.data.qpos[env._joint_qpos])}


def test_runner_registers_and_labels_a_sim_board():
    from chess_sim.perception_data import truth_labels
    from live.wrist_scan_runner import ScanInputs, run_scan, summarize

    if not Path(WRIST_MODEL_PATH).exists():
        pytest.skip("wrist perception model unavailable")
    plan = load_plan()
    env = _scene()
    try:
        calibration = _sim_calibration(env)
        env.reset("8/8/8/8/8/8/8/8")
        env.config = env.config.model_copy(update={"board": env.board})
        poses = _limited_poses(plan, calibration)[:2]
        env.reset(POSITION)
        captures = [_capture(env, pose) for pose in poses]
        truth = truth_labels(env)
        origin = tuple(env.board.origin)
        yaw = float(env.board.yaw)

        inputs = ScanInputs(
            calibration=calibration, poses=tuple(poses),
            model=PerceptionModel.load(WRIST_MODEL_PATH),
            square=env.board.square, board_z=env.board.top,
            marker_z=env.board.top,
            layout=CharucoLayout(square_mm=env.board.square * 1000.0),
            config_path=Path(__file__).resolve().parent)
        result = run_scan(captures, inputs=inputs)
        payload = summarize(result)
        threshold = inputs.model.confidence_threshold
    finally:
        env.close()

    assert result.registration is not None, "marker registration must succeed"
    assert len(result.predictions) == 64
    assert sorted(p.square for p in result.predictions) == list(range(64))
    assert payload["registered"] is True
    assert len(payload["predictions"]) == 64
    assert payload["frames_used"] == 2

    # Scored against the scene here in the test only: the scanner never sees it.
    assert math.hypot(result.registration.origin[0] - origin[0],
                      result.registration.origin[1] - origin[1]) < 0.002
    yaw_error = abs((result.registration.yaw - yaw + math.pi)
                    % (2 * math.pi) - math.pi)
    assert yaw_error < math.radians(1.0)

    confident_wrong = [
        prediction.square for prediction in result.predictions
        if truth[prediction.square] != CLASSES.index("empty")
        and prediction.label == "empty" and prediction.confidence >= threshold]
    assert confident_wrong == [], (
        f"occupied squares confidently labelled empty: {confident_wrong}")


def test_plan_pose_degrees_maps_the_sim_roll_to_the_bridge():
    """The plan is sim joint space; the bridge wrist_roll is offset by -87.21 deg."""
    from live.wrist_scan_runner import plan_pose_degrees

    plan = load_plan()
    rolls = []
    for pose in plan:
        values = plan_pose_degrees(pose)
        assert len(values) == 6 and all(math.isfinite(v) for v in values)
        rolls.append(values[4])
        assert 0.0 <= values[5] <= 100.0, values[5]
    assert min(rolls) >= -90.5 and max(rolls) <= 90.5, rolls
    # The vertical plan poses sit at the live zero, not at sim -87.2.
    assert any(abs(roll) < 0.5 for roll in rolls), rolls
    assert max(rolls) > 60.0 and min(rolls) < -60.0, rolls


def test_frames_from_captured_converts_degrees_to_radians():
    """The live calibration's camera() takes radians; captures are in degrees."""
    from dataclasses import dataclass

    from live.wrist_scan_runner import frames_from_captured

    @dataclass
    class Frame:
        image: np.ndarray
        joints_deg: tuple[float, ...]
        timestamp: float = 0.0

    frame = Frame(np.zeros((4, 4, 3), dtype=np.uint8),
                  (0.0, 0.0, 0.0, 0.0, -87.21, 0.0), 1.5)
    frames = frames_from_captured([("v1", frame)])
    assert len(frames) == 1
    assert frames[0].pose == "v1"
    assert frames[0].timestamp == 1.5
    assert abs(frames[0].joints[4] - math.radians(-87.21)) < 1e-12
