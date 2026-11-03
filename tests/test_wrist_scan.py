import math

import numpy as np
import pytest

from chess_sim import fiducials
from chess_sim.board_registration import BoardRegistration
from chess_sim.perception import CLASSES, SquarePrediction
from chess_sim.wrist_calibration import ReferenceEpoch, WristCalibration
from chess_sim.wrist_scan import ScanPose, fuse_predictions, load_plan


def _chain():
    """One hinge about +z at the origin, then the camera 0.2 m along +x."""
    return [{
        "name": "base", "position": [0.0, 0.0, 0.0],
        "rotation": np.eye(3).ravel().tolist(),
        "joint_index": 0, "anchor": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0],
        "reference": 0.0, "limits": [-math.pi, math.pi],
    }]


def _camera_in_tool():
    tool = np.eye(4)
    tool[:3, 3] = [0.2, 0.0, 0.0]
    return tool.tolist()


def _calibration(distortion=(0.0,) * 5):
    return WristCalibration(
        width=64, height=48, intrinsics=(50.0, 50.0, 32.0, 24.0),
        distortion=distortion, chain=tuple(_chain()),
        camera_in_tool=_camera_in_tool(), calibration_id="unit")


def test_forward_kinematics_places_the_lens_from_measured_joints():
    calibration = _calibration()
    camera = calibration.camera([math.pi / 2] + [0.0] * 5)
    assert np.allclose(camera.camera_position, (0.0, 0.2, 0.0), atol=1e-9)
    expected = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert np.allclose(np.asarray(camera.camera_rotation).reshape(3, 3), expected, atol=1e-9)


def test_forward_kinematics_rejects_bad_measurements():
    calibration = _calibration()
    with pytest.raises(ValueError):
        calibration.camera([0.0] * 5)
    with pytest.raises(ValueError):
        calibration.camera([math.nan] + [0.0] * 5)
    with pytest.raises(ValueError):
        calibration.camera([math.pi + 0.2] + [0.0] * 5)


def test_rectify_is_a_copy_without_distortion_and_preserves_shape_with_it():
    calibration = _calibration()
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    out = calibration.rectify(frame)
    assert out.shape == frame.shape and out.dtype == np.uint8
    warped = _calibration(distortion=(-0.1, 0.02, 0.0, 0.0, 0.0)).rectify(frame)
    assert warped.shape == frame.shape


def test_reference_epoch_clears_on_new_registration_and_clock_reversal():
    epoch = ReferenceEpoch()
    pose = BoardRegistration((0.0, 0.0), 0.0, 0.028, 0.4337, 1.0, 1.0)
    assert epoch.update(pose, "cal", 10.0) is True
    epoch.references = {"v1": 1}
    assert epoch.update(pose, "cal", 11.0) is False
    assert epoch.references == {"v1": 1}
    moved = BoardRegistration((0.001, 0.0), 0.0, 0.028, 0.4337, 1.0, 1.0)
    assert epoch.update(moved, "cal", 12.0) is True
    assert epoch.references == {}
    assert epoch.update(moved, "cal", 5.0) is True


def test_fused_median_rejects_one_confidently_wrong_view():
    def view(label, square=27, confidence=0.95, valid=1.0):
        row = [0.02, 0.02, 0.02, 0.02]
        row[CLASSES.index(label)] = confidence
        return [SquarePrediction(square, label, confidence, tuple(row), valid)]

    views = [view("white"), view("white"), view("empty")]
    fused = fuse_predictions(views, threshold=0.5)
    assert fused[27].label == "white"
    unknown = fuse_predictions([view("white", valid=0.1)], threshold=0.5)
    assert unknown[27].label == "unknown"


def test_marker_geometry_is_complete_and_ordered():
    ids, points = fiducials.object_points()
    assert len(points) == 4 * len(fiducials.MARKERS)
    assert set(ids.tolist()) == set(fiducials.MARKER_IDS)
    assert fiducials.marker_bits(0).shape == (6, 6)
    geoms = fiducials.mat_geoms(0.0032, 0.0005)
    assert all(part["size"][0] > 0 for part in geoms)
    assert fiducials.marker_height_above_table(0.0005) == pytest.approx(0.00056)


def test_scan_plan_loads_poses_with_six_finite_joints():
    poses = load_plan()
    assert len(poses) >= 2
    assert all(len(pose.joints) == 6 for pose in poses)
    with pytest.raises(ValueError):
        ScanPose("bad", (0.0,) * 5)

