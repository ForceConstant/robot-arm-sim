"""Camera yaw offset: file round-trip and the optical-axis rotation convention.

The offset is layered on top of the pinned hand-eye calibration, so these tests
pin the two things that would silently corrupt every wrist view if they drifted:
where the number is stored, and which way a positive offset turns the image.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import camera_yaw  # noqa: E402
from chess_sim.vision_geometry import CameraCalibration  # noqa: E402


def _camera() -> CameraCalibration:
    """Pinhole at the origin looking down -Z, axes +X right / +Y up."""
    return CameraCalibration(640, 480, 300.0, 300.0, 320.0, 240.0,
                             (0.0, 0.0, 0.0),
                             (1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0))


def _board_project(camera: CameraCalibration, depth_m: float = 0.3):
    """Board-mm (u right, v down) projection for a board parallel to the image."""
    def project(xy_mm, z_mm=0.0):
        return camera.project((xy_mm[0] / 1000.0,
                               xy_mm[1] / 1000.0,
                               -depth_m))

    return project


def test_rotation_matrix_is_rz_in_camera_axes():
    np.testing.assert_allclose(camera_yaw.rotation_matrix(0.0), np.eye(3))
    quarter = camera_yaw.rotation_matrix(90.0)
    np.testing.assert_allclose(quarter @ np.array([1.0, 0.0, 0.0]),
                               np.array([0.0, 1.0, 0.0]), atol=1e-12)
    np.testing.assert_allclose(quarter @ np.array([0.0, 0.0, 1.0]),
                               np.array([0.0, 0.0, 1.0]), atol=1e-12)


def test_apply_post_multiplies_and_leaves_position_and_intrinsics():
    camera = _camera()
    rotated = camera_yaw.apply(camera, 7.5)
    np.testing.assert_allclose(
        np.asarray(rotated.camera_rotation).reshape(3, 3),
        camera_yaw.rotation_matrix(7.5), atol=1e-12)
    assert rotated.camera_position == camera.camera_position
    assert (rotated.fx, rotated.fy, rotated.cx, rotated.cy) == (
        camera.fx, camera.fy, camera.cx, camera.cy)
    assert camera_yaw.apply(camera, 0.0) is camera


def test_positive_offset_turns_image_content_clockwise():
    """A positive offset adds to the apparent yaw measured with +v down."""
    base = _camera()
    assert camera_yaw.image_yaw_deg(_board_project(base), 56.0) == pytest.approx(
        0.0, abs=1e-9)
    tilted = camera_yaw.apply(base, 5.0)
    assert camera_yaw.image_yaw_deg(_board_project(tilted), 56.0) == pytest.approx(
        5.0, abs=1e-9)
    assert camera_yaw.image_yaw_deg(
        _board_project(camera_yaw.apply(base, -2.5)), 56.0
    ) == pytest.approx(-2.5, abs=1e-9)


def test_offset_new_equals_offset_minus_measured_nulls_the_yaw():
    base = _camera()
    offset = -3.0
    measured = camera_yaw.image_yaw_deg(
        _board_project(camera_yaw.apply(base, offset)), 56.0)
    assert measured == pytest.approx(offset, abs=1e-9)
    corrected = camera_yaw.apply(base, offset - measured)
    assert camera_yaw.image_yaw_deg(
        _board_project(corrected), 56.0) == pytest.approx(0.0, abs=1e-9)


def test_image_yaw_rejects_degenerate_inputs():
    camera = _camera()
    with pytest.raises(ValueError):
        camera_yaw.image_yaw_deg(_board_project(camera), 0.0)
    assert camera_yaw.image_yaw_deg(lambda xy, z: None, 56.0) is None
    assert camera_yaw.image_yaw_deg(lambda xy, z: (320.0, 240.0), 56.0) is None
    assert camera_yaw.image_yaw_deg(lambda xy, z: (320.0, 240.0 + xy[0]),
                                    56.0) == pytest.approx(90.0)


def test_offset_file_round_trip(tmp_path):
    path = tmp_path / "nested" / "camera_yaw.json"
    saved = camera_yaw.save(path, 2.5, source="button")
    assert saved["schema"] == camera_yaw.SCHEMA
    assert saved["offset_deg"] == pytest.approx(2.5)
    assert saved["source"] == "button"
    assert json.loads(path.read_text()) == saved
    loaded = camera_yaw.load(path)
    assert loaded["offset_deg"] == pytest.approx(2.5)
    assert loaded["source"] == "button"
    assert loaded["updated"] == saved["updated"]


def test_offset_file_missing_corrupt_and_foreign_schema_read_as_zero(tmp_path):
    assert camera_yaw.load(tmp_path / "absent.json")["offset_deg"] == 0.0
    corrupt = tmp_path / "corrupt.json"
    corrupt.write_text("{not json")
    assert camera_yaw.load(corrupt)["offset_deg"] == 0.0
    foreign = tmp_path / "foreign.json"
    foreign.write_text(json.dumps({"schema": "other/9", "offset_deg": 12.0}))
    assert camera_yaw.load(foreign)["offset_deg"] == 0.0
    huge = tmp_path / "huge.json"
    huge.write_text(json.dumps({"schema": camera_yaw.SCHEMA,
                                "offset_deg": 500.0}))
    assert camera_yaw.load(huge)["offset_deg"] == 0.0


@pytest.mark.parametrize("bad", [math.nan, math.inf, 90.0, -90.0])
def test_save_refuses_implausible_offsets(tmp_path, bad):
    with pytest.raises(ValueError):
        camera_yaw.save(tmp_path / "x.json", bad)


def test_yaw_error_is_the_residual_roll():
    """R_meas = R_pred @ Rz(theta) must report theta, whatever R_pred is."""
    from scipy.spatial.transform import Rotation

    import camera_yaw

    predicted = Rotation.from_euler("xyz", [12.0, -5.0, 30.0], degrees=True).as_matrix()
    for theta in (0.0, 1.7, -3.2, 11.0):
        measured = predicted @ Rotation.from_euler("z", theta, degrees=True).as_matrix()
        assert camera_yaw.yaw_error_deg(measured, predicted) == pytest.approx(theta, abs=1e-9)
