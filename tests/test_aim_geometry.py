"""Aim-ray geometry: the invariants that must not drift.

These are synthetic-geometry tests (no calibration file, no arm, no MuJoCo), so
they pin the *algebra* of `live/aim.py`:

* the OpenGL -> OpenCV camera flip is a proper rotation and a point straight in
  front of the camera lands on the principal point;
* `TIP_PIXEL` cannot vary with the pose — asserted as a **regression only**,
  because the camera and the tip are rigid on one body and the constancy is an
  identity, not a result (tip-center.md §1);
* a vertical tool's aim is directly under it, and a tilted tool's aim is not;
* a ray that grazes the plane, a stale plane and a missing plane all come back
  as ``ok: False`` with a reason rather than a plausible number;
* the plane recovered from a marker solve is the same plane the arm-frame
  composition puts there (the one place the two camera conventions meet);
* `PlaneTracker` averages offsets, refuses a bad solve, and drops the window
  when the board jumps.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "live"))

import aim  # noqa: E402


def mount(position_mm=(60.0, 0.0, -23.0), tilt_deg=30.0) -> aim.CameraMount:
    """A camera offset along +x and tilted back toward the tool axis.

    GL camera axes are +x right, +y up, -z view, so the third column of the
    rotation is -view and the frame must stay right-handed (x cross y = z).
    """
    angle = math.radians(tilt_deg)
    view = np.array([-math.sin(angle), 0.0, -math.cos(angle)])
    z_axis = -view                                   # the GL "back" axis
    right = np.array([0.0, -1.0, 0.0])
    up = np.cross(z_axis, right)                     # = (cos, 0, -sin)
    rotation = np.column_stack([right, up, z_axis])
    assert np.isclose(np.linalg.det(rotation), 1.0)
    return aim.CameraMount(position=np.array(position_mm, float) / 1000.0,
                           rotation_gl=rotation,
                           fx=831.897, fy=833.014, cx=320.0, cy=240.0)


def _orthonormalise(matrix: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(matrix)
    out = u @ vt
    if np.linalg.det(out) < 0:
        u[:, -1] *= -1
        out = u @ vt
    return out


def pose(translation_mm=(0, 0, 200), rotation=None) -> np.ndarray:
    """A tool pose; the translation is in MILLIMETRES on purpose."""
    value = np.eye(4)
    value[:3, :3] = np.eye(3) if rotation is None else np.asarray(rotation, float)
    value[:3, 3] = np.array(translation_mm, float) / 1000.0
    return value


def test_gl_flip_is_a_proper_rotation():
    assert np.isclose(np.linalg.det(aim.GL_TO_CV), 1.0)
    assert np.allclose(aim.GL_TO_CV @ aim.GL_TO_CV, np.eye(3))


def test_point_in_front_lands_on_the_principal_point():
    camera = mount()
    camera_pose = pose((0, 0, 1.0))          # tool at z = 1 m, tool axis down
    forward = camera.rotation_gl @ aim.GL_TO_CV @ np.array([0.0, 0.0, 1.0])
    point = camera_pose[:3, 3] + camera.position + 0.2 * forward
    uv, depth = camera.project(camera_pose, point)
    assert depth > 0.0
    assert uv[0] == pytest.approx(320.0, abs=1e-6)
    assert uv[1] == pytest.approx(240.0, abs=1e-6)


def test_tip_pixel_is_a_mount_constant_regression_only():
    """Regression: a rigid body cannot project differently from another pose."""
    camera = mount()
    first = camera.tip_pixel()
    for joints in ([0, 0, 0, 0, 0, 0], [30, -40, 55, 70, 40, 20],
                   [-70, 20, -60, -30, -80, 60]):
        # tip_pixel takes no joints at all: that is the point.  Recompute it
        # anyway so a future signature change that starts moving it fails here.
        assert camera.tip_pixel() == pytest.approx(first, abs=0.0)
        assert isinstance(joints, list)
    assert all(math.isfinite(v) for v in first)


def test_vertical_tool_aims_directly_below():
    camera = mount()
    horizontal = aim.Plane(normal=np.array([0.0, 0.0, 1.0]), offset=-0.2)
    world, reason = aim.aim_from_pose(pose((30, -40, 200)), horizontal)
    assert reason == ""
    assert world[0] == pytest.approx(0.030, abs=1e-9)
    assert world[1] == pytest.approx(-0.040, abs=1e-9)
    assert world[2] == pytest.approx(-0.200, abs=1e-9)


def test_tilted_tool_aims_off_axis():
    camera = mount()
    horizontal = aim.Plane(normal=np.array([0.0, 0.0, 1.0]), offset=-0.2)
    tilt = np.array([[math.cos(math.radians(10)), 0, math.sin(math.radians(10))],
                     [0, 1, 0],
                     [-math.sin(math.radians(10)), 0, math.cos(math.radians(10))]])
    world, _ = aim.aim_from_pose(pose((0, 0, 200), tilt), horizontal)
    # (200 - (-200)) mm of height at 10 deg of tilt is 70.5 mm of offset.
    assert world[0] == pytest.approx(-0.0705, abs=2e-4)
    assert world[2] == pytest.approx(-0.200, abs=1e-9)


def test_grazing_ray_is_refused():
    plane = aim.Plane(normal=np.array([0.0, 0.0, 1.0]), offset=-0.2)
    world, reason = plane.intersect((0.0, 0.0, 0.2), (1.0, 0.0, 0.0))
    assert world is None
    assert "grazes" in reason


def test_ray_pointing_away_is_refused():
    plane = aim.Plane(normal=np.array([0.0, 0.0, 1.0]), offset=0.3)
    world, reason = plane.intersect((0.0, 0.0, 0.2), (0.0, 0.0, -1.0))
    assert world is None
    assert "away" in reason


def test_stale_and_missing_plane_are_named():
    camera = mount()
    world = pose((0, 0, 0.2))
    assert aim.evaluate(world, camera, None)["reason"].startswith("no board plane")
    old = aim.Plane(normal=np.array([0.0, 0.0, 1.0]), offset=-0.2,
                    age_s=99.0, samples=3)
    out = aim.evaluate(world, camera, old, plane_max_age_s=5.0)
    assert out["ok"] is False
    assert "stale" in out["reason"]
    # the numbers are still reported, they are just flagged
    assert out["aim_pixel"] is not None


def test_plane_from_solution_round_trip():
    """The board plane recovered from a solve is the plane that was solved."""
    camera = mount()
    tool_pose = pose((10.0, -20.0, 0.35))
    board_frame = np.eye(4)
    # A board 0.16 m below the camera, tilted 3 deg about x, in the base frame.
    tilt = np.array([[1, 0, 0], [0, math.cos(0.05), -math.sin(0.05)],
                     [0, math.sin(0.05), math.cos(0.05)]])
    base_board = np.eye(4)
    base_board[:3, :3] = tilt
    base_board[:3, 3] = [0.02, -0.03, 0.19]
    # Project it into the camera, exactly as solvePnP would report it.
    base_camera = (board_frame @ tool_pose) @ np.block([
        [camera.rotation_gl @ aim.GL_TO_CV, camera.position.reshape(3, 1)],
        [np.zeros((1, 3)), 1.0]])
    cam_board = np.linalg.inv(base_camera) @ base_board
    from scipy.spatial.transform import Rotation
    rvec = Rotation.from_matrix(cam_board[:3, :3]).as_rotvec()
    solution = {"rvec": rvec.tolist(), "tvec": cam_board[:3, 3].tolist(),
                "rms_px": 1.2, "ids": [0, 1]}
    normal, point, info = aim.plane_from_solution(solution, tool_pose, camera,
                                                  board_frame)
    assert normal is not None
    assert abs(float(normal @ base_board[:3, 2])) == pytest.approx(1.0, abs=1e-9)
    assert abs(float(normal @ (point - base_board[:3, 3]))) < 1e-9
    assert info["ids"] == 2


def test_tracker_averages_and_rejects_and_resets():
    tracker = aim.PlaneTracker(window=5, jump_mm=25.0, max_rms_px=3.0)
    normal = np.array([0.0, 0.0, 1.0])
    assert tracker.add(normal, (0, 0, -0.200), key=1, rms_px=1.0)
    assert not tracker.add(normal, (0, 0, -0.200), key=1, rms_px=1.0)   # dedupe
    assert not tracker.add(normal, (0, 0, -0.200), key=2, rms_px=9.0)   # bad solve
    tracker.add(normal, (0, 0, -0.210), key=3, rms_px=1.0)
    tracker.add(normal, (0, 0, -0.190), key=4, rms_px=1.0)
    plane = tracker.plane()
    assert plane.samples == 3
    assert plane.offset == pytest.approx(-0.200, abs=1e-9)
    assert plane.spread_mm > 5.0
    assert tracker.rejected == 1
    # a jumped board drops the window instead of blending two boards
    tracker.add(normal, (0.05, 0, -0.400), key=5, rms_px=1.0)
    assert tracker.plane().samples == 1


def test_aim_translation_moves_the_aim_onto_the_target():
    camera = mount()
    plane = aim.Plane(normal=np.array([0.0, 0.0, 1.0]), offset=-0.2)
    world = pose((0, 0, 0.2))
    delta = aim.aim_translation(world, plane, (40.0, -25.0), np.eye(4))
    moved = world.copy()
    moved[:3, 3] = world[:3, 3] + delta
    world_aim, _ = aim.aim_from_pose(moved, plane)
    local = (np.linalg.inv(np.eye(4)) @ np.array([*world_aim, 1.0]))[:3] * 1000.0
    assert local[0] == pytest.approx(40.0, abs=1e-6)
    assert local[1] == pytest.approx(-25.0, abs=1e-6)


def test_square_mapping_is_the_board_target_mapping():
    assert aim.square_at(-98.0, -98.0) == "a1"
    assert aim.square_at(0.0, 0.0) == "e5"      # floor picks the upper square at a corner
    assert aim.square_at(14.0, -14.0) == "e4"
    assert aim.square_at(200.0, 0.0) is None
