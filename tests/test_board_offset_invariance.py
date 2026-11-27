"""tests/test_board_offset_invariance.py — the invariant behind the whole exercise.

The mount is a hardware constant; the board pose is a per-session measurement.
This test proves the algebra the investigation kept tripping over:

* with the *true* board pose, the mount recovered from every view is identical;
* with a *stale* board pose, the recovered mount is pose-dependent even though
  nothing about the arm or camera changed.

That second statement is why a per-view mount scatter can never be read as
"the arm is wrong".  It is exact — no noise, no fitting, no hardware.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))


def _flange_poses(n: int = 6) -> list[np.ndarray]:
    """A spread of base-from-flange poses, one per view."""
    poses = []
    for pan, lift, elbow in [(-35, -70, 40), (-10, -55, 60), (15, -80, 35),
                             (30, -45, 75), (0, -20, 20), (-25, -85, 55)]:
        pose = np.eye(4)
        pose[:3, :3] = (Rotation.from_euler("z", pan, degrees=True)
                        * Rotation.from_euler("y", lift, degrees=True)
                        * Rotation.from_euler("y", elbow, degrees=True)).as_matrix()
        # a plausible flange position ~150-250 mm above the board datum
        pose[:3, 3] = [0.0006 * pan, -0.0005 * lift, 0.0021 * abs(lift) + 0.06]
        poses.append(pose)
    return poses[:n]


def _mount() -> np.ndarray:
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler("z", 11.8, degrees=True).as_matrix()
    mount[:3, 3] = [0.052, 0.009, -0.002]
    return mount


def _board(yaw_deg: float) -> np.ndarray:
    board = np.eye(4)
    board[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    board[:3, 3] = [-0.020, -0.025, 0.431]
    return board


def _recover_mount(flange_poses, board_stored, mount, board_true):
    """The mount the solver infers, per view.

    The camera really observes the board at ``board_true``; the solver only has
    ``board_stored``.  Inferred base_from_board = board_stored @ inv(C_true), and
    the mount is that undone by the flange pose.  With board_stored == board_true
    every view returns the same mount; with a stale board it does not.
    """
    mounts = []
    for base_from_flange in flange_poses:
        cam_from_board_true = np.linalg.inv(base_from_flange @ mount) @ board_true
        board_from_base_inferred = board_stored @ np.linalg.inv(cam_from_board_true)
        mounts.append(np.linalg.inv(base_from_flange) @ board_from_base_inferred)
    return mounts


def _spread(mounts):
    translations = np.array([m[:3, 3] for m in mounts])
    reference = translations.mean(axis=0)
    worst_mm = max(np.linalg.norm(t - reference) * 1000.0 for t in translations)
    rotations = [m[:3, :3] for m in mounts]
    mean = Rotation.from_matrix(rotations).mean()
    worst_deg = max(math.degrees(Rotation.from_matrix(mean.as_matrix().T @ r).magnitude())
                    for r in rotations)
    return worst_mm, worst_deg


def test_true_board_recovers_one_mount():
    board = _board(184.32)
    mounts = _recover_mount(_flange_poses(), board, _mount(), board)
    worst_mm, worst_deg = _spread(mounts)
    assert worst_mm < 1e-9, f"{worst_mm:.3e} mm spread with the true board"
    assert worst_deg < 1e-9, f"{worst_deg:.3e} deg spread with the true board"


@pytest.mark.parametrize("stale_yaw_deg", [174.32, 194.32, 8.0])
def test_stale_board_makes_the_mount_pose_dependent(stale_yaw_deg):
    """Rotating only the stored board pose must scatter the recovered mount."""
    mounts = _recover_mount(_flange_poses(), _board(stale_yaw_deg), _mount(), _board(184.32))
    worst_mm, worst_deg = _spread(mounts)
    # an 8-10 deg stale yaw at ~100 mm of view-dependent lever arm is centimetres
    assert worst_mm > 5.0, f"only {worst_mm:.2f} mm spread from a {abs(stale_yaw_deg - 184.32):.1f} deg stale board"
    assert worst_deg > 0.5, f"only {worst_deg:.2f} deg spread"


def test_a_stale_board_both_shifts_the_mean_and_scatters_the_views():
    """Why averaging a scattered set still produces a plausible-looking mount.

    A 10 deg stale board moves the recovered mount by a similar order to the
    scatter it creates, so the averaged candidate is neither obviously wrong nor
    right — which is precisely why it must not be adopted without a board
    reference.  The exact magnitudes are geometry, not a tuned threshold.
    """
    board = _board(184.32)
    truth = _recover_mount(_flange_poses(), board, _mount(), board)
    stale = _recover_mount(_flange_poses(), _board(194.32), _mount(), board)
    mean_shift = np.linalg.norm(np.array([m[:3, 3] for m in truth]).mean(axis=0)
                                - np.array([m[:3, 3] for m in stale]).mean(axis=0)) * 1000.0
    worst_true, _ = _spread(truth)
    worst_stale, _ = _spread(stale)
    assert worst_true < 1e-9
    assert worst_stale > 5.0, f"{worst_stale:.2f} mm"
    assert mean_shift > 5.0, f"{mean_shift:.2f} mm"
    assert 0.3 < worst_stale / mean_shift < 3.0
