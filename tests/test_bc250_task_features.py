"""Observation boundary: truth and supervision cannot enter pilot features."""
import numpy as np
import pytest

from tools.export_bc250_task_pilot import features


def test_wrist_pixels_joints_and_instruction_are_the_only_inputs():
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    values = features(frame, np.zeros(6), "e2e4")
    assert len(values) == 202
    assert values[:192] == [-1.0]*192
    assert values[192:198] == [0.0]*6
    assert values[-4:] == [4/7, 1/7, 4/7, 3/7]
    assert features(np.full_like(frame, 255), np.zeros(6), "e2e4")[:192] == [1.0]*192


@pytest.mark.parametrize("move", ["e9e4", "e2z4", "e2e4q", "", "FEN"])
def test_malformed_task_is_rejected(move):
    with pytest.raises(ValueError):
        features(np.zeros((8, 8, 3), dtype=np.uint8), np.zeros(6), move)


def test_nonfinite_state_is_rejected():
    with pytest.raises(ValueError):
        features(np.zeros((8, 8, 3), dtype=np.uint8), [float("nan")]*6, "e2e4")
