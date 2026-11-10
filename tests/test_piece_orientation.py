import math
from types import SimpleNamespace

import pytest

from chess_sim.piece_orientation import AxisEstimate, axis_distance_degrees, fit_mesh_axis


@pytest.mark.parametrize(("a", "b", "expected"), [
    (0.0, 180.0, 0.0),
    (5.0, 175.0, 10.0),
    (45.0, 135.0, 90.0),
    (179.0, 1.0, 2.0),
])
def test_axis_distance_respects_180_degree_jaw_symmetry(a, b, expected):
    assert axis_distance_degrees(a, b) == pytest.approx(expected)


def test_axis_estimate_acceptance_requires_uncertainty_confidence_and_three_views():
    estimate = AxisEstimate(math.radians(35), math.radians(10), .8, 3, .1, {})
    assert estimate.acceptable()
    assert not AxisEstimate(math.radians(35), math.radians(16), .8, 3, .1, {}).acceptable()
    assert not AxisEstimate(math.radians(35), math.radians(10), .69, 3, .1, {}).acceptable()
    assert not AxisEstimate(math.radians(35), math.radians(10), .8, 2, .1, {}).acceptable()
    assert not AxisEstimate(float("nan"), math.radians(10), .8, 3, .1, {}).acceptable()


def test_mesh_axis_abstains_before_loading_mesh_without_three_distinct_views():
    views = [SimpleNamespace(name="v1"), SimpleNamespace(name="v2"),
             SimpleNamespace(name="v2")]
    assert fit_mesh_axis("does-not-exist.obj", views, (0.0, 0.0), 0.0) is None
