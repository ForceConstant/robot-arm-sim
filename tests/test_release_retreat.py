"""A settled release must not jump back to a stale descent endpoint."""
from types import SimpleNamespace

import numpy as np

from chess_sim.controller import PickPlaceController


def _retreat_fixture():
    endpoint = np.array([0.1, -0.2, 0.3, -0.4, 0.5])
    release = endpoint + np.array([0.02, 0.03, -0.04, 0.01, 0.0])
    actions = []
    env = SimpleNamespace(arm_joint_positions=lambda: np.append(release, 0.0),
                          apply_action=lambda action, callback: actions.append(action.copy()))
    controller = PickPlaceController(env)
    controller._placement_retreat_path = [endpoint - 0.01, endpoint]
    return controller, release, actions


def test_release_retreat_starts_at_measured_release_not_stale_descent():
    controller, release, actions = _retreat_fixture()
    controller._guard_gripper_neighbor_contact = lambda: False
    assert controller._reverse_placement_retreat(0.022, None)
    np.testing.assert_allclose(actions[0][:5], release)
    np.testing.assert_allclose(actions[-1][:5], release - 0.01)


def test_release_retreat_still_stops_at_first_neighbor_contact():
    controller, _, actions = _retreat_fixture()
    controller._guard_gripper_neighbor_contact = lambda: True
    assert not controller._reverse_placement_retreat(0.022, None)
    assert len(actions) == 1
