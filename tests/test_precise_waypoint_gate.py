from types import SimpleNamespace

import numpy as np
import pytest

from chess_sim.controller import IK_FALLBACK_TRIALS, PickPlaceController


class _UnreachableLocalIK:
    lower = np.full(5, -1.0)
    upper = np.full(5, 1.0)

    def __init__(self, residual):
        self.residual = residual
        self.solve_calls = 0

    def solve(self, data, target, *, offset, span, seed_q=None):
        self.solve_calls += 1
        return SimpleNamespace(q=np.zeros(5), position_error=self.residual,
                               tilt=0.0)

    def tool_pose(self, data, offset):
        return self.target + np.array([self.residual, 0.0, 0.0]), np.eye(3)


class _Env:
    def __init__(self, target, residual, phase):
        self.ik = _UnreachableLocalIK(residual)
        self.ik.target = np.asarray(target, dtype=float)
        self.data = object()
        self.diagnostic_phase = phase
        self.diagnostic_stall = None
        self.diagnostic_ik = None
        self.diagnostic_offset = np.zeros(3)
        self.noise_suppressed = False

    def arm_joint_positions(self):
        return np.zeros(6)

    def apply_action(self, action, on_step):
        pass


@pytest.mark.parametrize("phase,expected", [
    ("approach", False),
    ("close", False),
    ("place_descend", False),
    ("lift", True),
])
def test_precise_grasp_and_place_do_not_accept_8mm_residual(phase, expected):
    target = np.array([0.1, 0.2, 0.3])
    env = _Env(target, residual=0.0079, phase=phase)
    controller = PickPlaceController(env)

    result = controller._go(target, gap=0.01, offset=np.zeros(3),
                            steps=2, on_step=None, precise=True)

    assert result is expected
    if phase in ("approach", "close", "place_descend"):
        # The first local solve plus every deterministic fallback seed was
        # attempted; none reached the grasp/place precision gate.
        assert env.ik.solve_calls == 1 + IK_FALLBACK_TRIALS


@pytest.mark.parametrize("phase,expected_reason", [
    ("lift", "lift_unstable"),
    ("carry", "carry_unstable"),
])
@pytest.mark.parametrize("drift_m,aborted", [(0.0049, False), (0.0051, True)])
def test_held_piece_slip_aborts_in_lift_and_carry(phase, expected_reason, drift_m, aborted):
    env = _Env([0.0, 0.0, 0.0], residual=0.0, phase=phase)
    controller = PickPlaceController(env)
    controller._moving_slot = object()
    controller._lift_reference_local = np.zeros(3)
    controller._abort_reason = None
    controller._failure_reason = None
    controller._piece_gripper_local = lambda slot: np.array([drift_m, 0.0, 0.0])

    assert controller._guard_held_piece_slip() is aborted
    assert env.diagnostic_stall == (expected_reason if aborted else None)
    assert controller._abort_reason == (expected_reason if aborted else None)


@pytest.mark.parametrize("phase", ["transit", "vision_reobserve_park", "close"])
def test_observation_return_and_closure_stop_at_first_neighbor_contact(phase):
    env = _Env([0.0, 0.0, 0.0], residual=0.0, phase=phase)
    controller = PickPlaceController(env)
    controller._moving_slot = object()
    controller._failure_reason = None
    calls = []
    env.apply_action = lambda action, on_step: calls.append(action.copy())
    controller._gripper_has_neighbor_contact = lambda slot: True

    if phase == "vision_reobserve_park":
        controller._move_joints(np.ones(5), 0.0165, None)
    else:
        assert not controller._go(np.zeros(3), 0.0165, np.zeros(3), 8, None)

    assert len(calls) == 1
    assert env.diagnostic_stall == "gripper_neighbor_contact"
    assert controller._abort_reason == "gripper_neighbor_contact"


def test_close_promotes_only_an_over_tolerance_local_solve_to_precise():
    target = np.array([0.1, 0.2, 0.3])
    env = _Env(target, residual=0.0079, phase="close")
    controller = PickPlaceController(env)

    result = controller._go(target, gap=0.01, offset=np.zeros(3),
                            steps=2, on_step=None, precise=False)

    assert result is False
    assert env.noise_suppressed is False  # restored after the failed attempt
    assert env.ik.solve_calls == 1 + IK_FALLBACK_TRIALS
