"""A failed placement descent must put the piece back, never release mid-air."""
import chess
import numpy as np
import mujoco
import pytest

from chess_sim import AppearanceConfig, ControlConfig, EnvConfig, MoveTask
from chess_sim import controller as C
from chess_sim.dual import DualArmEnv


@pytest.fixture(scope="module")
def env():
    e = DualArmEnv(EnvConfig(control=ControlConfig(cameras=()),
                            appearance=AppearanceConfig(camera_mast=False)))
    e.reset(chess.STARTING_FEN)
    yield e
    e.close()


def test_failed_place_descent_returns_piece_to_source(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    original = C.PickPlaceController._descend

    def failing(self, target, gap, offset, approach, on_step):
        if self.env.diagnostic_phase == "place_descend":
            return False                     # arrive nowhere near the square
        return original(self, target, gap, offset, approach, on_step)

    monkeypatch.setattr(C.PickPlaceController, "_descend", failing)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"))

    assert result.success is False
    assert result.waypoints_precise is False
    assert env.diagnostic_stall == "place_descend"
    # the piece is back on its own square, standing, and e4 is still empty
    slot = env._square_slot.get(chess.E2)
    assert slot is not None
    centre = np.array(env.board.square_center(chess.E2))
    piece = env.piece_position(slot)
    assert np.linalg.norm(piece[:2] - centre) < 0.03, (piece, centre)
    assert env.piece_upright(slot) > env.config.tolerances.upright_min
    assert chess.E4 not in env._square_slot


def test_failed_source_recovery_descent_never_opens_gripper(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    original = C.PickPlaceController._descend
    phases = []
    place_attempts = 0

    def fail_place_and_recovery(self, target, gap, offset, approach, on_step):
        nonlocal place_attempts
        if self.env.diagnostic_phase == "place_descend":
            place_attempts += 1
            return False
        if self.env.diagnostic_phase == "abort_return":
            return False
        return original(self, target, gap, offset, approach, on_step)

    monkeypatch.setattr(C.PickPlaceController, "_descend", fail_place_and_recovery)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"),
                         on_step=lambda action: phases.append(env.diagnostic_phase))

    assert not result.success
    assert place_attempts == 2  # initial placement plus bounded alternate-span retry
    assert "abort_release" not in phases
    assert "release" not in phases


def test_clean_move_still_releases_normally(env):
    # Deliberately no manual reset: a stall from the previous test must not leak.
    env.reset(chess.STARTING_FEN)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"))
    assert result.success is True
    assert result.waypoints_precise is True
    assert result.phase10_complete is True
    assert result.released is True
    assert result.supported is True
    assert result.settled is True
    assert env.diagnostic_stall is None
    assert chess.E4 in env._square_slot


def test_failed_pickup_descent_never_closes_on_the_wrong_square(env,monkeypatch):
    env.reset(chess.STARTING_FEN)
    monkeypatch.setattr(C.PickPlaceController,'_descend',lambda *args,**kwargs:False)
    phases=[]
    result=env.execute(MoveTask(from_square='e2',to_square='e4'),
                       on_step=lambda action:phases.append(env.diagnostic_phase))
    assert not result.success
    assert env.diagnostic_stall=='grasp_descend'
    assert 'close' not in phases
    assert 'release' not in phases
    assert chess.E2 in env._square_slot and chess.E4 not in env._square_slot


def test_missed_grasp_stops_before_carrying(env, monkeypatch):
    """If nothing rises with the jaws, do not carry air to the target square."""
    import dataclasses
    env.reset(chess.STARTING_FEN)
    original = C.grasp_plan

    def closes_too_high(geometry, board):
        # A 60 mm error in grasp height: the jaws close on air above the piece.
        return dataclasses.replace(original(geometry, board),
                                   z_grasp=original(geometry, board).z_grasp + 0.06)

    monkeypatch.setattr(C, "grasp_plan", closes_too_high)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"))

    assert result.success is False
    assert result.picked is None
    # The 60 mm offset is rejected during the guarded grasp descent, before
    # the controller can close or attempt a lift.
    assert env.diagnostic_stall == "grasp_descend"
    slot = env._square_slot[chess.E2]
    centre = np.array(env.board.square_center(chess.E2))
    assert np.linalg.norm(env.piece_position(slot)[:2] - centre) < 0.01
    assert env.piece_upright(slot) > env.config.tolerances.upright_min
    assert chess.E4 not in env._square_slot


def test_failed_cross_board_carry_does_not_continue_to_placement(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    original = C.PickPlaceController._line
    carry_lines = 0
    phases = []

    def fail_board_transport(self, start, target, *args, **kwargs):
        nonlocal carry_lines
        if self.env.diagnostic_phase == "carry":
            carry_lines += 1
            if carry_lines == 2:  # first raises above source; second crosses board
                return False
        return original(self, start, target, *args, **kwargs)

    monkeypatch.setattr(C.PickPlaceController, "_line", fail_board_transport)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"),
                         on_step=lambda action: phases.append(env.diagnostic_phase))

    assert not result.success
    assert carry_lines == 2
    assert "place_approach" not in phases
    assert "release" not in phases


def test_first_lift_failure_is_not_overwritten_by_source_recovery(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    original = C.PickPlaceController._line
    phases = []

    def fail_lift(self, start, target, *args, **kwargs):
        if self.env.diagnostic_phase == "lift":
            return False
        return original(self, start, target, *args, **kwargs)

    monkeypatch.setattr(C.PickPlaceController, "_line", fail_lift)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"),
                         on_step=lambda action: phases.append(env.diagnostic_phase))

    assert not result.success
    assert env.diagnostic_stall == "lift_waypoint_miss"
    assert "abort_return" in phases
    assert "abort_release" in phases


def test_latched_place_contact_skips_retry_recovery_and_release(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    original = C.PickPlaceController._descend
    phases = []
    place_descents = 0

    def contact_on_place(self, target, gap, offset, approach, on_step):
        nonlocal place_descents
        if self.env.diagnostic_phase == "place_descend":
            place_descents += 1
            self._latch_abort("gripper_neighbor_contact")
            return False
        return original(self, target, gap, offset, approach, on_step)

    monkeypatch.setattr(C.PickPlaceController, "_descend", contact_on_place)
    result = env.execute(MoveTask(from_square="e2", to_square="e4"),
                         on_step=lambda action: phases.append(env.diagnostic_phase))

    assert not result.success
    assert env.diagnostic_stall == "gripper_neighbor_contact"
    assert place_descents == 1
    assert "abort_return" not in phases
    assert "release" not in phases


@pytest.mark.parametrize("phase", ["abort_return", "abort_release", "abort_grasp"])
def test_neighbor_contact_guard_remains_active_during_recovery(env, phase):
    env.reset(chess.STARTING_FEN)
    controller = C.PickPlaceController(env)
    controller._moving_slot = env.slot_at(chess.E2)
    # Direct guard invocation bypasses execute()'s per-attempt initialization.
    controller._failure_reason = None
    controller._gripper_has_neighbor_contact = lambda _slot: True
    env.diagnostic_phase = phase

    assert controller._guard_gripper_neighbor_contact()
    assert controller._abort_reason == "gripper_neighbor_contact"
    assert env.diagnostic_stall == "gripper_neighbor_contact"


def test_recovery_stops_when_held_piece_touches_neighbor(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    controller = C.PickPlaceController(env)
    controller._moving_slot = env.slot_at(chess.E2)
    controller._failure_reason = None
    controller._gripper_has_neighbor_contact = lambda _slot: False
    monkeypatch.setattr(controller, "_piece_has_neighbor_contact", lambda _slot: True)
    env.diagnostic_phase = "abort_return"

    assert controller._guard_gripper_neighbor_contact()
    assert controller._abort_reason == "recovery_piece_neighbor_contact"
    assert env.diagnostic_stall == "recovery_piece_neighbor_contact"


def test_oracle_geometry_cannot_override_controller_abort(env):
    from chess_sim import FailureReason

    env.reset(chess.STARTING_FEN)
    task = MoveTask(from_square="e2", to_square="e4")
    before = env.piece_snapshot()
    result = env.evaluate(task, before, waypoints_precise=False)

    assert not result.success
    assert result.reason == FailureReason.CONTROLLER_ABORT


def test_phase10_completion_rejects_target_xy_without_release_support_or_settling(env):
    env.reset(chess.STARTING_FEN)
    task = MoveTask(from_square="e2", to_square="e4")
    before = env.piece_snapshot()
    slot = env.slot_at(chess.E2)
    target = env.board.square_center(chess.E4)
    support_z = env.board.top - slot.geometry.bottom_offset + 0.0015
    env.set_piece_pose(slot,
        [target[0], target[1], support_z + 0.03],
        [1, 0, 0, 0], zero_velocity=True)
    mujoco.mj_forward(env.model, env.data)
    result = env.evaluate(task, before)

    # The historical XY/upright oracle still sees a placement, but Phase 10
    # completion requires the simulator physical state to show release and
    # settling on the correct support plane, not just target XY and uprightness.
    assert result.success
    assert not result.supported
    assert not result.phase10_complete
    assert not result.commit_eligible


def test_plan_override_shifts_the_grasp(env):
    """A correction is a parameter change, so it must reach the controller."""
    from chess_sim.controller import PlanOverride
    env.reset(chess.STARTING_FEN)
    seen = {}

    def watch(action=None):
        if env.diagnostic_phase == "close" and "base_z" not in seen and env.diagnostic_target:
            seen["base_z"] = env.diagnostic_target[2]

    env.execute(MoveTask(from_square="e2", to_square="e4"), on_step=watch)

    env.reset(chess.STARTING_FEN)
    def watch_override(action=None):
        if env.diagnostic_phase == "abort_invalid_grasp_height" and env.diagnostic_target:
            seen["override_z"] = env.diagnostic_target[2]

    result = env.execute(MoveTask(from_square="e2", to_square="e4"), on_step=watch_override,
                override=PlanOverride(grasp_dz=0.010))
    # This deliberately invalid correction is propagated into the planned
    # target, then rejected before motion because it would descend above the
    # piece crown and could hit a neighbor.
    assert result.success is False
    assert env.diagnostic_stall == "grasp_descend"
    assert seen["override_z"] - seen["base_z"] == pytest.approx(0.010)


def test_sweep_grid_starts_with_the_baseline():
    from robot_web.lab import Lab
    assert Lab.SWEEP_GRID[0] == {}
    assert any("grasp_dz" in p for p in Lab.SWEEP_GRID)
    assert any("span_deg" in p for p in Lab.SWEEP_GRID)


def test_missing_jaw_clearance_span_aborts_before_motion(env, monkeypatch):
    env.reset(chess.STARTING_FEN)
    monkeypatch.setattr(env, "grasp_span", lambda *args, **kwargs: None)
    phases = []
    result = env.execute(MoveTask(from_square="e2", to_square="e4"),
                         on_step=lambda action: phases.append(env.diagnostic_phase))

    assert result.success is False
    assert result.picked is None
    assert result.disturbed == []
    assert env.diagnostic_stall == "grasp_no_clear_span"
    assert chess.E2 in env._square_slot and chess.E4 not in env._square_slot
    assert set(phases) == {"abort_no_clear_span"}
