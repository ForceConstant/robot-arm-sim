"""Scripted pick-and-place expert for the SO-101.

Executes one chess move as a sequence of tool-space waypoints solved by IK:
approach along the tool axis, close the jaws onto the piece, carry it
world-upright above the other pieces, and set it down on the target square.
The piece is held purely by friction between the fingertip pads: the jaws
close SQUEEZE past its surface and the resulting clamp carries it, so the
carry is a slow straight tool path and the piece-to-pocket offset is measured
live before placement.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from . import gripper
from .scene import PieceSlot

StepCallback = Callable[[np.ndarray], None]  # receives the 6-D action applied

TRANSIT = 0.105
APPROACH_LENGTH = 0.06
RELEASE_TOLERANCE = 0.012   # how far the jaws may sit from the placement pose and still open
PICKUP_MIN = 0.006          # a gripped piece rises at least this much on the lift
PRECISION = 0.004            # executed pocket error accepted at grasp/place
SERVO_ROUNDS_PRECISE = 11
SERVO_ROUNDS_TRANSIT = 2
SERVO_ROUND_STEPS = 8
SETTLE_STEPS = 12            # control steps holding the closed jaws before lifting


@dataclass(frozen=True)
class GraspPlan:
    """How a piece is held: pocket height above the board, jaw gaps and the
    pocket offsets (site frame) for the open and the holding jaws."""

    z_grasp: float
    gap_open: float
    gap_hold: float
    off_open: np.ndarray
    off_hold: np.ndarray


def grasp_plan(geometry, board) -> GraspPlan:
    g = geometry
    if gripper.CUSTOM:
        # A symmetric parallel tool must clamp a section within its actual
        # aperture, not reuse the stock rotating jaw's head/waist calibration.
        # Include the complete straight finger band, not just an invisible pad.
        #
        # A set may carry a measured neck (printed sets do). Prefer it, clamp it
        # to the piece's usable height, and drive the *hold* from the section the
        # pads actually close on while the *opening* clears the widest feature
        # the fingers straddle. Using the band's widest radius for both is what
        # previously slammed the jaws shut on a crown or closed above the piece.
        low, high = gripper.PROFILE['aperture_range_m']
        proximal = gripper.PROFILE['finger_proximal_reach_m']
        if g.grasp_height is not None:
            candidates = [min(g.grasp_height, g.height - .004)]
        else:
            candidates = [g.waist] + [z1 - g.bottom_offset + gripper.TIP_X - .003
                                      for _, z1, _ in reversed(g.profile)]
        for height in candidates:
            height = max(height, gripper.TIP_CLEARANCE + gripper.TIP_X)
            if height >= g.height - .001:
                continue
            contact = g.radius_between(height - .0015, height + .0015)
            straddle = g.radius_between(height - gripper.TIP_X, height + proximal)
            hold = 2 * contact - gripper.SQUEEZE
            opening = 2 * straddle + .006
            if low <= hold and opening <= high and contact > .0005:
                return GraspPlan(z_grasp=board.top + height, gap_open=min(opening + .010, high),
                                 gap_hold=hold, off_open=np.zeros(3), off_hold=np.zeros(3))
        raise ValueError('No piece section fits the selected CAD gripper aperture and fingertip clearance')
    # Pocket height: the piece's grasp height, unless the fingertips would
    # touch the board or the pads would reach down into a wider segment
    # below the one they clamp (a rook's base flange).
    z_grasp = board.top + max(g.waist,
                              gripper.TIP_CLEARANCE + gripper.TIP_X - gripper.POCKET_X,
                              g.flange_top + gripper.PAD_REACH)
    # Open prongs must clear everything the pads span vertically on the way
    # in and out (a rook's base flange as much as a king's crown), so they
    # are positioned around the widest radius in that band; the hold gap
    # closes SQUEEZE past the grasp radius so the pads clamp the piece.
    pad_lo = z_grasp - board.top - gripper.PAD_REACH
    pad_hi = z_grasp - board.top + gripper.PAD_REACH
    clear = g.radius_between(pad_lo, pad_hi) + 0.003
    # while the jaws are open, keep the piece as close to the FIXED prong as
    # the widest radius in the pad band allows: the moving jaw opens toward
    # the free side, so the opening slack goes there rather than into an
    # occupied neighbor square
    gap_hold = 2 * g.grasp_radius - gripper.SQUEEZE
    return GraspPlan(z_grasp=z_grasp, gap_open=2 * clear + 0.010, gap_hold=gap_hold,
                     off_open=gripper.pocket_offset(2 * clear), off_hold=gripper.pocket_offset(gap_hold))


@dataclass
class PlanOverride:
    """A human correction expressed as plan parameters, not a trajectory.

    Chess failures are mostly parametric (grasp height, jaw span, placement
    offset), so a correction that changes those values stays executable by the
    same controller and is directly usable as a training episode.
    """

    grasp_dz: float = 0.0        # metres; + closes higher above the piece
    place_dx: float = 0.0
    place_dy: float = 0.0
    place_dz: float = 0.0
    span_deg: float | None = None   # force the jaw span direction
    lift_dz: float = 0.0         # extra carry clearance

    def as_dict(self) -> dict:
        return {k: v for k, v in self.__dict__.items() if v}


class PickPlaceController:
    def __init__(self, env):
        self.env = env
        self._span = None            # preferred jaw-span direction for the current phase

    # -- public --------------------------------------------------------------

    def pick_place(self, slot: PieceSlot, target_xy: tuple[float, float],
                   on_step: StepCallback | None = None,
                   source_z: float | None = None, target_z: float | None = None,
                   override: "PlanOverride | None" = None) -> bool:
        """Move `slot`'s piece so its axis stands on `target_xy`. True if all
        waypoints executed within tolerance (final outcome is verified by env).

        `source_z` / `target_z` are the surfaces the piece is lifted from and set
        down on, defaulting to the board. They matter: the discard tray stands on
        the table, a board's thickness lower, and a piece released at board
        height there falls the difference and topples."""
        env, board, g = self.env, self.env.board, slot.geometry
        env.diagnostic_stall = None      # per action, not per session: a stall must
        env.diagnostic_ik = None         # never leak into the next action's record
        px, py, _ = env.piece_position(slot)
        plan = grasp_plan(g, board)
        override = override or PlanOverride()
        z_grasp, gap_open, gap_hold = plan.z_grasp + override.grasp_dz, plan.gap_open, plan.gap_hold
        lift = z_grasp - board.top                      # grasp height above the surface
        source_z = board.top if source_z is None else source_z
        target_z = board.top if target_z is None else target_z
        off_open, off_hold = plan.off_open, plan.off_hold
        gap_release, off_release = gap_open, off_open
        grasp_pt = np.array([px, py, source_z + lift])
        place_pt = np.array([target_xy[0] + override.place_dx,
                             target_xy[1] + override.place_dy,
                             target_z + lift + override.place_dz])
        span_grasp = env.grasp_span(grasp_pt, off_open, exclude=slot)
        span_place = env.grasp_span(place_pt, off_hold, exclude=slot)
        if override.span_deg is not None:
            a = np.radians(override.span_deg)
            span_grasp = np.array([np.cos(a), np.sin(a)])
            span_place = span_grasp.copy()
        # On the opposite-side arm, an equally clear empty destination used to
        # select a 180-degree wrist flip. Avoid twisting a friction-held piece
        # when the original jaw orientation is still clear and reachable.
        if span_grasp is not None and env.jaw_clearance(place_pt[:2], span_grasp, exclude=slot) >= 0:
            trial = env.ik.solve(env.data, place_pt, offset=off_hold, span=span_grasp)
            if trial.position_error <= 0.008 and trial.tilt <= np.radians(30):
                span_place = span_grasp
        ok = True

        # approach the piece along the tool axis, close, let the clamp build
        self._span = span_grasp
        self._phase('transit', grasp_pt, gap_open, off_open)
        self._go(np.array([px, py, max(board.top, source_z) + TRANSIT]), gap_open, off_open, 22, on_step)
        approach = self._approach_axis(grasp_pt, off_open)
        self._phase('approach', grasp_pt, gap_open, off_open)
        self._go(grasp_pt - approach * APPROACH_LENGTH, gap_open, off_open, 14, on_step)
        ok &= self._descend(grasp_pt, gap_open, off_open, approach, on_step)
        self._phase('close', grasp_pt, gap_hold, off_hold)
        self._go(grasp_pt, gap_hold, off_hold, 10, on_step)
        self._settle(SETTLE_STEPS, gap_hold, on_step)

        # lift and carry high enough to clear standing pieces; a friction-held
        # piece cannot take jerk, so the carry is a slow straight tool path
        lift_pt = grasp_pt - approach * (APPROACH_LENGTH + 0.01)
        self._phase('lift', lift_pt, gap_hold, off_hold)
        was_z = float(env.piece_position(slot)[2])
        self._line(grasp_pt, lift_pt, gap_hold, off_hold, on_step)
        carry_z = board.top + TRANSIT + override.lift_dz
        above_src = np.array([px, py, carry_z])
        above_dst = np.array([target_xy[0], target_xy[1], carry_z])
        self._phase('carry', above_dst, gap_hold, off_hold)
        self._line(lift_pt, above_src, gap_hold, off_hold, on_step, segments=2, steps=6)
        # Measured at carry height, not at the end of the lift command: the servo
        # is still accelerating there, and a gripped piece has risen only a few
        # millimetres when the check used to fire — it rejected good grasps.
        if float(env.piece_position(slot)[2]) - was_z < PICKUP_MIN:
            # The jaws closed on air (or the piece slipped straight out). Carrying
            # it on reports the failure as a placement error the width of the move,
            # which is how a missed grasp hid as bad aiming.
            return self._abort_missed_grasp(px, py, gap_open, off_open, on_step)
        self._line(above_src, above_dst, gap_hold, off_hold, on_step,
                   segments=max(2, int(np.linalg.norm(above_dst - above_src) / 0.03)), steps=8)

        # turn to the placing span above the target (the roll may swing the
        # held piece around the tool axis), then measure where the piece sits
        # relative to the pocket so the PIECE lands centered on the square
        self._span = span_place
        approach = self._approach_axis(place_pt, off_hold)
        self._phase('place_approach', place_pt, gap_hold, off_hold)
        self._go(place_pt - approach * APPROACH_LENGTH, gap_hold, off_hold, 14, on_step)
        pocket, _ = env.ik.tool_pose(env.data, off_hold)
        axis = env.piece_position(slot)[:2] + g.center[:2]
        place_pt[:2] -= axis - pocket[:2]

        # set down along the tool axis, release, retreat
        self._phase('place_descend', place_pt, gap_hold, off_hold)
        arrived = self._descend(place_pt, gap_hold, off_hold, approach, on_step)
        held = env.ik.tool_pose(env.data, off_hold)[0]
        if not arrived or float(np.linalg.norm(held - place_pt)) > RELEASE_TOLERANCE:
            # A tracking failure here used to fall through to the release: the
            # jaws opened wherever the arm happened to stop, sometimes 50 mm up,
            # and the piece was dropped. Put it back instead.
            return self._abort_to_source(grasp_pt, px, py, span_grasp, span_place,
                                         gap_hold, off_hold, gap_open, off_open, on_step)
        ok &= arrived
        # open the jaws, then shift the tool so the piece sits centered between
        # the open prongs (the wider gap's pocket lies further from the fixed
        # prong) and retreat straight up: backing out along a tilted tool axis
        # would sweep a prong through the piece
        self._phase('release', place_pt, gap_release, off_hold)
        self._go(place_pt, gap_release, off_hold, 8, on_step)
        self._go(place_pt, gap_release, off_release, 6, on_step)
        clear_pt = np.array([target_xy[0], target_xy[1], target_z + g.height + 0.03])
        self._phase('retreat', clear_pt, gap_release, off_release)
        self._line(place_pt, clear_pt, gap_release, off_release, on_step, segments=4)
        self._go(np.array([target_xy[0], target_xy[1], board.top + TRANSIT]), gap_open, off_open, 10, on_step)
        self._phase('done', clear_pt, gap_open, off_open)
        return bool(ok)

    def _abort_missed_grasp(self, px, py, gap_open, off_open, on_step) -> bool:
        """Nothing came up with the jaws: open them, retreat, report the miss.

        The piece is still standing on its square, so no undo is needed — only
        the honesty of not carrying empty jaws to the target square.
        """
        env, board = self.env, self.env.board
        env.diagnostic_stall = 'lift'
        here = env.ik.tool_pose(env.data, off_open)[0]
        self._phase('abort_grasp', here, gap_open, off_open)
        self._go(here, gap_open, off_open, 8, on_step)
        clear = np.array([here[0], here[1], board.top + TRANSIT])
        self._line(here, clear, gap_open, off_open, on_step, segments=3)
        return False

    def _abort_to_source(self, grasp_pt, px, py, span_grasp, span_place,
                         gap_hold, off_hold, gap_open, off_open, on_step) -> bool:
        """Carry the piece back to the square it came from and set it down.

        Returning a piece to its source is the only honest recovery: leaving it
        in the jaws parks the arm holding a piece, and releasing mid-air is the
        failure this guard exists to stop. The attempt still reports failure.
        """
        env, board = self.env, self.env.board
        env.diagnostic_stall = 'place_descend'
        self._span = span_grasp
        carry_z = board.top + TRANSIT
        here = env.ik.tool_pose(env.data, off_hold)[0]
        safe = np.array([here[0], here[1], carry_z])
        self._phase('abort_return', safe, gap_hold, off_hold)
        self._line(here, safe, gap_hold, off_hold, on_step, segments=3)
        above_src = np.array([px, py, carry_z])
        self._line(safe, above_src, gap_hold, off_hold, on_step, segments=3)
        approach = self._approach_axis(grasp_pt, off_hold)
        ok = self._descend(grasp_pt, gap_hold, off_hold, approach, on_step)
        self._span = span_place
        self._phase('abort_release', grasp_pt, gap_open, off_open)
        self._go(grasp_pt, gap_open, off_hold, 8, on_step)
        self._go(grasp_pt, gap_open, off_open, 6, on_step)
        clear_pt = np.array([px, py, grasp_pt[2] + 0.06])
        self._line(grasp_pt, clear_pt, gap_open, off_open, on_step, segments=3)
        self._go(np.array([px, py, carry_z]), gap_open, off_open, 8, on_step)
        return False

    # -- motion primitives ---------------------------------------------------

    def _phase(self, name: str, target=None, gap=None, offset=None) -> None:
        """Label the intended phase for the recorder. It records intent only;
        the trajectory is still whatever the servos actually did."""
        env = self.env
        env.diagnostic_phase = name
        env.diagnostic_target = None if target is None else np.asarray(target, dtype=float).tolist()
        env.diagnostic_gap = None if gap is None else float(gap)
        env.diagnostic_offset = (np.zeros(3) if offset is None
                                 else np.asarray(offset, dtype=float).copy())

    def _approach_axis(self, target, offset) -> np.ndarray:
        res = self.env.ik.solve(self.env.data, target, offset=offset, span=self._span)
        _, mat = self.env.ik.forward(res.q, offset)
        return mat[:, 0]

    def _line(self, start, target, gap, offset, on_step, precise=False,
              segments: int = 3, steps: int = 5) -> bool:
        """Straight tool-space path: joint-space interpolation over a long move
        bows sideways by more than the clearance around a standing piece."""
        for k in range(1, segments):
            self._go(start + (target - start) * (k / segments), gap, offset, steps, on_step)
        return self._go(target, gap, offset, steps + 3, on_step, precise=precise)

    def _settle(self, steps: int, gap: float, on_step) -> None:
        """Hold the current arm command (bias included) for a few control steps."""
        action = self.env._current_action()
        action[5] = gripper.gap_to_angle(gap)
        for _ in range(steps):
            self.env.apply_action(action, on_step)

    def _descend(self, target, gap, offset, approach, on_step) -> bool:
        return self._line(target - approach * APPROACH_LENGTH, target, gap, offset,
                          on_step, precise=True)

    def _go(self, target, gap, offset, steps, on_step, precise=False) -> bool:
        """Interpolate to the IK solution, then hold until the servos converge."""
        env = self.env
        # Execution noise (ActionNoiseConfig) stays on the transit legs, where a
        # policy drifts; a grasp or place servo has a fixed budget of rounds and
        # cannot cancel a perturbation that keeps moving under it.
        env.noise_suppressed = precise
        res = env.ik.solve(env.data, target, offset=offset, span=self._span)
        # What the solver thought, not just where the arm ended up: a stalled
        # descent looks identical to a slow one in the trace otherwise.
        span = None if self._span is None else np.asarray(self._span, dtype=float)
        env.diagnostic_ik = {"residual_m": float(res.position_error),
                             "tilt_rad": float(res.tilt),
                             "span_deg": None if span is None else float(
                                 np.degrees(np.arctan2(span[1], span[0]))),
                             "offset": np.asarray(offset, dtype=float).tolist()}
        q_start = env.arm_joint_positions()[:5]
        grip = gripper.gap_to_angle(gap)
        for i in range(1, steps + 1):
            q = q_start + (res.q - q_start) * (i / steps)
            env.apply_action(np.append(q, grip), on_step)

        # Position servos have pose-dependent steady-state error (no gravity
        # compensation). Bias the command every few steps until the executed
        # tool point matches; per-step updates would limit-cycle.
        bias = np.zeros(5)
        rounds = SERVO_ROUNDS_PRECISE if precise else SERVO_ROUNDS_TRANSIT
        for _ in range(rounds):
            pos, _ = env.ik.tool_pose(env.data, offset)
            if np.linalg.norm(pos - target) < PRECISION:
                break
            bias = np.clip(bias + 0.6 * (res.q - env.arm_joint_positions()[:5]), -0.45, 0.45)
            for _ in range(SERVO_ROUND_STEPS):
                env.apply_action(np.append(res.q + bias, grip), on_step)
        pos, _ = env.ik.tool_pose(env.data, offset)
        env.noise_suppressed = False
        return bool(np.linalg.norm(pos - target) < 2.5 * PRECISION) if precise else True
