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
import mink
import mujoco

from . import gripper
from .scene import PieceSlot

StepCallback = Callable[[np.ndarray], None]  # receives the 6-D action applied

# Extra clearance for the symmetric CAD parallel tool prevents wrist/forearm
# sweep from disturbing neighboring pieces during dense-layout carries.
# Phase 10's current-layout queen case at g6-f5 separated at 0.13 m; 0.105 m
# caused a repeatable collateral displacement despite sub-millimeter fused
# localization. Keep the stock gripper at its previously qualified transit.
TRANSIT = 0.105
TRANSIT_CUSTOM = 0.13
APPROACH_LENGTH = 0.06
# Bounded release tolerance (approved 2026-11-09): opening the jaws on a dense
# square can sweep a prong into a neighbour by a few tenths of a millimetre.
# A graze this shallow that moves no other piece is contained, not a failure.
CONTAINED_CONTACT_DEPTH = 0.0005     # 0.5 mm deepest robot-to-neighbour overlap
CONTAINED_DISPLACEMENT = 0.0001      # 0.1 mm any other piece may have moved
CONTAINED_PHASES = ('release', 'retreat')
RELEASE_TOLERANCE = 0.012   # how far the jaws may sit from the placement pose and still open
PICKUP_MIN = 0.006          # a gripped piece rises at least this much on the lift
PRECISION = 0.004            # executed pocket error accepted at grasp/place
SERVO_ROUNDS_PRECISE = 11
SERVO_ROUNDS_TRANSIT = 2
SERVO_ROUND_STEPS = 8
SETTLE_STEPS = 12            # control steps holding the closed jaws before lifting
# Extra physical closure for an emergency source-return with the imported
# parallel CAD tool; ordinary grasps retain the calibrated grasp-plan squeeze.
RECOVERY_EXTRA_SQUEEZE = 0.0015
IK_FALLBACK_TRIALS = 120
IK_FALLBACK_SEED = 91027
IK_FALLBACK_MAX_JOINT_STEP = 0.5
# Seating allowance, not a free pass: at the 4.8 g sensitivity bound a piece
# can rotate a few millimetres into the pads before it is actually held. At
# 4.0 mm the guard aborted two such carries (held-out cases 20 and 95); at
# 5.0 mm both seat and complete with zero collateral and 2-3 mm placement.
# Any larger shift is still an abort, and every contact guard stays active.
GRIP_SLIP_TOLERANCE = 0.005
SPAN_MARGIN_TOLERANCE = 0.008  # heuristic slack only; physical contact guards remain active


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
            # Tip depth and width calibration are separate. A long parallel
            # finger can straddle two collars only when its tip sits below the
            # lower one; closing at the neck above it contacts just the crown
            # and leaves a tall piece free to pivot. Pack-specific calibration
            # may lower the tip without silently changing the jaw closure.
            candidates = [min(g.grasp_height + g.grasp_tip_offset, g.height - .004)]
        else:
            candidates = [g.waist] + [z1 - g.bottom_offset + gripper.TIP_X - .003
                                      for _, z1, _ in reversed(g.profile)]
        for height in candidates:
            height = max(height, gripper.TIP_CLEARANCE + gripper.TIP_X)
            if height >= g.height - .001:
                continue
            # Hold across the real width of the section, not an equivalent radius:
            # an asymmetric piece is wider along one board axis, and closing to
            # the circular estimate drives the jaws into it.
            contact = (0.5 * g.grasp_width) if g.grasp_width else g.radius_between(height - .0015, height + .0015)
            straddle = g.radius_between(height - gripper.TIP_X, height + proximal)
            squeeze = gripper.SQUEEZE if g.grasp_squeeze is None else g.grasp_squeeze
            hold = 2 * contact - squeeze
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
        self._moving_slot = None
        self._lift_reference_local = None
        self._abort_reason = None
        self._release_baseline = None   # piece poses when the release began
        self._release_contained = 0     # contained grazes counted this release

    @staticmethod
    def _transit_height() -> float:
        return TRANSIT_CUSTOM if gripper.CUSTOM else TRANSIT

    def _feasible_transit(self, xy, surface_z, span, offset, nominal, slot,
                          allow_flip=True):
        """Highest transit this wrist can actually hold over the source.

        The 0.13 m custom transit is calibrated for mid-board work. From the
        near-corner squares that waypoint is outside the arm's reach, and the
        executed move then stops short with a large IK residual instead of
        failing loudly. A parallel piece is two-fold symmetric, so the opposite
        jaw direction grips the same faces and is often the only one the wrist
        can present. Keep the nominal pose when it works; otherwise take the
        highest reachable height and direction that also has jaw clearance.
        """
        if span is None:
            return nominal, span
        env = self.env
        base_z = max(env.board.top, surface_z)
        seed = env.arm_joint_positions()[:5]
        candidates = (np.asarray(span, dtype=float), -np.asarray(span, dtype=float))
        for candidate in (candidates if allow_flip else candidates[:1]):
            if env.jaw_clearance(xy, candidate, exclude=slot) < 0:
                continue
            for height in (nominal, 0.11, 0.10, 0.09, 0.085, 0.075):
                if height > nominal + 1e-9:
                    continue
                solution = env.ik.solve(
                    env.data, np.r_[np.asarray(xy, dtype=float), base_z + height],
                    offset=offset, span=candidate, seed_q=seed)
                if (solution.position_error <= PRECISION
                        and solution.tilt <= np.radians(15)):
                    return height, candidate
        return nominal, span

    # -- public --------------------------------------------------------------

    def pick_place(self, *args, **kwargs):
        """Grip a hinted piece with a stronger upright preference.

        A parallel grasp on a narrow section only works if the wrist stays
        square to it; the default posture cost lets the solver trade that away
        for reach, which swings a jaw toward a neighbour. The cost is scoped to
        this call so the rest of the session is unchanged.
        """
        hint = kwargs.get("span_hint")
        old_cost = float(self.env.ik._tilt.cost[0])
        saved_config = self.env.config
        if hint is not None:
            self.env.ik._tilt.set_cost(1.0)
        try:
            return self._pick_place(*args, **kwargs)
        finally:
            self.env.ik._tilt.set_cost(old_cost)
            self.env.config = saved_config

    def _pick_place(self, slot: PieceSlot, target_xy: tuple[float, float],
                   on_step: StepCallback | None = None,
                   source_z: float | None = None, target_z: float | None = None,
                   override: "PlanOverride | None" = None,
                   source_xy: tuple[float, float] | None = None,
                   reobserve_source: Callable[[], object | None] | None = None,
                   span_hint: np.ndarray | None = None) -> bool:
        """Move `slot`'s piece so its axis stands on `target_xy`. True if all
        waypoints executed within tolerance (final outcome is verified by env).

        `source_z` / `target_z` are the surfaces the piece is lifted from and set
        down on, defaulting to the board. They matter: the discard tray stands on
        the table, a board's thickness lower, and a piece released at board
        height there falls the difference and topples."""
        env, board, g = self.env, self.env.board, slot.geometry
        self._moving_slot = slot
        self._abort_reason = None
        self._failure_reason = None
        self._placement_retreat_path = []
        transit = self._transit_height()
        # A piece may carry a measured transit height it actually tolerates.
        # The queen pivots at the 0.13 m default: unreachable from the near
        # rank, so the wrist tilts and the piece rotates in the pads (cases 37
        # and 111). 0.11 m completes both; 0.10 and 0.09 also pass, 0.13 does
        # not. Only lower, never raise, so ordinary pieces are unchanged.
        measured_transit = getattr(g, "transit_m", None)
        if gripper.CUSTOM and measured_transit:
            transit = min(transit, float(measured_transit))
        env.diagnostic_stall = None      # per action, not per session: a stall must
        env.diagnostic_ik = None         # never leak into the next action's record
        # A perception-driven caller supplies its observed localization here;
        # the normal expert path keeps the historical simulator-state target.
        px, py = (env.piece_position(slot)[:2] if source_xy is None
                  else np.asarray(source_xy, dtype=float))
        plan = grasp_plan(g, board)
        override = override or PlanOverride()
        z_grasp, gap_open, gap_hold = plan.z_grasp + override.grasp_dz, plan.gap_open, plan.gap_hold
        lift = z_grasp - board.top                      # grasp height above the surface
        source_z = board.top if source_z is None else source_z
        target_z = board.top if target_z is None else target_z
        if z_grasp > source_z + g.height - 0.001:
            # A bad calibration/override must not move the jaws through a
            # neighboring piece while trying to reach a grasp point above the
            # target's crown. Reject before any motion; lift monitoring is too
            # late to prevent collateral if the descent itself hits another
            # piece.
            self._record_failure('grasp_descend')
            self._phase('abort_invalid_grasp_height', np.array([px, py, z_grasp]),
                        gap_open, plan.off_open)
            return False
        off_open, off_hold = plan.off_open, plan.off_hold
        gap_release, off_release = gap_open, off_open
        if span_hint is not None and g.grasp_width:
            # The hint names the piece's flat faces, so the jaws only need to
            # straddle that calibrated width. Opening to the widest outline
            # instead is what sweeps a prong through a neighbouring piece at a
            # corner square.
            section = g.thin_width or g.grasp_width
            gap_open = min(gap_open, max(0.0165, section + 0.008))
            # A narrowed grasp opening is not a release opening: the piece lands
            # a millimetre off centre and the jaws must still clear it. Keep
            # this tied to the narrowed case so ordinary releases are unchanged.
            gap_release = max(gap_open, 0.022)
        grasp_pt = np.array([px, py, source_z + lift])
        place_pt = np.array([target_xy[0] + override.place_dx,
                             target_xy[1] + override.place_dy,
                             target_z + lift + override.place_dz])
        if span_hint is not None and override.span_deg is None:
            # A perception layer may know the piece's flat faces. A parallel
            # piece is two-fold symmetric, so either direction of that axis is a
            # valid grip; take the first one the jaws clear, and only fall back
            # to the clearance solver when neither does.
            hint = np.asarray(span_hint, dtype=float)
            hint = hint / np.linalg.norm(hint)
            # A narrow parallel grasp needs symmetric clearance around the
            # calibrated section. The stock heuristic reserves a wide arc for
            # the swinging jaw, which rejects the flat-face direction outright;
            # judge this grasp against the aperture it actually uses.
            # Keep these rooms for the whole hinted pick, not just this check:
            # the fallback span search must judge the same aperture, or a
            # reachable grasp is refused on the stock heuristic.
            room = gap_open / 2 + 0.0025 + 0.002
            env.config = env.config.model_copy(update={"clearance": env.config.clearance.model_copy(
                update={"moving_jaw_room": room, "fixed_prong_room": room})})
            span_grasp = next((c for c in (hint, -hint)
                               if env.jaw_clearance(grasp_pt[:2], c, exclude=slot)
                               >= -SPAN_MARGIN_TOLERANCE), None)
            if span_grasp is None:
                # A hint whose both directions are blocked must not turn a
                # graspable piece into a refusal; the clearance solver still
                # owns the fallback.
                span_grasp = env.grasp_span(grasp_pt, off_open, exclude=slot)
        else:
            span_grasp = env.grasp_span(grasp_pt, off_open, exclude=slot)
        # The nominal 0.13 m transit is out of reach from some squares, and an
        # unreachable waypoint leaves a large IK residual that tilts the wrist
        # and pivots the held piece instead of failing loudly (queen d1-e2).
        # Lower it to the highest height this span can hold; only a hinted
        # grasp may also switch the jaw direction.
        transit, feasible = self._feasible_transit(
            (px, py), source_z, span_grasp, off_open, transit, slot,
            allow_flip=span_hint is not None and override.span_deg is None)
        if span_hint is not None and override.span_deg is None:
            span_grasp = feasible
        span_place = env.grasp_span(place_pt, off_hold, exclude=slot)
        if override.span_deg is not None:
            a = np.radians(override.span_deg)
            span_grasp = np.array([np.cos(a), np.sin(a)])
            span_place = span_grasp.copy()
            if (env.jaw_clearance(grasp_pt[:2], span_grasp, exclude=slot) < -SPAN_MARGIN_TOLERANCE or
                    env.jaw_clearance(place_pt[:2], span_place, exclude=slot) < -SPAN_MARGIN_TOLERANCE):
                self._record_failure('override_no_clear_span')
                self._phase('abort_no_clear_span', grasp_pt, gap_open, off_open)
                return False
        # Never interpret a missing clearance solution as permission to use the
        # solver's default wrist orientation. That fallback is unconstrained and
        # can sweep a jaw through a neighboring piece during approach.
        if span_grasp is None:
            self._record_failure('grasp_no_clear_span')
            env.diagnostic_ik = {"reason": "no collision-clear jaw span",
                "candidates": [(d.tolist(), float(m))
                               for d, m in env.span_options(grasp_pt[:2], exclude=slot)]}
            self._phase('abort_no_clear_span', grasp_pt, gap_open, off_open)
            return False
        # On the opposite-side arm, an equally clear empty destination used to
        # select a 180-degree wrist flip. Avoid twisting a friction-held piece
        # when the original jaw orientation is still clear and reachable.
        if env.jaw_clearance(place_pt[:2], span_grasp, exclude=slot) >= 0:
            trial = env.ik.solve(env.data, place_pt, offset=off_hold, span=span_grasp)
            if trial.position_error <= 0.008 and trial.tilt <= np.radians(30):
                span_place = span_grasp
        if span_place is None:
            self._record_failure('place_no_clear_span')
            env.diagnostic_ik = {"reason": "no collision-clear destination jaw span",
                "candidates": [(d.tolist(), float(m))
                               for d, m in env.span_options(place_pt[:2], exclude=slot)]}
            self._phase('abort_no_clear_span', place_pt, gap_open, off_open)
            return False
        ok = True

        # approach the piece along the tool axis, close, let the clamp build
        self._span = span_grasp
        self._phase('transit', grasp_pt, gap_open, off_open)
        self._go(np.array([px, py, max(board.top, source_z) + transit]), gap_open, off_open, 22, on_step)
        if env.diagnostic_stall == 'gripper_neighbor_contact':
            return False
        if reobserve_source is not None:
            # The arm can still occlude the source from the transit waypoint.
            # Return along the known approach path to the parked observation pose
            # before taking a new frame. The callback consumes pixels/calibration,
            # not simulator piece pose.
            self._phase('vision_reobserve_park', None, gap_open, off_open)
            self._move_joints(env.home_pose()[:5], gap_open, on_step)
            if self._abort_reason:
                return False
            observed = reobserve_source()
            if observed is None or not observed.acceptable():
                self._record_failure('vision_reobserve_abstain')
                return False
            if np.linalg.norm(np.asarray(observed.world_xy) - (px, py)) > 0.004:
                # A second confident-looking but inconsistent component is
                # often an occluding arm link, not piece motion. Do not let it
                # redirect a grasp; retreat and surface the disagreement.
                self._record_failure('vision_reobserve_inconsistent')
                self._go(np.array([px, py, board.top + transit]), gap_open,
                         off_open, 10, on_step)
                return False
            px, py = observed.world_xy
            grasp_pt[:2] = (px, py)
            self._phase('transit', grasp_pt, gap_open, off_open)
            # Re-observation can leave the wrist on a different IK branch.
            # Use the full initial-transit servo budget here too: an 8-step
            # return overshoots below the high waypoint through the pawn rank.
            self._go(np.array([px, py, max(board.top, source_z) + transit]),
                     gap_open, off_open, 22, on_step)
            if self._abort_reason:
                return False
        approach = self._approach_axis(grasp_pt, off_open)
        self._phase('approach', grasp_pt, gap_open, off_open)
        self._go(grasp_pt - approach * APPROACH_LENGTH, gap_open, off_open, 14, on_step)
        grasp_contact = env.diagnostic_stall == 'gripper_neighbor_contact'
        if grasp_contact and span_grasp is not None:
            # The jaws are still open and nothing is held: the approach axis
            # only grazed a neighbour. A jaw axis is symmetric mod 180 degrees,
            # so a negated span is the same axis. Try the PERPENDICULAR axis
            # once; the descent that follows is guarded like any other.
            span_vec = np.asarray(span_grasp, dtype=float)
            perpendicular = np.array([-span_vec[1], span_vec[0]])
            self._clear_grasp_contact_abort()
            self._span = perpendicular
            self._go(grasp_pt - approach * APPROACH_LENGTH, gap_open, off_open, 14, on_step)
            if env.diagnostic_stall == 'gripper_neighbor_contact':
                self._latch_abort('gripper_neighbor_contact')
                return False
            span_grasp = perpendicular
            grasp_contact = False
        if not grasp_contact:
            ok &= self._descend(grasp_pt, gap_open, off_open, approach, on_step)
            if env.diagnostic_stall == 'gripper_neighbor_contact':
                return False
        if not ok and span_grasp is not None:
            # The descent is gated on the *executed* tool point, so a jaw swinging
            # into a neighbour leaves the IK residual tiny while the arm never
            # arrives. Retry once with the opposite span: which side is clear
            # depends on the piece and on the arm's mount, and hard-coding a sign
            # for the mirrored arm traded one passing case for another.
            flipped = -np.asarray(span_grasp, dtype=float)
            self._span = flipped
            self._go(grasp_pt - approach * APPROACH_LENGTH, gap_open, off_open, 14, on_step)
            if env.diagnostic_stall == 'gripper_neighbor_contact':
                # Second graze on the other span: restore the hard verdict.
                self._latch_abort('gripper_neighbor_contact')
                return False
            if self._descend(grasp_pt, gap_open, off_open, approach, on_step):
                span_grasp = flipped
                ok = True
                trial = env.ik.solve(env.data, place_pt, offset=off_hold, span=flipped)
                if (trial.position_error <= 0.008 and trial.tilt <= np.radians(30)
                        and env.jaw_clearance(place_pt[:2], flipped, exclude=slot) >= 0):
                    span_place = flipped
        if not ok:
            # Never turn a failed positioning waypoint into a grasp at the
            # neighbouring square. Retreat with open jaws and preserve failure.
            self._record_failure('grasp_descend')
            self._abort_missed_grasp(px, py, gap_open, off_open, on_step)
            return False
        self._phase('close', grasp_pt, gap_hold, off_hold)
        self._go(grasp_pt, gap_hold, off_hold, 10, on_step)
        if env.diagnostic_stall == 'gripper_neighbor_contact':
            return False
        self._settle(SETTLE_STEPS, gap_hold, on_step)
        if env.diagnostic_stall == 'gripper_neighbor_contact':
            return False

        # lift and carry high enough to clear standing pieces; a friction-held
        # piece cannot take jerk, so the carry is a slow straight tool path
        # Continue along the tool axis, matching the grip direction. The
        # per-step neighbor-contact and XY-stability guards below stop the
        # lift before a sideways sweep can become a collateral move.
        # First clear the board vertically. Following a slightly tilted tool
        # axis adds lateral travel while the piece is still near its neighbors
        # and can also make the wrist target unreachable at the rear rank.
        lift_pt = grasp_pt + np.array([0.0, 0.0, 0.04])
        self._phase('lift', lift_pt, gap_hold, off_hold)
        was_z = float(env.piece_position(slot)[2])
        self._lift_reference_local = self._piece_gripper_local(slot)
        lifted = self._line(grasp_pt, lift_pt, gap_hold, off_hold, on_step,
                            precise=True, steps=12)
        if env.diagnostic_stall in ('lift_neighbor_contact', 'lift_unstable',
                                    'gripper_neighbor_contact'):
            # Hold position and keep the jaws closed. Any further recovery
            # trajectory could turn the first contact into a shove or drop.
            return False
        if not lifted:
            self._record_failure('lift_waypoint_miss')
            return self._abort_to_source(grasp_pt, px, py, span_grasp, span_place,
                                         gap_hold, off_hold, gap_open, off_open, on_step)
        carry_z = board.top + transit + override.lift_dz
        above_src = np.array([px, py, carry_z])
        above_dst = np.array([target_xy[0], target_xy[1], carry_z])
        self._phase('carry', above_dst, gap_hold, off_hold)
        if not self._line(lift_pt, above_src, gap_hold, off_hold, on_step,
                          segments=2, steps=12) or self._abort_reason:
            return False
        # Measured at carry height, not at the end of the lift command: the servo
        # is still accelerating there, and a gripped piece has risen only a few
        # millimetres when the check used to fire — it rejected good grasps.
        if float(env.piece_position(slot)[2]) - was_z < PICKUP_MIN:
            # The jaws closed on air (or the piece slipped straight out). Carrying
            # it on reports the failure as a placement error the width of the move,
            # which is how a missed grasp hid as bad aiming.
            return self._abort_missed_grasp(px, py, gap_open, off_open, on_step)
        if not self._line(above_src, above_dst, gap_hold, off_hold, on_step,
                          segments=max(2, int(np.linalg.norm(above_dst - above_src) / 0.03)), steps=12):
            self._record_failure('carry_waypoint_miss')
            return False

        # turn to the placing span above the target (the roll may swing the
        # held piece around the tool axis), then measure where the piece sits
        # relative to the pocket so the PIECE lands centered on the square
        self._span = span_place
        # Placement must not inherit the tool's tilt as a lateral sweep. On
        # dense layouts that can drag a correctly grasped piece into the
        # neighboring square during the final descent. Keep the held piece's
        # XY fixed and descend vertically in world coordinates.
        approach = self._placement_axis(place_pt, slot)
        self._phase('place_approach', place_pt, gap_hold, off_hold)
        if not self._go(place_pt - approach * APPROACH_LENGTH, gap_hold, off_hold, 14, on_step):
            self._record_failure('place_approach_miss')
            return False
        pocket, _ = env.ik.tool_pose(env.data, off_hold)
        axis = env.piece_position(slot)[:2] + g.center[:2]
        place_pt[:2] -= axis - pocket[:2]

        # set down along the tool axis, release, retreat
        self._phase('place_descend', place_pt, gap_hold, off_hold)
        arrived = self._placement_descent(place_pt, gap_hold, off_hold, approach, on_step)
        if self._abort_reason:
            # Do not release or attempt another placement path once the held
            # piece touches a neighbor. Keep the grasp and deny game-state commit.
            return False
        if not arrived and span_place is not None:
            # Same gate as the grasp: this descent is judged on the executed tool
            # point, and on the mirrored arm the moving jaw can swing into the
            # destination's neighbours while the IK residual stays tiny. Measured
            # on black g8-f6: both grasp spans failed, yet forcing the opposite
            # placement span made the whole move succeed.
            flipped_place = -np.asarray(span_place, dtype=float)
            trial = env.ik.solve(env.data, place_pt, offset=off_hold, span=flipped_place)
            if trial.position_error <= 0.008 and trial.tilt <= np.radians(30):
                self._span = flipped_place
                retry_approach = self._go(place_pt - approach * APPROACH_LENGTH,
                                          gap_hold, off_hold, 14, on_step)
                if self._abort_reason:
                    return False
                if retry_approach and self._placement_descent(place_pt, gap_hold, off_hold, approach, on_step):
                    span_place = flipped_place
                    arrived = True
                if self._abort_reason:
                    return False
        held = env.ik.tool_pose(env.data, off_hold)[0]
        if not arrived or float(np.linalg.norm(held - place_pt)) > RELEASE_TOLERANCE:
            # A tracking failure here used to fall through to the release: the
            # jaws opened wherever the arm happened to stop, sometimes 50 mm up,
            # and the piece was dropped. Put it back instead.
            self._record_failure('place_descend')
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
        if gripper.CUSTOM:
            if not self._reverse_placement_retreat(gap_release, on_step):
                return False
        else:
            self._line(place_pt, clear_pt, gap_release, off_release, on_step, segments=4)
        self._go(np.array([target_xy[0], target_xy[1], board.top + transit]), gap_open, off_open, 10, on_step)
        self._phase('done', clear_pt, gap_open, off_open)
        return bool(ok)

    def _abort_missed_grasp(self, px, py, gap_open, off_open, on_step) -> bool:
        """Nothing came up with the jaws: open them, retreat, report the miss.

        The piece is still standing on its square, so no undo is needed — only
        the honesty of not carrying empty jaws to the target square.
        """
        env, board = self.env, self.env.board
        self._record_failure('lift')
        here = env.ik.tool_pose(env.data, off_open)[0]
        self._phase('abort_grasp', here, gap_open, off_open)
        self._go(here, gap_open, off_open, 8, on_step)
        clear = np.array([here[0], here[1], board.top + self._transit_height()])
        self._line(here, clear, gap_open, off_open, on_step, segments=3)
        return False

    def _abort_to_source(self, grasp_pt, px, py, span_grasp, span_place,
                         gap_hold, off_hold, gap_open, off_open, on_step) -> bool:
        """Carry the piece back to the square it came from and set it down.

        Returning a piece to its source is the only honest recovery: leaving it
        in the jaws parks the arm holding a piece, and releasing mid-air is the
        failure this guard exists to stop. The attempt still reports failure.
        """
        if self._abort_reason:
            return False
        env, board = self.env, self.env.board
        if self._failure_reason == 'lift_waypoint_miss':
            # The lift never confirmed: the piece may still be clutched between
            # the pads a few millimetres above its own square. Carrying it
            # sideways from here drags it through its neighbours - measured
            # 2026-11-09, queen d1-e1 shoved the white rook 20 mm during
            # abort_return, the only collateral in the 1536-case dev matrix.
            # Open the pads where the piece stands, then retreat vertically.
            here = np.asarray(env.ik.tool_pose(env.data, off_hold)[0], dtype=float)
            self._phase('abort_release', here, gap_open, off_open)
            self._go(here, gap_open, off_hold, 8, on_step)
            self._go(here, gap_open, off_open, 6, on_step)
            up = np.array([here[0], here[1], board.top + self._transit_height()])
            self._phase('abort_return', up, gap_open, off_open)
            self._line(here, up, gap_open, off_open, on_step, segments=3)
            return False
        self._span = span_grasp
        recovery_gap = gap_hold
        if gripper.CUSTOM:
            aperture_min = gripper.PROFILE['aperture_range_m'][0]
            recovery_gap = max(aperture_min, gap_hold - RECOVERY_EXTRA_SQUEEZE)
        carry_z = board.top + self._transit_height()
        here = env.ik.tool_pose(env.data, off_hold)[0]
        safe = np.array([here[0], here[1], carry_z])
        self._phase('abort_return', safe, recovery_gap, off_hold)
        if not self._line(here, safe, recovery_gap, off_hold, on_step, segments=3):
            return False
        above_src = np.array([px, py, carry_z])
        if not self._line(safe, above_src, recovery_gap, off_hold, on_step, segments=3):
            return False
        approach = self._approach_axis(grasp_pt, off_hold)
        ok = self._descend(grasp_pt, recovery_gap, off_hold, approach, on_step)
        if not ok or self._abort_reason:
            return False
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
        if name == 'release':
            # Frozen pose of every other piece: the containment test needs to
            # prove that a release graze moved nothing.
            self._release_baseline = {
                slot.body: np.array(env.piece_position(slot), dtype=float)
                for slot in env.active_slots()}
            self._release_contained = 0

    def _approach_axis(self, target, offset) -> np.ndarray:
        res = self.env.ik.solve(self.env.data, target, offset=offset, span=self._span)
        _, mat = self.env.ik.forward(res.q, offset)
        return mat[:, 0]

    def _line(self, start, target, gap, offset, on_step, precise=False,
              segments: int = 3, steps: int = 5) -> bool:
        """Straight tool-space path: joint-space interpolation over a long move
        bows sideways by more than the clearance around a standing piece."""
        if self._abort_reason:
            return False
        for k in range(1, segments):
            if not self._go(start + (target - start) * (k / segments),
                            gap, offset, steps, on_step, precise=precise):
                return False
        return self._go(target, gap, offset, steps + 3, on_step, precise=precise)

    def _settle(self, steps: int, gap: float, on_step) -> None:
        """Hold the current arm command (bias included) for a few control steps."""
        action = self.env._current_action()
        action[5] = gripper.gap_to_angle(gap)
        for _ in range(steps):
            self.env.apply_action(action, on_step)
            if self._guard_gripper_neighbor_contact():
                return

    def _move_joints(self, target: np.ndarray, gap: float, on_step,
                     steps: int = 18) -> None:
        """Interpolate a short, known-clear return to the parked observation pose."""
        start = self.env.arm_joint_positions()[:5].copy()
        grip = gripper.gap_to_angle(gap)
        for index in range(1, steps + 1):
            fraction = index / steps
            self.env.apply_action(np.append(start + (target - start) * fraction, grip), on_step)
            if self._guard_gripper_neighbor_contact():
                return
        parked = np.append(target, grip)
        for _ in range(18):
            self.env.apply_action(parked, on_step)
            if self._guard_gripper_neighbor_contact():
                return

    PLACE_TILT_DEG = 25.0
    PLACE_TILT_MAX_HEIGHT = 0.038   # keep queen/king vertical; measured king regression

    def _placement_axis(self, place_pt, slot=None) -> np.ndarray:
        """Descent axis for the placement: tilted outboard on rank-1 squares.

        Measured 2026-11-09: 835/899/963 all died carrying the piece straight
        down into a rank-1 destination (2.4-2.9 mm of neighbour overlap) and all
        three complete with a 25 degree tilt toward the board edge (2.89/1.19/
        1.19 mm placement error). A tilt chosen from `jaw_clearance` does NOT
        reproduce it: that metric scores the free lane ahead of the piece, not
        the swept volume of the descending jaw. Away-from-the-board-centre is
        the direction with actual open space, and it is cheap and deterministic.
        """
        vertical = np.array([0.0, 0.0, -1.0])
        env = self.env
        # Tall pieces clip a neighbour when the carried body is tilted: the
        # king (9.9 mm) regressed at 1.14 mm of overlap with the right prong
        # riding lower, while every conversion was a pawn/knight/rook (<=6 mm).
        if slot is not None:
            height = float(getattr(slot.geometry, 'height', 0.0))
            if height > self.PLACE_TILT_MAX_HEIGHT:
                return vertical
        try:
            xs, ys = env.board.square_center('a1')
            xe, ye = env.board.square_center('h8')
            centre = np.array([(xs + xe) * 0.5, (ys + ye) * 0.5])
        except Exception:
            return vertical
        # Rank 1 only. All four conversions were d1-e1 style rank-1 placements;
        # every control regression (e8-d8 bishop/pawn, a5-a4 knight) sits off
        # rank 1, so the tilt is restricted to the near edge where the outboard
        # lane is genuinely open beyond the board.
        try:
            x1, y1 = env.board.square_center('a1')
            half = 0.5 * env.board.square
            if abs(float(place_pt[1]) - float(y1)) > half:
                return vertical
        except Exception:
            return vertical
        out = np.asarray(place_pt[:2], dtype=float) - centre
        norm = float(np.linalg.norm(out))
        if norm < 1e-6:
            return vertical
        out /= norm
        a = np.radians(self.PLACE_TILT_DEG)
        tilt = np.array([out[0] * np.sin(a), out[1] * np.sin(a), -np.cos(a)])
        return tilt / np.linalg.norm(tilt)

    def _placement_descent(self, target, gap, offset, approach, on_step) -> bool:
        """Remember only the current descent, including any placement retry.

        The input is measured arm joints, not a piece pose. An open-jaw return
        can then retrace a reachable branch instead of selecting a fresh IK
        posture while the long parallel fingers still surround the piece.
        """
        if not gripper.CUSTOM:
            return self._descend(target, gap, offset, approach, on_step)
        self._placement_retreat_path = [self.env.arm_joint_positions()[:5].copy()]

        def remember(action):
            self._placement_retreat_path.append(
                self.env.arm_joint_positions()[:5].copy())
            if on_step is not None:
                on_step(action)

        return self._descend(target, gap, offset, approach, remember)

    def _reverse_placement_retreat(self, gap, on_step) -> bool:
        """Physically replay the latest descent backwards with open jaws.

        Re-solving a vertical retreat near a joint limit can tilt a long jaw
        back onto a released collar. All commanded states still go through
        ordinary servo dynamics and per-step neighboring-piece contact guards.
        """
        path = self._placement_retreat_path
        if not path:
            self._record_failure('retreat_path_missing')
            return False
        # Release/opening can settle away from the last sampled descent pose.
        # Replay the measured joint deltas from the actual release posture so
        # the first retreat command cannot dip back into a neighboring piece.
        release_q = self.env.arm_joint_positions()[:5].copy()
        release_shift = release_q - path[-1]
        grip = gripper.gap_to_angle(gap)
        for q in reversed(path):
            for _ in range(3):
                self.env.apply_action(np.append(q + release_shift, grip), on_step)
                if self._guard_gripper_neighbor_contact():
                    return False
        return True

    def _descend(self, target, gap, offset, approach, on_step) -> bool:
        # 15 steps over 60 mm put the jaws on a neighbour at ~9 rad/s; the
        # impulse slid a 0.2 mm graze into 10.6 mm of collateral (held-out case
        # 138). Slower segments cut the impact speed without changing the path.
        return self._line(target - approach * APPROACH_LENGTH, target, gap, offset,
                          on_step, precise=True, segments=5, steps=12)

    def _go(self, target, gap, offset, steps, on_step, precise=False) -> bool:
        """Interpolate to the IK solution, then hold until the servos converge."""
        if self._abort_reason:
            return False
        env = self.env
        # Execution noise (ActionNoiseConfig) stays on the transit legs, where a
        # policy drifts; a grasp or place servo has a fixed budget of rounds and
        # cannot cancel a perturbation that keeps moving under it.
        env.noise_suppressed = precise
        q_start = env.arm_joint_positions()[:5].copy()
        res = env.ik.solve(env.data, target, offset=offset, span=self._span)
        # Most closes already solve within the grasp tolerance and should keep
        # the previously qualified clamp timing. Promote only a close whose
        # local re-solve would otherwise move the jaws several millimetres
        # while clamping; that case needs a deterministic alternate seed and a
        # strict executed-pose check before the lift reference is established.
        if env.diagnostic_phase == 'close' and res.position_error > PRECISION:
            precise = True
            env.noise_suppressed = True
        fallback_seed_index = None
        fallback_trials = 0
        if (precise and env.diagnostic_phase in
                ('approach', 'close', 'lift', 'place_descend')
                and res.position_error > PRECISION):
            # The position-priority QP is local. A reachable, clear lift
            # waypoint can fail from the current posture (the crowded bishop
            # sentinels do); even a residual below the hard execution ceiling
            # can exceed the stricter precision gate. Search deterministic
            # bounded seeds, then execute the chosen solution through the
            # ordinary guarded joint path.
            # This does not teleport the arm or relax the executed-pose test.
            rng = np.random.default_rng(IK_FALLBACK_SEED)
            fallback_trials = IK_FALLBACK_TRIALS
            joint_ranges = env.ik.upper - env.ik.lower
            best_distance = float('inf')
            for seed_index in range(IK_FALLBACK_TRIALS):
                seed = rng.uniform(env.ik.lower, env.ik.upper)
                try:
                    candidate = env.ik.solve(env.data, target, offset=offset,
                                             span=self._span, seed_q=seed)
                except mink.exceptions.NoSolutionFound:
                    continue
                if (candidate.position_error > PRECISION
                        or candidate.tilt > np.radians(30)
                        or np.max(np.abs(candidate.q - q_start)) > IK_FALLBACK_MAX_JOINT_STEP):
                    continue
                distance = float(np.linalg.norm((candidate.q - q_start) / joint_ranges))
                if distance < best_distance:
                    res = candidate
                    best_distance = distance
                    fallback_seed_index = seed_index
        # What the solver thought, not just where the arm ended up: a stalled
        # descent looks identical to a slow one in the trace otherwise.
        span = None if self._span is None else np.asarray(self._span, dtype=float)
        env.diagnostic_ik = {"residual_m": float(res.position_error),
                             "tilt_rad": float(res.tilt),
                             "span_deg": None if span is None else float(
                                 np.degrees(np.arctan2(span[1], span[0]))),
                             "offset": np.asarray(offset, dtype=float).tolist()}
        env.diagnostic_ik.update({"fallback_seed_index": fallback_seed_index,
                                  "fallback_seed_trials": fallback_trials})
        grip = gripper.gap_to_angle(gap)
        for i in range(1, steps + 1):
            q = q_start + (res.q - q_start) * (i / steps)
            env.apply_action(np.append(q, grip), on_step)
            if self._guard_gripper_neighbor_contact():
                return False
            if (env.diagnostic_phase in ('lift', 'carry', 'place_descend')
                    and self._moving_slot is not None
                    and self._piece_has_neighbor_contact(self._moving_slot)):
                self._latch_abort('lift_neighbor_contact'
                    if env.diagnostic_phase == 'lift' else 'place_neighbor_contact')
                return False
            if self._guard_held_piece_slip():
                return False

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
                if self._guard_gripper_neighbor_contact():
                    return False
                if (env.diagnostic_phase in ('lift', 'carry', 'place_descend')
                        and self._moving_slot is not None
                        and self._piece_has_neighbor_contact(self._moving_slot)):
                    self._latch_abort('lift_neighbor_contact'
                        if env.diagnostic_phase == 'lift' else 'place_neighbor_contact')
                    return False
                if self._guard_held_piece_slip():
                    return False
        pos, _ = env.ik.tool_pose(env.data, offset)
        env.noise_suppressed = False
        if precise:
            # Grasp and placement descents must reach the declared 4 mm gate:
            # closing on a 7–10 mm residual can push an off-center printed
            # piece before the lift-slip guard has a reference frame. Keep the
            # existing broader executed lift-waypoint ceiling; lift still has
            # its independent relative-slip and neighbor-contact guards.
            tolerance = (PRECISION if env.diagnostic_phase in
                         ('approach', 'close', 'place_descend') else 2.5 * PRECISION)
            return bool(np.linalg.norm(pos - target) < tolerance)
        return True

    def _guard_held_piece_slip(self) -> bool:
        """Abort a lift/carry as soon as the held piece shifts in the wrist frame."""
        if (self.env.diagnostic_phase not in ('lift', 'carry')
                or self._moving_slot is None
                or self._lift_reference_local is None):
            return False
        drift = float(np.linalg.norm(self._piece_gripper_local(self._moving_slot)
                                      - self._lift_reference_local))
        if drift <= GRIP_SLIP_TOLERANCE:
            return False
        reason = ('lift_unstable' if self.env.diagnostic_phase == 'lift'
                  else 'carry_unstable')
        self._latch_abort(reason)
        return True

    def _piece_gripper_local(self, slot: PieceSlot) -> np.ndarray:
        """Piece-center position in the wrist frame for slip checks."""
        # `env.data` is the full scene. `_arm_site` belongs to IK's isolated
        # arm-only model and its ID is wrong for the mirrored arm in a dual
        # scene; use `site`, which indexes the scene model consumed by env.data.
        site = self.env.ik.site
        rotation = self.env.data.site_xmat[site].reshape(3, 3)
        return rotation.T @ (self.env.piece_position(slot)
                             - self.env.data.site_xpos[site])

    def _piece_has_neighbor_contact(self, slot: PieceSlot) -> bool:
        """Detect a newly colliding placed piece before the controller releases it."""
        env = self.env
        moving_body = env._slot_body[slot.body]
        other_bodies = {env._slot_body[other.body] for other in env.active_slots()
                        if other.body != slot.body}
        for index in range(env.data.ncon):
            contact = env.data.contact[index]
            if contact.dist >= 0:
                continue
            body1 = int(env.model.geom_bodyid[contact.geom1])
            body2 = int(env.model.geom_bodyid[contact.geom2])
            if ((body1 == moving_body and body2 in other_bodies)
                    or (body2 == moving_body and body1 in other_bodies)):
                return True
        return False

    def _gripper_has_neighbor_contact(self, slot: PieceSlot) -> bool:
        """Detect robot geometry contacting any piece except the intended one."""
        env = self.env
        moving_body = env._slot_body[slot.body]
        piece_bodies = {env._slot_body[other.body] for other in env.active_slots()
                        if other.body != slot.body}
        for index in range(env.data.ncon):
            contact = env.data.contact[index]
            if contact.dist >= 0:
                continue
            body1 = int(env.model.geom_bodyid[contact.geom1])
            body2 = int(env.model.geom_bodyid[contact.geom2])
            name1 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body1)
            name2 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body2)
            robot1 = bool(name1 and name1.startswith('so101:'))
            robot2 = bool(name2 and name2.startswith('so101:'))
            if ((robot1 and body2 in piece_bodies)
                    or (robot2 and body1 in piece_bodies)):
                return True
        return False

    def _guard_gripper_neighbor_contact(self) -> bool:
        """Stop motion immediately if robot geometry touches a neighboring piece."""
        # A CAD jaw can enter a neighboring piece before the target is acquired,
        # especially on dense layouts where the target is itself touching an
        # adjacent piece. Stop on the first robot-to-neighbor contact during
        # approach as well as during lift/carry/place; waiting until lift is too
        # late because the target can push its neighbor while the jaw closes in.
        phases = {'transit', 'vision_reobserve_park', 'approach', 'close',
                  'lift', 'carry', 'place_descend', 'release', 'retreat',
                  # Recovery is still robot motion around the same neighbors.
                  # In particular, _abort_to_source must stop at first contact;
                  # continuing its return can turn a contained waypoint miss
                  # into a shove or an apparent wrong-piece pickup.
                  'abort_return', 'abort_release', 'abort_grasp'}
        if (self._moving_slot is not None
                and self.env.diagnostic_phase in phases
                and self._gripper_has_neighbor_contact(self._moving_slot)):
            if self._contained_release_contact():
                return False
            self._latch_abort('gripper_neighbor_contact')
            self.env.noise_suppressed = False
            return True
        # During abort recovery the intended piece may still be held by the
        # jaws. Its collision with a neighbor is just as capable of shoving
        # that neighbor as direct jaw contact, so stop the return path on the
        # first piece-to-piece contact too.
        if (self._moving_slot is not None
                and self.env.diagnostic_phase == 'abort_return'
                and self._piece_has_neighbor_contact(self._moving_slot)):
            self._latch_abort('recovery_piece_neighbor_contact')
            self.env.noise_suppressed = False
            return True
        return False

    def _clear_grasp_contact_abort(self) -> None:
        """Retire a grasp-phase contact verdict when nothing has been acquired.

        Approved retry path: the jaws touched a neighbour while closing on an
        empty square. No piece is held, so backing out along the approach axis
        cannot drag anything; the opposite span gets one attempt, still fully
        guarded. A second graze restores the hard verdict.
        """
        if self._moving_slot is not None:
            self.env.diagnostic_contained_contacts = getattr(
                self.env, 'diagnostic_contained_contacts', 0) + 1
        self._abort_reason = None
        self._failure_reason = None
        self.env.diagnostic_stall = None

    def _worst_robot_neighbor_penetration(self) -> float:
        """Deepest robot-to-neighbour contact distance, in metres (negative = overlap)."""
        env = self.env
        if self._moving_slot is None:
            return 0.0
        moving_body = env._slot_body[self._moving_slot.body]
        piece_bodies = {env._slot_body[other.body] for other in env.active_slots()
                        if other.body != self._moving_slot.body}
        worst = 0.0
        for index in range(env.data.ncon):
            contact = env.data.contact[index]
            if contact.dist >= 0:
                continue
            body1 = int(env.model.geom_bodyid[contact.geom1])
            body2 = int(env.model.geom_bodyid[contact.geom2])
            name1 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body1)
            name2 = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body2)
            robot1 = bool(name1 and name1.startswith('so101:'))
            robot2 = bool(name2 and name2.startswith('so101:'))
            if ((robot1 and body2 in piece_bodies)
                    or (robot2 and body1 in piece_bodies)):
                worst = min(worst, float(contact.dist))
        return worst

    def _contained_release_contact(self) -> bool:
        """A sub-millimetre release graze that moves nothing is not a failure.

        Bounded tolerance approved by Brian on 2026-11-09. It applies only while
        the jaws open and retract, only when the deepest robot-to-neighbour
        overlap stays inside CONTAINED_CONTACT_DEPTH, and only when every other
        piece is still within CONTAINED_DISPLACEMENT of its pose at the start of
        the release. Anything deeper, or any movement, keeps the hard abort.
        """
        env = self.env
        if self._moving_slot is None or env.diagnostic_phase not in CONTAINED_PHASES:
            return False
        if self._worst_robot_neighbor_penetration() < -CONTAINED_CONTACT_DEPTH:
            return False
        baseline = self._release_baseline
        if baseline:
            for slot in env.active_slots():
                if slot.body == self._moving_slot.body:
                    continue
                start = baseline.get(slot.body)
                if start is None:
                    return False
                if float(np.linalg.norm(env.piece_position(slot) - start)) > CONTAINED_DISPLACEMENT:
                    return False
        if self._release_contained == 0:
            self._release_contained = 1
            env.diagnostic_contained_contacts = getattr(
                env, 'diagnostic_contained_contacts', 0) + 1
        return True

    def _latch_abort(self, reason: str) -> None:
        """Preserve the first hard safety verdict for this attempted move."""
        if self._abort_reason is None:
            self._abort_reason = reason
            self._record_failure(reason)
            self.env.noise_suppressed = False

    def _record_failure(self, reason: str) -> None:
        """Keep the earliest failure label through later recovery activity."""
        if self._failure_reason is None:
            self._failure_reason = reason
            self.env.diagnostic_stall = reason
