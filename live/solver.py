"""Solver selection for ``/move``: the least_squares path or the mink QP stack.

Phase 1 of the solver unification, behind a flag that defaults to the incumbent.
Nothing about the console's behaviour changes when the flag is off; when it is
on, *only* the solve is replaced.  Every other job ``/move`` does — board-frame
bookkeeping, mode expansion, span/roll-lock, keep-in, joint-limit and window
guards, dry-run, streaming — stays in ``live/kinematics.py`` and ``live/app.py``
exactly as it is.

Why an adapter and not a rewrite
--------------------------------
``ArmKinematics`` owns the board frame, the tip convention and the guard
helpers.  Duplicating those in the mink path would create the second model of
the arm that Phase 1 exists to delete.  So this class *delegates* to the chain
for everything except the numerical solve, and the two solvers are required to
return the same result shape:

    {"joints_deg": {name: deg, ...}, "position_error_mm": float,
     "approach_error_deg": float, "cost": float, "starts": int}

When ``use_mink`` is on, the chain-frame goal the caller built is passed through
``TIP_IN_CHAIN`` first: the mink model solves for the *tip*, while
``ArmKinematics.solve`` is handed the *chain* goal and applies the transform
itself.  That single line is the whole frame translation between the two.

The escalation policy is deliberately the same one ``solve_tip`` already has —
fast path from the current posture, then pan seeds and quarter-turn spans,
replacing the incumbent only when an attempt is clearly better.  Making the mink
path's search *different* would confound "which solver" with "which search".
"""
from __future__ import annotations

import math

import numpy as np

#: When an escalated attempt must beat the incumbent by this much to replace it.
IMPROVEMENT_MM = 5.0


class MoveSolver:
    """``ArmKinematics``'s solve API, backed by whichever solver is configured."""

    def __init__(self, kinematics, calibration=None, tip_in_chain=None,
                 use_mink: bool = False, mink=None) -> None:
        self.kinematics = kinematics
        self.tip_in_chain = (np.asarray(tip_in_chain, dtype=float)
                             if tip_in_chain is not None else None)
        self._mink = mink
        self._calibration = calibration
        self.use_mink = False
        if use_mink:
            self.enable_mink()

    # -- lifecycle ---------------------------------------------------------
    def enable_mink(self) -> None:
        """Build the chain-derived mink model and start using it.

        Raises if the model cannot be built: a flag that silently falls back is
        a flag nobody can trust, so a failure here is loud and leaves the
        incumbent in place only if the caller catches it.
        """
        if self._mink is None:
            from mink_arm import MinkArm
            if self._calibration is None:
                raise RuntimeError("mink solver needs the wrist calibration")
            self._mink = MinkArm(self._calibration, self.tip_in_chain,
                                 limits=self.kinematics.limits or None)
        self.use_mink = True

    def sync_limits(self, limits: dict) -> None:
        """Follow the bridge's measured limits, exactly as the chain does."""
        if not self.use_mink or self._mink is None or not limits:
            return
        model = self._mink.model
        for index, name in enumerate(("shoulder_pan", "shoulder_lift",
                                      "elbow_flex", "wrist_flex", "wrist_roll")):
            lo, hi = limits.get(name, (None, None))
            if lo is None:
                continue
            joint = model.joint("so101:" + name)
            model.jnt_range[joint.id] = [float(math.radians(lo)),
                                         float(math.radians(hi))]
            self._mink.ik.lower[index] = float(math.radians(lo))
            self._mink.ik.upper[index] = float(math.radians(hi))

    # -- the two entry points /move uses -----------------------------------
    def solve(self, target_tool, seed_deg, span_deg=None, span_weight=0.12,
              max_nfev=400, freeze_roll=False, tool_frame=None):
        if not self.use_mink:
            return self.kinematics.solve(
                target_tool, seed_deg, span_deg=span_deg, span_weight=span_weight,
                max_nfev=max_nfev, freeze_roll=freeze_roll, tool_frame=tool_frame)
        goal = self._tip_goal(target_tool, tool_frame)
        return self._mink.solve(goal, seed_deg, span_deg=span_deg,
                                freeze_roll=freeze_roll)

    def solve_multi(self, target_tool, seed_deg, span_deg=None,
                    freeze_roll=False, tool_frame=None):
        if not self.use_mink:
            return self.kinematics.solve_multi(
                target_tool, seed_deg, span_deg=span_deg, freeze_roll=freeze_roll,
                tool_frame=tool_frame)
        goal = self._tip_goal(target_tool, tool_frame)
        return self._solve_multi_goal(goal, seed_deg, span_deg, freeze_roll)

    def solve_tip(self, tip_position_m, seed_deg, jaw_frame, span_deg=0.0):
        if not self.use_mink:
            return self.kinematics.solve_tip(tip_position_m, seed_deg, jaw_frame,
                                             span_deg)
        return self._solve_tip_mink(tip_position_m, seed_deg, jaw_frame, span_deg)

    # -- internals ---------------------------------------------------------
    def _tip_goal(self, target_tool, tool_frame):
        """The chain goal plus ``TIP_IN_CHAIN`` is the tip pose the mink model solves."""
        goal = np.asarray(target_tool, dtype=float)
        frame = (np.asarray(tool_frame, dtype=float) if tool_frame is not None
                 else np.eye(4))
        return goal @ frame

    def _solve_multi_goal(self, tip_goal, seed_deg, span_deg, freeze_roll):
        """Same roll multi-start as ``ArmKinematics.solve_multi``, on mink."""
        seed = np.asarray(seed_deg, dtype=float)
        if seed.size < 6:
            seed = np.concatenate([seed, np.zeros(6 - seed.size)])
        roll_lo, roll_hi = self.kinematics.limits.get("wrist_roll", (-180.0, 180.0))
        best = None
        starts = 0
        for delta in (0.0, 180.0, 90.0, -90.0):
            attempt = seed.copy()
            attempt[4] = (seed[4] if freeze_roll else
                          float(np.clip(seed[4] + delta, roll_lo + 2.0, roll_hi - 2.0)))
            result = self._mink.solve(tip_goal, attempt, span_deg=span_deg,
                                      freeze_roll=freeze_roll)
            starts += 1
            if best is None or result["cost"] < best["cost"]:
                best = dict(result, starts=starts)
            if (result["position_error_mm"] < 4.0
                    and result["approach_error_deg"] < 3.0):
                request = result
                request["starts"] = starts
                return request
        best["starts"] = starts
        return best

    def _solve_tip_mink(self, tip_position_m, seed_deg, jaw_frame, span_deg):
        """``ArmKinematics.solve_tip``'s search, run on the mink solver.

        The goal construction and the seed list are copied from the chain's own
        ``solve_tip`` on purpose: the point of Phase 1 is to change the solver,
        not the search, so the two columns in the 64-square receipt are
        comparable.  Returns ``(result, chain_goal)`` like the original.
        """
        from kinematics import _orthonormal, transform

        seed = np.asarray(seed_deg, dtype=float)
        if seed.size < 6:
            seed = np.concatenate([seed, np.zeros(6 - seed.size)])
        target = np.asarray(tip_position_m, dtype=float)[:3]
        up = self.kinematics.board_axis("z")
        x_axis, y_axis = self.kinematics.board_axis("x"), self.kinematics.board_axis("y")
        inverse = np.linalg.inv(np.asarray(jaw_frame, dtype=float))

        def goal_for(angle_deg):
            angle = math.radians(angle_deg)
            hint = math.cos(angle) * x_axis + math.sin(angle) * y_axis
            # Chain goal (what the caller gets back) and tip goal (what mink
            # solves) differ only by the tip transform.
            chain_goal = transform(target, _orthonormal(up, hint)) @ inverse
            return chain_goal, chain_goal @ jaw_frame

        seeds = [list(seed)]
        for pan in (0.0, -45.0, 45.0, -80.0, 80.0):
            seeds.append([pan, -55.0, 72.0, 39.0, 0.0, seed[5]])
        for pan in (0.0, -20.0, 20.0, -45.0, 45.0):
            seeds.append([pan, 58.0, -64.0, 90.0, 0.0, seed[5]])

        chain_goal, tip_goal = goal_for(span_deg)
        probe = self._solve_multi_goal(tip_goal, seeds[0], None, False)
        best = (probe["position_error_mm"] + 15.0 * probe["approach_error_deg"],
                probe, chain_goal)
        if best[1]["position_error_mm"] > 15.0 or best[1]["approach_error_deg"] > 5.0:
            for seed_try in seeds:
                for extra in (0.0, 90.0, 180.0, -90.0):
                    chain_try, tip_try = goal_for(span_deg + extra)
                    attempt = self._solve_multi_goal(tip_try, seed_try, None, False)
                    score = (attempt["position_error_mm"]
                             + 15.0 * attempt["approach_error_deg"])
                    if score < best[0] - IMPROVEMENT_MM:
                        best = (score, attempt, chain_try)
        return best[1], best[2]
