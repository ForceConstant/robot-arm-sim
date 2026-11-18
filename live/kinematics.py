"""Board-frame kinematics for the live arm: forward chain, IK, look-at targeting.

Everything the dashboard commands is expressed in a *board frame* whose origin
is the centre of the mat, +x along the files (a -> h), +y along the ranks
(1 -> 8) and +z out of the board surface.  The loose centre target (ArUco id 4)
sits exactly at that origin, which is what makes it a useful reference point.

Three input modes funnel into one numerical solve:

``xyz``      place the tool point at a board coordinate with the jaws pointing
             straight down at the board -- the perpendicular approach that
             pick and place need.
``look_at``  aim the *camera* at a board point from a standoff distance with a
             chosen tilt.  Position and two orientation axes are pinned; the
             leftover roll is spent on gripper alignment.  This is the mode
             calibration captures want, because framing stops being guesswork.
``joint``    absolute joint angles, the escape hatch for poses IK cannot reach.

The chain and the camera mount come from the same calibration file the scan
uses, so FK here agrees with the numbers the hand-eye solve produced.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
ARM_JOINTS = JOINTS[:5]
SQUARE_MM = 28.0
FILES = "abcdefgh"
# Rotational error is folded into the position residual with this length, so a
# 1 radian error costs about 250 mm and the two terms stay comparable.
ROT_SCALE_M = 0.25
# Metres of residual per degree of joint travel away from the seed.  Enough to
# pin redundant wrist roll, loose enough not to block real motion.
CONTINUITY = (4e-4, 4e-4, 4e-4, 4e-4, 6e-3)
# The camera sits ~100 mm off the tool axis, so wrist roll moves it: at a
# light weight the solver rolls 30 deg to win a few millimetres, which
# reframes the view and twists the jaws.  Roll is priced high on purpose.


def transform(position, rotation) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, dtype=float)
    matrix[:3, 3] = np.asarray(position, dtype=float)
    return matrix


def frame_at(x_mm: float, y_mm: float, z_mm: float, yaw_deg: float) -> np.ndarray:
    return transform([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0],
                     Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix())


def square_xy(name: str) -> tuple[float, float]:
    """Board-frame millimetres for a square centre, e.g. ``e4``."""
    name = name.strip().lower()
    if len(name) != 2 or name[0] not in FILES or name[1] not in "12345678":
        raise ValueError(f"not a board square: {name!r}")
    half = SQUARE_MM * 4.0
    return ((FILES.index(name[0]) + 0.5) * SQUARE_MM - half,
            (int(name[1]) - 0.5) * SQUARE_MM - half)


def _rotation_error(current, target) -> np.ndarray:
    """Rotation vector taking ``current`` onto ``target``, valid at 180 degrees.

    ``cross(current, target)`` is the obvious error term and it silently
    vanishes when the two are antiparallel, which makes an exactly upside-down
    approach score as a perfect fit.  This keeps the magnitude honest.
    """
    a = np.asarray(current, dtype=float)
    b = np.asarray(target, dtype=float)
    a = a / max(1e-9, np.linalg.norm(a))
    b = b / max(1e-9, np.linalg.norm(b))
    cross = np.cross(a, b)
    sin_angle = float(np.linalg.norm(cross))
    cos_angle = float(np.clip(a @ b, -1.0, 1.0))
    angle = math.atan2(sin_angle, cos_angle)
    if sin_angle > 1e-8:
        return cross / sin_angle * angle
    if cos_angle > 0.0:
        return np.zeros(3)
    # Antiparallel: any axis perpendicular to `a` is a valid half-turn.
    axis = np.cross(a, [1.0, 0.0, 0.0])
    if np.linalg.norm(axis) < 1e-6:
        axis = np.cross(a, [0.0, 1.0, 0.0])
    return axis / np.linalg.norm(axis) * math.pi


def _orthonormal(z_axis: np.ndarray, x_hint: np.ndarray) -> np.ndarray:
    """Rotation whose third column is ``z_axis``, first column closest to ``x_hint``."""
    z = np.asarray(z_axis, dtype=float)
    z = z / max(1e-9, np.linalg.norm(z))
    x = np.asarray(x_hint, dtype=float)
    x = x - z * float(x @ z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0.0, 1.0, 0.0])
        if np.linalg.norm(x) < 1e-6:
            x = np.cross(z, [1.0, 0.0, 0.0])
    x = x / max(1e-9, np.linalg.norm(x))
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class ArmKinematics:
    """FK/IK over the calibrated SO-101 chain, plus board-frame conversions."""

    def __init__(self, calibration, limits: dict | None = None, board=None) -> None:
        self.calibration = calibration
        self.camera_in_tool = np.asarray(calibration.camera_in_tool, dtype=float)
        self.limits = {name: (float(lo), float(hi))
                       for name, (lo, hi) in (limits or {}).items()}
        self.board = np.asarray(board if board is not None else np.eye(4), dtype=float)

    # ------------------------------------------------------------ board frame
    def set_board(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float) -> None:
        self.board = frame_at(x_mm, y_mm, z_mm, yaw_deg)

    def board_mm(self) -> dict:
        return {"x_mm": round(float(self.board[0, 3]) * 1000.0, 1),
                "y_mm": round(float(self.board[1, 3]) * 1000.0, 1),
                "z_mm": round(float(self.board[2, 3]) * 1000.0, 1),
                "yaw_deg": round(float(math.degrees(
                    math.atan2(self.board[1, 0], self.board[0, 0]))), 2)}

    def board_point(self, x_mm: float, y_mm: float, z_mm: float = 0.0) -> np.ndarray:
        return (self.board @ np.array([x_mm / 1000.0, y_mm / 1000.0,
                                       z_mm / 1000.0, 1.0]))[:3]

    def to_board(self, point_m) -> np.ndarray:
        return (np.linalg.inv(self.board) @ np.array([*point_m, 1.0]))[:3]

    def board_axis(self, axis: str) -> np.ndarray:
        return self.board[:3, "xyz".index(axis)]

    # -------------------------------------------------------------------- FK
    def fk(self, joints_deg) -> np.ndarray:
        """``T_base_flange`` for measured joints in degrees."""
        angles = np.radians(np.asarray(joints_deg, dtype=float))
        world = np.eye(4)
        for link in self.calibration.chain:
            world = world @ transform(link["position"], link["rotation"])
            if "joint_index" not in link:
                continue
            index = int(link["joint_index"])
            angle = float(angles[index]) if index < angles.size else 0.0
            anchor = np.asarray(link["anchor"], dtype=float)
            rotation = Rotation.from_rotvec(
                np.asarray(link["axis"], dtype=float)
                * (angle - float(link["reference"]))).as_matrix()
            world = world @ transform(anchor - rotation @ anchor, rotation)
        return world

    def camera_pose(self, joints_deg) -> np.ndarray:
        return self.fk(joints_deg) @ self.camera_in_tool

    def tool_from_camera(self, camera_pose: np.ndarray) -> np.ndarray:
        return camera_pose @ np.linalg.inv(self.camera_in_tool)

    def report(self, joints_deg) -> dict:
        """Tool and camera poses in board coordinates, for the dashboard."""
        tool = self.fk(joints_deg)
        camera = tool @ self.camera_in_tool
        board_from_base = self.board[:3, :3].T
        tool_mm = self.to_board(tool[:3, 3]) * 1000.0
        camera_mm = self.to_board(camera[:3, 3]) * 1000.0
        view = board_from_base @ camera[:3, 2]
        view = view / max(1e-9, np.linalg.norm(view))
        # The camera looks along +z, so straight down is view_z == -1 and the
        # tilt away from straight-down is acos(-view_z).
        pitch = math.degrees(math.acos(max(-1.0, min(1.0, -float(view[2])))))
        jaw = board_from_base @ tool[:3, 1]
        return {"tool_mm": [round(float(v), 1) for v in tool_mm],
                "camera_mm": [round(float(v), 1) for v in camera_mm],
                "camera_height_mm": round(float(camera_mm[2]), 1),
                "view_pitch_deg": round(pitch, 1),
                "jaw_azimuth_deg": round(float(math.degrees(
                    math.atan2(float(jaw[1]), float(jaw[0])))), 1)}

    # -------------------------------------------------------------------- IK
    def _bounds(self, margin_deg: float = 1.5):
        lows, highs = [], []
        for name in ARM_JOINTS:
            lo, hi = self.limits.get(name, (-180.0, 180.0))
            lows.append(lo + margin_deg)
            highs.append(hi - margin_deg)
        return np.array(lows), np.array(highs)

    def solve(self, target_tool: np.ndarray, seed_deg, span_deg: float | None = None,
              span_weight: float = 0.12, max_nfev: int = 400,
              freeze_roll: bool = False, tool_frame=None) -> dict:
        """Joint angles placing the commanded point at ``target_tool``.

        ``tool_frame`` is the chain->tip transform (None means the chain frame
        itself).  When given, the residual and the reported error are measured
        at the jaw tip, so the number the solver reports is the number the guard
        checks -- a chain-frame error would hide the tip's offset whenever the
        in-plane rotation is free.

        Position (3) and approach direction (2) are the hard constraints.  The
        in-plane rotation is a weak objective, because a five-joint arm cannot
        hold position, approach *and* gripper azimuth independently: position
        wins and the residual reports what the alignment cost.
        """
        seed = np.asarray(seed_deg, dtype=float)
        if seed.size < 6:
            seed = np.concatenate([seed, np.zeros(6 - seed.size)])
        target = (target_tool if tool_frame is None
                  else target_tool @ np.asarray(tool_frame, dtype=float))
        target_position = target[:3, 3]
        target_z = target[:3, 2] / max(1e-9, np.linalg.norm(target[:3, 2]))
        if span_deg is not None:
            span_target = target[:3, :3] @ np.array(
                [math.cos(math.radians(span_deg)), math.sin(math.radians(span_deg)), 0.0])
            span_target = span_target / max(1e-9, np.linalg.norm(span_target))
            weight = float(span_weight)
        else:
            span_target, weight = np.zeros(3), 0.0

        def expand(q):
            """Full five arm joints from the free parameters."""
            if not freeze_roll:
                return q
            return np.concatenate([q[:4], [seed[4]]])

        def residual(q):
            free = expand(q)
            pull = np.asarray(CONTINUITY[:free.size], dtype=float) * (free - seed[:free.size])
            tool = self.fk(np.concatenate([free, seed[5:6]]))
            frame = (tool if tool_frame is None
                     else tool @ np.asarray(tool_frame, dtype=float))
            z_axis = frame[:3, 2] / max(1e-9, np.linalg.norm(frame[:3, 2]))
            parts = [frame[:3, 3] - target_position,
                     _rotation_error(z_axis, target_z) * ROT_SCALE_M,
                     pull]
            if weight > 0.0:
                x_axis = frame[:3, 0] - z_axis * float(frame[:3, 0] @ z_axis)
                norm = np.linalg.norm(x_axis)
                if norm > 1e-9:
                    parts.append(np.array([weight * float(
                        np.cross(x_axis / norm, span_target) @ target_z)]))
            return np.concatenate(parts)

        lows, highs = self._bounds(1.6)
        if freeze_roll:
            # Four unknowns for five constraints: the solver finds the best
            # position it can reach with the jaws held still, and reports the
            # shortfall instead of quietly twisting the wrist to hide it.
            keep = [0, 1, 2, 3]
            lows, highs = lows[keep], highs[keep]
            start = np.clip(seed[:5][keep], lows + 0.1, highs - 0.1)
        else:
            start = np.clip(seed[:5], lows + 0.1, highs - 0.1)
        free_lows, free_highs = lows, highs
        result = least_squares(residual, start, bounds=(free_lows, free_highs),
                               x_scale=10.0, max_nfev=max_nfev,
                               xtol=1e-10, ftol=1e-10)
        joints = np.concatenate([expand(result.x), seed[5:6]])
        tool = self.fk(joints)
        frame = (tool if tool_frame is None
                 else tool @ np.asarray(tool_frame, dtype=float))
        position_error_mm = float(np.linalg.norm(
            self.to_board(frame[:3, 3]) * 1000.0
            - self.to_board(target_position) * 1000.0))
        z_axis = frame[:3, 2] / max(1e-9, np.linalg.norm(frame[:3, 2]))
        approach_error_deg = math.degrees(math.acos(
            max(-1.0, min(1.0, float(z_axis @ target_z)))))
        return {"joints_deg": {name: round(float(v), 3)
                               for name, v in zip(JOINTS, joints)},
                "position_error_mm": round(position_error_mm, 1),
                "approach_error_deg": round(approach_error_deg, 2),
                "cost": float(np.linalg.norm(residual(result.x))),
                "starts": 1}

    def solve_multi(self, target_tool: np.ndarray, seed_deg, span_deg=None,
                    freeze_roll: bool = False, tool_frame=None) -> dict:
        """Retry from the seed and from quarter-turns of the wrist roll.

        Roll has about 320 degrees of travel, so a target that is a saddle from
        the current roll is often reachable from the opposite side.  The sim
        solver multi-starts for the same reason.
        """
        seed = np.asarray(seed_deg, dtype=float)
        if seed.size < 6:
            seed = np.concatenate([seed, np.zeros(6 - seed.size)])
        roll_lo, roll_hi = self.limits.get("wrist_roll", (-180.0, 180.0))
        best = None
        starts = 0
        for delta in (0.0, 180.0, 90.0, -90.0):
            attempt = seed.copy()
            # With the roll frozen there is nothing to multi-start: perturbing
            # it would just freeze a different branch each attempt.
            attempt[4] = (seed[4] if freeze_roll else
                          float(np.clip(seed[4] + delta, roll_lo + 2.0, roll_hi - 2.0)))
            result = self.solve(target_tool, attempt, span_deg=span_deg,
                                freeze_roll=freeze_roll, tool_frame=tool_frame)
            starts += 1
            if best is None or result["cost"] < best["cost"]:
                best = dict(result, starts=starts)
            # Prefer the branch closest to where the arm already is: a lower
            # cost on the far side of a roll singularity means swinging the
            # wrist a half turn, which reframes the camera for no benefit.
            if (result["position_error_mm"] < 4.0
                    and result["approach_error_deg"] < 3.0):
                result["starts"] = starts
                return result
        # A folded arm is a local minimum for a downward approach: the solver
        # holds position exactly while leaving the tool pointing the wrong way.
        # Sweep a coarse grid of plausible poses and keep the best landing.
        for pan in (-40.0, 0.0, 40.0):
            for lift in (-60.0, -20.0, 20.0):
                for elbow in (20.0, 55.0, 90.0):
                    for flex in (-50.0, -10.0, 30.0):
                        attempt = np.array([pan, lift, elbow, flex, seed[4], seed[5]])
                        result = self.solve(target_tool, attempt, span_deg=span_deg,
                                           max_nfev=200, freeze_roll=freeze_roll,
                                           tool_frame=tool_frame)
                        starts += 1
                        if result["cost"] < best["cost"]:
                            best = dict(result, starts=starts)
                        if (best["position_error_mm"] < 1.5
                                and best["approach_error_deg"] < 1.0):
                            best["starts"] = starts
                            return best
        best["starts"] = starts
        return best

    def violations(self, joints_deg) -> list[tuple[str, float, tuple[float, float]]]:
        """Joints outside their measured limits, so callers can refuse a move."""
        bad = []
        for index, name in enumerate(ARM_JOINTS):
            if index >= len(joints_deg):
                break
            lo, hi = self.limits.get(name, (-180.0, 180.0))
            value = float(joints_deg[index])
            if value < lo - 1e-6 or value > hi + 1e-6:
                bad.append((name, round(value, 2), (lo, hi)))
        return bad

    # ----------------------------------------------------------- target poses
    def xyz_target(self, x_mm: float, y_mm: float, z_mm: float,
                   span_deg: float = 0.0) -> np.ndarray:
        """TIP pose for a board coordinate with the jaws pointing straight down.

        The tip frame's +z points back up the fingers, so the approach (jaws
        pointing at the board) is -z.  This must stay identical to the frame
        built in ``Live.move`` -- an earlier version flipped it to -z, which
        pointed the jaws at the ceiling for every caller of this helper.
        """
        hint = (math.cos(math.radians(span_deg)) * self.board_axis("x")
                + math.sin(math.radians(span_deg)) * self.board_axis("y"))
        return transform(self.board_point(x_mm, y_mm, z_mm),
                         _orthonormal(self.board_axis("z"), hint))

    def solve_tip(self, tip_position_m, seed_deg, jaw_frame,
                  span_deg: float = 0.0) -> tuple[dict, np.ndarray]:
        """Solve a TIP-space target through the one chain->tip transform.

        ``tip_position_m`` is the jaw-tip point in the base frame (use
        ``board_point`` to build it).  The returned joint solution is for the
        chain frame; the second value is the chain-frame goal it was solved
        against, so a caller can re-check the tip without guessing.

        Fast path first (current posture, nominal span).  Only on a miss does
        it escalate over five pan seeds and quarter-turn spans -- running the
        full search every time made a 64-square sweep take half an hour.
        Escalation replaces the fast result only when it beats it by 5 mm, so a
        marginally better score cannot displace a valid basin.
        """
        seed = np.asarray(seed_deg, dtype=float)
        if seed.size < 6:
            seed = np.concatenate([seed, np.zeros(6 - seed.size)])
        target = np.asarray(tip_position_m, dtype=float)[:3]
        up = self.board_axis("z")
        x_axis, y_axis = self.board_axis("x"), self.board_axis("y")
        inverse = np.linalg.inv(np.asarray(jaw_frame, dtype=float))

        def goal_for(angle_deg: float) -> np.ndarray:
            angle = math.radians(angle_deg)
            hint = math.cos(angle) * x_axis + math.sin(angle) * y_axis
            return transform(target, _orthonormal(up, hint)) @ inverse

        seeds = [list(seed)]
        for pan in (0.0, -45.0, 45.0, -80.0, 80.0):
            seeds.append([pan, -55.0, 72.0, 39.0, 0.0, seed[5]])

        goal = goal_for(span_deg)
        probe = self.solve_multi(goal, seeds[0], tool_frame=jaw_frame)
        best = (probe["position_error_mm"]
                + 15.0 * probe["approach_error_deg"], probe, goal)
        if best[1]["position_error_mm"] > 15.0 or best[1]["approach_error_deg"] > 5.0:
            for seed_try in seeds:
                for extra in (0.0, 90.0, 180.0, -90.0):
                    goal_try = goal_for(span_deg + extra)
                    attempt = self.solve_multi(goal_try, seed_try, tool_frame=jaw_frame)
                    score = (attempt["position_error_mm"]
                             + 15.0 * attempt["approach_error_deg"])
                    if score < best[0] - 5.0:
                        best = (score, attempt, goal_try)
        return best[1], best[2]

    def look_at_target(self, x_mm: float, y_mm: float, z_mm: float,
                       distance_mm: float, pitch_deg: float = 0.0,
                       azimuth_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
        """Tool pose that aims the camera at a board point.

        ``pitch`` tilts away from straight-down, ``azimuth`` swings the offset
        around the point, ``roll`` twists the image about the view axis.
        """
        point = self.board_point(x_mm, y_mm, z_mm)
        tilt = (Rotation.from_euler("z", azimuth_deg, degrees=True).as_matrix()
                @ Rotation.from_euler("x", pitch_deg, degrees=True).as_matrix())
        view = tilt @ (self.board_axis("z") * -1.0)
        view = view / max(1e-9, np.linalg.norm(view))
        camera_position = point - view * (distance_mm / 1000.0)
        rotation = _orthonormal(view, tilt @ self.board_axis("y"))
        if roll_deg:
            rotation = rotation @ Rotation.from_euler(
                "z", roll_deg, degrees=True).as_matrix()
        return self.tool_from_camera(transform(camera_position, rotation))
