"""Two physical SO101 models sharing one board, one world, and one simulation clock.

The upstream contact-based expert is reused for each arm. This remains an
oracle-state scripted controller, NOT image-based perception or a learned policy.
"""
from __future__ import annotations

import chess
import mujoco
import numpy as np

from .config import JOINTS, EnvConfig, square_index
from .controller import grasp_plan
from .env import ChessSimEnv, GRASP_HEIGHT
from .ik import So101Ik
from .reach import ReachMap
from .scene import ARM_PREFIX, arm_rest_pose, build_arm
from .tasks import TaskFamily


class DualArmEnv(ChessSimEnv):
    @staticmethod
    def home_pose():
        home = arm_rest_pose().to_array()
        # The single-arm upstream rest reaches into the board centre. Mirroring
        # it makes both grippers intersect. Park each arm off its own right side.
        home[0] = -1.3
        return home

    def __init__(self, config: EnvConfig | None = None):
        config = (config or EnvConfig()).model_copy(update={"dual_arms": True})
        super().__init__(config)
        self._contexts = {"white": {
            "board": self.board, "ik": self.ik, "qpos": self._joint_qpos,
            "dof": self._joint_dof, "actuators": self._actuators, "reach": self.reach}}
        board = self.board.model_copy(update={"arm_side": "black"})
        prefix = "black:"
        ids = [self.model.joint(prefix + n).id for n in JOINTS]
        self._contexts["black"] = {
            "board": board,
            "ik": So101Ik(self.model, build_arm(board, prefix=prefix).compile(), prefix=prefix),
            "qpos": np.array([self.model.jnt_qposadr[j] for j in ids]),
            "dof": np.array([self.model.jnt_dofadr[j] for j in ids]),
            "actuators": [self.model.actuator(prefix + n).id for n in JOINTS],
            "reach": None}
        self.active_side = "white"
        self._capture_count = {"white": 0, "black": 0}
        self.select_arm("black")
        self.reach = ReachMap.compute(self.board, self._tool_query, GRASP_HEIGHT, executed={})
        self._contexts["black"]["reach"] = self.reach
        self.select_arm("white")

    def select_arm(self, side: str):
        if side not in ("white", "black"):
            raise ValueError("Unknown arm")
        c = self._contexts[side]
        self.active_side = side
        self.board, self.ik = c["board"], c["ik"]
        self._joint_qpos, self._joint_dof = c["qpos"], c["dof"]
        self._actuators, self.reach = c["actuators"], c["reach"]
        self.controller._span = None

    def reset(self, position=chess.STARTING_FEN, rng=None):
        self.select_arm("white")
        mujoco.mj_resetData(self.model, self.data)
        self._capture_count = {"white": 0, "black": 0}
        rest = self.home_pose()
        for context in self._contexts.values():
            self.data.qpos[context["qpos"]] = rest
            self.data.ctrl[context["actuators"]] = rest
        super().reset(position, rng=rng)
        for context in self._contexts.values():
            self.data.qpos[context["qpos"]] = rest
            self.data.ctrl[context["actuators"]] = rest
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(20):
            self.apply_action(rest)
        return self.observe()

    def free_tray_slot(self):
        # Separate 4x4 capture areas. The controller still has to reach them;
        # no hidden off-board teleportation on successful captures.
        n = self._capture_count[self.active_side]
        if n >= 16:
            return None
        x = -0.16 - (n % 4) * 0.027
        y = -0.07 + (n // 4) * 0.027
        return (-x, -y) if self.active_side == "black" else (x, y)

    def _commit(self, task, result):
        # Physical occupancy is separate from authoritative game history. In
        # particular a king's physical move must not logically teleport its rook.
        if not result.upright:
            return
        src = square_index(task.source)
        slot = self._square_slot.get(src)
        if slot is None:
            return
        if task.family is TaskFamily.CAPTURE:
            if result.placement_error < self.config.tolerances.tray and not self.board.on_field(
                    self.piece_position(slot)[:2]):
                self._captured.append(self._square_slot.pop(src))
                self.position.remove_piece_at(src)
                self._capture_count[self.active_side] += 1
        elif result.placement_error < self.config.tolerances.placement:
            dst = square_index(task.to_square)
            self._square_slot[dst] = self._square_slot.pop(src)
            self.position.set_piece_at(dst, self.position.remove_piece_at(src))

    def park(self, on_step=None):
        """Physically retreat and park the active arm; the other remains controlled."""
        # Clear pieces with a Cartesian lift before the joint-space return.
        from . import gripper
        current = self._current_action()
        gap = gripper.angle_to_gap(float(current[5]))
        offset = gripper.pocket_offset(gap)
        p, _ = self.ik.tool_pose(self.data, offset)
        high = p.copy()
        high[2] = max(p[2], self.board.top + 0.14)
        self.controller._line(p, high, gap, offset, on_step, segments=4, steps=6)
        start = self.arm_joint_positions().copy()
        rest = self.home_pose()
        for i in range(1, 46):
            self.apply_action(start + (rest - start) * (i / 45), on_step)
        for _ in range(15):
            self.apply_action(rest, on_step)

    def candidate_moves(self, game: chess.Board):
        """Legal moves with reachable endpoints; outcome is still contact-scored.

        This is deliberately not sold as a full collision planner. The inherited
        conservative clear-lane filter can reject most opening moves; the live
        lab attempts them and stops on measured failures instead of hiding them.
        """
        return [m for m in game.legal_moves if not m.promotion
                and m.from_square in self.reach and m.to_square in self.reach]

    def full_board_error(self, game: chess.Board):
        """Do not commit a legal turn if an unrelated piece fell or was nudged."""
        if set(game.piece_map()) != set(self._square_slot):
            return "Physical occupancy differs from the legal position"
        for sq, slot in self._square_slot.items():
            expected = np.array(self.board.square_center(sq))
            if np.linalg.norm(self.piece_position(slot)[:2] - expected) > self.config.tolerances.disturbance:
                return "Piece displaced on " + chess.square_name(sq)
            if self.piece_upright(slot) < self.config.tolerances.upright_min:
                return "Piece toppled on " + chess.square_name(sq)
        return None
