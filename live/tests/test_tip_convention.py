"""Regression: solver-reported error and the guard's tip agree at a1 and h8.

This is the block-1 gate.  The 2026-11-17 failures were a frame-composition
split, not reach: at a1 the solver reported 2.9 mm while the guard's tip was
24 mm out.  The difference matched the jaw frame's lateral y offset, because
the frame had been built from the CAD mount position and the solver/guard
applied different tip definitions.  With one ``TIP_IN_CHAIN`` transform the two
numbers must agree by construction -- and they must agree *mid-sequence*, seeded
from whatever pose the previous square left, not only from a fresh home posture.

No MuJoCo here: this checks the live maths (calibration chain + one constant).
The MJCF evidence for the constant itself is ``tools/audit_tip_frame.py``.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

LIVE = Path(__file__).resolve().parents[1]
ROOT = LIVE.parent
sys.path.insert(0, str(LIVE))

from kinematics import JOINTS, ArmKinematics, square_xy  # noqa: E402
from tip_frame import TIP_IN_CHAIN  # noqa: E402

CONFIG = json.loads((LIVE / "live_config.json").read_text())
CALIBRATION = ROOT / CONFIG.get("calibration", ".private/phase11d/calibration.json")
HOVER_MM = 60.0
TOLERANCE_MM = float(CONFIG.get("move_tolerance_mm", 20.0))
AGREEMENT_MM = 2.0


def board_frame() -> np.ndarray:
    spec = json.loads((LIVE / "board.json").read_text())
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = np.eye(4)
    frame[:3, :3] = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                              [math.sin(yaw), math.cos(yaw), 0.0],
                              [0.0, 0.0, 1.0]])
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    return frame


def kinematics() -> ArmKinematics:
    from chess_sim.wrist_calibration import WristCalibration
    calibration = WristCalibration.load(str(CALIBRATION))
    return ArmKinematics(calibration, {}, board_frame())


def home_joints() -> list[float]:
    home = CONFIG["home_joints"]
    return [float(home[name]) for name in JOINTS]


def solve_square(kin: ArmKinematics, square: str, seed, z_mm: float = HOVER_MM):
    """Solve one square exactly as ``/move`` does: tip pose -> chain goal."""
    x_mm, y_mm = square_xy(square)
    point = kin.board_point(x_mm, y_mm, z_mm)
    solved, goal = kin.solve_tip(point, seed, TIP_IN_CHAIN)
    joints = [float(solved["joints_deg"][name]) for name in JOINTS]
    return solved, joints, (x_mm, y_mm, z_mm)


def guard_tip_error(kin: ArmKinematics, joints, target) -> float:
    """What ``_keep_in_check`` sees: fk(joints) @ jaw frame, in board mm."""
    tip = (kin.fk(joints) @ TIP_IN_CHAIN)[:3, 3]
    tip_mm = kin.to_board(tip) * 1000.0
    return float(np.linalg.norm(tip_mm - np.asarray(target, dtype=float)))


@pytest.mark.skipif(not CALIBRATION.exists(),
                    reason="live calibration is not in this checkout")
def test_solver_and_guard_agree_mid_sequence():
    assert "tool_tip_mm" not in CONFIG
    assert "jaw_frame" not in CONFIG, "the tip frame must not be a config override"
    assert abs(float(TIP_IN_CHAIN[2, 3]) * 1000.0 + 104.425) < 1e-6
    # The tip must sit on the chain's approach axis: an off-axis origin turns
    # the solver's free in-plane rotation into millimetres of tip error.
    assert np.allclose(TIP_IN_CHAIN[:2, 3], 0.0, atol=1e-9)

    kin = kinematics()
    seed = home_joints()

    # a1 is the first move after home; h8 is solved from whatever a1 left.
    for square in ("a1", "h8"):
        solved, joints, target = solve_square(kin, square, seed)
        solver_mm = float(solved["position_error_mm"])
        guard_mm = guard_tip_error(kin, joints, target)
        print(f"{square}: solver {solver_mm:.2f} mm, guard tip {guard_mm:.2f} mm, "
              f"delta {abs(solver_mm - guard_mm):.3f} mm")
        # Both squares must clear the gate, and the solver and the guard must
        # be looking at the same tip.  h8 sits at the far edge (the base is at
        # board (0, -217) mm, so h8 is ~330 mm out); it clears the gate only
        # from the extended-branch seeds in ``solve_tip`` -- the folded family
        # alone stalls at 20.8 mm.
        assert solver_mm <= TOLERANCE_MM, (
            f"{square}: solver missed by {solver_mm} mm")
        assert abs(solver_mm - guard_mm) <= AGREEMENT_MM, (
            f"{square}: solver and guard disagree by "
            f"{abs(solver_mm - guard_mm):.2f} mm")
        seed = joints
