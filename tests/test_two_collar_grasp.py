"""The straight fingers must encompass both collars, not just the upper one."""
from dataclasses import replace

import numpy as np
import pytest

from chess_sim import gripper
from chess_sim.assets import PieceGeometry
from chess_sim.config import BoardConfig
from chess_sim.controller import grasp_plan


@pytest.mark.skipif(not gripper.CUSTOM, reason="Straight parallel-finger contact-band contract")
def test_lower_tip_encompasses_both_collars_without_changing_closure():
    # Public synthetic two-collar shape; no private mesh is needed.
    shape = PieceGeometry(scale=1.0, height=.047, width=.020,
        bottom_offset=0.0, center=np.array([0., 0., .0235]),
        profile=((0., .010, .010), (.010, .025, .004),
                 (.025, .027, .00625), (.027, .039, .0048),
                 (.039, .041, .006575), (.041, .047, .0024)),
        grasp_height=.0327, grasp_width=.009675)
    board = BoardConfig()
    old = grasp_plan(shape, board)
    new = grasp_plan(replace(shape, grasp_tip_offset=-.008), board)
    lower_collar, upper_collar = .026, .040
    old_tip = old.z_grasp - board.top
    new_tip = new.z_grasp - board.top
    assert lower_collar < old_tip < upper_collar
    assert new_tip < lower_collar < upper_collar < new_tip + gripper.PROFILE['finger_proximal_reach_m']
    assert new_tip - gripper.TIP_X >= gripper.TIP_CLEARANCE
    assert new.gap_open >= 2 * .006575 + .006
    assert new.gap_hold == pytest.approx(old.gap_hold)
    assert new.gap_open == pytest.approx(old.gap_open)
