"""Frame/rotation pins — FULL-PARITY-01 T2b (sim side, no arm, no MuJoCo).

These are the cheap, hardware-free guards against the drift pain points:
board reference/rotation, board set-down, jaw placement, tool frame, tip
constant and camera reference.  They pin *literal values* from the written
sources of truth, so a silent revert fails here even if someone changed both
the model and the constant the model is checked against.

This test is deliberately strict: changing a pinned value is a **deliberate
re-baseline**, not a test to waive.  Per docs/TESTS.md §3b it requires a written
reason, an updated source of truth, a fresh human-reviewed contact sheet at the
new commit, and Brian's approval recorded in the receipt.  Do not edit these
expectations to accept an unexplained revert.

Companion guards (already in the suite, and still required):
* ``test_tool_orientation.py`` — built model vs the tool_frames chain.
* ``test_board_facing.py`` — board facing against the arm's position.
* ``test_aim_geometry.py`` — GL<->OpenCV flip and tip constant invariance.
* ``test_hand_eye.py`` — mount perturb/recover.

Run on the BC-250 via ``robot-arm-sim test tests/test_frame_pins.py``.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
RIG = ROOT / "rig_state"

# --- frozen expectations (docs/TESTS.md §3 / §3b) --------------------------
BOARD_EXPECTED = {
    "x_mm": -27.62,
    "y_mm": -31.63,
    "z_mm": 432.65,
    "yaw_deg": 184.32,
}
MANIFEST_HASHES = {
    "board.json": "9928f253514dee0ad42914889d5a156b0eba511b5b6acd07cdbf367d471ec665",
    "live_config.json": "4c28d87905dd904787bf1802e1be0f617d8e41d4fa8346e8825a637bc2c73408",
    "wrist_calibration.json": "021bc97f14eabfb3581ae928fe31aff743ae5db90274fe91d634fb339dcc062b",
    "mount_calibration.json": "70f6c51893d294fdf1512c896f5f7171694b06fce8c888e6c04b00a788b144f4",
}
TIP_M = 0.11045                       # live/tip_frame.py -> -TIP_IN_CHAIN z
TIP_DEPTH_MM = 110.450                # chess_sim/tool_frames.py
MOUNT_POS_MM = (9.462, 23.429, -62.577)
MOUNT_QUAT = (0.707107, 0.0, 0.0, -0.707107)


def test_board_json_exact() -> None:
    """The board pose, including the 2026-11-25 delta yaw, is not negotiable.

    A wrong here moves every commanded square and fails silently; the picture
    review in T8 is the only other thing that catches it.
    """
    board = json.loads((RIG / "board.json").read_text())
    assert board == BOARD_EXPECTED, f"board.json reverted: {board}"


def test_rig_state_snapshot_matches_manifest() -> None:
    """Every committed rig snapshot still hashes to the manifest value.

    Also checks the machine-local canonical copy when it exists (it does on
    the BC-250), so a half-restored rig cannot pass.
    """
    manifest = json.loads((RIG / "MANIFEST.json").read_text())
    files = manifest["files"]
    assert set(files) == set(MANIFEST_HASHES), "manifest file set changed"
    for name, entry in files.items():
        expected = MANIFEST_HASHES[name]
        assert entry["sha256"] == expected, f"manifest entry {name} changed"
        for label, path in (("snapshot", RIG / name),
                            ("canonical", ROOT / entry["canonical"])):
            if not path.exists():
                continue
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            assert digest == expected, f"{name} {label} drifted: {digest} != {expected}"


def test_board_facing_config() -> None:
    """The arm plays black; the sim board set-down is the half-turn.

    This is the regression that made every square read one half-turn wrong
    while all geometry tests passed (they read through square_center too).
    """
    from chess_sim.config import ARM_PLAYS, BOARD_TURN

    assert ARM_PLAYS == "black", "board facing reverted to white"
    assert BOARD_TURN == pytest.approx(math.pi), "board set-down turn reverted"


def test_tool_chain_constants() -> None:
    """The written transform chain has not been reverted or re-authored."""
    from chess_sim import tool_frames as tf

    assert tf.MOUNT_POS_MM == pytest.approx(MOUNT_POS_MM)
    assert tf.MOUNT_QUAT == pytest.approx(MOUNT_QUAT)
    assert tf.TIP_DEPTH_MM == pytest.approx(TIP_DEPTH_MM)
    assert tf.TIP_DEPTH_TOL_MM == pytest.approx(0.05)
    lo, hi = tf.BASE_PART_BBOX_MM
    assert lo == pytest.approx((13.25, -32.0, -35.25))
    assert hi == pytest.approx((69.2, 32.0, 23.45))
    assert tf.JAW_BODY_ORIGIN_MM["left"] == pytest.approx(
        (-32.000, 18.550, -54.450))
    assert tf.JAW_BODY_ORIGIN_MM["right"] == pytest.approx(
        (32.000, -18.550, -54.450))
    # Mirror check: the built jaw pair spans x; the two origins are mirrored
    # by flipping x (and y), with z identical.
    left, right = tf.JAW_BODY_ORIGIN_MM["left"], tf.JAW_BODY_ORIGIN_MM["right"]
    assert left[0] == pytest.approx(-right[0], abs=0.01)
    assert left[1] == pytest.approx(-right[1], abs=0.01)
    assert left[2] == pytest.approx(right[2], abs=0.01)


def test_tip_constant_stamp() -> None:
    """The resolved tip constant feeding TIP_IN_CHAIN is the measured one.

    tip_frame.TIP_IN_CHAIN translation z = -gripper_origin_to_tip_m, so a
    reverted stamp silently moves the aim point and the solver target.
    """
    stamp = json.loads((ROOT / "live" / "tool_tip.json").read_text())
    assert stamp["pack"] == "cad_parallel"
    assert stamp["gripper_origin_to_tip_m"] == pytest.approx(TIP_M)
    assert stamp["ok"] is True
