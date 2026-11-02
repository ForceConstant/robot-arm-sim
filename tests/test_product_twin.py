"""Phase 8 product-twin contract.

These tests freeze the measured product scene and its interfaces.  They do not
claim physical manipulation or wrist-only perception; those remain later gates.
"""
import json
from pathlib import Path

import chess
import pytest

from chess_sim import JOINTS, BoardConfig, EnvConfig, ToleranceConfig
from chess_sim.product import (PRODUCT_TWIN_PATH, contract_digest,
                               load_product_twin, mass_properties,
                               piece_mass_properties, rook_mass_range)
from chess_sim.conventions import SO101_DEGREES

ROOT = Path(__file__).resolve().parent.parent
REPORT = ROOT / "reports" / "phase8_product_qualification.json"


def test_measured_board_and_single_follower():
    twin = load_product_twin()
    assert twin.board.square_mm == 28.0
    assert twin.board.border_mm == 15.0
    assert twin.board.thickness_mm == 3.2
    assert twin.board.labels is True
    assert twin.board.playing_field_mm == 224.0
    assert twin.board.overall_mm == 254.0
    assert twin.robot.arms == 1
    assert twin.robot.arm_side == "white"
    assert tuple(twin.robot.joint_order) == tuple(str(j) for j in JOINTS)
    assert twin.mount.arm_base_from_board_centre_mm == (0.0, -207.0)
    board = twin.board_config()
    assert board.square == pytest.approx(0.028)
    assert board.thickness == pytest.approx(0.0032)
    assert board.top == pytest.approx(0.4332)
    assert board.arm_base[1] == pytest.approx(-0.207)


def test_product_env_is_wrist_only_and_uses_frozen_evaluator():
    twin = load_product_twin()
    cfg = twin.env_config()
    assert cfg.dual_arms is False
    assert [str(c) for c in cfg.control.cameras] == ["wrist"]
    assert cfg.tolerances == ToleranceConfig(
        placement=0.011, tray=0.030, disturbance=0.010, upright_min=0.85)
    assert twin.evaluator.placement_mm == 11.0
    assert twin.evaluator.tray_mm == 30.0
    assert twin.evaluator.disturbance_mm == 10.0
    assert twin.evaluator.upright_min == 0.85


def test_rook_measurement_and_unknown_piece_ranges():
    twin = load_product_twin()
    rook = twin.pieces["rook"]
    assert rook.measured is True
    assert rook.assembled.mass_g == pytest.approx(4.8)
    assert rook.nut_mass_g == pytest.approx(4.8 - rook.unweighted_pla.mass_g)
    assert rook.nut_model_z_range_mm == (2.0, 4.0)
    low, high = rook_mass_range(twin)
    assert low.mass_g == high.mass_g == pytest.approx(4.8)
    assert low.com_mm[2] != high.com_mm[2]       # the nut-height bound moves COM
    for name in ("pawn", "knight", "bishop", "queen", "king"):
        piece = twin.pieces[name]
        assert piece.measured is False
        assert piece.assembled is None
        assert piece.mass_range_g[0] == pytest.approx(piece.unweighted_pla.mass_g)
        assert piece.mass_range_g[1] > piece.mass_range_g[0]
        nominal = piece_mass_properties(twin, name, "nominal")
        heavy = piece_mass_properties(twin, name, "heavy_bound")
        assert heavy.mass_g > nominal.mass_g


def test_point_mass_combination_is_physically_sane():
    twin = load_product_twin()
    base = twin.pieces["rook"].unweighted_pla
    light = mass_properties(base, 0.0, 3.0)
    heavy = mass_properties(base, 3.0, 3.0)
    assert light.mass_g == pytest.approx(base.mass_g)
    assert heavy.mass_g > light.mass_g
    assert heavy.com_mm[2] < light.com_mm[2]     # bottom weight lowers COM
    assert heavy.inertia_kg_m2[0][0] > 0
    assert heavy.inertia_kg_m2[2][2] > 0


def test_tool_selection_is_explicit():
    twin = load_product_twin()
    assert twin.tool.kind == twin.tool_selection.selected == "stock"
    rejected = {r.kind: r for r in twin.tool_selection.rejected}
    assert "cad_parallel" in rejected
    assert "failed" in rejected["cad_parallel"].result.lower()


def test_frozen_interface_and_dataset_units():
    twin = load_product_twin()
    assert "hxwx3" in twin.interfaces["observation"].lower()
    assert "joints order" in twin.interfaces["action"].lower()
    assert twin.interfaces["dataset_convention"].startswith("chess_sim.conventions.SO101")
    assert SO101_DEGREES.degrees is True
    assert SO101_DEGREES.signs == (1.0, -1.0, 1.0, 1.0, 1.0, 1.0)
    assert SO101_DEGREES.offsets == (0.0, 90.0, 90.0, 0.0, 0.0, 0.0)


def test_committed_qualification_report_is_complete_when_present():
    if not REPORT.exists():
        pytest.skip("run scripts/qualify_phase8.py to generate the report")
    report = json.loads(REPORT.read_text())
    assert report["schema"] == "chess-robot-phase8-qualification/1"
    assert report["product_twin"]["board"]["square_mm"] == 28.0
    assert report["product_twin"]["board"]["thickness_mm"] == 3.2
    for block in ("empty_board", "crowded_start"):
        squares = report[block]["squares"]
        assert len(squares) == 64
        assert set(squares) == {chess.square_name(s) for s in chess.SQUARES}
        assert all("source" in row and "destination" in row for row in squares.values())
    assert report["execution"]["total"] == 64
    assert report["tray_execution"]["total"] == 4
    assert report["phase8_pass"] is True
