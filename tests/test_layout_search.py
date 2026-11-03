"""Independent task definitions and the formerly short A8 position solution."""
import chess
import numpy as np
import pytest

from scripts.optimize_layout import CASES,case_fen,rank


def test_selected_runtime_layout_matches_qualification_and_defaults():
    import json
    from pathlib import Path
    from chess_sim import BoardConfig
    root=Path(__file__).resolve().parents[1]
    current=json.loads((root/'hardware/current_layout.json').read_text())
    board=BoardConfig()
    assert board.arm_gap*1000==current['mount']['board_edge_to_base_frame_mm']
    assert board.arm_riser*1000==current['mount']['base_mount_datum_above_table_mm']
    assert board.thickness*1000==current['board']['thickness_mm']
    full=json.loads((root/'reports/layout_full_board.json').read_text())['best']
    assert full['passed']==full['total']==64
    assert {c['source'] for c in full['cases']}==set(chess.SQUARE_NAMES)


def test_failed_alternate_roll_does_not_discard_valid_primary_solution():
    import mink
    from types import SimpleNamespace
    from chess_sim.ik import So101Ik
    obj=object.__new__(So101Ik)
    obj.qpos_adr=np.arange(5);obj.lower=np.full(5,-3.0);obj.upper=np.full(5,3.0)
    calls=[]
    def iterate(seed,*args):
        calls.append(seed)
        if len(calls)>1:raise mink.exceptions.NoSolutionFound('daqp')
        return np.zeros(5),0.0
    obj._iterate=iterate
    obj.forward=lambda q,offset:(np.zeros(3),np.eye(3))
    result=obj.solve(SimpleNamespace(qpos=np.zeros(5)),np.zeros(3),span=np.array([1.,0.]))
    assert len(calls)==2 and result.position_error==0


def test_layout_cases_are_independent_physical_tasks():
    assert len({c[0] for c in CASES})==len(CASES)
    assert {c[2] for c in CASES}>={'a1','h1','a8','h8','d1','e1'}
    for _,kind,source,destination in CASES:
        board=chess.Board(case_fen(kind,source))
        move=chess.Move.from_uci(source+destination)
        assert board.turn==chess.WHITE
        assert len(board.piece_map())==1
        assert board.piece_at(move.from_square).piece_type==getattr(chess,kind.upper())
        assert board.piece_at(move.to_square) is None
        assert not board.is_castling(move) and not board.is_capture(move)


def test_layout_ranking_prefers_execution_success_over_small_mean_error():
    base=dict(riser_mm=0,gap_mm=80)
    passed=dict(base,passed=8,worst_error_mm=9,mean_error_mm=5)
    missed=dict(base,passed=7,worst_error_mm=2,mean_error_mm=1)
    assert rank(passed)<rank(missed)


def test_served_layout_suite_mirrors_the_same_tasks_without_self_captures():
    from robot_web.torture import LAYOUT_CASES
    assert len(LAYOUT_CASES)==2*len(CASES)
    for case in LAYOUT_CASES:
        board=chess.Board(case.fen);move=chess.Move.from_uci(case.move)
        assert board.turn==(case.arm=='white')
        assert board.piece_at(move.from_square).color==board.turn
        assert not board.is_capture(move) and not board.is_castling(move)


def test_custom_tool_reaches_a8_without_sacrificing_position_for_verticality():
    from chess_sim import gripper
    if not gripper.CUSTOM:
        pytest.skip('requires explicitly supplied custom CAD pack')
    from chess_sim.env import ChessSimEnv
    from chess_sim.product import load_product_twin
    from chess_sim.controller import grasp_plan
    cfg=load_product_twin().env_config(piece_set='printed',cameras=())
    cfg=cfg.model_copy(update={'board':cfg.board.model_copy(update={'arm_gap':.1,'arm_riser':0.0})})
    env=ChessSimEnv(cfg)
    try:
        env.reset(case_fen('rook','a8'))
        slot=env.slot_at('a8');plan=grasp_plan(slot.geometry,env.board)
        point=np.array([*env.board.square_center('a8'),plan.z_grasp])
        result=env.ik.solve(env.data,point,offset=plan.off_open)
        assert result.position_error<.0015
        assert result.tilt<np.radians(30)
        assert np.all(result.q>=env.ik.lower-1e-8)
        assert np.all(result.q<=env.ik.upper+1e-8)
    finally:env.close()
