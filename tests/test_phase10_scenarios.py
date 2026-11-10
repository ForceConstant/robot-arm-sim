import chess

from tools.phase10_matrix import PIECE_TYPES, build_phase10_scenarios


def test_phase10_matrix_has_300_legal_moves_and_balanced_buckets():
    scenarios = build_phase10_scenarios(20261107)
    assert len(scenarios) == 300
    assert {name: sum(row[0] == name for row in scenarios) for name in
            ("quiet", "capture", "castle", "en_passant")} == {
                "quiet": 180, "capture": 72, "castle": 24, "en_passant": 24}

    for category, fen, uci in scenarios:
        board = chess.Board(fen)
        move = chess.Move.from_uci(uci)
        assert move in board.legal_moves
        if category == "quiet":
            assert not board.is_capture(move)
        elif category == "capture":
            assert board.is_capture(move) and not board.is_en_passant(move)
        elif category == "en_passant":
            assert board.is_capture(move)
        assert board.is_castling(move) == (category == "castle")
        assert board.is_en_passant(move) == (category == "en_passant")


def test_quiet_and_capture_moves_cover_each_actor_type_and_color():
    scenarios = build_phase10_scenarios(20261108)
    for category, expected_per_actor in (("quiet", 15), ("capture", 6)):
        for color in (chess.WHITE, chess.BLACK):
            for piece_type in PIECE_TYPES:
                count = 0
                for row_category, fen, uci in scenarios:
                    if row_category != category:
                        continue
                    board = chess.Board(fen)
                    move = chess.Move.from_uci(uci)
                    piece = board.piece_at(move.from_square)
                    count += piece.piece_type == piece_type and piece.color == color
                assert count == expected_per_actor, (category, color, piece_type, count)


def test_castle_and_en_passant_cover_both_colors_and_wings():
    scenarios = build_phase10_scenarios(20261109)
    castles = [(chess.Board(fen), chess.Move.from_uci(uci))
               for category, fen, uci in scenarios if category == "castle"]
    for color in (chess.WHITE, chess.BLACK):
        for king_side in (True, False):
            assert sum(board.turn == color and board.is_kingside_castling(move) == king_side
                       for board, move in castles) == 6
    for color in (chess.WHITE, chess.BLACK):
        assert sum(board.turn == color for category, fen, _ in scenarios
                   if category == "en_passant" for board in [chess.Board(fen)]) == 12


def test_every_position_fits_standard_piece_inventory():
    inventory = {chess.PAWN: 8, chess.KNIGHT: 2, chess.BISHOP: 2,
                 chess.ROOK: 2, chess.QUEEN: 1, chess.KING: 1}
    for category, fen, _ in build_phase10_scenarios(20261107):
        board = chess.Board(fen)
        for color in (chess.WHITE, chess.BLACK):
            for piece_type, maximum in inventory.items():
                assert len(board.pieces(piece_type, color)) <= maximum, (
                    category, color, piece_type, fen)


def test_seed_changes_compound_positions_and_all_rows_are_unique():
    from tools.phase10_matrix import scenario_key
    first = build_phase10_scenarios(20261201)
    second = build_phase10_scenarios(20261202)
    assert len({scenario_key(row) for row in first}) == 300
    for category in ('castle', 'en_passant'):
        a = {scenario_key(row) for row in first if row[0] == category}
        b = {scenario_key(row) for row in second if row[0] == category}
        assert len(a) == 24
        assert a != b
    assert first == build_phase10_scenarios(20261201)


def test_ep_right_has_a_legal_double_push_predecessor():
    for category, fen, uci in build_phase10_scenarios(20261201):
        board = chess.Board(fen)
        assert board.is_valid()
        if category != 'en_passant':
            continue
        move = chess.Move.from_uci(uci)
        victim = move.to_square + (-8 if board.turn else 8)
        origin = victim + (16 if board.turn else -16)
        previous = board.copy()
        previous.turn = not board.turn
        previous.ep_square = None
        previous.remove_piece_at(victim)
        previous.set_piece_at(origin, chess.Piece(chess.PAWN, previous.turn))
        double = chess.Move(origin, victim)
        assert previous.is_valid()
        assert double in previous.legal_moves
        previous.push(double)
        assert previous.board_fen() == board.board_fen()
        assert previous.ep_square == board.ep_square
        assert move in previous.legal_moves


# The private gate is deliberately not imported by production runtime.
import importlib.util
import json
from pathlib import Path
import sys
import pytest

_PRIVATE = Path(__file__).resolve().parents[1] / '.private/phase10'


def _private_module(name):
    path = _PRIVATE / (name + '.py')
    if not path.exists():
        pytest.skip('private qualification tooling is not distributed')
    sys.path.insert(0, str(_PRIVATE))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def selection_tools():
    return _private_module('prepare_fresh_selection')


@pytest.fixture(scope='module')
def gate():
    return _private_module('run_heldout_gate')


def test_fresh_selection_preserves_white_quiet_pawn_knight_and_compounds(selection_tools):
    from tools.phase10_matrix import scenario_key
    old = {scenario_key(r) for r in build_phase10_scenarios(20261110)}
    document = selection_tools.prepare(20261201, {20261201}, old)
    selection = document['selection']
    assert len(selection) == 300
    assert 20261201 not in document['reserved_seeds']
    assert not old.intersection(scenario_key((e['category'], e['fen'], e['uci']))
                                for e in selection)
    counts = selection_tools.validate_selection(selection)
    for key in ('actor:white:pawn', 'actor:white:knight', 'task:castle',
                'task:en_passant', 'crowding:dense', 'tray:capture',
                'mass:sensitivity_low', 'friction:0.75', 'offset_mm:-6', 'offset_mm:6'):
        assert counts[key] > 0
    duplicate = [*selection[:-1], selection[0]]
    with pytest.raises(ValueError, match='duplicate'):
        selection_tools.validate_selection(duplicate)


def _receipt_pair(tmp_path, gate):
    selection = [dict(seed=20261201, index=i, category=c, fen=f, uci=u)
                 for i, (c, f, u) in enumerate(build_phase10_scenarios(20261201))]
    scenario_sha = gate.hashlib.sha256(json.dumps(build_phase10_scenarios(20261201),
        separators=(',', ':')).encode()).hexdigest()
    approved = {'source_sha256': {'controller.py': 'frozen'}, 'env_config': {'arm_count': 1}}
    for mode in ('candidate', 'control'):
        directory = tmp_path / mode
        directory.mkdir()
        for e in selection:
            from chess_sim.transactions import plan_chess_transaction
            primitives = len(plan_chess_transaction(
                chess.Board(e['fen']), chess.Move.from_uci(e['uci'])).tasks)
            row = dict(matrix_index=e['index'], fen=e['fen'], uci=e['uci'], category=e['category'],
                       complete=True, commit_allowed=True, controller_outcome='complete',
                       mass_regime=('nominal', 'sensitivity_low', 'sensitivity_high')[e['index'] % 3],
                       board_friction_scale=(.75, 1., 1.25)[(e['index'] % 9) // 3],
                       phase10_primitive_complete=[True]*primitives,
                       runtime_commit_authorization=True,
                       input_game_state_unchanged_by_coordinator=True,
                       primitive_details=[dict(released=True, supported=True, settled=True, phase10_complete=True)] * primitives,
                       wrong_piece_picks_final_pose=0, collateral=0,
                       transient_wrong_piece_or_collateral_count=0)
            board = chess.Board(e['fen'])
            before = board.fen(); board.push_uci(e['uci'])
            row['caller_commit_spy'] = dict(board_fen_before=before,
                board_fen_after=board.fen(), commit_event=e['uci'])
            path = directory / f"case-{e['seed']}-{e['index']:03d}.json"
            path.write_text(json.dumps(dict(schema='robot-arm-phase10-transaction-matrix/1',
                cases=1, seed=e['seed'], records=[row],
                provenance={**approved, 'scenario_sha256': scenario_sha})))
            path.with_suffix('.execution.json').write_text(json.dumps(dict(mode=mode,
                exit=0, selection_sha256='selection', receipt_sha256=gate.digest(path))))
    return selection, approved


def _change_receipt(tmp_path, gate, index, mutate, mode='candidate'):
    path = tmp_path / mode / f'case-20261201-{index:03d}.json'
    receipt = json.loads(path.read_text())
    mutate(receipt)
    path.write_text(json.dumps(receipt))
    meta_path = path.with_suffix('.execution.json')
    meta = json.loads(meta_path.read_text())
    meta['receipt_sha256'] = gate.digest(path)
    meta_path.write_text(json.dumps(meta))


def test_runtime_grouping_ignores_only_seed_specific_scenario_hash(gate):
    assert gate.runtime_provenance(dict(source_sha256={'x': 'a'}, scenario_sha256='one')) == \
           gate.runtime_provenance(dict(source_sha256={'x': 'a'}, scenario_sha256='two'))
    assert gate.runtime_provenance(dict(source_sha256={'x': 'a'})) != \
           gate.runtime_provenance(dict(source_sha256={'x': 'b'}))


def test_paired_gate_passes_complete_frozen_evidence_and_reports_buckets(tmp_path, gate):
    selection, approved = _receipt_pair(tmp_path, gate)
    result = gate.audit(selection, tmp_path/'candidate', 'candidate', 'selection',
                        approved, tmp_path/'control')
    assert result['qualification_verdict'] == 'pass'
    assert result['paired']['matched_cases'] == 300
    assert result['buckets']['task:castle']['cases'] == 24
    assert result['buckets']['actor:white:knight']['cases'] >= 21
    assert result['paired_comparison']['both_complete'] == 300


@pytest.mark.parametrize('fault', ['missing', 'runtime', 'scenario', 'arm', 'unsafe',
                                  'collateral', 'transient', 'unreleased', 'evaluator', 'malformed', 'checkpoint'])
def test_gate_cannot_pass_partial_mixed_unsafe_or_mispaired_receipts(tmp_path, gate, fault):
    selection, approved = _receipt_pair(tmp_path, gate)
    if fault == 'missing':
        (tmp_path/'control'/'case-20261201-000.json').unlink()
    elif fault == 'arm':
        path = tmp_path/'control'/'case-20261201-000.execution.json'
        value = json.loads(path.read_text()); value['mode'] = 'candidate'
        path.write_text(json.dumps(value))
    else:
        def mutate(d):
            row = d['records'][0]
            if fault == 'runtime': d['provenance']['source_sha256'] = {'controller.py':'unapproved'}
            elif fault == 'scenario': d['provenance']['scenario_sha256'] = 'wrong'
            elif fault == 'unsafe': row['complete'] = False
            elif fault == 'collateral': row['collateral'] = 1
            elif fault == 'transient': row['transient_wrong_piece_or_collateral_count'] = 1
            elif fault == 'unreleased': row['primitive_details'][0]['released'] = False
            elif fault == 'evaluator': del row['collateral']
            elif fault == 'malformed': row['primitive_details'] = None
            elif fault == 'checkpoint': d['schema'] = 'robot-arm-phase10-transaction-checkpoint/2'
        _change_receipt(tmp_path, gate, 0, mutate)
    result = gate.audit(selection,tmp_path/'candidate','candidate','selection',approved,tmp_path/'control')
    assert result['qualification_verdict'] != 'pass'


def test_gate_refusals_count_against_99_percent_and_approval_is_required(tmp_path, gate):
    selection, approved = _receipt_pair(tmp_path, gate)
    def refuse(d):
        row = d['records'][0]
        row.update(complete=False, commit_allowed=False, runtime_commit_authorization=False,
                   controller_outcome='uncertain')
        spy = row['caller_commit_spy']
        spy.update(board_fen_after=spy['board_fen_before'], commit_event=None)
    for i in range(3): _change_receipt(tmp_path,gate,i,refuse)
    result = gate.audit(selection,tmp_path/'candidate','candidate','selection',approved,tmp_path/'control')
    assert result['rate'] == .99
    assert result['qualification_verdict'] == 'pass'
    assert len(result['failures']) == 3
    _change_receipt(tmp_path,gate,3,refuse)
    result = gate.audit(selection,tmp_path/'candidate','candidate','selection',approved,tmp_path/'control')
    assert result['qualification_verdict'] == 'fail'
    assert len(result['failures']) == 4
    result = gate.audit(selection,tmp_path/'candidate','candidate','selection',None,tmp_path/'control')
    assert result['qualification_verdict'] == 'incomplete'


def test_summarize_only_never_replaces_failure_receipts_or_previous_summaries(tmp_path, gate, monkeypatch, capsys):
    selection, _ = _receipt_pair(tmp_path, gate)
    pool = tmp_path/'selection.json'
    pool.write_text(json.dumps({'selection':selection}))
    receipt = tmp_path/'candidate'/'case-20261201-000.json'
    original = receipt.read_bytes()
    previous_summary = tmp_path/'candidate'/'summary.json'
    previous_summary.write_text('{"preserved_failure": true}\n')
    monkeypatch.setattr(sys,'argv',['run_heldout_gate.py','--mode','candidate',
        '--pool',str(pool),'--out',str(tmp_path/'candidate'),'--summarize-only'])
    assert gate.main() == 2
    assert receipt.read_bytes() == original
    assert previous_summary.read_text() == '{"preserved_failure": true}\n'
    new_summary = next((tmp_path/'candidate').glob('summary-audit-*.json'))
    assert json.loads(new_summary.read_text())['qualification_verdict'] != 'pass'
    capsys.readouterr()


def test_runtime_snapshot_matches_benchmark_provenance_contract(monkeypatch):
    import ast
    freeze = _private_module('freeze_runtime')
    root = Path(__file__).resolve().parents[1]
    if not (root/'.private/phase9/calibration.json').exists():
        pytest.skip('private calibration required for authored geometry freeze')
    for k,v in freeze.benchmark_environment().items():
        monkeypatch.setenv(k,v)
    snapshot = freeze.snapshot()
    tree = ast.parse((root/'tools/benchmark_phase10_compound.py').read_text())
    assignments = {n.targets[0].id:n.value for n in ast.walk(tree)
                   if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name)}
    expected_keys = {k.value for k in assignments['provenance'].keys} - {'scenario_sha256'}
    assert set(snapshot) == expected_keys
    sources = eval(compile(ast.Expression(assignments['source_paths']),'<benchmark-source-list>','eval'),
                   {'Path':Path,'ROOT':root,'__file__':str(root/'tools/benchmark_phase10_compound.py')})
    assert snapshot['source_sha256'] == {str(p.relative_to(root)):freeze.hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in sources}
    assert snapshot['self_model_source_sha256']


def test_execution_never_retries_existing_evidence_without_skip_flag(tmp_path, gate, monkeypatch, capsys):
    selection, approved = _receipt_pair(tmp_path, gate)
    pool = tmp_path/'selection.json'
    pool.write_text(json.dumps({'selection':selection,
        'matrix_sha256':gate.digest(gate.ROOT/'tools/phase10_matrix.py')}))
    receipt = tmp_path/'candidate'/'case-20261201-000.json'
    original = receipt.read_bytes()
    monkeypatch.setattr(gate,'approved_runtime',lambda _:approved)
    monkeypatch.setattr(gate,'check_current_sources',lambda _:[])
    monkeypatch.setattr(gate,'snapshot',lambda:approved)
    monkeypatch.setattr(gate,'benchmark_environment',lambda:{})
    def unexpected_simulation(*args,**kwargs):
        pytest.fail('an existing receipt must never be re-executed/replaced')
    monkeypatch.setattr(gate.subprocess,'run',unexpected_simulation)
    monkeypatch.setattr(sys,'argv',['run_heldout_gate.py','--mode','candidate',
        '--pool',str(pool),'--out',str(tmp_path/'candidate'),'--approved-runtime','test-freeze'])
    assert gate.main() == 2  # pair and sidecar selection digest intentionally absent
    assert receipt.read_bytes() == original
    assert not list((tmp_path/'candidate').glob('*.claim'))
    capsys.readouterr()


@pytest.mark.parametrize('king_side, expected', [(True, 2), (False, 3)])
@pytest.mark.parametrize('count_delta', [-1, 1])
def test_gate_rejects_wrong_castling_primitive_counts(tmp_path, gate, king_side, expected, count_delta):
    selection, approved = _receipt_pair(tmp_path, gate)
    entry = next(e for e in selection if e['category'] == 'castle' and
                 chess.Board(e['fen']).is_kingside_castling(chess.Move.from_uci(e['uci'])) == king_side)
    path = tmp_path/'candidate'/f"case-20261201-{entry['index']:03d}.json"
    assert len(json.loads(path.read_text())['records'][0]['primitive_details']) == expected
    def wrong_count(receipt):
        row = receipt['records'][0]
        row['primitive_details'] = [dict(released=True, supported=True, settled=True,
                                         phase10_complete=True)] * (expected + count_delta)
        row['phase10_primitive_complete'] = [True] * (expected + count_delta)
    _change_receipt(tmp_path, gate, entry['index'], wrong_count)
    result = gate.audit(selection, tmp_path/'candidate', 'candidate', 'selection', approved, tmp_path/'control')
    assert result['qualification_verdict'] == 'fail'
    assert result['unverified_completions'] == 1
