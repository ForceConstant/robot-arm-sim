"""Audit the rerun's evidence, including the formerly repeated tray target."""
import hashlib
import json
from pathlib import Path
import zlib

import chess
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('kind', ['stock', 'cad_parallel'])
def test_current_mount_qualification_receipt(kind):
    report = json.loads((ROOT / f'reports/phase8_updated_{kind}.json').read_text())
    layout = json.loads((ROOT / 'hardware/current_layout.json').read_text())
    assert report['stage'] == 'complete'
    assert report['tool']['kind'] == kind
    assert report['layout'] == layout
    assert report['config']['board']['arm_gap'] == .09
    assert report['config']['board']['arm_riser'] == .03
    assert report['config']['dual_arms'] is False
    squares = set(chess.SQUARE_NAMES)
    for state in ('empty_board', 'crowded_start'):
        assert set(report[state]['squares']) == squares
        assert all(set(r['source']) == {'pawn', 'rook', 'knight', 'bishop', 'queen', 'king'}
                   for r in report[state]['squares'].values())
    ex = report['execution']
    assert {r['square'] for r in ex['rows']} == squares
    assert ex['total'] == len(ex['rows']) == 64
    assert ex['first_pass'] == sum(r['first']['success'] for r in ex['rows'])
    assert ex['path_feasible'] == sum(r['path_feasible'] for r in ex['rows'])
    assert all(len(r['retries']) == 2 for r in ex['rows'] if not r['first']['success'])
    tray = report['tray_execution']
    targets = [tuple(r['target_mm']) for r in tray['rows']]
    assert len(set(targets)) == tray['total'] == 4
    for i, target in enumerate(targets):
        assert target == pytest.approx((-160, -100 + 40*i))
    assert report['phase8_pass'] == (ex['path_feasible'] == 64 and tray['passed'] == 4)
    for path, digest in report['source_sha256'].items():
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest


def test_span_verification_is_a_matched_case_parameter_experiment():
    baseline = json.loads((ROOT / 'reports/phase8_updated_cad_parallel.json').read_text())
    candidate = json.loads((ROOT / 'reports/phase8_updated_span_verification.json').read_text())
    before = {r['case']: r for r in baseline['crowded_execution']['rows']}
    after = {r['case']: r for r in candidate['rows']}
    assert candidate['total'] == len(after) == len(before) == 25
    assert candidate['passed'] == sum(r['success'] for r in after.values())
    assert candidate['layout'] == baseline['layout']
    changed = {'white-queen', 'white-queen-far', 'white-king', 'white-king-far'}
    for case, row in after.items():
        assert row['seed'] == before[case]['seed'] == zlib.crc32(case.encode()) & 0xffffffff
        assert row['move'] == before[case]['move']
        assert row['params'] == ({'span_deg': 20} if case in changed else {})
