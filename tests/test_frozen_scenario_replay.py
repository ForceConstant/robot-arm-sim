"""A consumed case retains its identity when generator revisions change."""
import importlib
import json
from pathlib import Path

import pytest


def test_frozen_identity_and_seed_are_preserved(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools'))
    benchmark = importlib.import_module('tools.benchmark_phase10_compound')
    entry = dict(seed=123, index=10, category='quiet',
                 fen='rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1',
                 uci='e2e4')
    path = tmp_path / 'selection.json'
    path.write_text(json.dumps({'selection': [entry, {**entry, 'seed': 124, 'uci': 'd2d4'}]}))
    monkeypatch.setattr(benchmark, 'build_phase10_scenarios', lambda _: pytest.fail('regenerated'))
    assert benchmark.frozen_scenarios(path, 123) == {10: ('quiet', entry['fen'], 'e2e4')}
    path.write_text(json.dumps([entry, entry]))
    with pytest.raises(ValueError, match='duplicate'):
        benchmark.frozen_scenarios(path, 123)
    path.write_text(json.dumps([{**entry, 'category': 'capture'}]))
    with pytest.raises(ValueError, match='identity'):
        benchmark.frozen_scenarios(path, 123)
