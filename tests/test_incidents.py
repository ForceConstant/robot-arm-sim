"""Recorder and torture-suite contracts.

The physics test is deliberately small: one pick, one recording, then a
restore round-trip. It proves saved states reproduce the scene exactly, not
that playback re-simulates physics.
"""
import json
from pathlib import Path

import chess
import numpy as np
import pytest

from chess_sim import AppearanceConfig, ControlConfig, EnvConfig, MoveTask
from chess_sim.dual import DualArmEnv
from robot_web.incidents import (LABELS, Recording, Store, bookkeeping,
                                 DIAGNOSTIC_DEFAULTS, model_signature, physics_state,
                                 reset_diagnostics, restore, runtime_signature)
from robot_web.torture import CASES, SUITE_VERSION


def test_suite_cases_are_distinct_and_parseable():
    ids = [c.id for c in CASES]
    assert len(ids) == len(set(ids))
    assert SUITE_VERSION == "crowded-v2"
    assert len(CASES) == 24
    for case in CASES:
        board = chess.Board(case.fen)
        assert board.is_valid()
        assert case.arm in ("white", "black")
        move = chess.Move.from_uci(case.move)
        piece = board.piece_at(move.from_square)
        assert piece is not None and piece.color == (case.arm == "white")
        # The arm's side must be the side to move, or python-chess reports an
        # illegal move as a capture and the case becomes a discard task.
        assert board.turn == (case.arm == "white"), case.id
        assert board.is_capture(move) is False, case.id
        assert board.piece_at(move.to_square) is None, case.id
        # Long illegal king moves are classified by python-chess as castling;
        # the suite must not trigger an unrelated second rook manipulation.
        assert not board.is_castling(move), case.id
    # The two observed failures stay in the suite as regression cases.
    assert {"white-bishop", "black-bishop"} <= set(ids)
    assert {"white-rook-corner", "black-rook-corner"} <= set(ids)
    assert {"white-pawn-far", "black-pawn-far", "white-knight-far", "black-knight-far",
            "white-bishop-far", "black-bishop-far", "white-queen-far", "black-queen-far",
            "white-king-far", "black-king-far"} <= set(ids)


def test_store_rejects_unknown_or_malformed_ids(tmp_path):
    store = Store(tmp_path)
    for bad in ("../../etc", "abc", "0" * 31):
        with pytest.raises(ValueError):
            store.path(bad)
    with pytest.raises(ValueError):
        store.path("f" * 32)


@pytest.fixture(scope="module")
def env():
    e = DualArmEnv(EnvConfig(control=ControlConfig(cameras=()),
                             appearance=AppearanceConfig(camera_mast=False)))
    e.reset()
    yield e
    e.close()


def test_recording_round_trip(env, tmp_path):
    store = Store(tmp_path)
    model_hash = model_signature(env)
    code_hash = runtime_signature(model_hash)
    before = physics_state(env)
    book = bookkeeping(env)
    rec = Recording(store, env, model_hash, code_hash,
                    {"action": "move", "task": {"kind": "move", "from": "e2", "to": "e4"},
                     "arm": "white", "case_id": "test"}, {})
    rec.target_body = env._square_slot[chess.E2].body

    result = env.execute(MoveTask(from_square="e2", to_square="e4"),
                         on_step=lambda action: (rec.contact_sample(), rec.sample(action)))
    meta = rec.finish(result.success, {"success": result.success,
                                      "error_mm": result.placement_error * 1000,
                                      "disturbed": [str(d) for d in result.disturbed],
                                      "upright": bool(result.upright)}, "", {})

    assert meta["frames"] == len(rec.states) > 10
    manifest = json.loads((store.root / meta["id"] / "manifest.json").read_text())
    assert manifest["model_signature"] == model_hash
    assert manifest["runtime_signature"] == code_hash
    assert manifest["initial"]["position"] == book["position"]
    trace = json.loads((store.root / meta["id"] / "trace.json").read_text())
    assert len(trace) == manifest["frames"]
    assert {"phase", "actual_tcp", "actual_gap_m"} <= set(trace[0])
    # Phases must be real labels, not the placeholder from before execution.
    assert {"approach", "close", "lift", "carry", "release", "retreat"} & {
        step["phase"] for step in trace}
    assert [r["id"] for r in store.rows()] == [meta["id"]]

    loaded, states, loaded_trace = store.load(meta["id"], model_hash)
    assert np.allclose(states[0], rec.states[0])
    assert loaded_trace[0]["phase"] == trace[0]["phase"]

    # Restoring frame 0 must reproduce the exact pre-action physics state.
    restore(env, states[0], loaded["initial"])
    assert np.allclose(physics_state(env), before)
    assert env.position.fen() == book["position"]
    assert len(env._square_slot) == 32

    # Playback is a state visualisation, never claimed as re-simulation.
    assert "not a physical re-simulation" in manifest["playback"]
    with pytest.raises(ValueError):
        store.annotate(meta["id"], "not-a-label", "")
    store.annotate(meta["id"], "slip_drop", "reviewed in test")
    assert store.read(meta["id"])["review_label"] == "slip_drop"
    assert "slip_drop" in LABELS


def test_camera_views_are_fixed_and_well_formed():
    from robot_web.lab import VIEWS
    assert {"overview", "top", "left", "right", "white_side", "black_side"} <= set(VIEWS)
    assert sum(1 for name in VIEWS if name.startswith("iso_")) == 4
    for name, (kind, value) in VIEWS.items():
        assert kind in ("camera", "orbit"), name
        if kind == "orbit":
            assert 0.0 <= value["azimuth"] < 360.0, name
            assert -89.0 < value["elevation"] < 0.0, name      # above the table, never past the pole
            assert 0.2 < value["distance"] < 1.5, name


def test_diagnostics_do_not_leak_between_actions():
    class Fake:
        diagnostic_phase = "abort_release"
        diagnostic_target = [1, 2, 3]
        diagnostic_gap = 0.008
        diagnostic_ik = {"residual_m": 0.05}
        diagnostic_stall = "place_descend"

    fake = Fake()
    reset_diagnostics(fake)
    for key, value in DIAGNOSTIC_DEFAULTS.items():
        assert getattr(fake, key) == value, key
