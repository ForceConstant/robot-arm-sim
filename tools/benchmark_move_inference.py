"""Held-out move/state inference evaluation from settled board observations.

The benchmark builds scenes with simulator truth, but the inference path only
ever receives per-square labels from :mod:`chess_sim.perception`.  Previous
positions are the confirmed history; observations are rendered after the move.
A case is correct only when the status and the move/state agree.

Categories:

- ``simple`` — quiet move, no capture, castling, en passant or promotion
- ``capture`` — normal capture
- ``castle`` — king-side castling with rights on the previous board
- ``en_passant`` — en-passant capture using the previous board's ep square
- ``promotion`` — promotion; the piece type is visually identical, so the
  expected outcome is an explicit ambiguous/ask outcome
- ``unchanged`` — same position observed twice
- ``unexplained`` — an illegal board change that no legal move explains
- ``occluded`` — a real move with its source square hidden; must abstain

Example::

    MUJOCO_GL=egl PYTHONPATH=. .venv/bin/python tools/benchmark_move_inference.py \
        --sessions 3 --seed 7001 --views fused \
        --out .private/move-inference/heldout.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import random
import time
from collections import defaultdict
from pathlib import Path

import chess
import mujoco
import numpy as np

from chess_sim.config import Camera
from chess_sim.move_inference import (SquareEvidence, infer_move,
                                      merge_observations)
from chess_sim.perception import (PerceptionModel, board_patches,
                                  predict_board, square_mask_fraction)
from chess_sim.perception_data import (make_session, register_for_perception,
                                       render_mask, robot_geometry_ids)
from chess_sim.positions import PositionConfig
from chess_sim.vision_geometry import CameraCalibration

UNKNOWN_SQUARE = SquareEvidence(0, "unknown", 0.0)
OBSERVATION_SHOULDER_PAN = -1.0


def _park_clear(env, shoulder_pan: float = OBSERVATION_SHOULDER_PAN) -> None:
    """Put the arm off the board for a settled observation.

    The published rest pose reaches into the board centre; leaving it there
    hides roughly a third of the squares.  This is a benchmark observation
    pose, not a changed simulator default or a hardware claim.
    """
    start = env.data.qpos[env._joint_qpos].copy()
    start[0] = shoulder_pan
    env.data.qpos[env._joint_qpos] = start
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)
    for _ in range(40):
        env.apply_action(start)


def _patched_clear_home(env, shoulder_pan: float = OBSERVATION_SHOULDER_PAN) -> None:
    """Make every ``reset`` park clear before pieces are placed."""
    home = env.home_pose()
    home[0] = shoulder_pan
    env.home_pose = lambda: home.copy()


def _prediction_evidence(predictions) -> list[SquareEvidence]:
    return [SquareEvidence(square=p.square, label=p.label,
                           confidence=p.confidence,
                           probabilities=tuple(p.probabilities))
            for p in predictions]


def _observe(env, board: chess.Board, calibration, registration, model,
             reference_frame, *, wrist_calibration=None,
             wrist_reference=None, wrist_occlusion=None):
    env.reset(board)
    frame = env.render(Camera.TOP).copy()
    top = _prediction_evidence(predict_board(
        frame, calibration, registration, model, reference_frame=reference_frame,
        recover_empty=True))
    if wrist_calibration is None:
        return top
    wrist_frame = env.render(Camera.WRIST).copy()
    wrist = _prediction_evidence(predict_board(
        wrist_frame, wrist_calibration, registration, model, view="wrist",
        reference_frame=wrist_reference, recover_empty=True))
    _, valid = board_patches(wrist_frame, wrist_calibration, registration)
    cleaned = []
    for item in wrist:
        square = item.square
        if valid[square] < 0.55 or (
                wrist_occlusion is not None and wrist_occlusion[square] > 0.25):
            cleaned.append(SquareEvidence(square, "unknown", 0.0))
        else:
            cleaned.append(item)
    return merge_observations(top, cleaned)


def _random_position(rng: random.Random, *, extra: int | None = None):
    config = PositionConfig(min_extra_pieces=2, max_extra_pieces=8)
    if extra is not None:
        config = PositionConfig(min_extra_pieces=extra, max_extra_pieces=extra)
    return config.sample(rng)


def _quiet_move(board: chess.Board, rng: random.Random) -> chess.Move | None:
    moves = [m for m in board.legal_moves
             if not board.is_capture(m) and not board.is_castling(m)
             and not board.is_en_passant(m) and m.promotion is None]
    return rng.choice(moves) if moves else None


def _capture_move(board: chess.Board, rng: random.Random) -> chess.Move | None:
    moves = [m for m in board.legal_moves
             if board.is_capture(m) and not board.is_en_passant(m)
             and m.promotion is None]
    return rng.choice(moves) if moves else None


def _illegal_change(board: chess.Board, rng: random.Random) -> chess.Board | None:
    """Move one piece to an empty square that no legal move reproduces."""
    sources = [s for s, p in board.piece_map().items() if p.piece_type != chess.KING]
    if not sources:
        return None
    legal_fens = set()
    for move in board.legal_moves:
        successor = board.copy()
        successor.push(move)
        legal_fens.add(successor.fen())
    for _ in range(64):
        source = rng.choice(sources)
        targets = [s for s in chess.SQUARES
                   if s not in board.piece_map() and s != source]
        target = rng.choice(targets)
        changed = board.copy()
        piece = changed.remove_piece_at(source)
        changed.set_piece_at(target, piece)
        if changed.fen() not in legal_fens:
            return changed
    return None


def _cases(rng: random.Random) -> list[dict]:
    cases: list[dict] = []

    def add(category: str, previous: chess.Board, successor: chess.Board,
            expected_status: str, expected_move: chess.Move | None = None,
            note: str = "") -> None:
        cases.append({"category": category, "previous": previous,
                      "successor": successor,
                      "expected_status": expected_status,
                      "expected_move": expected_move, "note": note})

    for _ in range(3):
        board = _random_position(rng)
        move = _quiet_move(board, rng)
        if move is not None:
            successor = board.copy()
            successor.push(move)
            add("simple", board, successor, "confirmed", move)

    for _ in range(2):
        board = _random_position(rng)
        move = _capture_move(board, rng)
        if move is not None:
            successor = board.copy()
            successor.push(move)
            add("capture", board, successor, "confirmed", move)

    castle = chess.Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    add("castle", castle, _pushed(castle, "e1g1"), "confirmed",
        chess.Move.from_uci("e1g1"))

    ep = chess.Board("4k3/8/8/4Pp2/8/8/8/4K3 w - f6 0 1")
    add("en_passant", ep, _pushed(ep, "e5f6"), "confirmed",
        chess.Move.from_uci("e5f6"))

    promotion = chess.Board("k7/5P2/8/8/8/8/8/4K3 w - - 0 1")
    add("promotion", promotion, _pushed(promotion, "f7f8q"), "ambiguous",
        chess.Move.from_uci("f7f8q"), "piece type is not observable")

    unchanged = _random_position(rng)
    add("unchanged", unchanged, unchanged.copy(), "unchanged")

    for _ in range(2):
        board = _random_position(rng)
        move = _quiet_move(board, rng)
        if move is None:
            continue
        successor = board.copy()
        successor.push(move)
        add("occluded", board, successor, "insufficient", move,
            f"hide {chess.square_name(move.from_square)}")

    for _ in range(2):
        board = _random_position(rng)
        illegal = _illegal_change(board, rng)
        if illegal is not None:
            add("unexplained", board, illegal, "unexplained")

    return cases


def _pushed(board: chess.Board, uci: str) -> chess.Board:
    successor = board.copy()
    successor.push(chess.Move.from_uci(uci))
    return successor


def _summarise(records: list[dict]) -> dict:
    total = len(records)
    accepted = [r for r in records if r["status"] in ("confirmed", "unchanged")]
    correct = [r for r in accepted if r["correct"]]
    false = [r for r in accepted if not r["correct"]]
    abstained = [r for r in records if r["status"] not in ("confirmed", "unchanged")]
    expected_abstain = [r for r in records
                        if r["expected_status"] in ("ambiguous", "insufficient",
                                                    "unexplained")]
    return {
        "n": total,
        "accepted": len(accepted),
        "accepted_correct": len(correct),
        "false_confirmations": len(false),
        "coverage": round(len(accepted) / total, 4) if total else 0.0,
        "accepted_precision": round(len(correct) / len(accepted), 4) if accepted else None,
        "false_confirmation_rate": round(len(false) / total, 4) if total else 0.0,
        "abstained": len(abstained),
        "abstention_rate": round(len(abstained) / total, 4) if total else 0.0,
        "status_correct": sum(1 for r in records if r["status_correct"]),
        "status_accuracy": round(sum(1 for r in records if r["status_correct"]) / total, 4)
        if total else 0.0,
        "expected_abstain_n": len(expected_abstain),
        "expected_abstain_honoured": sum(
            1 for r in expected_abstain
            if r["status"] not in ("confirmed", "unchanged")),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sessions", type=int, default=3)
    parser.add_argument("--seed", type=int, default=7001)
    parser.add_argument("--views", choices=("top", "fused"), default="fused")
    parser.add_argument("--model", default="chess_sim/assets/perception/model.json")
    parser.add_argument("--out", default=".private/move-inference/heldout.json")
    parser.add_argument("--min-support", type=float, default=0.5)
    parser.add_argument("--deviation-confidence", type=float, default=0.5)
    args = parser.parse_args()
    if args.sessions < 1:
        parser.error("sessions must be positive")

    model = PerceptionModel.load(args.model)
    started = time.time()
    records: list[dict] = []
    sessions_report: list[dict] = []
    rng = random.Random(args.seed)
    skipped = 0

    for session_index in range(args.sessions):
        scene = make_session(args.seed * 1000 + session_index, random_camera=False)
        env = scene.env
        try:
            arm_ids = robot_geometry_ids(env)
            _patched_clear_home(env)
            env.reset("8/8/8/8/8/8/8/8 w - - 0 1")
            reference_frame = env.render(Camera.TOP).copy()
            calibration = CameraCalibration.from_mujoco(
                env.model, env.data, str(Camera.TOP),
                reference_frame.shape[1], reference_frame.shape[0])
            registration = register_for_perception(
                reference_frame, calibration, env.board.top, env.board.square)
            if registration is None:
                skipped += 1
                sessions_report.append({"session": session_index,
                                        "registered": False})
                continue
            wrist_calibration = wrist_reference = wrist_occlusion = None
            if args.views == "fused":
                wrist_reference = env.render(Camera.WRIST).copy()
                wrist_calibration = CameraCalibration.from_mujoco(
                    env.model, env.data, str(Camera.WRIST),
                    wrist_reference.shape[1], wrist_reference.shape[0])
                wrist_occlusion = square_mask_fraction(
                    render_mask(env, Camera.WRIST, arm_ids),
                    wrist_calibration, registration)

            session_rows = []
            for case_index, case in enumerate(_cases(rng)):
                observed = _observe(
                    env, case["successor"], calibration, registration, model,
                    reference_frame, wrist_calibration=wrist_calibration,
                    wrist_reference=wrist_reference,
                    wrist_occlusion=wrist_occlusion)
                if case["category"] == "occluded":
                    source = case["expected_move"].from_square
                    observed[source] = SquareEvidence(source, "unknown", 0.0)
                result = infer_move(
                    case["previous"], observed,
                    min_support=args.min_support,
                    deviation_confidence=args.deviation_confidence)
                expected_move = case["expected_move"]
                expected_uci = expected_move.uci() if expected_move else None
                inferred_uci = result.move.uci() if result.move else None
                if case["expected_status"] == "unchanged":
                    correct = (result.status == "unchanged")
                elif case["expected_status"] == "confirmed":
                    correct = (result.status == "confirmed"
                               and inferred_uci == expected_uci)
                elif case["expected_status"] == "ambiguous":
                    correct = (result.status == "ambiguous"
                               and (inferred_uci is None or inferred_uci == expected_uci))
                else:
                    correct = result.status == case["expected_status"]
                status_correct = result.status == case["expected_status"]
                session_rows.append({
                    "case": f"{case['category']}-{case_index}",
                    "category": case["category"],
                    "expected_status": case["expected_status"],
                    "expected_move": expected_uci,
                    "status": result.status,
                    "move": inferred_uci,
                    "correct": bool(correct),
                    "status_correct": bool(status_correct),
                    "confidence": result.confidence,
                    "reason": result.reason,
                    "answered": result.answered,
                    "unknown": result.unknown,
                    "deviations": list(result.deviations),
                    "promotion_required": result.promotion_required,
                    "note": case["note"],
                })
            records.extend(session_rows)
            sessions_report.append({
                "session": session_index, "seed": scene.seed,
                "board_variant": scene.board_variant,
                "registered": True,
                "cases": session_rows,
            })
        finally:
            scene.close()

    by_category: dict[str, list[dict]] = defaultdict(list)
    for record in records:
        by_category[record["category"]].append(record)
    report = {
        "schema": "robot-arm-move-inference-benchmark/v1",
        "seed": args.seed,
        "views": args.views,
        "model": args.model,
        "model_sha256": hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
        "min_support": args.min_support,
        "deviation_confidence": args.deviation_confidence,
        "scope": "fixed overhead rig, sparse positions, settled observations; "
                 "no physical capture and no move execution",
        "sessions": args.sessions,
        "sessions_registered": args.sessions - skipped,
        "sessions_skipped": skipped,
        "overall": _summarise(records),
        "by_category": {name: _summarise(rows)
                        for name, rows in sorted(by_category.items())},
        "session_report": sessions_report,
        "records": records,
        "elapsed_s": round(time.time() - started, 1),
    }
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ("records", "session_report")}, indent=2))
    print(f"receipt: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
