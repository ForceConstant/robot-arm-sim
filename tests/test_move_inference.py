import chess
import pytest

from chess_sim.move_inference import (LABELS, MoveInference, SquareEvidence,
                                      evidence_from_labels, expected_labels,
                                      infer_move, merge_observations)


def labels(board: chess.Board) -> list[str]:
    names = {0: "empty", 1: "white", 2: "black"}
    return [names[int(value)] for value in expected_labels(board)]


def with_move(board: chess.Board, uci: str) -> chess.Board:
    successor = board.copy()
    successor.push(chess.Move.from_uci(uci))
    return successor


def test_expected_labels_are_python_chess_order():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    values = expected_labels(board)
    assert values[chess.E2] == 1
    assert values[chess.E8] == 2
    assert values[chess.A1] == 0
    assert len(values) == 64


def test_simple_move_is_confirmed_and_does_not_mutate_previous():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    before = board.fen()
    result = infer_move(board, labels(with_move(board, "e2e4")))
    assert result.status == "confirmed"
    assert result.move == chess.Move.from_uci("e2e4")
    assert result.board is not None
    assert result.board.piece_at(chess.E4) == chess.Piece(chess.PAWN, chess.WHITE)
    assert result.board.piece_at(chess.E2) is None
    assert result.board.move_stack[-1] == chess.Move.from_uci("e2e4")
    assert board.fen() == before
    assert result.confidence == 1.0


def test_no_change_is_unchanged():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    result = infer_move(board, labels(board))
    assert result.status == "unchanged"
    assert result.board is not None
    assert result.board.fen() == board.fen()
    assert result.move is None


def test_capture_is_confirmed():
    board = chess.Board("4k3/8/8/8/8/2p5/3P4/4K3 w - - 0 1")
    successor = with_move(board, "d2c3")
    result = infer_move(board, labels(successor))
    assert result.status == "confirmed"
    assert result.move == chess.Move.from_uci("d2c3")
    assert result.board is not None
    assert result.board.piece_at(chess.C3) == chess.Piece(chess.PAWN, chess.WHITE)


def test_castling_is_confirmed_and_updates_rights():
    board = chess.Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    result = infer_move(board, labels(with_move(board, "e1g1")))
    assert result.status == "confirmed"
    assert result.move == chess.Move.from_uci("e1g1")
    assert result.board is not None
    assert result.board.piece_at(chess.G1) == chess.Piece(chess.KING, chess.WHITE)
    assert result.board.piece_at(chess.F1) == chess.Piece(chess.ROOK, chess.WHITE)
    assert not result.board.has_kingside_castling_rights(chess.WHITE)
    assert result.board.has_queenside_castling_rights(chess.BLACK)


def test_en_passant_is_confirmed_and_removes_the_captured_pawn():
    board = chess.Board("4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1")
    successor = with_move(board, "e5d6")
    assert chess.Move.from_uci("e5d6") in board.legal_moves
    result = infer_move(board, labels(successor))
    assert result.status == "confirmed"
    assert result.move == chess.Move.from_uci("e5d6")
    assert result.board is not None
    assert result.board.piece_at(chess.D6) == chess.Piece(chess.PAWN, chess.WHITE)
    assert result.board.piece_at(chess.D5) is None
    assert len(result.board.piece_map()) == 3


def test_promotion_is_ambiguous_because_piece_type_is_invisible():
    board = chess.Board("4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    successor = with_move(board, "a7a8q")
    result = infer_move(board, labels(successor))
    assert result.status == "ambiguous"
    assert result.promotion_required
    assert result.move is None
    verifies = [candidate for candidate in result.candidates if candidate.verified]
    assert {candidate.uci for candidate in verifies} == {
        "a7a8q", "a7a8r", "a7a8b", "a7a8n"}
    assert "promotion" in result.reason


def test_illegal_board_change_is_unexplained():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    changed = board.copy()
    changed.remove_piece_at(chess.E2)
    changed.set_piece_at(chess.D4, chess.Piece(chess.PAWN, chess.WHITE))
    result = infer_move(board, labels(changed))
    assert result.status == "unexplained"
    assert result.board is None
    assert chess.D4 in result.deviations


def test_hidden_changed_square_abstains_instead_of_guessing():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    observed = labels(with_move(board, "e2e4"))
    observed[chess.E2] = "unknown"
    result = infer_move(board, observed)
    assert result.status == "insufficient"
    assert result.move is None


def test_weak_evidence_on_a_changed_square_abstains():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    observed = evidence_from_labels(labels(with_move(board, "e2e3")))
    observed[chess.E2] = SquareEvidence(chess.E2, "empty", confidence=0.2)
    observed[chess.E3] = SquareEvidence(chess.E3, "white", confidence=0.2)
    result = infer_move(board, observed, min_support=0.5)
    assert result.status == "insufficient"


def test_confident_conflict_outside_the_move_is_unexplained():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    observed = evidence_from_labels(labels(with_move(board, "e2e4")))
    observed[chess.A5] = SquareEvidence(chess.A5, "white", confidence=0.95)
    result = infer_move(board, observed)
    assert result.status == "unexplained"


def test_merge_observations_prefers_known_and_rejects_disagreement():
    top = evidence_from_labels(["empty"] * 64)
    top[chess.E4] = SquareEvidence(chess.E4, "unknown", confidence=0.0)
    wrist = evidence_from_labels(["unknown"] * 64)
    wrist[chess.E4] = SquareEvidence(chess.E4, "white", confidence=0.8)
    merged = merge_observations(top, wrist)
    assert merged[chess.E4].label == "white"
    assert merged[chess.A1].label == "empty"
    disagree = evidence_from_labels(["empty"] * 64)
    disagree[chess.E4] = SquareEvidence(chess.E4, "white", confidence=0.9)
    other = evidence_from_labels(["unknown"] * 64)
    other[chess.E4] = SquareEvidence(chess.E4, "black", confidence=0.9)
    fused = merge_observations(disagree, other)
    assert fused[chess.E4].label == "unknown"


def test_merge_resolves_an_occluded_changed_square():
    board = chess.Board("4k3/8/8/8/8/8/4P3/4K3 w - - 0 1")
    top = evidence_from_labels(labels(with_move(board, "e2e4")))
    top[chess.E2] = SquareEvidence(chess.E2, "unknown", confidence=0.0)
    wrist = evidence_from_labels(["unknown"] * 64)
    wrist[chess.E2] = SquareEvidence(chess.E2, "empty", confidence=0.9)
    result = infer_move(board, merge_observations(top, wrist))
    assert result.status == "confirmed"
    assert result.move == chess.Move.from_uci("e2e4")


def test_unknown_board_abstains():
    result = infer_move(chess.Board(), ["unknown"] * 64)
    assert result.status == "insufficient"
    assert result.answered == 0
    assert result.move is None


def test_candidate_list_is_bounded():
    board = chess.Board()
    result = infer_move(board, labels(board), max_candidates=3)
    assert len(result.candidates) <= 3


def test_invalid_evidence_is_rejected():
    with pytest.raises(ValueError):
        SquareEvidence(64, "empty")
    with pytest.raises(ValueError):
        SquareEvidence(0, "purple")
    with pytest.raises(ValueError):
        SquareEvidence(0, "empty", confidence=1.5)
    with pytest.raises(ValueError):
        evidence_from_labels(["empty"] * 63)


def test_labels_contract():
    assert LABELS == ("empty", "white", "black", "unknown")
    result: MoveInference = infer_move(chess.Board(), ["unknown"] * 64)
    assert len(result.observed) == 64
