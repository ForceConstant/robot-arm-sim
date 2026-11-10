import chess

from chess_sim.config import EnvConfig, FailureReason
from chess_sim.env import ChessSimEnv, TaskResult
from chess_sim.tasks import CaptureTask, MoveTask
from chess_sim.transactions import execute_chess_transaction, plan_chess_transaction
from chess_sim.vision_geometry import Localization


def test_interrupted_compound_transaction_never_authorizes_game_commit():
    env = ChessSimEnv(EnvConfig())
    try:
        tasks = (CaptureTask(square="d5"),
                 MoveTask(from_square="e5", to_square="d6"))
        localization = Localization((0.0, 0.0), (320.0, 240.0), 0.99, 0.001)
        executed = []

        def execute_localized(task, observed, **kwargs):
            executed.append(task)
            return TaskResult(task=task, success=True, placement_error=0.0,
                              upright=True, disturbed=[], picked=task.source,
                              right_piece=True, steps=1)

        env.execute_localized = execute_localized
        transaction = env.execute_transaction(
            tasks,
            localize_source=lambda task, index: localization if index == 0 else None)

        assert executed == [tasks[0]]
        assert transaction.results[0].success
        assert not transaction.complete
        assert not transaction.commit_allowed
        assert transaction.failed_step == 1
        assert "uncertain" in transaction.reason
    finally:
        env.close()


def test_controller_abort_is_rejected_by_caller_game_commit_spy():
    env = ChessSimEnv(EnvConfig())
    try:
        board = chess.Board("4k3/8/8/3p4/4P3/8/8/4K3 w - - 0 1")
        move = chess.Move.from_uci("e4d5")
        original_fen = board.fen()
        localization = Localization((0.0, 0.0), (320.0, 240.0), 0.99, 0.001)
        attempts = []

        def aborted(task, observed, **kwargs):
            attempts.append(task)
            return TaskResult(task=task, success=False, placement_error=0.0,
                              upright=True, disturbed=[], picked=None,
                              right_piece=False, steps=1,
                              reason=FailureReason.CONTROLLER_ABORT,
                              waypoints_precise=False)

        env.execute_localized = aborted
        outcome = execute_chess_transaction(
            env, board, move, lambda task, index: localization)
        commit_events = []
        if outcome.commit_allowed:
            board.push(move)
            commit_events.append(move.uci())

        assert attempts and not outcome.commit_allowed
        assert not commit_events
        assert board.fen() == original_fen
        assert board.move_stack == []
    finally:
        env.close()


def test_move_planner_covers_capture_castling_en_passant_and_promotion_gate():
    capture_board = chess.Board("4k3/8/8/3p4/4P3/8/8/4K3 w - - 0 1")
    capture = plan_chess_transaction(capture_board,
                                     chess.Move.from_uci("e4d5"))
    assert capture.category == "capture"
    assert [task.family for task in capture.tasks] == ["capture", "move"]
    assert capture.tasks[0].source == "d5"

    castle_board = chess.Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    castle = plan_chess_transaction(castle_board, chess.Move.from_uci("e1g1"))
    assert castle.category == "castle"
    # Conventional order: the king takes its castling square first, then the
    # rook. The previous four-primitive staging plan failed on the destination
    # placement (measured, castle020 e1g1) and crossed the king's square.
    assert [(task.from_square, task.to_square) for task in castle.tasks] == [
        ("e1", "g1"), ("h1", "f1")]
    assert castle.staging_square is None
    assert castle.king_staging_square is None

    ep_board = chess.Board("8/8/8/3pP3/8/8/8/4K2k w - d6 0 1")
    en_passant = plan_chess_transaction(ep_board, chess.Move.from_uci("e5d6"))
    assert en_passant.category == "en_passant"
    assert en_passant.tasks[0].source == "d5"
    assert (en_passant.tasks[1].from_square,
            en_passant.tasks[1].to_square) == ("e5", "d6")

    promotion_board = chess.Board("k7/5P2/8/8/8/8/8/4K3 w - - 0 1")
    promotion = plan_chess_transaction(
        promotion_board, chess.Move.from_uci("f7f8q"))
    assert promotion.category == "promotion"
    assert promotion.requires_promotion_choice
    assert not promotion.can_commit_after_tasks
