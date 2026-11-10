"""Map chess moves to physically executable primitive task sequences."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, TYPE_CHECKING

import chess

from .tasks import AnyTask, CaptureTask, MoveTask
from .vision_geometry import Localization

if TYPE_CHECKING:
    from .env import ChessSimEnv, TransactionResult


@dataclass(frozen=True)
class ChessTransactionPlan:
    move: chess.Move
    category: str
    tasks: tuple[AnyTask, ...]
    requires_promotion_choice: bool = False
    staging_square: str | None = None
    king_staging_square: str | None = None

    @property
    def can_commit_after_tasks(self) -> bool:
        """Promotions stay uncommitted until the replacement piece is chosen."""
        return not self.requires_promotion_choice


@dataclass(frozen=True)
class ChessExecutionResult:
    plan: ChessTransactionPlan
    physical: "TransactionResult"
    commit_allowed: bool
    reason: str | None = None


def plan_chess_transaction(board: chess.Board,
                           move: chess.Move) -> ChessTransactionPlan:
    """Build a safe physical task sequence for one legal standard-chess move.

    Captured material is cleared before its destination is occupied. Castling
    uses staging squares for piece clearance. En-passant identifies the
    off-destination victim.
    Promotion moves the pawn but deliberately leaves chess-state commit gated
    on the caller obtaining and physically placing the chosen replacement.
    """
    if move not in board.legal_moves:
        raise ValueError(f"illegal chess move: {move.uci()}")
    source = chess.square_name(move.from_square)
    target = chess.square_name(move.to_square)
    tasks: list[AnyTask] = []
    king_staging = None

    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        king_side = chess.square_file(move.to_square) > chess.square_file(move.from_square)
        rook_source = chess.square_name(chess.square(7 if king_side else 0, rank))
        rook_target = chess.square_name(chess.square(5 if king_side else 3, rank))
        if king_side:
            # Conventional order: king to its castling square, then rook.
            # Measured 2026-11-07 (castle020 e1g1): the previous four-primitive
            # staging plan (rook h1->h3, king e1->e2, rook h3->f1, king e2->g1)
            # failed even on the unmodified runtime, and the staged rook carry
            # crossed the king's square. The direct order completes both halves
            # in isolation (0.35 mm and 1.67 mm, precise).
            tasks.extend((MoveTask(from_square=source, to_square=target),
                          MoveTask(from_square=rook_source, to_square=rook_target)))
            staging = king_staging = None
        else:
            # Queenside still stages the rook: king e1->c1 and rook a1->d1
            # would otherwise cross through each other.
            candidates = ("b2", "a2", "c2", "b3", "a3", "c3", "b4", "a4", "c4")
            staging = next((sq for sq in candidates
                            if board.piece_at(chess.parse_square(sq)) is None
                            and sq not in (source, target, rook_source, rook_target)), None)
            if staging is None:
                raise ValueError("no empty square available to stage the castling rook")
            tasks.extend((MoveTask(from_square=rook_source, to_square=staging),
                          MoveTask(from_square=source, to_square=target),
                          MoveTask(from_square=staging, to_square=rook_target)))
        category = "castle"
    else:
        if board.is_en_passant(move):
            victim = chess.square_name(move.to_square - 8 if board.turn == chess.WHITE
                                       else move.to_square + 8)
            tasks.append(CaptureTask(square=victim))
            category = "en_passant"
        elif board.is_capture(move):
            tasks.append(CaptureTask(square=target))
            category = "capture"
        else:
            category = "promotion" if move.promotion else "quiet"
        tasks.append(MoveTask(from_square=source, to_square=target))

    return ChessTransactionPlan(move=move, category=category,
                                tasks=tuple(tasks),
                                requires_promotion_choice=bool(move.promotion),
                                staging_square=(staging if board.is_castling(move) else None),
                                king_staging_square=king_staging)


def execute_chess_transaction(
        env: "ChessSimEnv", board: chess.Board, move: chess.Move,
        localize_source: Callable[[AnyTask, int], Localization | None], **kwargs
) -> ChessExecutionResult:
    """Execute a planned chess move; expose commit only after every step.

    This is the caller-facing transaction boundary. The board argument is the
    pre-move game state and remains untouched; the caller may push ``move`` only
    when the returned ``commit_allowed`` is true. Promotion additionally waits
    for an explicit replacement-piece operation not yet performed here.
    """
    plan = plan_chess_transaction(board, move)
    physical = env.execute_transaction(plan.tasks, localize_source, **kwargs)
    commit_allowed = physical.commit_allowed and plan.can_commit_after_tasks
    reason = physical.reason
    if physical.commit_allowed and plan.requires_promotion_choice:
        reason = "promotion replacement choice and placement required"
    return ChessExecutionResult(plan, physical, commit_allowed, reason)
