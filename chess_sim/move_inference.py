"""Infer the move/state change between a confirmed position and an observation.

The previous confirmed :class:`chess.Board` is the *only* source of move
history, castling rights and en-passant rights.  An observation supplies one
label per square (``empty``, ``white``, ``black`` or ``unknown``); no FEN, piece
type, simulator pose or move history is read from the image.

Every legal successor of the previous board is a hypothesis.  A hypothesis is
*verified* only when every square that it changes is observed with sufficient
confidence and no confident observation elsewhere contradicts it.  The caller
gets a move only when exactly one legal successor is verified and no other
legal move remains possible under the unobserved squares.  Otherwise the result
is an explicit abstention:

``ambiguous``
    Several legal moves explain the observation.  Identical pieces (or a
    promotion whose piece type is invisible) produce this outcome.
``insufficient``
    The changed squares are hidden or weakly observed, or a second move cannot
    be ruled out.  Take another view or ask for help.
``unexplained``
    Confident observations differ from the confirmed position and no legal move
    explains them (a knocked piece, a hand, a board shift, a bad observation).

This is inference, not control.  It never mutates the previous board and never
guesses a move.  The returned board for a confirmation is a copy of the
previous board with the inferred move pushed, so python-chess preserves the
castling/en-passant/half-move history that an image cannot recover.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

import chess
import numpy as np

LABELS: tuple[str, ...] = ("empty", "white", "black", "unknown")
LABEL_INDEX = {name: index for index, name in enumerate(LABELS)}
UNKNOWN = LABEL_INDEX["unknown"]
MAX_CANDIDATES = 12


@dataclass(frozen=True)
class SquareEvidence:
    """One square's observed label and confidence.

    ``probabilities``, when present, is the four-class distribution in
    :data:`LABELS` order.  A caller that only has labels can omit it; the
    resulting distribution is then the confidence on the stated label and the
    remainder spread over the other three classes.
    """

    square: int
    label: str
    confidence: float = 1.0
    probabilities: tuple[float, float, float, float] | None = None

    def __post_init__(self) -> None:
        if not 0 <= int(self.square) < 64:
            raise ValueError(f"square must be 0..63, got {self.square!r}")
        if self.label not in LABEL_INDEX:
            raise ValueError(f"unknown square label {self.label!r}")
        if not 0.0 <= float(self.confidence) <= 1.0:
            raise ValueError("confidence must be within [0, 1]")
        if self.probabilities is not None and len(self.probabilities) != len(LABELS):
            raise ValueError("probabilities must have four entries")


@dataclass(frozen=True)
class MoveCandidate:
    """One legal successor of the previous position."""

    move: chess.Move | None
    uci: str | None
    fen: str
    changed_squares: tuple[int, ...]
    missing_squares: tuple[int, ...]
    conflict_squares: tuple[int, ...]
    unexplained_squares: tuple[int, ...]
    verified: bool
    possible: bool
    support: float


@dataclass(frozen=True)
class MoveInference:
    """The inference outcome; callers must honour every non-confirmed status."""

    status: str
    board: chess.Board | None
    move: chess.Move | None
    confidence: float
    margin: float
    reason: str
    observed: tuple[str, ...]
    answered: int
    unknown: int
    deviations: tuple[int, ...]
    candidates: tuple[MoveCandidate, ...]
    promotion_required: bool = False


def expected_labels(board: chess.Board) -> np.ndarray:
    """Per-square class indices for a position: empty, white or black."""
    labels = np.zeros(64, dtype=np.int64)
    for square, piece in board.piece_map().items():
        labels[int(square)] = 1 if piece.color == chess.WHITE else 2
    return labels


def evidence_from_labels(labels: Sequence[str]) -> list[SquareEvidence]:
    """Turn 64 labels into full-confidence evidence (``unknown`` allowed)."""
    if len(labels) != 64:
        raise ValueError(f"expected 64 labels, got {len(labels)}")
    return [SquareEvidence(square=index, label=str(label))
            for index, label in enumerate(labels)]


def _as_evidence(observations: Iterable[SquareEvidence] | Sequence[str]
                 | Mapping[int, str]) -> list[SquareEvidence]:
    if isinstance(observations, Mapping):
        return [SquareEvidence(square=int(square), label=str(label))
                for square, label in observations.items()]
    items = list(observations)
    if not items:
        return []
    if isinstance(items[0], str):
        return evidence_from_labels([str(item) for item in items])
    result: list[SquareEvidence] = []
    for item in items:
        if isinstance(item, SquareEvidence):
            result.append(item)
            continue
        # Duck-typed perception objects: SquarePrediction and friends.
        square = getattr(item, "square")
        label = getattr(item, "label")
        confidence = float(getattr(item, "confidence", 1.0))
        probabilities = getattr(item, "probabilities", None)
        if probabilities is not None:
            probabilities = tuple(float(value) for value in probabilities)
        result.append(SquareEvidence(square=int(square), label=str(label),
                                     confidence=confidence,
                                     probabilities=probabilities))
    return result


def _arrays(evidence: Sequence[SquareEvidence]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    labels = np.full(64, UNKNOWN, dtype=np.int64)
    confidence = np.zeros(64, dtype=float)
    probabilities = np.full((64, len(LABELS)), np.nan, dtype=float)
    for item in evidence:
        square = int(item.square)
        labels[square] = LABEL_INDEX[item.label]
        confidence[square] = float(item.confidence)
        if item.probabilities is not None:
            probabilities[square] = np.asarray(item.probabilities, dtype=float)
    for square in range(64):
        if np.isnan(probabilities[square, 0]):
            value = confidence[square] if labels[square] != UNKNOWN else 0.0
            row = np.full(len(LABELS), (1.0 - value) / (len(LABELS) - 1))
            row[labels[square]] = value
            probabilities[square] = np.clip(row, 0.0, 1.0)
        else:
            row = np.clip(probabilities[square], 0.0, None)
            total = float(row.sum())
            probabilities[square] = row / total if total > 0 else 1.0 / len(LABELS)
    return labels, confidence, probabilities


def merge_observations(*observations: Iterable[SquareEvidence] | Sequence[str]
                       | Mapping[int, str]) -> list[SquareEvidence]:
    """Fuse views square-by-square.

    A square known in one view and unknown in another keeps the known label.
    Two views that confidently disagree become ``unknown`` instead of picking
    a winner.  This is how the overhead and wrist views are combined without
    letting either one override the other.
    """
    per_view = [_as_evidence(view) for view in observations]
    merged: list[SquareEvidence] = []
    for square in range(64):
        known: list[SquareEvidence] = []
        for view in per_view:
            for item in view:
                if int(item.square) == square and item.label != "unknown":
                    known.append(item)
                    break
        if not known:
            merged.append(SquareEvidence(square=square, label="unknown",
                                         confidence=0.0))
            continue
        labels = {item.label for item in known}
        if len(labels) != 1:
            merged.append(SquareEvidence(square=square, label="unknown",
                                         confidence=0.0))
            continue
        merged.append(max(known, key=lambda item: item.confidence))
    return merged


def _candidate(previous: chess.Board, move: chess.Move | None,
               previous_labels: np.ndarray, labels: np.ndarray,
               confidence: np.ndarray, probabilities: np.ndarray,
               deviations: np.ndarray, *, min_support: float) -> MoveCandidate:
    if move is None:
        successor = previous
        changed = ()
    else:
        successor = previous.copy()
        successor.push(move)
        changed = tuple(int(s) for s in np.flatnonzero(
            expected_labels(successor) != previous_labels))
    changed_set = set(changed)
    expected = expected_labels(successor)
    missing: list[int] = []
    conflicts: list[int] = []
    for square in changed:
        if labels[square] == UNKNOWN or confidence[square] < min_support:
            missing.append(square)
        elif labels[square] != expected[square]:
            conflicts.append(square)
    unexplained = [int(s) for s in np.flatnonzero(deviations) if int(s) not in changed_set]
    observed_changed = [s for s in changed if s not in set(missing)]
    if observed_changed:
        support = float(min(
            probabilities[s, expected[s]] for s in observed_changed))
    else:
        support = 1.0 if not changed else 0.0
    verified = not missing and not conflicts and not unexplained
    possible = not conflicts and not unexplained
    move_uci = None if move is None else move.uci()
    return MoveCandidate(
        move=move,
        uci=move_uci,
        fen=successor.fen(),
        changed_squares=changed,
        missing_squares=tuple(missing),
        conflict_squares=tuple(conflicts),
        unexplained_squares=tuple(unexplained),
        verified=verified,
        possible=possible,
        support=support,
    )


def infer_move(previous: chess.Board,
               observations: Iterable[SquareEvidence] | Sequence[str]
               | Mapping[int, str], *,
               min_support: float = 0.5,
               deviation_confidence: float = 0.5,
               max_candidates: int = MAX_CANDIDATES) -> MoveInference:
    """Infer one settled observation against a confirmed position.

    ``previous`` must be the last *confirmed* position with its full move
    history; the observation is compared with every legal successor.  The
    function never raises for a chess-rule reason and never guesses: read
    ``status`` before ``move``/``board``.
    """
    if not 0.0 < min_support <= 1.0:
        raise ValueError("min_support must be in (0, 1]")
    if not 0.0 <= deviation_confidence <= 1.0:
        raise ValueError("deviation_confidence must be within [0, 1]")
    evidence = _as_evidence(observations)
    labels, confidence, probabilities = _arrays(evidence)
    previous_labels = expected_labels(previous)
    answered = int(np.sum(labels != UNKNOWN))
    unknown = 64 - answered
    deviations = np.zeros(64, dtype=bool)
    for square in range(64):
        deviations[square] = (
            labels[square] != UNKNOWN
            and confidence[square] >= deviation_confidence
            and labels[square] != previous_labels[square]
        )

    candidates: list[MoveCandidate] = [
        _candidate(previous, move, previous_labels, labels, confidence,
                   probabilities, deviations, min_support=min_support)
        for move in previous.legal_moves
    ]
    unchanged = _candidate(previous, None, previous_labels, labels, confidence,
                           probabilities, deviations, min_support=min_support)
    candidates.append(unchanged)
    candidates.sort(key=lambda c: (c.verified, c.support, c.uci or ""),
                    reverse=True)

    verified = [c for c in candidates if c.verified and c.support >= min_support]
    possible = [c for c in candidates if c.possible]
    deviation_squares = tuple(int(s) for s in np.flatnonzero(deviations))
    observed_labels = tuple(LABELS[index] for index in labels)
    top = candidates[:max_candidates]

    def abstain(status: str, reason: str, confidence_value: float = 0.0,
                promotion: bool = False) -> MoveInference:
        return MoveInference(
            status=status, board=None, move=None,
            confidence=round(float(confidence_value), 4), margin=0.0,
            reason=reason, observed=observed_labels, answered=answered,
            unknown=unknown, deviations=deviation_squares,
            candidates=tuple(top), promotion_required=promotion)

    if not verified:
        if possible:
            best = max(possible, key=lambda c: c.support)
            return abstain(
                "insufficient",
                "changed squares are hidden or weakly observed; take another view",
                best.support)
        if deviation_squares:
            return abstain(
                "unexplained",
                "confident squares differ from the confirmed position and no legal move explains them")
        return abstain("insufficient",
                       "no square was observed well enough to confirm the position")

    if len(verified) == 1:
        winner = verified[0]
        competitors = [c for c in possible if c is not winner]
        if competitors:
            best = max(competitors, key=lambda c: c.support)
            return abstain(
                "insufficient",
                "another legal move cannot be ruled out by the observed squares",
                min(winner.support, best.support))
        if winner.move is None:
            return MoveInference(
                status="unchanged", board=previous.copy(), move=None,
                confidence=round(float(min(
                    (probabilities[s, previous_labels[s]]
                     for s in range(64) if labels[s] != UNKNOWN), default=0.0)), 4),
                margin=round(float(winner.support), 4),
                reason="the settled observation matches the confirmed position",
                observed=observed_labels, answered=answered, unknown=unknown,
                deviations=deviation_squares, candidates=tuple(top))
        successor = previous.copy()
        successor.push(winner.move)
        runner_up = max((c.support for c in candidates if c is not winner),
                        default=0.0)
        return MoveInference(
            status="confirmed", board=successor, move=winner.move,
            confidence=round(float(winner.support), 4),
            margin=round(float(winner.support - runner_up), 4),
            reason="exactly one legal move is verified by the observation",
            observed=observed_labels, answered=answered, unknown=unknown,
            deviations=deviation_squares, candidates=tuple(top))

    # Several legal successors explain the observation.  A promotion whose four
    # piece choices are visually identical is the common, useful case; report it
    # explicitly so the caller asks which piece instead of guessing a queen.
    uci = {c.uci for c in verified}
    promotion_only = (
        len(verified) == 4
        and all(c.move is not None and c.move.promotion for c in verified)
        and len({(c.move.from_square, c.move.to_square) for c in verified}) == 1
    )
    same_state = len({c.fen for c in verified}) == 1
    best = verified[0]
    second = verified[1]
    reason = ("several legal moves explain the observation; ask for the move"
              if not same_state else
              "the resulting position is identical but the move identity is ambiguous")
    if promotion_only:
        reason = "promotion piece type is not observable; ask which piece"
    return abstain("ambiguous", reason,
                   min(best.support, second.support), promotion_only)
