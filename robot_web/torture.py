"""Versioned, independent manipulation cases, not chess-engine decisions."""
from dataclasses import dataclass
import chess

SUITE_VERSION = 'crowded-v1'

@dataclass(frozen=True)
class Case:
    id: str
    arm: str
    fen: str
    move: str
    description: str


def position(moves=()):
    board = chess.Board()
    for uci in moves:
        board.push_uci(uci)
    return board.fen()


CASES = [
    Case('white-pawn', 'white', position(), 'd2d4', 'Pawn out of crowded home rank'),
    Case('black-pawn', 'black', position(), 'd7d5', 'Mirrored pawn extraction'),
    Case('white-knight', 'white', position(), 'b1c3', 'Knight between rook and bishop'),
    Case('black-knight', 'black', position(), 'g8f6', 'Mirrored crowded knight'),
    Case('white-bishop', 'white', position(['d2d4','d7d5','c2c4','e7e6','g1f3','g8f6','g2g3','d5c4']),
         'f1g2', 'Bishop case from observed failure; freshly settled layout'),
    Case('black-bishop', 'black', position(['d2d4','d7d5','c2c4','e7e6','b1c3','c7c6','e2e4','d5e4','c3e4']),
         'f8b4', 'Bishop case from observed failure; freshly settled layout'),
    Case('white-rook', 'white', position(), 'a1a3', 'Corner rook extraction beside pawn/knight'),
    Case('black-rook', 'black', position(), 'h8h6', 'Mirrored corner rook'),
    Case('white-queen', 'white', position(), 'd1d3', 'Tall queen extraction beside king/bishop'),
    Case('black-queen', 'black', position(), 'd8d6', 'Mirrored tall queen'),
    Case('white-king', 'white', position(), 'e1e3', 'King clearance and release'),
    Case('black-king', 'black', position(), 'e8e6', 'Mirrored king clearance and release'),
]
# Rook/queen/king moves intentionally ignore chess legality: physical tasks only.
