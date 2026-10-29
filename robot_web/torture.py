"""Versioned, independent manipulation cases, not chess-engine decisions."""
from dataclasses import dataclass
import chess

SUITE_VERSION = 'crowded-v2'

@dataclass(frozen=True)
class Case:
    id: str
    arm: str
    fen: str
    move: str
    description: str


def position(moves=(), turn=chess.WHITE):
    """Board after `moves`, with the arm's side to move.

    The turn matters even though these are physical tasks: python-chess reports
    `is_capture` True for an illegal move by the side not to move, which once
    turned every black case into a capture of its own destination square.
    """
    board = chess.Board()
    for uci in moves:
        board.push_uci(uci)
    board.turn = turn
    return board.fen()


CASES = [
    Case('white-pawn', 'white', position(), 'd2d4', 'Pawn out of crowded home rank'),
    Case('black-pawn', 'black', position(turn=chess.BLACK), 'd7d5', 'Mirrored pawn extraction'),
    Case('white-knight', 'white', position(), 'b1c3', 'Knight between rook and bishop'),
    Case('black-knight', 'black', position(turn=chess.BLACK), 'g8f6', 'Mirrored crowded knight'),
    Case('white-bishop', 'white', position(['d2d4','d7d5','c2c4','e7e6','g1f3','g8f6','g2g3','d5c4']),
         'f1g2', 'Bishop case from observed failure; freshly settled layout'),
    Case('black-bishop', 'black', position(['d2d4','d7d5','c2c4','e7e6','b1c3','c7c6','e2e4','d5e4','c3e4'],
                                          turn=chess.BLACK),
         'f8b4', 'Bishop case from observed failure; freshly settled layout'),
    Case('white-rook', 'white', position(), 'a1a3', 'Corner rook extraction beside pawn/knight'),
    Case('black-rook', 'black', position(turn=chess.BLACK), 'h8h6', 'Mirrored corner rook'),
    Case('white-queen', 'white', position(), 'd1d3', 'Tall queen extraction beside king/bishop'),
    Case('black-queen', 'black', position(turn=chess.BLACK), 'd8d6', 'Mirrored tall queen'),
    Case('white-king', 'white', position(), 'e1e3', 'King clearance and release'),
    Case('black-king', 'black', position(turn=chess.BLACK), 'e8e6', 'Mirrored king clearance and release'),
    # Board-wide reach regressions. These are independent physical placements:
    # source squares are staged by setup moves, while the task itself may be
    # intentionally non-legal chess so it can probe the arm's full workspace.
    Case('white-pawn-far', 'white', position(), 'd2h4',
         'Pawn transfer to the far-file corner of the white-side half'),
    Case('black-pawn-far', 'black', position(turn=chess.BLACK), 'd7a5',
         'Mirrored pawn transfer to the far-file corner of the black-side half'),
    Case('white-knight-far', 'white', position(), 'b1g6',
         'Knight transfer across the board to the far quadrant'),
    Case('black-knight-far', 'black', position(turn=chess.BLACK), 'g8b3',
         'Mirrored knight transfer across the board'),
    Case('white-bishop-far', 'white', position(), 'f1a6',
         'Bishop transfer to the far-side edge'),
    Case('black-bishop-far', 'black', position(turn=chess.BLACK), 'f8h3',
         'Mirrored bishop transfer to the far-side edge'),
    Case('white-rook-corner', 'white', position(['g1f3', 'a7a6', 'h1g1', 'h7h6']), 'a1h1',
         'Rook transfer across the white-side rank into the opposite corner'),
    Case('black-rook-corner', 'black', position(['e2e3', 'b8c6', 'g2g3', 'a8b8'],
                                               turn=chess.BLACK), 'h8a8',
         'Mirrored rook transfer across the black-side rank into the opposite corner'),
    Case('white-queen-far', 'white', position(), 'd1h5',
         'Queen transfer to the far-side outer files'),
    Case('black-queen-far', 'black', position(turn=chess.BLACK), 'd8a4',
         'Mirrored queen transfer to the far-side outer files'),
    Case('white-king-far', 'white', position(), 'e1h5',
         'King transfer across files to the white arm\'s far-file reach boundary'),
    Case('black-king-far', 'black', position(turn=chess.BLACK), 'e8a4',
         'Mirrored king transfer across files to the black arm\'s far-file boundary'),
]
# Rook/queen/king moves intentionally ignore chess legality: physical tasks only.
