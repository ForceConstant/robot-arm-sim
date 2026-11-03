"""Fixed single-piece manipulation probes; deliberately not chess-legal moves."""
import chess

CASES=(
    ('near-king-e','king','e1','e2'),
    ('near-king-d','king','d1','d2'),
    ('near-rook-left','rook','a1','h1'),
    ('near-rook-right','rook','h1','a1'),
    ('far-rook-left','rook','a8','h8'),
    ('far-rook-right','rook','h8','a8'),
    ('diagonal-rook-out','rook','a1','h8'),
    ('diagonal-rook-in','rook','h8','a1'),
)

def case_fen(kind,source,color=chess.WHITE):
    board=chess.Board(None)
    board.set_piece_at(chess.parse_square(source),chess.Piece(getattr(chess,kind.upper()),color))
    board.turn=color
    return board.fen()
