import importlib.util
from collections import Counter
from pathlib import Path

import chess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / ".private/phase10/capability-square-matrix.py"
SPEC = importlib.util.spec_from_file_location("capability_square_matrix", SCRIPT)
matrix = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(matrix)


def test_square_matrix_covers_every_piece_color_square_and_density():
    entries = matrix.build_entries()
    assert len(entries) == 6 * 2 * 64 * 2
    actor_counts = Counter((r["piece"], r["color"], r["crowding"]) for r in entries)
    assert len(actor_counts) == 24
    assert set(actor_counts.values()) == {64}
    density_counts = Counter(r["crowding"] for r in entries)
    assert density_counts == {"sparse": 768, "crowded": 768}

    for row in entries:
        board = chess.Board(row["fen"])
        source = chess.parse_square(row["square"])
        destination = chess.parse_square(row["destination"])
        assert board.piece_at(source) == chess.Piece(
            chess.PIECE_NAMES.index(row["piece"]),
            row["color"] == "white")
        assert board.piece_at(destination) is None
        assert len(board.piece_map()) == row["piece_count"]
        assert chess.square_distance(source, destination) == 1
        source_neighbors = set(matrix.neighbors(source))
        target_neighbors = set(matrix.neighbors(destination))
        occupied = set(board.piece_map()) - {source}
        if row["crowding"] == "sparse":
            assert not occupied.intersection(source_neighbors | target_neighbors)
        else:
            assert occupied.intersection(source_neighbors)
            assert occupied.intersection(target_neighbors)


def test_board_pairing_covers_each_square_as_source_and_destination_once():
    cycle = matrix.board_cycle()
    assert len(cycle) == len(set(cycle)) == 64
    assert all(chess.square_distance(cycle[i], cycle[(i + 1) % 64]) == 1
               for i in range(64))
