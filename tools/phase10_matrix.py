"""Deterministic legal chess transaction stimuli for Phase 10."""
from __future__ import annotations

import random

import chess


PIECE_TYPES = (chess.PAWN, chess.ROOK, chess.KNIGHT,
               chess.BISHOP, chess.QUEEN, chess.KING)


def _find_game_action(seed: int, color: chess.Color, piece_type: chess.PieceType,
                      capture: bool) -> tuple[str, str, str]:
    """Find one reproducible legal quiet/capture move for a type and color."""
    rng = random.Random(seed)
    minimum_ply = rng.randrange(2, 25)
    for restart in range(48):
        board = chess.Board()
        for ply in range(160):
            matching = [move for move in board.legal_moves
                        if board.turn == color
                        and board.piece_type_at(move.from_square) == piece_type
                        and board.is_capture(move) == capture
                        and not board.is_castling(move)
                        and not board.is_en_passant(move)
                        and move.promotion is None]
            if matching and ply >= minimum_ply:
                move = rng.choice(matching)
                return ("capture" if capture else "quiet", board.fen(), move.uci())
            # Keep every generated position representable by the physical set.
            # A random playout that promotes can leave extra queens/knights on
            # the board even though the sampled final move is non-promotion.
            legal = [move for move in board.legal_moves if move.promotion is None]
            if not legal:
                break
            board.push(rng.choice(legal))
    raise RuntimeError(f"could not construct {'capture' if capture else 'quiet'} "
                       f"move for {chess.COLOR_NAMES[color]} {chess.piece_name(piece_type)}")


def _castle_scenario(color: chess.Color, king_side: bool,
                     variant: int, seed: int = 0) -> tuple[str, str, str]:
    board = chess.Board(None)
    home_rank = 0 if color == chess.WHITE else 7
    enemy_rank = 7 - home_rank
    king_square = chess.square(4, home_rank)
    rook_file = 7 if king_side else 0
    rook_square = chess.square(rook_file, home_rank)
    board.set_piece_at(king_square, chess.Piece(chess.KING, color))
    board.set_piece_at(rook_square, chess.Piece(chess.ROOK, color))
    board.set_piece_at(chess.square(4, enemy_rank),
                       chess.Piece(chess.KING, not color))
    board.turn = color
    board.castling_rights = chess.BB_SQUARES[rook_square]

    # Independently vary sparse blockers away from the castling corridor.
    # Keep them on pawn home ranks so all 24 positions remain legal.
    rng = random.Random(seed * 1000003 + 0xCA57_1E + variant * 97 + int(color) * 13 + int(king_side))
    for fill_color, rank in ((chess.WHITE, 1), (chess.BLACK, 6)):
        files = [file for file in range(8)
                 if chess.square(file, rank) not in board.piece_map()]
        rng.shuffle(files)
        for file in files[:rng.randint(2, 6)]:
            board.set_piece_at(chess.square(file, rank),
                               chess.Piece(chess.PAWN, fill_color))

    destination = chess.square(6 if king_side else 2, home_rank)
    move = chess.Move(king_square, destination)
    if move not in board.legal_moves:
        raise RuntimeError(f"generated illegal castle scenario: {board.fen()} {move.uci()}")
    return "castle", board.fen(), move.uci()


def _en_passant_scenario(color: chess.Color, variant: int, seed: int = 0) -> tuple[str, str, str]:
    board = chess.Board(None)
    board.set_piece_at(chess.A1, chess.Piece(chess.KING, chess.WHITE))
    board.set_piece_at(chess.H8, chess.Piece(chess.KING, chess.BLACK))
    actor_file = 2 + (variant % 4)  # c..f, leaving room for both kings
    victim_file = actor_file - 1 if variant % 2 == 0 else actor_file + 1
    if color == chess.WHITE:
        actor = chess.square(actor_file, 4)   # white pawn on fifth rank
        victim = chess.square(victim_file, 4) # black pawn just double-stepped
        target = chess.square(victim_file, 5)
    else:
        actor = chess.square(actor_file, 3)   # black pawn on fourth rank
        victim = chess.square(victim_file, 3) # white pawn just double-stepped
        target = chess.square(victim_file, 2)
    board.set_piece_at(actor, chess.Piece(chess.PAWN, color))
    board.set_piece_at(victim, chess.Piece(chess.PAWN, not color))
    # Home-rank pawns randomize the surroundings without promoted inventory.
    rng = random.Random(seed * 1000003 + variant * 97 + int(color) + 0xEFA55)
    for fill_color, rank in ((chess.WHITE, 1), (chess.BLACK, 6)):
        files = list(range(8))
        # The victim's just-vacated origin must stay empty.
        files.remove(victim_file) if fill_color != color else None
        rng.shuffle(files)
        for file in files[:rng.randint(1, 5)]:
            board.set_piece_at(chess.square(file, rank), chess.Piece(chess.PAWN, fill_color))
    board.turn = color
    board.ep_square = target
    # Demonstrate that the EP right can arise from an actual legal double push.
    previous = board.copy()
    previous.ep_square = None
    previous.turn = not color
    previous.remove_piece_at(victim)
    origin = chess.square(victim_file, 6 if color == chess.WHITE else 1)
    previous.set_piece_at(origin, chess.Piece(chess.PAWN, not color))
    double = chess.Move(origin, victim)
    if not previous.is_valid() or double not in previous.legal_moves:
        raise RuntimeError("invalid en-passant predecessor")
    move = chess.Move(actor, target)
    if move not in board.legal_moves or not board.is_en_passant(move):
        raise RuntimeError(f"generated illegal en-passant scenario: {board.fen()} {move.uci()}")
    return "en_passant", board.fen(), move.uci()


def build_phase10_scenarios(seed: int) -> list[tuple[str, str, str]]:
    """Return 300 deterministic legal transactions with explicit marginal coverage.

    180 quiet moves (15 per color/type), 72 captures (6 per color/type),
    24 castlings (both colors/wings) and 24 en-passant moves (both colors).
    The result is shuffled deterministically so category is not correlated with
    case index, board perturbation or execution shard.
    """
    rows: list[tuple[str, str, str]] = []
    stream = 0
    for capture, repeats in ((False, 15), (True, 6)):
        for color in (chess.WHITE, chess.BLACK):
            for piece_type in PIECE_TYPES:
                for _ in range(repeats):
                    rows.append(_find_game_action(seed + stream * 7919,
                                                  color, piece_type, capture))
                    stream += 1
    for color in (chess.WHITE, chess.BLACK):
        for king_side in (True, False):
            for variant in range(6):
                rows.append(_castle_scenario(color, king_side, variant, seed))
                stream += 1
    for color in (chess.WHITE, chess.BLACK):
        for variant in range(12):
            rows.append(_en_passant_scenario(color, variant, seed))
            stream += 1
    # Deduplicate chess stimuli, not FEN clocks. Initial quiet pawn/knight moves
    # used to repeat, making deduped holdouts lose those entire actor buckets.
    seen = set()
    for index, row in enumerate(rows):
        attempt = 0
        while scenario_key(row) in seen:
            attempt += 1
            category, fen, uci = row
            board = chess.Board(fen)
            actor = board.piece_at(chess.Move.from_uci(uci).from_square)
            retry_seed = seed + 10000019 * (index + 1) + attempt * 7919
            if category in ("quiet", "capture"):
                row = _find_game_action(retry_seed, actor.color, actor.piece_type,
                                        category == "capture")
            elif category == "castle":
                row = _castle_scenario(actor.color, board.is_kingside_castling(
                    chess.Move.from_uci(uci)), index + attempt, retry_seed)
            else:
                row = _en_passant_scenario(actor.color, index + attempt, retry_seed)
            if attempt > 1000:
                raise RuntimeError("cannot construct unique matrix")
        rows[index] = row
        seen.add(scenario_key(row))
    random.Random(seed ^ 0x10FA_5EED).shuffle(rows)
    if len(rows) != 300:
        raise AssertionError(f"Phase 10 matrix has {len(rows)} rows, expected 300")
    return rows


def scenario_key(row: tuple[str, str, str]) -> tuple[str, str, str]:
    """Identity ignores half/fullmove clocks but retains castling and EP rights."""
    category, fen, uci = row
    return category, " ".join(fen.split()[:4]), uci
