"""Held-out wrist-only board registration and complete-turn benchmark.

Every operational input is a rendered wrist frame plus the joints that produced
it.  Simulator truth is used only to build cases, to score the estimate, and to
choose an occlusion subset for a deliberate rejection case.

    MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
    ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
    PYTHONPATH=. .venv/bin/python tools/benchmark_phase9.py --sessions 8 --turns 3
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import time
from collections import defaultdict
from pathlib import Path

import chess
import mujoco
import numpy as np

from chess_sim.config import Camera, RandomizationConfig
from chess_sim import gripper
from chess_sim.fiducials import MarkerLayout, detect_markers
from chess_sim.move_inference import SquareEvidence, infer_move
from chess_sim.perception import PerceptionModel, square_mask_fraction
from chess_sim.env import ChessSimEnv
from chess_sim.perception_data import render_mask, robot_geometry_ids
from chess_sim.product import layout_env_config, load_product_twin
from chess_sim.positions import PositionConfig
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.wrist_scan import ScanFrame, WristScanner, load_plan

START = chess.STARTING_FEN
MAX_SCANS = 3
REGISTRATION_MM = 2.0
REGISTRATION_DEG = 1.0


def _evidence(predictions) -> list[SquareEvidence]:
    return [SquareEvidence(p.square, p.label, p.confidence, tuple(p.probabilities))
            for p in predictions]


EMPTY = "8/8/8/8/8/8/8/8 w - - 0 1"


def _scene(seed: int, stress_shadow_extreme: bool = False):
    """Product-twin scene with randomized board placement and lighting.

    stress_shadow_extreme enables a cast-shadow-extreme / harsh-lighting envelope
    for targeted robustness characterization (same geometry/randomization).
    """
    twin = load_product_twin()
    draw = np.random.default_rng(seed)
    if stress_shadow_extreme:
        # Hard/directional shadows and low-fill/high-contrast lighting extremes.
        if int(draw.integers(0, 2)) == 0:
            light_intensity = float(draw.uniform(0.30, 0.55))
            fill_intensity = float(draw.uniform(0.20, 0.55))
            ambient = float(draw.uniform(0.0, 0.02))
        else:
            light_intensity = float(draw.uniform(1.25, 1.55))
            fill_intensity = float(draw.uniform(0.25, 0.60))
            ambient = float(draw.uniform(0.0, 0.03))
        light_dir = (float(draw.uniform(-0.55, -0.15)),
                     float(draw.uniform(0.10, 0.55)), -1.0)
        shadow_softness = float(draw.uniform(0.001, 0.010))
    else:
        light_intensity = float(draw.uniform(0.82, 1.15))
        ambient = float(draw.uniform(0.0, 0.05))
        light_dir = (float(draw.uniform(-0.35, -0.2)),
                     float(draw.uniform(0.2, 0.4)), -1.0)
        fill_intensity = float(draw.uniform(0.7, 1.3))
        shadow_softness = float(draw.uniform(0.015, 0.035))
    appearance = twin.appearance("printed").model_copy(update={
        "light_intensity": light_intensity,
        "ambient": ambient,
        "light_dir": light_dir,
        "fill_intensity": fill_intensity,
        "shadow_softness": shadow_softness,
    })
    config = layout_env_config("printed", cameras=(Camera.WRIST,)).model_copy(update={
        "appearance": appearance,
        "randomization": RandomizationConfig(board_shift=0.008, board_yaw=math.radians(2.5),
                                             board_side=0.008, arm_joint_jitter=0.0,
                                             camera=False),
    })
    env = ChessSimEnv(config)
    return env, arithmetic(seed)


def arithmetic(seed: int) -> random.Random:
    return random.Random(seed)


def _park(env, joints) -> None:
    command = np.asarray(joints, dtype=float).copy()
    command[5] = np.clip(command[5], gripper.JOINT_MIN, gripper.JOINT_MAX)
    env.data.qpos[env._joint_qpos] = command
    gripper.seed_coupled(env.model, env.data, "so101:")
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)


def _visible(env, registration, calibration) -> np.ndarray:
    """Evaluator-only: squares whose patch is not covered by the arm."""
    mask = render_mask(env, Camera.WRIST, robot_geometry_ids(env))
    covered = square_mask_fraction(mask, calibration, registration)
    return covered < 0.05


def _capture(env, scanner, poses) -> list[ScanFrame]:
    frames: list[ScanFrame] = []
    for pose in poses:
        _park(env, pose.joints)
        frames.append(ScanFrame(env.render(Camera.WRIST).copy(),
                                tuple(env.data.qpos[env._joint_qpos]), 0.0, pose.name))
    return frames


def _random_position(rng: random.Random, crowded: bool) -> chess.Board:
    if crowded:
        board = chess.Board(START)
        for _ in range(rng.randrange(0, 6)):
            moves = list(board.legal_moves)
            board.push(rng.choice(moves))
        return board
    return PositionConfig(min_extra_pieces=2, max_extra_pieces=8).sample(rng)


def _pick_move(board: chess.Board, rng: random.Random) -> chess.Move:
    moves = list(board.legal_moves)
    return rng.choice(moves)


def _illegal(board: chess.Board, rng: random.Random) -> chess.Board | None:
    sources = [s for s, p in board.piece_map().items() if p.piece_type != chess.KING]
    # Compare piece placement only: full FENs also carry the move clocks, so a
    # rearranged board that happens to equal a legal successor would otherwise
    # be accepted as "illegal" and then correctly inferred as that move.
    legal = set()
    for move in board.legal_moves:
        copy = board.copy()
        copy.push(move)
        legal.add(copy.board_fen())
    for _ in range(64):
        source = rng.choice(sources)
        targets = [s for s in chess.SQUARES if s not in board.piece_map()]
        if not targets:
            return None
        changed = board.copy()
        piece = changed.remove_piece_at(source)
        changed.set_piece_at(rng.choice(targets), piece)
        if changed.board_fen() not in legal:
            return changed
    return None


def _category(board: chess.Board, move: chess.Move) -> str:
    if board.is_castling(move):
        return "castle"
    if board.is_en_passant(move):
        return "en_passant"
    if move.promotion:
        return "promotion"
    if board.is_capture(move):
        return "capture"
    return "quiet"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sessions", type=int, default=8)
    parser.add_argument("--turns", type=int, default=25)
    parser.add_argument("--seed", type=int, default=9300)
    parser.add_argument("--model", default="chess_sim/assets/perception/model_wrist.json")
    parser.add_argument("--calibration", default=".private/phase9/calibration.json")
    parser.add_argument("--plan", default=str(Path("chess_sim/assets/wrist_scan_plan.json")))
    parser.add_argument("--method", choices=("markers", "checker"), default="markers")
    parser.add_argument("--out", default=".private/phase9/heldout.json")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--crowded-only", action="store_true")
    parser.add_argument("--no-self-occlusion", action="store_true", help="paired unmasked ablation")
    parser.add_argument("--stress-shadow-extreme", action="store_true",
                        help="engineered cast-shadow-extreme/harsh lighting stress envelope")
    parser.add_argument("--stress-arm-over-changed", action="store_true",
                        help="engineered arm-over-changed-square stress (restrict views to those covering changed squares)")
    args = parser.parse_args()

    model = PerceptionModel.load(args.model)
    watch = WristCalibration.load(args.calibration)
    poses = load_plan(args.plan)
    rng = random.Random(args.seed)
    started = time.time()

    records: list[dict] = []
    sessions: list[dict] = []
    registration_rows: list[dict] = []

    for index in range(args.sessions):
        env, place_rng = _scene(args.seed * 1000 + index, stress_shadow_extreme=args.stress_shadow_extreme)
        crowded = args.crowded_only or index % 2 == 0
        try:
            # Place the board for this session, then freeze that pose as nominal.
            env.reset(EMPTY, rng=place_rng)
            env.config = env.config.model_copy(update={"board": env.board})
            scanner = WristScanner(
                watch, poses, model,
                square=env.board.square, board_z=env.board.top,
                marker_z=env.board.table_top + env.board.marker_above_table,
                marker_layout=MarkerLayout.for_board(env.board.field * 1000.0,
                                                     env.board.border * 1000.0,
                                                     env.board.marker_plane),
                method=args.method, self_occlusion=not args.no_self_occlusion)
            empty_frames = _capture(env, scanner, poses)
            references = {item.pose: item.frame for item in empty_frames}
            empty_registration = scanner.register(empty_frames)
            if empty_registration is None:
                sessions.append({"session": index, "registered": False})
                continue
            # Capture the per-pose empty references against the accepted pose.
            references = {item.pose: item.frame for item in empty_frames}
            truth_origin = env.board.origin
            truth_yaw = env.board.yaw

            state = _random_position(rng, crowded)
            env.reset(state)
            cases: list[dict] = []
            for _ in range(args.turns):
                move = _pick_move(state, rng)
                successor = state.copy()
                successor.push(move)
                cases.append({"kind": "move", "previous": state.copy(),
                              "successor": successor, "move": move,
                              "category": _category(state, move)})
                state = successor
            for _ in range(2):
                illegal = _illegal(state, rng)
                if illegal is not None:
                    cases.append({"kind": "invalid", "previous": state.copy(),
                                  "successor": illegal, "move": None,
                                  "category": "invalid"})
            for _ in range(2):
                move = _pick_move(state, rng)
                successor = state.copy()
                successor.push(move)
                cases.append({"kind": "occluded", "previous": state.copy(),
                              "successor": successor, "move": move,
                              "category": "occluded"})

            for case_index, case in enumerate(cases):
                env.reset(case["successor"])
                visible = {}
                need_visibility = (case["kind"] == "occluded") or (args.stress_arm_over_changed and case["move"] is not None)
                if need_visibility:
                    for pose in poses:
                        _park(env, pose.joints)
                        camera = scanner.cameras(pose.joints)
                        visible[pose.name] = _visible(env, empty_registration, camera)
                active = poses
                arm_over_changed = None
                if case["kind"] == "occluded":
                    square = case["move"].from_square
                    # Keep only poses that hide the changed square.
                    hidden = [p for p in poses if not visible[p.name][square]]
                    if len(hidden) < 1 or len(hidden) == len(poses):
                        continue
                    active = hidden
                elif args.stress_arm_over_changed and case["move"] is not None:
                    # Engineered arm-over-changed stress: prefer views where the arm
                    # (robot geometry) covers at least one changed square in the scan.
                    changed = {case["move"].from_square, case["move"].to_square}
                    covering = [p for p in poses if any(not visible[p.name][sq] for sq in changed)]
                    if covering:
                        active = covering
                    covered_count = sum(1 for p in active if any(not visible[p.name][sq] for sq in changed))
                    arm_over_changed = {
                        "changed_squares": sorted(changed),
                        "active_views": [p.name for p in active],
                        "covering_view_count": covered_count,
                        "any_view_covers_changed": bool(covering),
                    }
                evidence: list[SquareEvidence] = []
                registration = None
                used_scans = 0
                markers: tuple[int, ...] = ()
                for scan in range(MAX_SCANS):
                    used_scans = scan + 1
                    frames = _capture(env, scanner, active)
                    result = scanner.scan(frames, references, timestamp=float(case_index))
                    if result.registration is not None:
                        registration = result.registration
                        markers = tuple(sorted({d.marker_id for d in detect_markers(
                            scanner.calibration.rectify(frames[0].frame))}))
                    if result.predictions:
                        evidence = _evidence(result.predictions)
                        inference = infer_move(case["previous"], evidence)
                        # Rescan only when the observation was incomplete; a
                        # confident contradiction or a confirmation is final.
                        if inference.status != "insufficient":
                            break
                    else:
                        inference = None
                if registration is None:
                    # Abstaining because the board could not be located is a
                    # safe outcome: nothing is committed from an unregistered
                    # frame.  Only a confirmed/unchanged status can be unsafe.
                    records.append({"session": index, "case": case_index,
                                    "kind": case["kind"], "category": case["category"],
                                    "status": "no_registration",
                                    "correct": case["kind"] != "move",
                                    "safe": True, "scans": used_scans})
                    continue
                if case_index == 0:
                    registration_rows.append({
                        "session": index, "markers": markers,
                        "xy_error_mm": round(math.hypot(
                            registration.origin[0] - truth_origin[0],
                            registration.origin[1] - truth_origin[1]) * 1000, 4),
                        "yaw_error_deg": round(math.degrees(
                            (registration.yaw - truth_yaw + math.pi) % (2 * math.pi) - math.pi), 4),
                        "coverage": registration.coverage,
                    })
                inference = infer_move(case["previous"], evidence)
                accepted = inference.status in ("confirmed", "unchanged")
                if case["kind"] == "move":
                    if case["category"] == "promotion":
                        # The promoted piece type is not observable: asking is
                        # the correct outcome, not a confirmed promotion.
                        correct = inference.status in ("ambiguous", "insufficient")
                        safe = correct
                    else:
                        correct = (inference.status == "confirmed"
                                   and inference.move == case["move"])
                        safe = not accepted or correct
                else:
                    correct = not accepted
                    safe = not accepted
                rec = {
                    "session": index, "case": case_index, "kind": case["kind"],
                    "category": case["category"], "status": inference.status,
                    "expected": case["move"].uci() if case["move"] else None,
                    "got": inference.move.uci() if inference.move else None,
                    "correct": bool(correct), "safe": bool(safe),
                    "scans": used_scans, "answered": inference.answered,
                    "unknown": inference.unknown,
                    "unknown_squares": [p.square for p in result.predictions if p.label == "unknown"],
                }
                if arm_over_changed is not None:
                    rec["arm_over_changed"] = arm_over_changed
                records.append(rec)
            sessions.append({"session": index, "registered": True,
                             "crowded": crowded, "cases": len(cases)})
            if not args.quiet:
                print(f"session {index}: crowded={crowded} cases={len(cases)}", flush=True)
        finally:
            env.close()

    by_category: dict[str, list[dict]] = defaultdict(list)
    for record in records:
        by_category[record["category"]].append(record)
    moves = [r for r in records if r["kind"] == "move"]
    resolved = [r for r in moves if r["correct"]]
    false_confirm = [r for r in moves if r["status"] in ("confirmed", "unchanged") and not r["correct"]]
    rejection = [r for r in records if r["kind"] != "move"]
    summary = {
        "turns": len(moves),
        "resolved": len(resolved),
        "resolution_rate": round(len(resolved) / len(moves), 4) if moves else 0.0,
        "false_confirmations": len(false_confirm),
        "abstained": sum(1 for r in moves if r["status"] not in ("confirmed", "unchanged")),
        "scans_used": {str(k): sum(1 for r in moves if r["scans"] == k) for k in (1, 2, 3)},
        "rejection_cases": len(rejection),
        "rejection_safe": sum(1 for r in rejection if r.get("safe", r.get("status") not in ("confirmed", "unchanged"))),
        "registration_sessions": len(registration_rows),
        "registration_within_2mm_1deg": sum(
            1 for r in registration_rows
            if r["xy_error_mm"] <= REGISTRATION_MM and abs(r["yaw_error_deg"]) <= REGISTRATION_DEG),
    }
    stress_modes = {
        "shadow_extreme": args.stress_shadow_extreme,
        "arm_over_changed": args.stress_arm_over_changed,
    }
    report = {
        "schema": "robot-arm-phase9-wrist-only/v1",
        "seed": args.seed, "sessions": args.sessions, "turns_per_session": args.turns,
        "method": args.method, "views": len(poses),
        "model_sha256": hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
        "crowded_only": args.crowded_only,
        "self_occlusion": not args.no_self_occlusion,
        "stress_modes": stress_modes,
        "source_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in map(Path, [__file__, "chess_sim/wrist_scan.py",
                                             "chess_sim/self_occlusion.py", "chess_sim/wrist_calibration.py",
                                             args.calibration, args.plan])},
        "geometry_sources": scanner.self_model.sources if scanner.self_model is not None else {},
        "calibration": args.calibration,
        "scope": "wrist-only frames and measured joints; board pose from printed markers; "
                 "simulator truth used only for case construction and scoring",
        "summary": summary,
        "registration": registration_rows,
        "by_category": {name: {"n": len(rows),
                               "resolved": sum(1 for r in rows if r["correct"])}
                        for name, rows in sorted(by_category.items())},
        "sessions": sessions,
        "records": records,
        "elapsed_s": round(time.time() - started, 1),
    }
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    print("registration:", json.dumps(registration_rows, indent=2))
    print(f"receipt: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
