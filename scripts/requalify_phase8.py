"""Re-run the Phase 8 matrix at the current mount without rewriting history.

Run once per ROBOT_GRIPPER (stock/cad_parallel), with private piece assets and
the matching CAD assets. No hardware, web-service or deployment changes.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import sys
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import chess
import mujoco
import numpy as np
from chess_sim import gripper, ik
from chess_sim.env import ChessSimEnv
from chess_sim.product import apply_mass_model, load_product_twin
from qualify_phase8 import (EMPTY_FEN, EXEC_TYPES, adjacent_target, git_commit,
                            scan, single_piece_fen, _strip_internal)
from robot_web.torture import CASES, SUITE_VERSION


def main():
    layout_path = ROOT / "hardware/current_layout.json"
    layout = json.loads(layout_path.read_text())
    twin = load_product_twin()
    mount = layout["mount"]
    cfg = twin.env_config(piece_set="printed", cameras=())
    cfg = cfg.model_copy(update={"board": cfg.board.model_copy(update={
        "arm_gap": mount["board_edge_to_base_frame_mm"] / 1000,
        "arm_riser": mount["base_mount_datum_above_table_mm"] / 1000})})
    assert np.allclose(np.array(cfg.board.arm_base) * 1000,
                       mount["white_arm_base_frame_mm"])
    report_path = ROOT / "reports" / f"phase8_updated_{gripper.KIND}.json"
    report = dict(
        schema="robot-arm-phase8-current-layout/1",
        generated_at=dt.datetime.now(dt.timezone.utc).isoformat(),
        git_commit=git_commit(), tool=gripper.description(),
        layout=layout, layout_sha256=hashlib.sha256(layout_path.read_bytes()).hexdigest(),
        mujoco=mujoco.__version__, position_cost=ik.POSITION_COST,
        tilt_cost=ik.TILT_COST, config=cfg.model_dump(mode="json"),
        piece_manifest_sha256=hashlib.sha256(
            (Path(os.environ["ROBOT_PIECE_ASSETS"]) / "set.json").read_bytes()).hexdigest(),
        mass_model="Historical Phase 8 nominal model: measured rook 4.8 g; other types unmeasured PLA. Both tools identical.",
        scope="Single-follower simulation; identical Phase 8 scans, mixed-type executions with king/rook retries, actual four tray destinations, plus crowded-v3. No physical qualification.",
        source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in [Path(__file__), ROOT / "scripts/qualify_phase8.py",
                                 ROOT / "chess_sim/controller.py", ROOT / "chess_sim/ik.py"]},
    )

    def save(stage):
        report["stage"] = stage
        report_path.write_text(json.dumps(_strip_internal(report), indent=2) + "\n")
        print(gripper.KIND, stage, flush=True)

    def prepare(env, fen, seed=0):
        env.reset(fen)
        apply_mass_model(env, twin, all_slots=True)
        env.reset(fen)
        env._noise_rng = np.random.default_rng(seed)

    def attempt(env, source, target):
        slot = env.slot_at(source)
        z0 = float(env.piece_position(slot)[2])
        max_z = z0
        def record(action):
            nonlocal max_z
            max_z = max(max_z, float(env.piece_position(slot)[2]))
        try:
            result = env.capture(source, on_step=record) if target is None else env.move(source, target, on_step=record)
            return dict(success=bool(result.success), reason=str(result.reason) if result.reason else None,
                        error_mm=round(float(result.placement_error) * 1000, 4),
                        upright=bool(result.upright), disturbed=[str(s) for s in result.disturbed],
                        precise=bool(result.waypoints_precise), picked=str(result.picked),
                        lift_mm=round((max_z-z0)*1000, 3), stall=getattr(env, "diagnostic_stall", None))
        except Exception as exc:
            return dict(success=False, reason=f"{type(exc).__name__}: {exc}",
                        lift_mm=round((max_z-z0)*1000, 3))

    env = ChessSimEnv(cfg)
    try:
        for label, crowded, fen in (("empty_board", False, EMPTY_FEN),
                                     ("crowded_start", True, chess.STARTING_FEN)):
            prepare(env, fen)
            report[label] = scan(env, crowded=crowded, twin=twin)
            save(f"{label} scan complete")
        rows = []
        report["execution"] = dict(rows=rows)
        for i, square in enumerate(chess.SQUARES):
            kind = EXEC_TYPES[i % len(EXEC_TYPES)]
            target = adjacent_target(square)
            prepare(env, single_piece_fen(square, kind))
            first = attempt(env, square, target)
            retries = []
            if not first["success"]:
                for retry_kind in (chess.KING, chess.ROOK):
                    prepare(env, single_piece_fen(square, retry_kind))
                    retries.append(dict(piece=chess.piece_name(retry_kind), **attempt(env, square, target)))
            rows.append(dict(square=chess.square_name(square), target=chess.square_name(target),
                             piece=chess.piece_name(kind), first=first, retries=retries,
                             path_feasible=first["success"] or any(r["success"] for r in retries)))
            save(f"square {i+1}/64")
        report["execution"].update(total=64, first_pass=sum(r["first"]["success"] for r in rows),
                                   path_feasible=sum(r["path_feasible"] for r in rows))
        crowded_rows = []
        report["crowded_execution"] = dict(suite=SUITE_VERSION, rows=crowded_rows)
        for case in CASES:
            seed = zlib.crc32(case.id.encode()) & 0xffffffff
            prepare(env, case.fen, seed)
            move = chess.Move.from_uci(case.move)
            assert not chess.Board(case.fen).is_capture(move)
            crowded_rows.append(dict(case=case.id, seed=seed, move=case.move,
                                     **attempt(env, move.from_square, move.to_square)))
            save(f"crowded {len(crowded_rows)}/{len(CASES)}")
        report["crowded_execution"].update(total=len(CASES), passed=sum(r["success"] for r in crowded_rows))
    finally:
        env.close()

    tray_rows = []
    for i in range(cfg.board.tray_slots):
        # A fresh board means free_tray_slot() is always slot zero. Move that
        # slot's origin to the actual destination under test, without fake
        # captured-list entries or changing the controller/evaluator.
        destination = cfg.board.tray_slot(i)
        tray_cfg = cfg.model_copy(update={"board": cfg.board.model_copy(update={
            "tray_y0": cfg.board.tray_y0 + i * cfg.board.tray_pitch})})
        env = ChessSimEnv(tray_cfg)
        try:
            prepare(env, single_piece_fen(chess.A1, EXEC_TYPES[i]))
            assert np.allclose(env.free_tray_slot(), destination)
            tray_rows.append(dict(slot=i, target_mm=[v*1000 for v in destination],
                                  piece=chess.piece_name(EXEC_TYPES[i]), **attempt(env, chess.A1, None)))
        finally:
            env.close()
        report["tray_execution"] = dict(total=len(tray_rows), passed=sum(r["success"] for r in tray_rows), rows=tray_rows)
        save(f"tray {i+1}/{cfg.board.tray_slots}")
    report["phase8_pass"] = report["execution"]["path_feasible"] == 64 and report["tray_execution"]["passed"] == cfg.board.tray_slots
    save("complete")


if __name__ == "__main__":
    main()
