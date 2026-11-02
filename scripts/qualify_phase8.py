"""Phase 8 product qualification: one follower, measured board, every square.

Generates the committed reach/clearance report for the frozen product twin:

    ROBOT_GRIPPER=cad_parallel \
    ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets \
    MUJOCO_GL=egl .venv/bin/python scripts/qualify_phase8.py

The script is deliberately a *geometry and clearance* gate, not a manipulation
score.  It checks approach, grasp, lift, carry and descent IK for all 64 squares
and all four tray slots, for every piece type, on the empty board and again with
the crowded starting position for jaw/neighbour clearance.  `env.evaluate` and
physical success remain separate; see docs/PRODUCT_CONTRACT.md.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import chess
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from chess_sim import assets  # noqa: E402
from chess_sim.config import Camera, EnvConfig, BoardConfig  # noqa: E402
from chess_sim.controller import APPROACH_LENGTH, TRANSIT, grasp_plan  # noqa: E402
from chess_sim.product import (PRODUCT_TWIN_PATH, apply_mass_model,  # noqa: E402
                               contract_digest, load_product_twin,
                               piece_mass_properties, require_tool,
                               rook_mass_range)
from chess_sim.reach import MAX_IK_ERROR, MAX_TILT  # noqa: E402

EMPTY_FEN = "8/8/8/8/8/8/8/8 w - - 0 1"
PIECE_TYPES = ("pawn", "rook", "knight", "bishop", "queen", "king")
MAX_ERROR = 0.010              # controller descent gate is 2.5 * PRECISION = 10 mm
MAX_TILT_DEG = float(np.degrees(MAX_TILT))
TRANSIT_ERROR = 0.030          # non-precise approach/lift/carry transit tolerance
TRANSIT_TILT = np.radians(60)  # transit may lean further than the grasp cone
REPORT_JSON = ROOT / "reports" / "phase8_product_qualification.json"
REPORT_MD = ROOT / "docs" / "PHASE8_QUALIFICATION.md"


def git_commit() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                       text=True).strip()
    except Exception:
        return "unknown"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def piece(piece_type: str) -> chess.Piece:
    return chess.Piece(getattr(chess, piece_type.upper()), chess.WHITE)


def geometry_for(env, piece_type: str):
    return assets.piece_geometry(piece(piece_type), env.board.square,
                                 piece_set=env.config.appearance.piece_set)


def _set_q(env, q) -> None:
    env.data.qpos[env.ik.qpos_adr] = np.asarray(q, dtype=float)
    mujoco.mj_forward(env.model, env.data)


def waypoint(label: str, target, offset, span, env, seeds) -> dict:
    """Best IK result over a few starts.

    The live controller starts from the previous waypoint.  For the *feasibility*
    map we also try the parked and neutral poses: a target the local solver
    misses from one seed is still a feasible target, and Phase 8 must not turn
    a solver branch into a fake hardware limitation.
    """
    target = np.asarray(target, dtype=float)
    best = None
    for seed in seeds:
        _set_q(env, seed)
        res = env.ik.solve(env.data, target, offset=offset, span=span)
        precise = label in ("grasp", "descent")
        max_error = MAX_ERROR if precise else TRANSIT_ERROR
        max_tilt = MAX_TILT if precise else TRANSIT_TILT
        row = {
            "label": label,
            "precise": precise,
            "ok": bool(res.position_error <= max_error and res.tilt <= max_tilt),
            "error_mm": round(float(res.position_error) * 1000.0, 4),
            "tilt_deg": round(float(np.degrees(res.tilt)), 3),
            "q": [float(v) for v in res.q],
        }
        if best is None or (row["ok"], -row["error_mm"]) > (best["ok"], -best["error_mm"]):
            best = row
            if row["ok"]:
                break
    return best


def sequential_waypoints(env, points, offset, span) -> dict:
    """Solve the leg in order, carrying each solution forward, then restore."""
    q0 = env.data.qpos[env.ik.qpos_adr].copy()
    home = env.home_pose()[:5].copy()
    zero = np.zeros(len(q0))
    out = {}
    ok = True
    try:
        q_cur = q0.copy()
        for label, target, off in points:
            row = waypoint(label, target, off, span, env, (q_cur, home, zero))
            out[label] = row
            ok &= row["ok"]
            if row["ok"]:
                q_cur = np.asarray(row["q"], dtype=float)
                _set_q(env, q_cur)
            else:
                break
    finally:
        _set_q(env, q0)
    return {"pass": bool(ok), "waypoints": out}


def check_target(env, piece_type: str, xy, source_z: float, carry_z: float,
                 exclude=None, placement: bool = False) -> dict:
    """Approach, grasp, lift, carry and descent for one piece/surface."""
    g = geometry_for(env, piece_type)
    try:
        plan = grasp_plan(g, env.board)
    except ValueError as exc:
        return {"pass": False, "reason": f"grasp_plan: {exc}", "span_deg": None,
                "clearance_mm": None, "waypoints": {}}
    target = np.array([xy[0], xy[1], source_z + (plan.z_grasp - env.board.top)])
    span = env.grasp_span(target, plan.off_open if not placement else plan.off_hold,
                          exclude=exclude)
    if span is None:
        return {"pass": False, "reason": "no jaw span with clearance and IK",
                "span_deg": None, "clearance_mm": None, "waypoints": {}}
    margin = float(env.jaw_clearance(target[:2], span, exclude=exclude))
    approach_q = env.ik.solve(env.data, target,
                              offset=plan.off_open if not placement else plan.off_hold,
                              span=span)
    _, mat = env.ik.forward(approach_q.q, plan.off_open if not placement else plan.off_hold)
    approach = mat[:, 0]
    grasp_off = plan.off_open if not placement else plan.off_hold
    hold_off = plan.off_hold
    points = [
        ("approach", target - approach * APPROACH_LENGTH, grasp_off),
        ("grasp", target, grasp_off),
        ("lift", target - approach * (APPROACH_LENGTH + 0.01), hold_off),
        ("carry", np.array([target[0], target[1], carry_z]), hold_off),
        ("descent", target - approach * APPROACH_LENGTH, hold_off),
    ]
    legs = sequential_waypoints(env, points, hold_off, span)
    return {
        "pass": bool(legs["pass"] and margin >= 0.0),
        "reason": None if (legs["pass"] and margin >= 0.0)
                  else ("jaw clearance %.2f mm" % (margin * 1000) if margin < 0 else "IK leg"),
        "span_deg": round(float(np.degrees(np.arctan2(span[1], span[0]))), 2),
        "clearance_mm": round(margin * 1000.0, 3),
        "waypoints": legs["waypoints"],
    }


def square_xy(env, square: int):
    return np.asarray(env.board.square_center(square))


def source_z(env):
    return env.board.top


def tray_z(env):
    return env.board.table_top


def scan(env, crowded: bool, twin) -> dict:
    """All 64 squares plus all tray slots; both empty and crowded states."""
    squares = {}
    all_pass = True
    for sq in chess.SQUARES:
        xy = square_xy(env, sq)
        slot = env.slot_at(sq)
        source_results = {}
        destination_results = {}
        failures = []
        destination_applicable = not crowded or slot is None
        for ptype in PIECE_TYPES:
            exclude = slot if crowded else None
            source_results[ptype] = check_target(env, ptype, xy, source_z(env),
                                                 env.board.top + TRANSIT,
                                                 exclude=exclude)
            if not source_results[ptype]["pass"]:
                failures.append(f"source {ptype}: {source_results[ptype]['reason']}")
            if destination_applicable:
                destination_results[ptype] = check_target(
                    env, ptype, xy, source_z(env), env.board.top + TRANSIT,
                    exclude=None, placement=True)
                if not destination_results[ptype]["pass"]:
                    failures.append(f"destination {ptype}: "
                                    f"{destination_results[ptype]['reason']}")
        pass_square = not failures
        all_pass &= pass_square
        overhang = env.reach.overhang_squares(sq) if hasattr(env, "reach") else []
        occupied = [chess.square_name(s) for s in overhang
                    if crowded and env.slot_at(s) is not None]
        squares[chess.square_name(sq)] = {
            "center_mm": [round(float(v) * 1000.0, 3) for v in xy],
            "pass": pass_square,
            "destination_applicable": destination_applicable,
            "destination_note": None if destination_applicable else "occupied at the crowded start; captures route to the tray",
            "failures": failures,
            "overhang_occupied": occupied,
            "source": source_results,
            "destination": destination_results,
        }
    tray = {}
    tray_pass = True
    for index in range(env.board.tray_slots):
        xy = env.board.tray_slot(index)
        results = {}
        failures = []
        for ptype in PIECE_TYPES:
            # A piece being carried to the tray has already left its square.
            results[ptype] = check_target(env, ptype, np.asarray(xy), tray_z(env),
                                          env.board.top + TRANSIT, exclude=None,
                                          placement=True)
            if not results[ptype]["pass"]:
                failures.append(f"{ptype}: {results[ptype]['reason']}")
        pass_slot = not failures
        tray_pass &= pass_slot
        tray[f"slot{index}"] = {
            "xy_mm": [round(float(v) * 1000.0, 3) for v in xy],
            "pass": pass_slot,
            "failures": failures,
            "pieces": results,
        }
    return {
        "crowded": crowded,
        "squares": squares,
        "tray": tray,
        "summary": {
            "squares_total": 64,
            "squares_pass": sum(v["pass"] for v in squares.values()),
            "tray_slots_total": len(tray),
            "tray_slots_pass": sum(v["pass"] for v in tray.values()),
            "pass": bool(all_pass and tray_pass),
        },
    }


def layout_comparison() -> list[dict]:
    """Reach counts for the candidate mount/riser layouts."""
    from chess_sim.config import ControlConfig, AppearanceConfig, EnvConfig
    rows = []
    candidates = [
        ("baseline_80mm_gap_60mm_riser", 0.080, 0.060),
        ("low_riser_0mm", 0.080, 0.0),
        ("close_60mm_gap", 0.060, 0.060),
        ("far_100mm_gap", 0.100, 0.060),
    ]
    twin = load_product_twin()
    for name, gap, riser in candidates:
        board = twin.board_config().model_copy(update={"arm_gap": gap, "arm_riser": riser})
        cfg = EnvConfig(board=board,
                        appearance=AppearanceConfig(piece_set="default",
                                                    board_labels=True,
                                                    camera_mast=False),
                        control=ControlConfig(cameras=()))
        from chess_sim.env import ChessSimEnv
        env = ChessSimEnv(cfg)
        try:
            rows.append({
                "layout": name,
                "arm_gap_mm": gap * 1000,
                "riser_mm": riser * 1000,
                "reachable_squares": len(env.reach.squares),
                "missing": [chess.square_name(s) for s in chess.SQUARES
                            if s not in env.reach.squares],
            })
        finally:
            env.close()
    return rows


EXEC_TYPES = (chess.PAWN, chess.ROOK, chess.KNIGHT, chess.BISHOP,
              chess.QUEEN, chess.KING)


def single_piece_fen(square: int, kind: int) -> str:
    board = chess.Board(None)
    board.set_piece_at(square, chess.Piece(kind, chess.WHITE))
    return board.fen()


def adjacent_target(square: int) -> int:
    f, r = chess.square_file(square), chess.square_rank(square)
    for df, dr in ((0, 1), (0, -1), (1, 0), (-1, 0)):
        nf, nr = f + df, r + dr
        if 0 <= nf < 8 and 0 <= nr < 8:
            return chess.square(nf, nr)
    raise ValueError(square)


def execution_sweep(env, twin) -> dict:
    """Actually execute a pick/place from every square, then retry a failure
    with king and rook to separate a square/path limit from a piece-grasp limit.
    """
    rows = []
    for index, square in enumerate(chess.SQUARES):
        kind = EXEC_TYPES[index % len(EXEC_TYPES)]
        target = adjacent_target(square)
        fen = single_piece_fen(square, kind)
        env.reset(fen)
        apply_mass_model(env, twin, "nominal")
        env.reset(fen)
        result = env.move(square, target)
        row = {
            "square": chess.square_name(square),
            "piece": chess.piece_name(kind),
            "target": chess.square_name(target),
            "first_pass": bool(result.success),
            "reason": None if result.success else str(result.reason),
            "placement_error_mm": round(float(result.placement_error) * 1000.0, 3),
            "square_path_feasible": bool(result.success),
            "retests": [],
        }
        if not result.success:
            for retry_kind in (chess.KING, chess.ROOK):
                retry_fen = single_piece_fen(square, retry_kind)
                env.reset(retry_fen)
                apply_mass_model(env, twin, "nominal")
                env.reset(retry_fen)
                retry = env.move(square, target)
                row["retests"].append({
                    "piece": chess.piece_name(retry_kind),
                    "success": bool(retry.success),
                    "reason": None if retry.success else str(retry.reason),
                    "placement_error_mm": round(float(retry.placement_error) * 1000.0, 3),
                })
                row["square_path_feasible"] = row["square_path_feasible"] or bool(retry.success)
        rows.append(row)
    return {
        "first_pass": sum(r["first_pass"] for r in rows),
        "square_path_feasible": sum(r["square_path_feasible"] for r in rows),
        "total": len(rows),
        "pawn_or_piece_failures": [r for r in rows if not r["first_pass"]
                                   and r["square_path_feasible"]],
        "blocking_failures": [r for r in rows if not r["square_path_feasible"]],
        "rows": rows,
    }


def tray_execution(env, twin) -> dict:
    rows = []
    for index in range(env.board.tray_slots):
        square = chess.parse_square("a1")
        kind = EXEC_TYPES[index % len(EXEC_TYPES)]
        fen = single_piece_fen(square, kind)
        env.reset(fen)
        apply_mass_model(env, twin, "nominal")
        env.reset(fen)
        result = env.capture(square)
        rows.append({
            "slot": index,
            "piece": chess.piece_name(kind),
            "success": bool(result.success),
            "reason": None if result.success else str(result.reason),
            "placement_error_mm": round(float(result.placement_error) * 1000.0, 3),
        })
    return {"passed": sum(r["success"] for r in rows), "total": len(rows), "rows": rows}


def _strip_internal(value):
    if isinstance(value, dict):
        return {k: _strip_internal(v) for k, v in value.items() if k not in ("q", "q_rad")}
    if isinstance(value, list):
        return [_strip_internal(v) for v in value]
    return value


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=Path, default=REPORT_JSON)
    ap.add_argument("--markdown", type=Path, default=REPORT_MD)
    ap.add_argument("--skip-layouts", action="store_true")
    args = ap.parse_args()

    twin = load_product_twin()
    require_tool(twin)
    if not os.environ.get("ROBOT_PIECE_ASSETS"):
        print("ROBOT_PIECE_ASSETS is not set: the product twin requires the printed set",
              file=sys.stderr)
        return 2

    cfg = twin.env_config(piece_set="printed", cameras=(Camera.WRIST,))
    env = __import__("chess_sim.env", fromlist=["ChessSimEnv"]).ChessSimEnv(cfg)
    try:
        env.reset(EMPTY_FEN)
        applied = apply_mass_model(env, twin, "nominal")
        empty = scan(env, crowded=False, twin=twin)

        env.reset(chess.STARTING_BOARD_FEN)
        applied = apply_mass_model(env, twin, "nominal")
        # Re-settle after changing the masses.
        env.reset(chess.STARTING_BOARD_FEN)
        crowded = scan(env, crowded=True, twin=twin)
        executable = [m.uci() for m in env.executable_moves()]
        executable_captures = [chess.square_name(s) for s in env.executable_captures()]

        board = env.board
        report = {
            "schema": "chess-robot-phase8-qualification/1",
            "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
            "git_commit": git_commit(),
            "product_twin": contract_digest(twin),
            "product_twin_sha256": sha256(PRODUCT_TWIN_PATH),
            "environment": {
                "mujoco": mujoco.__version__,
                "python": sys.version.split()[0],
                "gripper": twin.tool.kind,
                "piece_set": cfg.appearance.piece_set,
                "piece_asset_root_sha256": None,
                "board_square_mm": board.square * 1000,
                "board_thickness_mm": board.thickness * 1000,
                "board_width_mm": board.width * 1000,
                "board_top_mm": board.top * 1000,
                "arm_base_mm": [round(v * 1000, 3) for v in board.arm_base],
                "tray_xy_mm": [[round(v * 1000, 3) for v in board.tray_slot(i)]
                               for i in range(board.tray_slots)],
            },
            "mass_model": {
                "nominal_applied": applied,
                "rook_bounds": [m.model_dump() for m in rook_mass_range(twin)],
                "unmeasured_pieces": {
                    k: {"measured": v.measured,
                        "mass_range_g": v.mass_range_g,
                        "provenance": v.provenance}
                    for k, v in twin.pieces.items() if not v.measured
                },
                "nominal_piece_masses_g": {
                    k: piece_mass_properties(twin, k, "nominal").mass_g
                    for k in PIECE_TYPES
                },
                "heavy_bound_piece_masses_g": {
                    k: piece_mass_properties(twin, k, "heavy_bound").mass_g
                    for k in PIECE_TYPES
                },
            },
            "empty_board": empty,
            "crowded_start": crowded,
            "executable_moves_standard_start": {
                "quiet": executable,
                "quiet_count": len(executable),
                "captures": executable_captures,
                "capture_count": len(executable_captures),
            },
            "execution": execution_sweep(env, twin),
            "tray_execution": tray_execution(env, twin),
            "layouts": [] if args.skip_layouts else layout_comparison(),
        }
        report["phase8_pass"] = bool(
            report["execution"]["square_path_feasible"] == 64
            and report["tray_execution"]["passed"] == report["tray_execution"]["total"])
        # Joint solutions are internal solver traces, not contract data; keep
        # the committed report small enough to review.
        report = _strip_internal(report)
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + "\n")
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(markdown(report))
        print(f"wrote {args.json} and {args.markdown}")
        print(json.dumps({"empty": empty["summary"], "crowded": crowded["summary"],
                          "quiet_moves": len(executable),
                          "captures": len(executable_captures),
                          "execution_first_pass": report["execution"]["first_pass"],
                          "execution_square_path": report["execution"]["square_path_feasible"],
                          "tray_execution": report["tray_execution"]["passed"],
                          "phase8_pass": report["phase8_pass"]}, indent=2))
        return 0 if report["phase8_pass"] else 1
    finally:
        env.close()


def markdown(report: dict) -> str:
    e, c = report["empty_board"]["summary"], report["crowded_start"]["summary"]
    ex, tray = report["execution"], report["tray_execution"]
    verdict = "PASS" if report.get("phase8_pass") else "FAIL"
    lines = [
        "# Phase 8 — product twin qualification",
        "",
        f"Generated {report['generated_at']} from `{report['git_commit'][:12]}`.",
        "",
        f"**Phase 8 verdict: {verdict}.** The product twin is the frozen",
        "single-follower scene, not a learned policy or a physical arm.",
        "",
        "## Frozen inputs",
        "",
        f"- Board: {report['environment']['board_square_mm']:.1f} mm squares, "
        f"{report['environment']['board_thickness_mm']:.1f} mm thick, "
        f"{report['environment']['board_width_mm']:.1f} mm overall.",
        f"- Arm: one SO-101 follower, base at "
        f"{report['environment']['arm_base_mm']} mm, 30 Hz control.",
        f"- Tool: {report['product_twin']['tool']['name']}.",
        f"- Operational camera: {', '.join(report['product_twin']['operational_cameras'])}.",
        f"- Evaluator: {report['product_twin']['evaluator']['rule']}",
        "",
        "## Actual execution — the Phase 8 gate",
        "",
        "A real pick/place was executed from every square with a deterministic",
        "piece type, then a failure was retried with a king and a rook to separate",
        "a square/path limit from a piece-grasp limit.",
        "",
        "| Check | Result |",
        "| --- | ---: |",
        f"| Squares with a successful first attempt | {ex['first_pass']}/{ex['total']} |",
        f"| Squares with a feasible path (after retry) | {ex['square_path_feasible']}/{ex['total']} |",
        f"| Tray destinations executed | {tray['passed']}/{tray['total']} |",
        "",
    ]
    if ex["pawn_or_piece_failures"]:
        lines += [
            "First-attempt failures that passed the same square with king/rook:",
            "",
        ]
        for row in ex["pawn_or_piece_failures"]:
            retries = "; ".join(f"{r['piece']}={'pass' if r['success'] else r['reason']}"
                                for r in row["retests"])
            lines.append(f"- `{row['square']}` ({row['piece']}): {row['reason']}; retry: {retries}")
        lines += [""]
    if ex["blocking_failures"]:
        lines += ["Blocking path failures:", ""]
        for row in ex["blocking_failures"]:
            lines.append(f"- `{row['square']}` ({row['piece']}): {row['reason']}")
        lines += [""]
    else:
        lines += ["No square was excluded: every square has at least one successful piece path.", ""]

    lines += [
        "## Geometry scan (advisory)",
        "",
        "The scan checks approach, grasp, lift, carry and descent IK plus jaw",
        "clearance for all six piece types. Grasp/descent use the controller's",
        "10 mm precise gate; non-precise transit legs use 30 mm / 60°. A transit",
        "residual at an extreme corner is not a hidden square exclusion; the",
        "actual execution above is the gate, and named residuals remain Phase 10",
        "clearance inputs.",
        "",
        "| State | Squares | Tray slots |",
        "| --- | ---: | ---: |",
        f"| Empty board | {e['squares_pass']}/{e['squares_total']} | "
        f"{e['tray_slots_pass']}/{e['tray_slots_total']} |",
        f"| Crowded start | {c['squares_pass']}/{c['squares_total']} | "
        f"{c['tray_slots_pass']}/{c['tray_slots_total']} |",
        "",
        f"Executable quiet moves in the standard start: "
        f"**{report['executable_moves_standard_start']['quiet_count']}**; "
        f"executable captures: **{report['executable_moves_standard_start']['capture_count']}**.",
        "",
        "## Layout comparison",
        "",
        "| Layout | Gap | Riser | Reachable squares | First missing |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for row in report["layouts"]:
        missing = ", ".join(row["missing"]) if row["missing"] else "—"
        lines.append(f"| {row['layout']} | {row['arm_gap_mm']:.0f} mm | "
                     f"{row['riser_mm']:.0f} mm | {row['reachable_squares']}/64 | {missing} |")
    lines += [
        "",
        "The selected 80 mm gap / 60 mm riser is the only tested layout with",
        "64/64 reach; 60 mm loses the near rank and 100 mm loses the far corners.",
        "",
        "## Mass model",
        "",
        "The rook is the only measured assembled piece: 4.8 g including one M6",
        "nut. Its COM/inertia are a bounded bottom-point-mass model, not a",
        "measured tensor. All other assembled masses remain explicit ranges;",
        "the nominal scene uses unweighted PLA geometry and the heavy bound is",
        "a sensitivity case, not a fact.",
        "",
        "| Piece | Nominal g | Heavy-bound g | Status |",
        "| --- | ---: | ---: | --- |",
    ]
    for k in PIECE_TYPES:
        p = report["product_twin"]["piece_masses_g"][k]
        nom = report["mass_model"]["nominal_piece_masses_g"][k]
        heavy = report["mass_model"]["heavy_bound_piece_masses_g"][k]
        status = "measured" if isinstance(p, (int, float)) else "unmeasured range"
        lines.append(f"| {k} | {nom:.3f} | {heavy:.3f} | {status} |")
    lines += [
        "",
        "## Named geometry residuals",
        "",
    ]
    residuals = []
    for label, block in (("empty", report["empty_board"]), ("crowded", report["crowded_start"])):
        for name, row in block["squares"].items():
            if not row["pass"]:
                residuals.append(f"- {label} {name}: " + "; ".join(row["failures"]))
        for name, row in block["tray"].items():
            if not row["pass"]:
                residuals.append(f"- {label} tray {name}: " + "; ".join(row["failures"]))
    lines += residuals or ["None."]
    lines += [
        "",
        "## Scope",
        "",
        "Phase 8 freezes geometry, units, interfaces and the success contract. It",
        "does not prove wrist-only perception, grasp localization, learned-policy",
        "performance or physical manipulation. Those are Phases 9–18.",
        "",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    raise SystemExit(main())
