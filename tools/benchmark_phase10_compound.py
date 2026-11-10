"""Registered wrist-only benchmark for compound chess transactions.

Unlike the scene evaluator, source localization and view choice use only the
Phase 9 calibration plus wrist ArUco registration. Simulator state is used to
construct each board and score resulting physical piece motion.
"""
from __future__ import annotations

import argparse
import chess
import hashlib
import json
from dataclasses import replace
import math
import os
import platform
import random
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from chess_sim import gripper
from chess_sim.config import Camera, RandomizationConfig
from chess_sim.controller import grasp_plan
from chess_sim.env import ChessSimEnv
from chess_sim.fiducials import MarkerLayout, estimate_board_pose
from chess_sim.self_occlusion import SelfOcclusionModel
from chess_sim.product import apply_mass_model, layout_env_config, load_product_twin
from chess_sim.tasks import CaptureTask, MoveTask
from chess_sim.transactions import execute_chess_transaction
from chess_sim.vision import detect_piece_base
from chess_sim.vision_geometry import Localization
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.wrist_scan import (PLAN_PATH, ScanFrame, ScanPose, load_plan)

from benchmark_phase10 import _align_empty_reference, _move_arm, _select_view
from phase10_matrix import build_phase10_scenarios


ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: str | Path | None) -> str | None:
    if not path:
        return None
    source = Path(path)
    return hashlib.sha256(source.read_bytes()).hexdigest() if source.is_file() else None


def _body_yaw_deg(quat) -> float:
    """Evaluation-only body yaw; never passed to the wrist orientation estimator."""
    w, x, y, z = (float(v) for v in quat)
    return math.degrees(math.atan2(2 * (w * z + x * y),
                                  1 - 2 * (y * y + z * z)))


# Exploratory fractions selected from a separate four-samples-per-type
# silhouette sweep. Kept here for held-out validation; not yet the production
# detector default because the source sample is intentionally small.
CENTROID_FRACTION = {"pawn": 0.05, "rook": 0.35, "knight": 0.30,
                     "bishop": 0.25, "queen": 0.40, "king": 0.15}
LOCALIZATION_SEARCH_RADIUS_M = 0.010


def frozen_scenarios(path: str | Path, seed: int) -> dict[int, tuple[str, str, str]]:
    """Load recorded identities, never regenerate a consumed diagnostic case."""
    document = json.loads(Path(path).read_text())
    entries = document.get("selection", []) if isinstance(document, dict) else document
    result = {}
    for entry in entries:
        if entry["seed"] != seed:
            continue
        index = entry["index"]
        row = (entry["category"], entry["fen"], entry["uci"])
        if index in result or not isinstance(index, int) or index < 0:
            raise ValueError("duplicate/invalid frozen scenario index")
        board = chess.Board(row[1])
        move = chess.Move.from_uci(row[2])
        category = ("castle" if board.is_castling(move) else
                    "en_passant" if board.is_en_passant(move) else
                    "capture" if board.is_capture(move) else "quiet")
        if not board.is_valid() or move not in board.legal_moves or category != row[0]:
            raise ValueError("invalid frozen scenario identity")
        result[index] = row
    if not result:
        raise ValueError("no frozen scenarios for requested seed")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=int, default=300)
    parser.add_argument("--start-case", type=int, default=0,
                        help="absolute matrix index, useful for isolated bucket diagnostics")
    parser.add_argument("--seed", type=int, default=20261106)
    parser.add_argument("--calibration", default=".private/phase9/calibration.json")
    parser.add_argument("--mass-regime", choices=("matrix", "nominal", "heavy_bound",
                        "sensitivity_low", "sensitivity_high"), default="matrix")
    parser.add_argument("--friction-scale", type=float, default=1.0)
    parser.add_argument("--orientation-yaw-deg", type=float, default=None,
                        help="controlled moving-piece yaw for a targeted orientation sweep")
    parser.add_argument("--out", default=".private/phase10/transactions-seed20261106.json")
    parser.add_argument("--scenario-file", default=None,
                        help="replay frozen selection identities instead of regenerating cases")
    parser.add_argument("--debug-dir", default=None)
    parser.add_argument("--no-span-hint", action="store_true",
                        help="control arm: withhold the wrist-axis hint from the controller")
    parser.add_argument("--trace-case", type=int, default=None,
                        help="record full deterministic per-control-step evidence for this absolute case")
    parser.add_argument("--trace-out", default=None,
                        help="JSON output for --trace-case, including MuJoCo integration states")
    args = parser.parse_args()
    if args.cases < 1 or not 0.25 <= args.friction_scale <= 2.0:
        parser.error("cases must be positive and base friction scale within [0.25, 2.0]")
    if (args.trace_case is None) != (args.trace_out is None):
        parser.error("--trace-case and --trace-out must be supplied together")
    scenarios = (frozen_scenarios(args.scenario_file, args.seed) if args.scenario_file
                 else build_phase10_scenarios(args.seed))
    if args.scenario_file and any(i not in scenarios for i in
            range(args.start_case, args.start_case + args.cases)):
        parser.error("requested index absent from frozen scenario file")

    calibration = WristCalibration.load(args.calibration)
    self_model = SelfOcclusionModel(calibration)
    poses = load_plan()
    env = ChessSimEnv(layout_env_config("printed", cameras=(Camera.WRIST,)).model_copy(
        update={"randomization": RandomizationConfig(
            board_shift=0.008, board_yaw=math.radians(2.5), board_side=0.008,
            arm_joint_jitter=0.0, camera=False)}))
    piece_assets = Path(os.environ.get("ROBOT_PIECE_ASSETS", ""))
    gripper_assets = Path(os.environ.get("ROBOT_GRIPPER_ASSETS", ""))
    source_paths = [Path(__file__), Path(__file__).with_name("phase10_matrix.py"),
        Path(__file__).with_name("benchmark_phase10.py"), ROOT / "chess_sim/controller.py",
        ROOT / "chess_sim/gripper.py", ROOT / "chess_sim/assets.py", ROOT / "chess_sim/env.py", ROOT / "chess_sim/config.py",
        ROOT / "chess_sim/transactions.py", ROOT / "chess_sim/product.py", ROOT / "chess_sim/scene.py",
        ROOT / "chess_sim/ik.py", ROOT / "chess_sim/vision.py",
        ROOT / "chess_sim/wrist_calibration.py", ROOT / "chess_sim/wrist_scan.py",
        ROOT / "chess_sim/piece_orientation.py", ROOT / "chess_sim/vision_geometry.py",
        ROOT / "chess_sim/fiducials.py", ROOT / "chess_sim/perception_data.py",
        ROOT / "chess_sim/perception.py", ROOT / "chess_sim/self_occlusion.py"]
    source_hashes = {str(path.relative_to(ROOT)): _sha256(path) for path in source_paths}
    scenario_hash = hashlib.sha256(json.dumps(scenarios, separators=(",", ":"))
                                    .encode()).hexdigest()
    try:
        git_revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        git_revision = None
    provenance = {
        "git_revision": git_revision,
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "scenario_sha256": scenario_hash,
        "self_model_source_sha256": dict(self_model.sources),
        "calibration_sha256": _sha256(args.calibration),
        "scan_plan_sha256": _sha256(PLAN_PATH),
        "product_twin_sha256": _sha256(ROOT / "hardware/product_twin.json"),
        "current_layout_sha256": _sha256(ROOT / "hardware/current_layout.json"),
        "piece_manifest_sha256": _sha256(piece_assets / "set.json"),
        "gripper_profile_sha256": _sha256(gripper_assets / "profile.json"),
        "source_sha256": source_hashes,
        "env_config": env.config.model_dump(mode="json")}
    baseline_board = env.config.board
    twin = load_product_twin()
    baseline_friction = float(env.model.geom("board").friction[0])
    active_case = {"index": None, "mass_regime": None, "friction_scale": None,
                   "category": None, "fen": None, "uci": None}
    records = []
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)

    def checkpoint(record: dict) -> None:
        """Persist each completed case so a long matrix can be resumed safely."""
        record.setdefault("scan_contact_events", list(getattr(env, "_scan_contact_events", [])))
        record.setdefault("scan_collateral", len(getattr(env, "_scan_disturbed", set())))
        record.setdefault("scan_motion_records", list(getattr(env, "_scan_motion_records", [])))
        record["collateral"] = record.get("collateral", 0) + record["scan_collateral"]
        record.setdefault("matrix_index", active_case["index"])
        record.setdefault("mass_regime", active_case["mass_regime"])
        record.setdefault("board_friction_scale", active_case["friction_scale"])
        record.setdefault("category", active_case["category"])
        record.setdefault("fen", active_case["fen"])
        record.setdefault("uci", active_case["uci"])
        records.append(record)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(json.dumps({
            "schema": "robot-arm-phase10-transaction-checkpoint/2",
            "seed": args.seed, "requested_cases": args.cases,
            "provenance": provenance,
            "matrix": "300 legal transactions: 180 quiet, 72 captures, 24 castling, 24 en-passant",
            "board_friction_scale": args.friction_scale,
            "completed_cases": len(records), "records": records}, indent=2) + "\n")
        temporary.replace(output)

    try:
        for index in range(args.cases):
            matrix_index = args.start_case + index
            env._scan_fault=None; env._scan_contact_events=[]; env._scan_disturbed=set(); env._scan_motion_records=[]
            trace_rows = []
            # Keep each seeded layout draw independent. Within a case the
            # realized board is copied into ``env.config`` so empty-reference
            # and task resets stay aligned; restore the product baseline before
            # drawing the next case rather than randomizing from the last draw.
            env.config = env.config.model_copy(update={"board": baseline_board})
            cell = matrix_index % 9
            case_mass = (args.mass_regime if args.mass_regime != "matrix"
                         else ("nominal", "sensitivity_low", "sensitivity_high")[cell % 3])
            case_friction = args.friction_scale * (0.75, 1.0, 1.25)[cell // 3]
            active_case.update(index=matrix_index, mass_regime=case_mass,
                               friction_scale=case_friction,
                               orientation_yaw_deg=args.orientation_yaw_deg)
            apply_mass_model(env, twin, case_mass, all_slots=True)
            env.model.geom("board").friction[0] = baseline_friction * case_friction
            category, fen, uci = scenarios[matrix_index if args.scenario_file
                                            else matrix_index % len(scenarios)]
            active_case.update(category=category, fen=fen, uci=uci)
            board = chess.Board(fen)
            move = chess.Move.from_uci(uci)
            empty_rng = random.Random(args.seed + matrix_index * 7919)
            env.reset(chess.Board(None), rng=empty_rng)
            env.config = env.config.model_copy(update={"board": env.board})
            if category == "capture":
                first_square = move.to_square
            elif category == "en_passant":
                first_square = (move.to_square - 8 if board.turn == chess.WHITE
                                else move.to_square + 8)
            elif category == "castle":
                rank = chess.square_rank(move.from_square)
                rook_file = 7 if board.is_kingside_castling(move) else 0
                first_square = chess.square(rook_file, rank)
            else:
                first_square = move.from_square
            first_source = chess.square_name(first_square)
            source_piece = board.piece_at(first_square)
            source_slot = next(slot for slot in env.slots if slot.piece == source_piece)
            open_angle = gripper.gap_to_angle(
                grasp_plan(source_slot.geometry, env.board).gap_open)
            frames = []
            for planned in poses:
                joints = np.asarray(planned.joints, dtype=float).copy()
                joints[-1] = open_angle
                pose = ScanPose(planned.name, tuple(joints))
                _move_arm(env, joints)
                measured = tuple(float(v) for v in env.arm_joint_positions())
                frames.append(ScanFrame(env.render(Camera.WRIST).copy(), measured,
                                        0.0, pose.name))
            registration = estimate_board_pose(
                [(calibration.rectify(item.frame), calibration.camera(item.joints))
                 for item in frames], env.board.square, env.board.top,
                marker_z=env.board.table_top + env.board.marker_above_table,
                layout=MarkerLayout.for_board(env.board.field * 1000,
                    env.board.border * 1000, env.board.marker_plane))
            if registration is None:
                checkpoint({"case": matrix_index, "category": category,
                                "registered": False, "commit_allowed": False,
                                "reason": "registration failed"})
                continue
            references_by_pose = {item.pose: calibration.rectify(item.frame)
                                  for item in frames}
            cameras_by_pose = {item.pose: calibration.camera(item.joints)
                               for item in frames}

            first_xy = registration.square_center(first_source)
            view = _select_view(env, calibration, first_xy, registration.board_z,
                                source_slot.geometry.height, poses, open_angle)
            if view is None:
                checkpoint({"case": matrix_index, "category": category,
                                "registered": True, "commit_allowed": False,
                                "reason": "no calibrated view"})
                continue
            pose, _, _, _ = view
            env.home_pose = lambda p=pose: np.asarray(p.joints, dtype=float)
            env.reset(chess.Board(None))
            empty = calibration.rectify(env.render(Camera.WRIST).copy())
            reference_camera = calibration.camera(env.arm_joint_positions())
            env.reset(board)
            # Seed a modest, known translation and yaw in each scenario's
            # moving source piece; this alters only stimulus construction.
            actor_slot = env.slot_at(move.from_square)
            actor_xy = np.asarray(env.board.square_center(move.from_square))
            if actor_slot is not None:
                perturb_rng = random.Random(args.seed ^ (matrix_index * 104729 + 0xA51C))
                offset_mm = perturb_rng.choice((-6.0, -3.0, 0.0, 3.0, 6.0))
                yaw = math.radians(args.orientation_yaw_deg
                    if args.orientation_yaw_deg is not None
                    else perturb_rng.randrange(0, 360))
                env.set_piece_pose(actor_slot,
                    [actor_xy[0] + offset_mm / 1000 - actor_slot.geometry.center[0],
                     actor_xy[1] - actor_slot.geometry.center[1],
                     env.board.top - actor_slot.geometry.bottom_offset + 0.0015],
                    [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)],
                    zero_velocity=True)
                mujoco.mj_forward(env.model, env.data)

            contexts = {}
            localization_errors = []
            localization_metrics = []
            orientation_metrics = []
            reobservation_metrics = []
            localization_calls = []
            monitored_task = {"task": None, "before": {}, "evented": set(), "events": []}

            def localize(task, step, *, record_metrics=True):
                if getattr(env, "_scan_fault", None):
                    return None
                localization_calls.append(step)
                if category == "interrupted_capture" and step == 1:
                    return None
                slot = env.slot_at(task.source)
                if slot is None:
                    return None
                xy = registration.square_center(task.source)
                angle = gripper.gap_to_angle(grasp_plan(slot.geometry, env.board).gap_open)
                candidates = []
                type_name = chess.piece_name(slot.piece.piece_type)
                for planned in poses:
                    joints = np.asarray(planned.joints, dtype=float).copy()
                    joints[-1] = angle
                    view_pose = ScanPose(planned.name, tuple(joints))
                    _move_arm(env, joints)
                    if getattr(env, "_scan_fault", None):
                        return None
                    frame = calibration.rectify(env.render(Camera.WRIST).copy())
                    mask = self_model.exclusion_mask(env.arm_joint_positions())
                    camera = calibration.camera(env.arm_joint_positions())
                    reference = _align_empty_reference(
                        references_by_pose[planned.name], cameras_by_pose[planned.name],
                        camera, registration)
                    found = detect_piece_base(frame, reference, camera, xy,
                        registration.board_z, slot.geometry.height,
                        search_radius_m=LOCALIZATION_SEARCH_RADIUS_M,
                        centroid_height_fraction=CENTROID_FRACTION[type_name],
                        exclusion_mask=mask)
                    if found is not None:
                        candidates.append((found, view_pose, camera, frame, mask,
                                           planned.name))
                    if index == 0 and args.debug_dir:
                        from PIL import Image
                        debug = Path(args.debug_dir)
                        debug.mkdir(parents=True, exist_ok=True)
                        Image.fromarray(frame).save(debug / f"view-{planned.name}.png")
                        Image.fromarray(reference).save(
                            debug / f"reference-{planned.name}.png")
                        Image.fromarray(mask.astype(np.uint8) * 255).save(
                            debug / f"mask-{planned.name}.png")
                        metadata = {"pose": planned.name,
                            "camera_position": list(camera.camera_position),
                            "camera_rotation": list(camera.camera_rotation),
                            "camera_intrinsics": [camera.fx, camera.fy, camera.cx, camera.cy],
                            "expected_xy": list(xy), "truth_xy": list(env.piece_position(slot)[:2]),
                            "board_z": registration.board_z,
                            "height_m": slot.geometry.height, "piece": type_name}
                        (debug / f"metadata-{planned.name}.json").write_text(
                            json.dumps(metadata, indent=2) + "\n")
                if not candidates:
                    return None
                best, selected_pose, camera, frame, mask, _ = min(
                    candidates, key=lambda row: row[0].sigma_m)
                _move_arm(env, np.asarray(selected_pose.joints, dtype=float))
                camera = calibration.camera(env.arm_joint_positions())
                # Fuse independent wrist views by a robust XY median. The
                # dispersion term prevents a confident single view from hiding
                # cross-view disagreement.
                points = np.asarray([row[0].world_xy for row in candidates])
                fused_xy = np.median(points, axis=0)
                distances = np.linalg.norm(points - fused_xy, axis=1)
                # Asymptotic standard error of a coordinate-wise median, with
                # robust MAD scale and n independent wrist viewpoints.
                spread_sigma = (1.2533 * 1.4826 * float(np.median(distances))
                                / math.sqrt(len(candidates)))
                # Confidence is view-dependent (piece silhouette/contrast) and
                # should be fused with the same outlier robustness as XY. Using
                # the minimum made one poor silhouette veto five agreeing
                # calibrated views, even when the robust spread was low.
                fused = Localization(tuple(map(float, fused_xy)), best.pixel_xy,
                    float(np.median([row[0].confidence for row in candidates])),
                    max(min(row[0].sigma_m for row in candidates), spread_sigma))
                orientation_record = None
                # Pieces with a calibrated asymmetric section also yield a grasp
                # axis from the same wrist views. Attach it as a hint; the
                # controller still enforces clearance, reach and its own gates.
                if slot.geometry.grasp_width and not args.no_span_hint:
                    from chess_sim.assets import (piece_obj_path, piece_texture_path,
                                                  piece_tint)
                    from chess_sim.piece_orientation import OrientationView, fit_mesh_axis
                    views = [OrientationView(row[5], row[3], row[2], row[4])
                             for row in candidates]
                    mesh = Path(piece_obj_path(slot.piece, env.config.appearance.piece_set))
                    tint = piece_tint(slot.piece, env.config.appearance.white_rgba,
                                      env.config.appearance.black_rgba,
                                      env.config.appearance.piece_set)
                    fitted = fit_mesh_axis(mesh, views, tuple(fused.world_xy),
                                           registration.board_z,
                                           texture_path=piece_texture_path(
                                               slot.piece, env.config.appearance.piece_set),
                                           tint=tint)
                    oracle_yaw = _body_yaw_deg(
                        env.data.body(env._slot_body[slot.body]).xquat) % 180.0
                    orientation_record = {
                        "step": step, "square": str(task.source), "piece": type_name,
                        "estimate_present": fitted is not None,
                        "axis_deg_mod_180": (None if fitted is None else
                            math.degrees(fitted.axis_rad) % 180.0),
                        "sigma_deg": (None if fitted is None else
                            math.degrees(fitted.sigma_rad)),
                        "confidence": (None if fitted is None else fitted.confidence),
                        "supporting_views": (0 if fitted is None else
                            fitted.supporting_views),
                        "fit_loss": (None if fitted is None else fitted.fit_loss),
                        "accepted_for_hint": bool(fitted is not None and
                            fitted.acceptable()),
                        "hint_forwarded": bool(fused.span_rad is not None),
                        "evaluation_only_oracle_yaw_deg_mod_180": oracle_yaw,
                        "evaluation_only_axis_error_deg": (None if fitted is None else
                            abs((math.degrees(fitted.axis_rad) - oracle_yaw + 90.0)
                                % 180.0 - 90.0)),
                    }
                    # Abstain unless uncertainty, fit confidence, and view
                    # support all meet the estimator's calibrated acceptance
                    # contract. The controller still re-checks clearance and
                    # reach before committing an accepted hint.
                    if fitted is not None and fitted.acceptable():
                        fused = replace(fused, span_rad=float(fitted.axis_rad))
                    if orientation_record is not None:
                        orientation_record["hint_forwarded"] = bool(
                            fused.span_rad is not None)
                        orientation_record["forwarded_axis_deg_mod_180"] = (
                            None if fused.span_rad is None else
                            math.degrees(fused.span_rad) % 180.0)
                contexts[step] = {"slot": slot, "xy": xy,
                                   "camera": camera, "pose": selected_pose,
                                   "fusion_count": len(candidates),
                                   "fusion_spread_mm": spread_sigma * 1000}
                if record_metrics:
                    monitored_task["task"] = task
                    monitored_task["before"] = {item.body: env.piece_position(item).copy()
                                                 for item in env.active_slots()}
                    monitored_task["evented"] = set()
                truth = env.piece_position(slot)[:2]
                error = float(np.linalg.norm(np.asarray(fused.world_xy) - truth) * 1000)
                if record_metrics:
                    localization_errors.append(error)
                if record_metrics and index == 0 and args.debug_dir:
                    from PIL import Image
                    debug = Path(args.debug_dir)
                    debug.mkdir(parents=True, exist_ok=True)
                    Image.fromarray(frame).save(debug / "observation.png")
                    reference = _align_empty_reference(
                    references_by_pose[selected_pose.name],
                        cameras_by_pose[selected_pose.name], camera, registration)
                    Image.fromarray(reference).save(debug / "reference.png")
                    Image.fromarray(mask.astype(np.uint8) * 255).save(debug / "arm-mask.png")
                    (debug / "localization.json").write_text(json.dumps({
                        "category": category, "square": task.source,
                        "matrix_index": matrix_index,
                        "board_z": registration.board_z,
                        "camera_intrinsics": [camera.fx, camera.fy, camera.cx, camera.cy],
                        "camera_position": list(camera.camera_position),
                        "camera_rotation": list(camera.camera_rotation),
                        "expected_xy": list(xy),
                        "truth_xy": list(truth),
                        "localization": list(fused.world_xy),
                        "view_localizations": [list(row[0].world_xy) for row in candidates],
                        "view_poses": [row[5] for row in candidates],
                        "expected_xy": list(xy), "truth_xy": list(truth),
                        "fusion_spread_mm": spread_sigma * 1000,
                        "pixel": list(best.pixel_xy),
                        "height_m": slot.geometry.height,
                        "centroid_height_fraction": CENTROID_FRACTION[
                            chess.piece_name(slot.piece.piece_type)],
                        "error_mm": error,
                        "sigma_mm": fused.sigma_m * 1000},
                        indent=2) + "\n")
                if record_metrics:
                    localization_metrics.append({"step": step, "square": task.source,
                        "piece": type_name, "views_fused": len(candidates),
                        "expected_xy": list(xy), "truth_xy": list(truth),
                        "view_localizations": [list(row[0].world_xy) for row in candidates],
                        "view_poses": [row[5] for row in candidates],
                        "view_sigmas_mm": [row[0].sigma_m * 1000 for row in candidates],
                        "view_confidences": [row[0].confidence for row in candidates],
                        "fusion_spread_mm": spread_sigma * 1000,
                        "detected": True,
                        "confidence": fused.confidence,
                        "sigma_mm": fused.sigma_m * 1000,
                        "error_mm": error,
                        "acceptable": fused.acceptable()})
                    if orientation_record is not None:
                        orientation_metrics.append(orientation_record)
                    elif args.no_span_hint and slot.geometry.grasp_width:
                        orientation_metrics.append({"step": step,
                            "square": str(task.source), "piece": type_name,
                            "estimate_present": False, "hint_forwarded": False,
                            "reason": "disabled_by_span_hint_ablation"})
                return fused

            def reobserve(task, step):
                # Use the same multi-view estimator as the initial observation.
                # A single parked frame can disagree with the robust median by
                # >4 mm on edge squares and falsely abort an otherwise valid
                # grasp. Re-observation remains independent: it is captured
                # after the arm returns to the observation pose and uses only
                # the wrist image/calibration path.
                found = localize(task, step, record_metrics=False)
                context = contexts.get(step)
                truth = (env.piece_position(context["slot"])[:2]
                         if context is not None else None)
                error = (float(np.linalg.norm(np.asarray(found.world_xy) - truth) * 1000)
                         if found is not None and truth is not None else None)
                reobservation_metrics.append({"step": step, "detected": found is not None,
                    "confidence": found.confidence if found else None,
                    "sigma_mm": found.sigma_m * 1000 if found else None,
                    "error_mm": error})
                return found

            def monitor_control_step(action):
                """Retain transient wrong-piece/collateral motion events.

                Final-pose scoring alone misses a neighbor that was shoved
                during lift and happened to settle back before evaluation.
                """
                task = monitored_task["task"]
                if task is None:
                    return
                if matrix_index == args.trace_case:
                    state_size = mujoco.mj_stateSize(
                        env.model, mujoco.mjtState.mjSTATE_INTEGRATION)
                    integration_state = np.empty(state_size, dtype=np.float64)
                    mujoco.mj_getState(env.model, env.data, integration_state,
                                       mujoco.mjtState.mjSTATE_INTEGRATION)
                    contacts = []
                    for contact_index in range(env.data.ncon):
                        contact = env.data.contact[contact_index]
                        force = np.zeros(6)
                        mujoco.mj_contactForce(env.model, env.data,
                                               contact_index, force)
                        geom1, geom2 = (env.model.geom(int(contact.geom1)).name,
                                        env.model.geom(int(contact.geom2)).name)
                        body1 = int(env.model.geom_bodyid[int(contact.geom1)])
                        body2 = int(env.model.geom_bodyid[int(contact.geom2)])
                        contacts.append({
                            "geom1": geom1, "body1": env.model.body(body1).name,
                            "geom2": geom2, "body2": env.model.body(body2).name,
                            "distance_m": float(contact.dist),
                            "force_wrench": force.tolist()})
                    offset = (np.zeros(3) if env.diagnostic_offset is None
                              else np.asarray(env.diagnostic_offset, dtype=float))
                    tool_position, tool_rotation = env.ik.tool_pose(env.data, offset)
                    joint_position = env.arm_joint_positions().copy()
                    trace_rows.append({
                        "time_s": float(env.data.time),
                        "phase": env.diagnostic_phase,
                        "target_tcp_m": env.diagnostic_target,
                        "jaw_gap_command_m": env.diagnostic_gap,
                        "tool_offset_m": offset.tolist(),
                        "tool_position_m": np.asarray(tool_position).tolist(),
                        "tool_rotation": np.asarray(tool_rotation).tolist(),
                        "joint_position": joint_position.tolist(),
                        "jaw_gap_actual_m": gripper.angle_to_gap(float(joint_position[-1])),
                        "ik": env.diagnostic_ik,
                        "diagnostic_stall": env.diagnostic_stall,
                        "fen": env.position.fen(),
                        "piece_positions_m": {slot.body: env.piece_position(slot).tolist()
                                               for slot in env.active_slots()},
                        "contacts": contacts,
                        "integration_state": integration_state.tolist()})
                allowed = env.slot_at(task.source)
                before = monitored_task["before"]
                for slot in env.active_slots():
                    if slot == allowed or slot.body not in before:
                        continue
                    displacement = float(np.linalg.norm(
                        env.piece_position(slot)[:2] - before[slot.body][:2]))
                    key = (task.source, slot.body)
                    if displacement > env.config.tolerances.disturbance and key not in monitored_task["evented"]:
                        monitored_task["evented"].add(key)
                        monitored_task["events"].append({
                            "task_source": str(task.source),
                            "other_piece_body": slot.body,
                            "first_displacement_mm": displacement * 1000})

            input_fen_before = board.fen()
            outcome = execute_chess_transaction(
                env, board, move, localize, on_step=monitor_control_step,
                reobserve_source=reobserve)
            if matrix_index == args.trace_case:
                trace_output = Path(args.trace_out)
                trace_output.parent.mkdir(parents=True, exist_ok=True)
                trace_payload = {
                    "schema": "robot-arm-phase10-control-trace/1",
                    "seed": args.seed, "matrix_index": matrix_index,
                    "mass_regime": active_case["mass_regime"],
                    "board_friction_scale": active_case["friction_scale"],
                    "category": category, "fen": fen, "uci": uci,
                    "orientation_yaw_deg": args.orientation_yaw_deg,
                    "mujoco": mujoco.__version__,
                    "model_state_kind": "mjSTATE_INTEGRATION",
                    "application_state_per_step": "fen and piece_positions_m",
                    "steps": trace_rows,
                    "outcome": {"complete": outcome.physical.complete,
                        "commit_allowed": outcome.commit_allowed,
                        "reason": outcome.reason,
                        "diagnostic_stall": env.diagnostic_stall}}
                temporary_trace = trace_output.with_suffix(trace_output.suffix + ".tmp")
                temporary_trace.write_text(json.dumps(trace_payload) + "\n")
                temporary_trace.replace(trace_output)
            # Exercise the caller-facing authorization on an isolated copy of
            # the supplied game board, and retain both the returned decision
            # and the actual board-mutation event. The coordinator itself must
            # never mutate the caller's board.
            caller_board = board.copy()
            caller_fen_before = caller_board.fen()
            caller_commit_event = None
            if outcome.commit_allowed:
                caller_board.push(move)
                caller_commit_event = move.uci()
            interrupted = category == "interrupted_capture"
            interruption_prefix_completed = bool(
                interrupted and outcome.physical.failed_step == 1
                and outcome.physical.results
                and all(item.commit_eligible for item in outcome.physical.results))
            for pose_metric in orientation_metrics:
                if pose_metric.get("piece") != "knight":
                    continue
                final_slot = env.slot_at(move.to_square)
                if final_slot is None:
                    pose_metric["evaluation_only_final_yaw_deg"] = None
                    pose_metric["evaluation_only_final_yaw_residual_90_deg"] = None
                    pose_metric["evaluation_only_net_yaw_change_deg_mod_180"] = None
                    continue
                final_yaw = _body_yaw_deg(
                    env.data.body(env._slot_body[final_slot.body]).xquat)
                initial_axis = pose_metric.get(
                    "evaluation_only_oracle_yaw_deg_mod_180")
                pose_metric["evaluation_only_final_yaw_deg"] = final_yaw
                pose_metric["evaluation_only_final_yaw_residual_90_deg"] = abs(
                    (final_yaw + 45.0) % 90.0 - 45.0)
                pose_metric["evaluation_only_net_yaw_change_deg_mod_180"] = (
                    None if initial_axis is None else
                    (final_yaw - initial_axis + 90.0) % 180.0 - 90.0)
            checkpoint({"case": matrix_index, "category": category,
                "orientation_yaw_deg": args.orientation_yaw_deg,
                "registered": True, "registration_rms_px": registration.score,
                "localization_error_mm": localization_errors,
                "localization_metrics": localization_metrics,
                "orientation_metrics": orientation_metrics,
                "reobservation_metrics": reobservation_metrics,
                "primitive_success": [item.success for item in outcome.physical.results],
                "phase10_primitive_complete": [item.phase10_complete
                    for item in outcome.physical.results],
                "primitive_details": [{"task": item.task.label,
                    "success": item.success,
                    "placement_error_mm": item.placement_error * 1000,
                    "upright": item.upright,
                    "picked": item.picked,
                    "right_piece": item.right_piece,
                    "disturbed": list(item.disturbed),
                    "released": item.released,
                    "supported": item.supported,
                    "settled": item.settled,
                    "phase10_complete": item.phase10_complete,
                    "reason": str(item.reason) if item.reason else None}
                    for item in outcome.physical.results],
                "failed_step": outcome.physical.failed_step,
                "complete": outcome.physical.complete,
                "commit_allowed": outcome.commit_allowed,
                "controller_outcome": ("complete" if outcome.physical.complete else
                    (env.diagnostic_stall or outcome.reason or "incomplete")),
                "oracle_primitive_scores": [item.success for item in outcome.physical.results],
                "runtime_commit_authorization": outcome.commit_allowed,
                "caller_commit_spy": {"board_fen_before": caller_fen_before,
                    "board_fen_after": caller_board.fen(),
                    "commit_event": caller_commit_event},
                "input_game_state_unchanged_by_coordinator": board.fen() == input_fen_before,
                "interruption_requested": interrupted and 1 in localization_calls,
                "interruption_prefix_completed": interruption_prefix_completed,
                "interruption_contract_passed": (not interrupted or
                    interruption_prefix_completed),
                "reason": outcome.reason,
                "diagnostic_stall": env.diagnostic_stall,
                "wrong_piece_picks_final_pose": sum(
                    item.picked is not None and not item.right_piece
                    for item in outcome.physical.results),
                "final_pose_collateral": sum(len(item.disturbed)
                                   for item in outcome.physical.results),
                "transient_wrong_piece_or_collateral_events": monitored_task["events"],
                "transient_wrong_piece_or_collateral_count": len(monitored_task["events"]),
                "collateral": (sum(len(item.disturbed)
                                   for item in outcome.physical.results)
                               + len(monitored_task["events"]))})
    finally:
        env.close()

    report = {"schema": "robot-arm-phase10-transaction-matrix/1", "seed": args.seed,
        "provenance": provenance,
        "cases": len(records), "mass_regime": args.mass_regime,
        "mass_sensitivity_g": twin.mass_sensitivity_g,
        "mass_regime_counts": {regime: sum(row.get("mass_regime") == regime for row in records)
                               for regime in ("nominal", "sensitivity_low", "sensitivity_high")},
        "board_friction_scale": args.friction_scale, "records": records,
        "fault_suite": {"included": False,
            "note": "uncertainty, interruption and promotion containment must be qualified separately"},
        "completed": sum(row.get("complete", False) for row in records),
        "commit_allowed": sum(row.get("commit_allowed", False) for row in records),
        "wrong_piece_picks_final_pose": sum(
            row.get("wrong_piece_picks_final_pose", 0) for row in records),
        "transient_wrong_piece_or_collateral_events": sum(
            row.get("transient_wrong_piece_or_collateral_count", 0) for row in records),
        "final_pose_collateral": sum(row.get("final_pose_collateral", 0)
                                      for row in records),
        "collateral": sum(row.get("collateral", 0) for row in records),
        "buckets": {category: {"cases": sum(row["category"] == category for row in records),
            "completed": sum(row.get("complete", False) and row["category"] == category
                             for row in records),
            "commit_allowed": sum(row.get("commit_allowed", False)
                                  and row["category"] == category for row in records)}
            for category in sorted({scenario[0] for scenario in
                                   (scenarios.values() if isinstance(scenarios, dict) else scenarios)})}}
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "records"}, indent=2))
    print(f"receipt: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
