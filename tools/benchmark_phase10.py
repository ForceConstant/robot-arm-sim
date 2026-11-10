"""Held-out wrist-only primitive-task stress benchmark for Phase 10.

The task supplies a source square and task family, as it will after Phase 9
move inference. A saved hand-eye/lens calibration plus the selected scan-plan
pose map the RGB wrist frame to board coordinates. Simulator state is used
only to construct controlled perturbations and score outcomes.

Example:
  MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
  ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
  ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
  PYTHONPATH=. .venv/bin/python tools/benchmark_phase10.py --cases 24
"""
from __future__ import annotations

import argparse
import chess
import json
import math
import random
from pathlib import Path

import mujoco
import numpy as np
from chess_sim import gripper
import cv2

from chess_sim.config import Camera, RandomizationConfig
from chess_sim import gripper
from chess_sim.controller import grasp_plan
from chess_sim.env import ChessSimEnv
from chess_sim.perception_data import render_mask, robot_geometry_ids
from chess_sim.product import layout_env_config
from chess_sim.product import apply_mass_model, load_product_twin
from chess_sim.tasks import CaptureTask, MoveTask
from chess_sim.vision import detect_piece_base
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.fiducials import MarkerLayout, estimate_board_pose
from chess_sim.wrist_scan import ScanFrame, ScanPose, load_plan
from chess_sim.vision_geometry import CameraCalibration, Localization, localize_piece_base

PIECE_TYPES = (chess.PAWN, chess.ROOK, chess.KNIGHT,
               chess.BISHOP, chess.QUEEN, chess.KING)
OFFSETS_MM = ((0, 0), (6, 0), (-6, 0), (0, 6), (0, -6), (4, 4), (-4, -4), (6, 6))
YAW_DEG = (0, 45, 90, 135, 30, 75, 120, 165)
CENTROID_HEIGHT_FRACTION = 0.37
# The synthetic placement matrix displaces a piece by at most 6 mm on either
# board axis (8.5 mm diagonally). A fused observation outside 11 mm of the
# registered square center is therefore not a plausible pose for this matrix;
# keep it as a detected candidate, but force the transaction to abstain.
MAX_SOURCE_DEVIATION_M = 0.011


def _move_arm(env: ChessSimEnv, joints: np.ndarray, steps: int = 24) -> None:
    """Move and hold via servo dynamics; never edit qpos/qvel to settle a view.

    Observation motion must not touch any active piece. Latch the first contact
    or displacement so the caller abstains instead of continuing to a grasp.
    Position snapshots here are scoring/safety instrumentation, not detector
    inputs; camera calibration still uses measured joints.
    """
    if getattr(env, '_scan_fault', None):
        return
    start = env.arm_joint_positions().copy()
    active = list(env.active_slots())
    before = {slot.body: env.piece_position(slot)[:2].copy() for slot in active}
    active_bodies = {env._slot_body[slot.body] for slot in active}
    robot_geoms = set(robot_geometry_ids(env))
    if not hasattr(env, '_scan_contact_events'): env._scan_contact_events = []
    if not hasattr(env, '_scan_disturbed'): env._scan_disturbed = set()
    if not hasattr(env, '_scan_motion_records'): env._scan_motion_records = []
    applied = 0
    def advance(action):
        nonlocal applied
        env.apply_action(action)
        applied += 1
        for slot in active:
            if np.linalg.norm(env.piece_position(slot)[:2]-before[slot.body]) > env.config.tolerances.disturbance:
                env._scan_disturbed.add(slot.body)
                env._scan_fault = 'scan_piece_displacement'
        for contact in env.data.contact:
            if contact.dist >= 0: continue
            g1, g2 = int(contact.geom1), int(contact.geom2)
            b1, b2 = int(env.model.geom_bodyid[g1]), int(env.model.geom_bodyid[g2])
            if ((g1 in robot_geoms and b2 in active_bodies) or
                    (g2 in robot_geoms and b1 in active_bodies)):
                env._scan_contact_events.append({'time_s':float(env.data.time),
                    'body1':env.model.body(b1).name,'body2':env.model.body(b2).name,
                    'distance_m':float(contact.dist)})
                env._scan_fault = 'scan_piece_contact'
                break
        if getattr(env, '_scan_fault', None):
            env.diagnostic_stall = env._scan_fault
            return False
        return True
    for alpha in np.linspace(0.0, 1.0, steps + 1)[1:]:
        if not advance(start + alpha * (joints - start)): return
    for _ in range(24):
        if not advance(joints): return
    env._scan_motion_records.append({'applied_control_steps':applied,
        'joint_error_rad':float(np.max(np.abs(env.arm_joint_positions()-joints))),
        'integration_state_edits':False})


def _select_view(env: ChessSimEnv, calibration: WristCalibration,
                 source_xy, board_z: float, height: float, poses,
                 gripper_angle: float):
    """Use the robot's known self-geometry to avoid shadowing the source patch."""
    if getattr(env, "_scan_fault", None):
        return None
    xy = np.asarray(source_xy, dtype=float)
    ranked = []
    for planned_pose in poses:
            joints = np.asarray(planned_pose.joints, dtype=float).copy()
            joints[-1] = gripper_angle
            pose = ScanPose(planned_pose.name, tuple(joints))
            _move_arm(env, np.asarray(pose.joints, dtype=float))
            if getattr(env, "_scan_fault", None):
                return None
            measured_joints = env.arm_joint_positions().copy()
            camera = calibration.camera(measured_joints)
            pixel = camera.project((xy[0], xy[1], board_z + 0.37 * height))
            radius = max(0.006, min(0.010, 0.35 * height))
            corners = [camera.project((xy[0] + dx, xy[1] + dy,
                                       board_z + dz))
                       for dx in (-radius, radius) for dy in (-radius, radius)
                       for dz in (0.0, height)]
            if pixel is None or any(p is None for p in corners):
                continue
            x, y = pixel
            if not (12 <= x < camera.width - 12 and 12 <= y < camera.height - 12):
                continue
            mask = render_mask(env, Camera.WRIST, robot_geometry_ids(env))
            projected = np.asarray(corners)
            x0 = max(0, int(np.floor(projected[:, 0].min())))
            x1 = min(camera.width, int(np.ceil(projected[:, 0].max())) + 1)
            y0 = max(0, int(np.floor(projected[:, 1].min())))
            y1 = min(camera.height, int(np.ceil(projected[:, 1].max())) + 1)
            if x0 >= x1 or y0 >= y1:
                continue
            occlusion = float(np.mean(mask[y0:y1, x0:x1]))
            center_distance = ((x - camera.width / 2) ** 2
                               + (y - camera.height / 2) ** 2) ** 0.5
            predicted = localize_piece_base(camera, pixel, board_z, 1.0,
                height_sigma_m=0.2 * height)
            if predicted is None:
                continue
            ranked.append((occlusion, predicted.sigma_m, center_distance, pose, camera))
    if not ranked:
        return None
    visible = [row for row in ranked if row[0] <= 0.08]
    if not visible:
        return None
    occlusion, predicted_sigma, _, pose, _ = min(visible,
        key=lambda row: (row[1], row[2], row[0]))
    _move_arm(env, np.asarray(pose.joints, dtype=float))
    camera = calibration.camera(env.arm_joint_positions())
    return pose, camera, occlusion, predicted_sigma


def _case(index: int, rng: random.Random) -> dict:
    piece_type = PIECE_TYPES[index % len(PIECE_TYPES)]
    color = chess.WHITE if (index // len(PIECE_TYPES)) % 2 == 0 else chess.BLACK
    source = rng.choice(tuple(chess.SQUARES))
    target_candidates = [sq for sq in chess.SQUARES if sq != source]
    rng.shuffle(target_candidates)
    capture = index % 4 == 3
    # Dense starts retain every original position; mixed cases span 8–24 pieces.
    crowded = index % 3 == 0
    board = chess.Board(None)
    # Respect the actual printed pack inventory: 8/2/2/2/1/1 per color.
    inventory = [(kind, side) for side in (chess.WHITE, chess.BLACK)
                 for kind, count in zip(PIECE_TYPES, (8, 2, 2, 2, 1, 1))
                 for _ in range(count)]
    inventory.remove((piece_type, color))
    if crowded:
        filler_count = 20 + (index % 13)
    else:
        filler_count = 7 + (index % 18)
    rng.shuffle(inventory)
    fillers = [sq for sq in chess.SQUARES if sq != source]
    rng.shuffle(fillers)
    for n, (kind, piece_color) in enumerate(inventory[:filler_count]):
        board.set_piece_at(fillers[n], chess.Piece(kind, piece_color))
    target_candidates = [sq for sq in target_candidates if board.piece_at(sq) is None]
    if not target_candidates:
        target_candidates = [sq for sq in chess.SQUARES if sq != source and board.piece_at(sq) is None]
    if not target_candidates:
        target_candidates = [sq for sq in chess.SQUARES if sq != source]
    board.set_piece_at(source, chess.Piece(piece_type, color))
    target = target_candidates[0]
    return {"index": index, "source": chess.square_name(source),
            "target": chess.square_name(target), "piece_type": chess.piece_name(piece_type),
            "color": "white" if color else "black",
            "family": "capture" if capture else "move",
            "crowding": "dense" if crowded else "mixed",
            "victim_square": None,
            "board": board.board_fen(), "offset_mm": list(OFFSETS_MM[index % len(OFFSETS_MM)]),
            "yaw_deg": YAW_DEG[index % len(YAW_DEG)]}


def _pose_source(env: ChessSimEnv, source: str, record: dict) -> None:
    slot = env.slot_at(source)
    assert slot is not None
    center = np.asarray(env.board.square_center(source))
    offset = np.asarray(record["offset_mm"], dtype=float) / 1000.0
    yaw = math.radians(record["yaw_deg"])
    xy = center + offset
    env.set_piece_pose(slot,
        [xy[0] - slot.geometry.center[0], xy[1] - slot.geometry.center[1],
         env.board.top - slot.geometry.bottom_offset + 0.0015],
        [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)], zero_velocity=True)
    mujoco.mj_forward(env.model, env.data)


def _align_empty_reference(reference: np.ndarray, source_camera, target_camera,
                           registration) -> np.ndarray:
    """Warp a same-epoch empty image into the current measured wrist view."""
    board_points = [(*registration.square_center(square), registration.board_z)
                    for square in ("a1", "h1", "a8", "h8")]
    src = [source_camera.project(point) for point in board_points]
    dst = [target_camera.project(point) for point in board_points]
    if any(point is None for point in (*src, *dst)):
        return reference
    homography = cv2.getPerspectiveTransform(np.asarray(src, dtype=np.float32),
                                              np.asarray(dst, dtype=np.float32))
    return cv2.warpPerspective(reference, homography,
        (target_camera.width, target_camera.height), flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=int, default=24)
    parser.add_argument("--start-case", type=int, default=0,
                        help="absolute matrix index for isolated bucket diagnostics")
    parser.add_argument("--seed", type=int, default=20261103)
    parser.add_argument("--calibration", default=".private/phase9/calibration.json")
    parser.add_argument("--mass-regime", choices=("nominal", "heavy_bound"), default="nominal")
    parser.add_argument("--friction-scale", type=float, default=1.0,
                        help="scale board sliding friction for sensitivity runs")
    parser.add_argument("--out", default=".private/phase10/pilot.json")
    parser.add_argument("--debug-dir", default=None,
                        help="optional directory for one first-case frame pair and truth metadata")
    parser.add_argument("--height-sweep", action="store_true",
                        help="calibrate silhouette centroid height by printed piece type; skip execution")
    args = parser.parse_args()
    if args.cases < 1 or args.start_case < 0:
        parser.error("--cases must be positive and --start-case non-negative")
    if not math.isfinite(args.friction_scale) or not 0.25 <= args.friction_scale <= 2.5:
        parser.error("--friction-scale must be between 0.25 and 2.5")

    calibration = WristCalibration.load(args.calibration)
    poses = load_plan()
    env = ChessSimEnv(layout_env_config("printed", cameras=(Camera.WRIST,)).model_copy(
        update={"randomization": RandomizationConfig(
            board_shift=0.008, board_yaw=math.radians(2.5), board_side=0.008,
            arm_joint_jitter=0.0, camera=False)}))
    baseline_board = env.config.board
    apply_mass_model(env, load_product_twin(), args.mass_regime, all_slots=True)
    env.model.geom("board").friction[0] *= args.friction_scale
    rng = random.Random(args.seed)
    # Isolated bucket diagnostics must reproduce the same board/layout draw
    # as that absolute index in a full matrix, not merely reuse its piece type.
    for skipped_index in range(args.start_case):
        _case(skipped_index, rng)
    records = []
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)

    def checkpoint(record: dict) -> None:
        """Atomically preserve completed cases during long held-out runs."""
        # This harness calls one primitive task, not the caller-facing chess
        # transaction coordinator. It cannot claim pre-commit detection.
        record.setdefault("transaction_commit_gate_exercised", False)
        record.setdefault("phase10_complete", False)
        records.append(record)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(json.dumps({
            "schema": "robot-arm-phase10-checkpoint/2", "seed": args.seed,
            "qualification_scope": "primitive-task stress; chess transaction commit gate not exercised",
            "start_case": args.start_case, "requested_cases": args.cases,
            "piece_set": "printed", "mass_regime": args.mass_regime,
            "board_friction_scale": args.friction_scale,
            "completed_cases": len(records), "records": records}, indent=2) + "\n")
        temporary.replace(output)

    try:
        for index in range(args.start_case, args.start_case + args.cases):
            env._scan_fault=None; env._scan_contact_events=[]; env._scan_disturbed=set(); env._scan_motion_records=[]
            # Each held-out case is an independent layout draw. ``env.config``
            # is temporarily updated to the realized board below so all resets
            # within that case share its pose; restore the immutable product
            # baseline here to prevent random shifts/yaw accumulating between
            # cases in a long matrix.
            env.config = env.config.model_copy(update={"board": baseline_board})
            case_seed = args.seed + index * 7919
            case_rng = random.Random(case_seed)
            case = _case(index, case_rng)
            empty_rng = random.Random(case_seed)
            env.reset(chess.Board(None), rng=empty_rng)
            # Hold board pose for scene construction/scoring only. The source
            # coordinate passed to vision comes from marker pixels below.
            env.config = env.config.model_copy(update={"board": env.board})
            requested_piece = chess.Piece(PIECE_TYPES[index % len(PIECE_TYPES)],
                chess.WHITE if (index // len(PIECE_TYPES)) % 2 == 0 else chess.BLACK)
            source_slot = next(slot for slot in env.slots if slot.piece == requested_piece)
            open_angle = gripper.gap_to_angle(
                grasp_plan(source_slot.geometry, env.board).gap_open)
            registration_frames = []
            for planned_pose in poses:
                joints = np.asarray(planned_pose.joints, dtype=float).copy()
                joints[-1] = open_angle
                scan_pose = ScanPose(planned_pose.name, tuple(joints))
                _move_arm(env, joints)
                measured = tuple(float(v) for v in env.arm_joint_positions())
                registration_frames.append(ScanFrame(
                    env.render(Camera.WRIST).copy(), measured, 0.0, scan_pose.name))
            registration = estimate_board_pose(
                [(calibration.rectify(item.frame), calibration.camera(item.joints))
                 for item in registration_frames],
                env.board.square, env.board.top,
                marker_z=env.board.table_top + env.board.marker_above_table,
                layout=MarkerLayout.for_board(env.board.field * 1000.0,
                    env.board.border * 1000.0, env.board.marker_plane))
            if registration is None:
                checkpoint({**case, "registered": False, "detected": False,
                    "abstained": True, "success": False,
                    "reason": "wrist marker registration failed"})
                continue
            nominal = registration.square_center(case["source"])
            view = _select_view(env, calibration, nominal, registration.board_z,
                                source_slot.geometry.height, poses, open_angle)
            if view is None:
                checkpoint({**case, "registered": True, "detected": False,
                                "abstained": True,
                                "success": False, "reason": "no calibrated view"})
                continue
            pose, camera_cal, view_occlusion, predicted_sigma = view
            # Keep one same-epoch, wrist-only empty reference per scan pose.
            # Later task localization and re-observation fuse these calibrated
            # views instead of letting one silhouette decide a crowded grasp.
            references_by_pose = {item.pose: calibration.rectify(item.frame)
                                  for item in registration_frames}
            cameras_by_pose = {item.pose: calibration.camera(item.joints)
                               for item in registration_frames}
            env.home_pose = lambda p=pose: np.asarray(p.joints, dtype=float)
            env.reset(case["board"])
            _pose_source(env, case["source"], case)
            # Reset parks at the task-conditioned observation pose selected
            # from the registered wrist plan.
            initial = calibration.rectify(env.render(Camera.WRIST).copy())
            arm_mask = render_mask(env, Camera.WRIST, robot_geometry_ids(env))
            measured_camera = calibration.camera(env.arm_joint_positions())
            slot = env.slot_at(case["source"])
            centroid_height_fraction = CENTROID_HEIGHT_FRACTION

            def fuse_piece_views() -> tuple[Localization | None, list[dict]]:
                candidates = []
                for planned_pose in poses:
                    joints = np.asarray(planned_pose.joints, dtype=float).copy()
                    joints[-1] = open_angle
                    _move_arm(env, joints)
                    frame = calibration.rectify(env.render(Camera.WRIST).copy())
                    mask = render_mask(env, Camera.WRIST, robot_geometry_ids(env))
                    camera = calibration.camera(env.arm_joint_positions())
                    reference = _align_empty_reference(
                        references_by_pose[planned_pose.name],
                        cameras_by_pose[planned_pose.name], camera, registration)
                    found = detect_piece_base(
                        frame, reference, camera, nominal, registration.board_z,
                        slot.geometry.height, search_radius_m=0.018,
                        centroid_height_fraction=centroid_height_fraction,
                        exclusion_mask=mask)
                    if found is not None:
                        candidates.append((found,
                            ScanPose(planned_pose.name, tuple(joints)),
                            frame, mask, camera))
                if not candidates:
                    return None, []
                points = np.asarray([row[0].world_xy for row in candidates])
                fused_xy = np.median(points, axis=0)
                distances = np.linalg.norm(points - fused_xy, axis=1)
                spread_sigma = (1.2533 * 1.4826 * float(np.median(distances))
                                / math.sqrt(len(candidates)))
                best = min(candidates, key=lambda row: row[0].sigma_m)
                source_deviation = float(np.linalg.norm(fused_xy - nominal))
                fused_sigma = max(min(row[0].sigma_m for row in candidates), spread_sigma)
                if source_deviation > MAX_SOURCE_DEVIATION_M:
                    fused_sigma = max(fused_sigma, 0.0041)
                fused = Localization(tuple(map(float, fused_xy)), best[0].pixel_xy,
                    float(np.median([row[0].confidence for row in candidates])),
                    fused_sigma)
                _move_arm(env, np.asarray(best[1].joints, dtype=float))
                details = [{"pose": row[1].name,
                    "world_xy": list(row[0].world_xy),
                    "sigma_mm": row[0].sigma_m * 1000,
                    "confidence": row[0].confidence} for row in candidates]
                return fused, details

            localization, localization_views = fuse_piece_views()
            true_xy = env.piece_position(slot)[:2].copy()
            if args.height_sweep:
                sweep = []
                initial_reference = _align_empty_reference(
                    references_by_pose[pose.name], cameras_by_pose[pose.name],
                    measured_camera, registration)
                for fraction in np.linspace(0.05, 0.95, 19):
                    candidate = detect_piece_base(initial, initial_reference,
                        measured_camera, nominal, registration.board_z, slot.geometry.height,
                        search_radius_m=0.018, centroid_height_fraction=float(fraction),
                        exclusion_mask=arm_mask)
                    sweep.append({"fraction": float(fraction),
                        "error_mm": (float(np.linalg.norm(
                            np.asarray(candidate.world_xy) - true_xy) * 1000)
                            if candidate else None),
                        "sigma_mm": (candidate.sigma_m * 1000 if candidate else None)})
                checkpoint({**case, "registered": True,
                    "registration_rms_px": registration.score,
                    "registration_coverage": registration.coverage,
                    "pose": pose.name,
                    "view_occlusion": view_occlusion,
                    "predicted_sigma_mm": predicted_sigma * 1000,
                    "true_xy": list(true_xy),
                    "height_sweep": sweep,
                    "detected": localization is not None,
                    "abstained": localization is None})
                continue
            if index == args.start_case and args.debug_dir:
                from PIL import Image
                debug = Path(args.debug_dir)
                debug.mkdir(parents=True, exist_ok=True)
                Image.fromarray(references_by_pose[pose.name]).save(debug / "empty.png")
                Image.fromarray(initial).save(debug / "initial.png")
                Image.fromarray(arm_mask.astype(np.uint8) * 255).save(debug / "arm-mask.png")
                (debug / "case.json").write_text(json.dumps({
                    "case": case, "pose": pose.name, "joints": list(pose.joints),
                    "camera": {"position": list(measured_camera.camera_position),
                               "rotation": list(measured_camera.camera_rotation)},
                    "true_xy": list(env.piece_position(slot)[:2]),
                    "nominal_xy": list(nominal),
                    "detected_xy": list(localization.world_xy) if localization else None,
                    "localization_views": localization_views,
                    "pixel": list(localization.pixel_xy) if localization else None,
                    "piece_height": slot.geometry.height}, indent=2) + "\n")
            record = {**case, "registered": True,
                      "transaction_commit_gate_exercised": False,
                      "registration_rms_px": registration.score,
                      "registration_coverage": registration.coverage,
                      "registration_origin_error_mm": float(np.linalg.norm(
                          np.asarray(registration.origin) - np.asarray(env.board.origin)) * 1000),
                      "pose": pose.name, "view_occlusion": view_occlusion,
                      "predicted_sigma_mm": predicted_sigma * 1000,
                      "board_origin": list(env.board.origin),
                      "board_yaw_rad": env.board.yaw,
                      "true_xy": list(env.piece_position(source_slot)[:2]),
                      "detected": localization is not None,
                      "abstained": localization is None,
                      "localization_views": localization_views,
                      "localization_error_mm": (float(np.linalg.norm(
                          np.asarray(localization.world_xy) - true_xy) * 1000)
                          if localization else None),
                      "localization_error_xy_mm": ([float(
                          (localization.world_xy[i] - true_xy[i]) * 1000)
                          for i in (0, 1)] if localization else None),
                      "source_deviation_mm": (float(np.linalg.norm(
                          np.asarray(localization.world_xy) - nominal) * 1000)
                          if localization else None)}
            if localization is None or not localization.acceptable():
                record.update({"success": False, "abstained": True,
                               "confidence": getattr(localization, "confidence", None),
                               "sigma_mm": (localization.sigma_m * 1000 if localization else None),
                               "reason": "vision abstention or uncertainty gate"})
                checkpoint(record)
                continue
            record["confidence"] = localization.confidence
            record["sigma_mm"] = localization.sigma_m * 1000
            record["localization_views"] = localization_views
            record["localization_error_mm"] = float(
                np.linalg.norm(np.asarray(localization.world_xy) - true_xy) * 1000)
            record["localization_error_xy_mm"] = (
                float((localization.world_xy[0] - true_xy[0]) * 1000),
                float((localization.world_xy[1] - true_xy[1]) * 1000))
            reobservations = []
            reobserve_meta = []

            def reobserve():
                detected_again, view_details = fuse_piece_views()
                observed_camera = calibration.camera(env.arm_joint_positions())
                rendered_camera = CameraCalibration.from_mujoco(
                    env.model, env.data, str(Camera.WRIST), 640, 480)
                if detected_again is not None:
                    reobservations.append(detected_again)
                error_mm = (float(np.linalg.norm(
                    np.asarray(detected_again.world_xy) - true_xy) * 1000)
                    if detected_again is not None else None)
                reobserve_meta.append({"joints": env.arm_joint_positions().tolist(),
                    "camera_position": list(observed_camera.camera_position),
                    "rendered_camera_position": list(rendered_camera.camera_position),
                    "rendered_camera_rotation": list(rendered_camera.camera_rotation),
                    "localization_xy": (list(detected_again.world_xy)
                                        if detected_again else None),
                    "pixel": (list(detected_again.pixel_xy) if detected_again else None),
                    "sigma_mm": (detected_again.sigma_m * 1000
                                 if detected_again else None),
                    "confidence": (detected_again.confidence
                                   if detected_again else None),
                    "error_mm": error_mm, "views": view_details})
                return detected_again

            if case["family"] == "capture":
                task = CaptureTask(square=case["source"])
            else:
                task = MoveTask(from_square=case["source"], to_square=case["target"])
            before = env.piece_snapshot()
            diagnostic_trace = []
            diagnostic_dir = (Path(args.debug_dir) if args.debug_dir and
                              index == args.start_case else None)
            original_apply_action = None
            if diagnostic_dir is not None:
                diagnostic_dir.mkdir(parents=True, exist_ok=True)
                from PIL import Image
                Image.fromarray(env.render(Camera.WRIST).copy()).save(
                    diagnostic_dir / "pre-transaction-wrist.png")
                integration = np.empty(mujoco.mj_stateSize(
                    env.model, mujoco.mjtState.mjSTATE_INTEGRATION), dtype=np.float64)
                mujoco.mj_getState(env.model, env.data, integration,
                                   mujoco.mjtState.mjSTATE_INTEGRATION)
                np.savez_compressed(diagnostic_dir / "frame0.npz",
                    integration_state=integration,
                    joint_positions=env.arm_joint_positions(),
                    control_targets=env.data.ctrl.copy(),
                    step_count=np.asarray([env._step_count]),
                    piece_positions=np.asarray([env.piece_position(slot)
                                                for slot in env.slots]))
                original_apply_action = env.apply_action

                def traced_apply_action(action, on_step=None):
                    sample = {"period": len(diagnostic_trace),
                        "sim_time_before": float(env.data.time),
                        "phase": env.diagnostic_phase,
                        "target_tcp": env.diagnostic_target,
                        "commanded_gap": env.diagnostic_gap,
                        "tool_offset": np.asarray(env.diagnostic_offset).tolist(),
                        "ik_verdict": env.diagnostic_ik,
                        "joint_positions_before": env.arm_joint_positions().tolist(),
                        "control_targets_before": env.data.ctrl.copy().tolist(),
                        "intended_action": np.asarray(action, dtype=float).tolist()}
                    def capture_intent(command):
                        sample["callback_action"] = np.asarray(command, dtype=float).tolist()
                        if on_step is not None:
                            on_step(command)
                    original_apply_action(action, capture_intent)
                    sample["sim_time_after"] = float(env.data.time)
                    sample["joint_positions_after"] = env.arm_joint_positions().tolist()
                    sample["control_targets_after"] = env.data.ctrl.copy().tolist()
                    offset = np.asarray(env.diagnostic_offset, dtype=float)
                    sample["tcp_after"] = env.ik.tool_pose(env.data, offset)[0].tolist()
                    sample["jaw_gap_after"] = float(gripper.angle_to_gap(
                        env.arm_joint_positions()[-1]))
                    slot_qpos = env._slot_qpos[slot.body]
                    sample["moving_piece_pose_after"] = env.data.qpos[
                        slot_qpos:slot_qpos + 7].tolist()
                    contacts = []
                    for contact_index in range(env.data.ncon):
                        contact = env.data.contact[contact_index]
                        force = np.zeros(6, dtype=float)
                        mujoco.mj_contactForce(env.model, env.data, contact_index, force)
                        contacts.append({
                            "geom1": mujoco.mj_id2name(env.model,
                                mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                            "geom2": mujoco.mj_id2name(env.model,
                                mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
                            "force_local": force.tolist(),
                            "distance_m": float(contact.dist)})
                    sample["contacts_after_period"] = contacts
                    diagnostic_trace.append(sample)

                env.apply_action = traced_apply_action
            result = env.execute_localized(task, localization, reobserve_source=reobserve)
            if diagnostic_dir is not None:
                from PIL import Image
                Image.fromarray(env.render(Camera.WRIST).copy()).save(
                    diagnostic_dir / "post-transaction-wrist.png")
                (diagnostic_dir / "action-trace.json").write_text(json.dumps({
                    "schema": "robot-arm-phase10-action-diagnostic/1",
                    "seed": args.seed, "case": case,
                    "calibration": args.calibration,
                    "piece_set": "printed", "mass_regime": args.mass_regime,
                    "board_origin": list(env.board.origin),
                    "board_yaw_rad": env.board.yaw,
                    "before_piece_snapshot": {str(k): list(v)
                        for k, v in before.items()},
                    "after_piece_snapshot": {str(k): list(v)
                        for k, v in env.piece_snapshot().items()},
                    "localization": {"xy": list(localization.world_xy),
                        "sigma_mm": localization.sigma_m * 1000,
                        "confidence": localization.confidence,
                        "views": localization_views,
                        "reobservations": reobserve_meta},
                    "result": {"success": bool(result.success),
                        "picked": str(result.picked) if result.picked else None,
                        "right_piece": bool(result.right_piece),
                        "disturbed": list(result.disturbed),
                        "placement_error_mm": float(result.placement_error * 1000),
                        "upright": bool(result.upright),
                        "failure": str(result.reason) if result.reason else None,
                        "diagnostic_stall": env.diagnostic_stall,
                        "steps": result.steps},
                    "control_period_trace": diagnostic_trace}, indent=2) + "\n")
                env.apply_action = original_apply_action
            record.update({"success": bool(result.success),
                           "transaction_commit_gate_exercised": False,
                           "phase10_complete": bool(result.phase10_complete),
                           "released": bool(result.released),
                           "supported": bool(result.supported),
                           "settled": bool(result.settled),
                           "picked": str(result.picked) if result.picked else None,
                           "right_piece": bool(result.right_piece), "collateral": len(result.disturbed),
                           "placement_error_mm": float(result.placement_error * 1000),
                           "upright": bool(result.upright), "failure": str(result.reason) if result.reason else None,
                           "diagnostic": env.diagnostic_stall,
                           "diagnostic_ik": env.diagnostic_ik,
                           "steps": result.steps,
                           "reobserved": bool(reobservations),
                           "reobserve_meta": reobserve_meta,
                           "reobserve_error_mm": (float(np.linalg.norm(
                               np.asarray(reobservations[-1].world_xy) - true_xy) * 1000)
                               if reobservations else None),
                           "reobserve_delta_mm": (float(np.linalg.norm(
                               np.asarray(reobservations[-1].world_xy) -
                               np.asarray(localization.world_xy)) * 1000)
                               if reobservations else None),
                           "reobserve_epoch_reference": "same randomized board/layout and scan pose"})
            checkpoint(record)
    finally:
        env.close()

    detected = [r for r in records if r.get("detected")]
    completed = [r for r in records if r.get("success")]
    phase10_completed = [r for r in records if r.get("phase10_complete")]
    failures = [r for r in records if r.get("detected") and not r.get("success")]
    report = {"schema": "robot-arm-phase10-pilot/2", "seed": args.seed,
              "start_case": args.start_case,
              "piece_set": "printed", "camera_contract": "wrist-only calibrated plan pose",
              "mass_regime": args.mass_regime,
              "board_friction_scale": args.friction_scale,
              "centroid_height_fraction": CENTROID_HEIGHT_FRACTION,
              "cases": len(records), "detected": len(detected),
              "abstentions": sum(bool(r.get("abstained")) for r in records),
              "primitive_tasks_completed": len(completed),
              "phase10_complete_cases": len(phase10_completed),
              "wrong_piece_picks": sum(r.get("right_piece") is False and r.get("picked") is not None for r in records),
              "collateral_cases": sum(bool(r.get("collateral")) for r in records),
              "detected_failures": len(failures),
              "localization_error_mm": {"mean": float(np.mean([r["localization_error_mm"] for r in detected if "localization_error_mm" in r])) if any("localization_error_mm" in r for r in detected) else None,
                                         "max": float(np.max([r["localization_error_mm"] for r in detected if "localization_error_mm" in r])) if any("localization_error_mm" in r for r in detected) else None},
              "buckets": {}, "records": records}
    for key in ("piece_type", "color", "family", "crowding"):
        for value in sorted({r[key] for r in records}):
            group = [r for r in records if r[key] == value]
            report["buckets"][f"{key}:{value}"] = {
                "cases": len(group), "detected": sum(bool(r.get("detected")) for r in group),
                "primitive_tasks_completed": sum(bool(r.get("success")) for r in group),
                "phase10_complete_cases": sum(bool(r.get("phase10_complete")) for r in group),
                "wrong_piece_picks": sum(r.get("right_piece") is False and r.get("picked") is not None for r in group),
                "collateral": sum(bool(r.get("collateral")) for r in group)}
    if args.height_sweep:
        fractions = sorted({row["fraction"] for r in records
                            for row in r.get("height_sweep", [])})
        report["centroid_height_calibration"] = {}
        for piece_type in sorted({r["piece_type"] for r in records}):
            group = [r for r in records if r["piece_type"] == piece_type]
            by_fraction = []
            for fraction in fractions:
                errors = [row["error_mm"] for r in group
                    for row in r.get("height_sweep", [])
                    if row["fraction"] == fraction and row["error_mm"] is not None]
                by_fraction.append({"fraction": fraction, "detected": len(errors),
                    "mean_error_mm": float(np.mean(errors)) if errors else None,
                    "p95_error_mm": float(np.percentile(errors, 95)) if errors else None,
                    "within_4mm": sum(error <= 4 for error in errors)})
            report["centroid_height_calibration"][piece_type] = sorted(
                by_fraction, key=lambda row: (row["mean_error_mm"] is None,
                    row["mean_error_mm"] if row["mean_error_mm"] is not None else float("inf")))
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "records"}, indent=2))
    print(f"receipt: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
