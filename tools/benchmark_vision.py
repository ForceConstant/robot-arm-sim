"""Measure calibrated image targeting and localized pickup in held-out layouts.

Example: ``MUJOCO_GL=egl PYTHONPATH=. .venv/bin/python tools/benchmark_vision.py
--cases 12 --out .private/vision-localization.json``. Simulator piece poses are
used only to inject controlled offsets and score the final attempt; detector
and controller inputs are camera frames, calibration, and the named task.
"""
from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

import mujoco
import numpy as np
import chess

from chess_sim.config import Camera, EnvConfig, RandomizationConfig
from chess_sim.env import ChessSimEnv
from chess_sim.tasks import MoveTask
from chess_sim.vision import detect_piece_base
from chess_sim.vision_geometry import CameraCalibration

LAYOUTS = ("e4", "d4", "e5", "d5", "c3", "f6", "b2", "g7")
PIECE_TYPES = (chess.PAWN, chess.ROOK, chess.KNIGHT,
               chess.BISHOP, chess.QUEEN, chess.KING)
OFFSETS_MM = ((0, 0), (6, 0), (-6, 0), (0, 6), (0, -6), (4, 4), (-4, -4), (6, 6))
YAW_DEG = (0, 45, 90, 135, 30, 75, 120, 165)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=int, default=8)
    parser.add_argument("--seed", type=int, default=74123)
    parser.add_argument("--camera", choices=("rig", "random"), default="random",
                        help="published fixed overhead rig or randomized viewpoints")
    parser.add_argument("--reobserve-camera", choices=("top", "external"), default="top",
                        help="camera used for the post-motion observation")
    parser.add_argument("--out", default=".private/vision-localization.json")
    args = parser.parse_args()
    if args.cases < 1:
        parser.error("--cases must be positive")

    rng = random.Random(args.seed)
    env = ChessSimEnv(EnvConfig(randomization=RandomizationConfig(
        board_shift=0.005, board_yaw=math.radians(5), board_side=0.005,
        arm_joint_jitter=0.0,
        camera=args.camera == "random")))
    records = []
    false_targets = 0
    try:
        for index in range(args.cases):
            source = LAYOUTS[index % len(LAYOUTS)]
            piece_type = PIECE_TYPES[index % len(PIECE_TYPES)]
            target = source[0] + str(min(8, int(source[1]) + 1))
            task = MoveTask(from_square=source, to_square=target)
            position = chess.Board(None)
            position.set_piece_at(chess.parse_square(source), chess.Piece(piece_type, chess.WHITE))
            env.reset(position, rng=rng)
            slot = env.slot_at(source)
            assert slot is not None
            nominal = np.asarray(env.board.square_center(source))
            offset_mm = OFFSETS_MM[index % len(OFFSETS_MM)]
            offset = np.asarray(offset_mm, dtype=float) / 1000.0
            yaw_deg = YAW_DEG[index % len(YAW_DEG)]
            yaw = math.radians(yaw_deg)
            start_xy = nominal + offset
            env.set_piece_pose(slot,
                [start_xy[0] - slot.geometry.center[0],
                 start_xy[1] - slot.geometry.center[1],
                 env.board.top - slot.geometry.bottom_offset + 0.0015],
                [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)], zero_velocity=True)
            mujoco.mj_forward(env.model, env.data)
            frame = env.render(Camera.TOP).copy()
            from PIL import Image
            Image.fromarray(frame).save(f".private/vision-case-{index}-initial.png")
            calibration = CameraCalibration.from_mujoco(
                env.model, env.data, str(Camera.TOP), frame.shape[1], frame.shape[0])
            external_frame = env.render(Camera.EXTERNAL).copy()
            external_calibration = CameraCalibration.from_mujoco(
                env.model, env.data, str(Camera.EXTERNAL),
                external_frame.shape[1], external_frame.shape[0])

            # Move the piece out of the board ROI to capture the same randomized
            # board/camera as an empty reference, then restore only for scoring.
            saved_pose = env.data.qpos[env._slot_qpos[slot.body]:
                                       env._slot_qpos[slot.body] + 7].copy()
            tray = env.board.graveyard_slot(0)
            env.set_piece_pose(slot,
                [tray[0] - slot.geometry.center[0], tray[1] - slot.geometry.center[1],
                 env.board.table_top - slot.geometry.bottom_offset + 0.0015],
                [1, 0, 0, 0], zero_velocity=True)
            mujoco.mj_forward(env.model, env.data)
            empty = env.render(Camera.TOP).copy()
            external_empty = env.render(Camera.EXTERNAL).copy()
            Image.fromarray(empty).save(f".private/vision-case-{index}-empty.png")
            env.data.qpos[env._slot_qpos[slot.body]:env._slot_qpos[slot.body] + 7] = saved_pose
            mujoco.mj_forward(env.model, env.data)
            frame = env.render(Camera.TOP).copy()
            external_frame = env.render(Camera.EXTERNAL).copy()

            arm_geom_ids = []
            for geom_id in range(env.model.ngeom):
                body_id = int(env.model.geom_bodyid[geom_id])
                body_name = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                if body_name and body_name.startswith("so101:"):
                    arm_geom_ids.append(geom_id)

            def arm_mask(camera_name):
                segmentation = env.render_segmentation(camera_name)
                return np.isin(segmentation[:, :, 0], arm_geom_ids)

            def detect(image, camera_calibration, reference, exclusion=None):
                return detect_piece_base(
                image, reference, camera_calibration, tuple(nominal), env.board.top,
                slot.geometry.height, search_radius_m=0.75 * env.board.square,
                exclusion_mask=exclusion)
            selected_camera = (Camera.EXTERNAL if args.reobserve_camera == "external"
                               else Camera.TOP)
            selected_calibration = (external_calibration
                                    if args.reobserve_camera == "external"
                                    else calibration)
            selected_empty = external_empty if args.reobserve_camera == "external" else empty
            selected_initial = external_frame if args.reobserve_camera == "external" else frame
            false_target = detect(selected_empty, selected_calibration,
                                  selected_empty) is not None
            if false_target:
                false_targets += 1
            initial = detect(selected_initial, selected_calibration, selected_empty,
                             arm_mask(selected_camera))
            if initial is None or not initial.acceptable():
                records.append({"case": index, "source": source, "offset_mm": list(offset_mm),
                                "piece_type": chess.piece_name(piece_type),
                                "yaw_deg": yaw_deg, "detected": False, "abstained": True,
                                "false_target": false_target,
                                "success": False, "localization_error_mm": None,
                                "collateral": None})
                continue
            localization_error = float(np.linalg.norm(
                np.asarray(initial.world_xy) - start_xy) * 1000.0)
            reobservations = []
            truth_at_reobserve = []
            piece_yaw_at_reobserve = []
            arm_at_reobserve = []

            def reobserve():
                # Evaluator-only: determine whether arm motion displaced the
                # piece, independently of what the detector reports.
                truth_at_reobserve.append(env.piece_position(slot)[:2].copy())
                q = env.data.qpos[env._slot_qpos[slot.body]:env._slot_qpos[slot.body] + 7][3:7]
                piece_yaw_at_reobserve.append(float(math.atan2(
                    2 * (q[0] * q[3] + q[1] * q[2]),
                    1 - 2 * (q[2] * q[2] + q[3] * q[3]))))
                arm_at_reobserve.append({
                    "joint_error_rad": float(np.linalg.norm(
                        env.arm_joint_positions()[:5] - env.home_pose()[:5])),
                    "max_joint_speed": float(np.max(np.abs(env.data.qvel[env._joint_dof[:5]]))),
                })
                observed_frame = env.render(selected_camera).copy()
                observed_arm_mask = arm_mask(selected_camera)
                Image.fromarray(observed_frame).save(
                    f".private/vision-case-{index}-reobserve-{args.reobserve_camera}.png")
                observation = detect(observed_frame, selected_calibration,
                                     selected_empty, observed_arm_mask)
                if observation is not None:
                    reobservations.append(observation)
                return observation

            result = env.execute_localized(task, initial, reobserve_source=reobserve)
            reobserve_error = (float(np.linalg.norm(
                np.asarray(reobservations[-1].world_xy) - start_xy) * 1000.0)
                if reobservations else None)
            reobserve_truth_error = (float(np.linalg.norm(
                truth_at_reobserve[-1] - start_xy) * 1000.0)
                if truth_at_reobserve else None)
            records.append({
                "case": index, "source": source, "offset_mm": list(offset_mm),
                "piece_type": chess.piece_name(piece_type),
                "yaw_deg": yaw_deg, "board_origin": list(env.board.origin),
                "board_yaw_rad": env.board.yaw,
                "camera_position": list(calibration.camera_position),
                "detected": True, "abstained": False,
                "false_target": false_target,
                "confidence": initial.confidence, "sigma_mm": initial.sigma_m * 1000,
                "initial_pixel": list(initial.pixel_xy),
                "reobserved_pixel": (list(reobservations[-1].pixel_xy)
                                      if reobservations else None),
                "localization_error_mm": localization_error,
                "reobserved": bool(reobservations),
                "reobserved_error_mm": reobserve_error,
                "truth_at_reobserve_error_mm": reobserve_truth_error,
                "arm_at_reobserve": arm_at_reobserve[-1] if arm_at_reobserve else None,
                "piece_yaw_at_reobserve_deg": (math.degrees(piece_yaw_at_reobserve[-1])
                                                if piece_yaw_at_reobserve else None),
                "success": result.success, "placement_error_mm": result.placement_error * 1000,
                "collateral": len(result.disturbed), "picked": str(result.picked) if result.picked else None,
                "reason": str(result.reason) if result.reason else None,
                "reobserve_abstain": bool(env.diagnostic_stall
                    and env.diagnostic_stall.startswith("vision_reobserve_")),
                "reobserve_diagnostic": env.diagnostic_stall,
            })
    finally:
        env.close()

    detected = [r for r in records if r["detected"]]
    errors = [r["localization_error_mm"] for r in detected]
    reobserved_errors = [r["reobserved_error_mm"] for r in records
                         if r.get("reobserved_error_mm") is not None]
    report = {
        "schema": "robot-arm-vision-benchmark/v1", "seed": args.seed,
        "camera_mode": args.camera,
        "cases": len(records), "detected": len(detected),
        "abstained": sum(bool(r["abstained"]) for r in records),
        "localization_error_mm": {
            "mean": float(np.mean(errors)) if errors else None,
            "max": float(np.max(errors)) if errors else None,
        },
        "grasp_successes": sum(bool(r.get("success")) for r in records),
        "collateral_cases": sum(bool(r.get("collateral")) for r in records),
        "false_targets": false_targets,
        "reobserve_abstentions": sum(bool(r.get("reobserve_abstain")) for r in records),
        "reobserved_localization_error_mm": {
            "mean": float(np.mean(reobserved_errors)) if reobserved_errors else None,
            "max": float(np.max(reobserved_errors)) if reobserved_errors else None,
        },
        "records": records,
    }
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "records"}, indent=2))
    print(f"receipt: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
