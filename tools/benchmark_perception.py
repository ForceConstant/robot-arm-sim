"""Held-out board-registration and occupancy/colour evaluation.

Fresh sessions, different seeds from training.  Each session is an empty board
first: that is the calibration reference (never a piece pose).  The board and
camera stay fixed for all positions; registration is estimated once from the
empty frame and a low-confidence fit is reported as an abstention.

Example::

    MUJOCO_GL=egl PYTHONPATH=. .venv/bin/python tools/benchmark_perception.py \
        --sessions 4 --positions 5 --seed 9001 \
        --model chess_sim/assets/perception/model.json \
        --out .private/perception/heldout.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import time
from pathlib import Path

import numpy as np

from chess_sim.config import Camera
from chess_sim.perception import (CLASSES, PerceptionModel, board_patches,
                                  expected_background, normalize_per_frame,
                                  predict_board, square_features,
                                  square_mask_fraction)
from chess_sim.perception_data import (augment_patches, make_session,
                                       register_for_perception, render_mask,
                                       robot_geometry_ids, truth_labels)
from chess_sim.positions import PositionConfig
from chess_sim.vision_geometry import CameraCalibration

LABEL_INDEX = {name: index for index, name in enumerate(CLASSES)}
UNKNOWN = LABEL_INDEX["unknown"]


def _confusion(records: list[dict]) -> list[list[int]]:
    matrix = [[0] * 4 for _ in range(4)]
    for record in records:
        matrix[int(record["truth"])][int(record["predicted"])] += 1
    return matrix


def _summary(records: list[dict]) -> dict:
    total = len(records)
    if not total:
        return {"n": 0}
    answered = [r for r in records if r["predicted"] != UNKNOWN]
    correct = [r for r in answered if r["predicted"] == r["truth"]]
    occupied = [r for r in records if r["truth"] in (1, 2)]
    occ_answered = [r for r in occupied if r["predicted"] != UNKNOWN]
    occ_correct = [r for r in occ_answered if r["predicted"] == r["truth"]]
    empty = [r for r in records if r["truth"] == 0]
    empty_answered = [r for r in empty if r["predicted"] != UNKNOWN]
    empty_correct = [r for r in empty_answered if r["predicted"] == 0]
    return {
        "n": total,
        "answered": len(answered),
        "coverage": round(len(answered) / total, 4),
        "answered_accuracy": round(len(correct) / max(1, len(answered)), 4),
        "confident_errors": len(answered) - len(correct),
        "confident_error_rate": round((len(answered) - len(correct)) / total, 4),
        "unknown_rate": round(1 - len(answered) / total, 4),
        "occupied_n": len(occupied),
        "occupied_coverage": round(len(occ_answered) / max(1, len(occupied)), 4),
        "occupied_answered_accuracy": round(
            len(occ_correct) / max(1, len(occ_answered)), 4),
        "empty_n": len(empty),
        "empty_coverage": round(len(empty_answered) / max(1, len(empty)), 4),
        "empty_answered_accuracy": round(
            len(empty_correct) / max(1, len(empty_answered)), 4),
    }


def _registration_summary(errors: list[dict]) -> dict:
    if not errors:
        return {"n": 0}
    origin = np.array([e["origin_error_mm"] for e in errors], dtype=float)
    yaw = np.array([abs(e["yaw_error_deg"]) for e in errors], dtype=float)
    square = np.array([abs(e["square_error_mm"]) for e in errors], dtype=float)
    return {
        "n": len(errors),
        "origin_error_mm": {"median": round(float(np.median(origin)), 2),
                            "mean": round(float(origin.mean()), 2),
                            "max": round(float(origin.max()), 2)},
        "yaw_error_deg": {"median": round(float(np.median(yaw)), 3),
                          "max": round(float(yaw.max()), 3)},
        "square_error_mm": {"median": round(float(np.median(square)), 2),
                            "max": round(float(square.max()), 2)},
        "within_5mm_1deg": round(float(np.mean((origin <= 5.0) & (yaw <= 1.0))), 4),
    }


def _square_records(predictions, truth: np.ndarray, arm_fraction: np.ndarray,
                    valid: np.ndarray) -> list[dict]:
    records = []
    for prediction in predictions:
        square = prediction.square
        records.append({
            "square": square,
            "truth": int(truth[square]),
            "predicted": LABEL_INDEX[prediction.label],
            "confidence": round(prediction.confidence, 4),
            "valid_fraction": round(float(valid[square]), 3),
            "arm_fraction": round(float(arm_fraction[square]), 3),
            "occluded": bool(arm_fraction[square] > 0.25),
        })
    return records


def _stress(patches, valid, reference, truth, model, rng) -> dict:
    np_rng = np.random.default_rng(rng.getrandbits(32))
    chosen = np_rng.choice(64, size=8, replace=False)
    stressed, _ = augment_patches(patches, np.zeros(64, dtype=np.int64), np_rng,
                                  occluders=8, shadows=0)
    expected = expected_background(stressed, valid)
    features = normalize_per_frame(
        square_features(stressed, valid, expected=expected, reference=reference),
        np.zeros(64, dtype=int))
    indices, confidence, _ = model.predict(features)
    abstained = sum(1 for square in chosen if indices[square] == UNKNOWN)
    # shadow-only stress: labels unchanged; measure answered accuracy
    shadowed, _ = augment_patches(patches, np.zeros(64, dtype=np.int64), np_rng,
                                  occluders=0, shadows=8)
    expected = expected_background(shadowed, valid)
    features = normalize_per_frame(
        square_features(shadowed, valid, expected=expected, reference=reference),
        np.zeros(64, dtype=int))
    indices, _, _ = model.predict(features)
    answered = correct = 0
    for square in range(64):
        if truth[square] in (0, 1, 2) and indices[square] != UNKNOWN:
            answered += 1
            correct += int(indices[square] == truth[square])
    return {
        "occluders": {"squares": [int(v) for v in chosen],
                      "abstained": int(abstained),
                      "abstention_rate": round(abstained / 8.0, 4)},
        "shadows": {"answered": answered,
                    "answered_accuracy": round(correct / max(1, answered), 4)},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sessions", type=int, default=4)
    parser.add_argument("--positions", type=int, default=5)
    parser.add_argument("--seed", type=int, default=9001)
    parser.add_argument("--model", default="chess_sim/assets/perception/model.json")
    parser.add_argument("--out", default=".private/perception/heldout.json")
    args = parser.parse_args()
    if args.sessions < 1 or args.positions < 1:
        parser.error("sessions and positions must be positive")

    model = PerceptionModel.load(args.model)
    started = time.time()
    top_records: list[dict] = []
    wrist_records: list[dict] = []
    registration_errors: list[dict] = []
    session_report = []
    stress_report = []
    rng = random.Random(args.seed)
    skipped = 0

    for session_index in range(args.sessions):
        scene = make_session(args.seed * 1000 + session_index, random_camera=False)
        env = scene.env
        try:
            arm_ids = robot_geometry_ids(env)
            env.reset("8/8/8/8/8/8/8/8 w - - 0 1", rng=rng)
            reference_frame = env.render(Camera.TOP).copy()
            calibration = CameraCalibration.from_mujoco(
                env.model, env.data, str(Camera.TOP),
                reference_frame.shape[1], reference_frame.shape[0])
            registration = register_for_perception(
                reference_frame, calibration, env.board.top, env.board.square)
            if registration is None:
                skipped += 1
                session_report.append({"session": session_index,
                                       "registered": False})
                continue
            env.config = env.config.model_copy(update={"board": env.board})
            reference_patches, reference_valid = board_patches(
                reference_frame, calibration, registration)
            wrist_reference_frame = env.render(Camera.WRIST).copy()
            wrist_cal = CameraCalibration.from_mujoco(
                env.model, env.data, str(Camera.WRIST),
                wrist_reference_frame.shape[1], wrist_reference_frame.shape[0])
            wrist_reference_patches, _ = board_patches(
                wrist_reference_frame, wrist_cal, registration)
            registration_errors.append({
                "origin_error_mm": float(np.hypot(
                    registration.origin[0] - env.board.origin[0],
                    registration.origin[1] - env.board.origin[1]) * 1000.0),
                "yaw_error_deg": float(math.degrees(registration.yaw - env.board.yaw)),
                "square_error_mm": float((registration.square - env.board.square) * 1000.0),
                "score": registration.score, "edge": registration.edge_score,
            })

            rows = []
            for position_index in range(args.positions):
                position = PositionConfig(min_extra_pieces=2, max_extra_pieces=8).sample(rng)
                env.reset(position, rng=None)
                frame = env.render(Camera.TOP).copy()
                truth = truth_labels(env)
                arm_fraction = square_mask_fraction(
                    render_mask(env, Camera.TOP, arm_ids), calibration, registration)
                patches, valid = board_patches(frame, calibration, registration)
                predictions = predict_board(frame, calibration, registration, model,
                                            reference_frame=reference_frame)
                records = _square_records(predictions, truth, arm_fraction, valid)
                top_records.extend(records)
                if position_index % 2 == 0:
                    stress_report.append({
                        "session": session_index, "position": position_index,
                        **_stress(patches, valid, reference_patches, truth, model, rng),
                    })

                wrist_frame = env.render(Camera.WRIST).copy()
                wrist_predictions = predict_board(
                    wrist_frame, wrist_cal, registration, model, view="wrist",
                    reference_frame=wrist_reference_frame)
                _, wrist_valid = board_patches(wrist_frame, wrist_cal, registration)
                wrist_occlusion = square_mask_fraction(
                    render_mask(env, Camera.WRIST, arm_ids), wrist_cal, registration)
                for prediction in wrist_predictions:
                    square = prediction.square
                    if wrist_valid[square] < 0.55 or wrist_occlusion[square] > 0.25:
                        continue
                    wrist_records.append({
                        "square": square,
                        "truth": int(truth[square]),
                        "predicted": LABEL_INDEX[prediction.label],
                        "confidence": round(prediction.confidence, 4),
                    })
                rows.append({"position": position_index,
                             "answer_rate": round(
                                 sum(1 for r in records if r["predicted"] != UNKNOWN) / 64, 3)})
            session_report.append({
                "session": session_index, "seed": scene.seed,
                "board_variant": scene.board_variant,
                "registration": registration_errors[-1],
                "positions": rows,
            })
        finally:
            scene.close()

    unoccluded = [r for r in top_records if not r["occluded"]]
    occluded = [r for r in top_records if r["occluded"]]
    report = {
        "schema": "robot-arm-perception-benchmark/v2",
        "seed": args.seed,
        "model": args.model,
        "model_sha256": hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
        "model_metadata": model.metadata,
        "scope": "fixed overhead rig, bounded board placement, empty-board reference",
        "sessions": args.sessions, "positions": args.positions,
        "sessions_registered": args.sessions - skipped,
        "sessions_skipped": skipped,
        "registration": _registration_summary(registration_errors),
        "top": {
            "all": _summary(top_records),
            "unoccluded": _summary(unoccluded),
            "occluded": _summary(occluded),
            "confusion": _confusion(top_records),
            "unoccluded_confusion": _confusion(unoccluded),
            "occluded_abstention_rate": round(
                sum(1 for r in occluded if r["predicted"] == UNKNOWN)
                / max(1, len(occluded)), 4),
        },
        "wrist": {"all": _summary(wrist_records), "confusion": _confusion(wrist_records)},
        "stress": {
            "occlusion_abstention_rate": round(float(np.mean(
                [s["occluders"]["abstention_rate"] for s in stress_report])), 4)
                if stress_report else None,
            "shadow_answered_accuracy": round(float(np.mean(
                [s["shadows"]["answered_accuracy"] for s in stress_report])), 4)
                if stress_report else None,
            "frames": stress_report,
        },
        "session_report": session_report,
        "records": top_records,
        "wrist_records": wrist_records,
        "elapsed_s": round(time.time() - started, 1),
    }
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items()
                      if k not in ("records", "wrist_records", "session_report",
                                   "stress", "model_metadata")}, indent=2))
    print(f"receipt: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
