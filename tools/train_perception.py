"""Collect domain-randomised square samples and train the perception model.

Sessions share the calibrated overhead rig but vary board, piece set/scale,
lighting and board placement within the qualified envelope.  Registration is
estimated from each frame and a low-confidence fit is skipped, never used to
label the wrong pixels.  Training/validation is split by whole session.

Examples::

    MUJOCO_GL=egl PYTHONPATH=. .venv/bin/python tools/train_perception.py \
        --sessions 10 --positions 8 --seed 61
"""
from __future__ import annotations

import argparse
import json
import random
import time
from pathlib import Path

import numpy as np

from chess_sim.config import Camera
from chess_sim.perception import (CLASSES, FEATURE_NAMES, board_patches,
                                  calibrate_threshold, expected_background,
                                  normalize_per_frame, square_features,
                                  square_mask_fraction, train_model)
from chess_sim.perception_data import (make_session, register_for_perception,
                                       render_mask, robot_geometry_ids, truth_labels)
from chess_sim.positions import PositionConfig
from chess_sim.vision_geometry import CameraCalibration

CLASS_INDEX = {name: index for index, name in enumerate(CLASSES)}
UNKNOWN = CLASS_INDEX["unknown"]


def collect_session(env, seed: int, positions: int,
                    rng: random.Random) -> tuple[list[np.ndarray], list[int], list[int],
                                                 list[int], list[dict]]:
    """Capture one empty-board reference, then many piece positions.

    The board and camera do not move within a session; the empty reference is
    therefore valid for every position.  This is a calibration observation, not
    piece truth.  A session whose empty board cannot be registered is skipped.
    """
    features_out: list[np.ndarray] = []
    labels_out: list[int] = []
    views_out: list[int] = []
    frames_out: list[int] = []
    stats: list[dict] = []
    frame_index = 0
    np_rng = np.random.default_rng(seed)
    arm_ids = robot_geometry_ids(env)

    # Empty-board reference at the session's placed pose.
    env.reset("8/8/8/8/8/8/8/8 w - - 0 1", rng=rng)
    reference_frame = env.render(Camera.TOP).copy()
    reference_cal = CameraCalibration.from_mujoco(
        env.model, env.data, str(Camera.TOP),
        reference_frame.shape[1], reference_frame.shape[0])
    registration = register_for_perception(reference_frame, reference_cal,
                                           env.board.top, env.board.square)
    if registration is None:
        # A wider local retry, still inside the qualified envelope.
        from chess_sim.board_registration import register_board, BoardRegistration
        registration = register_board(
            reference_frame, reference_cal, env.board.top,
            prior=BoardRegistration((0.0, 0.0), 0.0, env.board.square,
                                    env.board.top, score=0.0, coverage=1.0),
            origin_search=0.024, yaw_search_deg=6.0, coarse_step=0.003)
        if registration is not None and registration.score < 0.35:
            registration = None
    if registration is None:
        return [], [], [], [], [{"reference_registered": False}]
    # Keep this placed pose as the nominal for subsequent resets.
    env.config = env.config.model_copy(update={"board": env.board})
    reference_patches, reference_valid = board_patches(
        reference_frame, reference_cal, registration)

    wrist_reference_frame = env.render(Camera.WRIST).copy()
    wrist_reference_cal = CameraCalibration.from_mujoco(
        env.model, env.data, str(Camera.WRIST),
        wrist_reference_frame.shape[1], wrist_reference_frame.shape[0])
    wrist_reference_patches, _ = board_patches(
        wrist_reference_frame, wrist_reference_cal, registration)

    for index in range(positions):
        position = PositionConfig(min_extra_pieces=2, max_extra_pieces=8).sample(rng)
        env.reset(position, rng=None)
        frame = env.render(Camera.TOP).copy()
        calibration = CameraCalibration.from_mujoco(
            env.model, env.data, str(Camera.TOP), frame.shape[1], frame.shape[0])
        patches, valid = board_patches(frame, calibration, registration)
        arm_fraction = square_mask_fraction(
            render_mask(env, Camera.TOP, arm_ids), calibration, registration)
        clean_labels = truth_labels(env)
        labels = clean_labels.copy()
        labels[arm_fraction > 0.25] = UNKNOWN
        expected = expected_background(patches, valid)
        features = normalize_per_frame(
            square_features(patches, valid, expected=expected,
                            reference=reference_patches),
            np.zeros(64, dtype=int))
        for square in range(64):
            features_out.append(features[square])
            labels_out.append(int(labels[square]))
            views_out.append(0)
            frames_out.append(frame_index)
        frame_index += 1

        wrist_frame = env.render(Camera.WRIST).copy()
        wrist_cal = CameraCalibration.from_mujoco(
            env.model, env.data, str(Camera.WRIST),
            wrist_frame.shape[1], wrist_frame.shape[0])
        wrist_patches, wrist_valid = board_patches(wrist_frame, wrist_cal, registration)
        wrist_occlusion = square_mask_fraction(
            render_mask(env, Camera.WRIST, arm_ids), wrist_cal, registration)
        wrist_expected = expected_background(wrist_patches, wrist_valid)
        wrist_features = normalize_per_frame(
            square_features(wrist_patches, wrist_valid, view="wrist",
                            expected=wrist_expected,
                            reference=wrist_reference_patches),
            np.zeros(64, dtype=int))
        wrist_count = 0
        for square in range(64):
            if wrist_valid[square] < 0.55 or wrist_occlusion[square] > 0.25:
                continue
            if clean_labels[square] not in (0, 1, 2):
                continue
            features_out.append(wrist_features[square])
            labels_out.append(int(clean_labels[square]))
            views_out.append(1)
            frames_out.append(frame_index)
            wrist_count += 1
        frame_index += 1
        stats.append({"position": index, "registered": True,
                      "wrist_squares": wrist_count,
                      "naturally_occluded": int((arm_fraction > 0.25).sum())})
    stats.insert(0, {"reference_registered": True,
                     "reference_score": registration.score,
                     "reference_edge": registration.edge_score})
    return features_out, labels_out, views_out, frames_out, stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sessions", type=int, default=10)
    parser.add_argument("--positions", type=int, default=12)
    parser.add_argument("--seed", type=int, default=61)
    parser.add_argument("--hidden", type=int, default=32)
    parser.add_argument("--epochs", type=int, default=500)
    parser.add_argument("--out-model", default="chess_sim/assets/perception/model.json")
    parser.add_argument("--out-report", default=".private/perception/training.json")
    parser.add_argument("--data", default=".private/perception/dataset.npz")
    args = parser.parse_args()
    if args.sessions < 2 or args.positions < 1:
        parser.error("need at least two sessions and one position per session")

    started = time.time()
    data_path = Path(args.data)
    if data_path.exists():
        archive = np.load(data_path, allow_pickle=True)
        features = archive["features"]
        labels = archive["labels"]
        views = archive["views"]
        frames = archive["frames"]
        sessions = archive["sessions"]
        session_stats = json.loads(str(archive["session_stats"]))
        print(f"loaded dataset {data_path} ({len(features)} rows)", flush=True)
    else:
        rng = random.Random(args.seed)
        all_features: list[np.ndarray] = []
        all_labels: list[int] = []
        all_views: list[int] = []
        all_frames: list[int] = []
        all_sessions: list[int] = []
        session_stats = []
        for session in range(args.sessions):
            scene = make_session(args.seed * 1000 + session, random_camera=False)
            try:
                session_features, session_labels, session_views, session_frames, stats = (
                    collect_session(scene.env, scene.seed, args.positions,
                                    random.Random(rng.getrandbits(32))))
            finally:
                scene.close()
            all_features.extend(session_features)
            all_labels.extend(session_labels)
            all_views.extend(session_views)
            all_frames.extend(session_frames)
            all_sessions.extend([session] * len(session_features))
            session_stats.append({
                "session": session,
                "seed": scene.seed,
                "board_variant": scene.board_variant,
                "camera_height": scene.camera_height,
                "camera_offset": scene.camera_offset,
                "positions": stats,
                "rows": len(session_features),
            })
            print(f"session {session + 1}/{args.sessions}: {len(session_features)} rows "
                  f"({sum(1 for v in session_views if v == 1)} wrist)", flush=True)

        features = np.asarray(all_features, dtype=np.float32)
        labels = np.asarray(all_labels, dtype=np.int64)
        views = np.asarray(all_views, dtype=np.int64)
        frames = np.asarray(all_frames, dtype=np.int64)
        sessions = np.asarray(all_sessions, dtype=np.int64)
        data_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(data_path, features=features, labels=labels,
                            views=views, frames=frames, sessions=sessions,
                            session_stats=json.dumps(session_stats))
        print(f"dataset: {data_path}", flush=True)

    features = normalize_per_frame(features, frames)
    split = max(1, int(round(0.8 * args.sessions)))
    train_mask = sessions < split
    val_mask = ~train_mask
    if val_mask.sum() == 0:
        val_mask = sessions == sessions.max()
        train_mask = ~val_mask
    model, training = train_model(
        features[train_mask], labels[train_mask],
        hidden=args.hidden, epochs=args.epochs, seed=args.seed,
        validation=(features[val_mask], labels[val_mask]))
    probabilities = model.probabilities(features[val_mask])
    threshold, operating = calibrate_threshold(probabilities, labels[val_mask])
    model.confidence_threshold = threshold
    model.metadata.update({
        "trained_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "sessions": args.sessions, "positions": args.positions,
        "train_rows": int(train_mask.sum()), "validation_rows": int(val_mask.sum()),
        "operating_point": operating, "session_seed": args.seed,
        "scope": "fixed overhead rig, bounded board placement",
    })
    output = Path(args.out_model)
    model.save(output)
    report = {
        "schema": "robot-arm-perception-training/v1",
        "seed": args.seed,
        "model": str(output),
        "features": list(FEATURE_NAMES),
        "classes": list(CLASSES),
        "rows": int(len(features)),
        "labels": {name: int((labels == index).sum()) for index, name in enumerate(CLASSES)},
        "views": {"top": int((views == 0).sum()), "wrist": int((views == 1).sum())},
        "split_sessions": split,
        "training": training,
        "operating_point": operating,
        "session_stats": session_stats,
        "elapsed_s": round(time.time() - started, 1),
    }
    report_path = Path(args.out_report)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items()
                      if k not in ("session_stats", "training", "features")}, indent=2))
    print(f"model: {output}")
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
