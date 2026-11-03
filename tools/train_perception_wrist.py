"""Train the wrist-scan occupancy/colour model on the product scene.

Sessions randomise board placement and lighting; frames are rendered at the
planned scan poses with the arm in place, so natural self-occlusion is part of
the data and such squares are labelled ``unknown`` rather than guessed.  The
empty-board reference for each pose is captured before pieces are placed, which
is how the product's calibration epoch works.

    MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
    ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
    PYTHONPATH=. .venv/bin/python tools/train_perception_wrist.py --sessions 12 --positions 6
"""
from __future__ import annotations

import argparse
import json
import math
import random
import time
from pathlib import Path

import mujoco
import numpy as np

from chess_sim.config import Camera, RandomizationConfig
from chess_sim.env import ChessSimEnv
from chess_sim.perception import (CLASSES, FEATURE_NAMES, board_patches,
                                  calibrate_threshold, expected_background,
                                  normalize_per_frame, square_features,
                                  square_mask_fraction, train_model)
from chess_sim.perception_data import (render_mask, robot_geometry_ids,
                                       truth_labels)
from chess_sim.positions import PositionConfig
from chess_sim.product import layout_env_config, load_product_twin
from chess_sim.wrist_scan import load_plan
from chess_sim.board_registration import BoardRegistration
from chess_sim.vision_geometry import CameraCalibration

CLASS_INDEX = {name: index for index, name in enumerate(CLASSES)}
UNKNOWN = CLASS_INDEX["unknown"]
EMPTY = "8/8/8/8/8/8/8/8 w - - 0 1"


def scene(seed: int) -> ChessSimEnv:
    twin = load_product_twin()
    draw = np.random.default_rng(seed)
    appearance = twin.appearance("printed").model_copy(update={
        "light_intensity": float(draw.uniform(0.82, 1.15)),
        "ambient": float(draw.uniform(0.0, 0.05)),
        "light_dir": (float(draw.uniform(-0.35, -0.2)),
                      float(draw.uniform(0.2, 0.4)), -1.0),
        "fill_intensity": float(draw.uniform(0.7, 1.3)),
        "shadow_softness": float(draw.uniform(0.015, 0.035)),
    })
    config = layout_env_config("printed", cameras=(Camera.WRIST,)).model_copy(update={
        "appearance": appearance,
        "randomization": RandomizationConfig(board_shift=0.008, board_yaw=math.radians(2.5),
                                             board_side=0.008, arm_joint_jitter=0.0,
                                             camera=False),
    })
    return ChessSimEnv(config)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sessions", type=int, default=12)
    parser.add_argument("--positions", type=int, default=6)
    parser.add_argument("--seed", type=int, default=4100)
    parser.add_argument("--hidden", type=int, default=40)
    parser.add_argument("--epochs", type=int, default=500)
    parser.add_argument("--plan", default=str(Path("chess_sim/assets/wrist_scan_plan.json")))
    parser.add_argument("--out-model", default="chess_sim/assets/perception/model_wrist.json")
    parser.add_argument("--out-report", default=".private/phase9/wrist_training.json")
    args = parser.parse_args()

    poses = load_plan(args.plan)
    started = time.time()
    features_out: list[np.ndarray] = []
    labels_out: list[int] = []
    frames_out: list[int] = []
    sessions_out: list[int] = []
    stats: list[dict] = []
    rng = random.Random(args.seed)
    frame_id = 0

    for session in range(args.sessions):
        env = scene(args.seed * 1000 + session)
        try:
            env.reset(EMPTY, rng=random.Random(args.seed * 1000 + session))
            env.config = env.config.model_copy(update={"board": env.board})
            arm_ids = robot_geometry_ids(env)
            references = {}
            for pose in poses:
                env.data.qpos[env._joint_qpos] = np.asarray(pose.joints)
                mujoco.mj_forward(env.model, env.data)
                camera = CameraCalibration.from_mujoco(
                    env.model, env.data, str(Camera.WRIST), 640, 480)
                frame = env.render(Camera.WRIST).copy()
                registration = BoardRegistration(
                    env.board.origin, env.board.yaw, env.board.square, env.board.top, 1.0, 1.0)
                patches, valid = board_patches(frame, camera, registration)
                references[pose.name] = (patches, valid)
            for position_index in range(args.positions):
                board = PositionConfig(min_extra_pieces=2, max_extra_pieces=8).sample(rng)
                env.reset(board)
                truth = truth_labels(env)
                for pose in poses:
                    env.data.qpos[env._joint_qpos] = np.asarray(pose.joints)
                    mujoco.mj_forward(env.model, env.data)
                    camera = CameraCalibration.from_mujoco(
                        env.model, env.data, str(Camera.WRIST), 640, 480)
                    frame = env.render(Camera.WRIST).copy()
                    registration = BoardRegistration(
                        env.board.origin, env.board.yaw, env.board.square, env.board.top, 1.0, 1.0)
                    patches, valid = board_patches(frame, camera, registration)
                    ref_patches, _ = references[pose.name]
                    occlusion = square_mask_fraction(
                        render_mask(env, Camera.WRIST, arm_ids), camera, registration)
                    expected = expected_background(patches, valid)
                    values = normalize_per_frame(
                        square_features(patches, valid, view="wrist",
                                        expected=expected, reference=ref_patches),
                        np.zeros(64, dtype=int))
                    labels = truth.copy()
                    labels[occlusion > 0.10] = UNKNOWN
                    labels[valid < 0.9] = UNKNOWN
                    for square in range(64):
                        features_out.append(values[square])
                        labels_out.append(int(labels[square]))
                        frames_out.append(frame_id)
                        sessions_out.append(session)
                    frame_id += 1
            stats.append({"session": session, "board": env.board.origin,
                          "yaw": env.board.yaw})
            print(f"session {session + 1}/{args.sessions}", flush=True)
        finally:
            env.close()

    features = np.asarray(features_out, dtype=np.float32)
    labels = np.asarray(labels_out, dtype=np.int64)
    frames = np.asarray(frames_out, dtype=np.int64)
    sessions = np.asarray(sessions_out, dtype=np.int64)
    features = normalize_per_frame(features, frames)
    split = max(1, int(round(0.8 * args.sessions)))
    train_mask = sessions < split
    val_mask = ~train_mask
    model, training = train_model(features[train_mask], labels[train_mask],
                                  hidden=args.hidden, epochs=args.epochs,
                                  seed=args.seed,
                                  validation=(features[val_mask], labels[val_mask]))
    probabilities = model.probabilities(features[val_mask])
    threshold, operating = calibrate_threshold(probabilities, labels[val_mask])
    model.confidence_threshold = threshold
    model.metadata.update({
        "trained_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "sessions": args.sessions, "positions": args.positions,
        "views": len(poses), "scene": "product twin + marker mat, wrist scan poses",
        "operating_point": operating,
    })
    output = Path(args.out_model)
    model.save(output)
    report = {
        "schema": "robot-arm-wrist-perception-training/v1",
        "seed": args.seed, "model": str(output),
        "rows": int(len(features)),
        "labels": {name: int((labels == index).sum()) for index, name in enumerate(CLASSES)},
        "training": training, "operating_point": operating,
        "sessions_report": stats, "elapsed_s": round(time.time() - started, 1),
    }
    path = Path(args.out_report)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items()
                      if k not in ("sessions_report", "training")}, indent=2))
    print(f"model: {output}\nreport: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
