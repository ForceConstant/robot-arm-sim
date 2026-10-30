"""Scene and data helpers shared by perception training and evaluation.

These helpers render simulator scenes and produce labels.  They are deliberately
outside the inference path: :mod:`chess_sim.perception` never imports this
module, so a controller cannot accidentally receive a FEN, a piece pose or a
simulator handle.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import chess
import mujoco
import numpy as np

from .board_registration import BoardRegistration, register_board
from .config import (AppearanceConfig, BoardConfig, Camera, ControlConfig,
                     EnvConfig, PUBLISHED_RIG_CAMERA, RandomizationConfig)
from .env import ChessSimEnv
from .vision_geometry import CameraCalibration

BOARD_VARIANTS = (
    dict(square=0.026, thickness=0.008, border=0.008),
    dict(square=0.028, thickness=0.012, border=0.015),
    dict(square=0.030, thickness=0.016, border=0.022),
)
LABELS = ("empty", "white", "black", "unknown")
MIN_REGISTRATION_SCORE = 0.40
REGISTRATION_ORIGIN_SEARCH = 0.016
REGISTRATION_YAW_SEARCH_DEG = 4.5
REGISTRATION_COARSE_STEP = 0.003


@dataclass(frozen=True)
class Session:
    env: ChessSimEnv
    seed: int
    board_variant: int
    camera_height: float | None
    camera_offset: tuple[float, float] | None

    def close(self) -> None:
        self.env.close()


def robot_geometry_ids(env: ChessSimEnv, prefix: str = "so101:") -> list[int]:
    ids = []
    for geom_id in range(env.model.ngeom):
        body_id = int(env.model.geom_bodyid[geom_id])
        name = mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name and name.startswith(prefix):
            ids.append(geom_id)
    return ids


def render_mask(env: ChessSimEnv, camera: Camera, geom_ids: list[int]) -> np.ndarray:
    return np.isin(env.render_segmentation(camera)[:, :, 0], geom_ids)


def make_session(seed: int, *, random_camera: bool = True,
                 board_variant: int | None = None) -> Session:
    """Build one randomized scene; the seed determines its look and layout."""
    rng = np.random.default_rng(seed)
    index = (int(rng.integers(len(BOARD_VARIANTS)))
             if board_variant is None else int(board_variant))
    variant = BOARD_VARIANTS[index]
    appearance = AppearanceConfig.sample(
        rng, piece_scale_range=(0.88, 1.05), piece_sets=("default",))
    if not random_camera or rng.random() < 0.35:
        appearance = appearance.model_copy(update=PUBLISHED_RIG_CAMERA)
    config = EnvConfig(
        board=BoardConfig(**variant),
        appearance=appearance,
        control=ControlConfig(cameras=(Camera.TOP, Camera.WRIST)),
        randomization=RandomizationConfig(board_shift=0.006,
                                          board_yaw=math.radians(2.5),
                                          board_side=0.006,
                                          arm_joint_jitter=0.0,
                                          camera=False),
    )
    env = ChessSimEnv(config)
    return Session(env=env, seed=int(rng.integers(1 << 31)), board_variant=index,
                   camera_height=appearance.camera_height,
                   camera_offset=appearance.camera_offset)


def truth_labels(env: ChessSimEnv) -> np.ndarray:
    """Per-square truth as class indices: 0 empty, 1 white, 2 black."""
    labels = np.zeros(64, dtype=np.int64)
    for square in range(64):
        slot = env.slot_at(square)
        if slot is None:
            labels[square] = 0
        else:
            labels[square] = 1 if slot.piece.color == chess.WHITE else 2
    return labels


def augment_patches(patches: np.ndarray, labels: np.ndarray,
                    rng: np.random.Generator, *,
                    occluders: int = 6, shadows: int = 6) -> tuple[np.ndarray, np.ndarray]:
    """Add hand/arm-like opaque occluders (unknown) and soft shadows.

    Opaque occluders covering more than about a third of a square are labelled
    ``unknown`` so the classifier learns to abstain.  Shadows keep the label:
    lighting changes appearance, not occupancy.
    """
    patches = patches.copy()
    labels = labels.copy()
    size = patches.shape[1]
    yy, xx = np.mgrid[0:size, 0:size]
    unknown = 3
    for square in rng.choice(64, size=occluders, replace=False):
        cx, cy = rng.uniform(0.15, 0.85, 2) * size
        rx, ry = rng.uniform(0.30, 0.55, 2) * size
        ellipse = ((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2 <= 1.0
        colour = rng.choice([
            (0.75, 0.55, 0.42), (0.30, 0.20, 0.15), (0.05, 0.35, 0.36),
            (0.55, 0.55, 0.55), (0.90, 0.75, 0.65)])
        noise = rng.normal(0.0, 0.02, (size, size, 1))
        patches[square] = np.where(ellipse[..., None], np.clip(colour + noise, 0, 1),
                                   patches[square])
        labels[square] = unknown
    for square in rng.choice(64, size=shadows, replace=False):
        if labels[square] == unknown:
            continue
        direction = int(rng.integers(4))
        ramp = np.linspace(1.0, rng.uniform(0.45, 0.8), size, dtype=np.float32)
        shade = np.ones((size, size), dtype=np.float32)
        if direction == 0:
            shade *= ramp[None, :]
        elif direction == 1:
            shade *= ramp[::-1][None, :]
        elif direction == 2:
            shade *= ramp[:, None]
        else:
            shade *= ramp[::-1][:, None]
        patches[square] *= shade[..., None]
    return patches, labels


def register_for_perception(frame: np.ndarray, calibration: CameraCalibration,
                            board_z: float, square: float,
                            prior: BoardRegistration | None = None) -> BoardRegistration | None:
    """Register within the calibrated board-placement envelope, or abstain.

    The fixed rig and the known square size/height are calibration inputs; the
    image supplies the residual board pose.  A low-score fit is rejected rather
    than handed to perception, because a one-square slip would label the wrong
    pixels.  Callers may keep the previous accepted pose as their prior.
    """
    prior = prior or BoardRegistration((0.0, 0.0), 0.0, float(square), float(board_z),
                                       score=0.0, coverage=1.0)
    result = register_board(
        frame, calibration, board_z, prior=prior,
        origin_search=REGISTRATION_ORIGIN_SEARCH,
        yaw_search_deg=REGISTRATION_YAW_SEARCH_DEG,
        coarse_step=REGISTRATION_COARSE_STEP,
        square_range=(prior.square, prior.square),
    )
    if result is None or result.score < MIN_REGISTRATION_SCORE or result.coverage < 0.80:
        return None
    return result
