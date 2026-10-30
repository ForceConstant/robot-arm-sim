"""Square occupancy and piece-colour perception on a registered board.

The pipeline is deliberately split from registration.  Given a frame, a
calibrated camera and a :class:`~chess_sim.board_registration.BoardRegistration`,
it samples each square as a small rectified patch, computes colour/texture
features and classifies the square as empty, white piece, black piece or
``unknown``.  A small MLP is trained on rendered, domain-randomised sessions;
training labels come from the simulator, but inference consumes pixels only.

No function here accepts a FEN, a piece pose or an ``Env``.  A controller can
call :func:`predict_board` with the frame and calibration alone and must honour
the ``unknown`` outcome rather than guessing.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy import ndimage

from .board_registration import BoardRegistration
from .vision_geometry import CameraCalibration

CLASSES: tuple[str, ...] = ("empty", "white", "black", "unknown")
UNKNOWN_INDEX = CLASSES.index("unknown")
MODEL_PATH = Path(__file__).resolve().parent / "assets" / "perception" / "model.json"

FEATURE_NAMES: tuple[str, ...] = (
    "mean_r", "mean_g", "mean_b",
    "std_r", "std_g", "std_b",
    "luma_mean", "luma_std", "gradient_mean", "saturation_mean",
    "expected_luma", "delta_luma", "delta_r", "delta_g", "delta_b",
    "fraction_deviant", "fraction_brighter", "fraction_darker",
    "centre_ring_delta", "vertical_gradient",
    "largest_component_fraction", "edge_density",
    "valid_fraction", "expected_parity", "view_wrist",
    "ref_delta_luma", "ref_abs_delta_luma", "ref_max_abs_delta",
    "ref_fraction_changed", "ref_delta_r", "ref_delta_g", "ref_delta_b",
    "ref_changed_component", "ref_changed_edge_density",
)
FEATURE_COUNT = len(FEATURE_NAMES)


@dataclass(frozen=True)
class SquarePrediction:
    """One square's perception result; ``unknown`` means the caller must abstain."""

    square: int
    label: str
    confidence: float
    probabilities: tuple[float, ...]
    valid_fraction: float
    features: tuple[float, ...] = field(default=(), repr=False)

    @property
    def occupied(self) -> bool:
        return self.label in ("white", "black")


def _bilinear_rgb(image: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Sample an H x W x 3 image at sub-pixel coordinates, clamped at the edges."""
    height, width = image.shape[:2]
    x = np.clip(np.asarray(x, dtype=np.float32), 0.0, width - 1.001)
    y = np.clip(np.asarray(y, dtype=np.float32), 0.0, height - 1.001)
    x0 = np.floor(x).astype(np.int32)
    y0 = np.floor(y).astype(np.int32)
    x1, y1 = x0 + 1, y0 + 1
    fx, fy = (x - x0)[..., None], (y - y0)[..., None]
    return (image[y0, x0] * (1 - fx) * (1 - fy) + image[y0, x1] * fx * (1 - fy)
            + image[y1, x0] * (1 - fx) * fy + image[y1, x1] * fx * fy)


def project_board_points(calibration: CameraCalibration,
                         registration: BoardRegistration,
                         board_uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Project board-local coordinates (file/rank square units) to pixels."""
    points = np.asarray(board_uv, dtype=float)
    if points.shape[-1] != 2:
        raise ValueError("board_uv must end in two coordinates")
    c, s = np.cos(registration.yaw), np.sin(registration.yaw)
    dx = (points[..., 0] - 3.5) * registration.square
    dy = (points[..., 1] - 3.5) * registration.square
    world_x = registration.origin[0] + c * dx - s * dy
    world_y = registration.origin[1] + s * dx + c * dy
    world_z = np.full_like(world_x, registration.board_z)
    rotation = np.asarray(calibration.camera_rotation, dtype=float).reshape(3, 3)
    camera_xyz = (np.stack([world_x, world_y, world_z], axis=-1)
                  - np.asarray(calibration.camera_position)) @ rotation
    depth = -camera_xyz[..., 2]
    valid = depth > 1e-6
    safe = np.where(valid, depth, 1.0)
    u = calibration.cx + calibration.fx * camera_xyz[..., 0] / safe
    v = calibration.cy - calibration.fy * camera_xyz[..., 1] / safe
    valid &= (u >= 0) & (u < calibration.width) & (v >= 0) & (v < calibration.height)
    return np.stack([u, v], axis=-1), valid


def board_patches(frame: np.ndarray, calibration: CameraCalibration,
                  registration: BoardRegistration, *,
                  patch: int = 16, margin: float = 0.34
                  ) -> tuple[np.ndarray, np.ndarray]:
    """Sample one rectified RGB patch per square.

    Returns ``(patches, valid_fraction)`` with patches in 0..1 float RGB and
    shape (64, patch, patch, 3).  Invalid (out-of-frame) samples are filled with
    the patch's valid mean; ``valid_fraction`` tells the caller how much of the
    square was actually observed.
    """
    frame = np.asarray(frame)
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be H x W x 3")
    if frame.shape[:2] != (calibration.height, calibration.width):
        raise ValueError("frame shape does not match the calibration")
    if not 0 < margin < 0.5:
        raise ValueError("margin must be between 0 and 0.5")
    if patch < 4:
        raise ValueError("patch must be at least 4 pixels")
    offsets = np.linspace(-margin, margin, patch, dtype=float)
    files = np.tile(np.arange(8, dtype=float), 8)
    ranks = np.repeat(np.arange(8, dtype=float), 8)
    uu = files[:, None, None] + offsets[None, None, :]
    vv = ranks[:, None, None] + offsets[None, :, None]
    uv = np.stack([np.broadcast_to(uu, (64, patch, patch)),
                   np.broadcast_to(vv, (64, patch, patch))], axis=-1)
    pixels, valid = project_board_points(calibration, registration, uv)
    image = frame.astype(np.float32) / 255.0
    sampled = _bilinear_rgb(image, pixels[..., 0], pixels[..., 1])
    sampled = np.where(valid[..., None], sampled, np.nan)
    valid_fraction = valid.reshape(64, -1).mean(axis=1)
    patches = np.empty_like(sampled, dtype=np.float32)
    for index in range(64):
        good = valid[index]
        if good.any():
            fill = sampled[index][good].mean(axis=0)
        else:
            fill = np.zeros(3, dtype=np.float32)
        patches[index] = np.where(good[..., None], sampled[index], fill)
    return patches, valid_fraction


def square_mask_fraction(mask: np.ndarray, calibration: CameraCalibration,
                         registration: BoardRegistration, *,
                         patch: int = 16, margin: float = 0.34) -> np.ndarray:
    """Fraction of each square's sampled area covered by a boolean image mask.

    Used by evaluation to identify pixels occupied by the robot arm, and by
    training to label naturally occluded squares as ``unknown``.  It is a
    measurement of the image, not of piece identity.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.shape != (calibration.height, calibration.width):
        raise ValueError("mask shape does not match the calibration")
    offsets = np.linspace(-margin, margin, patch, dtype=float)
    files = np.tile(np.arange(8, dtype=float), 8)
    ranks = np.repeat(np.arange(8, dtype=float), 8)
    uu = files[:, None, None] + offsets[None, None, :]
    vv = ranks[:, None, None] + offsets[None, :, None]
    uv = np.stack([np.broadcast_to(uu, (64, patch, patch)),
                   np.broadcast_to(vv, (64, patch, patch))], axis=-1)
    pixels, valid = project_board_points(calibration, registration, uv)
    x = np.clip(pixels[..., 0], 0, calibration.width - 1).astype(int)
    y = np.clip(pixels[..., 1], 0, calibration.height - 1).astype(int)
    covered = mask[y, x] & valid
    return covered.reshape(64, -1).mean(axis=1)


def _square_parity() -> np.ndarray:
    return np.array([(index % 8 + index // 8) % 2 for index in range(64)], dtype=float)


def expected_background(patches: np.ndarray, valid_fraction: np.ndarray) -> np.ndarray:
    """Estimate each square's empty-board colour from same-colour neighbours.

    A checkerboard's light squares and dark squares each form a diagonal
    lattice.  The median of the four same-parity diagonal neighbours is robust
    to a few occupied squares and, unlike one global colour, follows lighting
    gradients across the board.  The global parity median is the fallback when a
    square has no observed neighbour.
    """
    parity = _square_parity()
    means = patches.reshape(64, -1, 3).mean(axis=1)
    observed = valid_fraction > 0.35
    global_median = np.zeros((2, 3), dtype=np.float32)
    for value in (0.0, 1.0):
        selected = (parity == value) & observed
        if selected.any():
            global_median[int(value)] = np.median(means[selected], axis=0)
    expected = np.empty_like(means)
    for square in range(64):
        file, rank = square % 8, square // 8
        neighbours = []
        for df, dr in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            nf, nr = file + df, rank + dr
            if 0 <= nf < 8 and 0 <= nr < 8:
                neighbour = nr * 8 + nf
                if observed[neighbour]:
                    neighbours.append(means[neighbour])
        expected[square] = (np.median(neighbours, axis=0) if neighbours
                            else global_median[int(parity[square])])
    return expected


# Per-frame standardisation removes session-specific lighting, board colour and
# exposure.  The structural fields are left alone.
NORMALISED_FEATURES: tuple[int, ...] = tuple(
    index for index, name in enumerate(FEATURE_NAMES)
    if name not in ("valid_fraction", "expected_parity", "view_wrist")
    and not name.startswith("ref_"))


def normalize_per_frame(features: np.ndarray, frames: np.ndarray) -> np.ndarray:
    """Robustly standardise appearance features within each frame.

    A board is mostly empty, so the median/MAD across its 64 squares is a
    session-local reference.  This is what lets one small model survive a
    different board colour, lamp or camera exposure in a held-out session.
    Structural features (validity, parity, view) are preserved.
    """
    output = np.asarray(features, dtype=np.float32).copy()
    frames = np.asarray(frames)
    if output.ndim != 2 or output.shape[1] != FEATURE_COUNT:
        raise ValueError("features must be (N, FEATURE_COUNT)")
    if frames.shape != (len(output),):
        raise ValueError("frames must have one id per feature row")
    columns = np.asarray(NORMALISED_FEATURES, dtype=int)
    for frame in np.unique(frames):
        rows = np.flatnonzero(frames == frame)
        if len(rows) < 8:
            continue
        block = output[rows][:, columns]
        median = np.median(block, axis=0)
        mad = np.median(np.abs(block - median), axis=0) * 1.4826
        scale = np.maximum(mad, 1e-3)
        output[np.ix_(rows, columns)] = np.clip((block - median) / scale, -8.0, 8.0)
    return output


def square_features(patches: np.ndarray, valid_fraction: np.ndarray,
                    *, view: str = "top",
                    expected: np.ndarray | None = None,
                    reference: np.ndarray | None = None) -> np.ndarray:
    """Compute the fixed feature vector for all 64 squares.

    ``reference`` is the same session's empty-board patch set, registered the
    same way.  Difference features against it are what make a piece visible
    when its colour happens to match the board; the absolute features remain so
    the model still works without a reference.
    """
    if patches.shape != (64, patches.shape[1], patches.shape[1], 3):
        raise ValueError("patches must have shape (64, P, P, 3)")
    if reference is not None and reference.shape != patches.shape:
        raise ValueError("reference must have the same patch shape")
    if view not in ("top", "wrist"):
        raise ValueError("view must be 'top' or 'wrist'")
    if expected is None:
        expected = expected_background(patches, valid_fraction)
    parity = _square_parity()
    luma = patches @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
    size = patches.shape[1]
    features = np.zeros((64, FEATURE_COUNT), dtype=np.float32)
    for square in range(64):
        patch = patches[square]
        lum = luma[square]
        flat = patch.reshape(-1, 3)
        mean_rgb = flat.mean(axis=0)
        std_rgb = flat.std(axis=0)
        luma_mean, luma_std = float(lum.mean()), float(lum.std())
        grad_x = np.abs(np.diff(lum, axis=1)).mean()
        grad_y = np.abs(np.diff(lum, axis=0)).mean()
        gradient = float(0.5 * (grad_x + grad_y))
        saturation = float((patch.max(axis=2) - patch.min(axis=2)).mean())
        expected_rgb = expected[square]
        expected_luma = float(expected_rgb @ np.array([0.299, 0.587, 0.114]))
        delta = float(luma_mean - expected_luma)
        deviation = np.abs(lum - expected_luma) > 0.07
        fraction_deviant = float(deviation.mean())
        fraction_brighter = float((lum > expected_luma + 0.07).mean())
        fraction_darker = float((lum < expected_luma - 0.07).mean())
        centre = lum[size // 4: 3 * size // 4, size // 4: 3 * size // 4].mean()
        ring = np.concatenate([lum[:2, :].ravel(), lum[-2:, :].ravel(),
                               lum[:, :2].ravel(), lum[:, -2:].ravel()]).mean()
        centre_ring = float(centre - ring)
        vertical = float(np.diff(lum, axis=0).mean())
        strong = np.abs(lum - expected_luma) > 0.09
        labels, count = ndimage.label(strong)
        largest = 0.0
        if count:
            areas = np.bincount(labels.ravel())[1:]
            largest = float(areas.max() / float(size * size))
        edge_density = float((np.hypot(np.diff(lum, axis=1, append=lum[:, -1:]),
                                       np.diff(lum, axis=0, append=lum[-1:, :])) > 0.08).mean())
        if reference is not None:
            ref = reference[square]
            ref_lum = ref @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
            diff_luma = lum - ref_lum
            diff_rgb = patch - ref
            changed = np.abs(diff_luma) > 0.08
            changed_labels, changed_count = ndimage.label(changed)
            changed_largest = 0.0
            if changed_count:
                changed_areas = np.bincount(changed_labels.ravel())[1:]
                changed_largest = float(changed_areas.max() / float(size * size))
            changed_edges = float((np.hypot(
                np.diff(diff_luma, axis=1, append=diff_luma[:, -1:]),
                np.diff(diff_luma, axis=0, append=diff_luma[-1:, :])) > 0.08).mean())
            ref_features = (
                float(diff_luma.mean()), float(np.abs(diff_luma).mean()),
                float(np.abs(diff_luma).max()), float(changed.mean()),
                float(diff_rgb[..., 0].mean()), float(diff_rgb[..., 1].mean()),
                float(diff_rgb[..., 2].mean()), changed_largest, changed_edges,
            )
        else:
            ref_features = (0.0,) * 9
        features[square] = (
            mean_rgb[0], mean_rgb[1], mean_rgb[2],
            std_rgb[0], std_rgb[1], std_rgb[2],
            luma_mean, luma_std, gradient, saturation,
            expected_luma, delta,
            float(mean_rgb[0] - expected_rgb[0]),
            float(mean_rgb[1] - expected_rgb[1]),
            float(mean_rgb[2] - expected_rgb[2]),
            fraction_deviant, fraction_brighter, fraction_darker,
            centre_ring, vertical,
            largest, edge_density,
            float(valid_fraction[square]), float(parity[square]),
            0.0 if view == "top" else 1.0,
            *ref_features,
        )
    return features


class PerceptionModel:
    """A tiny standardised MLP over :data:`FEATURE_NAMES`.

    The model is intentionally small and dependency-free (NumPy only).  It is
    not a foundation model; it is a measured baseline for a controlled board,
    with an explicit ``unknown`` class trained on occlusions and low-confidence
    inputs.
    """

    def __init__(self, mean, std, W1, b1, W2, b2, *, classes=CLASSES,
                 confidence_threshold: float = 0.75,
                 metadata: dict | None = None,
                 feature_names: tuple[str, ...] = FEATURE_NAMES):
        self.mean = np.asarray(mean, dtype=np.float32)
        self.std = np.asarray(std, dtype=np.float32)
        self.W1 = np.asarray(W1, dtype=np.float32)
        self.b1 = np.asarray(b1, dtype=np.float32)
        self.W2 = np.asarray(W2, dtype=np.float32)
        self.b2 = np.asarray(b2, dtype=np.float32)
        self.classes = tuple(classes)
        self.confidence_threshold = float(confidence_threshold)
        self.metadata = dict(metadata or {})
        self.feature_names = tuple(feature_names)
        if self.mean.shape != (len(self.feature_names),):
            raise ValueError("mean does not match the feature names")
        if self.W1.shape[0] != len(self.feature_names):
            raise ValueError("W1 does not match the feature names")

    @property
    def feature_count(self) -> int:
        return len(self.feature_names)

    def probabilities(self, features: np.ndarray) -> np.ndarray:
        values = np.atleast_2d(np.asarray(features, dtype=np.float32))
        if values.shape[1] != self.feature_count:
            raise ValueError(f"expected {self.feature_count} features, got {values.shape[1]}")
        standardised = (values - self.mean) / np.where(self.std < 1e-6, 1.0, self.std)
        hidden = np.tanh(standardised @ self.W1 + self.b1)
        logits = hidden @ self.W2 + self.b2
        logits -= logits.max(axis=1, keepdims=True)
        exp = np.exp(logits)
        return exp / exp.sum(axis=1, keepdims=True)

    def predict(self, features: np.ndarray, *,
                min_confidence: float | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (class indices, confidence, full probability matrix)."""
        probabilities = self.probabilities(features)
        threshold = self.confidence_threshold if min_confidence is None else float(min_confidence)
        indices = probabilities.argmax(axis=1)
        confidence = probabilities.max(axis=1)
        indices = np.where(confidence < threshold, UNKNOWN_INDEX, indices)
        # A predicted ``unknown`` class stays unknown regardless of softmax.
        indices = np.where(indices == UNKNOWN_INDEX, UNKNOWN_INDEX, indices)
        return indices, confidence, probabilities

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": "robot-arm-perception-model/v1",
            "classes": list(self.classes),
            "feature_names": list(self.feature_names),
            "mean": self.mean.tolist(),
            "std": self.std.tolist(),
            "W1": self.W1.tolist(), "b1": self.b1.tolist(),
            "W2": self.W2.tolist(), "b2": self.b2.tolist(),
            "confidence_threshold": self.confidence_threshold,
            "metadata": self.metadata,
        }
        path.write_text(json.dumps(payload, indent=1) + "\n")
        return path

    @classmethod
    def load(cls, path: str | Path = MODEL_PATH) -> "PerceptionModel":
        payload = json.loads(Path(path).read_text())
        if payload.get("schema") != "robot-arm-perception-model/v1":
            raise ValueError("unsupported perception model schema")
        return cls(payload["mean"], payload["std"], payload["W1"], payload["b1"],
                   payload["W2"], payload["b2"], classes=tuple(payload["classes"]),
                   confidence_threshold=payload["confidence_threshold"],
                   metadata=payload.get("metadata", {}),
                   feature_names=tuple(payload["feature_names"]))


def train_model(features: np.ndarray, labels: np.ndarray, *,
                feature_names: tuple[str, ...] = FEATURE_NAMES,
                hidden: int = 24, epochs: int = 500, learning_rate: float = 0.01,
                l2: float = 1e-3, batch: int = 128, seed: int = 0,
                validation: tuple[np.ndarray, np.ndarray] | None = None,
                class_balanced: bool = True) -> tuple[PerceptionModel, dict]:
    """Fit the small MLP with Adam; deterministic for a given seed and data."""
    features = np.asarray(features, dtype=np.float32)
    labels = np.asarray(labels, dtype=np.int64)
    if features.ndim != 2 or features.shape[1] != len(feature_names):
        raise ValueError("features do not match the feature names")
    if labels.shape != (len(features),):
        raise ValueError("labels must be one per feature row")
    classes = len(CLASSES)
    mean = features.mean(axis=0)
    std = features.std(axis=0)
    std = np.where(std < 1e-6, 1.0, std)
    x = (features - mean) / std
    rng = np.random.default_rng(seed)
    W1 = rng.normal(0.0, 1.0 / np.sqrt(len(feature_names)), (len(feature_names), hidden)).astype(np.float32)
    b1 = np.zeros(hidden, dtype=np.float32)
    W2 = rng.normal(0.0, 1.0 / np.sqrt(hidden), (hidden, classes)).astype(np.float32)
    b2 = np.zeros(classes, dtype=np.float32)
    params = [W1, b1, W2, b2]
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    counts = np.bincount(labels, minlength=classes).astype(np.float32)
    weights = np.ones(classes, dtype=np.float32)
    if class_balanced:
        present = counts > 0
        weights[present] = (counts.sum() / (present.sum() * counts[present]))
        weights /= weights[present].mean()
    onehot = np.eye(classes, dtype=np.float32)[labels]
    best_state = [p.copy() for p in params]
    best_loss = float("inf")
    history = []
    step = 0
    for epoch in range(epochs):
        order = rng.permutation(len(x))
        total = 0.0
        for start in range(0, len(x), batch):
            index = order[start:start + batch]
            xb, yb, wb = x[index], onehot[index], weights[labels[index]]
            hidden_pre = xb @ W1 + b1
            h = np.tanh(hidden_pre)
            logits = h @ W2 + b2
            logits -= logits.max(axis=1, keepdims=True)
            exp = np.exp(logits)
            prob = exp / exp.sum(axis=1, keepdims=True)
            loss = -np.mean(wb * np.sum(yb * np.log(prob + 1e-9), axis=1))
            total += loss * len(index)
            dlogits = (prob - yb) * wb[:, None] / len(index)
            dW2 = h.T @ dlogits + l2 * W2
            db2 = dlogits.sum(axis=0)
            dh = (dlogits @ W2.T) * (1 - h * h)
            dW1 = xb.T @ dh + l2 * W1
            db1 = dh.sum(axis=0)
            gradients = [dW1, db1, dW2, db2]
            step += 1
            for i, (param, grad) in enumerate(zip(params, gradients)):
                m[i] = 0.9 * m[i] + 0.1 * grad
                v[i] = 0.999 * v[i] + 0.001 * grad * grad
                mhat = m[i] / (1 - 0.9 ** step)
                vhat = v[i] / (1 - 0.999 ** step)
                param -= learning_rate * mhat / (np.sqrt(vhat) + 1e-8)
        train_loss = total / len(x)
        record = {"epoch": epoch, "train_loss": float(train_loss)}
        if validation is not None:
            vx, vy = validation
            vx = (np.asarray(vx, dtype=np.float32) - mean) / std
            vh = np.tanh(vx @ W1 + b1)
            vl = vh @ W2 + b2
            vl -= vl.max(axis=1, keepdims=True)
            ve = np.exp(vl)
            vp = ve / ve.sum(axis=1, keepdims=True)
            val_loss = float(-np.mean(np.log(vp[np.arange(len(vy)), vy] + 1e-9)))
            accuracy = float((vp.argmax(axis=1) == vy).mean())
            record.update({"val_loss": val_loss, "val_accuracy": accuracy})
            if val_loss < best_loss:
                best_loss = val_loss
                best_state = [p.copy() for p in params]
        history.append(record)
    if validation is not None and np.isfinite(best_loss):
        W1, b1, W2, b2 = best_state
    model = PerceptionModel(mean, std, W1, b1, W2, b2,
                            metadata={"seed": seed, "epochs": epochs,
                                      "hidden": hidden, "l2": l2,
                                      "train_rows": int(len(x)),
                                      "class_counts": counts.astype(int).tolist()})
    return model, {"history": history, "best_val_loss": None if not np.isfinite(best_loss) else best_loss}


def calibrate_threshold(probabilities: np.ndarray, labels: np.ndarray, *,
                        max_confident_error: float = 0.02) -> tuple[float, dict]:
    """Pick the lowest confidence threshold meeting a confident-error budget.

    ``unknown`` truth labels are excluded from the error count: abstaining on
    them is correct.  Returns the threshold and the operating point.
    """
    probabilities = np.asarray(probabilities, dtype=float)
    labels = np.asarray(labels, dtype=int)
    best = None
    total = len(labels)
    for threshold in np.arange(0.40, 0.991, 0.01):
        predicted = probabilities.argmax(axis=1)
        confidence = probabilities.max(axis=1)
        answered = (confidence >= threshold) & (predicted != UNKNOWN_INDEX) & (labels != UNKNOWN_INDEX)
        if answered.sum() == 0:
            continue
        correct = (predicted == labels) & answered
        errors = int((answered & ~correct).sum())
        error_rate = errors / total
        if error_rate <= max_confident_error:
            best = {
                "threshold": float(threshold),
                "answered": int(answered.sum()),
                "coverage": float(answered.mean()),
                "confident_errors": errors,
                "confident_error_rate": float(error_rate),
                "answered_accuracy": float(correct.sum() / answered.sum()),
            }
            break
    if best is None:
        threshold = 0.99
        predicted = probabilities.argmax(axis=1)
        confidence = probabilities.max(axis=1)
        answered = (confidence >= threshold) & (predicted != UNKNOWN_INDEX) & (labels != UNKNOWN_INDEX)
        correct = (predicted == labels) & answered
        best = {"threshold": threshold,
                "answered": int(answered.sum()),
                "coverage": float(answered.mean()),
                "confident_errors": int((answered & ~correct).sum()),
                "confident_error_rate": float((answered & ~correct).sum() / total),
                "answered_accuracy": float(correct.sum() / max(1, answered.sum()))}
    return float(best["threshold"]), best


def predict_board(frame: np.ndarray, calibration: CameraCalibration,
                  registration: BoardRegistration, model: PerceptionModel,
                  *, view: str = "top", patch: int = 16,
                  min_confidence: float | None = None,
                  reference_frame: np.ndarray | None = None) -> list[SquarePrediction]:
    """Classify all 64 squares from pixels, calibration and board geometry only.

    ``reference_frame`` is an optional empty-board frame from the same session
    and pose.  It is a calibration observation, not piece truth.
    """
    image = np.asarray(frame)
    luminance = image.astype(np.float32) @ np.array([0.299, 0.587, 0.114],
                                                    dtype=np.float32)
    # A black/blank frame is not a board observation; abstain explicitly rather
    # than letting darkness look like black pieces.
    invalid_frame = float(luminance.mean()) < 12.0 or float(luminance.std()) < 2.0
    patches, valid_fraction = board_patches(frame, calibration, registration,
                                            patch=patch)
    reference = None
    if reference_frame is not None:
        reference, _ = board_patches(reference_frame, calibration, registration,
                                     patch=patch)
    expected = expected_background(patches, valid_fraction)
    features = square_features(patches, valid_fraction, view=view,
                               expected=expected, reference=reference)
    features = normalize_per_frame(features, np.zeros(64, dtype=int))
    indices, confidence, probabilities = model.predict(features,
                                                       min_confidence=min_confidence)
    results = []
    for square in range(64):
        label = model.classes[int(indices[square])]
        if invalid_frame or valid_fraction[square] < 0.45:
            label = "unknown"
        results.append(SquarePrediction(
            square=square,
            label=label,
            confidence=float(confidence[square]),
            probabilities=tuple(float(value) for value in probabilities[square]),
            valid_fraction=float(valid_fraction[square]),
            features=tuple(float(value) for value in features[square]),
        ))
    return results


def load_default_model(path: str | Path = MODEL_PATH) -> PerceptionModel:
    """Load the shipped model, with a clear error if it has not been trained."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"no perception model at {path}; run tools/train_perception.py")
    return PerceptionModel.load(path)
