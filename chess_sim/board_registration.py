"""Calibrated board registration from a single overhead frame.

Registration answers one question: where is the 8x8 playing field, given the
camera's calibrated intrinsics/extrinsics, a measured board-surface height and
an ordinary RGB frame?  It is deliberately a geometry problem, not a learned
one.  The detector finds the checkerboard's interior saddle points (a chess
board has 7x7 of them) and fits a planar pose - origin, yaw and square size -
whose projected lattice lands on those corners.

Nothing here reads simulator piece poses, a FEN or the true board transform.
The camera calibration and board-surface height are calibration inputs; the
returned pose is estimated from pixels.  Simulator truth is used by evaluation
tools only, to score the estimate.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize

from .vision_geometry import CameraCalibration

# A standard 8x8 field has 7x7 interior corners.
INTERIOR = 7
MIN_COVERAGE = 0.40
DEFAULT_MIN_SCORE = 0.18


@dataclass(frozen=True)
class BoardRegistration:
    """An estimated board pose in the camera/world frame.

    ``origin`` is the centre of the 8x8 field in world x/y, ``yaw`` its rotation
    about +z (radians), ``square`` the estimated square edge in metres and
    ``board_z`` the supplied playing-surface height.  ``score`` is the mean
    checker-corner response at the fitted lattice (0..1); ``coverage`` is the
    fraction of the 7x7 interior corners that fall inside the frame.
    """

    origin: tuple[float, float]
    yaw: float
    square: float
    board_z: float
    score: float
    coverage: float
    edge_score: float = 0.0
    corner_pixels: tuple[tuple[float, float], ...] = field(default_factory=tuple)

    def board_xy(self, file: float, rank: float) -> tuple[float, float]:
        """Map board-local coordinates (files/ranks, 0..7 square units) to world."""
        dx = (file - 3.5) * self.square
        dy = (rank - 3.5) * self.square
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return (self.origin[0] + c * dx - s * dy,
                self.origin[1] + s * dx + c * dy)

    def square_center(self, square: int | str) -> tuple[float, float]:
        """World x/y of a square centre; accepts an index or a square name."""
        if isinstance(square, str):
            file = ord(square[0].lower()) - ord("a")
            rank = int(square[1]) - 1
        else:
            file, rank = square % 8, square // 8
        return self.board_xy(float(file), float(rank))


def _bilinear(image: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Sample a 2-D float image at sub-pixel coordinates, clamped at the edges."""
    height, width = image.shape
    x = np.clip(np.asarray(x, dtype=np.float32), 0.0, width - 1.001)
    y = np.clip(np.asarray(y, dtype=np.float32), 0.0, height - 1.001)
    x0 = np.floor(x).astype(np.int32)
    y0 = np.floor(y).astype(np.int32)
    x1, y1 = x0 + 1, y0 + 1
    fx, fy = x - x0, y - y0
    return (image[y0, x0] * (1 - fx) * (1 - fy) + image[y0, x1] * fx * (1 - fy)
            + image[y1, x0] * (1 - fx) * fy + image[y1, x1] * fx * fy)


def checker_corner_response(luminance: np.ndarray, radius: float = 4.0) -> np.ndarray:
    """Score each pixel as a checkerboard interior saddle point.

    Four points on the diagonals of a square of side ``2*radius`` are sampled.
    At an interior corner the four quadrants alternate dark/light, so every
    adjacent pair differs; along a straight edge two adjacent pairs are equal
    and on a flat square all four are equal.  Dividing by the strongest local
    contrast keeps dim boards and strong lighting from changing the scale.
    """
    luminance = np.asarray(luminance, dtype=np.float32)
    if luminance.ndim != 2:
        raise ValueError("luminance must be a 2-D array")
    if radius <= 0 or not np.isfinite(radius):
        raise ValueError("radius must be positive and finite")
    height, width = luminance.shape
    ys, xs = np.mgrid[0:height, 0:width].astype(np.float32)
    ne = _bilinear(luminance, xs + radius, ys - radius)
    nw = _bilinear(luminance, xs - radius, ys - radius)
    sw = _bilinear(luminance, xs - radius, ys + radius)
    se = _bilinear(luminance, xs + radius, ys + radius)
    response = np.minimum.reduce([np.abs(ne - nw), np.abs(nw - sw),
                                  np.abs(sw - se), np.abs(se - ne)])
    contrast = np.maximum.reduce([np.abs(ne - nw), np.abs(nw - sw),
                                  np.abs(sw - se), np.abs(se - ne),
                                  np.full_like(ne, 1e-3)])
    return response / contrast


def _offset_lattice() -> np.ndarray:
    """Interior corner offsets in square units around the field centre.

    Seven interior grid lines sit at -3 .. +3 square units; the field spans
    -4 .. +4.  The corner lattice is periodic in one square, so a boundary
    consistency term is what fixes its phase.
    """
    values = np.arange(INTERIOR, dtype=np.float32) - 3.0
    yy, xx = np.meshgrid(values, values, indexing="ij")
    return np.stack([xx.ravel(), yy.ravel()], axis=1)


_LATTICE = _offset_lattice()


def _project_poses(calibration: CameraCalibration, poses: np.ndarray,
                   board_z: float) -> tuple[np.ndarray, np.ndarray]:
    """Project the 7x7 corner lattice for many candidate poses at once.

    ``poses`` is (N, 4): origin x, origin y, yaw, square.  Returns pixel
    coordinates (N, 49, 2) and a validity mask (N, 49).
    """
    poses = np.asarray(poses, dtype=np.float64)
    count = len(poses)
    offsets = _LATTICE[None, :, :] * poses[:, 3, None, None]      # (N,49,2)
    c = np.cos(poses[:, 2])[:, None, None]
    s = np.sin(poses[:, 2])[:, None, None]
    dx, dy = offsets[:, :, 0], offsets[:, :, 1]
    world_x = poses[:, 0, None] + c[:, :, 0] * dx - s[:, :, 0] * dy
    world_y = poses[:, 1, None] + s[:, :, 0] * dx + c[:, :, 0] * dy
    world_z = np.full_like(world_x, float(board_z))
    rotation = np.asarray(calibration.camera_rotation, dtype=np.float64).reshape(3, 3)
    camera_xyz = (np.stack([world_x, world_y, world_z], axis=-1)
                  - np.asarray(calibration.camera_position)[None, None, :]) @ rotation
    depth = -camera_xyz[:, :, 2]
    valid = depth > 1e-6
    safe_depth = np.where(valid, depth, 1.0)
    u = calibration.cx + calibration.fx * camera_xyz[:, :, 0] / safe_depth
    v = calibration.cy - calibration.fy * camera_xyz[:, :, 1] / safe_depth
    valid &= (u >= 0) & (u < calibration.width) & (v >= 0) & (v < calibration.height)
    return np.stack([u, v], axis=-1), valid


def _score_poses(calibration: CameraCalibration, response: np.ndarray,
                 poses: np.ndarray, board_z: float, trim: float = 0.2,
                 exclusion_mask: np.ndarray | None = None,
                 coverage_power: float = 0.3) -> np.ndarray:
    """Score candidate poses by checker-corner response at their lattice.

    When a known-geometry mask is supplied (the robot arm), corners hidden by
    it are removed from both the mean and the coverage denominator.  A pose
    that shifts a square off the playing field then loses seven real corners
    instead of having them hidden behind the arm, which is what disambiguates
    the checker lattice's one-square period.
    """
    pixels, valid = _project_poses(calibration, poses, board_z)
    sampled = _bilinear(response, pixels[:, :, 0], pixels[:, :, 1])
    sampled = np.where(valid, sampled, 0.0)
    if exclusion_mask is not None:
        x = np.clip(np.rint(pixels[:, :, 0]).astype(int), 0, exclusion_mask.shape[1] - 1)
        y = np.clip(np.rint(pixels[:, :, 1]).astype(int), 0, exclusion_mask.shape[0] - 1)
        visible = ~exclusion_mask[y, x]
        sampled = np.where(visible, sampled, 0.0)
        valid = valid & visible
    # Robust to an unmasked occluder: drop the weakest slice of visible corners.
    ordered = np.sort(sampled, axis=1)
    visible_count = valid.sum(axis=1)
    keep = np.minimum((trim * visible_count).astype(int), np.maximum(visible_count - 1, 0))
    # Gather the top (visible - keep) values per row.
    trimmed = np.empty(len(poses), dtype=float)
    for index in range(len(poses)):
        count = int(visible_count[index])
        if count < 12:
            trimmed[index] = 0.0
        else:
            trimmed[index] = ordered[index, len(ordered[index]) - count + int(keep[index]):].mean()
    coverage = visible_count / float(INTERIOR * INTERIOR)
    return trimmed * np.power(coverage, coverage_power)


def _score_poses_chunked(calibration: CameraCalibration, response: np.ndarray,
                         poses: np.ndarray, board_z: float,
                         chunk: int = 4096,
                         exclusion_mask: np.ndarray | None = None,
                         coverage_power: float = 0.3) -> np.ndarray:
    """Score a large pose grid without materialising every projection at once."""
    scores = np.empty(len(poses), dtype=float)
    for start in range(0, len(poses), chunk):
        stop = min(len(poses), start + chunk)
        scores[start:stop] = _score_poses(
            calibration, response, poses[start:stop], board_z,
            exclusion_mask=exclusion_mask, coverage_power=coverage_power)
    return scores


def _edge_consistency(luminance: np.ndarray, calibration: CameraCalibration,
                      pose: np.ndarray, board_z: float, *,
                      samples: int = 21, offset: float = 0.20,
                      inset: float = 0.06) -> float:
    """Resolve the checker lattice's one-square phase ambiguity.

    The corner lattice repeats every square, so a shifted field still fits it.
    The playing field's edge is different: just inside is alternating checker
    colour, just outside is the uniform border.  An interior grid line has
    alternating colours on both sides.  For each of the four edges compare the
    standard deviation along the edge just inside and just outside; a real
    boundary has a low outside/inside ratio.  Returns the mean edge score
    (0..1, 1 = all four edges look like true field boundaries).
    """
    ox, oy, yaw, sq = (float(v) for v in pose)
    c, s = np.cos(yaw), np.sin(yaw)
    t = np.linspace(-1.0 + inset, 1.0 - inset, samples) * 4.0
    height, width = luminance.shape
    scores: list[float] = []
    for edge in range(4):
        if edge == 0:
            lx, ly, nx, ny = t, np.full_like(t, 4.0), np.zeros_like(t), np.ones_like(t)
        elif edge == 1:
            lx, ly, nx, ny = t, np.full_like(t, -4.0), np.zeros_like(t), -np.ones_like(t)
        elif edge == 2:
            lx, ly, nx, ny = np.full_like(t, 4.0), t, np.ones_like(t), np.zeros_like(t)
        else:
            lx, ly, nx, ny = np.full_like(t, -4.0), t, -np.ones_like(t), np.zeros_like(t)

        def side(sign: float):
            wx = ox + c * (lx - sign * nx * offset) * sq - s * (ly - sign * ny * offset) * sq
            wy = oy + s * (lx - sign * nx * offset) * sq + c * (ly - sign * ny * offset) * sq
            points = [calibration.project((x, y, board_z)) for x, y in zip(wx, wy)]
            valid = [p for p in points if p is not None]
            if len(valid) < 0.5 * samples:
                return None
            pixels = np.asarray(valid, dtype=float)
            if np.any(pixels[:, 0] < 0) or np.any(pixels[:, 0] >= width - 1):
                return None
            if np.any(pixels[:, 1] < 0) or np.any(pixels[:, 1] >= height - 1):
                return None
            return _bilinear(luminance, pixels[:, 0], pixels[:, 1])

        inside, outside = side(1.0), side(-1.0)
        if inside is None or outside is None:
            scores.append(0.0)
            continue
        ratio = float(np.std(outside) / (np.std(inside) + 1e-3))
        scores.append(float(np.clip(1.0 - ratio, 0.0, 1.0)))
    return float(np.mean(scores)) if scores else 0.0


def _refine(seed: np.ndarray, response: np.ndarray, calibration: CameraCalibration,
            luminance: np.ndarray, board_z: float,
            bounds: tuple[float, float, float, float],
            max_iter: int = 400, edge_weight: float = 0.35,
            exclusion_mask: np.ndarray | None = None,
            coverage_power: float = 0.3) -> tuple[np.ndarray, float, float]:
    """Nelder-Mead refinement with an explicit penalty outside the search box."""
    low, high = bounds
    low = np.asarray(low, dtype=float)
    high = np.asarray(high, dtype=float)

    def components(params: np.ndarray) -> tuple[float, float]:
        corner = float(_score_poses(calibration, response, params[None, :], board_z,
                                    exclusion_mask=exclusion_mask,
                                    coverage_power=coverage_power)[0])
        edge = _edge_consistency(luminance, calibration, params, board_z)
        return corner, edge

    def objective(params: np.ndarray) -> float:
        if np.any(params < low) or np.any(params > high):
            overshoot = float(np.sum(np.maximum(low - params, 0)
                                     + np.maximum(params - high, 0)))
            return 1.0 + overshoot
        corner, edge = components(params)
        return -(corner + edge_weight * edge)

    result = minimize(objective, np.asarray(seed, dtype=float), method="Nelder-Mead",
                      options={"maxiter": max_iter, "xatol": 1e-6, "fatol": 1e-7})
    params = np.clip(result.x, low, high)
    corner, edge = components(params)
    return params, corner, edge


def register_board(frame: np.ndarray, calibration: CameraCalibration,
                   board_z: float, *,
                   square_range: tuple[float, float] = (0.024, 0.032),
                   origin_search: float = 0.09,
                   yaw_search_deg: float = 15.0,
                   coarse_step: float = 0.008,
                   coarse_radius: float | None = None,
                   prior: BoardRegistration | None = None,
                   exclusion_mask: np.ndarray | None = None,
                   min_score: float = DEFAULT_MIN_SCORE,
                   refine_iterations: int = 400) -> BoardRegistration | None:
    """Estimate the board playing field from one calibrated RGB frame.

    ``prior`` narrows the search around a previous registration (for example
    the last frame) but never replaces image evidence: a frame with no visible
    checker pattern still returns ``None``.  ``exclusion_mask`` may hide known
    moving geometry (a robot arm) from the corner response; it is optional and
    the fit is already trimmed against a contiguous occluder.
    """
    frame = np.asarray(frame)
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be an H x W x 3 RGB image")
    if frame.shape[:2] != (calibration.height, calibration.width):
        raise ValueError(f"frame must have shape {(calibration.height, calibration.width, 3)}")
    if frame.dtype != np.uint8:
        if not np.issubdtype(frame.dtype, np.floating):
            raise ValueError("frame must be uint8 RGB")
        frame = np.clip(frame, 0, 255)
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)
    if not np.isfinite(board_z):
        raise ValueError("board_z must be finite")
    low_square, high_square = map(float, square_range)
    if not 0 < low_square <= high_square:
        raise ValueError("square_range must be positive and increasing")
    if origin_search <= 0 or yaw_search_deg <= 0 or coarse_step <= 0:
        raise ValueError("search ranges and coarse_step must be positive")

    luminance = frame.astype(np.float32) @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
    # The fine response localises corners to sub-millimetres but its basin is
    # only a couple of pixels wide - far too sharp for a global pose search.  A
    # larger sampling radius widens the coarse basin; the final refinement
    # returns to the fine map.
    if coarse_radius is None:
        coarse_radius = max(8.0, 14.0 * frame.shape[1] / 640.0)
    coarse_response = checker_corner_response(luminance, radius=coarse_radius)
    response = checker_corner_response(luminance)
    if exclusion_mask is not None:
        mask = np.asarray(exclusion_mask, dtype=bool)
        if mask.shape != luminance.shape:
            raise ValueError("exclusion_mask must match the frame's height and width")
        response = response.copy()
        response[mask] = 0.0
        coarse_response = coarse_response.copy()
        coarse_response[mask] = 0.0

    if prior is not None:
        centre = np.asarray(prior.origin, dtype=float)
        yaw_centre = prior.yaw
        origin_span = min(origin_search, max(0.02, 2.5 * prior.square))
        yaw_span = np.radians(min(yaw_search_deg, 6.0))
        square_centre = prior.square
    else:
        centre = np.asarray(calibration.camera_position[:2], dtype=float) * 0.0
        # The board is roughly in front of the camera; use the board's nominal
        # table position as the search centre when no prior exists.
        yaw_centre = 0.0
        origin_span = float(origin_search)
        yaw_span = np.radians(float(yaw_search_deg))
        square_centre = 0.5 * (low_square + high_square)

    # Include the exact centre in the coarse grid: an offset grid can start a
    # local refinement half a square away from the true pose.
    steps = int(np.floor(origin_span / coarse_step))
    offsets = np.arange(-steps, steps + 1, dtype=float) * coarse_step
    yaw_offsets = np.linspace(-yaw_span, yaw_span, 9)
    if prior is not None:
        square_offsets = np.array([square_centre])
    else:
        square_offsets = np.array([low_square, square_centre, high_square])
    candidates = [(centre[0] + ox, centre[1] + oy, yaw_centre + yaw, sq)
                  for ox in offsets for oy in offsets
                  for yaw in yaw_offsets for sq in square_offsets]
    poses = np.asarray(candidates, dtype=float)
    scores = _score_poses_chunked(calibration, coarse_response, poses, board_z,
                                  exclusion_mask=exclusion_mask,
                                  coverage_power=1.0)

    bounds = (
        (centre[0] - origin_span, centre[1] - origin_span,
         yaw_centre - yaw_span, low_square),
        (centre[0] + origin_span, centre[1] + origin_span,
         yaw_centre + yaw_span, high_square),
    )
    # Corner response ranks the lattice; boundary consistency resolves which
    # one-square phase is the actual playing field.  Evaluate it for a short
    # shortlist, then refine the best few distinct seeds jointly.
    shortlist = np.argsort(scores)[::-1][:32]
    ranked = sorted(shortlist,
                    key=lambda index: scores[index]
                    + 0.35 * _edge_consistency(luminance, calibration, poses[index], board_z),
                    reverse=True)
    seeds: list[np.ndarray] = []
    for index in ranked:
        candidate = poses[index]
        if all(np.linalg.norm(candidate[:2] - seed[:2]) > 0.75 * coarse_step
               or abs(candidate[2] - seed[2]) > np.radians(2.0)
               for seed in seeds):
            seeds.append(candidate)
        if len(seeds) == 4:
            break

    best_params = None
    best_score = -1.0
    best_edge = 0.0
    for seed in seeds:
        params, score, edge = _refine(seed, response, calibration, luminance,
                                      board_z, bounds, max_iter=refine_iterations,
                                      exclusion_mask=exclusion_mask,
                                      coverage_power=1.0)
        if score + 0.35 * edge > best_score + 0.35 * best_edge:
            best_params, best_score, best_edge = params, score, edge

    if best_params is None:
        return None
    pixels, valid = _project_poses(calibration, best_params[None, :], board_z)
    coverage = float(valid.sum()) / (INTERIOR * INTERIOR)
    if best_score < min_score or coverage < MIN_COVERAGE:
        return None
    corner_pixels = tuple((float(u), float(v)) for (u, v), ok in zip(pixels[0], valid[0]) if ok)
    return BoardRegistration(
        origin=(float(best_params[0]), float(best_params[1])),
        yaw=float(best_params[2]),
        square=float(best_params[3]),
        board_z=float(board_z),
        score=float(best_score),
        coverage=coverage,
        edge_score=float(best_edge),
        corner_pixels=corner_pixels,
    )


def validate_registration(registration: BoardRegistration, calibration: CameraCalibration) -> bool:
    """Cheap structural check: all corner pixels and the field centre are finite."""
    if not np.all(np.isfinite([registration.origin[0], registration.origin[1],
                               registration.yaw, registration.square])):
        return False
    if registration.square <= 0 or registration.coverage <= 0:
        return False
    centre = calibration.project((registration.origin[0], registration.origin[1],
                                  registration.board_z))
    return centre is not None and all(np.isfinite(p) for p in centre)
