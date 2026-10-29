"""Image-only foreground localization for a task-selected piece.

This is a controlled-scene baseline, not general chess-piece recognition: it
compares a fresh frame with a registered empty-board reference and searches a
bounded region around the requested source square. The reference, camera
calibration and task are observations/configuration; simulator piece poses are
never inputs to the detector.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

from .vision_geometry import CameraCalibration, Localization, localize_piece_base


def detect_piece_base(frame: np.ndarray, empty_reference: np.ndarray,
                      calibration: CameraCalibration,
                      expected_xy: tuple[float, float], board_z: float,
                      piece_height_m: float,
                      search_radius_m: float = 0.75 * 0.030,
                      centroid_height_fraction: float = 0.37,
                      difference_threshold: float = 35.0,
                      min_area_pixels: int = 25,
                      pixel_sigma: float = 1.5,
                      exclusion_mask: np.ndarray | None = None) -> Localization | None:
    """Find a foreground object near the expected square and localize its base.

    A task may supply the source square and piece type, but not its true pose.
    The foreground component's image centroid is interpreted at a calibrated
    effective height above the board to compensate for oblique-camera parallax.
    Return ``None`` for missing, too-small, out-of-frame or geometrically
    invalid detections; downstream confidence/uncertainty gates may abstain too.
    """
    frame = np.asarray(frame)
    reference = np.asarray(empty_reference)
    expected_shape = (calibration.height, calibration.width, 3)
    if frame.shape != expected_shape or reference.shape != expected_shape:
        raise ValueError(f"frames must both have shape {expected_shape}")
    if frame.dtype != np.uint8 or reference.dtype != np.uint8:
        raise ValueError("camera frames must be uint8 RGB")
    if exclusion_mask is not None:
        exclusion_mask = np.asarray(exclusion_mask, dtype=bool)
        if exclusion_mask.shape != expected_shape[:2]:
            raise ValueError(f"exclusion_mask must have shape {expected_shape[:2]}")
    if not np.isfinite(piece_height_m) or piece_height_m <= 0:
        raise ValueError("piece_height_m must be positive and finite")
    if not 0.0 < centroid_height_fraction < 1.0:
        raise ValueError("centroid_height_fraction must be between 0 and 1")
    if search_radius_m <= 0 or min_area_pixels <= 0 or difference_threshold < 0:
        raise ValueError("search radius, area and threshold must be non-negative")

    effective_z = board_z + piece_height_m * centroid_height_fraction
    points = [calibration.project((expected_xy[0] + dx,
                                   expected_xy[1] + dy, effective_z))
              for dx, dy in ((-search_radius_m, -search_radius_m),
                             (-search_radius_m, search_radius_m),
                             (search_radius_m, -search_radius_m),
                             (search_radius_m, search_radius_m))]
    if any(point is None for point in points):
        return None
    pixels = np.asarray(points, dtype=float)
    x0 = max(0, int(np.floor(pixels[:, 0].min())))
    x1 = min(calibration.width, int(np.ceil(pixels[:, 0].max())) + 1)
    y0 = max(0, int(np.floor(pixels[:, 1].min())))
    y1 = min(calibration.height, int(np.ceil(pixels[:, 1].max())) + 1)
    if x0 >= x1 or y0 >= y1:
        return None

    # Bound the foreground component by the projected physical envelope of the
    # named upright piece. During re-observation the moved robot itself differs
    # from the empty-scene reference; without this gate its arm can be the
    # largest component near the source and masquerade as the piece.
    piece_radius = 0.35 * piece_height_m
    envelope = [calibration.project((expected_xy[0] + dx,
                                     expected_xy[1] + dy, board_z + dz))
                for dx in (-piece_radius, piece_radius)
                for dy in (-piece_radius, piece_radius)
                for dz in (0.0, piece_height_m)]
    if any(point is None for point in envelope):
        return None
    envelope = np.asarray(envelope, dtype=float)
    expected_width = max(1.0, float(np.ptp(envelope[:, 0])))
    expected_height = max(1.0, float(np.ptp(envelope[:, 1])))
    max_component_area = max(4 * min_area_pixels,
                             1.8 * expected_width * expected_height)

    delta = np.linalg.norm(frame[y0:y1, x0:x1].astype(np.float32)
                           - reference[y0:y1, x0:x1].astype(np.float32), axis=2)
    if exclusion_mask is not None:
        delta[exclusion_mask[y0:y1, x0:x1]] = 0.0
    labels, count = ndimage.label(delta >= difference_threshold,
                                  structure=np.ones((3, 3), dtype=np.uint8))
    if count == 0:
        return None
    areas = np.bincount(labels.ravel(), minlength=count + 1)
    areas[0] = 0
    candidates = np.flatnonzero((areas >= min_area_pixels)
                                & (areas <= max_component_area))
    candidates = candidates[candidates != 0]
    if len(candidates) == 0:
        return None
    label = int(candidates[np.argmax(areas[candidates])])
    area = int(areas[label])

    yy, xx = np.nonzero(labels == label)
    pixel = (float(xx.mean() + x0), float(yy.mean() + y0))
    mean_delta = float(delta[labels == label].mean())
    # min_area_pixels is already the detection floor; beyond it, confidence is
    # dominated by contrast. Larger components do not represent independent
    # position measurements, so area should not impose an arbitrary 4x penalty.
    area_confidence = min(1.0, area / float(min_area_pixels))
    contrast_confidence = min(1.0, mean_delta / max(2.0 * difference_threshold, 1.0))
    confidence = float(min(0.99, area_confidence * contrast_confidence))
    # The extracted pixel is an image centroid, not a base-edge pixel. Raycast
    # it at the corresponding effective centroid height; the upright piece's
    # XY is invariant with height. Intersecting this pixel with board_z applies
    # parallax a second time and becomes badly biased for moved/oblique cameras.
    return localize_piece_base(calibration, pixel, effective_z, confidence,
                               pixel_sigma=pixel_sigma,
                               height_sigma_m=0.2 * piece_height_m)
