#!/usr/bin/env python3
"""Solve camera intrinsics and distortion from ChArUco captures.

Replaces the stored `fx 746` with a measured lens: `cv2.calibrateCamera` over
every capture that has at least one marker, using the printed board's exact
geometry as the object points. No arm pose and no hand-eye are involved, so the
result is independent of the mount error that has been confusing everything
downstream.

    robot-arm-sim python tools/solve_intrinsics.py --captures live/captures \
        --out reports/intrinsics.json

Point coverage matters more than view count: distortion is only separable when
markers appear near the frame edges as well as the centre. Report the per-view
reprojection too -- a single bad view can pull a focal length by tens of pixels.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from chess_sim import fiducials
from chess_sim.charuco_face import FIRST_ID, corners_mm

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


def board_points(marker_id: int, square_mm: float, marker_mm: float) -> np.ndarray:
    corners = corners_mm(marker_id, square_mm, marker_mm)
    return np.array([[x / 1000.0, y / 1000.0, 0.0] for x, y in corners], np.float32)


def collect(captures: Path, square_mm: float, marker_mm: float):
    frames = []
    for item in sorted(captures.glob("*.json")):
        try:
            record = json.loads(item.read_text())
        except ValueError:
            continue
        image_path = item.with_suffix(".jpg")
        if not image_path.exists():
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        detections = [d for d in fiducials.detect_markers(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                      if FIRST_ID <= d.marker_id < FIRST_ID + 32]
        if not detections:
            continue
        objects, pixels = [], []
        for d in detections:
            objects.append(board_points(d.marker_id, square_mm, marker_mm))
            pixels.append(np.asarray(d.corners, np.float32))
        spread = float(np.std(np.vstack(pixels)))
        frames.append({"name": record.get("name", item.stem),
                       "objects": np.vstack(objects),
                       "pixels": np.vstack(pixels),
                       "spread_px": spread,
                       "size": image.shape[1::-1]})
    return frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--out", default="reports/intrinsics.json")
    parser.add_argument("--square-mm", type=float, default=28.0)
    parser.add_argument("--marker-mm", type=float, default=20.0)
    parser.add_argument("--fx-guess", type=float, default=746.0)
    args = parser.parse_args()

    frames = collect(Path(args.captures), args.square_mm, args.marker_mm)
    print(f"frames with markers: {len(frames)}")
    if len(frames) < 3:
        print("need at least 3 views to solve intrinsics")
        return

    width, height = frames[0]["size"]
    object_points = [f["objects"] for f in frames]
    image_points = [f["pixels"] for f in frames]
    guess = np.array([[args.fx_guess, 0, width / 2.0],
                      [0, args.fx_guess, height / 2.0],
                      [0, 0, 1.0]])

    # Five distortion coefficients on ten thin views overfits badly (k3 came out
    # at -13) and then drags the focal length with it.  Solve every model, show
    # them all, and prefer a constrained one unless it is clearly worse.
    candidates = []
    for flags in (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3,
                  cv2.CALIB_FIX_K3,
                  cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
                  0):
        try:
            rms, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
                object_points, image_points, (width, height), guess.copy(), None, flags=flags)
        except cv2.error:
            continue
        candidates.append((rms, matrix, dist, rvecs, tvecs, flags))
    if not candidates:
        print("no solution")
        return
    best_rms = min(c[0] for c in candidates)
    stable = [c for c in candidates if c[5] & cv2.CALIB_FIX_K3 and c[0] <= best_rms * 1.25]
    chosen = min(stable, key=lambda c: c[0]) if stable else min(candidates, key=lambda c: c[0])
    for rms, matrix, dist, _, _, flags in candidates:
        print(f"  model k3-fixed={bool(flags & cv2.CALIB_FIX_K3)} guess={bool(flags & cv2.CALIB_USE_INTRINSIC_GUESS)}"
              f"  rms {rms:.3f}  fx {matrix[0,0]:.1f} fy {matrix[1,1]:.1f}")
    rms, matrix, dist, rvecs, tvecs, flags = chosen
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])

    per_view = []
    for frame, rvec, tvec in zip(frames, rvecs, tvecs):
        projected, _ = cv2.projectPoints(frame["objects"], rvec, tvec, matrix, dist)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - frame["pixels"]) ** 2, axis=1))))
        per_view.append((frame["name"], round(error, 2), round(frame["spread_px"], 1)))

    print(f"rms reprojection {rms:.3f} px   (flags {'guess' if flags else 'none'})")
    print(f"fx {fx:.2f}  fy {fy:.2f}  cx {cx:.2f}  cy {cy:.2f}")
    print("distortion", np.round(dist.ravel(), 5).tolist())
    worst = sorted(per_view, key=lambda r: -r[1])[:5]
    print("worst views:", worst)

    report = {
        "captures_used": len(frames),
        "rms_px": round(float(rms), 3),
        "fx": round(fx, 3), "fy": round(fy, 3),
        "cx": round(cx, 3), "cy": round(cy, 3),
        "distortion": [round(float(v), 6) for v in dist.ravel()],
        "image_size": [width, height],
        "worst_views": worst,
        "per_view": [{"name": n, "rms_px": e, "spread_px": s} for n, e, s in per_view],
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2) + "\n")
    print("->", args.out)


if __name__ == "__main__":
    sys.exit(main())
