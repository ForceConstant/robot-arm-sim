#!/usr/bin/env python3
"""Block 4c: intrinsics stability across the full capture set vs the cal subset.

Wraps the exact estimator in tools/solve_intrinsics.py (same object points, same
five models, same selection rule) and adds:
  * a cal-only solve (the 37 fixed-board frames that also carry joints),
  * a principal-point-fixed variant, because MuJoCo's wrist camera can only
    express a centred pinhole (cx=320, cy=240) -- if the measured cx is real
    the sim render can never match IRL pixel-for-pixel,
  * corner coverage, which is what separates distortion from focal length.

    PYTHONPATH=. robot-arm-sim python tools/block4c_intrinsics.py \
        --captures live/captures --out reports/block4c_intrinsics.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from solve_intrinsics import board_points, collect  # noqa: E402

from chess_sim import fiducials  # noqa: E402
from chess_sim.charuco_face import FIRST_ID  # noqa: E402


def is_cal(name: str) -> bool:
    return name.endswith("_cal")


def solve(frames, width, height, flags, guess=None):
    object_points = [f["objects"] for f in frames]
    image_points = [f["pixels"] for f in frames]
    if guess is None:
        guess = np.array([[746.0, 0, width / 2.0], [0, 746.0, height / 2.0], [0, 0, 1.0]])
    full = flags | (cv2.CALIB_USE_INTRINSIC_GUESS if flags & cv2.CALIB_USE_INTRINSIC_GUESS else 0)
    rms, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, (width, height), guess.copy(), None, flags=full)
    per_view = []
    for frame, rvec, tvec in zip(frames, rvecs, tvecs):
        projected, _ = cv2.projectPoints(frame["objects"], rvec, tvec, matrix, dist)
        err = np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - frame["pixels"]) ** 2, axis=1)))
        per_view.append({"name": frame["name"], "rms_px": round(float(err), 3)})
    return {
        "views": len(frames),
        "corners": int(sum(len(f["objects"]) for f in frames)),
        "rms_px": round(float(rms), 3),
        "fx": round(float(matrix[0, 0]), 3), "fy": round(float(matrix[1, 1]), 3),
        "cx": round(float(matrix[0, 2]), 3), "cy": round(float(matrix[1, 2]), 3),
        "distortion": [round(float(v), 6) for v in dist.ravel()],
        "worst": sorted(per_view, key=lambda r: -r["rms_px"])[:5],
    }


def coverage(frames):
    pixels = np.vstack([f["pixels"] for f in frames])
    return {
        "u_p05_p50_p95": [round(float(v), 1) for v in np.percentile(pixels[:, 0], [5, 50, 95])],
        "v_p05_p50_p95": [round(float(v), 1) for v in np.percentile(pixels[:, 1], [5, 50, 95])],
        "u_min_max": [round(float(pixels[:, 0].min()), 1), round(float(pixels[:, 0].max()), 1)],
        "v_min_max": [round(float(pixels[:, 1].min()), 1), round(float(pixels[:, 1].max()), 1)],
        "median_spread_px": round(float(np.median([f["spread_px"] for f in frames])), 1),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--out", default="reports/block4c_intrinsics.json")
    parser.add_argument("--square-mm", type=float, default=28.0)
    parser.add_argument("--marker-mm", type=float, default=20.0)
    args = parser.parse_args()

    frames = collect(Path(args.captures), args.square_mm, args.marker_mm)
    cal = [f for f in frames if is_cal(f["name"])]
    width, height = frames[0]["size"]
    print(f"all frames {len(frames)}, cal frames {len(cal)}, image {width}x{height}")

    models = {
        "k3_fixed_guess": cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3,
        "k3_fixed": cv2.CALIB_FIX_K3,
        "k2k3_fixed_guess": cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3,
        "free_k3": cv2.CALIB_FIX_K3 * 0,
    }
    results = {"all": {}, "cal": {}}
    for scope, fs in (("all", frames), ("cal", cal)):
        for name, flags in models.items():
            out = solve(fs, width, height, flags)
            results[scope][name] = out
            print(f"{scope:3s} {name:18s} rms {out['rms_px']:.3f}  "
                  f"fx {out['fx']:.2f} fy {out['fy']:.2f} cx {out['cx']:.2f} cy {out['cy']:.2f}")
        # principal point fixed at the sensor centre, k3 fixed
        fixed = solve(fs, width, height,
                      cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_PRINCIPAL_POINT)
        results[scope]["k3_fixed_centre_pp"] = fixed
        print(f"{scope:3s} {'k3_fixed_centre_pp':18s} rms {fixed['rms_px']:.3f}  "
              f"fx {fixed['fx']:.2f} fy {fixed['fy']:.2f} cx {fixed['cx']:.2f} cy {fixed['cy']:.2f}")

    # Shared-lens candidate: keep the measured focal lengths, force the
    # principal point to the sensor centre (the only pinhole MuJoCo can
    # render) and refit distortion. Reference: the measured all-frame K.
    ref = results["all"]["k3_fixed_guess"]
    guess = np.array([[ref["fx"], 0, width / 2.0], [0, ref["fy"], height / 2.0], [0, 0, 1.0]])
    shared_flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
                    | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_FOCAL_LENGTH)
    shared = solve(frames, width, height, shared_flags, guess=guess)
    results["all"]["shared_centre_pp_fixed_focal"] = shared
    print(f"{'all':3s} {'shared_centre_pp':18s} rms {shared['rms_px']:.3f}  "
          f"fx {shared['fx']:.2f} fy {shared['fy']:.2f} cx {shared['cx']:.2f} cy {shared['cy']:.2f} "
          f"dist {shared['distortion']}")

    payload = {"captures": len(frames), "cal_captures": len(cal),
               "image_size": [width, height], "coverage_all": coverage(frames),
               "coverage_cal": coverage(cal), "models": results}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=2) + "\n")
    print("coverage all:", json.dumps(payload["coverage_all"]))
    print("coverage cal:", json.dumps(payload["coverage_cal"]))
    print("->", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
