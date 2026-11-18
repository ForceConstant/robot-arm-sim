#!/usr/bin/env python3
"""Block 4c acceptance test: does the sim wrist camera see what the real one saw?

For every fixed-board ``_cal`` capture, take the recorded joint vector, pose the
camera with the *sim* rig (FK(q) @ solved camera-in-tool, OpenGL axes, -Z view),
project the ChArUco corner grid at the solved board pose, and compare against
the corners the real camera measured.  Real corners are undistorted with the
measured Brown-Conrady model first, so the comparison is ray-for-ray: the
principal point cancels out of the difference, which leaves only mount, board
pose, focal length and the model's own error.

Coordinate conventions, spelled out because they are where this goes wrong:

  * the calibration file stores the camera pose in OpenGL camera axes
    (+X right, +Y up, -Z view);
  * ``cv2.projectPoints`` wants OpenCV axes (+Z view), so the pose is rotated
    by ``diag(1,-1,-1)`` on the rotation only -- the flip is its own inverse;
  * MuJoCo's ``fovy`` sets ``fy = H / (2 tan(fovy/2))`` with square pixels, so
    the sim camera's focal length is ``fy`` from ``2 atan(240/fy)``.

    PYTHONPATH=. robot-arm-sim python tools/block4c_viewmatch.py
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from chess_sim.wrist_calibration import WristCalibration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handeye_bundle import board_corners, chain_fk  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
FIRST, LAST = 5, 36
FLIP = np.diag([1.0, -1.0, -1.0])
WIDTH, HEIGHT = 640, 480


def gl_to_cv(rot: np.ndarray) -> np.ndarray:
    return np.asarray(rot, float) @ FLIP


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--mount", default="reports/block4c_mount.json")
    parser.add_argument("--intrinsics", default="reports/block4c_intrinsics.json")
    parser.add_argument("--out", default="reports/block4c_viewmatch.json")
    parser.add_argument("--overlays", default="reports/block4c_overlays")
    parser.add_argument("--limit", type=int, default=4, help="overlay images to write")
    args = parser.parse_args()

    mount = json.loads(Path(args.mount).read_text())
    lens_all = json.loads(Path(args.intrinsics).read_text())
    measured = lens_all["models"]["all"]["k3_fixed_guess"]
    shared = lens_all["models"]["all"]["shared_centre_pp_fixed_focal"]

    lens_models = {
        "measured_unconstrained": (measured["fx"], measured["fy"],
                                   measured["cx"], measured["cy"], measured["distortion"]),
        "shared_centre_pp": (shared["fx"], shared["fy"],
                             shared["cx"], shared["cy"], shared["distortion"]),
    }
    fy_sim = float(shared["fy"])
    fovy = 2.0 * math.degrees(math.atan(HEIGHT / 2.0 / fy_sim))
    k_sim = np.array([[fy_sim, 0.0, WIDTH / 2.0],
                      [0.0, fy_sim, HEIGHT / 2.0],
                      [0.0, 0.0, 1.0]])
    print(f"sim lens: fy {fy_sim:.3f} px, fovy {fovy:.3f} deg, "
          f"fx {fy_sim:.3f} (square pixels)")

    camera_gl = np.asarray(mount["camera_in_tool_gl"], float)
    camera_cv = camera_gl.copy()
    camera_cv[:3, :3] = gl_to_cv(camera_gl[:3, :3])
    board = mount["board"]
    theta = math.radians(board["yaw_deg"])
    board_t = np.eye(4)
    board_t[:3, :3] = [[math.cos(theta), -math.sin(theta), 0.0],
                       [math.sin(theta), math.cos(theta), 0.0],
                       [0.0, 0.0, 1.0]]
    board_t[:3, 3] = [board["x_mm"] / 1000.0, board["y_mm"] / 1000.0, board["z_mm"] / 1000.0]

    calibration = WristCalibration.load(".private/phase11d/calibration.json")

    stats = {}
    overlays = []
    for lens_name, (lfx, lfy, lcx, lcy, ldist) in lens_models.items():
      k_lens = np.array([[lfx, 0.0, lcx], [0.0, lfy, lcy], [0.0, 0.0, 1.0]])
      dist_lens = np.asarray(ldist, float).reshape(-1, 1)
      per_view, all_errors, raw_all = [], [], []
      for item in sorted(Path(args.captures).glob("*_cal.json")):
        record = json.loads(item.read_text())
        markers = [m for m in record.get("markers", []) if FIRST <= int(m["id"]) <= LAST]
        if not markers:
            continue
        joints = record.get("state_joints_deg") or record.get("joints_deg")
        base_tool = chain_fk(calibration, [float(joints[n]) for n in JOINTS])
        world_camera = base_tool @ camera_cv
        target = np.linalg.inv(world_camera) @ board_t
        rvec = cv2.Rodrigues(target[:3, :3])[0]
        tvec = target[:3, 3].reshape(3, 1)

        objects = np.vstack([board_corners(int(m["id"])) for m in markers])
        pixels = np.vstack([np.asarray(m["corners"], float) for m in markers])
        projected, _ = cv2.projectPoints(objects.reshape(-1, 1, 3), rvec, tvec, k_sim, None)
        projected = projected.reshape(-1, 2)

        # Ray-for-ray: undistort the real corners with the SAME lens model the
        # mount was solved against, then put them on the sim's pixel scale.
        normalised = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k_lens, dist_lens)
        on_sim_grid, _ = cv2.projectPoints(
            np.column_stack([normalised.reshape(-1, 2), np.ones(len(normalised))]).reshape(-1, 1, 3),
            np.zeros(3), np.zeros(3), k_sim, None)
        on_sim_grid = on_sim_grid.reshape(-1, 2)
        distance = np.linalg.norm(projected - on_sim_grid, axis=1)
        all_errors.extend(distance.tolist())

        # Raw-pixel overlay: sim projection through the same lens onto the
        # untouched frame, so the drawing shows the true image agreement.
        raw_projected, _ = cv2.projectPoints(objects.reshape(-1, 1, 3), rvec, tvec,
                                             k_lens, dist_lens)
        raw_projected = raw_projected.reshape(-1, 2)
        raw_distance = np.linalg.norm(raw_projected - pixels, axis=1)
        raw_all.extend(raw_distance.tolist())

        name = record.get("name", item.stem)
        per_view.append({"name": name, "markers": len(markers), "corners": int(len(distance)),
                         "median_px": round(float(np.median(distance)), 3),
                         "max_px": round(float(distance.max()), 3),
                         "raw_median_px": round(float(np.median(raw_distance)), 3),
                         "raw_max_px": round(float(raw_distance.max()), 3)})
        if lens_name == "shared_centre_pp":
            overlays.append((name, item.with_suffix(".jpg"), raw_projected, pixels))

      errors = np.asarray(all_errors)
      raw_errors = np.asarray(raw_all)
      per_view.sort(key=lambda r: -r["max_px"])
      stats[lens_name] = {
          "views": len(per_view),
          "corners": int(errors.size),
          "median_px": round(float(np.median(errors)), 3),
          "mean_px": round(float(errors.mean()), 3),
          "p90_px": round(float(np.percentile(errors, 90)), 3),
          "max_px": round(float(errors.max()), 3),
          "raw_median_px": round(float(np.median(raw_errors)), 3),
          "raw_max_px": round(float(raw_errors.max()), 3),
          "per_view": per_view,
      }
      print(f"[{lens_name}] views {len(per_view)} corners {int(errors.size)}: "
            f"median {np.median(errors):.3f} p90 {np.percentile(errors, 90):.3f} max {errors.max():.3f}")

    accepted = stats["shared_centre_pp"]
    out_dir = Path(args.overlays)
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for name, jpg, projected, pixels in sorted(overlays, key=lambda o: -(o[2].shape[0]))[:args.limit]:
        image = cv2.imread(str(jpg))
        if image is None:
            continue
        for point in projected:
            cv2.circle(image, tuple(np.round(point).astype(int)), 3, (0, 0, 255), -1)
        for start in range(0, len(projected), 4):
            for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
                cv2.line(image, tuple(np.round(projected[start + a]).astype(int)),
                         tuple(np.round(projected[start + b]).astype(int)), (0, 255, 0), 1)
        path = out_dir / f"{name}_overlay.png"
        cv2.imwrite(str(path), image)
        written.append(str(path))

    summary = {
        "adopted_lens": "shared_centre_pp",
        "lens_models": stats,
        "gate": {"median_max_px": 2.0, "max_px": 4.0,
                 "median_pass": bool(accepted["median_px"] <= 2.0),
                 "max_pass": bool(accepted["max_px"] <= 4.0)},
        "sim_camera": {"fovy_deg": round(fovy, 4), "fy_px": round(fy_sim, 3),
                       "camera_in_tool_gl": [[round(float(v), 9) for v in row] for row in camera_gl]},
        "overlays": written,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(summary, indent=2) + "\n")

    print("gate on the adopted (sim-representable) lens:", summary["gate"])
    for view in accepted["per_view"][:6]:
        print("  ", view)
    print("overlays:", written)
    print("->", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
