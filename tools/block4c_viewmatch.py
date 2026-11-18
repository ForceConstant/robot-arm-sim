#!/usr/bin/env python3
"""Block 4c acceptance test: does the sim wrist camera see what the real one saw?

For every fixed-board ``_cal`` capture, take the recorded joint vector, pose the
camera with the *sim* rig (FK(q) @ solved camera-in-tool, OpenGL axes, -Z view),
project the ChArUco corner grid at the solved board pose, and compare against
the corners the real camera measured.  Real corners are undistorted with the
measured Brown-Conrady model first, so the comparison is ray-for-ray: the
principal point cancels out of the difference, which leaves only mount, board
pose, focal length and the model's own error.

Two statistics are reported and both are reproducible here:

  * ``all`` -- every corner of every cal view that recorded a detection;
  * ``valid`` -- only views that can actually be measured: **>=2 detected
    markers** (a single tag is a two-fold-ambiguous planar PnP and gives no
    cross-check) and **no tag clipped by the frame border**.  A tag counts as
    clipped when one of its corners is closer to the image border than one
    marker module (the marker's own black border, ``edge_px / 6``), plus 1 px.
    Excluded frames are listed with the reason and get an overlay of their own.

The gate is unchanged: median <= 2 px and max <= 4 px on the valid set.

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


def border_distance(pixels: np.ndarray) -> float:
    return float(min(pixels[:, 0].min(), pixels[:, 1].min(),
                     WIDTH - 1 - pixels[:, 0].max(), HEIGHT - 1 - pixels[:, 1].max()))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="live/captures")
    parser.add_argument("--mount", default="reports/block4c_mount.json")
    parser.add_argument("--intrinsics", default="reports/block4c_intrinsics.json")
    parser.add_argument("--out", default="reports/block4c_viewmatch.json")
    parser.add_argument("--overlays", default="reports/block4c_overlays")
    parser.add_argument("--limit", type=int, default=4, help="valid-set overlays to write")
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
    print(f"sim lens: fy {fy_sim:.3f} px, fovy {fovy:.3f} deg, fx {fy_sim:.3f} (square pixels)")

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

    stats, overlays = {}, []
    for lens_name, (lfx, lfy, lcx, lcy, ldist) in lens_models.items():
        k_lens = np.array([[lfx, 0.0, lcx], [0.0, lfy, lcy], [0.0, 0.0, 1.0]])
        dist_lens = np.asarray(ldist, float).reshape(-1, 1)
        per_view, valid_errors, all_errors, multi_errors = [], [], [], []
        for item in sorted(Path(args.captures).glob("*_cal.json")):
            record = json.loads(item.read_text())
            markers = [m for m in record.get("markers", []) if FIRST <= int(m["id"]) <= LAST]
            if not markers:
                continue
            joints = record.get("state_joints_deg") or record.get("joints_deg")
            base_tool = chain_fk(calibration, [float(joints[n]) for n in JOINTS])
            target = np.linalg.inv(base_tool @ camera_cv) @ board_t
            rvec = cv2.Rodrigues(target[:3, :3])[0]
            tvec = target[:3, 3].reshape(3, 1)

            objects = np.vstack([board_corners(int(m["id"])) for m in markers])
            pixels = np.vstack([np.asarray(m["corners"], float) for m in markers])
            projected, _ = cv2.projectPoints(objects.reshape(-1, 1, 3), rvec, tvec, k_sim, None)
            projected = projected.reshape(-1, 2)

            normalised = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k_lens, dist_lens)
            on_sim_grid, _ = cv2.projectPoints(
                np.column_stack([normalised.reshape(-1, 2), np.ones(len(normalised))]).reshape(-1, 1, 3),
                np.zeros(3), np.zeros(3), k_sim, None)
            on_sim_grid = on_sim_grid.reshape(-1, 2)
            distance = np.linalg.norm(projected - on_sim_grid, axis=1)
            all_errors.extend(distance.tolist())

            raw_projected, _ = cv2.projectPoints(objects.reshape(-1, 1, 3), rvec, tvec,
                                                 k_lens, dist_lens)
            raw_projected = raw_projected.reshape(-1, 2)

            # A tag is clipped when its outer border (one module) can fall
            # outside the frame: the detected corners span the 6-module marker,
            # so a corner less than edge_px/6 + 1 px from the edge has its
            # border cut, and its sub-pixel centre is biased by the cut.
            smallest = min(float(m.get("edge_px") or 0.0) for m in markers)
            margin = (smallest / 6.0 + 1.0) if smallest else 16.0
            border = border_distance(pixels)
            clipped = border < margin
            single = len(markers) < 2
            valid = not (clipped or single)
            reasons = [r for r, flag in (("single tag", single), ("tag clipped at frame border", clipped)) if flag]

            name = record.get("name", item.stem)
            entry = {"name": name, "markers": len(markers), "corners": int(len(distance)),
                     "median_px": round(float(np.median(distance)), 3),
                     "max_px": round(float(distance.max()), 3),
                     "border_px": round(border, 1), "clip_margin_px": round(margin, 1),
                     "valid": valid, "excluded_because": ", ".join(reasons)}
            per_view.append(entry)
            if valid:
                valid_errors.extend(distance.tolist())
            if not single:
                multi_errors.extend(distance.tolist())
            if lens_name == "shared_centre_pp" and (valid or not valid):
                overlays.append((name, item.with_suffix(".jpg"), raw_projected, valid))

        all_array = np.asarray(all_errors)
        valid_array = np.asarray(valid_errors)
        multi_array = np.asarray(multi_errors)
        stats[lens_name] = {
            "views": len(per_view),
            "corners": int(all_array.size),
            "median_px": round(float(np.median(all_array)), 3),
            "max_px": round(float(all_array.max()), 3),
            "valid": {
                "views": sum(1 for v in per_view if v["valid"]),
                "corners": int(valid_array.size),
                "median_px": round(float(np.median(valid_array)), 3),
                "mean_px": round(float(valid_array.mean()), 3),
                "p90_px": round(float(np.percentile(valid_array, 90)), 3),
                "max_px": round(float(valid_array.max()), 3),
                "gate": {"median_max_px": 2.0, "max_px": 4.0,
                         "median_pass": bool(np.median(valid_array) <= 2.0),
                         "max_pass": bool(valid_array.max() <= 4.0),
                         "pass": bool(np.median(valid_array) <= 2.0 and valid_array.max() <= 4.0)},
            },
            "multi_tag_only": {
                "rule": ">=2 detected markers, clipping ignored",
                "views": sum(1 for v in per_view if v["markers"] >= 2),
                "corners": int(multi_array.size),
                "median_px": round(float(np.median(multi_array)), 3),
                "p90_px": round(float(np.percentile(multi_array, 90)), 3),
                "max_px": round(float(multi_array.max()), 3),
            },
            "excluded": [v for v in per_view if not v["valid"]],
            "per_view": sorted(per_view, key=lambda r: (r["valid"], -r["max_px"])),
        }
        print(f"[{lens_name}] multi-tag {stats[lens_name]['multi_tag_only']['views']} views / "
              f"{stats[lens_name]['multi_tag_only']['corners']} corners: median "
              f"{stats[lens_name]['multi_tag_only']['median_px']:.3f} max "
              f"{stats[lens_name]['multi_tag_only']['max_px']:.3f}")
        print(f"[{lens_name}] all {stats[lens_name]['corners']} corners; "
              f"valid {stats[lens_name]['valid']['views']} views / "
              f"{stats[lens_name]['valid']['corners']} corners: "
              f"median {stats[lens_name]['valid']['median_px']:.3f} "
              f"p90 {stats[lens_name]['valid']['p90_px']:.3f} "
              f"max {stats[lens_name]['valid']['max_px']:.3f} "
              f"-> {'PASS' if stats[lens_name]['valid']['gate']['pass'] else 'FAIL'}")

    accepted = stats["shared_centre_pp"]
    out_dir = Path(args.overlays)
    out_dir.mkdir(parents=True, exist_ok=True)
    written, excluded_written = [], []
    kept, dropped = [], []
    for name, jpg, projected, valid in sorted(overlays, key=lambda o: -o[2].shape[0]):
        image = cv2.imread(str(jpg))
        if image is None:
            continue
        for point in projected:
            cv2.circle(image, tuple(np.round(point).astype(int)), 3, (0, 0, 255), -1)
        for start in range(0, len(projected), 4):
            for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
                cv2.line(image, tuple(np.round(projected[start + a]).astype(int)),
                         tuple(np.round(projected[start + b]).astype(int)), (0, 255, 0), 1)
        if valid and len(kept) >= args.limit:
            continue
        path = out_dir / f"{name}_overlay{'' if valid else '_excluded'}.png"
        cv2.imwrite(str(path), image)
        (kept if valid else dropped).append(str(path))
    written = kept
    excluded_written = dropped

    summary = {
        "adopted_lens": "shared_centre_pp",
        "valid_view_rule": ">=2 detected markers and no tag clipped at the frame border "
                           "(corner closer than edge_px/6 + 1 px)",
        "gate_thresholds_px": {"median": 2.0, "max": 4.0},
        "lens_models": stats,
        "sim_camera": {"fovy_deg": round(fovy, 4), "fy_px": round(fy_sim, 3),
                       "camera_in_tool_gl": [[round(float(v), 9) for v in row] for row in camera_gl]},
        "overlays_valid": written,
        "overlays_excluded": excluded_written,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(summary, indent=2) + "\n")

    print("gate:", accepted["valid"]["gate"])
    print("excluded:")
    for view in accepted["excluded"]:
        print(f"  {view['name']} markers {view['markers']} border {view['border_px']} px "
              f"< {view['clip_margin_px']} px: {view['excluded_because']} "
              f"(median {view['median_px']}, max {view['max_px']})")
    print("excluded overlays:", excluded_written)
    print("->", args.out)
    return 0 if accepted["valid"]["gate"]["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
