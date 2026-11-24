#!/usr/bin/env python3
"""Aim-point validation receipts, offline from saved captures.

Three independent checks, none of which needs the arm to move:

1. **Plane reconstruction** — the board lies flat on a table, so the plane
   carried into the arm-base frame from marker PnP + the calibrated camera
   mount must be *level*.  Across the phase-3 captures (127 deg of wrist roll)
   the deployed mount gives a few degrees of mean tilt; the B5 reference mount
   in ``chess_sim/tool_reference.py`` gives tens of degrees.  That answers the
   "which mount" question with geometry instead of preference.
2. **Image-anchored aim** — the board's own projected grid is the ground truth
   in pixels.  For a pose commanded over a square the aim ray is the tool axis,
   so `AIM_PIXEL` must land inside that square's projected quad.  The same
   numbers under the B5 mount are printed for contrast.
3. **One-axis behaviour** — the tool axis projects to a fixed image line, so
   every `AIM_PIXEL` must lie on it.  A principal-line fit reports the
   perpendicular scatter; the span along the line is the motion.  `TIP_PIXEL`
   is reported as a *regression* (it must not move), never as a finding.

    ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \\
    PYTHONPATH=. .venv/bin/python tools/aim_validate.py [--mount both] \\
        [--captures live/captures_phase3] [--out reports/aim_validate.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import cv2
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

import aim  # noqa: E402
from chess_sim.board_target import Board  # noqa: E402
from chess_sim.tool_reference import camera_in_gripper  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


def fk(calibration: dict, joints_deg) -> np.ndarray:
    """``T_base_tool`` — the same chain the live solver uses."""
    angles = np.radians(np.asarray(joints_deg, float))
    world = np.eye(4)
    for link in calibration["chain"]:
        pose = np.eye(4)
        pose[:3, :3] = np.asarray(link["rotation"], float).reshape(3, 3)
        pose[:3, 3] = np.asarray(link["position"], float)
        world = world @ pose
        if "joint_index" not in link:
            continue
        rotation = Rotation.from_rotvec(
            np.asarray(link["axis"], float)
            * (angles[int(link["joint_index"])] - float(link["reference"]))
        ).as_matrix()
        anchor = np.asarray(link["anchor"], float)
        step = np.eye(4)
        step[:3, :3] = rotation
        step[:3, 3] = anchor - rotation @ anchor
        world = world @ step
    return world


def mount_from_matrix(matrix: np.ndarray, calibration: dict) -> aim.CameraMount:
    return aim.CameraMount.from_calibration(
        {"camera_in_tool": np.asarray(matrix, float).tolist(),
         "intrinsics": calibration["intrinsics"],
         "width": calibration["width"], "height": calibration["height"]})


def b5_mount() -> np.ndarray:
    position, rotation = camera_in_gripper()
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = position
    return matrix


def fit_line(points: np.ndarray) -> dict:
    centre = points.mean(axis=0)
    centred = points - centre
    _, _, vt = np.linalg.svd(centred, full_matrices=False)
    direction = vt[0]
    along = centred @ direction
    across = centred @ np.array([-direction[1], direction[0]])
    return {"direction": [round(float(v), 4) for v in direction],
            "angle_deg": round(float(math.degrees(math.atan2(direction[1],
                                                             direction[0]))), 2),
            "perp_rms_px": round(float(np.sqrt((across ** 2).mean())), 3),
            "perp_max_px": round(float(np.abs(across).max()), 3),
            "along_span_px": round(float(along.max() - along.min()), 2)}


def square_quad(grid: dict, name: str) -> np.ndarray | None:
    for index, centre in enumerate(grid["centres"]):
        if centre["name"] == name:
            rank, col = divmod(index, 8)
            points = grid["points"]
            return np.array([points[rank][col], points[rank][col + 1],
                             points[rank + 1][col + 1], points[rank + 1][col]], float)
    return None


def inside(quad: np.ndarray, uv) -> bool:
    contour = quad.astype(np.float32)
    return cv2.pointPolygonTest(contour, (float(uv[0]), float(uv[1])), False) >= 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mount", choices=("live", "b5", "both"), default="both")
    parser.add_argument("--captures", default="live/captures_phase3")
    parser.add_argument("--calibration", default=None,
                        help="default: live_config.json's calibration")
    parser.add_argument("--board-frame", default="live/board.json",
                        help="board placement used only to name a square")
    parser.add_argument("--out", default="reports/aim_validate.json")
    parser.add_argument("--rows", type=int, default=0,
                        help="print at most this many per-view rows (0 = all)")
    args = parser.parse_args()

    config = json.loads((ROOT / "live" / "live_config.json").read_text())
    calibration_path = Path(args.calibration or config["calibration"])
    if not calibration_path.is_absolute():
        calibration_path = ROOT / calibration_path
    calibration = json.loads(calibration_path.read_text())
    board = Board(calibration)
    square_mm = float(config.get("square_m", 0.028)) * 1000.0
    board_half_mm = float(config.get("board_half_mm", 122.0))
    from kinematics import frame_at
    placement = json.loads((ROOT / args.board_frame).read_text())
    configured_board = frame_at(placement["x_mm"], placement["y_mm"],
                                placement["z_mm"], placement["yaw_deg"])

    mounts = []
    if args.mount in ("live", "both"):
        mounts.append(("live", aim.CameraMount.from_calibration(calibration)))
    if args.mount in ("b5", "both"):
        mounts.append(("b5", mount_from_matrix(b5_mount(), calibration)))

    capture_dir = ROOT / args.captures
    files = sorted(capture_dir.glob("*.jpg"))
    if not files:
        print(f"no captures in {capture_dir}", file=sys.stderr)
        return 2

    report = {"captures": str(capture_dir.relative_to(ROOT)), "views": 0, "mounts": {}}
    for name, mount in mounts:
        tracker = aim.PlaneTracker()
        rows = []
        for path in files:
            record = json.loads(path.with_suffix(".json").read_text())
            joints = [float(record["joints_deg"][k]) for k in JOINTS]
            image = cv2.imread(str(path))
            if image is None:
                continue
            solution, detections = board.solve(board.rectify(image))
            if solution is None or len(detections) < 2:
                continue
            cam_board = np.eye(4)
            cam_board[:3, :3] = solution["R"]
            cam_board[:3, 3] = solution["tvec"]
            pose = fk(calibration, joints)
            base_camera = pose @ np.block([
                [mount.rotation_gl @ aim.GL_TO_CV, mount.position.reshape(3, 1)],
                [np.zeros((1, 3)), 1.0]])
            base_board = base_camera @ cam_board
            normal = base_board[:3, 2] / np.linalg.norm(base_board[:3, 2])
            tracker.add(normal, base_board[:3, 3], key=path.name,
                        rms_px=solution["rms_px"], ids=len(detections))
            averaged = tracker.plane()
            # Speak board millimetres in the frame this very image solved, so
            # the square names and the projected grid mean the same thing.
            result = aim.evaluate(pose, mount, averaged, board_frame=base_board,
                                  board_half_mm=board_half_mm, square_mm=square_mm)
            # The placement-configured frame, for comparison with commanded mm.
            placed = aim.evaluate(pose, mount, averaged, board_frame=configured_board,
                                   board_half_mm=board_half_mm, square_mm=square_mm)
            uv_board = None
            if result.get("aim_board_mm"):
                uv_board = board.project(
                    {"rvec": solution["rvec"], "tvec": solution["tvec"]},
                    (result["aim_board_mm"][0], result["aim_board_mm"][1]), 0.0)
            grid = board.grid(solution)
            quad = square_quad(grid, result.get("square") or "")
            label = str(record.get("label") or path.stem)
            plan_square = next((tok for tok in reversed(label.split("_"))
                                if len(tok) == 2 and tok[0] in "abcdefgh"
                                and tok[1] in "12345678"), None)
            rows.append({
                "name": label,
                "plan_square": plan_square,
                "target_mm": record.get("target"),
                "tip_reported_mm": record.get("tip_mm"),
                "roll": round(joints[4], 1),
                "ids": len(detections),
                "rms_px": round(solution["rms_px"], 2),
                "plane_tilt_deg": round(math.degrees(math.acos(
                    min(1.0, abs(float(normal[2]))))), 2),
                "plane_z_mm": round(float(base_board[2, 3]) * 1000.0, 1),
                "aim_pixel": result.get("aim_pixel"),
                "tip_pixel": result.get("tip_pixel"),
                "offset_px": result.get("offset_px"),
                "offset_uv": result.get("offset_uv"),
                "aim_board_mm": result.get("aim_board_mm"),
                "aim_board_placed_mm": placed.get("aim_board_mm"),
                "square": result.get("square"),
                "in_square": bool(quad is not None and result.get("aim_pixel")
                                  and inside(quad, result["aim_pixel"])),
                "matches_plan": (plan_square is not None
                                 and result.get("square") == plan_square),
                "aim_vs_target_mm": (
                    None if not record.get("target") or not placed.get("aim_board_mm")
                    else round(float(np.hypot(
                        placed["aim_board_mm"][0] - record["target"][0],
                        placed["aim_board_mm"][1] - record["target"][1])), 1)),
                "tip_vs_target_mm": (
                    None if not record.get("target") or not record.get("tip_mm")
                    else round(float(np.hypot(
                        record["tip_mm"][0] - record["target"][0],
                        record["tip_mm"][1] - record["target"][1])), 1)),
                "square_uv": None if quad is None else
                [round(float(v), 1) for v in quad.mean(axis=0)],
                "round_trip_px": None if uv_board is None or not result.get("aim_pixel")
                else round(float(np.hypot(uv_board[0] - result["aim_pixel"][0],
                                          uv_board[1] - result["aim_pixel"][1])), 2),
                "ok": result.get("ok"),
                "reason": result.get("reason"),
            })

        tilts = np.array([r["plane_tilt_deg"] for r in rows], float)
        zs = np.array([r["plane_z_mm"] for r in rows], float)
        # Summarise the planes the tracker would actually accept: a 65 px RMS
        # solve that reaches 74 deg of tilt is a bad solve, not a bad mount.
        usable = [r for r in rows if r["rms_px"] <= tracker.max_rms_px]
        good_tilts = np.array([r["plane_tilt_deg"] for r in usable], float)
        good_zs = np.array([r["plane_z_mm"] for r in usable], float)
        tips = np.array([r["tip_pixel"] for r in rows], float)
        aims = np.array([r["aim_pixel"] for r in rows if r["aim_pixel"]], float)
        offs = np.array([r["offset_uv"] for r in rows if r["offset_uv"]], float)
        round_trip = np.array([r["round_trip_px"] for r in rows
                               if r["round_trip_px"] is not None], float)
        aim_err = np.array([r["aim_vs_target_mm"] for r in rows
                            if r["aim_vs_target_mm"] is not None], float)
        summary = {
            "views": len(rows),
            "plane_tilt_deg": {"mean": round(float(good_tilts.mean()), 2),
                               "p50": round(float(np.median(good_tilts)), 2),
                               "max": round(float(good_tilts.max()), 2),
                               "n": int(len(good_tilts)),
                               "rejected": int(len(rows) - len(usable))},
            "plane_z_mm": {"mean": round(float(good_zs.mean()), 1),
                           "spread": round(float(good_zs.max() - good_zs.min()), 1)},
            "tip_pixel": {"mean": [round(float(v), 2) for v in tips.mean(axis=0)],
                          "spread_px": [round(float(v), 3)
                                        for v in (tips.max(axis=0) - tips.min(axis=0))]},
            "aim_pixel": {"count": int(len(aims)),
                          "line": fit_line(aims) if len(aims) >= 3 else None,
                          "u_span": round(float(aims[:, 0].max() - aims[:, 0].min()), 1),
                          "v_span": round(float(aims[:, 1].max() - aims[:, 1].min()), 2)},
            "offset_uv": {"u_mean": round(float(offs[:, 0].mean()), 1),
                          "u_span": round(float(offs[:, 0].max() - offs[:, 0].min()), 1),
                          "v_span": round(float(offs[:, 1].max() - offs[:, 1].min()), 2)},
            "round_trip_px": None if not len(round_trip) else
            {"mean": round(float(round_trip.mean()), 2),
             "max": round(float(round_trip.max()), 2)},
            "in_square": int(sum(1 for r in rows if r["in_square"])),
            "matches_plan": int(sum(1 for r in rows if r["matches_plan"])),
            "plan_views": int(sum(1 for r in rows if r["plan_square"])),
            "aim_vs_target_mm": None if not len(aim_err) else
            {"n": int(len(aim_err)), "mean": round(float(aim_err.mean()), 1),
             "p50": round(float(np.median(aim_err)), 1),
             "max": round(float(aim_err.max()), 1)},
            "rows": rows,
        }
        report["mounts"][name] = summary

    report["views"] = max((m["views"] for m in report["mounts"].values()), default=0)
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")

    for name, summary in report["mounts"].items():
        print(f"\n=== mount {name} : {summary['views']} views")
        print("  plane tilt          mean %.2f  p50 %.2f  max %.2f deg  (%d views, %d rejected)"
              % (summary["plane_tilt_deg"]["mean"], summary["plane_tilt_deg"]["p50"],
                 summary["plane_tilt_deg"]["max"], summary["plane_tilt_deg"]["n"],
                 summary["plane_tilt_deg"]["rejected"]))
        print("  plane height        mean %.1f mm  spread %.1f mm"
              % (summary["plane_z_mm"]["mean"], summary["plane_z_mm"]["spread"]))
        print("  TIP_PIXEL (regr.)   mean (%.2f, %.2f)  spread (u %.3f, v %.3f) px"
              % (summary["tip_pixel"]["mean"][0], summary["tip_pixel"]["mean"][1],
                 summary["tip_pixel"]["spread_px"][0], summary["tip_pixel"]["spread_px"][1]))
        print("  AIM_PIXEL           u span %.1f  v span %.2f px"
              % (summary["aim_pixel"]["u_span"], summary["aim_pixel"]["v_span"]))
        line = summary["aim_pixel"]["line"]
        if line:
            print("  AIM_PIXEL line fit  %.1f deg  perp rms %.2f px  max %.2f px"
                  % (line["angle_deg"], line["perp_rms_px"], line["perp_max_px"]))
        print("  aim inside its projected square: %d / %d views"
              % (summary["in_square"], summary["views"]))
        print("  aim square == the plan's target square: %d / %d views"
              % (summary["matches_plan"], summary["plan_views"]))
        if summary["aim_vs_target_mm"]:
            block = summary["aim_vs_target_mm"]
            print("  aim vs commanded square centre: n %d  mean %.1f  p50 %.1f  max %.1f mm"
                  % (block["n"], block["mean"], block["p50"], block["max"]))
        if summary["round_trip_px"]:
            print("  chain<->PnP round trip  mean %.2f  max %.2f px"
                  % (summary["round_trip_px"]["mean"],
                     summary["round_trip_px"]["max"]))
        limit = len(summary["rows"]) if args.rows <= 0 else args.rows
        print("  roll  ids rms  tilt  z_mm   aim_pixel      tip   aim-tip  square plan sq_uv")
        for row in summary["rows"][:limit]:
            ap = row["aim_pixel"] or [float("nan"), float("nan")]
            sq = row["square_uv"] or [float("nan"), float("nan")]
            print("  %5.1f %2d %4.1f %5.1f %6.1f  (%5.1f,%5.1f) %6.1f %7s  %-4s %-4s (%5.1f,%5.1f)"
                  % (row["roll"], row["ids"], row["rms_px"], row["plane_tilt_deg"],
                     row["plane_z_mm"], ap[0], ap[1], row["tip_pixel"][0],
                     "-" if row["offset_px"] is None else "%.1f" % row["offset_px"],
                     row["square"] or "?", row["plan_square"] or "?", sq[0], sq[1]))
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
