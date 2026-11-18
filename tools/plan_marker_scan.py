"""Plan settled wrist poses that resolve all four board markers.

Phase 9 planned for the 64 squares; the markers sit 28 mm beyond the corner
squares and the committed plan never sees ids 0 and 1.  This tool solves camera
targets that point at the marker ring itself, scores each pose with the same
pure-geometry rules the live runtime uses (:mod:`chess_sim.marker_scan`), and
greedily picks the smallest set that sees every marker ``--min-views`` times.

Run on the BC-250 simulation workstation (or any host with EGL rendering and the
private tool/piece asset packs):

    MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
    ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
    PYTHONPATH=. .venv/bin/python tools/plan_marker_scan.py
"""
from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image

from chess_sim import fiducials, gripper
from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.marker_scan import (MarkerLayout, MarkerObservation,
                                   MarkerScanPlan, PoseCoverage, load_plan,
                                   observe_markers, plan_from_observations,
                                   save_plan)
from chess_sim.product import layout_env_config
from chess_sim.self_occlusion import SelfOcclusionModel
from chess_sim.wrist_calibration import WristCalibration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plan_wrist_scan import solve_pose  # noqa: E402


def robust_cover(entries, visibility, placements, *, min_markers: int = 3,
                 seed=(), positions=None):
    """Pick poses so every sampled placement sees ``min_markers`` distinct ids."""
    by_name = {}
    joints_by_name = {}
    for name, joints, found in entries:
        by_name[name] = found
        joints_by_name[name] = tuple(float(v) for v in joints)
    names = list(by_name)
    covered = [set() for _ in placements]
    chosen: list[str] = [name for name in seed if name in by_name]
    for name in chosen:
        for index, found in enumerate(visibility[name]):
            covered[index] |= found
    while True:
        best, best_gain = None, -1
        for name in names:
            if name in chosen:
                continue
            gain = 0
            for index, found in enumerate(visibility[name]):
                if len(covered[index]) >= min_markers:
                    continue
                if len(covered[index] | found) >= min_markers:
                    gain += 1
            if gain > best_gain:
                best, best_gain = name, gain
        if best is None or best_gain <= 0:
            break
        chosen.append(best)
        for index, found in enumerate(visibility[best]):
            covered[index] |= found
    uncovered = sum(1 for found in covered if len(found) < min_markers)
    positions = positions or {}
    poses = [
        PoseCoverage(name=name, joints=joints_by_name[name],
                     visible=tuple(sorted(by_name[name])),
                     edge_px=tuple(by_name[name][m].edge_px
                                   for m in sorted(by_name[name])),
                     camera_position=positions.get(name))
        for name in chosen]
    return poses, uncovered


def order_poses(poses: list[PoseCoverage]) -> list[PoseCoverage]:
    """Nearest-neighbour ordering in joint space; travel dominates scan time."""
    if len(poses) <= 2:
        return list(poses)
    remaining = list(poses)
    ordered = [remaining.pop(0)]
    while remaining:
        here = np.asarray(ordered[-1].joints)
        nxt = min(remaining, key=lambda pose: float(np.abs(
            np.asarray(pose.joints) - here).sum()))
        remaining.remove(nxt)
        ordered.append(nxt)
    return ordered


def _candidates(board, layout: fiducials.MarkerLayout, args) -> list[dict]:
    items: list[dict] = []
    heights = [float(v) for v in args.heights.split(",")]
    offsets = [(0.0, 0.0)]
    for step in (float(v) for v in args.offsets.split(",") if v):
        offsets += [(step, 0.0), (-step, 0.0), (0.0, step), (0.0, -step)]
    for height in heights:
        for marker_id in fiducials.MARKER_IDS:
            corners = np.asarray(layout.corners_mm(marker_id), dtype=float) / 1000.0
            centre = corners.mean(axis=0)
            # The near-edge markers (ids 0 and 1) only resolve from directly
            # overhead, so give them a lateral fan: a second, independent
            # viewpoint beats a duplicate pose at another height.
            fan = list(offsets) if marker_id in (0, 1) else list(offsets[:1])
            if args.tolerance_mm > 0:
                sx = 1.0 if centre[0] > 0 else -1.0
                sy = 1.0 if centre[1] > 0 else -1.0
                pad = args.tolerance_mm / 1000.0
                fan.append((sx * pad, sy * pad))
                fan.append((sx * pad, 0.0))
                fan.append((0.0, sy * pad))
            for dx, dy in fan:
                items.append({
                    "kind": "marker", "marker_id": marker_id,
                    "position": np.array([centre[0] + dx, centre[1] + dy,
                                          board.top + height]),
                    "direction": np.array([0.0, 0.0, -1.0]),
                })
        if abs(height - heights[len(heights) // 2]) > 1e-9:
            continue
        extent = args.extent + args.tolerance_mm / 1000.0
        for gx in np.linspace(-extent, extent, args.grid):
            for gy in np.linspace(-extent, extent, args.grid):
                items.append({
                    "kind": "grid",
                    "position": np.array([gx, gy, board.top + height]),
                    "direction": np.array([0.0, 0.0, -1.0]),
                })
    if args.oblique:
        aim = np.array([board.origin[0], board.origin[1], board.top])
        for elevation in (float(v) for v in args.elevations.split(",")):
            for radius in (float(v) for v in args.radii.split(",")):
                for index in range(args.azimuths):
                    theta = 2.0 * math.pi * index / args.azimuths
                    tilt = math.radians(elevation)
                    position = aim + radius * np.array([
                        math.sin(tilt) * math.cos(theta),
                        math.sin(tilt) * math.sin(theta),
                        math.cos(tilt)])
                    if position[2] - board.top < 0.06:
                        continue
                    if max(abs(position[0]), abs(position[1])) > 0.42:
                        continue
                    direction = aim - position
                    items.append({"kind": "oblique", "position": position,
                                  "direction": direction / np.linalg.norm(direction)})
    return items


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="chess_sim/assets/marker_scan_plan.json")
    parser.add_argument("--calibration", default=".private/phase9/calibration.json")
    parser.add_argument("--min-views", type=int, default=2)
    parser.add_argument("--min-separation", type=float, default=0.025,
                        help="reject a repeat view nearer than this, in metres")
    parser.add_argument("--tolerance-mm", type=float, default=0.0,
                        help="board placement tolerance the plan must survive")
    parser.add_argument("--tolerance-yaw-deg", type=float, default=0.0)
    parser.add_argument("--min-markers", type=int, default=3,
                        help="distinct markers required for every sampled placement")
    parser.add_argument("--scores", default=".private/phase11d/marker_scores.json",
                        help="cache of solved candidates; reused when it exists")
    parser.add_argument("--heights", default="0.20,0.26,0.32")
    parser.add_argument("--offsets", default="0.04",
                        help="lateral fan around the near markers, comma-separated metres")
    parser.add_argument("--extent", type=float, default=0.15)
    parser.add_argument("--grid", type=int, default=3)
    parser.add_argument("--oblique", action="store_true")
    parser.add_argument("--azimuths", type=int, default=8)
    parser.add_argument("--elevations", default="35,55")
    parser.add_argument("--radii", default="0.20,0.28")
    parser.add_argument("--reuse-plan", default="",
                        help="score an existing marker plan first and keep its poses")
    parser.add_argument("--report", default="reports/marker_scan_plan.json")
    parser.add_argument("--save-dir", default=".private/phase11d/marker-plan")
    args = parser.parse_args()

    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset("8/8/8/8/8/8/8/8")
        board = env.board
        calibration = WristCalibration.load(args.calibration)
        field_mm = (board.width - 2 * board.border) * 1000.0
        layout = fiducials.MarkerLayout.for_board(
            field_mm, board.border * 1000.0, plane=board.marker_plane,
            margin_mm=40.0)
        marker_z = board.table_top + fiducials.marker_height_above_table(board.mat)
        self_model = SelfOcclusionModel(calibration)
        jaw_angle = 0.5 * (gripper.JOINT_MIN + gripper.JOINT_MAX)
        print(f"board {board.width*1000:.1f} mm, marker offset {layout.offset_mm:.1f} mm, "
              f"marker_z {marker_z*1000:.2f} mm", flush=True)

        candidates = _candidates(board, layout, args)
        if args.reuse_plan:
            for pose in load_plan(args.reuse_plan).poses:
                joints = np.asarray(pose.joints, dtype=float)
                joints[5] = float(np.clip(joints[5], gripper.JOINT_MIN,
                                          gripper.JOINT_MAX))
                candidates.insert(0, {"kind": "reused", "name": pose.name,
                                      "joints": joints})

        entries: list[tuple[str, tuple[float, ...], dict]] = []
        positions: dict[str, tuple[float, float, float]] = {}
        meta: list[dict] = []
        cache_path = Path(args.scores) if args.scores else None
        if cache_path and cache_path.exists():
            cached = json.loads(cache_path.read_text())
            if cached.get("calibration") == calibration.calibration_id:
                print(f"reusing {len(cached['candidates'])} solved candidates",
                      flush=True)
                for item in cached["candidates"]:
                    found = {int(marker): MarkerObservation(
                        marker_id=int(marker),
                        corners_px=tuple(tuple(map(float, corner)) for corner in value["corners_px"]),
                        edge_px=float(value["edge_px"]))
                        for marker, value in item["visible"].items()}
                    joints = tuple(map(float, item["joints"]))
                    entries.append((item["name"], joints, found))
                    positions[item["name"]] = tuple(
                        float(v) for v in calibration.camera(joints).camera_position)
                    meta.append(item)
                candidates = []
            else:
                print("score cache is for another calibration; re-solving", flush=True)
        for index, candidate in enumerate(candidates):
            if "joints" in candidate:
                joints, clearance, gap = candidate["joints"], 0.0, 0.0
            else:
                joints, clearance, gap = solve_pose(
                    env, candidate["position"], seed=2000 + index,
                    direction=candidate["direction"])
                # solve_pose hardcodes the stock jaw angle; the installed tool
                # has its own travelled range, and the occlusion model refuses
                # an out-of-limit measurement.
                joints = np.r_[np.asarray(joints, dtype=float)[:5], jaw_angle]
                if clearance < 0.10 or gap > 0.005:
                    print(f"{index}/{len(candidates)} {candidate['kind']} reject "
                          f"(clearance {clearance:.3f} m, gap {gap*1000:.1f} mm)", flush=True)
                    continue
            env.data.qpos[env._joint_qpos] = joints
            mujoco.mj_forward(env.model, env.data)
            camera = calibration.camera(joints)
            mask = self_model.exclusion_mask(joints)
            found = observe_markers(
                camera, joints, layout, board_origin=board.origin, board_yaw=board.yaw,
                marker_z=marker_z, occlusion=mask)
            name = candidate.get("name") or f"{candidate['kind']}{index}"
            entries.append((name, tuple(float(v) for v in joints), found))
            positions[name] = tuple(float(v) for v in camera.camera_position)
            meta.append({"name": name, "kind": candidate["kind"],
                         "joints": [float(v) for v in joints],
                         "position": [float(v) for v in candidate["position"]],
                         "visible": sorted(found),
                         "edge_px": {str(k): round(v.edge_px, 1)
                                     for k, v in sorted(found.items())},
                         "clearance_m": round(clearance, 4)})
            print(f"{index}/{len(candidates)} {name} visible={sorted(found)} "
                  f"edges={ {k: round(v.edge_px, 1) for k, v in sorted(found.items())} }",
                  flush=True)

        if cache_path and candidates:
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            cache_path.write_text(json.dumps({
                "schema": "robot-arm-marker-scores/1",
                "calibration": calibration.calibration_id,
                "candidates": [
                    {**item, "visible": {
                        str(marker): {"corners_px": [list(map(float, c))
                                                     for c in found[marker].corners_px],
                                      "edge_px": float(found[marker].edge_px)}
                        for marker in sorted(found)}}
                    for item, (_, _, found) in zip(meta, entries)],
            }, indent=2) + "\n")

        if args.tolerance_mm > 0 and args.min_markers < 2:
            raise SystemExit("a tolerant plan needs at least two markers per placement")

        if args.tolerance_mm > 0:
            span = args.tolerance_mm / 1000.0
            yaws = np.linspace(-math.radians(args.tolerance_yaw_deg),
                               math.radians(args.tolerance_yaw_deg),
                               3 if args.tolerance_yaw_deg else 1)
            placements = [(dx, dy, yaw)
                          for dx in np.linspace(-span, span, 5)
                          for dy in np.linspace(-span, span, 5)
                          for yaw in yaws]
            visibility: dict[str, list[set[int]]] = {}
            for name, joints, _ in entries:
                camera = calibration.camera(joints)
                mask = self_model.exclusion_mask(joints)
                visibility[name] = [
                    set(observe_markers(
                        camera, joints, layout,
                        board_origin=(dx, dy), board_yaw=yaw,
                        marker_z=marker_z, occlusion=mask))
                    for dx, dy, yaw in placements]
            # Start from the nominal-quality plan: a tolerant scan must not
            # trade away the case that actually happens most of the time.
            seed_plan = plan_from_observations(
                entries, min_views=args.min_views,
                layout_offset_mm=layout.offset_mm, positions=positions,
                min_separation_m=args.min_separation)
            seed_names = [pose.name for pose in seed_plan.poses]
            chosen, uncovered = robust_cover(
                entries, visibility, placements, min_markers=args.min_markers,
                seed=seed_names, positions=positions)
            chosen_names = {pose.name for pose in chosen}
            nominal = Counter()
            for name, _, found in entries:
                if name in chosen_names:
                    nominal.update(found.keys())
            ordered = order_poses(chosen)
            plan = MarkerScanPlan(
                poses=tuple(ordered),
                min_views=args.min_views,
                layout_offset_mm=layout.offset_mm,
                note=(f"placement tolerance +/-{args.tolerance_mm:.0f} mm, "
                      f"+/-{args.tolerance_yaw_deg:.0f} deg; "
                      f"{len(placements)-uncovered}/{len(placements)} placements "
                      f"reach {args.min_markers} markers; "
                      f"nominal views {dict(sorted(nominal.items()))}"))
            print(f"placements {len(placements)}, uncovered {uncovered}, "
                  f"poses {len(ordered)}", flush=True)
        else:
            plan = plan_from_observations(entries, min_views=args.min_views,
                                          layout_offset_mm=layout.offset_mm,
                                          positions=positions,
                                          min_separation_m=args.min_separation)
        counts = plan.views_per_marker()
        print("views per marker:", counts, flush=True)
        if plan.missing():
            print(f"WARNING: uncovered markers {plan.missing()}; "
                  "add heights/oblique candidates or relax min-views", flush=True)

        chosen_names = {pose.name for pose in plan.poses}

        def visible_ids(item) -> set[int]:
            value = item["visible"]
            return {int(key) for key in value} if isinstance(value, dict) else set(value)

        recovery = {}
        for marker_id in fiducials.MARKER_IDS:
            options = [item for item in meta
                       if marker_id in visible_ids(item)
                       and item["name"] not in chosen_names]
            if not options:
                continue
            best = max(options, key=lambda item: item["edge_px"][str(marker_id)])
            recovery[str(marker_id)] = {
                "name": best["name"], "joints": best["joints"],
                "edge_px": best["edge_px"][str(marker_id)]}
        plan = MarkerScanPlan(poses=plan.poses, min_views=plan.min_views,
                              layout_offset_mm=plan.layout_offset_mm,
                              note=plan.note, recovery=recovery)
        out = save_plan(plan, args.out,
                        generated_at="2026-11-15",
                        tolerance_mm=args.tolerance_mm,
                        tolerance_yaw_deg=args.tolerance_yaw_deg,
                        min_markers=args.min_markers,
                        layout="hardware/current_layout.json (90 mm gap, 30 mm riser)",
                        calibration=args.calibration,
                        method="wrist-lens targets over the marker ring and board grid, "
                               "solved inside hard joint limits; coverage scored by "
                               "projected marker edge, frame margin and authored-mesh "
                               "arm occlusion; greedy set cover")
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report).write_text(json.dumps({
            "schema": "robot-arm-marker-scan-report/1",
            "plan": str(out),
            "min_views": plan.min_views,
            "views_per_marker": {str(k): v for k, v in counts.items()},
            "missing": list(plan.missing()),
            "candidates": meta,
            "sources": {"calibration": calibration.calibration_id,
                        "occlusion": self_model.sources},
        }, indent=2, default=str) + "\n")

        if args.save_dir:
            Path(args.save_dir).mkdir(parents=True, exist_ok=True)
            for order, pose in enumerate(plan.poses):
                env.data.qpos[env._joint_qpos] = np.asarray(pose.joints)
                mujoco.mj_forward(env.model, env.data)
                Image.fromarray(env.render(Camera.WRIST).copy()).save(
                    Path(args.save_dir) / f"pose-{order + 1:02d}.png")
        print(f"poses: {len(plan.poses)} -> {out}")
        return 0 if plan.complete() else 1
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
