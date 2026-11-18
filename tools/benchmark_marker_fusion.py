"""Fusion gate: recover a moved board from the planned marker scan.

The planner proves the six poses *should* see the markers.  This tool closes the
loop: move the board to a non-nominal pose in the simulator, render the real
wrist frames from the committed plan, run the actual ArUco detector, fuse the
corners through :func:`chess_sim.fiducials.estimate_board_pose`, and compare the
recovered board pose with the truth.

It reports two errors, because they fail differently:

* board-pose error (mm, degrees) — the registration itself;
* base-frame offset error (mm) — what Phase 11d's GUI will display, which also
  includes the arm-base datum.

No simulator camera or board transform reaches the estimator: each view is an
RGB frame plus the calibrated camera for the measured joints, exactly what the
Pi sends.

    MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
    ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
    PYTHONPATH=. .venv/bin/python tools/benchmark_marker_fusion.py
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from chess_sim import fiducials
from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.marker_scan import (board_offset, load_plan, marker_corners_world,
                                   observe_markers)
from chess_sim.product import current_layout, layout_env_config
from chess_sim.self_occlusion import SelfOcclusionModel
from chess_sim.wrist_calibration import WristCalibration

DEFAULT_SCENARIOS = (
    ("nominal", 0.0, 0.0, 0.0),
    ("shift_x_20mm", 0.020, 0.0, 0.0),
    ("shift_y_15mm", 0.0, -0.015, 0.0),
    ("yaw_3deg", 0.0, 0.0, 3.0),
    ("combined", -0.025, 0.018, -3.5),
)


def build_env(origin, yaw):
    config = layout_env_config("printed", cameras=())
    board = config.board.model_copy(update={"origin": (float(origin[0]), float(origin[1])),
                                            "yaw": float(yaw)})
    return ChessSimEnv(config.model_copy(update={"board": board}))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", default="chess_sim/assets/marker_scan_plan.json")
    parser.add_argument("--calibration", default=".private/phase9/calibration.json")
    parser.add_argument("--out", default="reports/marker_fusion.json")
    parser.add_argument("--save-dir", default=".private/phase11d/fusion")
    parser.add_argument("--samples", type=int, default=1,
                        help="repeats per scenario (renderer is deterministic)")
    parser.add_argument("--sweep", action="store_true",
                        help="grid of placements instead of the named scenarios")
    parser.add_argument("--sweep-mm", default="40", help="half-extent of the placement grid")
    parser.add_argument("--sweep-step-mm", default="20")
    parser.add_argument("--sweep-yaw-deg", default="5")
    parser.add_argument("--only", default="",
                        help="regex filter on scenario name; run one per process "
                             "when the full sweep exhausts memory")
    args = parser.parse_args()

    plan = load_plan(args.plan)
    recovery = json.loads(Path(args.plan).read_text()).get("recovery", {})
    calibration = WristCalibration.load(args.calibration)
    self_model = SelfOcclusionModel(calibration)
    layout = fiducials.MarkerLayout.for_board(224.0, 15.0, plane="mat", margin_mm=40.0)
    mount = current_layout()["mount"]
    base_xy = (mount["white_arm_base_frame_mm"][0] / 1000.0,
               mount["white_arm_base_frame_mm"][1] / 1000.0)

    if args.sweep:
        span = float(args.sweep_mm)
        step = float(args.sweep_step_mm)
        yaws = (-float(args.sweep_yaw_deg), 0.0, float(args.sweep_yaw_deg))
        values = np.arange(-span, span + step / 2, step)
        def tag(value: float) -> str:
            return f"m{abs(int(round(value)))}" if value < 0 else f"p{int(round(value))}"

        scenarios = [
            (f"x{tag(dx)}_y{tag(dy)}_r{tag(yaw)}",
             dx / 1000.0, dy / 1000.0, yaw)
            for dx in values for dy in values for yaw in yaws
        ]
    else:
        scenarios = list(DEFAULT_SCENARIOS)
    if args.only:
        import re
        pattern = re.compile(args.only)
        scenarios = [item for item in scenarios if pattern.search(item[0])]
        if not scenarios:
            raise SystemExit(f"no scenario matches {args.only!r}")
    rows = []
    for name, dx, dy, dyaw_deg in scenarios:
        env = build_env((dx, dy), math.radians(dyaw_deg))
        try:
            env.reset("8/8/8/8/8/8/8/8")
            board = env.board
            marker_z = board.table_top + fiducials.marker_height_above_table(board.mat)
            truth_origin = (board.origin[0], board.origin[1])
            truth_yaw = board.yaw
            plan_missing = []
            for pose in plan.poses:
                env.data.qpos[env._joint_qpos] = np.asarray(pose.joints)
                mujoco.mj_forward(env.model, env.data)
                camera = calibration.camera(pose.joints)
                predicted = observe_markers(
                    camera, pose.joints, layout,
                    board_origin=(truth_origin[0], truth_origin[1]),
                    board_yaw=truth_yaw, marker_z=marker_z,
                    occlusion=self_model.exclusion_mask(pose.joints))
                missing = set(fiducials.MARKER_IDS) - set(predicted)
                if missing:
                    plan_missing.append((pose.name, sorted(missing)))

            views = []
            detections = {}
            for index, pose in enumerate(plan.poses, start=1):
                env.data.qpos[env._joint_qpos] = np.asarray(pose.joints)
                mujoco.mj_forward(env.model, env.data)
                frame = env.render(Camera.WRIST).copy()
                camera = calibration.camera(pose.joints)
                found = fiducials.detect_markers(frame)
                detections[pose.name] = sorted(d.marker_id for d in found)
                if found:
                    views.append((frame, camera))
                if args.save_dir:
                    path = Path(args.save_dir) / name
                    path.mkdir(parents=True, exist_ok=True)
                    Image.fromarray(frame).save(path / f"pose-{index:02d}.png")

            # Recovery: a placement that pushes a marker out of every planned
            # view is exactly the case the live scan must survive.  Visit the
            # best scored pose for each unseen marker before giving up.
            used = []
            seen = {marker for ids in detections.values() for marker in ids}
            for marker in fiducials.MARKER_IDS:
                if marker in seen or str(marker) not in recovery:
                    continue
                entry = recovery[str(marker)]
                env.data.qpos[env._joint_qpos] = np.asarray(entry["joints"])
                mujoco.mj_forward(env.model, env.data)
                frame = env.render(Camera.WRIST).copy()
                camera = calibration.camera(entry["joints"])
                found = fiducials.detect_markers(frame)
                detections[f"recovery-{entry['name']}"] = sorted(d.marker_id for d in found)
                used.append(marker)
                if found:
                    views.append((frame, camera))

            registration = fiducials.estimate_board_pose(
                views, board.square, board.top, marker_z=marker_z, layout=layout,
                min_markers=2) if views else None
            if registration is None:
                rows.append({"scenario": name, "dx_mm": dx * 1000, "dy_mm": dy * 1000,
                             "dyaw_deg": dyaw_deg, "detections": detections,
                             "truth": {"x_mm": truth_origin[0] * 1000,
                                       "y_mm": truth_origin[1] * 1000,
                                       "yaw_deg": math.degrees(truth_yaw)},
                             "plan_missing": plan_missing,
                             "registration": None, "error": "solve failed"})
                print(f"{name}: SOLVE FAILED detections={detections}", flush=True)
                continue

            dx_mm = (registration.origin[0] - truth_origin[0]) * 1000.0
            dy_mm = (registration.origin[1] - truth_origin[1]) * 1000.0
            dyaw = math.degrees(registration.yaw - truth_yaw)
            truth_offset = board_offset(truth_origin, truth_yaw, base_xy=base_xy,
                                        board_half_mm=board.width * 500.0)
            seen_offset = board_offset(registration.origin, registration.yaw,
                                       base_xy=base_xy,
                                       board_half_mm=board.width * 500.0)
            rows.append({
                "scenario": name, "dx_mm": dx * 1000, "dy_mm": dy * 1000,
                "dyaw_deg": dyaw_deg, "detections": detections,
                "recovery_used": used,
                "marker_views": {str(marker): sum(1 for ids in detections.values()
                                                  if marker in ids)
                                 for marker in fiducials.MARKER_IDS},
                "full_coverage": all(
                    sum(1 for ids in detections.values() if marker in ids) >= 1
                    for marker in fiducials.MARKER_IDS),
                "coverage": round(registration.coverage, 3),
                "views": len(views), "plan_missing": plan_missing,
                "registration": {"x_mm": round(registration.origin[0] * 1000, 3),
                                 "y_mm": round(registration.origin[1] * 1000, 3),
                                 "yaw_deg": round(math.degrees(registration.yaw), 4),
                                 "score": round(registration.score, 3)},
                "board_error": {"dx_mm": round(dx_mm, 3), "dy_mm": round(dy_mm, 3),
                                "dyaw_deg": round(dyaw, 4),
                                "position_mm": round(math.hypot(dx_mm, dy_mm), 3)},
                "truth_offset": truth_offset.as_dict(),
                "seen_offset": seen_offset.as_dict(),
                "offset_error_mm": round(abs(seen_offset.x_mm - truth_offset.x_mm)
                                         + abs(seen_offset.y_mm - truth_offset.y_mm), 3),
            })
            print(f"{name}: err {math.hypot(dx_mm, dy_mm):.2f} mm / {dyaw:+.3f} deg "
                  f"views={len(views)} det={detections}", flush=True)
        finally:
            env.close()

    def passed(row):
        return (row.get("registration") is not None
                and row["board_error"]["position_mm"] <= 2.0
                and abs(row["board_error"]["dyaw_deg"]) <= 1.0)

    ok = all(passed(row) for row in rows)
    failing = [row["scenario"] for row in rows if not passed(row)]
    payload = {"schema": "robot-arm-marker-fusion/1", "generated_at": "2026-11-15",
               "plan": args.plan, "calibration": calibration.calibration_id,
               "gate": {"position_mm": 2.0, "yaw_deg": 1.0},
               "scenarios": rows, "passed": ok,
               "scenario_count": len(rows), "failing": failing}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(payload, indent=2) + "\n")
    print(f"passed={ok} -> {args.out}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
