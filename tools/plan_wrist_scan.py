"""Plan settled wrist scan poses that see every square of the product board.

Candidates are wrist-lens targets on a grid above the board; each is solved for
joints (near-vertical, inside hard limits) by a bounded global search, rendered,
and scored by the arm's own occlusion of each square.  A greedy set cover then
picks poses so every square is clearly visible in at least ``--min-views``.

    MUJOCO_GL=egl ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-v3 \
    ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
    PYTHONPATH=. .venv/bin/python tools/plan_wrist_scan.py --out chess_sim/assets/wrist_scan_plan.json
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image
from scipy.optimize import differential_evolution

from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.perception import board_patches, square_mask_fraction
from chess_sim.perception_data import render_mask, robot_geometry_ids
from chess_sim.product import layout_env_config
from chess_sim.vision_geometry import CameraCalibration

SCHEMA = "robot-arm-wrist-scan-plan/1"


def solve_pose(env, target, seed: int) -> tuple[np.ndarray, float, float]:
    """Near-vertical wrist camera with the lens at ``target``.

    The wrist camera looks along the gripper's approach axis, so a board-facing
    view needs a bent wrist, not the grasp pose.  The search therefore runs on
    the arm-only model (cheap, same camera mount), seeded from the grasp IK and
    bounded by the scene's actuator limits, then verified on the full model.
    """
    target = np.asarray(target, dtype=float)
    data, model = env.data, env.model
    cid = model.camera(str(Camera.WRIST)).id
    limits = np.asarray(model.actuator_ctrlrange[env._actuators][:5], dtype=float)

    def cost(q):
        data.qpos[env._joint_qpos[:5]] = q
        mujoco.mj_forward(model, data)
        direction = -data.cam_xmat[cid].reshape(3, 3)[:, 2]
        return (100.0 * float(np.sum((data.cam_xpos[cid] - target) ** 2))
                + 3.0 * float(np.sum((direction - np.array([0.0, 0.0, -1.0])) ** 2)))

    bounds = [(float(lo), float(hi)) for lo, hi in limits]
    result = differential_evolution(cost, bounds, seed=seed, popsize=10,
                                    maxiter=140, tol=1e-8, polish=True)
    joints = np.r_[result.x, 0.6]
    data.qpos[env._joint_qpos] = joints
    mujoco.mj_forward(model, data)
    cid = model.camera(str(Camera.WRIST)).id
    gap = float(np.linalg.norm(data.cam_xpos[cid] - target))
    clearance = float(data.cam_xpos[cid][2] - env.board.top)
    return joints, clearance, gap


def score_pose(env, joints, registration) -> tuple[np.ndarray, np.ndarray]:
    model, data = env.model, env.data
    data.qpos[env._joint_qpos] = joints
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    camera = CameraCalibration.from_mujoco(model, data, str(Camera.WRIST), 640, 480)
    mask = render_mask(env, Camera.WRIST, robot_geometry_ids(env))
    covered = square_mask_fraction(mask, camera, registration)
    frame = env.render(Camera.WRIST).copy()
    _, valid = board_patches(frame, camera, registration)
    clear = (covered < 0.08) & (valid > 0.95)
    return clear, covered


def greedy(clear_rows: list[np.ndarray], min_views: int) -> list[int]:
    chosen: list[int] = []
    counts = np.zeros(64, dtype=int)
    while counts.min() < min_views and len(chosen) < len(clear_rows):
        best, best_gain = None, -1
        for index, row in enumerate(clear_rows):
            if index in chosen:
                continue
            gain = int(np.sum((counts < min_views) & row))
            if gain > best_gain:
                best, best_gain = index, gain
        if best is None or best_gain <= 0:
            break
        chosen.append(best)
        counts += clear_rows[best]
    return chosen


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="chess_sim/assets/wrist_scan_plan.json")
    parser.add_argument("--min-views", type=int, default=3)
    parser.add_argument("--heights", default="0.16,0.22,0.28")
    parser.add_argument("--extent", type=float, default=0.15)
    parser.add_argument("--grid", type=int, default=5)
    parser.add_argument("--save-dir", default=".private/phase9/plan")
    args = parser.parse_args()

    env = ChessSimEnv(layout_env_config("printed", cameras=()))
    try:
        env.reset("8/8/8/8/8/8/8/8")
        camera = CameraCalibration.from_mujoco(env.model, env.data, str(Camera.WRIST), 640, 480)
        from chess_sim.board_registration import BoardRegistration
        registration = BoardRegistration(env.board.origin, env.board.yaw, env.board.square,
                                         env.board.top, 1.0, 1.0)
        targets = []
        for height in (float(v) for v in args.heights.split(",")):
            for gx in np.linspace(-args.extent, args.extent, args.grid):
                for gy in np.linspace(-args.extent, args.extent, args.grid):
                    targets.append((float(gx), float(gy), env.board.top + height))
        rows, meta = [], []
        for index, target in enumerate(targets):
            joints, clearance, gap = solve_pose(env, target, seed=1000 + index)
            if clearance < 0.10 or gap > 0.005:
                continue
            clear, covered = score_pose(env, joints, registration)
            rows.append(clear)
            meta.append({"target": target, "joints": joints.tolist(),
                         "clearance_m": clearance,
                         "clear_squares": int(clear.sum()),
                         "occluded": int((covered > 0.05).sum())})
            print(f"{index}/{len(targets)} clear={int(clear.sum())} occ={int((covered>0.05).sum())}",
                  flush=True)
        chosen = greedy(rows, args.min_views)
        counts = np.sum([rows[i] for i in chosen], axis=0) if chosen else np.zeros(64, dtype=int)
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)
        for order, index in enumerate(chosen):
            env.data.qpos[env._joint_qpos] = np.asarray(meta[index]["joints"])
            mujoco.mj_forward(env.model, env.data)
            Image.fromarray(env.render(Camera.WRIST).copy()).save(
                Path(args.save_dir) / f"pose-{order + 1:02d}.png")
        plan = {
            "schema": SCHEMA,
            "generated_at": "2026-11-03",
            "layout": "hardware/current_layout.json (90 mm gap, 30 mm riser, custom jaw v3)",
            "method": "wrist-lens targets on a grid, near-vertical, inside hard joint limits; "
                      "greedy set cover on arm occlusion",
            "min_views": args.min_views,
            "candidates": len(targets),
            "poses": [{"name": f"v{i + 1}", "joints": meta[index]["joints"]}
                      for i, index in enumerate(chosen)],
            "coverage": [int(v) for v in counts],
            "covered_squares": int((counts >= args.min_views).sum()),
        }
        out = Path(args.out)
        out.write_text(json.dumps(plan, indent=2) + "\n")
        print("candidate clear-square counts:", [m["clear_squares"] for m in meta])
        print("best candidate:", max(m["clear_squares"] for m in meta))
        print(json.dumps({k: v for k, v in plan.items() if k != "poses" and k != "coverage"}, indent=2))
        print("min/median views per square:", int(counts.min()), int(np.median(counts)))
        print(f"poses: {len(chosen)} -> {out}")
        Path(args.save_dir, "candidates.json").write_text(json.dumps(meta, indent=2) + "\n")
        return 0
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
