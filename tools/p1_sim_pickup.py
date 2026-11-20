#!/usr/bin/env python3
"""P1 sim pickup — first version: see a piece from the wrist frame, then grasp it.

The pick is driven by the image, not by simulator truth:

1. pose the arm so the target square is inside the wrist view (projection search);
2. register the view on the printed ChArUco face with solvePnP (same detector as
   the real rig, tools/p1_piece_detect.py);
3. turn the detected square into board millimetres and then world metres;
4. hand that Localization to ``env.execute_localized`` so the planner approaches
   the *measured* point.  ``env.evaluate`` still scores against physical state,
   which is what makes the number worth anything.

    env ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=$PWD/.private/gripper-assets-5y \
        ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=$PWD/.private/piece-assets-v5 \
        PYTHONPATH=. MUJOCO_GL=egl .venv/bin/python tools/p1_sim_pickup.py --square e2
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from chess_sim.config import Camera
from chess_sim.env import ChessSimEnv
from chess_sim.product import layout_env_config
from chess_sim.tasks import CaptureTask
from chess_sim.vision_geometry import Localization, localize_piece_base
from chess_sim.wrist_calibration import WristCalibration
from p1_piece_detect import Board, square_table

REFERENCE = ROOT / "chess_sim/assets/scene/generated/board_charuco.png"


def sim_intrinsics(env, camera: str) -> dict:
    cid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
    fy = 480.0 / (2 * math.tan(math.radians(float(env.model.cam_fovy[cid])) / 2))
    return {"intrinsics": [fy, fy, 320.0, 240.0], "distortion": [0.0] * 5,
            "chain": (), "camera_in_tool": np.eye(4).tolist(), "calibrated": True}


class Poser:
    """Joints that put a target point inside the wrist frame."""

    LO = np.array([-2.0, -2.9, -1.7, -1.7, -2.75])
    HI = np.array([2.0, 0.5, 3.0, 1.8, 2.85])

    def __init__(self, env: ChessSimEnv) -> None:
        self.env = env
        self.cid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
        self.qadr = env._joint_qpos[:5]
        self.fy = 480.0 / (2 * math.tan(math.radians(float(env.model.cam_fovy[self.cid])) / 2))

    def state(self, joints):
        self.env.data.qpos[self.qadr] = joints
        mujoco.mj_forward(self.env.model, self.env.data)
        return (self.env.data.cam_xpos[self.cid].copy(),
                self.env.data.cam_xmat[self.cid].reshape(3, 3).copy())

    def pixel(self, pos, mat, point):
        camera = mat.T @ (point - pos)
        depth = -camera[2]
        if depth <= 1e-4:
            return None
        return np.array([320.0 + self.fy * camera[0] / depth,
                         240.0 - self.fy * camera[1] / depth])

    def score(self, joints, target):
        pos, mat = self.state(joints)
        height = (pos[2] - self.env.board.top) * 1000
        downward = float(np.dot(-mat[:, 2], [0.0, 0.0, -1.0]))
        uv = self.pixel(pos, mat, target)
        if uv is None:
            return -1e6
        error = np.hypot((uv[0] - 320) / 320.0, (uv[1] - 240) / 240.0)
        return (-2.0 * error - 0.4 * max(0.0, 120 - height) / 100.0
                - 0.4 * max(0.0, height - 210) / 100.0 - 2.0 * (1.0 - downward))

    def solve(self, target, seed=None):
        rng = np.random.default_rng(11)
        best = None
        for _ in range(3000):
            joints = rng.uniform(self.LO, self.HI)
            value = self.score(joints, target)
            if best is None or value > best[0]:
                best = (value, joints)
        joints, value = best[1].copy(), best[0]
        step = np.radians(6.0)
        for _ in range(400):
            improved = False
            for k in range(5):
                for sign in (1, -1):
                    trial = joints.copy()
                    trial[k] = np.clip(trial[k] + sign * step, self.LO[k], self.HI[k])
                    candidate = self.score(trial, target)
                    if candidate > value:
                        joints, value, improved = trial, candidate, True
            if not improved:
                step *= 0.6
                if step < 1e-4:
                    break
        return joints, value


def piece_at_square(rows, rect, valid, n, pxm, reference, board, solution, square):
    """Deviation local to one square, with a compactness gate."""
    row = next((r for r in rows if r["square"] == square), None)
    if row is None:
        return None, "square not in view"
    empty = (row["ncc"] > 0.70) if row["parity"] == "light" else (row["std"] < 20.0)
    if empty:
        return None, (f"printed face visible (ncc {row['ncc']:.2f}, std {row['std']:.1f}) "
                      "- no piece on this square")
    difference = cv2.absdiff(rect, reference)
    mask = ((difference > 60) & (valid > 0)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    u = int(round((row["centre_mm"][0] + 128.0) * pxm))
    v = int(round((128.0 - row["centre_mm"][1]) * pxm))
    label = labels[min(max(v, 0), n - 1), min(max(u, 0), n - 1)]
    if label == 0:
        # a piece standing on the square projects outward; take the closest blob
        best = None
        for index in range(1, count):
            area = stats[index, cv2.CC_STAT_AREA] / pxm ** 2
            if not 150.0 <= area <= 2500.0:
                continue
            cx_mm = centroids[index][0] / pxm - 128.0
            cy_mm = 128.0 - centroids[index][1] / pxm
            distance = math.hypot(cx_mm - row["centre_mm"][0], cy_mm - row["centre_mm"][1])
            if distance > 26.0:
                continue
            if best is None or distance < best[0]:
                best = (distance, area, cx_mm, cy_mm)
        if best is None:
            return None, "no deviating blob within 26 mm of the square"
        _, area, cx_mm, cy_mm = best
        return {"square": square, "component_area_mm2": round(float(area), 0),
                "centroid_on_plane_mm": [round(float(cx_mm), 1), round(float(cy_mm), 1)],
                "offset_from_centre_mm": round(float(math.hypot(
                    cx_mm - row["centre_mm"][0], cy_mm - row["centre_mm"][1])), 1)}, None
    area = stats[label, cv2.CC_STAT_AREA] / pxm ** 2
    if not 150.0 <= area <= 2500.0:
        return None, f"component at the square is {area:.0f} mm2"
    cx_mm = centroids[label][0] / pxm - 128.0
    cy_mm = 128.0 - centroids[label][1] / pxm
    return {"square": square, "component_area_mm2": round(float(area), 0),
            "centroid_on_plane_mm": [round(float(cx_mm), 1), round(float(cy_mm), 1)],
            "offset_from_centre_mm": round(float(math.hypot(
                cx_mm - row["centre_mm"][0], cy_mm - row["centre_mm"][1])), 1)}, None


def board_xy_to_world(env, xy_mm):
    yaw = float(env.board.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = xy_mm[0] / 1000.0, xy_mm[1] / 1000.0
    return (env.board.origin[0] + c * x - s * y, env.board.origin[1] + s * x + c * y)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--square", default="e2")
    parser.add_argument("--face", default=None, choices=("charuco", "wood"),
                        help="override the scene board face; the default follows the config")
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--no-grasp", action="store_true")
    parser.add_argument("--out", default="reports/p1_sim_pickup.json")
    args = parser.parse_args()

    config = layout_env_config()
    if args.face is not None:
        config = config.model_copy(update={"board": config.board.model_copy(
            update={"face": args.face})})
    env = ChessSimEnv(config)
    obs = env.reset()
    slot = env.slot_at(args.square)
    if slot is None:
        raise SystemExit(f"{args.square} is empty")
    truth = env.piece_position(slot)
    print(f"target {args.square} truth {np.round(truth * 1000, 1)} mm")

    poser = Poser(env)
    joints, value = poser.solve(truth)
    env.data.qpos[poser.qadr] = joints
    mujoco.mj_forward(env.model, env.data)
    cam = env.data.cam_xpos[poser.cid].copy()
    print("pose", np.round(np.degrees(joints), 1), "score", round(value, 3),
          "camera mm", np.round(cam * 1000, 1),
          "height", round((cam[2] - env.board.top) * 1000, 1))

    board = Board(sim_intrinsics(env, "wrist"))
    reference = cv2.imread(str(REFERENCE), cv2.IMREAD_GRAYSCALE)
    frames, estimates = [], []
    for index in range(max(1, args.frames)):
        image = env.render(Camera.WRIST)[:, :, :3]
        frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        solution, detections = board.solve(frame)
        if solution is None:
            print(f"frame {index}: {len(detections)} markers, no pose")
            continue
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rect, valid, n, pxm = board.rectified(grey, solution, 4.0)
        expected = cv2.resize(reference, (n, n), interpolation=cv2.INTER_AREA)
        rows = square_table(frame, expected, rect, valid, n, pxm, board.layout, board, solution)
        piece, why = piece_at_square(rows, rect, valid, n, pxm, expected, board, solution,
                                     args.square)
        row = next((r for r in rows if r["square"] == args.square), None)
        print(f"frame {index}: ids {solution['ids']} rms {solution['rms_px']:.2f}px "
              f"squares {len(rows)} {args.square} row {row and (row['ncc'], row['std'])} "
              f"piece {piece and piece['centroid_on_plane_mm']} {why or ''}")
        frames.append((frame, solution, rows, piece))
        if piece and "centroid_on_plane_mm" in piece:
            estimates.append(piece)
        env.data.qpos[poser.qadr] = joints + np.radians([0.15, -0.1, 0.05, 0.1, -0.2])
        mujoco.mj_forward(env.model, env.data)
    if not estimates:
        print("detector found no piece; aborting before any arm motion")
        return 3

    # pick the detection nearest the requested square (the task names the piece)
    square_centre = ((ord(args.square[0]) - 97 + 0.5) * 28.0 - 112.0,
                     (int(args.square[1]) - 1 + 0.5) * 28.0 - 112.0)
    estimates.sort(key=lambda p: math.hypot(p["centroid_on_plane_mm"][0] - square_centre[0],
                                            p["centroid_on_plane_mm"][1] - square_centre[1]))
    print("candidates:", [(p["square"], p["centroid_on_plane_mm"], p["component_area_mm2"])
                          for p in estimates])
    chosen = estimates[0]
    xy_mm = chosen["centroid_on_plane_mm"]
    world = board_xy_to_world(env, xy_mm)
    error_mm = float(np.hypot(world[0] - truth[0], world[1] - truth[1]) * 1000)
    spread = np.array([e["centroid_on_plane_mm"] for e in estimates], float).std(axis=0) \
        if len(estimates) > 1 else np.zeros(2)
    print(f"detected {chosen['square']} at {xy_mm} mm, area {chosen['component_area_mm2']} mm2")
    print(f"world {np.round(np.array(world) * 1000, 1)} truth {np.round(truth[:2] * 1000, 1)} "
          f"error {error_mm:.2f} mm, spread {np.round(spread, 2)} mm")

    # return the arm to the observation pose before planning the grasp
    env.data.qpos[poser.qadr] = joints
    mujoco.mj_forward(env.model, env.data)

    localization = Localization(world_xy=(float(world[0]), float(world[1])),
                                pixel_xy=(float(chosen["centroid_on_plane_mm"][0]),
                                          float(chosen["centroid_on_plane_mm"][1])),
                                confidence=1.0, sigma_m=0.002)
    result = None
    clip = []
    wrist = []
    if not args.no_grasp:
        before = env.piece_snapshot()
        orbit = env.orbit_default()

        def on_step(action):
            if env._step_count % 6 == 0:
                picture = env.render(orbit=orbit)
                clip.append(cv2.cvtColor(picture, cv2.COLOR_RGB2BGR).copy())
                eye = cv2.cvtColor(env.render(Camera.WRIST), cv2.COLOR_RGB2BGR)
                cv2.putText(eye, f"wrist | {args.square} | step {env._step_count}", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                wrist.append(eye)

        result = env.execute_localized(CaptureTask(square=args.square), localization,
                                       on_step=on_step)
        after = env.piece_snapshot()
        captured = env.slot_at(args.square) is None
        cv2.imwrite(str(ROOT / "reports/p1_sim_after.png"),
                    cv2.cvtColor(env.render(orbit=orbit), cv2.COLOR_RGB2BGR))
    else:
        before = after = env.piece_snapshot()
        captured = None

    receipt = {
        "schema": "p1-sim-pickup/1",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "square": args.square,
        "board_face": config.board.face,
        "pose_joints_deg": {name: round(float(v), 2) for name, v in
                            zip(("shoulder_pan", "shoulder_lift", "elbow_flex",
                                 "wrist_flex", "wrist_roll"), np.degrees(joints))},
        "camera_world_mm": np.round(cam * 1000, 1).tolist(),
        "camera_height_mm": round(float((cam[2] - env.board.top) * 1000), 1),
        "detection": chosen,
        "truth_world_mm": np.round(truth[:2] * 1000, 1).tolist(),
        "board_mm": xy_mm,
        "world_mm": [round(v * 1000, 2) for v in world],
        "error_mm": round(error_mm, 2),
        "spread_mm": np.round(spread, 2).tolist(),
        "frames": len(frames),
        "grasp": None if result is None else {
            "success": bool(result.success),
            "reason": result.reason,
            "steps": int(result.steps),
            "square_empty_after": captured,
        },
        "localization": {"world_xy_m": [float(world[0]), float(world[1])],
                         "confidence": 1.0, "sigma_m": 0.002},
    }
    (ROOT / args.out).write_text(json.dumps(receipt, indent=2) + "\n")
    print("grasp:", receipt["grasp"])
    print("wrote", args.out)

    if frames:
        frame, solution, rows, piece = frames[0]
        annotated = annotate_sim(frame, board, solution, rows, piece)
        cv2.imwrite(str(ROOT / "reports/p1_sim_before.png"), annotated)
    if clip:
        height, width = clip[0].shape[:2]
        writer = cv2.VideoWriter(str(ROOT / "reports/p1_sim_clip.mp4"),
                                 cv2.VideoWriter_fourcc(*"mp4v"), 12.0, (width, height))
        for index, picture in enumerate(clip):
            cv2.putText(picture, f"sim capture e2 | frame {index}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            writer.write(picture)
        writer.release()
        picks = clip[::max(1, len(clip) // 12)][:12]
        tiles = [cv2.resize(f, (320, 240)) for f in picks]
        cols = 4
        rows_n = (len(tiles) + cols - 1) // cols
        sheet = np.zeros((rows_n * 240, cols * 320, 3), np.uint8)
        for index, tile in enumerate(tiles):
            r, c = divmod(index, cols)
            sheet[r * 240:(r + 1) * 240, c * 320:(c + 1) * 320] = tile
        cv2.imwrite(str(ROOT / "reports/p1_sim_sheet.png"), sheet)
        print("clip frames", len(clip))
    if wrist:
        height, width = wrist[0].shape[:2]
        writer = cv2.VideoWriter(str(ROOT / "reports/p1_sim_wrist.mp4"),
                                 cv2.VideoWriter_fourcc(*"mp4v"), 12.0, (width, height))
        for picture in wrist:
            writer.write(picture)
        writer.release()
        picks = wrist[::max(1, len(wrist) // 12)][:12]
        cols = 4
        rows_n = (len(picks) + cols - 1) // cols
        sheet = np.zeros((rows_n * 240, cols * 320, 3), np.uint8)
        for index, frame in enumerate(picks):
            r, c = divmod(index, cols)
            sheet[r * 240:(r + 1) * 240, c * 320:(c + 1) * 320] = cv2.resize(frame, (320, 240))
        cv2.imwrite(str(ROOT / "reports/p1_sim_wrist_sheet.png"), sheet)
        print("wrist frames", len(wrist))
    return 0


def annotate_sim(frame, board, solution, rows, piece):
    canvas = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
    for detection in __import__("chess_sim.fiducials", fromlist=["x"]).detect_markers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)):
        quad = np.array(detection.corners, np.float32) * 2
        cv2.polylines(canvas, [quad.astype(np.int32)], True, (0, 220, 120), 2)
        cv2.putText(canvas, str(detection.marker_id), tuple(quad.mean(0).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 220, 120), 2, cv2.LINE_AA)
    for row in rows:
        uv = (np.array(board.project(solution, row["centre_mm"])) * 2).astype(int)
        cv2.circle(canvas, tuple(uv), 5, (0, 220, 255), -1)
        cv2.putText(canvas, f"{row['square']} {row['ncc']:.2f}", tuple(uv + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1, cv2.LINE_AA)
    if piece and "centroid_on_plane_mm" in piece:
        uv = (np.array(board.project(solution, piece["centroid_on_plane_mm"])) * 2).astype(int)
        cv2.drawMarker(canvas, tuple(uv), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
    lines = [f"sim wrist | PnP {solution['rms_px']:.2f}px over {solution['n_corners']} corners",
             f"cam board {np.round(solution['camera_in_board_mm'], 1)} mm"]
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (12, 30 + index * 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


if __name__ == "__main__":
    raise SystemExit(main())
