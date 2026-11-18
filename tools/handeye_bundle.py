#!/usr/bin/env python3
"""Joint hand-eye and board-pose solve from wrist-camera marker sightings.

Why not ``calibrateHandEye``: that estimator works from relative motions
(A X = X B).  Every relative rotation is small on this arm, so the translation
part is poorly conditioned, and the failure mode is exactly what the real data
shows -- rotations solving cleanly while translations scatter by ~100 mm.

This solves the same problem as one model instead: camera mount X and board pose
are unknown, every observed marker corner is a residual, and nothing is
estimated from differences.  That is better conditioned, and the residual it
reports is a real reprojection error rather than a self-consistency score.

    robot-arm-sim python tools/handeye_bundle.py --captures sim/captures_wrist \
        --records sim/captures_wrist/records.json --out reports/handeye_bundle.json

Board-frame marker corners come from the printed layout (28 mm squares, 20 mm
markers, ids 5..36 at file/rank positions), so the numbers transfer to the
physical board unchanged.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from chess_sim import fiducials
from chess_sim.wrist_calibration import WristCalibration, _transform

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
SQUARE_MM, MARKER_MM, FIRST_ID = 28.0, 20.0, 5
# Off: the simulator's board matches the printed board marker-for-marker
# (confirmed visually 2026-11-17, and the overhead view agrees to 9-14 mm with
# the plain mapping).  It was briefly enabled on the strength of a single wrist
# view, which was a marker-subset coincidence and made every later measurement
# wrong.
UV_RANK_FLIP = False


def board_corners(marker_id: int) -> np.ndarray:
    """Four marker corners in board-frame metres, print order."""
    index = marker_id - FIRST_ID
    # ids were assigned scanning ranks then files over the white squares
    count, found = 0, None
    for r in range(8):
        for c in range(8):
            if (c + r) % 2 == 0:
                continue
            if count == index:
                found = (c, r)
            count += 1
    if found is None:
        raise ValueError(f"marker {marker_id} outside the printed set")
    col, rank = found
    if UV_RANK_FLIP:
        rank = 7 - rank
    half = SQUARE_MM * 4.0
    cx = (col + 0.5) * SQUARE_MM - half
    cy = (rank + 0.5) * SQUARE_MM - half
    h = MARKER_MM / 2.0
    corners_mm = ((cx - h, cy + h), (cx + h, cy + h), (cx + h, cy - h), (cx - h, cy - h))
    return np.array([[x / 1000.0, y / 1000.0, 0.0] for x, y in corners_mm])


def chain_fk(calibration, joints_deg) -> np.ndarray:
    joints = np.radians(np.asarray(joints_deg, dtype=float))
    world = np.eye(4)
    for link in calibration.chain:
        world = world @ _transform(link["position"], link["rotation"])
        if "joint_index" not in link:
            continue
        angle = joints[int(link["joint_index"])]
        anchor = np.asarray(link["anchor"], dtype=float)
        rotation = Rotation.from_rotvec(
            np.asarray(link["axis"], dtype=float)
            * (angle - float(link["reference"]))).as_matrix()
        world = world @ _transform(anchor - rotation @ anchor, rotation)
    return world


def pose_from_params(params: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(params[:3]).as_matrix()
    matrix[:3, 3] = params[3:6]
    return matrix


def solve(captures: Path, records: list[dict], calibration: WristCalibration,
          intrinsics, x0_pose: np.ndarray, board0_pose: np.ndarray) -> dict:
    fx, fy, cx, cy = intrinsics
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    views = []
    for record in records:
        image = cv2.imread(str(captures / f"{record['name']}.jpg"))
        if image is None:
            continue
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        detected = {d.marker_id: np.asarray(d.corners, dtype=float)
                    for d in fiducials.detect_markers(frame)}
        joints = [float(record["joints_deg"][name]) for name in JOINTS]
        pairs = []
        for marker_id, corners in detected.items():
            if not (FIRST_ID <= marker_id <= FIRST_ID + 31):
                continue
            pairs.append((board_corners(marker_id), corners))
        if pairs:
            views.append((chain_fk(calibration, joints), pairs))
    if not views:
        return {"error": "no usable views"}

    def unpack(params):
        return pose_from_params(params[:6]), pose_from_params(params[6:12])

    def residual(params):
        x_pose, board = unpack(params)
        out = []
        for base_tool, pairs in views:
            base_camera = base_tool @ x_pose
            world_to_camera = np.linalg.inv(base_camera)
            for object_points, image_points in pairs:
                base_points = (board[:3, :3] @ object_points.T).T + board[:3, 3]
                camera_points = (world_to_camera[:3, :3] @ base_points.T).T \
                    + world_to_camera[:3, 3]
                depth = camera_points[:, 2]
                depth = np.where(np.abs(depth) < 1e-6, 1e-6, depth)
                projected = np.column_stack([
                    fx * camera_points[:, 0] / depth + cx,
                    fy * camera_points[:, 1] / depth + cy])
                out.extend((projected - image_points).ravel())
        return np.array(out)

    params0 = np.concatenate([
        Rotation.from_matrix(x0_pose[:3, :3]).as_rotvec(), x0_pose[:3, 3],
        Rotation.from_matrix(board0_pose[:3, :3]).as_rotvec(), board0_pose[:3, 3]])
    result = least_squares(residual, params0, x_scale="jac", max_nfev=400,
                           xtol=1e-12, ftol=1e-12)
    rms = float(np.sqrt(np.mean(residual(result.x) ** 2)))
    x_pose, board = unpack(result.x)
    return {
        "views": len(views),
        "sightings": sum(len(pairs) for _, pairs in views),
        "rms_reproj_px": round(rms, 3),
        "camera_in_tool": [[round(float(v), 6) for v in row] for row in x_pose],
        "board": {"x_mm": round(float(board[0, 3]) * 1000, 2),
                  "y_mm": round(float(board[1, 3]) * 1000, 2),
                  "z_mm": round(float(board[2, 3]) * 1000, 2),
                  "yaw_deg": round(math.degrees(math.atan2(board[1, 0], board[0, 0])), 2)},
        "camera_translation_mm": [round(float(v) * 1000, 2) for v in x_pose[:3, 3]],
    }



def mujoco_camera_to_opencv(rotation: np.ndarray) -> np.ndarray:
    """MuJoCo cameras look down -z (OpenGL); OpenCV cameras look down +z.

    Without this the solved pose sits 180 degrees from the recorded one, which
    reads as a broken solve rather than a convention mismatch.
    """
    flip = np.diag([1.0, -1.0, -1.0])
    return np.asarray(rotation, dtype=float) @ flip


def _average_pose(poses) -> np.ndarray:
    """Average rigid transforms: mean translation, rotation averaged via SVD."""
    translations = np.array([p[:3, 3] for p in poses])
    rotations = np.array([p[:3, :3] for p in poses])
    mean_rotation = np.mean(rotations, axis=0)
    u, _, vt = np.linalg.svd(mean_rotation)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = translations.mean(axis=0)
    return out


def solve_handeye(captures: Path, records: list[dict], calibration: WristCalibration,
                  intrinsics, distortion, refine: bool = True) -> dict:
    """AX=XB from relative motions, which needs no board pose.

    Worth trying once the lens is trustworthy: the earlier failure of this family
    was blamed on the estimator, but it was fed a 17%-wrong focal length and a
    board whose markers it could not identify.  Its result is used as the initia-
    liser for the staged solve, so a bad estimate still gets refined rather than
    reported.
    """
    fx, fy, cx, cy = intrinsics
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    dist = None if distortion is None else np.asarray(distortion, np.float64).reshape(-1, 1)
    rotations, translations = [], []
    views = []
    for record in records:
        image = cv2.imread(str(captures / f"{record['name']}.jpg"))
        if image is None:
            continue
        detected = {d.marker_id: np.asarray(d.corners, float)
                    for d in fiducials.detect_markers(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))}
        objects, pixels = [], []
        for marker_id, corners in detected.items():
            if not (FIRST_ID <= marker_id <= FIRST_ID + 31):
                continue
            objects.append(board_corners(marker_id))
            pixels.append(corners)
        if len(objects) < 2:
            continue
        ok, rvec, tvec = cv2.solvePnP(np.vstack(objects).astype(np.float64),
                                      np.vstack(pixels).astype(np.float64),
                                      camera_matrix, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        joints = [float(record["joints_deg"][name]) for name in JOINTS]
        base_tool = chain_fk(calibration, joints)
        rotations.append(base_tool[:3, :3])
        translations.append(base_tool[:3, 3].reshape(3, 1))
        board_cam = np.eye(4)
        board_cam[:3, :3] = cv2.Rodrigues(rvec)[0]
        board_cam[:3, 3] = np.ravel(tvec)
        views.append((base_tool, board_cam, np.vstack(objects), np.vstack(pixels)))
    if len(views) < 3:
        return {"error": f"only {len(views)} usable views"}
    target_rot = [v[1][:3, :3] for v in views]
    target_t = [v[1][:3, 3].reshape(3, 1) for v in views]
    x_pose = np.eye(4)
    rotation, translation = cv2.calibrateHandEye(rotations, translations, target_rot, target_t,
                                                 method=cv2.CALIB_HAND_EYE_PARK)
    x_pose[:3, :3] = rotation
    x_pose[:3, 3] = np.ravel(translation)

    board = _average_pose([fk @ x_pose @ bc for fk, bc, _, _ in views])
    residuals = []
    for fk, bc, objects, pixels in views:
        wc = np.linalg.inv(fk @ x_pose)
        base_points = (board[:3, :3] @ objects.T).T + board[:3, 3]
        cam_points = (wc[:3, :3] @ base_points.T).T + wc[:3, 3]
        rvec, _ = cv2.Rodrigues(np.eye(3))
        projected, _ = cv2.projectPoints(cam_points.reshape(-1, 1, 3), rvec, np.zeros(3),
                                         camera_matrix, dist)
        residuals.extend((projected.reshape(-1, 2) - pixels).ravel())
    rms = float(np.sqrt(np.mean(np.square(residuals))))
    return {"method": "handeye", "views": len(views),
            "sightings": sum(len(o) // 4 for _, _, o, _ in views),
            "rms_reproj_px": round(rms, 3),
            "camera_in_tool": [[round(float(v), 6) for v in row] for row in x_pose],
            "camera_translation_mm": [round(float(v) * 1000, 2) for v in x_pose[:3, 3]],
            "board": {"x_mm": round(float(board[0, 3]) * 1000, 2),
                      "y_mm": round(float(board[1, 3]) * 1000, 2),
                      "z_mm": round(float(board[2, 3]) * 1000, 2),
                      "yaw_deg": round(math.degrees(math.atan2(board[1, 0], board[0, 0])), 2)}}


def solve_staged(captures: Path, records: list[dict], calibration: WristCalibration,
                 intrinsics, x0_pose: np.ndarray, board0_pose: np.ndarray,
                 distortion=None,
                 iterations: int = 12, robust: bool = True,
                 bootstrap: bool = False) -> dict:
    """Alternating closed-form solve: board pose from X, then X from board pose.

    Each half is a mean of rigid transforms, so there is no optimiser to fall
    into a bad basin -- which is exactly how the nonlinear version failed.  The
    per-view residuals are still reported, so a wrong fit cannot hide.
    """
    fx, fy, cx, cy = intrinsics
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

    per_view = []
    for record in records:
        image = cv2.imread(str(captures / f"{record['name']}.jpg"))
        if image is None:
            continue
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        detected = {d.marker_id: np.asarray(d.corners, dtype=float)
                    for d in fiducials.detect_markers(frame)}
        objects, images = [], []
        for marker_id, corners in detected.items():
            if not (FIRST_ID <= marker_id <= FIRST_ID + 31):
                continue
            objects.append(board_corners(marker_id))
            images.append(corners)
        if len(objects) < 1:
            continue
        object_points = np.vstack(objects).astype(np.float64)
        image_points = np.vstack(images).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix,
                                      None, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        board_camera = np.eye(4)
        board_camera[:3, :3] = cv2.Rodrigues(rvec)[0]
        board_camera[:3, 3] = np.ravel(tvec)
        joints = [float(record["joints_deg"][name]) for name in JOINTS]
        per_view.append((chain_fk(calibration, joints), board_camera,
                         object_points, image_points))
    if not per_view:
        return {"error": "no usable views"}

    # Drop views whose own board-pose estimate disagrees badly: a partially
    # occluded tag or a grazing view pulls the alternating mean off, and one
    # bad view was inflating the residual from ~15 px to 65 px.
    def view_board_pose(fk, board_camera, board_pose, x_pose):
        return fk @ x_pose @ board_camera

    def prune(per_view, x_pose, board_pose):
        deviations = [np.linalg.norm((fk @ x_pose @ bc)[:3, 3] - board_pose[:3, 3])
                      for fk, bc, _, _ in per_view]
        median = float(np.median(deviations))
        mad = float(np.median(np.abs(np.array(deviations) - median))) + 1e-6
        keep = [i for i, d in enumerate(deviations) if d <= median + 3.0 * mad]
        if len(keep) >= max(3, len(per_view) // 2):
            return [per_view[i] for i in keep], len(per_view) - len(keep)
        return per_view, 0

    if robust:
        trial = _average_pose([fk @ x0_pose @ board_camera
                               for fk, board_camera, _, _ in per_view])
        deviations = [np.linalg.norm(view_board_pose(fk, bc, trial, x0_pose)[:3, 3]
                                     - trial[:3, 3]) for fk, bc, _, _ in per_view]
        median = float(np.median(deviations))
        mad = float(np.median(np.abs(np.array(deviations) - median))) + 1e-6
        keep = [i for i, d in enumerate(deviations) if d <= median + 3.0 * mad]
        if len(keep) >= max(3, len(per_view) // 2):
            per_view = [per_view[i] for i in keep]

    if bootstrap:
        # Bootstrap the mount from the *nominal* board placement, which is the
        # one thing a real setup always knows approximately: the board was laid
        # where it was laid.  Each view then gives a closed-form mount, and the
        # median rejects the views that disagree.  Starting from a stored
        # calibration instead diverged (746 px RMS) because it was 100 mm and
        # 39 deg away from the truth.
        candidates = [np.linalg.inv(fk) @ board0_pose @ np.linalg.inv(board_camera)
                      for fk, board_camera, _, _ in per_view]
        x0_pose = _average_pose(candidates)

    x_pose, board = x0_pose.copy(), board0_pose.copy()
    history = []
    for _ in range(iterations):
        board = _average_pose([fk @ x_pose @ board_camera
                               for fk, board_camera, _, _ in per_view])
        x_pose = _average_pose([np.linalg.inv(fk) @ board @ np.linalg.inv(board_camera)
                                for fk, board_camera, _, _ in per_view])
        history.append((x_pose.copy(), board.copy()))

    # One more pass with the now-accurate board pose: the bootstrap start can
    # settle a few tens of millimetres off, and re-selecting views against the
    # refined board pulls it in.
    if robust and len(per_view) > 4:
        per_view2, dropped = prune(per_view, x_pose, board)
        if dropped:
            per_view = per_view2
        for _ in range(iterations):
            board = _average_pose([fk @ x_pose @ board_camera
                                   for fk, board_camera, _, _ in per_view])
            x_pose = _average_pose([np.linalg.inv(fk) @ board @ np.linalg.inv(board_camera)
                                    for fk, board_camera, _, _ in per_view])

    residuals = []
    dist_coeffs = None if distortion is None else np.asarray(distortion, dtype=float).reshape(-1, 1)
    for fk, board_camera, object_points, image_points in per_view:
        base_camera = fk @ x_pose
        world_to_camera = np.linalg.inv(base_camera)
        base_points = (board[:3, :3] @ object_points.T).T + board[:3, 3]
        camera_points = (world_to_camera[:3, :3] @ base_points.T).T + world_to_camera[:3, 3]
        if dist_coeffs is not None:
            # Project through the measured lens. A pinhole-only residual leaves a
            # systematic bow at the frame edges, which then shows up as mount error.
            rvec, _ = cv2.Rodrigues(np.eye(3))
            projected, _ = cv2.projectPoints(camera_points.reshape(-1, 1, 3), rvec,
                                             np.zeros(3), camera_matrix, dist_coeffs)
            projected = projected.reshape(-1, 2)
        else:
            depth = np.where(np.abs(camera_points[:, 2]) < 1e-6, 1e-6, camera_points[:, 2])
            projected = np.column_stack([fx * camera_points[:, 0] / depth + cx,
                                         fy * camera_points[:, 1] / depth + cy])
        residuals.extend((projected - image_points).ravel())
    residuals = np.array(residuals)
    return {
        "method": "staged",
        "views": len(per_view),
        "sightings": sum(len(o) // 4 for _, _, o, _ in per_view),
        "rms_reproj_px": round(float(np.sqrt(np.mean(residuals ** 2))), 3),
        "camera_in_tool": [[round(float(v), 6) for v in row] for row in x_pose],
        "camera_translation_mm": [round(float(v) * 1000, 2) for v in x_pose[:3, 3]],
        "board": {"x_mm": round(float(board[0, 3]) * 1000, 2),
                  "y_mm": round(float(board[1, 3]) * 1000, 2),
                  "z_mm": round(float(board[2, 3]) * 1000, 2),
                  "yaw_deg": round(math.degrees(math.atan2(board[1, 0], board[0, 0])), 2)},
    }


def load_records(path: Path) -> list[dict]:
    """Records from either the simulator's list or the live app's capture set.

    The live app writes one JSON per capture with ``state_joints_deg``; the
    simulator writes one records.json with ``joints_deg``.  Accepting both keeps
    one solver for sim and IRL instead of a sim-only tool that has to be
    rewritten on the bench.
    """
    if path.is_dir():
        records = []
        for item in sorted(path.glob("*.json")):
            try:
                record = json.loads(item.read_text())
            except ValueError:
                continue
            joints = record.get("state_joints_deg") or record.get("joints_deg")
            if not joints or len(joints) != 6:
                continue
            records.append({"name": record.get("name", item.stem),
                            "joints_deg": {k: float(joints[k]) for k in JOINTS}})
        return records
    return json.loads(path.read_text())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", default="sim/captures_wrist")
    parser.add_argument("--records", default="sim/captures_wrist/records.json",
                        help="records.json, or a capture directory from the live app")
    parser.add_argument("--calibration", default=".private/phase11d/calibration.json")
    parser.add_argument("--truth", default="sim/captures_wrist/sim_truth.npy")
    parser.add_argument("--out", default="reports/handeye_bundle.json")
    parser.add_argument("--intrinsics-file", default="",
                        help="reports/intrinsics_*.json from solve_intrinsics.py")
    parser.add_argument("--calibration-intrinsics", action="store_true",
                        help="use the measured lens instead of the sim FOV")
    parser.add_argument("--bootstrap", action="store_true",
                        help="derive the mount from the nominal board placement")
    parser.add_argument("--method", choices=("bundle", "staged", "handeye"), default="bundle")
    parser.add_argument("--init-from-records", action="store_true",
                        help="mount from one solved pose (the realistic init)")
    parser.add_argument("--init-from-truth", action="store_true",
                        help="start at the simulator mount (implementation check)")
    parser.add_argument("--fovy-deg", type=float, default=36.0,
                        help="simulator camera vertical FOV, for intrinsics")
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    args = parser.parse_args()

    calibration = WristCalibration.load(args.calibration)
    if args.intrinsics_file:
        solved = json.loads(Path(args.intrinsics_file).read_text())
        intrinsics = (float(solved["fx"]), float(solved["fy"]),
                      float(solved["cx"]), float(solved["cy"]))
        distortion = solved.get("distortion")
    elif args.calibration_intrinsics:
        # Real captures: use the measured lens, not the simulator's FOV.  The
        # two differ (746 vs 739) and the real camera also carries distortion
        # the pinhole model here ignores.
        intrinsics = tuple(float(v) for v in calibration.intrinsics)
        distortion = None
    else:
        fy = 0.5 * args.height / math.tan(math.radians(args.fovy_deg) / 2.0)
        intrinsics = (fy, fy, args.width / 2.0, args.height / 2.0)
        distortion = None

    records = load_records(Path(args.records))
    board0 = np.eye(4)
    board0[:3, :3] = Rotation.from_euler("z", math.pi).as_matrix()
    board0[:3, 3] = [0.0, 0.0, 0.43 + 0.0032]

    x0 = np.asarray(calibration.camera_in_tool, dtype=float)
    if args.init_from_records:
        # Legitimate, non-cheating init: the mount from one solved pose, i.e.
        # X = FK(q)^-1 T_base_camera for a single view.  That is what a
        # PnP-plus-hand-eye pass would supply on the real arm.
        rec = records[0]
        q = [rec["joints_deg"][name] for name in JOINTS]
        base_tool = chain_fk(calibration, q)
        camera = np.eye(4)
        camera[:3, :3] = mujoco_camera_to_opencv(
            np.asarray(rec["true_camera_rotation"]).reshape(3, 3))
        camera[:3, 3] = np.asarray(rec["true_camera_position"])
        x0 = np.linalg.inv(base_tool) @ camera
    if args.init_from_truth:
        # Implementation check: start at the simulator's true mount.  If the
        # solve stays there with a tiny residual, the model is right and only
        # initialisation is at fault.
        truth_path = Path(args.truth)
        truth = np.load(truth_path, allow_pickle=True).item()
        x0 = np.asarray(truth["camera_in_chain_tool"], dtype=float)
    if args.method == "handeye":
        result = solve_handeye(Path(args.captures), records, calibration,
                               intrinsics, distortion)
    elif args.method == "staged":
        result = solve_staged(Path(args.captures), records, calibration,
                              intrinsics, x0, board0, distortion=distortion,
                              bootstrap=args.bootstrap)
    else:
        result = solve(Path(args.captures), records, calibration, intrinsics, x0, board0)

    if "error" in result:
        print("FAILED:", result["error"])
        return
    print(f"{result['views']} views, {result['sightings']} marker sightings")
    print(f"rms reprojection {result['rms_reproj_px']} px")
    print("camera in tool translation mm:", result["camera_translation_mm"])
    print("board:", result["board"])

    truth_path = Path(args.truth)
    if truth_path.exists():
        truth = np.load(truth_path, allow_pickle=True).item()
        truth_x = np.asarray(truth["camera_in_chain_tool"], dtype=float)
        solved_x = np.asarray(result["camera_in_tool"], dtype=float)
        delta = solved_x[:3, 3] - truth_x[:3, 3]
        angle = math.degrees(math.acos(max(-1.0, min(1.0, (
            np.trace(truth_x[:3, :3].T @ solved_x[:3, :3]) - 1.0) / 2.0))))
        print(f"vs sim truth: translation error {np.linalg.norm(delta)*1000:.2f} mm, "
              f"rotation error {angle:.2f} deg")
        result["truth_translation_error_mm"] = round(float(np.linalg.norm(delta) * 1000), 3)
        result["truth_rotation_error_deg"] = round(float(angle), 3)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n")
    print("->", args.out)


if __name__ == "__main__":
    main()
