#!/usr/bin/env python3
"""P1 perception: register the wrist view on the ChArUco board and locate a piece.

Everything is decided from printed geometry, never from a simulator state:

* the board pose comes from the ChArUco face itself (solvePnP over the detected
  markers), so it is immune to the arm's deadband and gravity sag;
* a square is called *empty* only when its rectified patch matches the printed
  face (normalised cross-correlation), so exposure drift cannot fake a piece;
* a piece is called *found* only when a square fails that test by a wide margin
  and the deviating pixels form one compact component.

The tool prints numbers only.  It never asserts that a frame "looks right".

    PYTHONPATH=. .venv/bin/python tools/p1_piece_detect.py --frames 5 --clip-s 4

Outputs (git-tracked):
    reports/p1_perception.json     receipt, every number quoted in the report
    reports/p1_annotated.png       portrait frame with the overlay
    reports/p1_clip.mp4            the same overlay on the live stream
    reports/p1_sheet.png           labelled contact sheet fallback
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from chess_sim import fiducials
from chess_sim.charuco_face import CharucoLayout

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
SQUARE_MM = 28.0
# Staunton proportions at the 28 mm square: a pawn is 27 mm, a king 46 mm.
PAWN_MM, KING_MM = 27.2, 46.4
NOMINAL_PIECE_MID_MM = 20.0      # used to undo the parallax of a standing piece


# --------------------------------------------------------------------- bridge
class Bridge:
    def __init__(self, url: str, token: str) -> None:
        self.url, self.token = url.rstrip("/"), token

    def _get(self, path: str) -> dict:
        req = urllib.request.Request(self.url + path,
                                     headers={"X-Bridge-Token": self.token})
        with urllib.request.urlopen(req, timeout=20) as response:
            return json.loads(response.read().decode())

    def state(self) -> dict:
        return self._get("/state")

    def capture(self) -> tuple[np.ndarray, dict]:
        payload = self._get("/capture")
        raw = np.frombuffer(base64.b64decode(payload["jpeg_b64"]), np.uint8)
        image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("bridge returned an undecodable frame")
        return image, payload


# ------------------------------------------------------------------ geometry
class Board:
    """Camera pose in board millimetres, from the printed markers alone."""

    def __init__(self, calibration: dict) -> None:
        self.fx, self.fy, self.cx, self.cy = (float(v) for v in calibration["intrinsics"])
        self.distortion = np.asarray(calibration["distortion"], float)
        self.K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], float)
        self.layout = CharucoLayout(square_mm=SQUARE_MM)
        self.chain = tuple(calibration["chain"])
        self.camera_in_tool = np.asarray(calibration["camera_in_tool"], float)

    def rectify(self, raw: np.ndarray) -> np.ndarray:
        if not any(self.distortion):
            return raw.copy()
        return cv2.undistort(raw, self.K, self.distortion)

    def solve(self, frame: np.ndarray):
        detections = fiducials.detect_markers(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if len(detections) < 2:
            return None, detections
        object_points, image_points = [], []
        for detection in detections:
            quad = fiducials_marker_corners(self.layout, detection.marker_id)
            object_points.append(quad)
            image_points.append(np.asarray(detection.corners, float))
        object_points = np.vstack(object_points)
        image_points = np.vstack(image_points)
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, self.K, None,
                                      flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return None, detections
        rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, self.K, None,
                                          rvec, tvec)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.K, None)
        residuals = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
        rotation, _ = cv2.Rodrigues(rvec)
        per_marker = {}
        for detection in detections:
            mask = np.array([d.marker_id for d in detections for _ in range(4)]) == detection.marker_id
            per_marker[str(detection.marker_id)] = round(float(residuals[mask].mean()), 3)
        return {
            "rvec": rvec.ravel(), "tvec": tvec.ravel(), "R": rotation,
            "rms_px": float(np.sqrt((residuals ** 2).mean())),
            "max_px": float(residuals.max()),
            "n_corners": int(residuals.size),
            "ids": [d.marker_id for d in detections],
            "per_marker_px": per_marker,
            "camera_in_board_mm": (-rotation.T @ tvec).ravel() * 1000.0,
            "range_mm": float(np.linalg.norm(tvec)) * 1000.0,
        }, detections

    def project(self, solution: dict, xy_mm, z_mm: float = 0.0) -> np.ndarray:
        point = np.array([[[xy_mm[0] / 1000.0, xy_mm[1] / 1000.0, z_mm / 1000.0]]], np.float64)
        uv, _ = cv2.projectPoints(point, solution["rvec"].reshape(3, 1),
                                  solution["tvec"].reshape(3, 1), self.K, None)
        return uv.reshape(2)

    def unproject(self, solution: dict, uv, z_mm: float) -> np.ndarray:
        """Pixel -> board millimetres on the horizontal plane at `z_mm`."""
        R = solution["R"]
        t = solution["tvec"]
        ray_cam = np.array([(uv[0] - self.cx) / self.fx,
                            (uv[1] - self.cy) / self.fy, 1.0])
        ray = R.T @ ray_cam
        origin = -R.T @ t
        if abs(ray[2]) < 1e-9:
            return np.array([np.nan, np.nan])
        s = (z_mm / 1000.0 - origin[2]) / ray[2]
        point = origin + s * ray
        return point[:2] * 1000.0

    def rectified(self, frame: np.ndarray, solution: dict, px_per_mm: float):
        """Warp the board plane to a top-down board image (origin at centre)."""
        n = int(round(256 * px_per_mm))
        src = np.array([[0, 0], [n - 1, 0], [n - 1, n - 1], [0, n - 1]], np.float32)
        dst = np.array([self.project(solution, (x / px_per_mm - 128.0, 128.0 - y / px_per_mm))
                        for x, y in src], np.float32)
        matrix = cv2.getPerspectiveTransform(src, dst)
        flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
        image = cv2.warpPerspective(frame, matrix, (n, n), flags=flags)
        valid = cv2.warpPerspective(np.full(frame.shape, 255, np.uint8), matrix, (n, n),
                                    flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
        return image, valid, n, px_per_mm

    # ---------------------------------------------------------------- FK / IK
    def _transform(self, position, rotation) -> np.ndarray:
        value = np.eye(4)
        value[:3, :3] = np.asarray(rotation).reshape(3, 3)
        value[:3, 3] = position
        return value

    def camera_from_joints(self, joints_deg) -> np.ndarray:
        world = np.eye(4)
        radians = np.deg2rad(joints_deg)
        for link in self.chain:
            world = world @ self._transform(link["position"], link["rotation"])
            if "joint_index" not in link:
                continue
            angle = radians[link["joint_index"]]
            rotation = Rotation.from_rotvec(
                np.asarray(link["axis"]) * (angle - link["reference"])).as_matrix()
            anchor = np.asarray(link["anchor"])
            world = world @ self._transform(anchor - rotation @ anchor, rotation)
        return world @ self.camera_in_tool

    def solve_joints(self, image_points: np.ndarray, object_points_mm: np.ndarray,
                     board_pose: dict) -> dict:
        """Best 5-DOF joint vector that reproduces the detected marker corners."""
        yaw = math.radians(board_pose["yaw_deg"])
        base_board = np.array([
            [math.cos(yaw), -math.sin(yaw), 0, board_pose["x_mm"] / 1000],
            [math.sin(yaw), math.cos(yaw), 0, board_pose["y_mm"] / 1000],
            [0, 0, 1, board_pose["z_mm"] / 1000], [0, 0, 0, 1]])
        world = (base_board @ np.column_stack([object_points_mm / 1000.0,
                                               np.ones(len(object_points_mm))]).T).T[:, :3]
        links = [link for link in self.chain if "joint_index" in link]
        limits = np.rad2deg([[link["limits"][0], link["limits"][1]] for link in links])

        def project(joints):
            pose = self.camera_from_joints(joints)
            camera = (world - pose[:3, 3]) @ pose[:3, :3]
            depth = np.where(-camera[:, 2] > 1e-6, -camera[:, 2], 1e-6)
            return np.stack([self.cx + self.fx * camera[:, 0] / depth,
                             self.cy - self.fy * camera[:, 1] / depth], axis=1)

        def residual(joints):
            return (project(joints) - image_points).ravel()

        seeds = [np.zeros(len(links)),
                 np.array([8.7, -80.75, 79.43, 77.32, -0.26])[:len(links)]]
        rng = np.random.default_rng(7)
        for _ in range(40):
            seeds.append(rng.uniform(limits[:, 0] * 0.8, limits[:, 1] * 0.8))
        best = None
        for seed in seeds:
            try:
                fit = least_squares(residual, seed, bounds=(limits[:, 0], limits[:, 1]),
                                    xtol=1e-9, ftol=1e-9)
            except ValueError:
                continue
            error = float(np.sqrt((fit.fun ** 2).mean()))
            if best is None or error < best[0]:
                best = (error, fit.x)
        if best is None:
            return {"ok": False}
        return {"ok": True, "rms_px": round(best[0], 3),
                "joints_deg": {name: round(float(v), 2)
                               for name, v in zip(JOINTS, best[1])},
                "method": "FK fit to detected marker corners (image-derived, not telemetry)"}


def fiducials_marker_corners(layout, marker_id) -> np.ndarray:
    corners = np.asarray(layout.corners_mm(marker_id), float) / 1000.0
    return np.column_stack([corners, np.zeros(4)])


# -------------------------------------------------------------- square test
def square_table(frame, reference, rect, valid, n, px_per_mm, layout, board, solution):
    """Per-square evidence: does the printed face show through?"""
    half = int(round(14 * px_per_mm))
    trim = int(round(3 * px_per_mm))
    rows = []
    for rank in range(8):
        for col in range(8):
            centre_mm = ((col + 0.5) * SQUARE_MM - 112.0, (rank + 0.5) * SQUARE_MM - 112.0)
            u = int(round((centre_mm[0] + 128.0) * px_per_mm))
            v = int(round((128.0 - centre_mm[1]) * px_per_mm))
            if u - half < 0 or v - half < 0 or u + half >= n or v + half >= n:
                continue
            window = valid[v - half:v + half, u - half:u + half]
            if (window == 0).mean() > 0.02:
                continue
            inner = slice(trim, -trim)
            patch = rect[v - half:v + half, u - half:u + half][inner, inner]
            expected = reference[v - half:v + half, u - half:u + half][inner, inner]
            a = cv2.resize(patch, (64, 64), interpolation=cv2.INTER_AREA).astype(np.float32)
            b = cv2.resize(expected, (64, 64), interpolation=cv2.INTER_AREA).astype(np.float32)
            a -= a.mean()
            b -= b.mean()
            denominator = np.linalg.norm(a) * np.linalg.norm(b)
            ncc = float((a * b).sum() / denominator) if denominator > 1e-6 else 0.0
            dark_square = (col + rank) % 2 == 0
            rows.append({
                "square": f"{chr(97 + col)}{rank + 1}",
                "col": col, "rank": rank,
                "parity": "dark" if dark_square else "light",
                "centre_mm": [round(centre_mm[0], 1), round(centre_mm[1], 1)],
                "ncc": round(ncc, 3),
                "mean": round(float(patch.mean()), 1),
                "std": round(float(patch.std()), 1),
            })
    return rows


def piece_from_squares(rows, reference, rect, valid, n, px_per_mm, board, solution):
    """A piece hides the print: big NCC loss on a light square, or a bright/dark
    lump on a dark one.  Requires one compact deviating component."""
    dark_stds = [r["std"] for r in rows if r["parity"] == "dark"]
    dark_means = [r["mean"] for r in rows if r["parity"] == "dark"]
    dark_floor = float(np.median(dark_stds)) if dark_stds else 0.0
    dark_mean = float(np.median(dark_means)) if dark_means else 0.0
    candidates = []
    for row in rows:
        if row["parity"] == "light":
            hit = row["ncc"] < 0.55
        else:
            hit = (row["std"] > max(45.0, dark_floor + 30.0)
                   or abs(row["mean"] - dark_mean) > 55.0)
        if hit:
            candidates.append(row)
    if not candidates:
        return None
    row = max(candidates, key=lambda r: (1.0 - r["ncc"]) if r["parity"] == "light" else r["std"])
    difference = cv2.absdiff(rect, reference)
    mask = ((difference > 60) & (valid > 0)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    u = int(round((row["centre_mm"][0] + 128.0) * px_per_mm))
    v = int(round((128.0 - row["centre_mm"][1]) * px_per_mm))
    label = labels[min(v, n - 1), min(u, n - 1)]
    if label == 0:
        return {"square": row["square"], "rejected": "no deviating component at the square centre"}
    area_mm2 = stats[label, cv2.CC_STAT_AREA] / px_per_mm ** 2
    if not 150.0 <= area_mm2 <= 2000.0:
        return {"square": row["square"], "rejected": f"component area {area_mm2:.0f} mm2"}
    cx_px, cy_px = centroids[label]
    rect_mm = np.array([cx_px / px_per_mm - 128.0, 128.0 - cy_px / px_per_mm])
    return {
        "square": row["square"],
        "component_area_mm2": round(float(area_mm2), 0),
        "centroid_on_plane_mm": [round(float(rect_mm[0]), 1), round(float(rect_mm[1]), 1)],
        "note": "plane projection of a standing piece; the base is displaced from the centroid",
    }



def board_view(rect, valid, rows, reference, px_per_mm):
    """Whole-board view at the rectified scale: what the camera covers, and where."""
    n = rect.shape[0]
    canvas = np.zeros((n, n, 3), np.uint8)
    covered = (valid > 0).astype(np.uint8)
    tint = cv2.cvtColor(cv2.GaussianBlur(rect, (5, 5), 0), cv2.COLOR_GRAY2BGR)
    canvas[covered > 0] = tint[covered > 0]
    ghost = (cv2.cvtColor(reference, cv2.COLOR_GRAY2BGR) * 0.28).astype(np.uint8)
    canvas[covered == 0] = ghost[covered == 0]
    step = int(round(SQUARE_MM * px_per_mm))
    for i in range(9):
        v = i * step
        cv2.line(canvas, (v, 0), (v, n - 1), (70, 70, 70), 1)
        cv2.line(canvas, (0, v), (n - 1, v), (70, 70, 70), 1)
    for row in rows:
        u = int(round((row["centre_mm"][0] + 128.0) * px_per_mm))
        v = int(round((128.0 - row["centre_mm"][1]) * px_per_mm))
        colour = (0, 220, 255) if row["parity"] == "light" else (150, 150, 150)
        cv2.circle(canvas, (u, v), 5, colour, -1)
        cv2.putText(canvas, row["square"], (u + 7, v + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, colour, 1, cv2.LINE_AA)
    contours, _ = cv2.findContours(covered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        box = cv2.boundingRect(max(contours, key=cv2.contourArea))
        cv2.rectangle(canvas, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                      (0, 200, 0), 2)
    area = float(covered.sum()) / px_per_mm ** 2
    cv2.putText(canvas, f"board 224x224 mm | camera covers {area:.0f} mm2 "
                        f"({100.0 * area / 50176.0:.1f}%)",
                (12, n - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, "dim = outside the wrist view", (12, n - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1, cv2.LINE_AA)
    return canvas


# ------------------------------------------------------------------ overlay
def annotate(frame, board, solution, rows, piece, meta, scale=2):
    portrait = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    canvas = cv2.resize(portrait, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    height, width = frame.shape[:2]

    def to_portrait(uv):
        return np.array([(height - 1 - uv[1]) * scale, uv[0] * scale])

    for detection in fiducials.detect_markers(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)):
        quad = np.array([to_portrait(p) for p in detection.corners], np.int32)
        cv2.polylines(canvas, [quad], True, (0, 220, 120), 2)
        centre = quad.mean(axis=0).astype(int)
        cv2.putText(canvas, str(detection.marker_id), tuple(centre),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 220, 120), 2, cv2.LINE_AA)

    origin = to_portrait(board.project(solution, (0.0, 0.0))).astype(int)
    for vector, colour, label in (((60.0, 0.0), (60, 60, 240), "X"),
                                  ((0.0, 60.0), (240, 160, 60), "Y")):
        tip = to_portrait(board.project(solution, vector)).astype(int)
        cv2.arrowedLine(canvas, tuple(origin), tuple(tip), colour, 3, tipLength=0.18)
        cv2.putText(canvas, label, tuple(tip + 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, colour, 2, cv2.LINE_AA)

    for row in rows:
        tag = f"{row['square']} {row['ncc']:.2f}"
        uv = to_portrait(board.project(solution, row["centre_mm"])).astype(int)
        cv2.circle(canvas, tuple(uv), 4, (0, 220, 255), -1)
        cv2.putText(canvas, tag, tuple(uv + np.array([6, -6])), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 220, 255), 1, cv2.LINE_AA)

    height_px, width_px = canvas.shape[:2]
    if piece and "centroid_on_plane_mm" in piece:
        uv = to_portrait(board.project(solution, piece["centroid_on_plane_mm"])).astype(int)
        cv2.drawMarker(canvas, tuple(uv), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
        cv2.putText(canvas, f"{piece['square']} {piece['centroid_on_plane_mm']}",
                    tuple(uv + np.array([12, 10])), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(canvas, "NO PIECE IN VIEW", (12, height_px - 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)

    lines = [
        f"PnP {solution['rms_px']:.2f}px max {solution['max_px']:.2f} n={solution['n_corners']}",
        f"cam board {np.round(solution['camera_in_board_mm'], 1)} mm",
        f"joints {meta.get('joints_text', 'n/a')}",
        f"ts {meta.get('frame_ts', 0):.0f} mode {meta.get('mode', '?')}",
        f"frame {meta.get('index', 0)}",
    ]
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (10, 26 + index * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


def contact_sheet(frames, cols=4):
    thumbs = [cv2.resize(frame, (320, 427)) for frame in frames]
    rows = (len(thumbs) + cols - 1) // cols
    sheet = np.zeros((rows * 427, cols * 320, 3), np.uint8)
    for index, thumb in enumerate(thumbs):
        r, c = divmod(index, cols)
        sheet[r * 427:(r + 1) * 427, c * 320:(c + 1) * 320] = thumb
    return sheet


# -------------------------------------------------------------------- main
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--clip-s", type=float, default=4.0)
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--out", default="reports/p1_perception.json")
    parser.add_argument("--bridge", default="http://192.168.1.167:8100")
    args = parser.parse_args()

    token = (ROOT / "live/bridge_token.txt").read_text().strip()
    board_pose = json.loads((ROOT / "live/board.json").read_text())
    calibration = json.loads((ROOT / ".private/phase11d/calibration.json").read_text())
    bridge = Bridge(args.bridge, token)
    board = Board(calibration)

    reference = cv2.imread(str(ROOT / "chess_sim/assets/scene/generated/board_charuco.png"),
                           cv2.IMREAD_GRAYSCALE)
    if reference is None:
        raise SystemExit("board_charuco.png missing; run the asset generator first")

    state = bridge.state()
    frames, solutions, notices = [], [], []
    for index in range(args.frames):
        raw, payload = bridge.capture()
        solution, detections = board.solve(board.rectify(raw))
        if solution is None:
            notices.append(f"frame {index}: {len(detections)} markers, no pose")
            time.sleep(0.25)
            continue
        frames.append(raw)
        solutions.append(solution)
        if len(solutions) == 1:
            first = (raw, solution, detections, payload)
        time.sleep(0.25)

    if not solutions:
        print("no board pose from any frame"); return 2

    raw, solution, detections, payload = first
    frame = board.rectify(raw)
    px_per_mm = 4.0
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rect, valid, n, pxm = board.rectified(grey, solution, px_per_mm)
    expected = cv2.resize(reference, (n, n), interpolation=cv2.INTER_AREA)
    rows = square_table(frame, expected, rect, valid, n, pxm, board.layout, board, solution)
    light = [r for r in rows if r["parity"] == "light"]
    dark = [r for r in rows if r["parity"] == "dark"]
    search = {
        "criterion_light": "printed-face NCC < 0.55 means the marker is hidden",
        "criterion_dark": "std > max(45, median+30) or |mean - median| > 55 grey",
        "light_squares": len(light),
        "dark_squares": len(dark),
        "light_ncc_min": round(min((r["ncc"] for r in light), default=0.0), 3),
        "light_ncc_median": round(float(np.median([r["ncc"] for r in light])) if light else 0.0, 3),
        "dark_std_max": round(max((r["std"] for r in dark), default=0.0), 1),
    }

    object_points = np.vstack([fiducials_marker_corners(board.layout, d.marker_id)
                               for d in detections])
    image_points = np.vstack([np.asarray(d.corners, float) for d in detections])
    joint_fit = board.solve_joints(image_points, object_points, board_pose)

    piece = piece_from_squares(rows, expected, rect, valid, n, pxm, board, solution)

    # five-frame repeat: how repeatable is the thing we would hand to the picker
    spreads = {}
    if len(solutions) >= 2:
        translations = np.array([s["tvec"] * 1000 for s in solutions])
        cameras = np.array([s["camera_in_board_mm"] for s in solutions])
        spreads = {
            "frames": len(solutions),
            "camera_in_board_mm": {
                "mean": np.round(cameras.mean(axis=0), 2).tolist(),
                "std": np.round(cameras.std(axis=0), 2).tolist(),
                "range": np.round(cameras.max(axis=0) - cameras.min(axis=0), 2).tolist(),
            },
            "board_plane_mm_at_principal_point": round(float(
                np.std([board.unproject(s, (board.cx, board.cy), 0.0)
                        for s in solutions], axis=0).max()), 2),
            "rms_px": round(float(np.mean([s["rms_px"] for s in solutions])), 3),
        }

    meta = {"frame_ts": payload.get("frame_ts", 0.0), "mode": payload.get("mode", "?"),
            "index": 0,
            "joints_text": (", ".join(f"{k[:3]}{v:+.1f}" for k, v in
                                      payload.get("joints", {}).items())
                            if payload.get("joints") else "none")}
    annotated = annotate(frame, board, solution, rows, piece, meta)
    (ROOT / "reports").mkdir(exist_ok=True)
    cv2.imwrite(str(ROOT / "reports/p1_annotated.png"), annotated)
    cv2.imwrite(str(ROOT / "reports/p1_board_view.png"),
                board_view(rect, valid, rows, expected, pxm))

    # clip: the same overlay on the live stream
    clip_path = ROOT / "reports/p1_clip.mp4"
    writer = cv2.VideoWriter(str(clip_path), cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (annotated.shape[1], annotated.shape[0]))
    clip_frames = []
    total = max(1, int(round(args.clip_s * args.fps)))
    for index in range(total):
        raw_i, payload_i = bridge.capture()
        frame_i = board.rectify(raw_i)
        solution_i, _ = board.solve(frame_i)
        if solution_i is None:
            continue
        meta_i = dict(meta, frame_ts=payload_i.get("frame_ts", 0.0), index=index,
                      mode=payload_i.get("mode", "?"))
        canvas = annotate(frame_i, board, solution_i, rows, piece, meta_i)
        writer.write(canvas)
        clip_frames.append(canvas)
        time.sleep(max(0.0, 1.0 / args.fps - 0.09))
    writer.release()
    if len(clip_frames) >= 4:
        cv2.imwrite(str(ROOT / "reports/p1_sheet.png"), contact_sheet(clip_frames))

    receipt = {
        "schema": "p1-perception/1",
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "bridge": {"url": args.bridge, "state_mode": state.get("mode"),
                   "state_armed": state.get("armed"),
                   "joints_source": state.get("joints_source"),
                   "joints_age_s": state.get("joints_age_s"),
                   "capture_joints": payload.get("joints")},
        "intrinsics_file": ".private/phase11d/calibration.json",
        "intrinsics": calibration["intrinsics"],
        "board_pose_applied": board_pose,
        "markers": {"ids": solution["ids"], "n_corners": solution["n_corners"],
                    "rms_px": round(solution["rms_px"], 3),
                    "max_px": round(solution["max_px"], 3),
                    "per_marker_px": solution["per_marker_px"]},
        "camera_in_board_mm": np.round(solution["camera_in_board_mm"], 1).tolist(),
        "range_mm": round(solution["range_mm"], 1),
        "squares_in_view": rows,
        "piece_search": search,
        "piece": piece,
        "joints_from_image": joint_fit,
        "repeatability": spreads,
        "notices": notices,
    }
    (ROOT / args.out).write_text(json.dumps(receipt, indent=2) + "\n")

    print(f"markers {solution['ids']} rms {solution['rms_px']:.2f}px max {solution['max_px']:.2f}")
    print(f"camera in board {np.round(solution['camera_in_board_mm'], 1)} mm, range {solution['range_mm']:.1f} mm")
    print(f"squares in view {len(rows)}: " + ", ".join(
        f"{r['square']}({r['parity'][0]},{r['ncc']:.2f})" for r in rows))
    print("piece:", piece)
    print("joints fit:", joint_fit)
    if spreads:
        print("spread:", json.dumps(spreads))
    print("wrote", args.out, "reports/p1_annotated.png", "reports/p1_clip.mp4")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
