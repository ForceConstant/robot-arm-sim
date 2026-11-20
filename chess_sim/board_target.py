"""Board registration and piece targeting, shared by the P1 tool and the dashboard.

The math is the P1 perception path, unchanged: the board pose comes from the
printed ChArUco face (solvePnP), a square is empty when its rectified patch still
matches the print, and a piece is one compact deviating component on a square
that fails that test.  Everything below works in camera pixels and board
millimetres; nothing here knows about the arm, the bridge or the dashboard.

The dashboard draws exactly the numbers the P1 receipt quotes, so a target on
screen and a target in reports/p1_perception.json come from one code path.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from chess_sim import fiducials
from chess_sim.charuco_face import CharucoLayout

SQUARE_MM = 28.0
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


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

    def grid(self, solution: dict, z_mm: float = 0.0) -> dict:
        """The board's own grid, projected: the 9x9 lattice and 64 named centres.

        The names come from the same mapping `nearest_square` inverts, so a label
        on screen and the square the target line prints cannot drift apart. Each
        centre carries the projected length of one square edge, which is what the
        overlay scales its text by.
        """
        def mm(col: float, rank: float) -> tuple[float, float]:
            return (col * SQUARE_MM - 4.0 * SQUARE_MM,
                    rank * SQUARE_MM - 4.0 * SQUARE_MM)

        points = [[[round(float(v), 2) for v in self.project(solution, mm(col, rank), z_mm)]
                   for col in range(9)] for rank in range(9)]
        centres = []
        for rank in range(8):
            for col in range(8):
                quad = np.array([points[rank][col], points[rank][col + 1],
                                 points[rank + 1][col + 1], points[rank + 1][col]], float)
                centre = quad.mean(axis=0)
                edge = 0.5 * (np.linalg.norm(quad[1] - quad[0])
                              + np.linalg.norm(quad[2] - quad[3]))
                centres.append({"name": f"{chr(97 + col)}{rank + 1}",
                                "uv": [round(float(centre[0]), 2),
                                       round(float(centre[1]), 2)],
                                "edge_px": round(float(edge), 2)})
        return {"points": points, "centres": centres}

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
    # Size gate relative to the square, not an absolute window (Brian 2026-11-20:
    # with a full set of pieces a blob must not be able to pass as a different
    # piece).  A piece seen from above cannot exceed ~1.5 square areas; the lower
    # bound keeps small pawn specks out.
    square_area_mm2 = SQUARE_MM * SQUARE_MM
    lo_area, hi_area = 0.20 * square_area_mm2, 1.5 * square_area_mm2
    if not lo_area <= area_mm2 <= hi_area:
        return {"square": row["square"],
                "rejected": f"component area {area_mm2:.0f} mm2 outside "
                            f"{lo_area:.0f}-{hi_area:.0f} mm2 (0.2-1.5 square)"}
    cx_px, cy_px = centroids[label]
    rect_mm = np.array([cx_px / px_per_mm - 128.0, 128.0 - cy_px / px_per_mm])
    # The candidate must be contained by the square it is named after, otherwise
    # a shadow spilling across a border gets labelled as a neighbour.
    centre = np.asarray(row["centre_mm"], dtype=float)
    off = float(np.linalg.norm(rect_mm - centre))
    if off > 0.5 * SQUARE_MM:
        return {"square": row["square"],
                "rejected": f"centroid {off:.0f} mm from the square centre "
                            f"(limit {0.5 * SQUARE_MM:.0f} mm)"}
    return {
        "square": row["square"],
        "component_area_mm2": round(float(area_mm2), 0),
        "centroid_on_plane_mm": [round(float(rect_mm[0]), 1), round(float(rect_mm[1]), 1)],
        "note": "plane projection of a standing piece; the base is displaced from the centroid",
    }


# ---------------------------------------------------------------- reporting
def nearest_square(x_mm: float, y_mm: float) -> str | None:
    """Board millimetres -> a1..h8, or None when the point is off the board."""
    col = int(math.floor((float(x_mm) + 112.0) / SQUARE_MM))
    rank = int(math.floor((float(y_mm) + 112.0) / SQUARE_MM))
    if not (0 <= col < 8 and 0 <= rank < 8):
        return None
    return f"{chr(97 + col)}{rank + 1}"


class TargetDetector:
    """One detection pass: markers, board pose, squares, piece centroid.

    Holds the calibration, the printed-face reference image and the rectified
    reference at the working scale, so a call does the work and nothing else.
    ``detect`` takes a *landscape* frame exactly as the bridge delivers it; the
    dashboard rotates the drawing, not the measurement.
    """

    def __init__(self, calibration_path, reference_path, px_per_mm: float = 4.0):
        self.calibration_path = Path(calibration_path)
        self.calibration = json.loads(self.calibration_path.read_text())
        self.board = Board(self.calibration)
        self.reference = cv2.imread(str(reference_path), cv2.IMREAD_GRAYSCALE)
        if self.reference is None:
            raise FileNotFoundError(f"printed-face reference missing: {reference_path}")
        self.px_per_mm = float(px_per_mm)
        self._expected = None

    def _reference_at(self, n: int) -> np.ndarray:
        if self._expected is None or self._expected.shape[0] != n:
            self._expected = cv2.resize(self.reference, (n, n),
                                        interpolation=cv2.INTER_AREA)
        return self._expected

    def detect(self, image_bgr: np.ndarray) -> dict:
        started = time.perf_counter()
        result = {
            "schema": "p2-target/1",
            "ts": time.time(),
            "status": "no target",
            "reason": "",
            "markers": [],
            "board": None,
            "piece": None,
            "squares_in_view": 0,
            "timing_ms": {},
        }
        rect_frame = self.board.rectify(image_bgr)
        t_solve = time.perf_counter()
        solution, detections = self.board.solve(rect_frame)
        result["timing_ms"]["solve"] = round((time.perf_counter() - t_solve) * 1000.0, 2)
        result["markers"] = [
            {"id": int(d.marker_id),
             "corners": [[round(float(x), 2), round(float(y), 2)] for x, y in d.corners]}
            for d in detections]
        if solution is None:
            result["reason"] = f"{len(detections)} markers, no board pose"
            result["timing_ms"]["total"] = round((time.perf_counter() - started) * 1000.0, 2)
            return result

        result["board"] = {
            "ids": solution["ids"],
            "n_corners": solution["n_corners"],
            "rms_px": round(solution["rms_px"], 3),
            "max_px": round(solution["max_px"], 3),
            "range_mm": round(solution["range_mm"], 1),
            "camera_in_board_mm": [round(float(v), 1)
                                   for v in solution["camera_in_board_mm"]],
            "n_in_view": len(detections),
        }

        # The grid goes out on every posed frame, target or not: the dashboard
        # has to show where a1..h8 are before it can show what is on one.
        result["grid"] = self.board.grid(solution)

        t_squares = time.perf_counter()
        grey = cv2.cvtColor(rect_frame, cv2.COLOR_BGR2GRAY)
        rect, valid, n, pxm = self.board.rectified(grey, solution, self.px_per_mm)
        expected = self._reference_at(n)
        rows = square_table(rect_frame, expected, rect, valid, n, pxm,
                            self.board.layout, self.board, solution)
        piece = piece_from_squares(rows, expected, rect, valid, n, pxm,
                                   self.board, solution)
        result["timing_ms"]["squares"] = round((time.perf_counter() - t_squares) * 1000.0, 2)
        result["squares_in_view"] = len(rows)
        result["light_ncc_min"] = round(min((r["ncc"] for r in rows
                                             if r["parity"] == "light"), default=0.0), 3)
        if piece is None:
            result["status"] = "no target"
            result["reason"] = "no square fails the printed-face test"
            result["timing_ms"]["total"] = round((time.perf_counter() - started) * 1000.0, 2)
            return result
        if "centroid_on_plane_mm" not in piece:
            result["status"] = "no target"
            result["piece"] = piece
            result["reason"] = str(piece.get("rejected", "piece candidate rejected"))
            result["timing_ms"]["total"] = round((time.perf_counter() - started) * 1000.0, 2)
            return result

        x_mm, y_mm = piece["centroid_on_plane_mm"]
        uv = self.board.project(solution, (x_mm, y_mm))
        result["status"] = "target"
        result["piece"] = {
            "square": piece["square"],
            "nearest_square": nearest_square(x_mm, y_mm),
            "x_mm": round(float(x_mm), 1),
            "y_mm": round(float(y_mm), 1),
            "area_mm2": piece.get("component_area_mm2"),
            # pixel of the plane centroid, in the *undistorted landscape*
            # frame the dashboard draws on
            "uv": [round(float(uv[0]), 2), round(float(uv[1]), 2)],
            "label": f"{nearest_square(x_mm, y_mm) or 'off-board'}  "
                     f"{x_mm:.1f},{y_mm:.1f} mm",
        }
        result["timing_ms"]["total"] = round((time.perf_counter() - started) * 1000.0, 2)
        return result
