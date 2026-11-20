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
from chess_sim.board_target import (JOINTS, SQUARE_MM, Board,
                                   fiducials_marker_corners,
                                   piece_from_squares, square_table)

ROOT = Path(__file__).resolve().parents[1]
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
