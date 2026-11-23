"""Target overlay for the dashboard camera view.

The dashboard shows the wrist camera; this module puts the *detected target* on
it — the markers the pose came from, the piece centroid as a crosshair, the
board millimetres and the nearest square, and a status line that says "no
target" outright when detection found nothing.  The joint vector and the age of
the servo read sit on the same screen, so a stale number is visible rather than
plausible.

Cost is kept off the stream thread: ``submit`` only hands over the frame the
stream already captured, a worker thread runs one detection at ``refresh_s``
cadence, and every frame in between redraws the last result.  The age of that
result is printed on screen.
"""
from __future__ import annotations

import threading
import time
from collections import deque

import cv2
import numpy as np

from chess_sim.board_target import TargetDetector

# Saturated on purpose: the stream is JPEG at quality 75 with chroma
# subsampling, and a 1 px pastel line loses most of its colour.  Measured with
# tools/p2_verify_overlay.py, which counts pixels at these exact values.
GREEN = (60, 255, 90)
RED = (60, 60, 255)
AMBER = (40, 200, 255)
WHITE = (250, 250, 250)
GREY = (175, 175, 175)
# The commanded aim point and the stored jaw-tip pixel: two markers Brian asked
# to see together on the grasp frames (2026-11-23). Magenta and cyan, so they
# stay apart from the green markers, the blue grid and the red detection target
# even after JPEG chroma subsampling.
MAGENTA = (255, 60, 235)
CYAN = (240, 220, 40)
# The board's own grid. Blue, because the markers are green and the target is
# red: three colours, three meanings, still separable after JPEG q75 chroma.
GRID = (255, 120, 60)
HALO = (18, 18, 18)
LABEL_EDGE_MIN_PX = 13.0   # below this a square name is a smudge, not a label
LABEL_SCALE_MIN = 0.26     # floor: legible over JPEG at 480 px wide
LABEL_SCALE_MAX = 0.95
LABEL_FIT = 0.90           # a name may use this much of its square's width


class TargetOverlay:
    """Latest target detection, computed off the stream thread."""

    def __init__(self, calibration: str, reference: str, *,
                 px_per_mm: float = 4.0, refresh_s: float = 0.5,
                 state_fn=None) -> None:
        self.detector = TargetDetector(calibration, reference, px_per_mm)
        self.refresh_s = max(0.05, float(refresh_s))
        self.state_fn = state_fn
        self.lock = threading.Lock()
        self._frame = None
        self._frame_ts = 0.0
        self._tool_pose = None
        self._tool_pose_solved = None
        self.result: dict | None = None
        self.result_ts = 0.0
        self.state: dict | None = None
        self.state_ts = 0.0
        self.runs = 0
        self.costs: deque[float] = deque(maxlen=40)
        self.submitted = 0
        self.errors = 0
        self.last_error = ""
        self._stop = False
        self.enabled = True
        self.aim = None          # (x_mm, y_mm, z_mm, label) set per grasp run
        self.thread = threading.Thread(target=self._loop, name="target-detect",
                                       daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ input
    def rectify(self, image_bgr: np.ndarray) -> np.ndarray:
        """Undistorted landscape frame — what the overlay geometry is measured in."""
        return self.detector.board.rectify(image_bgr)

    def submit(self, image_bgr: np.ndarray, capture_ts: float,
               tool_pose: np.ndarray | None = None) -> None:
        """Hand the newest frame over, with the arm's tool pose for that frame."""
        if not self.enabled:
            return
        with self.lock:
            self._frame = image_bgr
            self._frame_ts = capture_ts
            self._tool_pose = tool_pose
            self.submitted += 1

    def propagated(self, tool_now: np.ndarray | None) -> dict | None:
        """Project the board->camera pose forward from the last solve plus FK.

        The markers drop out as the jaws approach the board, and the grid and
        aim point vanish with them (2026-11-23). Both are rigid to the board and
        the camera is rigid to the tool, so the last solved pose can be carried
        forward: T_cam_board(now) = T_cam_tool @ T_tool_board(now), with
        T_cam_tool derived once from that solve and the tool pose at the time.
        """
        if tool_now is None:
            return None
        with self.lock:
            result = self.result
            tool_then = self._tool_pose_solved
        board = (result or {}).get("board") or {}
        if tool_then is None or "rvec" not in board:
            return None
        try:
            cam_board_then = np.eye(4)
            cam_board_then[:3, :3] = cv2.Rodrigues(
                np.asarray(board["rvec"], float).reshape(3, 1))[0]
            cam_board_then[:3, 3] = np.asarray(board["tvec"], float).ravel()
            cam_tool = cam_board_then @ np.linalg.inv(np.asarray(tool_then, float))
            cam_board_now = cam_tool @ np.asarray(tool_now, float)
            rvec_now, _ = cv2.Rodrigues(cam_board_now[:3, :3])
            return {"rvec": rvec_now.ravel(), "tvec": cam_board_now[:3, 3],
                    "propagated": True}
        except Exception:
            return None

    def set_aim(self, aim) -> None:
        """Board-mm point the arm is aiming at, drawn from the same pose."""
        with self.lock:
            self.aim = tuple(aim) if aim else None

    # ----------------------------------------------------------------- worker
    def _loop(self) -> None:
        while not self._stop:
            started = time.perf_counter()
            with self.lock:
                frame = self._frame
                ts = self._frame_ts
                tool = self._tool_pose
                self._frame = None
            if frame is not None and self.enabled:
                try:
                    result = self.detector.detect(frame, aim_xy=self.aim)
                    result["frame_ts"] = ts
                    with self.lock:
                        self.result = result
                        self.result_ts = time.time()
                        # pair the solved pose with the tool pose it was solved
                        # at; keep the last good pair when this frame has none
                        if result.get("board"):
                            self._tool_pose_solved = tool
                        self.runs += 1
                        self.costs.append(result["timing_ms"]["total"])
                except Exception as exc:          # keep the stream alive
                    with self.lock:
                        self.errors += 1
                        self.last_error = f"{type(exc).__name__}: {exc}"
            if self.state_fn is not None:
                try:
                    state = self.state_fn()
                    with self.lock:
                        self.state = state
                        self.state_ts = time.time()
                except Exception as exc:
                    with self.lock:
                        self.last_error = f"state: {type(exc).__name__}: {exc}"
            spent = time.perf_counter() - started
            time.sleep(max(0.0, self.refresh_s - spent))

    def stop(self) -> None:
        self._stop = True

    # -------------------------------------------------------------- reporting
    def snapshot(self) -> dict:
        with self.lock:
            return {
                "result": self.result,
                "age_s": (time.time() - self.result_ts) if self.result_ts else None,
                "state": self.state,
                "state_age_s": (time.time() - self.state_ts) if self.state_ts else None,
                "runs": self.runs,
                "mean_cost_ms": round(float(np.mean(self.costs)), 2) if self.costs else None,
                "submitted": self.submitted,
                "errors": self.errors,
                "last_error": self.last_error,
            }


# --------------------------------------------------------------------- drawing
def _panel(canvas: np.ndarray, x: int, y: int, w: int, h: int,
           alpha: float = 0.62) -> None:
    h = min(h, canvas.shape[0] - y)
    w = min(w, canvas.shape[1] - x)
    if h <= 0 or w <= 0:
        return
    roi = canvas[y:y + h, x:x + w]
    roi[:] = (roi * (1.0 - alpha)).astype(np.uint8)


def _draw_grid(canvas: np.ndarray, grid: dict, to_portrait, stats) -> None:
    """Board grid and square names, in the grid's own projected positions."""
    points = [[to_portrait(point) for point in row] for row in grid["points"]]
    for row in points:
        cv2.polylines(canvas, [np.array(row, np.int32)], False, GRID, 1, cv2.LINE_AA)
    for col in range(len(points[0])):
        cv2.polylines(canvas, [np.array([points[row][col] for row in range(len(points))],
                                        np.int32)], False, GRID, 1, cv2.LINE_AA)
    drawn, heights, widths, names, edges, over = 0, [], [], [], [], 0
    for centre in grid["centres"]:
        edge = float(centre["edge_px"])
        if edge < LABEL_EDGE_MIN_PX:
            continue
        u, v = to_portrait(centre["uv"])
        if not (0 <= u < canvas.shape[1] and 0 <= v < canvas.shape[0]):
            continue
        # Size the name to the square it sits in: the largest scale whose glyphs
        # still fit inside LABEL_FIT of the projected edge, bounded so it stays
        # legible when the squares shrink and does not swamp them when they grow.
        probe = float(cv2.getTextSize(centre["name"], cv2.FONT_HERSHEY_SIMPLEX, 1.0, 1)[0][0])
        fit = LABEL_FIT * edge / probe if probe > 0 else LABEL_SCALE_MIN
        scale = float(np.clip(fit, LABEL_SCALE_MIN, LABEL_SCALE_MAX))
        if fit < LABEL_SCALE_MIN:
            over += 1
        (tw, th), _ = cv2.getTextSize(centre["name"], cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        org = (int(u - tw / 2), int(v + th / 2))
        cv2.putText(canvas, centre["name"], org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    HALO, 2, cv2.LINE_AA)
        cv2.putText(canvas, centre["name"], org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    WHITE, 1, cv2.LINE_AA)
        drawn += 1
        heights.append(th)
        widths.append(tw)
        edges.append(edge)
        names.append(centre["name"])
    if stats is not None:
        stats["labels"] = drawn
        stats["label_names"] = names
        stats["label_px"] = int(np.median(heights)) if heights else 0
        stats["label_px_min"] = int(min(heights)) if heights else 0
        stats["label_px_max"] = int(max(heights)) if heights else 0
        stats["label_w_px"] = int(np.median(widths)) if widths else 0
        stats["square_px"] = int(np.median(edges)) if edges else 0
        stats["labels_over_square"] = over


def draw(canvas: np.ndarray, snap: dict, *, layout_hw: tuple[int, int],
         stream_fps: float | None = None, stats: dict | None = None,
         tip_center=None) -> np.ndarray:
    """Draw the overlay onto a portrait canvas.

    ``layout_hw`` is the landscape (height, width) of the frame the detection
    ran on; rotating 90 deg clockwise maps (u, v) -> (H - 1 - v, u).

    ``tip_center`` is the stored jaw-tip midpoint pixel in that same landscape
    frame. It is rigid to the camera, so it is drawn every frame; the commanded
    aim point (``result["aim"]``) is drawn beside it so the two can be compared
    on the very frames where the jaws close.
    """
    height, width = layout_hw

    def to_portrait(uv):
        return (int(round(height - 1 - float(uv[1]))), int(round(float(uv[0]))))

    result = snap.get("result")
    age = snap.get("age_s")
    age = 99.0 if age is None else age
    fresh = age < max(1.5, 3.0)
    propagated = bool(snap.get("propagated"))
    colour = AMBER if propagated else (GREEN if fresh else AMBER)
    # Detection only runs while someone is watching, so the first frame after a
    # reconnect carries the previous session's result.  Corner geometry from an
    # older frame drawn on a new one is exactly the wrong thing to show: drop the
    # geometry and say the detection is idle instead.
    stale = result is not None and age > 2.0 and not propagated

    if result and not stale:
        for marker in result.get("markers", []):
            quad = np.array([to_portrait(c) for c in marker["corners"]], np.int32)
            cv2.polylines(canvas, [quad], True, colour, 2, cv2.LINE_AA)
            centre = quad.mean(axis=0).astype(int)
            px, py = int(centre[0]), int(centre[1])
            cv2.putText(canvas, str(marker["id"]), (px - 8, py + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2, cv2.LINE_AA)

        if result.get("grid"):
            _draw_grid(canvas, result["grid"], to_portrait, stats)

        piece = result.get("piece") or {}
        if result.get("status") == "target" and piece.get("uv"):
            u, v = to_portrait(piece["uv"])
            arm = 26
            cv2.line(canvas, (u - arm, v), (u + arm, v), RED, 2, cv2.LINE_AA)
            cv2.line(canvas, (u, v - arm), (u, v + arm), RED, 2, cv2.LINE_AA)
            cv2.circle(canvas, (u, v), 14, RED, 1, cv2.LINE_AA)
            label = f"target {piece['nearest_square']}  {piece['x_mm']:.1f},{piece['y_mm']:.1f} mm"
            tx = min(max(u + 10, 4), max(4, canvas.shape[1] - 330))
            ty = min(max(v - 16, 16), canvas.shape[0] - 8)
            cv2.putText(canvas, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, RED, 2, cv2.LINE_AA)

        # ---- commanded aim point (the arm's own target), magenta
        aim = result.get("aim") or {}
        if aim.get("uv"):
            u, v = to_portrait(aim["uv"])
            for dx0, dy0, dx1, dy1 in ((-26, 0, -9, 0), (9, 0, 26, 0),
                                       (0, -26, 0, -9), (0, 9, 0, 26)):
                cv2.line(canvas, (u + dx0, v + dy0), (u + dx1, v + dy1),
                         MAGENTA, 2, cv2.LINE_AA)
            cv2.circle(canvas, (u, v), 20, MAGENTA, 1, cv2.LINE_AA)
            text = f"aim {aim.get('x_mm', 0):.1f},{aim.get('y_mm', 0):.1f}"
            tx = min(max(u + 12, 4), max(4, canvas.shape[1] - 210))
            ty = min(max(v + 34, 16), canvas.shape[0] - 8)
            cv2.putText(canvas, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, MAGENTA, 2, cv2.LINE_AA)

    # ---- jaw-tip centre: a fixed pixel, so it is drawn even with no detect
    tip_uv = None
    if tip_center is not None:
        tip_uv = to_portrait(tip_center)
        u, v = tip_uv
        cv2.line(canvas, (u - 18, v), (u + 18, v), CYAN, 2, cv2.LINE_AA)
        cv2.line(canvas, (u, v - 18), (u, v + 18), CYAN, 2, cv2.LINE_AA)
        cv2.putText(canvas, "tip", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, CYAN, 1, cv2.LINE_AA)

    # ---- status block: the headline is the target or its absence
    lines = []
    if propagated:
        head, head_colour = (f"aim carried by FK - last solve {age:.1f} s ago "
                             f"(markers lost)"), AMBER
    elif stale:
        head, head_colour = f"no target - detection idle {age:.0f} s", AMBER
    elif result is None:
        head, head_colour = "no target - starting", AMBER
    elif result["status"] == "target":
        piece = result["piece"]
        head = f"target {piece['nearest_square']}  {piece['x_mm']:.1f},{piece['y_mm']:.1f} mm"
        head_colour = GREEN
    else:
        head = f"no target - {result.get('reason') or 'detection failed'}"
        head_colour = RED

    if result:
        board = result.get("board")
        if board:
            lines.append(f"pnp {board['rms_px']:.2f} px  ids {len(board['ids'])}"
                         f"  d {board['range_mm']:.0f} mm")
        else:
            lines.append("pnp - no board pose")
        lines.append(f"detect {age:4.1f} s ago  {len(result.get('markers', []))} markers"
                     f"  {result.get('squares_in_view', 0)} squares")
        piece = result.get("piece") or {}
        if tip_uv is not None and not stale and piece.get("uv"):
            puv = to_portrait(piece["uv"])
            cv2.line(canvas, tip_uv, puv, WHITE, 1, cv2.LINE_AA)
            cv2.circle(canvas, puv, 5, WHITE, 1, cv2.LINE_AA)
            gap_px = float(np.hypot(puv[0] - tip_uv[0], puv[1] - tip_uv[1]))
            lines.append(f"tip->piece {gap_px:.0f} px"
                         + (f"  {piece['x_mm']:.1f},{piece['y_mm']:.1f} mm"
                            if piece.get("x_mm") is not None else ""))
    if snap.get("mean_cost_ms") is not None:
        lines.append(f"cost {snap['mean_cost_ms']:.1f} ms  runs {snap['runs']}"
                     + (f"  fps {stream_fps:.1f}" if stream_fps else ""))

    state = snap.get("state") or {}
    joints = state.get("joints") or {}
    if joints:
        vector = " ".join(f"{v:+.1f}" for v in joints.values())
        lines.append("joints " + vector)
        servo_age = state.get("joints_age_s")
        src = state.get("joints_source") or "?"
        lines.append(f"joints_age_s {'none' if servo_age is None else f'{servo_age:.2f}'}"
                     f"  src {src}  read {snap.get('state_age_s') or 0:.1f} s "
                     f"ago  {state.get('mode', '?')}")
    else:
        lines.append("joints - none  joints_age_s none")

    panel_h = 26 + 20 * (len(lines) + 1)
    _panel(canvas, 0, canvas.shape[0] - panel_h, canvas.shape[1], panel_h, 0.68)
    cv2.putText(canvas, head, (8, canvas.shape[0] - panel_h + 19),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, head_colour, 2, cv2.LINE_AA)
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (8, canvas.shape[0] - panel_h + 40 + 20 * index),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, WHITE, 1, cv2.LINE_AA)
    return canvas
