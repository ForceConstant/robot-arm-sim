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
        self.thread = threading.Thread(target=self._loop, name="target-detect",
                                       daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ input
    def rectify(self, image_bgr: np.ndarray) -> np.ndarray:
        """Undistorted landscape frame — what the overlay geometry is measured in."""
        return self.detector.board.rectify(image_bgr)

    def submit(self, image_bgr: np.ndarray, capture_ts: float) -> None:
        """Hand the newest frame over; never blocks on detection."""
        if not self.enabled:
            return
        with self.lock:
            self._frame = image_bgr
            self._frame_ts = capture_ts
            self.submitted += 1

    # ----------------------------------------------------------------- worker
    def _loop(self) -> None:
        while not self._stop:
            started = time.perf_counter()
            with self.lock:
                frame = self._frame
                ts = self._frame_ts
                self._frame = None
            if frame is not None and self.enabled:
                try:
                    result = self.detector.detect(frame)
                    result["frame_ts"] = ts
                    with self.lock:
                        self.result = result
                        self.result_ts = time.time()
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
         stream_fps: float | None = None, stats: dict | None = None) -> np.ndarray:
    """Draw the overlay onto a portrait canvas.

    ``layout_hw`` is the landscape (height, width) of the frame the detection
    ran on; rotating 90 deg clockwise maps (u, v) -> (H - 1 - v, u).
    """
    height, width = layout_hw

    def to_portrait(uv):
        return (int(round(height - 1 - float(uv[1]))), int(round(float(uv[0]))))

    result = snap.get("result")
    age = snap.get("age_s")
    age = 99.0 if age is None else age
    fresh = age < max(1.5, 3.0)
    colour = GREEN if fresh else AMBER
    # Detection only runs while someone is watching, so the first frame after a
    # reconnect carries the previous session's result.  Corner geometry from an
    # older frame drawn on a new one is exactly the wrong thing to show: drop the
    # geometry and say the detection is idle instead.
    stale = result is not None and age > 2.0

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

    # ---- status block: the headline is the target or its absence
    lines = []
    if stale:
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
