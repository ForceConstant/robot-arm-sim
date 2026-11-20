#!/usr/bin/env python3
"""P2 evidence: record the dashboard stream and re-run the target detection.

Two things are measured here, both printed as numbers:

* the stream — how many frames the dashboard actually served in a window, so the
  overlay's cost can be judged against the rate before it existed;
* the detection — the same ``TargetDetector`` the dashboard runs, applied to
  every recorded frame, so the detection rate, the PnP RMS and the per-frame
  cost are measured on consecutive real frames rather than on a hand-picked one.

Artefacts are written for the report: PNG stills of the first, middle and last
served frame, an MP4 of the whole window, and the JSON receipt.

    PYTHONPATH=. .venv/bin/python tools/p2_capture_stream.py --seconds 8
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "live"))

from bridge_client import HttpFrames  # noqa: E402
from chess_sim.board_target import TargetDetector  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BOUNDARY = b"--frame"


def read_stream(url: str, seconds: float) -> list[tuple[float, np.ndarray]]:
    """Every JPEG the dashboard serves during ``seconds``, with arrival times."""
    frames: list[tuple[float, np.ndarray]] = []
    started = time.time()
    buffer = b""
    with urllib.request.urlopen(url, timeout=20) as response:
        while time.time() - started < seconds:
            chunk = response.read(8192)
            if not chunk:
                break
            buffer += chunk
            while True:
                head, sep, rest = buffer.partition(BOUNDARY)
                if not sep:
                    break
                header_end = rest.find(b"\r\n\r\n")
                if header_end < 0:
                    break
                body_start = header_end + 4
                next_at = rest.find(BOUNDARY, body_start)
                if next_at < 0:
                    break
                payload = rest[body_start:next_at].rstrip(b"\r\n")
                buffer = rest[next_at:]
                image = cv2.imdecode(np.frombuffer(payload, np.uint8),
                                     cv2.IMREAD_COLOR)
                if image is not None:
                    frames.append((time.time() - started, image))
    return frames


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8080")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--out", default="reports/p2_overlay.json")
    parser.add_argument("--clip", default="reports/p2_stream.mp4")
    parser.add_argument("--still-dir", default="reports")
    parser.add_argument("--detect-frames", type=int, default=20)
    parser.add_argument("--before-fps", type=float, default=None,
                        help="frame rate measured on this host before the overlay")
    args = parser.parse_args()

    state_before = json.loads(urllib.request.urlopen(
        args.url + "/state", timeout=10).read().decode())
    frames = read_stream(args.url + "/stream", args.seconds)
    state_after = json.loads(urllib.request.urlopen(
        args.url + "/state", timeout=10).read().decode())
    if len(frames) < 2:
        print(f"only {len(frames)} frames in {args.seconds}s; is a client attached?")
        return 2

    span = frames[-1][0] - frames[0][0]
    fps = (len(frames) - 1) / span if span > 0 else 0.0
    gaps = np.diff([t for t, _ in frames]) * 1000.0
    print(f"served {len(frames)} frames in {span:.2f}s -> {fps:.2f} fps "
          f"(gap mean {gaps.mean():.0f} ms, p95 {np.percentile(gaps, 95):.0f} ms)")

    # clip at the measured rate, so playback matches what the dashboard served
    height, width = frames[0][1].shape[:2]
    writer = cv2.VideoWriter(str(ROOT / args.clip), cv2.VideoWriter_fourcc(*"mp4v"),
                             max(1.0, min(fps, 10.0)), (width, height))
    for _, image in frames:
        writer.write(image)
    writer.release()

    picks = {"first": 0, "middle": len(frames) // 2, "last": len(frames) - 1}
    for name, index in picks.items():
        cv2.imwrite(str(ROOT / args.still_dir / f"p2_stream_{name}.png"),
                    frames[index][1])

    detector = TargetDetector(str(ROOT / ".private/phase11d/calibration.json"),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    # Measure on the *raw* bridge frames, not on the served ones: the served
    # image already carries this overlay, and detecting markers under drawn
    # polylines measures the drawing, not the perception.
    bridge_url = json.loads((ROOT / "live/live_config.json").read_text()).get(
        "bridge_url", "http://192.168.1.167:8100")
    bridge_frames = HttpFrames(bridge_url,
                               token_file=str(ROOT / "live/bridge_token.txt"))
    raw_frames = []
    for _ in range(args.detect_frames):
        captured = bridge_frames.capture()
        raw_frames.append((0.0, cv2.cvtColor(captured.image, cv2.COLOR_RGB2BGR)))
    runs = []
    for index, (arrival, image) in enumerate(raw_frames):
        result = detector.detect(image)
        runs.append({
            "index": index,
            "arrival_s": round(arrival, 3),
            "status": result["status"],
            "reason": result["reason"],
            "markers": [m["id"] for m in result["markers"]],
            "rms_px": result["board"]["rms_px"] if result["board"] else None,
            "squares_in_view": result["squares_in_view"],
            "piece": result["piece"],
            "cost_ms": result["timing_ms"]["total"],
        })
    detected = [r for r in runs if r["status"] == "target"]
    posed = [r for r in runs if r["rms_px"] is not None]
    rms = np.array([r["rms_px"] for r in posed]) if posed else np.array([0.0])
    costs = np.array([r["cost_ms"] for r in runs])
    print(f"detect: {len(posed)}/{len(runs)} frames got a board pose, "
          f"{len(detected)}/{len(runs)} reported a target")
    print(f"rms: mean {rms.mean():.2f} px, max {rms.max():.2f} px over {len(posed)} frames")
    print(f"cost: mean {costs.mean():.1f} ms, p95 {np.percentile(costs, 95):.1f} ms, "
          f"max {costs.max():.1f} ms per detection pass")

    receipt = {
        "schema": "p2-overlay/1",
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "url": args.url,
        "window_s": round(span, 2),
        "stream": {
            "frames": len(frames),
            "fps": round(float(fps), 2),
            "gap_ms_mean": round(float(gaps.mean()), 1),
            "gap_ms_p95": round(float(np.percentile(gaps, 95)), 1),
            "fps_before": args.before_fps,
            "reported_fps": state_after.get("stream_fps"),
            "frame_cost_ms": state_after.get("frame_cost_ms"),
            "shape": [int(width), int(height)],
        },
        "target_from_dashboard": state_after.get("target"),
        "detection": {
            "source": "raw /capture frames from the bridge (un-annotated)",
            "frames_tested": len(runs),
            "board_pose_frames": len(posed),
            "target_frames": len(detected),
            "rms_px_mean": round(float(rms.mean()), 3),
            "rms_px_max": round(float(rms.max()), 3),
            "cost_ms_mean": round(float(costs.mean()), 2),
            "cost_ms_p95": round(float(np.percentile(costs, 95)), 2),
            "cost_ms_max": round(float(costs.max()), 2),
            "runs": runs,
        },
        "overlay_fields": [
            "marker quads + ids (green, amber when the result is stale)",
            "piece centroid crosshair + 'target <square> <x>,<y> mm'",
            "status line: 'target ...' or 'no target - <reason>'",
            "pnp <rms> px, ids, board range mm",
            "detect age s, marker count, squares in view",
            "cost ms, detection runs, stream fps",
            "joint vector, joints_age_s, joints source, read age, mode",
        ],
        "artefacts": [args.clip, "reports/p2_stream_first.png",
                      "reports/p2_stream_middle.png", "reports/p2_stream_last.png"],
        "hook": "live/app.py :: Live.annotate() -> TargetOverlay.submit() + "
                "target_overlay.draw() (worker thread, every target_refresh_s)",
    }
    (ROOT / args.out).write_text(json.dumps(receipt, indent=2) + "\n")
    print("wrote", args.out, args.clip)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
