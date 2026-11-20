#!/usr/bin/env python3
"""P3 evidence: does a locked camera make consecutive frames agree?

The complaint was that consecutive frames disagree: same code, frame means of
83.6 / 40.9 / 39.6 and 8 / 4 / 2 detected markers.  The mechanism measured here
is the one that produces it: the bridge releases the camera when nobody is
watching, and every reopen restarts auto-exposure from scratch, so the first
frames after a reopen are exposed differently from the ones before it.

So each set is measured the way the dashboard actually meets the camera —
write the profile, let the supervisor close the device, then take the first
frames after it reopens.  Same machinery for both sets; only the values differ.

Per frame this prints the grey mean, the grey standard deviation, the marker
count and what the piece detector said.

    PYTHONPATH=. .venv/bin/python tools/p3_camera_stability.py --out reports/p3_camera_tab.json
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "live"))

from bridge_client import HttpBridge, HttpFrames  # noqa: E402
from chess_sim.board_target import TargetDetector  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]

# The device's own defaults, read from ``v4l2-ctl --list-ctrls-menus``.  Sent
# explicitly rather than "whatever is there now", so the baseline is the same
# baseline every run.
DEFAULTS = {
    "brightness": 128,
    "contrast": 32,
    "saturation": 32,
    "white_balance_automatic": 1,
    "gain": 64,
    "power_line_frequency": 2,
    "sharpness": 24,
    "backlight_compensation": 0,
    "auto_exposure": 3,
    "exposure_dynamic_framerate": 1,
}

# A deliberate hand-tuned set: more contrast, a fixed exposure long enough for
# the dark squares, manual white balance so the colour cast stops moving.
TUNED = {
    "brightness": 128,
    "contrast": 64,
    "saturation": 40,
    "gain": 110,
    "sharpness": 32,
    "power_line_frequency": 2,
    "backlight_compensation": 0,
    "auto_exposure": 1,
    "exposure_time_absolute": 1200,
    "white_balance_automatic": 0,
    "white_balance_temperature": 4000,
    "exposure_dynamic_framerate": 0,
}


def measure(frames: HttpFrames, detector: TargetDetector, count: int,
            gap: float = 0.05) -> list[dict]:
    rows = []
    for index in range(count):
        captured = frames.capture()
        image = cv2.cvtColor(captured.image, cv2.COLOR_RGB2BGR)
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        result = detector.detect(image)
        piece = result.get("piece") or {}
        rows.append({
            "frame": index + 1,
            "mean": round(float(grey.mean()), 2),
            "std": round(float(grey.std()), 2),
            "markers": len(result.get("markers") or []),
            "status": result.get("status"),
            "piece": piece.get("label") or result.get("reason") or "",
            "rms_px": (result.get("board") or {}).get("rms_px"),
        })
        time.sleep(gap)
    return rows


def summarise(rows: list[dict]) -> dict:
    means = [r["mean"] for r in rows]
    stds = [r["std"] for r in rows]
    markers = [r["markers"] for r in rows]
    return {
        "frames": len(rows),
        "mean_first": means[0],
        "mean_last": means[-1],
        "mean_spread": round(max(means) - min(means), 2),
        "mean_sd": round(statistics.pstdev(means), 2),
        "std_spread": round(max(stds) - min(stds), 2),
        "markers_min": min(markers),
        "markers_max": max(markers),
        "markers_stable": min(markers) == max(markers),
        "targets": sum(1 for r in rows if r["status"] == "target"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", default=None)
    parser.add_argument("--frames", type=int, default=6)
    parser.add_argument("--idle-s", type=float, default=10.0,
                        help="closed longer than the supervisor's 5 s idle window")
    parser.add_argument("--out", default="reports/p3_camera_tab.json")
    parser.add_argument("--persist-check", action="store_true")
    parser.add_argument("--leave", choices=("tuned", "defaults"), default="tuned",
                        help="which profile stays saved afterwards")
    args = parser.parse_args()

    base = args.bridge
    if not base:
        config = json.loads((ROOT / "live/live_config.json").read_text())
        base = config.get("bridge_url", "http://192.168.1.167:8100")
    token = str(ROOT / "live/bridge_token.txt")
    bridge = HttpBridge(base, token_file=token, timeout=30.0)
    frames = HttpFrames(base, token_file=token)
    detector = TargetDetector(str(ROOT / ".private/phase11d/calibration.json"),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))

    receipt: dict = {"schema": "p3-camera/1", "bridge": base,
                     "frames_per_set": args.frames,
                     "idle_s": args.idle_s, "sets": {}}

    order = [("defaults", DEFAULTS), ("tuned", TUNED)]
    if args.leave == "defaults":
        order.reverse()

    for name, settings in order:
        applied = bridge.set_camera_controls(settings, save=True)
        print(f"\n=== {name}: requested "
              + ", ".join(f"{k}={v}" for k, v in settings.items()))
        print("read back: " + ", ".join(f"{k}={v}"
                                       for k, v in applied["applied"].items()))
        if applied.get("mismatched"):
            print("did not take: " + json.dumps(applied["mismatched"]))
        print(f"closing the camera for {args.idle_s:.0f} s, then the first "
              f"{args.frames} frames after it reopens")
        time.sleep(args.idle_s)
        rows = measure(frames, detector, args.frames)
        summary = summarise(rows)
        print(f"{'frame':>5} {'mean':>7} {'std':>7} {'mark':>5}  {'status':<9} piece")
        for row in rows:
            print(f"{row['frame']:>5} {row['mean']:>7.2f} {row['std']:>7.2f} "
                  f"{row['markers']:>5}  {row['status']:<9} {row['piece']}")
        print(f"spread of the frame means: {summary['mean_spread']:.2f} levels "
              f"(sd {summary['mean_sd']:.2f}); markers "
              f"{summary['markers_min']}–{summary['markers_max']}; "
              f"targets {summary['targets']}/{len(rows)}")
        receipt["sets"][name] = {"requested": settings,
                                 "read_back": applied["applied"],
                                 "mismatched": applied["mismatched"],
                                 "saved_at": applied.get("saved_at"),
                                 "rows": rows, "summary": summary}

    if args.persist_check:
        print(f"\nreopening once more to prove the profile re-applies")
        time.sleep(args.idle_s)
        frames.capture()
        after = bridge.camera_controls()
        values = {c["name"]: c["value"] for c in after["controls"]}
        final = TUNED if args.leave == "tuned" else DEFAULTS
        print("read back after reopen: " + ", ".join(
            f"{k}={values.get(k)}" for k in final))
        receipt["reopen"] = {"values": {k: values.get(k) for k in final},
                             "saved_at": after.get("saved_at")}

    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
