#!/usr/bin/env python3
"""One vision-task receipt: settle exposure, measure, record.

Standing rule (Brian, 2026-11-21): exposure is re-settled before every vision
task against the frame about to be measured, judged by numbers only, and the
settled value is written alongside the result.  Nothing here is judged by eye.

    PYTHONPATH=. .venv/bin/python tools/p4_vision_receipt.py --target e5
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.board_target import TargetDetector  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from p4_auto_exposure import (api, adequate, grab, measure,  # noqa: E402
                              roi_edges, solve_any, square_quad, GRID,
                              base_metrics)
from p4_board_frame_check import stream_frames, JOINTS  # noqa: E402

LAYOUT = CharucoLayout(square_mm=28.0)


def settle(solver, target: str, grid) -> tuple[dict, list]:
    """Sweep, pick by the edge rule, apply for this session only."""
    rows = []
    for exposure, gain in grid:
        applied = api("/camera/controls",
                      {"controls": {"exposure_time_absolute": exposure,
                                    "gain": gain}, "save": False})["applied"]
        time.sleep(0.4)
        m = measure(solver, target, 3)
        rows.append({"requested_exposure": exposure, "gain": gain,
                     "applied_exposure": applied["exposure_time_absolute"],
                     "metrics": {k: v for k, v in m.items()
                                 if isinstance(v, (int, float))}})
    pool = [r for r in rows
            if adequate(r["metrics"]) and "edge_mean" in r["metrics"]]
    if target and pool:
        best = max(pool, key=lambda r: (r["metrics"]["edge_mean"],
                                        -r["metrics"]["clip"]))
    elif pool:
        best = min(pool, key=lambda r: (r["metrics"]["rms"],
                                        r["metrics"]["clip"]))
    else:
        raise SystemExit("no setting met the clipping budget")
    applied = api("/camera/controls",
                  {"controls": {"exposure_time_absolute": best["requested_exposure"],
                                "gain": best["gain"]}, "save": False})
    time.sleep(0.8)
    return applied, rows


def otsu_blob(gray, quad) -> dict:
    mask = np.zeros(gray.shape, np.uint8)
    cv2.fillPoly(mask, [quad.astype(np.int32)], 255)
    ys, xs = np.nonzero(mask)
    if not len(xs):
        return {}
    patch = gray[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    if patch.size < 64:
        return {}
    thr, binary = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    n, lab, stats, _ = cv2.connectedComponentsWithStats((binary > 0).astype(np.uint8), 8)
    comps = sorted([s for s in stats[1:]], key=lambda s: -s[4])[:3]
    if not comps:
        return {"otsu": float(thr)}
    big = comps[0]
    return {"otsu": float(thr), "blob_area_px": int(big[4]),
            "blob_w": int(big[2]), "blob_h": int(big[3]),
            "blob_fill": round(float(big[4]) / (big[2] * big[3] + 1e-9), 3),
            "blob_x": int(big[0]), "blob_y": int(big[1]),
            "blob_count": len(comps)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="e5")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--config", default="live/live_config.json")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    board = json.loads((ROOT / "live/board.json").read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board

    before = api("/camera/controls")
    before_vals = {c["name"]: c["value"] for c in before["controls"]}
    print(f"before: exposure {before_vals['exposure_time_absolute']} "
          f"gain {before_vals['gain']} contrast {before_vals['contrast']}")

    applied, sweep = settle(solver, args.target, GRID)
    settled = applied["applied"]
    print(f"settled: {settled} (save={applied.get('saved')})")
    print(f"read back mismatched {applied['mismatched']}")

    # Everything below runs on the settled frame.
    state = api("/state")
    joints = [float(state["bridge"]["joints"][n]) for n in JOINTS]
    yaw = math.radians(board["yaw_deg"])
    frame_in_arm = np.array([
        [math.cos(yaw), -math.sin(yaw), 0, board["x_mm"] / 1000],
        [math.sin(yaw), math.cos(yaw), 0, board["y_mm"] / 1000],
        [0, 0, 1, board["z_mm"] / 1000], [0, 0, 0, 1]])
    fk_cam = (np.linalg.inv(frame_in_arm) @ solver.camera_from_joints(joints))[:3, 3] * 1000.0

    frames = []
    for portrait in stream_frames("http://127.0.0.1:8080", args.frames, 12.0):
        frames.append(solver.rectify(cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)))
    stats = [base_metrics(f) for f in frames]
    agg = {k: float(np.mean([s[k] for s in stats])) for k in stats[0]}
    print(f"frame: mean {agg['mean']:.1f} std {agg['std']:.1f} "
          f"clip {agg['clip']:.2f}% dark {agg['dark']:.2f}%")

    ids, sol, roi, blob, marker_rows = [], None, {}, {}, []
    for frame in frames:
        s, dets = solve_any(solver, frame)
        ids = [d.marker_id for d in dets]
        if sol is None and s is not None:
            sol = s
    if sol is not None and args.target:
        quad, centre = square_quad(solver, sol, args.target)
        e, _, _ = roi_edges(cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY), quad)
        roi = {k: round(float(v), 2) for k, v in e.items()}
        roi["centre_uv"] = [round(float(centre[0]), 1), round(float(centre[1]), 1)]
        blob = otsu_blob(cv2.cvtColor(frames[0], cv2.COLOR_BGR2GRAY), quad)
        print(f"roi {roi}")
        print(f"blob {blob}")

    pose = {}
    if sol is not None:
        cam = np.asarray(sol.get("camera_in_board_mm"), float)
        delta = float(np.linalg.norm(cam - fk_cam))
        half = np.array([-fk_cam[0], -fk_cam[1], fk_cam[2]])
        pose = {"markers": len(sol.get("ids", ids)), "ids": sol.get("ids", ids),
                "single_marker": bool(sol.get("single_marker", False)),
                "rms_px": round(float(sol.get("rms_px", float("nan"))), 2),
                "camera_in_board_mm": [round(float(v), 1) for v in cam],
                "fk_camera_mm": [round(float(v), 1) for v in fk_cam],
                "disagreement_mm": round(delta, 1),
                "half_turn_prediction_mm": [round(float(v), 1) for v in half],
                "half_turn_distance_mm": round(float(np.linalg.norm(cam - half)), 1),
                "weak": len(sol.get("ids", ids)) < 2}
        print(f"pose {pose}")
    else:
        print("pose: no board pose")

    target_state = (api("/state").get("target") or {})
    detector_out = {"status": target_state.get("status"),
                    "reason": target_state.get("reason"),
                    "piece": target_state.get("piece"),
                    "board": target_state.get("board")}
    print(f"detector {detector_out['status']} — {detector_out['reason']}")

    receipt = {
        "schema": "vision-receipt/1",
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "target": args.target,
        "settled_controls": {"exposure_time_absolute": settled.get("exposure_time_absolute"),
                             "gain": settled.get("gain"),
                             "contrast": settled.get("contrast"),
                             "brightness": settled.get("brightness"),
                             "saved": False},
        "controls_before": {"exposure_time_absolute": before_vals["exposure_time_absolute"],
                            "gain": before_vals["gain"],
                            "contrast": before_vals["contrast"]},
        "settle_mismatched": applied.get("mismatched"),
        "sweep": sweep,
        "frame": {k: round(v, 2) for k, v in agg.items()},
        "markers": ids,
        "roi": roi,
        "otsu_blob": blob,
        "pose": pose,
        "detector": detector_out,
        "joints": [round(v, 2) for v in joints],
    }
    out = ROOT / (args.out or f"reports/p4_vision_receipt_{time.strftime('%Y%m%d_%H%M')}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
