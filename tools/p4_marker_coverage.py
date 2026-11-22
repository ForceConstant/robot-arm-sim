#!/usr/bin/env python3
"""How many marker squares are actually complete in the current frame?

Projects every printed marker square through the board pose and classifies it as
fully inside the image, clipped by the frame edge, or out of view.  Then compares
that with what the detector decodes — so "only one marker decodes" can be blamed
on coverage or on image quality with a number instead of an opinion.

    PYTHONPATH=. .venv/bin/python tools/p4_marker_coverage.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402

LAYOUT = CharucoLayout(square_mm=28.0)
from chess_sim.board_target import TargetDetector  # noqa: E402
from p4_auto_exposure import api, adequate, measure, solve_any, GRID  # noqa: E402
from p4_board_frame_check import stream_frames  # noqa: E402

MARGIN = 2.0


def settle(solver):
    rows = []
    for exposure, gain in GRID:
        applied = api("/camera/controls",
                      {"controls": {"exposure_time_absolute": exposure,
                                    "gain": gain}, "save": False})["applied"]
        time.sleep(0.4)
        rows.append((exposure, gain, applied["exposure_time_absolute"],
                     measure(solver, "", 3)))
    pool = [r for r in rows if adequate(r[3])]
    best = max(pool, key=lambda r: r[3]["mean"]) if pool else rows[-1]
    api("/camera/controls",
        {"controls": {"exposure_time_absolute": best[0], "gain": best[1]},
         "save": False})
    time.sleep(0.8)
    return best


def main() -> int:
    config = json.loads((ROOT / "live/live_config.json").read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board

    exposure, gain, applied_exp, m = settle(solver)
    print(f"settled: exposure {applied_exp} gain {gain} | mean {m['mean']:.1f} "
          f"std {m['std']:.1f} clip {m['clip']:.2f}% dark {m['dark']:.2f}%")

    frame = None
    for portrait in stream_frames("http://127.0.0.1:8080", 1, 10.0):
        frame = solver.rectify(cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE))
    height, width = frame.shape[:2]
    sol, dets = solve_any(solver, frame)
    ids = sorted(d.marker_id for d in dets)
    if sol is None and dets:
        det = dets[0]
        quad_m = np.asarray(LAYOUT.corners_mm(det.marker_id), float).reshape(4, 2) / 1000.0
        obj = np.hstack([quad_m, np.zeros((4, 1))]).reshape(4, 1, 3)
        img = det.corners.astype(np.float64).reshape(4, 1, 2)
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, solver.K, None,
                                                 flags=cv2.SOLVEPNP_IPPE)
        best = None
        for k in range(n):
            R, _ = cv2.Rodrigues(rvecs[k])
            C = (-R.T @ tvecs[k]).ravel()
            if C[2] <= 0:
                continue
            proj, _ = cv2.projectPoints(obj, rvecs[k], tvecs[k], solver.K, None)
            rms = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).mean()))
            if best is None or rms < best[0]:
                best = (rms, rvecs[k], tvecs[k])
        if best is not None:
            sol = {"rvec": best[1].ravel(), "tvec": best[2].ravel(),
                   "rms_px": best[0], "ids": ids, "single_marker": True}
            print(f"single-marker pose: id {det.marker_id} rms {best[0]:.2f} px")
    print(f"frame {width}x{height}  decoded ids {ids}")
    if sol is None:
        print("no board pose")
        return 1
    print(f"pose from {len(sol.get('ids', ids))} marker(s), rms {sol['rms_px']:.2f} px")

    grid = solver.grid(sol, 0.0)
    pts = grid["points"]
    cols = "abcdefgh"
    inside, clipped, outside = [], [], []
    for rank in range(8):
        for col in range(8):
            if (col + rank) % 2 == 0:
                continue                      # markers sit on the (col+rank) odd squares
            quad = np.array([pts[rank][col], pts[rank][col + 1],
                             pts[rank + 1][col + 1], pts[rank + 1][col]], float)
            name = cols[col] + str(rank + 1)
            xs, ys = quad[:, 0], quad[:, 1]
            if (xs.max() < MARGIN or xs.min() > width - MARGIN
                    or ys.max() < MARGIN or ys.min() > height - MARGIN):
                outside.append(name)
                continue
            fully = (xs.min() >= MARGIN and xs.max() <= width - MARGIN
                     and ys.min() >= MARGIN and ys.max() <= height - MARGIN)
            (inside if fully else clipped).append(name)

    print(f"marker squares fully inside frame ({len(inside)}): {' '.join(inside)}")
    print(f"clipped by frame edge ({len(clipped)}): {' '.join(clipped)}")
    print(f"fully out of view ({len(outside)}): {' '.join(outside)}")
    decoded_inside = [n for n in inside]
    print(f"decoded {len(ids)} of {len(inside)} fully-inside squares -> "
          f"{100.0*len(ids)/max(1,len(inside)):.0f}% decode rate on complete markers")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
