#!/usr/bin/env python3
"""Auto-exposure for the wrist camera.

Two objectives:

* default (marker mode)  minimise marker PnP RMS subject to a clipping budget;
* ``--target e5``        maximise edge sharpness on that square's piece, using
                         the board pose from the printed markers to place the ROI.

All decisions are numeric: brightness/clipping statistics, marker PnP RMS, and
Sobel/Canny edge energy inside the target square.  Nothing is judged by eye.

    PYTHONPATH=. .venv/bin/python tools/p4_auto_exposure.py --check
    PYTHONPATH=. .venv/bin/python tools/p4_auto_exposure.py --target e5 --annotate reports/queen_edges.png
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.board_target import TargetDetector  # noqa: E402
from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from p4_board_frame_check import stream_frames  # noqa: E402

LAYOUT = CharucoLayout(square_mm=28.0)

URL = "http://127.0.0.1:8080"
GRID = [(e, g) for e in (140, 180, 220, 260, 300) for g in (64, 96)] + \
       [(220, 48), (220, 80)]
CLIP_BUDGET = 1.5      # % of pixels >= 250
DARK_BUDGET = 5.0      # % of pixels <= 5
MIN_MARKERS = 1
ROI_EXPAND = 1.02      # keep the ROI inside the square so neighbours do not dominate
CANNY = (40, 100)
TRIES = 4              # frames per setting; marker detection is intermittent


def api(path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(URL + path, data=data,
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=20).read())


def grab(solver, frames: int = 1):
    out = []
    for portrait in stream_frames(URL, frames, 10.0):
        out.append(solver.rectify(cv2.rotate(portrait, cv2.ROTATE_90_COUNTERCLOCKWISE)))
    return out


def solve_any(solver, rect):
    """Board pose from all markers, or a single marker when only one is visible."""
    sol, dets = solver.solve(rect)
    if sol is not None or not dets:
        return sol, dets
    det = dets[0]
    # board_target's own solutions are in metres (fiducials_marker_corners
    # divides by 1000), so match that here or grid()/project() disagree.
    quad = np.asarray(LAYOUT.corners_mm(det.marker_id), float).reshape(4, 2) / 1000.0
    obj = np.hstack([quad, np.zeros((4, 1))]).reshape(4, 1, 3)
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
            best = (rms, rvecs[k], tvecs[k], R, C)
    if best is None or best[0] > 1.0:
        return None, dets
    rms, rv, tv, R, C = best
    return {"rvec": rv.ravel(), "tvec": tv.ravel(), "R": R, "rms_px": rms,
            "single_marker": True, "ids": [d.marker_id for d in dets],
            "camera_in_board_mm": C * 1000.0}, dets


def base_metrics(rect) -> dict:
    gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    b, g, r = (rect[:, :, k].astype(np.float32) for k in range(3))
    return {"mean": float(gray.mean()), "median": float(np.median(gray)),
            "std": float(gray.std()),
            "clip": float(100*(gray >= 250).mean()),
            "dark": float(100*(gray <= 5).mean()),
            "clip_g": float(100*(g >= 250).mean())}


def square_quad(solver, solution, name: str, expand: float = ROI_EXPAND):
    """Projected square quad (expanded about its centre) and its centre uv.

    name="centre" uses a fixed central box instead of the board pose.
    """
    if name == "centre":
        h, w = 480, 640
        q = np.array([[w*0.22, h*0.22], [w*0.78, h*0.22],
                      [w*0.78, h*0.78], [w*0.22, h*0.78]], float)
        return q, q.mean(axis=0)
    grid = solver.grid(solution, 0.0)
    cols = "abcdefgh"
    col, rank = cols.index(name[0]), int(name[1:]) - 1
    pts = grid["points"]
    quad = np.array([pts[rank][col], pts[rank][col + 1],
                     pts[rank + 1][col + 1], pts[rank + 1][col]], float)
    centre = quad.mean(axis=0)
    return centre + (quad - centre) * expand, centre


def roi_edges(gray, quad):
    mask = np.zeros(gray.shape, np.uint8)
    cv2.fillPoly(mask, [quad.astype(np.int32)], 255)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)
    edges = cv2.Canny(gray, *CANNY)
    inside = mask > 0
    roi_mag = mag[inside]
    n_edge = int((edges[inside] > 0).sum())
    strong = roi_mag[roi_mag > 20]
    return {
        "edge_px": n_edge,
        "edge_density": float(100 * n_edge / max(1, inside.sum())),
        "edge_mean": float(strong.mean()) if strong.size else 0.0,
        "edge_p99": float(np.percentile(roi_mag, 99)) if roi_mag.size else 0.0,
        "roi_contrast": float(gray[inside].std()) if inside.any() else 0.0,
    }, edges, mask


def measure(solver, target: str, tries: int = TRIES) -> dict:
    """Brightness over every frame; edge metrics from the first frame with a pose."""
    rects = grab(solver, tries)
    if not rects:
        return {}
    base = [base_metrics(r) for r in rects]
    m = {k: float(np.mean([b[k] for b in base])) for k in base[0]}
    m["markers"] = 0.0
    m["rms"] = float("nan")
    m["has_pose"] = False
    edge_rows = []
    for rect in rects:
        dets = fiducials.detect_markers(cv2.cvtColor(rect, cv2.COLOR_BGR2RGB))
        m["markers"] = max(m["markers"], float(len(dets)))
        sol, _ = solve_any(solver, rect)
        if sol is not None:
            m["has_pose"] = True
            m["rms"] = float(sol["rms_px"])
        if target and (target == "centre" or sol is not None):
            quad, centre = square_quad(solver, sol, target)
            e, _, _ = roi_edges(cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY), quad)
            edge_rows.append((e, centre))
    if edge_rows:
        for k in edge_rows[0][0]:
            m[k] = float(np.mean([e[k] for e, _ in edge_rows]))
        m["target_uv"] = np.mean([c for _, c in edge_rows], axis=0)
    return m


def fmt(m: dict, target: str) -> str:
    if not m:
        return "no frames"
    s = (f"mean {m['mean']:6.1f} median {m['median']:6.1f} std {m['std']:5.1f} "
         f"| clip {m['clip']:5.2f}% dark {m['dark']:5.2f}% "
         f"| markers {m['markers']:.0f} rms {m['rms']:.2f} px")
    if target and "edge_mean" in m:
        s += (f" | {target} edge_px {m['edge_px']:5.0f} "
              f"dens {m['edge_density']:4.1f}% mean {m['edge_mean']:5.1f} "
              f"p99 {m['edge_p99']:5.1f} contrast {m['roi_contrast']:4.1f}")
    return s


def adequate(m: dict) -> bool:
    return bool(m) and (m["clip"] <= CLIP_BUDGET and m["dark"] <= DARK_BUDGET
                        and m["markers"] >= MIN_MARKERS)


def annotate(solver, target: str, path: Path, note: str) -> dict:
    rect, sol = None, None
    for candidate in grab(solver, TRIES):
        s, _ = solve_any(solver, candidate)
        if s is not None or target == "centre":
            rect, sol = candidate, s
            break
    if rect is None:
        return {}
    gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    quad, centre = square_quad(solver, sol, target)
    e, edges, _ = roi_edges(gray, quad)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)
    norm = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    heat = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    out = cv2.addWeighted(rect, 0.55, heat, 0.45, 0)
    out[edges > 0] = (0, 255, 0)                      # green Canny edges
    cv2.polylines(out, [quad.astype(np.int32)], True, (255, 0, 0), 2)
    cv2.circle(out, (int(centre[0]), int(centre[1])), 4, (0, 0, 255), -1)
    cv2.putText(out, f"{target} edge mean {e['edge_mean']:.0f} dens {e['edge_density']:.1f}%",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, note, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), out)
    return e


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--target", default="", help="square name to optimise edges on")
    ap.add_argument("--annotate", default="", help="save the highlighted capture here")
    ap.add_argument("--config", default="live/live_config.json")
    ap.add_argument("--grid", default="", help="e.g. 340x64,380x64")
    ap.add_argument("--no-save", action="store_true",
                    help="apply for this session only; leave the saved profile alone")
    args = ap.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    detector = TargetDetector(str(ROOT / config["calibration"]),
                              str(ROOT / "chess_sim/assets/scene/generated/"
                                        "board_charuco.png"))
    solver = detector.board
    live = {c["name"]: c["value"] for c in api("/camera/controls")["controls"]}
    print(f"current: exposure {live['exposure_time_absolute']} gain {live['gain']} "
          f"contrast {live['contrast']} brightness {live['brightness']}")
    base = measure(solver, args.target)
    print(f"  now      {fmt(base, args.target)}")
    if args.target and base.get("has_pose"):
        print(f"  {args.target} centre uv {np.round(base['target_uv'], 1)}")

    if args.check:
        ok = adequate(base)
        print("VERDICT: " + ("adequate" if ok else "NOT adequate"))
        return 0 if ok else 1

    grid = GRID
    if args.grid:
        grid = [tuple(int(v) for v in pair.split("x")) for pair in args.grid.split(",")]
    rows = []
    for exposure, gain in grid:
        applied = api("/camera/controls",
                      {"controls": {"exposure_time_absolute": exposure, "gain": gain},
                       "save": False})["applied"]
        time.sleep(0.4)
        m = measure(solver, args.target)
        rows.append((applied["exposure_time_absolute"], applied["gain"], m))
        print(f"  exp {applied['exposure_time_absolute']:>4} gain {applied['gain']:>3} "
              f"{fmt(m, args.target)}")

    if args.target:
        pool = [r for r in rows if adequate(r[2]) and "edge_mean" in r[2]]
        if pool:
            best = max(pool, key=lambda r: (r[2]["edge_mean"], -r[2]["clip"]))
            why = f"highest {args.target} edge energy within the clipping budget"
        else:
            print("no setting produced a board pose; leaving the camera alone")
            return 2
    else:
        valid = [r for r in rows if adequate(r[2])]
        if valid:
            best = min(valid, key=lambda r: (r[2]["rms"], r[2]["clip"]))
            why = "lowest marker PnP rms within the clipping budget"
        else:
            best = max(rows, key=lambda r: (r[2]["markers"], -abs(r[2]["mean"] - 110)))
            why = "no setting met the budget; best marker count / mean"
    e, g, m = best
    print(f"chosen: exposure {e} gain {g} ({why})")

    applied = api("/camera/controls",
                  {"controls": {"exposure_time_absolute": e, "gain": g},
                   "save": not args.no_save})
    time.sleep(1.0)
    final = measure(solver, args.target)
    print(f"read back: requested {applied['requested']} applied {applied['applied']} "
          f"mismatched {applied['mismatched']} saved_at {applied.get('saved_at')}")
    print(f"  final    {fmt(final, args.target)}")

    if args.annotate:
        note = f"exp {applied['applied']['exposure_time_absolute']} gain {applied['applied']['gain']}"
        e2 = annotate(solver, args.target or "e5", ROOT / args.annotate, note)
        print(f"annotated {args.annotate} {e2}")
    print("VERDICT: " + ("adequate" if adequate(final) else "still NOT adequate"))
    return 0 if adequate(final) else 1


if __name__ == "__main__":
    raise SystemExit(main())
