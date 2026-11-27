#!/usr/bin/env python3
"""One session's board pose with the mount pinned — and the rotation since then.

Question this answers: **did the mat move, or did the camera mount move?**

The mount (``camera_in_tool``) is a hardware constant.  The mat is put down by
hand, so its pose is a per-session measurement (AGENTS.md, 2026-11-27).  This
tool captures a handful of wrist views, solves the *board* pose per view through
the pinned mount, and reports:

* the pooled board pose (median x/y/z, circular-mean yaw);
* the per-view scatter about that pool (mm / deg);
* the implied session rotation — pooled yaw minus a stated reference yaw —
  which is what a board re-registration should absorb.  Nothing here rewrites
  the mount, ``live/board.json`` or any calibration file.

    # default: print the resolved plan and the reference; no motion, no HTTP
    robot-arm-sim python tools/board_rotation_check.py

    # real capture (homes first, prints every leg, stops on an HTTP 409)
    robot-arm-sim python tools/board_rotation_check.py --execute

Motion rules, all enforced here: nothing moves without ``--execute``; the arm is
homed through ``/recover`` first and the tip verified before any leg; every leg
is printed before it is sent; a 409 or a refused move stops the run.  The
gripper is never actuated (its measured value is carried through).
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))
sys.path.insert(0, str(ROOT / "tools"))

from chess_sim import fiducials  # noqa: E402
from chess_sim.marker_scan import marker_corners_world  # noqa: E402
from chess_sim.projection import pnp_tool_frame  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from solve_wrist_live import flange_pose  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
APP = "http://127.0.0.1:8080"
CENTRE_TOLERANCE_MM = 20.0


class Refused(RuntimeError):
    """A move or capture the console refused (HTTP 409 or an error payload)."""


def _token(path: Path) -> str:
    try:
        return path.read_text().strip()
    except OSError:
        return ""


def app_post(app: str, path: str, body: dict, token: str, timeout: float = 120.0) -> dict:
    request = urllib.request.Request(
        app.rstrip("/") + path, data=json.dumps(body).encode(), method="POST",
        headers={"Content-Type": "application/json", "x-bridge-token": token})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            return json.loads(reply.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors="replace")[:300]
        raise Refused(f"{path} -> {exc.code}: {detail}") from exc


def app_get(app: str, path: str, timeout: float = 10.0) -> dict:
    with urllib.request.urlopen(app.rstrip("/") + path, timeout=timeout) as reply:
        return json.loads(reply.read().decode() or "{}")


def layout_for(config: dict):
    from chess_sim.charuco_face import CharucoLayout
    from chess_sim.marker_scan import MarkerLayout
    square_mm = float(config.get("square_m", 0.028)) * 1000.0
    if str(config.get("layout_kind", "mat")) == "charuco":
        return CharucoLayout(square_mm=square_mm)
    return MarkerLayout(offset_mm=float(config["layout_offset_mm"]))


def resolve_plan(plan_file: Path, names, count: int) -> list[tuple[str, list[float]]]:
    """Named poses, or ``auto``: the ``count`` most spread-apart distinct poses.

    A rotation check needs genuinely different viewpoints; ``live/poses.json``
    contains near-duplicate names (pos_1/pos_2, pos_3/pos_4), so an automatic
    plan dedupes and then runs a farthest-point spread over the joint vectors,
    the way ``block5q_wide_cal`` selects capture poses.
    """
    poses = json.loads(plan_file.read_text())
    if names != ["auto"]:
        out = []
        for name in names:
            record = poses.get(name)
            if record is None:
                raise SystemExit(f"{plan_file}: no pose {name!r}; available: "
                                 f"{', '.join(sorted(poses)[:12])} ...")
            out.append((name, [float(record[n]) for n in NAMES]))
        return out

    unique: dict[tuple, tuple[str, list[float]]] = {}
    for name in sorted(poses):
        joints = [float(poses[name][n]) for n in NAMES]
        if not all(math.isfinite(v) for v in joints):
            continue
        key = tuple(round(v, 1) for v in joints[:5])
        unique.setdefault(key, (name, joints))
    items = list(unique.values())
    if len(items) < count:
        raise SystemExit(f"{plan_file}: only {len(items)} distinct poses, need {count}")
    scale = np.array([40.0, 60.0, 25.0, 40.0, 45.0])
    points = [np.asarray(joints[:5]) / scale for _, joints in items]
    chosen = [0]
    while len(chosen) < count:
        best, best_d = None, -1.0
        for index, point in enumerate(points):
            if index in chosen:
                continue
            distance = min(float(np.linalg.norm(point - points[j])) for j in chosen)
            if distance > best_d:
                best, best_d = index, distance
        chosen.append(best)
    return [items[index] for index in chosen]


def board_pose_from_frame(frame_bgr, joints_deg, calibration, layout, mount) -> dict | None:
    """One view's board pose in the base frame, mount pinned."""
    detections = fiducials.detect_markers(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    obj, img, ids = [], [], []
    for detection in detections:
        try:
            corners = np.asarray(marker_corners_world(
                layout, detection.marker_id, origin_xy=(0, 0), yaw=0,
                marker_z=0), dtype=float)
        except (ValueError, KeyError):
            continue
        obj.extend(corners)
        img.extend(np.asarray(detection.corners, dtype=float))
        ids.append(detection.marker_id)
    if len(ids) < 2:
        return None
    solved = pnp_tool_frame(np.asarray(obj, float), np.asarray(img, float), calibration)
    if solved is None:
        return None
    cam_from_board, residual = solved
    base_from_flange = flange_pose(calibration, np.radians(np.asarray(joints_deg, float)))
    base_from_board = base_from_flange @ np.asarray(mount, float) @ cam_from_board
    return {"x_mm": float(base_from_board[0, 3]) * 1000.0,
            "y_mm": float(base_from_board[1, 3]) * 1000.0,
            "z_mm": float(base_from_board[2, 3]) * 1000.0,
            "yaw_deg": math.degrees(math.atan2(base_from_board[1, 0],
                                               base_from_board[0, 0])),
            "markers": ids, "residual_px": round(float(residual), 2)}


def pooled(rows: list[dict]) -> dict:
    value = {key: float(np.median([row[key] for row in rows]))
             for key in ("x_mm", "y_mm", "z_mm")}
    yaws = np.radians([row["yaw_deg"] for row in rows])
    value["yaw_deg"] = math.degrees(math.atan2(float(np.sin(yaws).mean()),
                                               float(np.cos(yaws).mean())))
    return value


def wrap(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--app", default=APP)
    ap.add_argument("--bridge", default="http://192.168.1.167:8100")
    ap.add_argument("--config", default="live/live_config.json")
    ap.add_argument("--calibration", default=".private/phase11d/calibration.json")
    ap.add_argument("--plan", default="live/poses.json")
    ap.add_argument("--poses", default="auto",
                    help="comma-separated pose names in --plan, or 'auto' for a "
                         "spread of distinct poses")
    ap.add_argument("--count", type=int, default=6,
                    help="views when --poses auto")
    ap.add_argument("--reference", default="live/board.json",
                    help="board pose the rotation is measured against (read only)")
    ap.add_argument("--settle-s", type=float, default=1.5)
    ap.add_argument("--out", default="reports/board_rotation_check.json")
    ap.add_argument("--execute", action="store_true",
                    help="home, move and capture (default: print the plan only)")
    args = ap.parse_args()

    config = json.loads((ROOT / args.config).read_text())
    calibration = WristCalibration.load(str(ROOT / args.calibration))
    layout = layout_for(config)
    mount = np.asarray(calibration.camera_in_tool, float)     # pinned, never solved
    reference = json.loads((ROOT / args.reference).read_text())
    plan = resolve_plan(ROOT / args.plan,
                        [n.strip() for n in args.poses.split(",") if n.strip()],
                        args.count)

    print(f"mount pinned: {calibration.calibration_id} "
          f"t {np.round(mount[:3, 3] * 1000, 2)} mm (not solved, not written)")
    print(f"reference: {args.reference} x {reference['x_mm']} y {reference['y_mm']} "
          f"z {reference['z_mm']} yaw {reference['yaw_deg']}")
    print(f"plan: {len(plan)} poses from {args.plan}")
    for name, joints in plan:
        print("  {:<10s} pan {:+.1f} lift {:+.1f} elbow {:+.1f} wflex {:+.1f} "
              "roll {:+.1f}".format(name, *joints[:5]))

    if not args.execute:
        print("\ndry run: nothing moved, nothing captured, no HTTP call")
        print("re-run with --execute to home, capture and solve")
        return 0

    token = _token(ROOT / config.get("token_file", "live/bridge_token.txt"))
    try:
        state = app_get(args.app, "/state").get("bridge") or {}
    except OSError as exc:
        print(f"console at {args.app} unreachable: {exc}")
        return 2
    if state.get("mode") != "REMOTE":
        print(f"bridge is {state.get('mode')!r}; needs REMOTE before any motion")
        return 2

    # Home first (AGENTS.md, 2026-11-25): the only pose the camera is known to
    # see the board from.  Homing never touches the gripper.
    print("\nleg 0 home: POST /recover")
    try:
        app_post(args.app, "/recover", {"reason": "board rotation check"}, token)
    except Refused as exc:
        print(f"  refused: {exc}")
        return 1
    time.sleep(1.5)
    tip = app_get(args.app, "/keep-in").get("tip_mm") or [float("nan")] * 3
    lateral = float((tip[0] ** 2 + tip[1] ** 2) ** 0.5)
    print(f"  tip ({tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f}) mm | "
          f"{lateral:.1f} mm from centre")
    if lateral > CENTRE_TOLERANCE_MM:
        print("  not at home; refusing to capture from an unknown pose")
        return 1

    from bridge_client import HttpBridge
    bridge = HttpBridge(args.bridge, token=token)

    rows = []
    measured_now = bridge.state().get("joints") or {}
    if "gripper" not in measured_now:
        print("bridge reports no measured gripper; refusing to guess a value")
        return 1
    gripper = float(measured_now["gripper"])   # carried through, never closed
    for index, (name, joints) in enumerate(plan, start=1):
        target = dict(zip(NAMES, joints))
        target["gripper"] = gripper      # carried, never commanded to close
        print(f"leg {index}/{len(plan)} {name}: move to {target}")
        try:
            move = app_post(args.app, "/move",
                            {"mode": "joint", "joints": target}, token)
        except Refused as exc:
            print(f"  stopped: {exc}")
            return 1
        if "error" in move or "http_error" in move:
            print(f"  stopped: move refused ({move.get('error') or move.get('detail')})")
            return 1
        time.sleep(args.settle_s)
        frame = bridge._request("/capture")
        raw = base64.b64decode(frame["jpeg_b64"])
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        measured = [float(frame["joints"][n]) for n in NAMES]
        pose = board_pose_from_frame(image, measured, calibration, layout, mount)
        if pose is None:
            print(f"  no board pose from {name} (too few markers or PnP failed)")
            continue
        pose["pose"] = name
        pose["move_error_deg"] = move.get("error_deg")
        rows.append(pose)
        print("  board x {:8.2f} y {:8.2f} z {:8.2f} yaw {:8.2f} | "
              "markers {} residual {:.2f} px".format(
                  pose["x_mm"], pose["y_mm"], pose["z_mm"], pose["yaw_deg"],
                  len(pose["markers"]), pose["residual_px"]))

    if len(rows) < 3:
        print(f"\nonly {len(rows)} usable views; not reporting a pose")
        return 1

    value = pooled(rows)
    scatter_mm = [math.sqrt((row["x_mm"] - value["x_mm"]) ** 2
                            + (row["y_mm"] - value["y_mm"]) ** 2
                            + (row["z_mm"] - value["z_mm"]) ** 2) for row in rows]
    scatter_deg = [abs(wrap(row["yaw_deg"] - value["yaw_deg"])) for row in rows]
    rotation = wrap(value["yaw_deg"] - float(reference["yaw_deg"]))
    shift = math.sqrt((value["x_mm"] - float(reference["x_mm"])) ** 2
                      + (value["y_mm"] - float(reference["y_mm"])) ** 2
                      + (value["z_mm"] - float(reference["z_mm"])) ** 2)

    print(f"\npooled board pose (n={len(rows)}): x {value['x_mm']:.2f} y "
          f"{value['y_mm']:.2f} z {value['z_mm']:.2f} yaw {value['yaw_deg']:.2f}")
    print(f"per-view scatter: median {np.median(scatter_mm):.2f} mm / "
          f"{np.median(scatter_deg):.2f} deg, max {max(scatter_mm):.2f} mm / "
          f"{max(scatter_deg):.2f} deg")
    print(f"implied session rotation vs {args.reference}: {rotation:+.2f} deg "
          f"(board moved {shift:.1f} mm from the stored pose)")
    print("mount untouched — a session rotation moves the board, never the mount")

    report = {"schema": "board-rotation-check/1",
              "calibration_id": calibration.calibration_id,
              "mount_translation_mm": (mount[:3, 3] * 1000).round(3).tolist(),
              "reference": {"path": args.reference,
                            "x_mm": reference["x_mm"], "y_mm": reference["y_mm"],
                            "z_mm": reference["z_mm"], "yaw_deg": reference["yaw_deg"]},
              "pooled": {key: round(float(v), 3) for key, v in value.items()},
              "session_rotation_deg": round(rotation, 3),
              "board_shift_mm": round(shift, 2),
              "scatter_mm": {"median": round(float(np.median(scatter_mm)), 2),
                             "max": round(float(max(scatter_mm)), 2)},
              "scatter_deg": {"median": round(float(np.median(scatter_deg)), 2),
                              "max": round(float(max(scatter_deg)), 2)},
              "views": [{k: (round(v, 3) if isinstance(v, float) else v)
                         for k, v in row.items()} for row in rows]}
    out_path = ROOT / args.out
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2) + "\n")
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
