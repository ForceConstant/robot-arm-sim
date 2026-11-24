#!/usr/bin/env python3
"""Phase 3 capture plan: the same board targets seen from five wrist rolls.

The point of the sweep is identifiability.  A camera-mount translation and a
board-origin translation are only separable when the tool orientation actually
varies; every capture ever taken before this ran at wrist_roll ~67 deg, which
made the two models collinear.  Here each roll is its own group and every group
covers both board halves.

Poses are solved on the BC-250 (they are robot-arm calculations): roll is
pinned, the other four joints are optimised so the wrist lens sits over the aim
point at the historical ~160 mm standoff, looking within a few degrees of
vertical, inside the bridge's own joint limits.

    .venv/bin/python tools/phase3_plan.py --out reports/phase3_plan_20261124.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.kinematics import frame_at, square_xy  # noqa: E402
from live.kinematics import ArmKinematics  # noqa: E402
from live.tip_frame import TIP_IN_CHAIN  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")


class Model:
    def __init__(self, cal, board, limits_deg):
        self.cal = cal
        self.board = board
        self.lo = np.radians([limits_deg[n][0] for n in NAMES[:5]])
        self.hi = np.radians([limits_deg[n][1] for n in NAMES[:5]])
        self.fx, self.fy = cal.intrinsics[0], cal.intrinsics[1]
        self.cx, self.cy = cal.intrinsics[2], cal.intrinsics[3]

    def camera(self, q_rad5, roll_rad):
        joints = np.array([q_rad5[0], q_rad5[1], q_rad5[2], q_rad5[3],
                           roll_rad, 0.0])
        c = self.cal.camera(tuple(float(v) for v in joints))
        return np.asarray(c.camera_position), np.asarray(c.camera_rotation).reshape(3, 3)

    def project(self, pos, rot, point):
        cam = rot.T @ (np.asarray(point) - pos)
        depth = -cam[2]
        if depth <= 1e-6:
            return None
        return (self.cx + self.fx * cam[0] / depth,
                self.cy - self.fy * cam[1] / depth)


def solve_target(model, point, standoff_m, roll_deg, seeds):
    """Lens at ``point`` + standoff; roll pinned; report where ``point`` lands."""
    roll = math.radians(roll_deg)
    station = np.asarray(point, float) + np.array([0.0, 0.0, float(standoff_m)])

    def residual(q):
        pos, rot = model.camera(q, roll)
        # The target must be in the frame (that is what makes the pose useful)
        # and the lens should stay near the historical standoff; both are
        # normalised so neither term swamps the other.
        uv = model.project(pos, rot, point)
        if uv is None:
            pix = np.array([4000.0, 4000.0])
        else:
            pix = np.array([uv[0] - model.cx, uv[1] - model.cy])
        return np.r_[(station - pos) * 1000.0 / 10.0, pix / 50.0]

    best = None
    for seed in seeds:
        try:
            result = least_squares(residual, np.asarray(seed[:4], float),
                                   bounds=(model.lo[:4], model.hi[:4]),
                                   xtol=1e-12, ftol=1e-12, gtol=1e-12,
                                   max_nfev=1200)
        except Exception:
            continue
        if not result.success and best is not None:
            continue
        cost = float(np.sum(result.fun ** 2))
        if best is None or cost < best[0]:
            best = (cost, result.x, result)
    if best is None:
        return None
    cost, q, result = best
    pos, rot = model.camera(q, roll)
    uv = model.project(pos, rot, point)
    view_axis = -rot[:, 2]
    incidence = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(view_axis[2]))))))
    return {"q5_rad": np.r_[q, roll].tolist(), "cost": cost,
            "pixel": None if uv is None else [float(uv[0]), float(uv[1])],
            "distance_mm": float(np.linalg.norm(np.asarray(point) - pos) * 1000.0),
            "station_error_mm": float(np.linalg.norm(station - pos) * 1000.0),
            "incidence_deg": incidence,
            "camera_mm": (pos * 1000.0).tolist(),
            "above_board_mm": float((pos[2] - model.board[2, 3]) * 1000.0)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_plan_20261124.json"))
    ap.add_argument("--rolls", default="0,40,-40,68,87")
    ap.add_argument("--targets", default="c2,f2,c7,f7,d4,e5,e3,d6")
    ap.add_argument("--standoff-mm", type=float, default=160.0)
    ap.add_argument("--bridge", default="http://127.0.0.1:8080")
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    board_json = json.loads((ROOT / "live/board.json").read_text())
    board = frame_at(board_json["x_mm"], board_json["y_mm"],
                     board_json["z_mm"], board_json["yaw_deg"])
    kb = json.loads((ROOT / "live/wrist_calibration.json").read_text())
    limits = kb["servo"]["limits"]
    model = Model(cal, board, limits)

    # seeds: the historical plan's poses, plus a few neutral arm postures
    seeds = []
    plan_path = ROOT / cfg["plan"]
    if plan_path.exists():
        plan = json.loads(plan_path.read_text())
        for pose in plan.get("poses", []):
            seeds.append([math.radians(float(v)) for v in pose["joints"][:5]])
    seeds += [
        [math.radians(-15), math.radians(-60), math.radians(75),
         math.radians(45), 0],
        [math.radians(0), math.radians(-50), math.radians(65),
         math.radians(40), 0],
        [math.radians(15), math.radians(-70), math.radians(85),
         math.radians(55), 0],
    ]

    rolls = [float(v) for v in args.rolls.split(",")]
    targets = [t.strip() for t in args.targets.split(",")]
    rows = []
    for roll in rolls:
        for name in targets:
            sx, sy = square_xy(name)
            point = (board[:3, :3] @ np.array([sx / 1000.0, sy / 1000.0, 0.0])
                     + board[:3, 3])
            sol = solve_target(model, point, float(args.standoff_mm) / 1000.0,
                               roll, seeds)
            if sol is None:
                print(f"roll {roll:6.1f} {name}: NO SOLUTION", flush=True)
                continue
            deg = [math.degrees(v) for v in sol["q5_rad"]]
            margins = {n: round(min(deg[i] - limits[n][0], limits[n][1] - deg[i]), 1)
                       for i, n in enumerate(NAMES[:5])}
            sol["margins_deg"] = margins
            # keep-in / clearance for the physical tip at this joint pose
            kin = ArmKinematics(cal, {}, board)
            tip = (kin.fk([float(v) for v in deg] + [0.0]) @ TIP_IN_CHAIN)[:3, 3]
            tip_board = (np.linalg.inv(board) @ np.r_[tip, 1.0])[:3] * 1000.0
            sol["tip_board_mm"] = [round(float(v), 1) for v in tip_board]
            keep = (abs(tip_board[0]) <= 112.0 and abs(tip_board[1]) <= 112.0
                    and 15.0 <= tip_board[2] <= 320.0)
            sol["keep_in"] = bool(keep)
            ok = (sol["pixel"] is not None
                  and math.hypot(sol["pixel"][0] - model.cx,
                                 sol["pixel"][1] - model.cy) < 200
                  and sol["incidence_deg"] <= 38.0
                  and sol["station_error_mm"] <= 50.0
                  and sol["above_board_mm"] >= 110.0
                  and keep
                  and min(margins.values()) >= 1.5)
            sol["ok"] = bool(ok)
            sol.update({"roll": roll, "target": name,
                        "joints_deg": [round(v, 2) for v in deg]})
            rows.append(sol)
            print(f"roll {roll:6.1f} {name}: pixel "
                  f"{None if sol['pixel'] is None else [round(v,1) for v in sol['pixel']]} "
                  f"inc {sol['incidence_deg']:.1f} st {sol['station_error_mm']:.0f} "
                  f"above {sol['above_board_mm']:.0f} tipz {tip_board[2]:.0f} ok {ok} "
                  f"joints {sol['joints_deg']} margin {min(margins.values()):.1f}",
                  flush=True)

    good = [r for r in rows if r["ok"]]
    out = {"schema": "phase3-plan/1",
           "created": "2026-11-24",
           "board": board_json, "rolls": rolls, "targets": targets,
           "standoff_mm": args.standoff_mm,
           "note": ("roll-pinned look-at solves; each roll is one group and the "
                    "targets cover both board halves (ranks 2/4/5/7)"),
           "poses": good, "rejected": [r for r in rows if not r["ok"]]}
    Path(args.out).write_text(json.dumps(out, indent=2) + "\n")
    print(f"\n{len(good)}/{len(rows)} poses accepted -> {args.out}")
    per_roll = {r: sum(1 for g in good if g["roll"] == r) for r in rolls}
    print("per-roll accepted:", per_roll)
    return 0 if good else 3


if __name__ == "__main__":
    raise SystemExit(main())
