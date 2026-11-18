#!/usr/bin/env python3
"""Validate a scan plan end to end, in sim, without touching the arm.

Runs the **production** ``ScanController`` against a simulator-backed fake
bridge (the same fakes the unit tests use) at a nominal board placement and at
a set of displaced placements, and reports what the live scan would report:

* whether registration succeeds inside the estimator's 1.2 px RMS gate
  (``fiducials.MAX_RMS_PX`` is enforced inside ``estimate_board_pose``; a
  registration of ``None`` means it did not pass),
* the recovered offset error against the truth the scene was built with,
* how many views were used and which markers were seen.

    PYTHONPATH=. robot-arm-sim python tools/validate_scan_plan.py \
        --plan chess_sim/assets/marker_scan_plan_charuco.json \
        --layout charuco --calibration reports/block3_sim_calibration.json

Placements are ``dx_mm,dy_mm,dyaw_deg`` triples; the default set is nominal
plus ±8 mm on both axes and ±2.5° of yaw.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.charuco_face import CharucoLayout  # noqa: E402
from chess_sim.config import Camera  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.marker_scan import board_offset, load_plan  # noqa: E402
from chess_sim.product import current_layout, layout_env_config  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from scan_controller import Frame, ScanController, plan_in_degrees  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
DEFAULT_PLACEMENTS = "0,0,0;8,0,0;-8,0,0;0,8,0;0,-8,0;0,0,2.5;0,0,-2.5"


def parse_limits(spec: str | None, bridge: str | None) -> dict:
    """Joint limits in degrees from an inline/path JSON blob or the bridge.

    Only GET is used against the bridge; this never commands a joint.
    """
    limits: dict = {}
    if bridge:
        url = bridge.rstrip("/") + "/state"
        with urllib.request.urlopen(url, timeout=10) as response:
            state = json.loads(response.read())
        raw = state.get("limits") or {}
        if not raw:
            raise SystemExit(f"bridge at {bridge} returned no limits")
        limits = {name: [float(v[0]), float(v[1])] for name, v in raw.items()}
    if spec:
        text = Path(spec).read_text() if Path(spec).is_file() else spec
        raw = json.loads(text)
        if "limits" in raw and isinstance(raw["limits"], dict):
            raw = raw["limits"]
        limits.update({name: [float(v[0]), float(v[1])]
                       for name, v in raw.items()})
    return limits


def assert_plan_limits(plan, limits: dict) -> dict:
    """Hard gate: every plan and recovery pose must sit inside ``limits``.

    The controller refuses at the bench with a precheck like
    ``pose 'marker71' is outside wrist_roll limits (-87.2 vs -81.8..91.0)``;
    this makes that a sim-stage failure instead of a bench trip.
    """
    entries = [(pose.name, pose.joints) for pose in plan.poses]
    entries += [(entry["name"], tuple(float(v) for v in entry["joints"]))
                for entry in (plan.recovery or {}).values()]
    worst = float("inf")
    worst_where = None
    bad: list[dict] = []
    for name, joints in entries:
        for index, joint in enumerate(NAMES):
            lo, hi = limits.get(joint, (-180.0, 180.0))
            value = float(joints[index])
            margin = min(value - lo, hi - value)
            if margin < worst:
                worst, worst_where = margin, f"{name}/{joint} {value:.2f} in {lo:.2f}..{hi:.2f}"
            if not lo <= value <= hi:
                bad.append({"pose": name, "joint": joint, "value": round(value, 2),
                            "limit": [lo, hi]})
    if bad:
        raise SystemExit("plan exceeds the joint limits: " + json.dumps(bad[:6]))
    return {"poses": len(plan.poses), "recovery": len(plan.recovery or {}),
            "worst_margin_deg": round(worst, 2), "worst": worst_where}


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeBridge:
    """Moves the MuJoCo model toward the commanded joints, degree-for-degree."""

    def __init__(self, env, rate_deg_s: float = 240.0) -> None:
        self.env = env
        self.rate = rate_deg_s
        self.calls = 0

    def joints(self):
        return tuple(float(np.degrees(v))
                     for v in self.env.data.qpos[self.env._joint_qpos])

    def state(self) -> dict:
        return {"mode": "REMOTE", "armed": True,
                "joints": dict(zip(NAMES, self.joints()))}

    def command(self, joints, *, seq: int, ttl_ms: int, speed_scale: float) -> dict:
        self.calls += 1
        target = np.radians(np.asarray(joints, dtype=float))
        current = self.env.data.qpos[self.env._joint_qpos].copy()
        step = math.radians(self.rate) * 0.1
        self.env.data.qpos[self.env._joint_qpos] = current + np.clip(
            target - current, -step, step)
        mujoco.mj_forward(self.env.model, self.env.data)
        return {"accepted": True, "seq": seq}


class FakeFrames:
    """Renders the wrist camera and pairs the frame with the joints that made it."""

    def __init__(self, env, clock) -> None:
        self.env = env
        self.clock = clock
        self.captures = 0

    def joints(self):
        return tuple(float(np.degrees(v))
                     for v in self.env.data.qpos[self.env._joint_qpos])

    def capture(self) -> Frame:
        self.captures += 1
        return Frame(image=self.env.render(Camera.WRIST).copy(),
                     joints_deg=self.joints(), timestamp=self.clock())


def build_env(origin, yaw_rad, face: str):
    config = layout_env_config("printed", cameras=())
    board = config.board.model_copy(update={
        "face": face, "origin": (float(origin[0]), float(origin[1])),
        "yaw": float(yaw_rad)})
    env = ChessSimEnv(config.model_copy(update={"board": board}))
    env.reset("8/8/8/8/8/8/8/8")
    return env


def parse_placements(spec: str):
    out = []
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        dx, dy, dyaw = (float(v) for v in chunk.split(","))
        out.append((dx, dy, dyaw))
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", default="chess_sim/assets/marker_scan_plan_charuco.json")
    parser.add_argument("--layout", choices=("mat", "charuco"), default="charuco")
    parser.add_argument("--calibration", default="reports/block3_sim_calibration.json")
    parser.add_argument("--placements", default=DEFAULT_PLACEMENTS)
    parser.add_argument("--out", default="reports/scan_plan_validation.json")
    parser.add_argument("--tolerance-mm", type=float, default=2.0,
                        help="offset error budget for a placement to pass")
    parser.add_argument("--tolerance-yaw-deg", type=float, default=1.0)
    parser.add_argument("--limits", default=None,
                        help="joint limits in degrees: inline JSON or a JSON file")
    parser.add_argument("--bridge-limits", default=None, metavar="URL",
                        help="read joint limits from a Pi bridge base URL (GET only)")
    args = parser.parse_args()

    calibration = WristCalibration.load(str(ROOT / args.calibration)
                                        if not Path(args.calibration).is_absolute()
                                        else args.calibration)
    plan = plan_in_degrees(load_plan(ROOT / args.plan
                                     if not Path(args.plan).is_absolute() else args.plan))
    # A plan that is valid in sim but outside the arm's servo range fails at
    # the bench in the controller's precheck (Block 5, 2026-11-18: every
    # ChArUco pose asked for wrist_roll -87.2 deg against -81.82). Fail it
    # here instead, before the expensive placement sweep.
    limits = parse_limits(args.limits, args.bridge_limits)
    limit_report = assert_plan_limits(plan, limits) if limits else None
    if limit_report is not None:
        print(f"plan limits: worst margin {limit_report['worst_margin_deg']:+.2f} deg "
              f"over {limit_report['poses']} poses + "
              f"{limit_report['recovery']} recovery poses")
    mount = current_layout()["mount"]
    base_xy = (mount["white_arm_base_frame_mm"][0] / 1000.0,
               mount["white_arm_base_frame_mm"][1] / 1000.0)

    rows = []
    print(f"{'placement':>18} {'ok':>3} {'views':>5} {'markers':>7} "
          f"{'ex mm':>7} {'ey mm':>7} {'eyaw':>7}")
    failures = 0
    for dx_mm, dy_mm, dyaw_deg in parse_placements(args.placements):
        env = build_env((dx_mm / 1000.0, dy_mm / 1000.0), math.radians(dyaw_deg),
                        "charuco" if args.layout == "charuco" else "wood")
        try:
            board = env.board
            if args.layout == "charuco":
                layout = CharucoLayout(square_mm=board.square * 1000.0)
                marker_z = board.top
            else:
                from chess_sim.marker_scan import MarkerLayout
                layout = MarkerLayout.for_board(224.0, 15.0, plane="mat",
                                               margin_mm=40.0)
                marker_z = board.table_top + 0.00056
            clock = Clock()
            ctrl = ScanController(
                FakeBridge(env), FakeFrames(env, clock),
                lambda joints_deg: calibration.camera(
                    tuple(math.radians(v) for v in joints_deg)),
                plan, layout=layout, square_m=board.square, board_z_m=board.top,
                marker_z_m=float(marker_z), base_xy_m=base_xy,
                board_half_mm=board.width * 500.0, clock=clock, sleep=clock.sleep)
            report = ctrl.run()
            truth = board_offset(board.origin, board.yaw, base_xy=base_xy,
                                board_half_mm=board.width * 500.0)
            ex = ey = eyaw = None
            ok = bool(report.ok)
            if report.offset is not None:
                ex = report.offset.x_mm - truth.x_mm
                ey = report.offset.y_mm - truth.y_mm
                eyaw = report.offset.yaw_deg - truth.yaw_deg
                ok = (abs(ex) <= args.tolerance_mm and abs(ey) <= args.tolerance_mm
                      and abs(eyaw) <= args.tolerance_yaw_deg)
            if not ok:
                failures += 1
            rows.append({"placement": [dx_mm, dy_mm, dyaw_deg], "ok": ok,
                         "views_used": report.views_used,
                         "markers_seen": list(report.markers_seen),
                         "note": report.note,
                         "ex_mm": None if ex is None else round(ex, 3),
                         "ey_mm": None if ey is None else round(ey, 3),
                         "eyaw_deg": None if eyaw is None else round(eyaw, 3)})
            label = f"{dx_mm:+.0f},{dy_mm:+.0f},{dyaw_deg:+.1f}"
            print(f"{label:>18} {str(ok):>3} {report.views_used:>5} "
                  f"{len(report.markers_seen):>7} "
                  f"{'-' if ex is None else f'{ex:+.2f}':>7} "
                  f"{'-' if ey is None else f'{ey:+.2f}':>7} "
                  f"{'-' if eyaw is None else f'{eyaw:+.3f}':>7}"
                  f"{'' if ok else '   ' + report.note[:40]}")
        finally:
            env.close()

    seen = [len(row["markers_seen"]) for row in rows if row["markers_seen"]]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "plan": str(args.plan), "layout": args.layout,
        "calibration": calibration.calibration_id,
        "rms_gate_px": 1.2, "tolerance_mm": args.tolerance_mm,
        "tolerance_yaw_deg": args.tolerance_yaw_deg,
        "placements": rows,
        "joint_limits": limits or None,
        "joint_limit_check": limit_report,
        "summary": {"passed": len(rows) - failures, "total": len(rows),
                    "markers_min": min(seen) if seen else 0,
                    "markers_max": max(seen) if seen else 0},
    }, indent=2) + "\n")
    print(f"\n{len(rows) - failures}/{len(rows)} placements pass -> {out}")
    raise SystemExit(0 if failures == 0 else 1)


if __name__ == "__main__":
    main()
