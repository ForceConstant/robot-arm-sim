#!/usr/bin/env python3
"""One 64-square Cartesian sweep, two backends: simulator and real arm.

The point is correlation, not coverage: the same target list and the same solver
(``live/kinematics`` — exactly what the dashboard uses) run against

* ``sim``  — MuJoCo sets the joints the solver returned; the *achieved* jaw-tip
  pose and any markers in the wrist view are read back from the model;
* ``irl``  — the live app receives the same ``/move`` request; its own reported
  error and marker read-back are recorded.

Comparing the two columns is how you tell a solver problem from a hardware one:
identical targets and identical maths, so differences must be physical.

    robot-arm-sim python tools/cartesian_squares.py --backend sim --out reports/squares_sim.json
    robot-arm-sim python tools/cartesian_squares.py --backend irl --out reports/squares_irl.json
    robot-arm-sim python tools/cartesian_squares.py --compare reports/squares_sim.json reports/squares_irl.json

Real-arm joint limits
---------------------
By default the solver runs *unconstrained* (``limits={}``, i.e. ±180) which is
what the historical 64/64 sim result was measured with.  The Pi bridge reports
much narrower servo ranges; a solve that needs wrist_roll 160.8 deg is valid in
sim and impossible on the arm.  Pass ``--bridge-limits`` (or ``--limits``) to
bound the solver with the real ranges so the count is honest:

    robot-arm-sim python tools/cartesian_squares.py --backend sim \
        --bridge-limits http://192.168.1.167:8100 \
        --out reports/squares_sim_bridge_limits.json

``--bridge-limits`` reads ``/state`` with a plain GET (no motion command).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
ARM_JOINTS = JOINTS[:5]
FILES = "abcdefgh"


def square_targets(hover_mm: float = 60.0, square_mm: float = 28.0):
    """Board-frame targets for every square: (name, x, y, z, span_deg)."""
    half = square_mm * 4.0
    out = []
    for rank in range(1, 9):
        for index, file in enumerate(FILES):
            x = (index + 0.5) * square_mm - half
            y = (rank - 1 + 0.5) * square_mm - half
            out.append((f"{file}{rank}", x, y, hover_mm, 0.0))
    return out


def _board_frame(live_dir: Path) -> np.ndarray:
    import numpy as _np
    spec = json.loads((live_dir / "board.json").read_text())
    yaw = math.radians(float(spec["yaw_deg"]))
    frame = _np.eye(4)
    frame[:3, :3] = _np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                               [math.sin(yaw), math.cos(yaw), 0.0],
                               [0.0, 0.0, 1.0]])
    frame[:3, 3] = [spec["x_mm"] / 1000.0, spec["y_mm"] / 1000.0,
                    spec["z_mm"] / 1000.0]
    return frame


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


def _solver(live_dir: Path, limits: dict | None = None):
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from chess_sim.wrist_calibration import WristCalibration

    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    kinematics = ArmKinematics(calibration, limits or {}, _board_frame(live_dir))
    kinematics.camera_in_tool = np.asarray(calibration.camera_in_tool, float)
    return kinematics, TIP_IN_CHAIN


def _margins(kinematics, joints_deg) -> dict:
    """Degrees of headroom to each real limit; negative means past it."""
    out = {}
    for index, name in enumerate(ARM_JOINTS):
        if index >= len(joints_deg):
            break
        lo, hi = kinematics.limits.get(name, (-180.0, 180.0))
        value = float(joints_deg[index])
        out[name] = round(min(value - lo, hi - value), 2)
    return out


def run_sim(live_dir: Path, squares, hover_mm: float,
            limits: dict | None = None) -> dict:
    import mujoco
    from chess_sim.config import AppearanceConfig, BoardConfig, EnvConfig
    from chess_sim.env import ChessSimEnv
    from chess_sim import fiducials

    kinematics, jaw = _solver(live_dir, limits)
    board = BoardConfig(face="charuco", yaw=math.pi)
    env = ChessSimEnv(EnvConfig(board=board,
                                appearance=AppearanceConfig(board_markers=False)))
    env.reset("8/8/8/8/8/8/8/8")
    model, data = env.model, env.data
    rows = []
    for name, x, y, z, span in squares:
        point = kinematics.board_point(x, y, z)
        seed = [float(data.qpos[env._joint_qpos[i]]) for i in range(6)]
        solved, goal = kinematics.solve_tip(point, seed, jaw, span)
        ok = solved["position_error_mm"] <= 20.0 and solved["approach_error_deg"] <= 8.0
        achieved, markers = None, 0
        q = [solved["joints_deg"][j] for j in JOINTS]
        margins = _margins(kinematics, q)
        violations = [{"joint": j, "value": v, "limit": [lo, hi]}
                      for j, v, (lo, hi) in kinematics.violations(q)]
        if ok:
            data.qpos[env._joint_qpos] = np.radians(q)
            mujoco.mj_forward(model, data)
            # Same chain->tip transform the live app uses, so the sim column
            # and the IRL column mean the same thing.
            tip = kinematics.fk(q) @ jaw
            achieved = [round(float(v), 1) for v in
                        (kinematics.to_board(tip[:3, 3]) * 1000.0)]
            image = env.render("wrist")
            markers = len([d for d in fiducials.detect_markers(image)
                           if 5 <= d.marker_id <= 36])
        rows.append({"square": name, "target": [x, y, z], "solved": bool(ok),
                     "ik_mm": solved["position_error_mm"],
                     "ik_deg": solved["approach_error_deg"],
                     "joints": {j: q[i] for i, j in enumerate(JOINTS)},
                     "margins_deg": margins, "violations": violations,
                     "starts": solved.get("starts"),
                     "achieved": achieved, "markers": markers})
    env.close()
    return {"backend": "sim", "hover_mm": hover_mm,
            "limits": {k: list(v) for k, v in kinematics.limits.items()},
            "squares": rows}


def run_irl(squares, hover_mm: float, base: str) -> dict:
    rows = []
    for name, x, y, z, span in squares:
        body = json.dumps({"mode": "xyz", "x_mm": x, "y_mm": y, "z_mm": z,
                           "span_deg": span}).encode()
        request = urllib.request.Request(base + "/move", data=body,
                                         headers={"Content-Type": "application/json"},
                                         method="POST")
        try:
            with urllib.request.urlopen(request, timeout=180) as response:
                result = json.loads(response.read())
            ok, detail = True, None
        except urllib.error.HTTPError as exc:
            result = {}
            ok = False
            detail = exc.read().decode()[:120]
        rows.append({"square": name, "target": [x, y, z], "solved": ok,
                     "settle_deg": result.get("error_deg"), "detail": detail,
                     "achieved": ((result.get("pose") or {}).get("tip_mm")
                                  or (result.get("pose") or {}).get("tool_mm"))})
    return {"backend": "irl", "hover_mm": hover_mm, "squares": rows}


def compare(paths) -> None:
    sim = json.loads(Path(paths[0]).read_text())
    irl = json.loads(Path(paths[1]).read_text())
    by_sim = {r["square"]: r for r in sim["squares"]}
    print(f"{'square':>6} {'sim':>5} {'irl':>5} {'sim ik mm':>10} {'irl settle':>11}  note")
    agree = 0
    for row in irl["squares"]:
        other = by_sim.get(row["square"], {})
        same = bool(other.get("solved")) == bool(row["solved"])
        agree += 1 if same else 0
        print(f"{row['square']:>6} {str(other.get('solved')):>5} {str(row['solved']):>5} "
              f"{other.get('ik_mm', '?'):>10} {str(row.get('settle_deg')):>11}  "
              f"{'' if same else 'MISMATCH ' + str(row.get('detail'))[:40]}")
    total = len(irl["squares"])
    print(f"\nagreement {agree}/{total}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("sim", "irl"), default="sim")
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--hover-mm", type=float, default=60.0)
    parser.add_argument("--base", default="http://192.168.1.171:8080")
    parser.add_argument("--out", default="reports/squares_sim.json")
    parser.add_argument("--compare", nargs=2)
    parser.add_argument("--reverse", action="store_true",
                        help="visit the squares in reverse order (order check)")
    parser.add_argument("--limits", default=None,
                        help="joint limits in degrees: inline JSON or a JSON file")
    parser.add_argument("--bridge-limits", default=None,
                        metavar="URL",
                        help="read joint limits from a Pi bridge base URL (GET only)")
    args = parser.parse_args()

    if args.compare:
        compare(args.compare)
        return

    limits = parse_limits(args.limits, args.bridge_limits)
    if limits:
        print("limits: " + ", ".join(
            f"{name} {lo:g}..{hi:g}" for name, (lo, hi) in limits.items()))
    else:
        print("limits: NONE (unconstrained ±180; not the real arm)")

    targets = square_targets(args.hover_mm)
    if args.reverse:
        targets = list(reversed(targets))
    if args.backend == "sim":
        result = run_sim(Path(args.live_dir), targets, args.hover_mm, limits)
    else:
        result = run_irl(targets, args.hover_mm, args.base)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n")
    solved = sum(1 for r in result["squares"] if r["solved"])
    print(f"{args.backend}: {solved}/{len(result['squares'])} solved -> {args.out}")
    for row in result["squares"]:
        if not row["solved"]:
            tight = min(row.get("margins_deg", {"-": 999}).items(),
                        key=lambda kv: kv[1])
            print(f"  FAIL {row['square']}: "
                  f"{row['ik_mm']} mm / {row['ik_deg']} deg; "
                  f"tightest {tight[0]} margin {tight[1]} deg")


if __name__ == "__main__":
    main()
