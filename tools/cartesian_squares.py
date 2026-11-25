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

Real-arm safety (added 2026-11-25 after the FULL-PARITY-01 first pass)
---------------------------------------------------------------------
The IRL column is a gate, so it must not send a target the plan has not
cleared.  Before every ``/move`` the runner now:

* solves the same tip target with the *app's* solver (``MoveSolver`` on the
  mink model) and refuses locally on the configured move tolerance / 8 deg
  approach gate or a joint-limit violation (nothing is sent);
* walks the joint-space path in 2 deg steps and refuses if the jaw tip leaves
  the app's keep-in box (``GET /keep-in``) or drops below the table;
* prints the leg — target, solve error, roll and margins — before executing;
* holds a ``/stream`` reader open for the sweep (the detector only runs with a
  client attached);
* stops the sweep on the first HTTP refusal (409), bridge error or wrist-roll
  branch change instead of driving on.

``--continue-on-refusal`` records a 409 "no reachable pose" refusal (the arm
did not move) and continues to the next square; every other 409 and every 5xx
still stops the sweep. ``--only a1,b2`` restricts a pass to named squares.

``--dry-run`` does the solve, clearance and print for all 64 squares without
sending anything, which is how the runner itself is checked before motion.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import urllib.error
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


def _get_json(url: str, timeout: float = 10.0) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read())


def _post_json(url: str, body: dict, timeout: float = 180.0) -> dict:
    request = urllib.request.Request(
        url, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read())


class _StreamReader(threading.Thread):
    """Hold one ``/stream`` client open so the app's detector runs."""

    def __init__(self, url: str) -> None:
        super().__init__(daemon=True)
        self.url = url
        self.stop = threading.Event()
        self.error = None

    def run(self) -> None:
        try:
            with urllib.request.urlopen(self.url, timeout=20) as response:
                while not self.stop.is_set():
                    if not response.read(65536):
                        break
        except Exception as exc:                     # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"


def _irl_preflight(base: str) -> tuple[dict, dict]:
    """Bridge state + keep-in zone; refuses an unarmed or faulted bridge."""
    state = _get_json(base + "/state")
    bridge = state.get("bridge") or {}
    if bridge.get("estop"):
        raise SystemExit("bridge estop is engaged; motion is not authorised")
    if bridge.get("fault"):
        raise SystemExit(f"bridge fault: {bridge['fault']}; motion is not authorised")
    if not bridge.get("armed"):
        raise SystemExit("bridge is not armed; motion is not authorised")
    if not bridge.get("joints") or not bridge.get("limits"):
        raise SystemExit("bridge returned no joint state or limits")
    zone = _get_json(base + "/keep-in")
    return bridge, zone


def _irl_solver(live_dir: Path, limits: dict | None):
    """The solver the app itself uses (mink), with the app's tool frame."""
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from solver import MoveSolver
    from chess_sim import gripper
    from chess_sim.wrist_calibration import WristCalibration

    if not gripper.CUSTOM:
        raise SystemExit(
            "the IRL sweep needs the installed CAD tool: set "
            "ROBOT_GRIPPER=cad_parallel and "
            "ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y "
            "(the launcher default is the pre-5y pack)")
    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    kinematics = ArmKinematics(calibration, limits or {}, _board_frame(live_dir))
    kinematics.camera_in_tool = np.asarray(calibration.camera_in_tool, float)
    solver = MoveSolver(kinematics, calibration, TIP_IN_CHAIN, use_mink=True)
    if limits:
        solver.sync_limits(limits)
    return kinematics, TIP_IN_CHAIN, solver, config


def _path_clearance(kinematics, jaw, start, target, zone,
                    step_deg: float = 2.0) -> dict:
    """Walk the joint-space leg and check the jaw tip stays in keep-in."""
    span = max(abs(a - b) for a, b in zip(target[:5], start[:5]))
    steps = max(2, int(math.ceil(span / step_deg)))
    worst = {"x": 0.0, "y": 0.0, "zmin": 1e9}
    bad = 0
    for k in range(steps + 1):
        t = k / steps
        q = [a + (b - a) * t for a, b in zip(start, target)]
        tip = kinematics.to_board((kinematics.fk(q) @ jaw)[:3, 3]) * 1000.0
        worst["x"] = max(worst["x"], abs(float(tip[0])))
        worst["y"] = max(worst["y"], abs(float(tip[1])))
        worst["zmin"] = min(worst["zmin"], float(tip[2]))
        inside = (abs(float(tip[0])) <= zone["x_mm"]
                  and abs(float(tip[1])) <= zone["y_mm"]
                  and zone["z_min_mm"] <= float(tip[2]) <= zone["z_max_mm"])
        bad += 0 if inside else 1
    return {"clear": bad == 0, "violations": bad,
            "worst": {k: round(v, 1) for k, v in worst.items()},
            "samples": steps + 1}


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


def run_irl(squares, hover_mm: float, base: str, live_dir: Path,
            limits: dict | None = None, dry_run: bool = False,
            gate_mm: float | None = None, gate_deg: float = 8.0,
            attach_stream: bool = True,
            continue_on_refusal: bool = False) -> dict:
    """Paired real-arm sweep with the plan's preconditions applied per leg."""
    bridge, zone = _irl_preflight(base)
    limits = limits or {k: [float(v[0]), float(v[1])]
                        for k, v in bridge["limits"].items()}
    kinematics, jaw, solver, config = _irl_solver(live_dir, limits)
    gate = float(gate_mm if gate_mm is not None
                 else config.get("move_tolerance_mm", 20.0))
    seed = [float(bridge["joints"][name]) for name in JOINTS]

    reader = None
    if attach_stream and not dry_run:
        reader = _StreamReader(base + "/stream")
        reader.start()
        deadline = time.time() + 5.0
        seen = False
        while time.time() < deadline:
            try:
                seen = _get_json(base + "/state").get("stream_age_s") is not None
            except Exception:                        # noqa: BLE001
                seen = False
            if seen:
                break
            time.sleep(0.2)
        if not seen:
            reader.stop.set()
            raise SystemExit(
                f"no /stream client registered by the app ({reader.error or 'timeout'}); "
                "refusing to sweep with the detector idle")
        print(f"stream: attached {base}/stream (detector active)")

    rows = []
    stopped = None
    total = len(squares)

    def _summarise(q):
        margins = _margins(kinematics, q)
        violations = [{"joint": j, "value": v, "limit": [lo, hi]}
                      for j, v, (lo, hi) in kinematics.violations(q)]
        tight = min(margins.items(), key=lambda kv: kv[1]) if margins else ("-", 0)
        return margins, violations, tight

    for index, (name, x, y, z, span) in enumerate(squares, 1):
        point = kinematics.board_point(x, y, z)
        solved, _goal = solver.solve_tip(point, seed, jaw, span)
        q = [float(solved["joints_deg"][j]) for j in JOINTS]
        margins, violations, tight = _summarise(q)
        roll = q[4]
        print(f"leg {index:02d}/{total} {name}: board ({x:+.1f},{y:+.1f},{z:+.1f}) "
              f"span {span:g} | solve {solved['position_error_mm']:.2f} mm / "
              f"{solved['approach_error_deg']:.1f} deg | roll {roll:+.1f} | "
              f"tightest {tight[0]} {tight[1]:+.1f} deg")

        reason = None
        if violations:
            reason = f"joint limit violated: {violations[0]}"
        elif solved["position_error_mm"] > gate:
            reason = f"solve {solved['position_error_mm']:.2f} mm > {gate:g} mm"
        elif solved["approach_error_deg"] > gate_deg:
            reason = (f"approach {solved['approach_error_deg']:.1f} deg > "
                      f"{gate_deg:g} deg")
        clearance = _path_clearance(kinematics, jaw, seed, q, zone)
        if reason is None and not clearance["clear"]:
            reason = (f"path leaves keep-in ({clearance['violations']} samples, "
                      f"worst {clearance['worst']})")
        planned_roll = q[4] - seed[4]
        if reason is None and abs(planned_roll) > 45.0:
            reason = (f"planned wrist-roll branch change "
                      f"{seed[4]:+.1f} -> {q[4]:+.1f} deg")
        if reason is not None:
            print(f"  REFUSED before send: {reason}")
            rows.append({"square": name, "target": [x, y, z], "solved": False,
                         "settle_deg": None, "detail": f"pre-check: {reason}",
                         "achieved": None, "joints_deg": dict(
                             zip(JOINTS, q)), "margins_deg": margins,
                         "clearance": clearance})
            continue

        if dry_run:
            print(f"  dry-run: clearance ok ({clearance['samples']} samples, "
                  f"worst {clearance['worst']}); not sent")
            rows.append({"square": name, "target": [x, y, z], "solved": True,
                         "settle_deg": None, "detail": "dry-run",
                         "achieved": None, "joints_deg": dict(
                             zip(JOINTS, q)), "margins_deg": margins,
                         "clearance": clearance})
            seed = q
            continue

        try:
            result = _post_json(base + "/move",
                                {"mode": "xyz", "x_mm": x, "y_mm": y,
                                 "z_mm": z, "span_deg": span})
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode()[:200]
            print(f"  REFUSED by app: HTTP {exc.code} {detail[:120]}")
            rows.append({"square": name, "target": [x, y, z], "solved": False,
                         "settle_deg": None,
                         "detail": f"HTTP {exc.code}: {detail}",
                         "achieved": None, "clearance": clearance})
            if (continue_on_refusal and exc.code == 409
                    and "no reachable pose" in detail):
                # The app refused the target; no motion was commanded. The
                # plan's stop rule exists for an unexpected state after motion,
                # which this is not — record the refusal and move on.
                print("  recorded refusal (no motion); continuing")
                continue
            stopped = f"HTTP {exc.code} on {name}"
            break
        except Exception as exc:                     # noqa: BLE001
            print(f"  STOP: {type(exc).__name__}: {exc}")
            rows.append({"square": name, "target": [x, y, z], "solved": False,
                         "settle_deg": None,
                         "detail": f"{type(exc).__name__}: {exc}",
                         "achieved": None, "clearance": clearance})
            stopped = f"{type(exc).__name__} on {name}"
            break

        pose = result.get("pose") or {}
        after = result.get("joints_deg") or {}
        roll_after = float(after.get("wrist_roll", roll))
        roll_delta = roll_after - roll
        rows.append({"square": name, "target": [x, y, z], "solved": True,
                     "settle_deg": result.get("error_deg"), "detail": None,
                     "achieved": pose.get("tip_mm"),
                     "joints_deg": after,
                     "roll_deg": round(roll_after, 2),
                     "roll_delta_deg": round(roll_delta, 2),
                     "marker": result.get("square"),
                     "aim_pixel": result.get("aim_pixel"),
                     "offset_px": result.get("offset_px"),
                     "clearance": clearance})
        print(f"  done: settle {result.get('error_deg')} deg | tip {pose.get('tip_mm')}"
              f" | roll {roll_after:+.1f} ({roll_delta:+.1f})")
        seed = [float(after.get(j, q[i])) for i, j in enumerate(JOINTS)]
        if abs(roll_delta) > 45.0:
            print(f"  STOP: wrist-roll branch change {roll:+.1f} -> {roll_after:+.1f}")
            stopped = f"branch change on {name}"
            break

    if reader is not None:
        reader.stop.set()
    return {"backend": "irl", "hover_mm": hover_mm, "dry_run": dry_run,
            "gate_mm": gate, "gate_deg": gate_deg,
            "limits": {k: list(v) for k, v in kinematics.limits.items()},
            "keep_in": {"x_mm": zone.get("x_mm"), "y_mm": zone.get("y_mm"),
                        "z_min_mm": zone.get("z_min_mm"),
                        "z_max_mm": zone.get("z_max_mm")},
            "stream": (None if reader is None
                       else {"attached": True, "error": reader.error}),
            "stopped": stopped, "squares": rows}


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
    parser.add_argument("--dry-run", action="store_true",
                        help="irl: solve, clearance-check and print every leg, send nothing")
    parser.add_argument("--gate-mm", type=float, default=None,
                        help="irl local solve gate; defaults to live_config move_tolerance_mm")
    parser.add_argument("--gate-deg", type=float, default=8.0,
                        help="irl approach-angle gate in degrees (default 8.0, the app's)")
    parser.add_argument("--no-stream", action="store_true",
                        help="irl: do not attach a /stream reader (detector stays idle)")
    parser.add_argument("--only", default=None,
                        help="comma-separated square names to visit (e.g. c8,d8,e8)")
    parser.add_argument("--continue-on-refusal", action="store_true",
                        help="irl: record a 409 'no reachable pose' (no motion) and "
                             "continue; every other 409 still stops the sweep")
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
    if args.only:
        wanted = [s.strip() for s in args.only.split(",") if s.strip()]
        known = {row[0] for row in targets}
        unknown = [s for s in wanted if s not in known]
        if unknown:
            parser.error(f"unknown square(s): {', '.join(unknown)}")
        targets = [row for row in targets if row[0] in wanted]
    if args.backend == "sim":
        result = run_sim(Path(args.live_dir), targets, args.hover_mm, limits)
    else:
        result = run_irl(targets, args.hover_mm, args.base, Path(args.live_dir),
                         limits=limits, dry_run=args.dry_run,
                         gate_mm=args.gate_mm, gate_deg=args.gate_deg,
                         attach_stream=not args.no_stream,
                         continue_on_refusal=args.continue_on_refusal)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n")
    solved = sum(1 for r in result["squares"] if r["solved"])
    print(f"{args.backend}: {solved}/{len(result['squares'])} solved -> {args.out}")
    if result.get("dry_run"):
        print("dry run: nothing was sent to the arm")
    if result.get("stopped"):
        print(f"STOPPED: {result['stopped']}")
    for row in result["squares"]:
        if not row["solved"]:
            print(f"  FAIL {row['square']}: {row.get('detail') or 'refused'}")


if __name__ == "__main__":
    main()
