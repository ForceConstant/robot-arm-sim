#!/usr/bin/env python3
"""Block 4 offline dry run: solve the known-problem squares, never move the arm.

This is the bench pre-flight for the IRL sweep.  It runs *the production solver*
(``live/kinematics`` -> ``solve_tip``, exactly what ``/move`` calls) against the
*real* servo limits read from the Pi bridge, and classifies every square into one
of three outcomes:

``OK``              the solver lands inside the 20 mm gate, every joint inside
                    the bridge range and the tip inside the keep-in box;
``IK_FAILURE``      no pose within the real joint limits gets inside the gate
                    (5 mm / 8 deg is the solver's own shortfall, not a refusal);
``JOINT_LIMIT``     the pose the solver wants sits outside a bridge limit and
                    the bounded solve cannot recover;
``KEEP_IN_REFUSAL`` the joints are legal but the resulting tip leaves the
                    keep-in box, so the live app would refuse the move.

It also runs an *unconstrained* probe (limits={}) for every square so a failure
can be attributed: "needs wrist_roll 160.8 deg, the bridge stops at 91.05" is a
limit finding; "unconstrained also misses" is a solver finding.

Nothing here talks to the arm.  The bridge is read with a single GET /state.

    robot-arm-sim python tools/block4_dryrun.py
    robot-arm-sim python tools/block4_dryrun.py --squares a1,b1,a8 --rescue
    robot-arm-sim python tools/block4_dryrun.py --out reports/block4_dryrun.json

See ``docs/BLOCK4_RUN_SHEET.md`` for the live equivalent (``POST /move`` with
``"dry_run": true``), which additionally requires the bridge in REMOTE + armed.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))
sys.path.insert(0, str(ROOT / "tools"))

from cartesian_squares import (JOINTS, ARM_JOINTS, _board_frame,  # noqa: E402
                               _margins, parse_limits)
from kinematics import ArmKinematics, square_xy  # noqa: E402
from tip_frame import TIP_IN_CHAIN  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402

GATE_MM = 20.0
GATE_DEG = 8.0
# The eight squares the planner flagged as problem cases across Blocks 1-3.
DEFAULT_SQUARES = "a1,b1,f1,g1,h1,a2,g2,g3"


def build(live_dir: Path, limits: dict) -> tuple[ArmKinematics, ArmKinematics, dict]:
    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    board = _board_frame(live_dir)
    bounded = ArmKinematics(calibration, limits, board)
    free = ArmKinematics(calibration, {}, board)
    for kin in (bounded, free):
        kin.camera_in_tool = np.asarray(calibration.camera_in_tool, float)
    return bounded, free, config


def keep_in_problems(config: dict, kinematics: ArmKinematics, joints) -> list[str]:
    """The same test ``Live._keep_in_check`` applies, on the jaw tip."""
    zone = {"x": float(config.get("keep_in_x_mm", 118.0)),
            "y": float(config.get("keep_in_y_mm", 118.0)),
            "zmin": float(config.get("keep_in_z_min_mm", -5.0)),
            "zmax": float(config.get("keep_in_z_max_mm", 320.0))}
    if not config.get("keep_in_enforce", True):
        return []
    tip = (kinematics.fk(joints) @ TIP_IN_CHAIN)[:3, 3]
    board = kinematics.to_board(tip) * 1000.0
    out = []
    if abs(float(board[0])) > zone["x"]:
        out.append(f"x {board[0]:.0f} outside ±{zone['x']:.0f}")
    if abs(float(board[1])) > zone["y"]:
        out.append(f"y {board[1]:.0f} outside ±{zone['y']:.0f}")
    if not zone["zmin"] <= float(board[2]) <= zone["zmax"]:
        out.append(f"z {board[2]:.0f} outside {zone['zmin']:.0f}..{zone['zmax']:.0f}")
    return out


def _tightest(margins: dict) -> tuple[str, float]:
    if not margins:
        return "-", 999.0
    name = min(margins, key=lambda k: margins[k])
    return name, float(margins[name])


def rescue(free: ArmKinematics, bounded: ArmKinematics, target, seed, span: float,
           samples: int = 240, rng_seed: int = 7):
    """Is the failure a seed-basin miss or a real limit?  Bounded random search."""
    rng = np.random.default_rng(rng_seed)
    lows, highs = bounded._bounds(1.6)
    best = None
    for _ in range(samples):
        start = rng.uniform(lows, highs)
        for extra in (0.0, 90.0, 180.0, -90.0):
            solved, _ = bounded.solve_tip(target, start, TIP_IN_CHAIN, span + extra)
            score = solved["position_error_mm"] + 15.0 * solved["approach_error_deg"]
            if best is None or score < best[0]:
                best = (score, solved, extra)
            if (solved["position_error_mm"] <= GATE_MM
                    and solved["approach_error_deg"] <= GATE_DEG):
                return best[1], extra, True
    return (best[1] if best else None), (best[2] if best else None), False


def classify(bounded, free, config, name, seed, do_rescue: bool,
             hover_mm: float = 60.0) -> dict:
    x, y = square_xy(name)
    z = float(hover_mm)
    target = bounded.board_point(x, y, z)
    solved, _ = bounded.solve_tip(target, seed, TIP_IN_CHAIN, 0.0)
    joints = [solved["joints_deg"][j] for j in JOINTS]
    margins = _margins(bounded, joints)
    ik_ok = (solved["position_error_mm"] <= GATE_MM
             and solved["approach_error_deg"] <= GATE_DEG)
    zone_bad = keep_in_problems(config, bounded, joints) if ik_ok else []
    violations = [{"joint": j, "value": v, "limit": [lo, hi]}
                  for j, v, (lo, hi) in bounded.violations(joints)]

    row = {"square": name, "target_mm": [x, y, z],
           "ik_mm": solved["position_error_mm"],
           "ik_deg": solved["approach_error_deg"],
           "joints_deg": {j: joints[i] for i, j in enumerate(JOINTS)},
           "margins_deg": margins,
           "limit_violations": violations}

    if not ik_ok:
        free_solved, _ = free.solve_tip(target, seed, TIP_IN_CHAIN, 0.0)
        need = [free_solved["joints_deg"][j] for j in ARM_JOINTS]
        excess = []
        for index, joint in enumerate(ARM_JOINTS):
            lo, hi = bounded.limits.get(joint, (-180.0, 180.0))
            value = float(need[index])
            if value < lo - 1e-6:
                excess.append({"joint": joint, "required_deg": round(value, 2),
                               "limit_deg": lo, "over_by_deg": round(lo - value, 2)})
            elif value > hi + 1e-6:
                excess.append({"joint": joint, "required_deg": round(value, 2),
                               "limit_deg": hi, "over_by_deg": round(value - hi, 2)})
        row["unconstrained"] = {"ik_mm": free_solved["position_error_mm"],
                                "ik_deg": free_solved["approach_error_deg"],
                                "joints_deg": {j: free_solved["joints_deg"][j]
                                               for j in ARM_JOINTS},
                                "outside_limits": excess}
        row["verdict"] = "JOINT_LIMIT" if excess else "IK_FAILURE"
        if do_rescue:
            found, span, ok = rescue(free, bounded, target, seed, 0.0)
            row["rescue"] = {"gate_ok": ok, "span_deg": span,
                             "ik_mm": None if found is None else found["position_error_mm"],
                             "ik_deg": None if found is None else found["approach_error_deg"],
                             "joints_deg": None if found is None else
                             {j: found["joints_deg"][j] for j in JOINTS}}
            if ok:
                row["verdict"] = "RESCUED_BY_SEED"
    elif zone_bad:
        row["verdict"] = "KEEP_IN_REFUSAL"
        row["keep_in"] = zone_bad
    else:
        row["verdict"] = "OK"
    tight_joint, tight_margin = _tightest(margins)
    row["tightest"] = {"joint": tight_joint, "margin_deg": round(tight_margin, 2)}
    return row


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--bridge-limits", default="http://192.168.1.167:8100",
                        help="bridge base URL for GET /state; 'none' to skip")
    parser.add_argument("--limits", default=None,
                        help="inline JSON or file of {joint: [lo, hi]} in degrees")
    parser.add_argument("--squares", default=DEFAULT_SQUARES)
    parser.add_argument("--hover-mm", type=float, default=60.0)
    parser.add_argument("--rescue", action="store_true",
                        help="wider bounded seed search for every failure")
    parser.add_argument("--out", default="reports/block4_dryrun.json")
    args = parser.parse_args()

    bridge = None if args.bridge_limits.lower() in ("none", "off", "") else args.bridge_limits
    limits = parse_limits(args.limits, bridge)
    if not limits:
        raise SystemExit("refusing to dry-run without real limits: pass --bridge-limits")

    bounded, free, config = build(Path(args.live_dir), limits)
    seed = [float(config["home_joints"][j]) for j in JOINTS]
    names = [n.strip() for n in args.squares.split(",") if n.strip()]

    rows = []
    print(f"{'sq':>3}  {'verdict':<16} {'ik mm':>6} {'ik°':>5}  tightest joint (margin°)")
    for name in names:
        row = classify(bounded, free, config, name, seed, args.rescue,
                       args.hover_mm)
        rows.append(row)
        tight = row["tightest"]
        print(f"{name:>3}  {row['verdict']:<16} {row['ik_mm']:>6.1f} "
              f"{row['ik_deg']:>5.2f}  {tight['joint']} ({tight['margin_deg']:+.1f})")
        for bad in row.get("unconstrained", {}).get("outside_limits", []):
            print(f"      needs {bad['joint']} {bad['required_deg']:+.1f}°, "
                  f"limit {bad['limit_deg']:+.2f}° → over by {bad['over_by_deg']:.1f}°")
        for note in row.get("keep_in", []):
            print(f"      keep-in: {note}")
        if "rescue" in row:
            print(f"      rescue: {'YES' if row['rescue']['gate_ok'] else 'no'} "
                  f"(span {row['rescue']['span_deg']}, "
                  f"{row['rescue']['ik_mm']} mm)")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"limits": {k: list(v) for k, v in limits.items()},
                               "seed_joints_deg": dict(zip(JOINTS, seed)),
                               "gate_mm": GATE_MM, "gate_deg": GATE_DEG,
                               "squares": rows}, indent=2) + "\n")
    tally: dict[str, int] = {}
    for row in rows:
        tally[row["verdict"]] = tally.get(row["verdict"], 0) + 1
    print("\n" + ", ".join(f"{k} {v}" for k, v in sorted(tally.items())))
    print(f"-> {out}")
    if any(row["verdict"] != "OK" for row in rows):
        sys.exit(1)


if __name__ == "__main__":
    main()
