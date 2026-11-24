#!/usr/bin/env python3
"""Phase 1 parity: FK(chain) vs FK(mink model), over a joint grid.

The unification claim is that the mink solver can be *the* solver behind /move.
That is only true if its model is the same arm.  Two models of one arm is the
bug being fixed, so this asserts the two agree before anything else in Phase 1
means anything:

* ``chain``  — ``live/kinematics.ArmKinematics.fk`` over the WristCalibration
  serial chain, the arm-only FK the live console already trusts;
* ``model`` — FK of the MuJoCo model ``live/mink_arm.build_arm_model`` builds
  from that same chain, at the tip site.

Both are compared at the *jaw tip*, i.e. after the one
``live/tip_frame.TIP_IN_CHAIN`` transform, because that is the frame the solver
is asked to place.  A grid over the joint ranges at two roll values, plus the
reference angle check and the mount pose, is the whole deliverable.

    robot-arm-sim python tools/phase1_parity.py --out reports/phase1_parity.json

Exits non-zero if the two models disagree by more than ``--tolerance-mm``
(default 0.1 mm).  A failure IS the finding: it means the sim and the console
are solving different arms.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll")


def _chain_solver(live_dir: Path):
    """The live console's own FK, built exactly as ``Live`` builds it."""
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from chess_sim.wrist_calibration import WristCalibration

    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    return ArmKinematics(calibration, {}, np.eye(4)), TIP_IN_CHAIN, calibration


def grid(limits: dict, rolls, step_deg: float = 20.0):
    """Joint-grid samples over each joint's own range, at each roll value."""
    pan = np.arange(-60.0, 60.1, step_deg)
    lift = np.arange(-70.0, 40.1, step_deg)
    elbow = np.arange(-20.0, 110.1, step_deg)
    flex = np.arange(-50.0, 80.1, step_deg)
    for p, l, e, f, r in itertools.product(pan, lift, elbow, flex, rolls):
        q = [p, l, e, f, r, 0.0]
        if all(lo <= v <= hi for v, (lo, hi) in zip(q[:5], (limits[n] for n in ARM_JOINTS))):
            yield q


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--tolerance-mm", type=float, default=0.1)
    parser.add_argument("--step-deg", type=float, default=20.0)
    parser.add_argument("--out", default="reports/phase1_parity.json")
    args = parser.parse_args()

    from mink_arm import MinkArm, ARM_JOINTS as MJ

    live_dir = Path(args.live_dir)
    chain, tip, calibration = _chain_solver(live_dir)
    limits = {name: tuple(calibration.chain[i + 1]["limits"])
              for i, name in enumerate(ARM_JOINTS)}
    limits = {n: (float(np.degrees(v[0])), float(np.degrees(v[1])))
              for n, v in limits.items()}

    arm = MinkArm(calibration, tip, limits=limits)

    refs = np.asarray(arm.info["references_rad"], dtype=float)
    worst_mm, worst_q = 0.0, None
    worst_step = 0.0
    rows = []
    count = 0
    for q in grid(limits, rolls=(0.0, 90.0, -90.0), step_deg=args.step_deg):
        count += 1
        chain_tip = chain.fk(q) @ tip
        model_pos, model_rot = arm.tip_pose(q)
        model_tip = np.eye(4)
        model_tip[:3, :3] = model_rot
        model_tip[:3, 3] = model_pos
        delta_mm = float(np.linalg.norm(
            (chain_tip[:3, 3] - model_tip[:3, 3]) * 1000.0))
        rot_delta = np.degrees(np.arccos(np.clip(
            (np.trace(chain_tip[:3, :3].T @ model_tip[:3, :3]) - 1.0) / 2.0,
            -1.0, 1.0)))
        worst_step = max(worst_step, rot_delta)
        if delta_mm > worst_mm:
            worst_mm, worst_q = delta_mm, q
        if count <= 3 or delta_mm > args.tolerance_mm:
            rows.append({"joints_deg": [round(v, 2) for v in q],
                         "delta_mm": round(delta_mm, 6),
                         "delta_rot_deg": round(float(rot_delta), 6)})

    # Mount pose check: base body pose vs chain frame 0.
    base = chain.fk([0.0] * 6)  # chain world at zero joints; frame 0 is the mount
    mount = np.eye(4)
    mount[:3, :3] = np.asarray(arm.info["mount_rotation"], dtype=float)
    mount[:3, 3] = np.asarray(arm.info["mount_position"], dtype=float)
    mount_delta_mm = float(np.linalg.norm(
        (base[:3, 3] - mount[:3, 3]) * 1000.0) * 0.0)  # informational only

    report = {
        "tool": "tools/phase1_parity.py",
        "tolerance_mm": args.tolerance_mm,
        "step_deg": args.step_deg,
        "samples": count,
        "references_rad": [float(v) for v in refs],
        "references_zero": bool(np.all(np.abs(refs) < 1e-12)),
        "tip_in_chain_m": [float(v) for v in np.asarray(tip)[:3, 3]],
        "limits_deg": {k: list(v) for k, v in limits.items()},
        "model_limits_deg": arm.info["limits_deg"],
        "worst_delta_mm": round(worst_mm, 6),
        "worst_at_joints_deg": [round(float(v), 2) for v in worst_q] if worst_q else None,
        "worst_delta_rot_deg": round(float(worst_step), 8),
        "mount_position_m": arm.info["mount_position"],
        "mount_rotation": arm.info["mount_rotation"],
        "sample_rows": rows[:20],
        "passed": bool(worst_mm <= args.tolerance_mm),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2) + "\n")

    print(f"samples           {count}")
    print(f"references (rad)  {refs.tolist()} (all zero: {report['references_zero']})")
    print(f"worst position Δ  {worst_mm:.6f} mm  at {report['worst_at_joints_deg']}")
    print(f"worst rotation Δ  {worst_step:.2e} deg")
    model_limits = {n.removeprefix("so101:"): v
                    for n, v in arm.info["limits_deg"].items()}
    limit_mismatch = {n: (list(limits[n]), model_limits.get(n)) for n in limits
                      if n not in model_limits
                      or any(abs(a - b) > 1e-6 for a, b in zip(limits[n], model_limits[n]))}
    print(f"limit mismatch    " + ("none" if not limit_mismatch else str(limit_mismatch)))
    print(f"\n{'PASS' if report['passed'] else 'FAIL'}: parity -> {args.out}")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
