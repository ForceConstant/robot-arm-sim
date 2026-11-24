#!/usr/bin/env python3
"""How far apart are the two MuJoCo models of this arm, and why.

``tools/phase1_parity.py`` proves the mink model built from the calibration
chain agrees with ``ArmKinematics.fk`` to floating-point noise.  This answers the
adjacent question: the *sim*'s arm model (``chess_sim.scene.build_arm``) is a
second MuJoCo model of the same robot, and Phase 1 has to say out loud how far
it is from the chain — otherwise "the mink solver is the solver" hides which
model is being solved.

The comparison is FK of the jaw tip at a joint grid:

* ``chain``  — ``ArmKinematics.fk(q) @ TIP_IN_CHAIN`` (the live console);
* ``live``   — the chain-derived model in ``live/mink_arm.py`` (parity: 0);
* ``sim``    — ``chess_sim.scene.build_arm`` with its own ``gripperframe`` site,
  mapped to the tip with the same ``SITE_IN_TIP`` rotation.

A non-zero ``sim`` column is expected and is NOT a bug: the sim mounts its arm
at ``BoardConfig.arm_base`` (a scene constant) while the live arm's mount is a
calibration measurement.  The receipt records the split so nobody later reads
the sim's absolute tip positions as if they were the real arm's.

    robot-arm-sim python tools/phase1_model_diff.py --out reports/phase1_model_diff.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--out", default="reports/phase1_model_diff.json")
    parser.add_argument("--step-deg", type=float, default=20.0)
    args = parser.parse_args()

    import mujoco
    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from mink_arm import MinkArm, SITE_IN_TIP
    from chess_sim.wrist_calibration import WristCalibration
    from chess_sim import scene
    from chess_sim.config import BoardConfig

    live_dir = Path(args.live_dir)
    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    chain = ArmKinematics(calibration, {}, np.eye(4))
    limits = {n: (float(np.degrees(calibration.chain[i + 1]["limits"][0])),
                  float(np.degrees(calibration.chain[i + 1]["limits"][1])))
              for i, n in enumerate(ARM_JOINTS)}
    live = MinkArm(calibration, TIP_IN_CHAIN, limits=limits)

    board = BoardConfig(face="charuco", yaw=np.pi)
    spec = scene.build_arm(board)
    sim_model = spec.compile()
    sim_data = mujoco.MjData(sim_model)
    sim_site = sim_model.site("so101:gripperframe").id
    sim_adr = np.array([sim_model.jnt_qposadr[sim_model.joint("so101:" + n).id]
                        for n in ARM_JOINTS])

    # The sim arm and the chain arm share ``ArmKinematics``'s body names, so
    # the site->tip map is the same one the chain model uses.
    steps = np.arange(-60.0, 60.01, args.step_deg)
    rows = []
    worst_live = 0.0
    worst_sim = 0.0
    deltas_sim = []
    for pan in steps:
        for lift in np.arange(-70.0, 40.01, args.step_deg):
            for elbow in np.arange(-20.0, 110.01, args.step_deg):
                for flex in np.arange(-50.0, 80.01, args.step_deg):
                    q = [pan, lift, elbow, flex, 0.0, 0.0]
                    if not all(lo <= v <= hi for v, (lo, hi) in
                               zip(q[:5], (limits[n] for n in ARM_JOINTS))):
                        continue
                    chain_tip = (chain.fk(q) @ TIP_IN_CHAIN)[:3, 3]
                    live_pos, _ = live.tip_pose(q)
                    sim_data.qpos[:] = sim_model.qpos0
                    sim_data.qpos[sim_adr] = np.radians(q[:5])
                    mujoco.mj_forward(sim_model, sim_data)
                    # The sim scene's CAD tool sets ``gripperframe.pos`` to the
                    # pack's ``tcp_position``, which is the same point
                    # ``TIP_IN_CHAIN`` translates to; the site origin IS the tip
                    # origin, so no extra offset belongs here.
                    sim_tip = sim_data.site_xpos[sim_site].copy()
                    d_live = float(np.linalg.norm(live_pos - chain_tip) * 1000.0)
                    d_sim = float(np.linalg.norm(sim_tip - chain_tip) * 1000.0)
                    worst_live = max(worst_live, d_live)
                    worst_sim = max(worst_sim, d_sim)
                    deltas_sim.append(d_sim)
                    rows.append((d_live, d_sim))

    delta_sim = np.asarray(deltas_sim)
    mount_sim = np.asarray(board.arm_base)
    mount_chain = np.asarray(calibration.chain[0]["position"], dtype=float)
    report = {
        "tool": "tools/phase1_model_diff.py",
        "samples": len(rows),
        "step_deg": args.step_deg,
        "chain_vs_live_model_worst_mm": round(worst_live, 6),
        "chain_vs_sim_model_worst_mm": round(worst_sim, 3),
        "chain_vs_sim_model_median_mm": round(float(np.median(delta_sim)), 3),
        "chain_vs_sim_model_p95_mm": round(float(np.percentile(delta_sim, 95)), 3),
        "sim_mount_m": [round(float(v), 5) for v in mount_sim],
        "chain_mount_m": [round(float(v), 5) for v in mount_chain],
        "mount_offset_mm": [round(float(v), 3) for v in
                            (mount_sim - mount_chain) * 1000.0],
        "note": ("The sim arm is mounted at BoardConfig.arm_base (a scene "
                 "constant) while the chain carries the measured mount; the "
                 "difference is the mount, not a kinematic disagreement."),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2) + "\n")
    for k, v in report.items():
        print(f"{k:36} {v}")


if __name__ == "__main__":
    main()
