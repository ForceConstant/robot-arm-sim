#!/usr/bin/env python3
"""Honest landing residuals -> a proposed ``move_tolerance_mm``.

``move_tolerance_mm`` is compared against the solver's own reported
``position_error_mm``.  Both sides of that comparison are FK against FK, so the
gate is blind to two things by construction:

* the ~18.7 mm **model bias** documented in ``reports/T2_frame_refit_20261123.md``
  — the arm lands 19 mm off the board point while the solver reports 0.3 mm;
* **hardware scatter** — servo quantisation and settle tolerance, which move the
  real tip after the solve has finished.

A 20 mm gate on the solver residual therefore launders the bias: it passes a move
that lands 19 mm off, and it is so loose it would also pass a *bad solve*.
Phase 1's job is to say what the gate should be and, just as important, what
still needs its own check.

The three terms, and where each one belongs
-------------------------------------------
===================  ==========================  ==============================
term                 size on this 64-square run   who can see it
===================  ==========================  ==============================
solver residual      mink p50 0.34 / p99 1.47 mm  ``move_tolerance_mm``
model bias           18.7 mm (T2 refit)           a physical check, Phase 2
hardware scatter     ~8 mm p95 at 0.7 deg settle  the settle tolerance, not a gate
===================  ==========================  ==============================

So the proposal splits in two:

* **``move_tolerance_mm`` = a bound on the SOLVER.**  Set from the mink sweep's
  p99 with headroom.  Below it, a solve is bad; above it, hardware.
* **physical landing accuracy needs a marker read-back** and cannot be gated by
  any FK-internal number.  This receipt proposes the value and says plainly that
  tightening past ~19 mm would refuse *every* move until the bias is removed —
  which is why the gate must bound the solver only, and the bias must be fixed
  separately.

    robot-arm-sim python tools/phase1_gate.py \
        --squares reports/phase1_squares.json --out reports/phase1_landing.json
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

# STS3215: 4096 counts over 360 deg.
SERVO_STEP_DEG = 360.0 / 4096.0
# Console default settle tolerance (``settle_deg``).
SETTLE_DEG = 0.7
# Documented physical model bias, from the T2 refit (mean over 57 views).
MODEL_BIAS_MM = 18.74


def _percentile(values, p):
    values = sorted(float(v) for v in values)
    if not values:
        return 0.0
    k = (len(values) - 1) * p / 100.0
    lo = int(math.floor(k))
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (k - lo)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--squares", default="reports/phase1_squares.json")
    parser.add_argument("--live-dir", default=str(ROOT / "live"))
    parser.add_argument("--samples", type=int, default=400)
    parser.add_argument("--out", default="reports/phase1_landing.json")
    args = parser.parse_args()

    from kinematics import ArmKinematics
    from tip_frame import TIP_IN_CHAIN
    from chess_sim.wrist_calibration import WristCalibration

    live_dir = Path(args.live_dir)
    config = json.loads((live_dir / "live_config.json").read_text())
    calibration = WristCalibration.load(str(ROOT / config.get(
        "calibration", ".private/phase11d/calibration.json")))
    chain = ArmKinematics(calibration, {}, np.eye(4))
    jaw = TIP_IN_CHAIN

    squares = json.loads(Path(args.squares).read_text())
    rng = np.random.default_rng(20261124)

    solver = {"least_squares": [], "mink": []}
    detail = []
    hardware = {"quantisation": [], "settle": []}
    for square in squares["squares"]:
        for key in ("least_squares", "mink"):
            q = [square[key]["joints"][n] for n in
                 ("shoulder_pan", "shoulder_lift", "elbow_flex",
                  "wrist_flex", "wrist_roll", "gripper")]
            solver[key].append(square[key]["mm"])
            detail.append({"label": f"{square['square']}/{key}",
                           "mm": square[key]["mm"], "deg": square[key]["deg"]})
            nominal = (chain.fk(q) @ jaw)[:3, 3]
            for bucket, sigma in (("quantisation", SERVO_STEP_DEG),
                                  ("settle", SETTLE_DEG)):
                noise = rng.normal(0.0, sigma, size=(args.samples, 5))
                spread = []
                for row in noise:
                    perturbed = list(q)
                    for i in range(5):
                        perturbed[i] += float(row[i])
                    tip = (chain.fk(perturbed) @ jaw)[:3, 3]
                    spread.append(float(np.linalg.norm(tip - nominal) * 1000.0))
                hardware[bucket].append(float(np.percentile(spread, 95)))

    mink = solver["mink"]
    ls = solver["least_squares"]
    mink_p99 = _percentile(mink, 99)
    mink_max = max(mink)
    # Bound the SOLVER: p99 with 2x headroom, rounded up to 0.5 mm.  The max
    # observed (1.49 mm) sits inside it, so a valid solve is never refused.
    proposed = round(math.ceil(max(mink_p99, mink_max) * 2.0 * 2) / 2.0, 1)

    # What a gate at each candidate value would admit, per solver.  This is the
    # evidence that tightening only makes sense once mink is the solver.
    admission = []
    for gate in (2.0, 2.5, 3.0, 4.0, 5.0, 8.0, 12.0, 20.0):
        admission.append({"gate_mm": gate,
                          "mink_pass": sum(1 for v in mink if v <= gate),
                          "least_squares_pass": sum(1 for v in ls if v <= gate)})

    hw_settle_p95 = float(np.median(hardware["settle"]))
    hw_quant_p95 = float(np.median(hardware["quantisation"]))
    honest_p50 = math.hypot(_percentile(mink, 50), MODEL_BIAS_MM)
    honest_p95 = math.hypot(_percentile(mink, 95), MODEL_BIAS_MM, hw_settle_p95)

    report = {
        "tool": "tools/phase1_gate.py",
        "squares": len(squares["squares"]),
        "current_gate_mm": 20.0,
        "solver_residual_mm": {
            "least_squares": {"p50": round(_percentile(ls, 50), 2),
                              "p90": round(_percentile(ls, 90), 2),
                              "p95": round(_percentile(ls, 95), 2),
                              "max": round(max(ls), 2)},
            "mink": {"p50": round(_percentile(mink, 50), 2),
                     "p90": round(_percentile(mink, 90), 2),
                     "p95": round(_percentile(mink, 95), 2),
                     "p99": round(mink_p99, 2), "max": round(mink_max, 2)},
        },
        "model_bias_mm": MODEL_BIAS_MM,
        "hardware_residual_mm": {
            "servo_quantisation_sigma_deg": round(SERVO_STEP_DEG, 4),
            "servo_quantisation_p95_median_mm": round(hw_quant_p95, 2),
            "settle_sigma_deg": SETTLE_DEG,
            "settle_p95_median_mm": round(hw_settle_p95, 2),
        },
        "honest_landing_mm": {"p50": round(honest_p50, 1),
                              "p95": round(honest_p95, 1)},
        "gate_admission": admission,
        "proposed_gate_mm": float(proposed),
        "proposed_percentile": 99,
        "derivation": ("max(solver p99, solver max) x 2 headroom, rounded up to "
                       "0.5 mm. Bounds the SOLVER only: the model bias and "
                       "hardware scatter are invisible to an FK-vs-FK gate."),
        "blocked_on": ("a physical landing check (marker read-back) for the 18.7 "
                       "mm model bias; and, for the gate to be *usable*, the mink "
                       "solver must be the one running (least_squares cannot meet "
                       f"{proposed} mm: {admission[2]['least_squares_pass']}/64 "
                       f"at {admission[2]['gate_mm']} mm)"),
        "warning": (f"Any gate below {math.ceil(MODEL_BIAS_MM)} mm refuses EVERY "
                    "move until the bias is fixed, if it is ever compared against "
                    "physical landing rather than solver residual."),
        "samples_detail": detail,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2) + "\n")

    print(f"current gate            {report['current_gate_mm']} mm")
    print(f"solver residual  mink   p50 {report['solver_residual_mm']['mink']['p50']}  "
          f"p95 {report['solver_residual_mm']['mink']['p95']}  "
          f"p99 {report['solver_residual_mm']['mink']['p99']}  max {mink_max:.2f} mm")
    print(f"solver residual  LS     p50 {report['solver_residual_mm']['least_squares']['p50']}  "
          f"max {report['solver_residual_mm']['least_squares']['max']} mm")
    print(f"model bias              {MODEL_BIAS_MM} mm   (invisible to the gate)")
    print(f"hardware scatter        quant {hw_quant_p95:.2f}  settle {hw_settle_p95:.2f} mm p95")
    print(f"honest landing          p50 {honest_p50:.1f}  p95 {honest_p95:.1f} mm")
    print(f"PROPOSED gate           {proposed} mm  (solver bound)")
    print("admission:")
    for row in admission:
        print(f"  {row['gate_mm']:5.1f} mm -> mink {row['mink_pass']:2}/64  "
              f"LS {row['least_squares_pass']:2}/64")
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
