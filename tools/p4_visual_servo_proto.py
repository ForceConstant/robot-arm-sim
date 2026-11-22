#!/usr/bin/env python3
"""Mount-free close-and-servo prototype (2026-11-22).

Design: the square/piece coordinates come from marker PnP alone (Board.solve),
which is independent of the suspect camera-in-tool mount. We command the arm in
board axes with `rel` moves to drive the piece onto the target square.

This module contains the SERVO CORE only: it takes a detected piece position
and a target position in board millimetres and returns the next `rel` command.
It is deliberately mount-free and hand-eye-free. Running it needs a detection
source (TargetDetector) and a motion source (live /move rel), which the harness
below wires up in simulation first.

Servo law (position-based, damped):
    error = target_mm - piece_mm
    step  = clamp(gain * error, max_step_mm)
    command rel dx_mm,dy_mm in BOARD axes (z held: approach handled separately)

Stop when |error| <= tolerance_mm. Abort if no progress after N steps or if the
detector abstains (no board pose / no piece). No gripper actuation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ServoState:
    target_xy_mm: np.ndarray
    tolerance_mm: float = 3.0
    gain: float = 0.6
    max_step_mm: float = 20.0
    max_steps: int = 12
    stalled_limit: int = 2


def servo_step(state: ServoState, piece_xy_mm: np.ndarray | None):
    """Return (command, done, info). command is {'dx_mm', 'dy_mm'} in board axes.

    piece_xy_mm: piece centroid in board mm from the mount-free detector.
    Returns done=True when within tolerance. Returns command=None on abstain.
    """
    if piece_xy_mm is None:
        return None, False, {"reason": "no piece detected"}
    err = np.asarray(state.target_xy_mm, float) - np.asarray(piece_xy_mm, float)
    dist = float(np.linalg.norm(err))
    if dist <= state.tolerance_mm:
        return None, True, {"reason": "converged", "error_mm": round(dist, 2)}
    step = np.clip(state.gain * err, -state.max_step_mm, state.max_step_mm)
    return ({"dx_mm": round(float(step[0]), 2),
             "dy_mm": round(float(step[1]), 2)},
            False, {"error_mm": round(dist, 2),
                    "step_mm": round(float(np.linalg.norm(step)), 2)})


def simulate_detector_noise(piece_xy_mm, sigma_mm=1.0, rng=None):
    """Synthetic detector reading for unit-testing the servo law offline."""
    rng = rng or np.random.default_rng(0)
    return np.asarray(piece_xy_mm, float) + rng.normal(0, sigma_mm, 2)


def run_core_test():
    """Pure-math test of the servo law against a synthetic mount-free detector."""
    rng = np.random.default_rng(7)
    results = []
    for trial in range(5):
        target = np.array([14.0, -14.0])          # e5-like square centre
        piece = target + rng.uniform(-25, 25, 2)  # start within view
        state = ServoState(target_xy_mm=target, tolerance_mm=3.0)
        errs = []
        for k in range(state.max_steps):
            obs = simulate_detector_noise(piece, sigma_mm=0.8, rng=rng)
            cmd, done, info = servo_step(state, obs)
            if done:
                break
            # simulate the arm moving by the commanded board-frame delta
            piece = piece + np.array([cmd["dx_mm"], cmd["dy_mm"]])
            errs.append(info["error_mm"])
        results.append({"trial": trial, "steps": len(errs),
                        "final_error_mm": errs[-1] if errs else 0.0,
                        "converged": len(errs) < state.max_steps})
    for r in results:
        print(f"trial {r['trial']}: steps {r['steps']} final {r['final_error_mm']:.2f}mm "
              f"converged={r['converged']}")
    assert all(r["converged"] for r in results), "servo law failed to converge"
    print("servo core test: PASS")


if __name__ == "__main__":
    run_core_test()
