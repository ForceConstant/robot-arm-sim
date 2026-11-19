#!/usr/bin/env python3
"""Where is the real camera relative to the marker plane?

Marker sides give a range per marker (range = marker_mm * f / side_px) at the
measured lens.  With the board-frame marker centres known, fit the camera's
in-plane position and height above the plane that explain those ranges, and
print it next to the sim's own camera height at the same joint vector.

This is the number behind "the sim camera is 13.4% too close": it is derived
from the image alone, with no mount and no chain.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from handeye_bundle import MARKER_MM, board_corners  # noqa: E402

LENS_F = 832.0     # measured fy (px); fx/fy agree to 0.1%


def main() -> int:
    d = json.loads((ROOT / "reports/b5b/reference_fovy.json").read_text())
    irl = d["irl"]
    ids = sorted(int(k) for k in irl)
    centres = np.array([board_corners(i).mean(axis=0)[:2] for i in ids])
    sides = np.array([irl[str(i)]["side_px"] for i in ids])
    ranges = MARKER_MM / 1000.0 * LENS_F / sides

    def residual(p):
        x0, y0, h = p
        return np.hypot(centres[:, 0] - x0, centres[:, 1] - y0) ** 2 + h ** 2 - ranges ** 2

    sol = least_squares(residual, [0.0, 0.0, 0.17],
                        bounds=([-0.4, -0.4, 0.05], [0.4, 0.4, 0.5]))
    x0, y0, h = sol.x
    print("marker   side_px   range_mm")
    for i, s, r in zip(ids, sides, ranges):
        print(f"  {i:2d}    {s:7.2f}   {1000 * r:7.1f}")
    print(f"\nfit camera (board frame): x {1000 * x0:+.1f} mm  y {1000 * y0:+.1f} mm  "
          f"height {1000 * h:.1f} mm   (range rms {1000 * np.sqrt(np.mean(sol.fun ** 2)):.1f} mm)")
    sim_h = float(d["camera_above_plane_mm"])
    print(f"sim camera height at the same joints: {sim_h:.1f} mm")
    print(f"difference: {1000 * h - sim_h:+.1f} mm  "
          f"(ratio {sim_h / (1000 * h):.4f})")
    out = ROOT / "reports/b5b/range_check.json"
    out.write_text(json.dumps({
        "lens_f_px": LENS_F, "marker_mm": MARKER_MM,
        "per_marker": [{"id": int(i), "side_px": float(s), "range_mm": round(1000 * float(r), 1)}
                       for i, s, r in zip(ids, sides, ranges)],
        "irl_camera_board_frame": {"x_mm": round(1000 * float(x0), 1),
                                   "y_mm": round(1000 * float(y0), 1),
                                   "height_mm": round(1000 * float(h), 1)},
        "sim_camera_above_plane_mm": sim_h,
        "difference_mm": round(1000 * float(h) - sim_h, 1),
        "sim_over_irl_range": round(sim_h / (1000 * float(h)), 4),
    }, indent=2) + "\n")
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
