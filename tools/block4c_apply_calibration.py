#!/usr/bin/env python3
"""Write the Block 4c calibration candidate and a human-readable diff.

The candidate is the live file with three things replaced: the lens (measured on
the 106 existing captures, principal point on the sensor centre so the sim can
represent it), the mount (re-solved on the 37 fixed-board cal frames), and the
distortion (refitted with the centred principal point).  Everything else --
chain, limits, size -- is carried over byte for byte from the stored file, so the
diff shows exactly what changed.

Nothing is written to `.private/` unless ``--apply`` is passed; the default only
produces ``reports/block4c_calibration.json``.

    PYTHONPATH=. robot-arm-sim python tools/block4c_apply_calibration.py
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np

LIVE = Path(".private/phase11d/calibration.json")


def rotation_delta(a, b) -> float:
    a = np.asarray(a, float)[:3, :3]
    b = np.asarray(b, float)[:3, :3]
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1.0) / 2.0))))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", default=str(LIVE))
    parser.add_argument("--mount", default="reports/block4c_mount.json")
    parser.add_argument("--out", default="reports/block4c_calibration.json")
    parser.add_argument("--diff", default="reports/block4c_calibration_diff.md")
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    live_path = Path(args.live)
    stored = json.loads(live_path.read_text())
    mount = json.loads(Path(args.mount).read_text())
    lens = mount["lens"]

    candidate = dict(stored)
    candidate["intrinsics"] = [lens["fx"], lens["fy"], lens["cx"], lens["cy"]]
    candidate["distortion"] = lens["distortion"]
    candidate["camera_in_tool"] = mount["camera_in_tool_gl"]
    candidate["calibration_id"] = "handeye-2026-11-18T11-block4c"

    old = np.asarray(stored["camera_in_tool"], float)
    new = np.asarray(candidate["camera_in_tool"], float)
    delta = np.linalg.inv(old) @ new
    lines = [
        "# Block 4c calibration diff",
        "",
        f"stored: `{stored.get('calibration_id')}` -> candidate: `{candidate['calibration_id']}`",
        "",
        "## Lens",
        "",
        "| item | stored | candidate |",
        "|---|---|---|",
        f"| fx | {stored['intrinsics'][0]:.3f} | {candidate['intrinsics'][0]:.3f} |",
        f"| fy | {stored['intrinsics'][1]:.3f} | {candidate['intrinsics'][1]:.3f} |",
        f"| cx | {stored['intrinsics'][2]:.3f} | {candidate['intrinsics'][2]:.3f} |",
        f"| cy | {stored['intrinsics'][3]:.3f} | {candidate['intrinsics'][3]:.3f} |",
        f"| distortion | {stored['distortion']} | {candidate['distortion']} |",
        f"| horizontal FOV | {2*math.degrees(math.atan(320/stored['intrinsics'][0])):.1f} deg "
        f"| {2*math.degrees(math.atan(320/candidate['intrinsics'][0])):.1f} deg |",
        "",
        "## Mount (`camera_in_tool`, OpenGL camera axes in the so101:gripper body frame)",
        "",
        "```",
        "stored",
        np.array2string(old, precision=6, suppress_small=True),
        "candidate",
        np.array2string(new, precision=6, suppress_small=True),
        "```",
        "",
        f"- camera displacement: **{mount['delta_vs_stored']['camera_displacement_mm']:.2f} mm**",
        f"- rotation difference: **{rotation_delta(old, new):.2f} deg**",
        f"- translation change in the tool frame: {mount['delta_vs_stored']['translation_vector_mm']} mm",
        "",
        "The rotation difference is not a 3-D small angle: the stored matrix is the",
        "OpenCV-convention solve with its camera y/z axes flipped, which puts the",
        "board behind the lens for every consumer that reads the file as OpenGL",
        "(see `tools/block4c_conv.py`: stored-as-OpenGL rejects all 264 corners,",
        "the candidate reprojects them at 1.7 px median).",
        "",
        "## Board pose this mount produces",
        "",
        f"- solved: x {mount['board']['x_mm']} mm, y {mount['board']['y_mm']} mm, "
        f"z {mount['board']['z_mm']} mm, yaw {mount['board']['yaw_deg']} deg",
        f"- live/board.json: x -13.74, y -3.57, z 409.5, yaw 7.14",
        f"- delta: {mount['board_delta_vs_live']}",
        "",
        "## Nothing else changed",
        "",
        "`chain`, `width`, `height` are carried over unchanged.",
        "",
    ]
    Path(args.diff).parent.mkdir(parents=True, exist_ok=True)
    Path(args.diff).write_text("\n".join(lines) + "\n")
    Path(args.out).write_text(json.dumps(candidate, indent=2) + "\n")
    print("wrote", args.out, "and", args.diff)
    if args.apply:
        backup = live_path.with_suffix(".json.bak-block4c")
        if not backup.exists():
            shutil.copy2(live_path, backup)
            print("backup ->", backup)
        live_path.write_text(json.dumps(candidate, indent=2) + "\n")
        print("applied ->", live_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
