#!/usr/bin/env python3
"""Block 4c: point the simulated wrist camera at the measured lens and mount.

The scene camera and ``camera_in_tool`` use the same axes (OpenGL camera: +X
right, +Y up, -Z view) and the same parent body (``so101:gripper``), so the
mount written by ``tools/block4c_mount.py`` is used *unchanged*: no conversion
is applied to the pose.  What does need care is:

  * MuJoCo takes ``quat`` as **w x y z** while scipy's ``as_quat`` returns
    **x y z w** -- the ordering is spelled out here rather than transposed by
    eye;
  * MuJoCo has no principal point or distortion, so the only lens parameter it
    can express is ``fovy``.  It is set from the *vertical* focal length,
    ``fovy = 2 atan(H/2 / fy)``, with square pixels.

The camera's visual proxy box keeps its 10 mm offset in front of the lens.

    PYTHONPATH=. robot-arm-sim python tools/block4c_camera.py \
        --mount reports/block4c_mount.json --scene chess_sim/assets/scene/chess_so101.xml
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

WIDTH, HEIGHT = 640, 480


def quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    x, y, z, w = Rotation.from_matrix(np.asarray(matrix, float)).as_quat()
    return np.array([w, x, y, z])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mount", default="reports/block4c_mount.json")
    parser.add_argument("--intrinsics", default="reports/block4c_intrinsics.json")
    parser.add_argument("--scene", default="chess_sim/assets/scene/chess_so101.xml")
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    mount = json.loads(Path(args.mount).read_text())
    lens = json.loads(Path(args.intrinsics).read_text())["models"]["all"]["shared_centre_pp_fixed_focal"]
    fy = float(lens["fy"])
    fovy = 2.0 * math.degrees(math.atan((HEIGHT / 2.0) / fy))

    camera = np.asarray(mount["camera_in_tool_gl"], float)
    position = camera[:3, 3]
    quat = quat_wxyz(camera[:3, :3])

    def fmt(values):
        return " ".join(f"{float(v):.9g}" for v in values)

    camera_line = (f'                <camera name="wrist" quat="{fmt(quat)}" '
                   f'pos="{fmt(position)}" fovy="{fovy:.6g}"/>')
    geometry = position + np.array([0.0, 0.0, 0.010])
    geom_line = (f'                <geom name="wrist_camera" size="0.012 0.012 0.008" '
                 f'pos="{fmt(geometry)}" type="box" contype="0" conaffinity="0" '
                 f'group="1" rgba="0.08 0.08 0.08 1" density="0"/>')

    print("mount (OpenGL, gripper body frame):")
    print(np.round(camera, 6))
    print(f"position m {np.round(position, 6)}  quat wxyz {np.round(quat, 6)}")
    print(f"fovy {fovy:.4f} deg from fy {fy:.3f} px")
    print("rotation check: matrix from quat should equal the mount")
    print(np.round(Rotation.from_quat(np.r_[quat[1:], quat[0]]).as_matrix(), 6))

    scene = Path(args.scene)
    text = scene.read_text()
    if not args.apply:
        print("\n--- would write ---")
        print(camera_line)
        print(geom_line)
        return 0

    backup = scene.with_suffix(".xml.bak-block4c")
    if not backup.exists():
        shutil.copy2(scene, backup)
        print("backup ->", backup)

    text, n_camera = re.subn(r'^\s*<camera name="wrist"[^/]*/>', camera_line, text, flags=re.M)
    text, n_geom = re.subn(r'^\s*<geom name="wrist_camera"[^/]*/>', geom_line, text, flags=re.M)
    if (n_camera, n_geom) != (1, 1):
        raise SystemExit(f"expected one wrist camera and one proxy geom, replaced {n_camera}/{n_geom}")
    scene.write_text(text)
    print("wrote", scene)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
