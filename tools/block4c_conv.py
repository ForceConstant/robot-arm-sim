"""Which convention does the stored camera_in_tool actually satisfy?

Project the solved board through CameraCalibration.project() (OpenGL: -Z view)
for (a) the stored matrix as-is and (b) the stored matrix flipped by
diag(1,-1,-1), and report pixel error against the real detections.  Only one
convention can explain the data; that fixes what has to be written in the file.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from chess_sim import fiducials
from chess_sim.vision_geometry import CameraCalibration
from chess_sim.wrist_calibration import WristCalibration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handeye_bundle import board_corners, chain_fk
from block4c_mount import board_from_params, pose_from_params

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
FIRST, LAST = 5, 36
FLIP = np.diag([1.0, -1.0, -1.0])
FLIP4 = np.diag([1.0, -1.0, -1.0, 1.0])

cal = WristCalibration.load(".private/phase11d/calibration.json")
mount = json.loads(Path("reports/block4c_mount.json").read_text())
board = mount["board"]
board_t = np.eye(4)
theta = np.radians(board["yaw_deg"])
board_t[:3, :3] = [[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]]
board_t[:3, 3] = [board["x_mm"] / 1000, board["y_mm"] / 1000, board["z_mm"] / 1000]

variants = {
    "stored_as_GL": np.asarray(cal.camera_in_tool, float),
    "stored_flipped_GL": np.asarray(cal.camera_in_tool, float) @ FLIP4,
    "solved_GL": np.asarray(mount["camera_in_tool_gl"], float),
}

for name, x in variants.items():
    errors = []
    behind = 0
    for item in sorted(Path("live/captures").glob("*_cal.json")):
        record = json.loads(item.read_text())
        jpg = item.with_suffix(".jpg")
        image = cv2.imread(str(jpg)) if jpg.exists() else None
        if image is None:
            continue
        detected = {d.marker_id: np.asarray(d.corners, float)
                    for d in fiducials.detect_markers(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))}
        detected = {m: c for m, c in detected.items() if FIRST <= m <= LAST}
        if not detected:
            continue
        base_tool = chain_fk(cal, [float(record["joints_deg"][n]) for n in JOINTS])
        world_cam = base_tool @ x
        camera = CameraCalibration(640, 480, 831.897, 833.014, 320.0, 240.0,
                                   tuple(world_cam[:3, 3]), tuple(world_cam[:3, :3].ravel()))
        for marker_id, corners in detected.items():
            local = board_corners(marker_id)
            world = (board_t[:3, :3] @ local.T).T + board_t[:3, 3]
            for point, pixel in zip(world, corners):
                projected = camera.project(point)
                if projected is None:
                    behind += 1
                    continue
                errors.append(np.linalg.norm(np.asarray(projected) - pixel))
    errors = np.asarray(errors) if errors else np.array([np.nan])
    print(f"{name:18s} n {errors.size:4d} behind {behind:3d} "
          f"median {np.median(errors):8.2f} p90 {np.percentile(errors,90):8.2f} max {errors.max():8.2f}")
