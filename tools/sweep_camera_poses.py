"""Stare at all 64 squares, verify the aim from the wrist camera, and record it.

Each transit ends with a camera-pose solve (position + view direction), a render,
a projection check that the target square is near frame centre, and a marker
count.  Results go to JSON so a fast test can assert on them without paying for
64 multi-start solves on every run.
"""
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from chess_sim.config import BoardConfig, EnvConfig, AppearanceConfig
from chess_sim.env import ChessSimEnv
from chess_sim.camera_ik import CameraIK
from chess_sim import fiducials

board = BoardConfig(face="charuco", yaw=math.pi)
env = ChessSimEnv(EnvConfig(board=board, appearance=AppearanceConfig(board_markers=False)))
env.reset("8/8/8/8/8/8/8/8")
m, d = env.model, env.data
cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
H, W = 480, 640
f = 0.5 * H / math.tan(math.radians(float(m.cam_fovy[cam])) / 2.0)
ik = CameraIK(env, "wrist")

SEEDS = [np.radians(v) for v in ([0, -55, 72, 39, 22], [0, -40, 50, 20, 0],
                                 [25, -70, 80, 10, -30], [-25, -60, 65, 30, 30],
                                 [-45, -60, 70, 25, 0], [-70, -55, 65, 20, -20],
                                 [45, -60, 70, 25, 0], [70, -50, 60, 20, 20])]
seed = SEEDS[0]
yaw = board.yaw
Rz = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])

frames, rows = [], []
for rank in range(1, 9):
    for i, file in enumerate("abcdefgh"):
        x = (i + 0.5) * 28.0 - 112.0
        y = (rank - 1 + 0.5) * 28.0 - 112.0
        p2 = Rz @ np.array([x / 1000.0, y / 1000.0])
        point = np.array([board.origin[0] + p2[0], board.origin[1] + p2[1], board.top])

        best = None
        for az in (0.0, 45.0, -45.0, 90.0, -90.0, 135.0, -135.0):
            for dist in (230.0, 200.0, 175.0, 150.0, 130.0, 110.0):
                pos, rot = ik.aim_at(point, dist, pitch_deg=20.0, azimuth_deg=az)
                for candidate in [seed] + SEEDS:
                    t = ik.solve(pos, rot, seed=candidate)
                    score = t["position_error_mm"] + 20.0 * t["aim_error_deg"]
                    if best is None or score < best[0]:
                        best = (score, t, dist)
                    if t["position_error_mm"] < 3 and t["aim_error_deg"] < 4:
                        break
                if best[0] < 3 + 4 * 20 and best[1]["aim_error_deg"] < 4:
                    break
            if best[0] < 3 + 4 * 20 and best[1]["aim_error_deg"] < 4:
                break

        score, sol, dist = best
        ok = sol["position_error_mm"] < 12 and sol["aim_error_deg"] < 6
        offset, markers = None, 0
        if ok:
            seed = sol["q"]
            cpos, crot = ik.camera_pose(sol["q"])
            pt = crot.T @ (point - cpos)
            depth = -pt[2]
            if depth > 1e-6:
                u = f * pt[0] / depth + W / 2
                v = -f * pt[1] / depth + H / 2
                offset = float(np.hypot(u - W / 2, v - H / 2))
            img = env.render("wrist")
            markers = len([q for q in fiducials.detect_markers(img) if 5 <= q.marker_id <= 36])
            frames.append(Image.fromarray(img).resize((320, 240)))
        rows.append((file + str(rank), ok, offset, markers, dist, sol))
        dshow = "?" if dist is None else f"{dist:.0f}"
        cshow = "?" if offset is None else f"{offset:.0f}"
        print(f"{file}{rank}: d{dshow} ik {sol['position_error_mm']:5.1f} mm aim {sol['aim_error_deg']:4.1f}° "
              f"centre {cshow} markers {markers}", flush=True)

report = {"solved": sum(1 for r in rows if r[1]), "total": 64,
          "aimed_within_60px": sum(1 for r in rows if r[1] and r[2] is not None and r[2] < 60),
          "all_have_markers": all(r[3] >= 1 for r in rows if r[1]),
          "squares": [{"square": r[0], "ok": bool(r[1]),
                       "aim_offset_px": None if r[2] is None else round(r[2], 1),
                       "markers": int(r[3]), "standoff_mm": None if r[4] is None else round(r[4], 1),
                       "position_error_mm": round(r[5]["position_error_mm"], 2),
                       "aim_error_deg": round(r[5]["aim_error_deg"], 2),
                       "joints_deg": [round(float(np.degrees(v)), 2) for v in r[5]["q"]]}
                      for r in rows]}
open("/tmp/camera_sweep.json", "w").write(json.dumps(report, indent=2) + "\n")
done = [r for r in rows if r[1]]
centred = [r for r in done if r[2] is not None and r[2] < 60]
print(f"\nsolved {len(done)}/64   aimed within 60 px {len(centred)}/{len(done) or 1}")
print("failed:", [r[0] for r in rows if not r[1]])
if frames:
    frames[0].save("/tmp/stare64.gif", save_all=True, append_images=frames[1:], duration=170, loop=0)
    print("gif frames", len(frames))
env.close()
