#!/usr/bin/env python3
"""Block 5b safety + visibility pre-check: every cal target, in sim, before motion.

For each of the 37 documented ``_cal`` joint vectors it reports

  * the per-joint delta from the pose the arm holds now and whether every joint
    is inside the bridge's own limits,
  * the minimum fingertip height over the board plane along a densified
    joint-space path (current pose -> target, then target -> next target),
  * the camera height over the marker plane and the marker count the sim render
    actually detects at that pose (portrait 480x640, baked fovy),
  * the camera-to-marker-plane range implied by the IRL marker sides versus the
    sim's own camera position, which is the number the FOV fit depends on.

Nothing here commands the arm.  It refuses (exit 2) if any path point is below
the 15 mm floor.
"""
from __future__ import annotations

import json
import math
import sys
import urllib.request
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
from chess_sim import config, fiducials, scene  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
W, H = 480, 640
FLOOR_MM = 15.0
BRIDGE = "http://192.168.1.167:8100"
TIP_Z_M = -0.11045          # renderer/tool_reference fingertip plane in the gripper body
MEASURED_F = 831.9          # measured lens fx (portrait-rotated: fy=831.897... use fx)
MARKER_MM = 20.0


def get(path: str) -> dict:
    token = (ROOT / "live/bridge_token.txt").read_text().strip()
    req = urllib.request.Request(BRIDGE + path, headers={"X-Bridge-Token": token})
    with urllib.request.urlopen(req, timeout=20) as r:
        return json.loads(r.read().decode())


def cal_vectors() -> list[tuple[str, dict]]:
    out = []
    for item in sorted((ROOT / "live/captures").glob("*_cal.json")):
        rec = json.loads(item.read_text())
        out.append((rec["name"], {k: float(rec["joints_deg"][k]) for k in JOINTS}))
    return out


def tip_z(data, model, body_id: int, offset_m: float) -> float:
    r = data.xmat[body_id].reshape(3, 3)
    return float(data.xpos[body_id][2] + (r @ np.array([0.0, 0.0, offset_m]))[2])


def main() -> int:
    state = get("/state")
    limits = state["limits"]
    here = {k: float(state["joints"][k]) for k in JOINTS}
    print("arm now:", {k: round(v, 2) for k, v in here.items()})
    print("bridge limits:", {k: v for k, v in limits.items()})
    print("mode", state["mode"], "armed", state["armed"], "last_seq", state["last_seq"])

    board = json.loads((ROOT / "live/board.json").read_text())
    plane_z = board["z_mm"] / 1e3
    bc = config.BoardConfig(face="charuco",
                            origin=(board["x_mm"] / 1e3, board["y_mm"] / 1e3),
                            yaw=math.radians(board["yaw_deg"]), mat=0.0,
                            table_top=plane_z - 0.0032)
    model = scene.build_scene(board=bc,
                              appearance=config.AppearanceConfig()).compile()
    data = mujoco.MjData(model)
    cid = model.camera("wrist").id
    fovy = float(model.cam_fovy[cid])
    f = H / 2 / math.tan(math.radians(fovy) / 2)
    gid = model.body("so101:gripper").id
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    renderer = mujoco.Renderer(model, height=H, width=W)

    def pose(q):
        for n, v in q.items():
            data.qpos[model.jnt_qposadr[model.joint("so101:" + n).id]] = math.radians(v)
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera="wrist", scene_option=opt)
        frame = renderer.render().copy()
        return frame

    def sim_q(operator: dict) -> dict:
        q = dict(operator)
        q["wrist_roll"] = config.sim_wrist_roll(operator["wrist_roll"])
        return q

    vectors = cal_vectors()
    print(f"\n{len(vectors)} cal vectors; fovy {fovy:.3f} deg -> f {f:.2f} px; "
          f"board plane z {1000 * plane_z:.2f} mm")

    # IRL reference sides at the pose the arm is holding (already captured).
    irl_sides = []
    ref = ROOT / "reports/b5b/irl_reference.jpg"
    if ref.exists():
        irl = cv2.rotate(cv2.imread(str(ref)), cv2.ROTATE_90_CLOCKWISE)
        for d in fiducials.detect_markers(cv2.cvtColor(irl, cv2.COLOR_BGR2RGB)):
            c = np.asarray(d.corners, float).reshape(4, 2)
            irl_sides.append(float(np.mean([np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)])))
    if irl_sides:
        mean_side = float(np.mean(irl_sides))
        print(f"IRL marker side now {mean_side:.2f} px -> implied camera range "
              f"{1000 * MARKER_MM / 1e3 * MEASURED_F / mean_side:.1f} mm")

    rows = []
    worst = (1e9, None, None)
    prev = here
    for name, op in vectors:
        q = sim_q(op)
        # limits delta
        out_of = [n for n in JOINTS if n != "gripper"
                  and not (limits[n][0] <= op[n] <= limits[n][1])]
        # path current/prev -> target
        path_min, path_at = 1e9, None
        for a, b in ((prev, op),):
            steps = max(2, int(max(abs(b[n] - a[n]) for n in JOINTS) / 1.0) + 1)
            for i in range(steps + 1):
                t = i / steps
                p = {n: a[n] + t * (b[n] - a[n]) for n in JOINTS}
                sq = sim_q(p)
                pose(sq)
                z = min(tip_z(data, model, gid, TIP_Z_M), float(data.xpos[gid][2]))
                if z < path_min:
                    path_min, path_at = z, p
        frame = pose(q)
        dets = fiducials.detect_markers(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        sides = []
        for d in dets:
            c = np.asarray(d.corners, float).reshape(4, 2)
            sides.append(float(np.mean([np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)])))
        cam_above = 1000 * (float(data.cam_xpos[cid][2]) - plane_z)
        rows.append({"name": name, "joints_deg": op, "out_of_limits": out_of,
                     "path_min_tip_mm": round(1000 * (path_min - plane_z), 1),
                     "target_tip_mm": round(1000 * (tip_z(data, model, gid, TIP_Z_M) - plane_z), 1),
                     "cam_above_mm": round(cam_above, 1),
                     "markers": len(dets), "sides": [round(s, 1) for s in sides]})
        if path_min - plane_z < worst[0]:
            worst = (path_min - plane_z, name, path_at)
        prev = op
        print(f"{name}  path_min_tip {rows[-1]['path_min_tip_mm']:7.1f} mm  "
              f"target_tip {rows[-1]['target_tip_mm']:7.1f}  cam {cam_above:6.1f}  "
              f"n {len(dets)}  sides {rows[-1]['sides']}"
              + (f"  OUT OF LIMITS {out_of}" if out_of else ""))

    renderer.close()
    print(f"\nworst path tip over board: {1000 * worst[0]:.1f} mm at {worst[1]} "
          f"(floor {FLOOR_MM} mm) -> {'REFUSE' if 1000 * worst[0] < FLOOR_MM else 'OK'}")
    out = ROOT / "reports/b5b/simcheck.json"
    out.write_text(json.dumps({"fovy_deg": fovy, "f_px": f,
                               "board_plane_z_mm": 1000 * plane_z,
                               "floor_mm": FLOOR_MM, "rows": rows}, indent=2) + "\n")
    print("wrote", out)
    return 2 if 1000 * worst[0] < FLOOR_MM else 0


if __name__ == "__main__":
    raise SystemExit(main())
