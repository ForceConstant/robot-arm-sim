#!/usr/bin/env python3
"""Block 5b, step 1: the baked wrist FOV must not move anything but the scale.

Captures the raw wrist frame at the pose the arm is holding, detects the
ChArUco markers on it, renders the sim at the *measured* joints with the FOV
now baked into ``chess_sim.scene`` (no runtime override), and prints ids and
marker sides for both.  The IRL side is compared against the sides recorded in
``reports/sim_vs_irl_matched.json`` as well as the fresh capture.

Numbers only: this tool prints what the detector returns.
"""
from __future__ import annotations

import base64
import json
import math
import urllib.request
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
import sys
sys.path.insert(0, str(ROOT))
from chess_sim import config, fiducials, scene  # noqa: E402

OUT = ROOT / "reports/b5b"
W, H = 480, 640
BRIDGE = "http://192.168.1.167:8100"
TOKEN = (ROOT / "live/bridge_token.txt").read_text().strip()


def get(path: str) -> dict:
    req = urllib.request.Request(BRIDGE + path, headers={"X-Bridge-Token": TOKEN})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode())


def marker_stats(frame_rgb: np.ndarray) -> dict:
    out = {}
    for d in fiducials.detect_markers(frame_rgb):
        c = np.asarray(d.corners, float).reshape(4, 2)
        sides = [float(np.linalg.norm(c[(i + 1) % 4] - c[i])) for i in range(4)]
        out[int(d.marker_id)] = {"side_px": round(float(np.mean(sides)), 2),
                                 "centre": [round(float(v), 1) for v in c.mean(axis=0)]}
    return out


def marked(img_rgb: np.ndarray, label: str) -> np.ndarray:
    out = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    for d in fiducials.detect_markers(img_rgb):
        c = np.asarray(d.corners, float).reshape(4, 2)
        cv2.polylines(out, [np.round(c).astype(int).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        cx, cy = np.round(c.mean(axis=0)).astype(int)
        cv2.putText(out, str(int(d.marker_id)), (int(cx), int(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        side = float(np.mean([np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]))
        cv2.putText(out, f"{side:.1f}px", (int(cx) - 24, int(cy) + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
    cv2.putText(out, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return out


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    state = get("/state")
    payload = get("/capture")
    joints = {k: float(v) for k, v in payload["joints"].items()}
    meta = {k: payload.get(k) for k in ("ts", "frame_ts", "joints_ts") if k in payload}
    jpg = OUT / "irl_reference.jpg"
    jpg.write_bytes(base64.b64decode(payload["jpeg_b64"]))
    print("bridge joints:", {k: round(v, 2) for k, v in joints.items()})
    print("state mode/armed:", state["mode"], state["armed"], "last_seq", state["last_seq"])

    raw = cv2.imread(str(jpg))
    print("IRL raw frame:", raw.shape)
    irl = cv2.rotate(raw, cv2.ROTATE_90_CLOCKWISE)
    irl_rgb = cv2.cvtColor(irl, cv2.COLOR_BGR2RGB)
    irl_stats = marker_stats(irl_rgb)

    board = json.loads((ROOT / "live/board.json").read_text())
    bc = config.BoardConfig(face="charuco",
                            origin=(board["x_mm"] / 1e3, board["y_mm"] / 1e3),
                            yaw=math.radians(board["yaw_deg"]), mat=0.0,
                            table_top=board["z_mm"] / 1e3 - 0.0032)
    model = scene.build_scene(board=bc,
                              appearance=config.AppearanceConfig(camera_mast=False)).compile()
    data = mujoco.MjData(model)
    q = dict(joints)
    q["wrist_roll"] = config.sim_wrist_roll(joints["wrist_roll"])
    for name, value in q.items():
        jid = model.joint("so101:" + name).id
        data.qpos[model.jnt_qposadr[jid]] = math.radians(value)
    cid = model.camera("wrist").id
    baked_fovy = float(model.cam_fovy[cid])
    mujoco.mj_forward(model, data)

    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    renderer = mujoco.Renderer(model, height=H, width=W)
    renderer.update_scene(data, camera="wrist", scene_option=opt)
    sim_rgb = renderer.render().copy()
    renderer.close()
    sim_stats = marker_stats(sim_rgb)
    cv2.imwrite(str(OUT / "sim_reference.png"), cv2.cvtColor(sim_rgb, cv2.COLOR_RGB2BGR))

    # geometry context: camera height above the printed plane at this pose
    cam_z = float(data.cam_xpos[cid][2])
    plane_z = board["z_mm"] / 1e3
    print(f"baked fovy {baked_fovy:.4f} deg -> f = {H / 2 / math.tan(math.radians(baked_fovy) / 2):.2f} px")
    print(f"camera {1000 * (cam_z - plane_z):.1f} mm above the marker plane")

    recorded = json.loads((ROOT / "reports/sim_vs_irl_matched.json").read_text())["irl_stats"]
    print("\nid   IRL side   SIM side   delta   | recorded IRL side")
    for mid in sorted(set(irl_stats) | set(sim_stats)):
        a = irl_stats.get(mid, {}).get("side_px")
        b = sim_stats.get(mid, {}).get("side_px")
        r = recorded.get(str(mid), [None])[0]
        delta = f"{b - a:+.2f}" if (a is not None and b is not None) else "-"
        print(f"{mid:3d}  {str(a):>9}  {str(b):>9}  {delta:>7}  | {r}")

    sheet = np.hstack([marked(irl_rgb, "IRL rot 90cw (fresh)"),
                       marked(sim_rgb, f"SIM fovy {baked_fovy:.2f} baked")])
    cv2.imwrite(str(OUT / "reference_sheet.png"), sheet)
    receipt = {"bridge": {"mode": state["mode"], "armed": state["armed"],
                          "last_seq": state["last_seq"], "joints_deg": joints,
                          "capture_meta": meta},
               "board": board, "baked_fovy_deg": round(baked_fovy, 4),
               "f_px": round(H / 2 / math.tan(math.radians(baked_fovy) / 2), 2),
               "camera_above_plane_mm": round(1000 * (cam_z - plane_z), 2),
               "irl": irl_stats, "sim": sim_stats,
               "recorded_irl_sides": recorded}
    (OUT / "reference_fovy.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print("\nwrote", OUT / "reference_sheet.png", "and", OUT / "reference_fovy.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
