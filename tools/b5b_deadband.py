#!/usr/bin/env python3
"""Deadband / backlash measurement on shoulder_lift at one pose.

Same target, approached from above and from below, three times each.  Records
the settled reported angle (waiting for it to actually stop moving), the drift
over the next 3 s, and the model tip at those joints as an mm indicator.
"""
from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import mujoco
import math
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
from chess_sim import config, scene  # noqa: E402

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
ARM = JOINTS[:5]
TIP = -0.112
TARGET_LIFT = 30.0
ABOVE = 40.0
BELOW = 20.0
BRIDGE = "http://192.168.1.167:8100"
TOKEN = (ROOT / "live/bridge_token.txt").read_text().strip()


def http(method, path, body=None, timeout=15):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BRIDGE + path, data=data, method=method,
                                 headers={"X-Bridge-Token": TOKEN,
                                          "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def main() -> int:
    st = http("GET", "/state")
    base = {k: float(st["joints"][k]) for k in JOINTS}
    print("start:", {k: round(v, 2) for k, v in base.items()})
    if not st["armed"]:
        print("arm:", http("POST", "/arm", {"confirm": True}))
    seq = int(http("GET", "/state")["last_seq"])

    def move(lift: float, label: str) -> dict:
        nonlocal seq
        target = dict(base)
        target["shoulder_lift"] = lift
        t0 = time.time()
        last, still_since, settled = None, None, None
        while True:
            seq += 1
            try:
                http("POST", "/command", {"v": 1, "seq": seq, "ttl_ms": 300,
                                          "joints": [float(target[n]) for n in JOINTS],
                                          "speed_scale": 0.2})
            except urllib.error.HTTPError as exc:
                if exc.code == 409:
                    seq = int(http("GET", "/state")["last_seq"])
                    time.sleep(0.05)
                    continue
                raise
            st = http("GET", "/state")
            if st["mode"] != "REMOTE" or st["fault"]:
                raise SystemExit(f"abort: {st['mode']} {st['fault']}")
            j = {k: float(st["joints"][k]) for k in JOINTS}
            now = time.time()
            if last is not None:
                d = abs(j["shoulder_lift"] - last[1]["shoulder_lift"])
                if d < 0.02:
                    still_since = still_since or now
                    if now - still_since > 1.5:
                        settled = j
                        break
                else:
                    still_since = None
            last = (now, j)
            if now - t0 > 25:
                settled = j
                break
            time.sleep(0.1)
        time.sleep(3.0)
        after = {k: float(v) for k, v in http("GET", "/state")["joints"].items()}
        print(f"  {label:<28} lift {settled['shoulder_lift']:7.2f}  "
              f"after 3 s {after['shoulder_lift']:7.2f}  ({time.time() - t0:.1f} s)")
        return settled

    board = json.loads((ROOT / "live/board.json").read_text())
    plane_z = board["z_mm"] / 1e3
    bc = config.BoardConfig(face="charuco",
                            origin=(board["x_mm"] / 1e3, board["y_mm"] / 1e3),
                            yaw=math.radians(board["yaw_deg"]), mat=0.0,
                            table_top=plane_z - 0.0032)
    model = scene.build_scene(board=bc,
                              appearance=config.AppearanceConfig(camera_mast=False)).compile()
    data = mujoco.MjData(model)
    gid = model.body("so101:gripper").id

    def tipz(q):
        for n in JOINTS:
            v = q[n]
            if n == "wrist_roll":
                v = config.sim_wrist_roll(v)
            data.qpos[model.jnt_qposadr[model.joint("so101:" + n).id]] = math.radians(v)
        mujoco.mj_forward(model, data)
        R = data.xmat[gid].reshape(3, 3)
        return 1000 * float((data.xpos[gid] + R @ np.array([0.0, 0.0, TIP]))[2] - plane_z)

    print("\nphase A: settle at the test target first")
    move(TARGET_LIFT, "initial -> 30")

    rows = []
    for direction, pre in (("above", ABOVE), ("below", BELOW)):
        for rep in range(3):
            move(pre, f"{direction} preposition {pre:.0f} #{rep + 1}")
            j = move(TARGET_LIFT, f"{direction} approach -> 30 #{rep + 1}")
            rows.append((direction, rep + 1, j["shoulder_lift"], tipz(j)))

    print("\nresult")
    for d in ("above", "below"):
        vals = [r[2] for r in rows if r[0] == d]
        tips = [r[3] for r in rows if r[0] == d]
        print(f"  from {d}: lift {min(vals):.2f}..{max(vals):.2f} "
              f"(spread {max(vals) - min(vals):.2f} deg), tip {min(tips):.1f}..{max(tips):.1f} mm")
    a = [r[2] for r in rows if r[0] == "above"]
    b = [r[2] for r in rows if r[0] == "below"]
    print(f"  hysteresis (above mean {sum(a)/len(a):.2f} vs below mean {sum(b)/len(b):.2f}): "
          f"{sum(b)/len(b) - sum(a)/len(a):+.2f} deg")
    print(f"  implied tip difference at 3.7 mm/deg: "
          f"{3.7 * (sum(b)/len(b) - sum(a)/len(a)):+.1f} mm")
    print("final joints:", {k: round(float(v), 2) for k, v in
                            http("GET", "/state")["joints"].items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
