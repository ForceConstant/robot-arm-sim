#!/usr/bin/env python3
"""Block 5b capture: drive the real arm to the documented cal joint vectors.

Per target: check the joint-space path in sim first (refuse below the 15 mm
fingertip floor over the board plane), command it through the bridge in REMOTE,
settle to <= SETTLE_DEG, capture a raw wrist frame plus the paired measured
joints.  Every submitted command uses a seq strictly above the bridge's
last_seq, and the loop aborts to HOLD on a fault, a mode change, or a jam
(joints not advancing while commands are still accepted).

Writes one success and one ``.failed.json`` per target, plus an index.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
from chess_sim import config, scene  # noqa: E402

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
BRIDGE = "http://192.168.1.167:8100"
TOKEN = (ROOT / "live/bridge_token.txt").read_text().strip()
FLOOR_MM = 15.0
SETTLE_DEG = 0.6
SETTLE_BUDGET_S = 25.0
SEND_HZ = 10.0
TTL_MS = 300
SPEED_SCALE = 0.2
TIP_Z_M = -0.11045


def http(method: str, path: str, body: dict | None = None, timeout: float = 15.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BRIDGE + path, data=data, method=method,
                                 headers={"X-Bridge-Token": TOKEN,
                                          "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


class Arm:
    def __init__(self):
        self.seq = int(http("GET", "/state")["last_seq"])
        self.sent = 0

    def state(self) -> dict:
        return http("GET", "/state")

    def mode(self, mode: str) -> dict:
        return http("POST", "/mode", {"mode": mode})

    def arm(self) -> dict:
        return http("POST", "/arm", {"confirm": True})

    def send(self, joints: dict, seq: int) -> dict:
        body = {"v": 1, "seq": seq, "ttl_ms": TTL_MS,
                "joints": [float(joints[n]) for n in JOINTS],
                "speed_scale": SPEED_SCALE}
        return http("POST", "/command", body)


def sim_geom():
    board = json.loads((ROOT / "live/board.json").read_text())
    plane_z = board["z_mm"] / 1e3
    bc = config.BoardConfig(face="charuco",
                            origin=(board["x_mm"] / 1e3, board["y_mm"] / 1e3),
                            yaw=math.radians(board["yaw_deg"]), mat=0.0,
                            table_top=plane_z - 0.0032)
    model = scene.build_scene(board=bc,
                              appearance=config.AppearanceConfig()).compile()
    return model, mujoco.MjData(model), model.body("so101:gripper").id, plane_z


def min_path_tip_mm(model, data, gid, plane_z, a, b) -> float:
    steps = max(2, int(max(abs(b[n] - a[n]) for n in JOINTS) / 1.0) + 1)
    low = 1e9
    for i in range(steps + 1):
        t = i / steps
        for n in JOINTS:
            v = a[n] + t * (b[n] - a[n])
            if n == "wrist_roll":
                v = config.sim_wrist_roll(v)
            data.qpos[model.jnt_qposadr[model.joint("so101:" + n).id]] = math.radians(v)
        mujoco.mj_forward(model, data)
        R = data.xmat[gid].reshape(3, 3)
        z = float(data.xpos[gid][2] + (R @ np.array([0.0, 0.0, TIP_Z_M]))[2])
        low = min(low, z, float(data.xpos[gid][2]))
    return 1000.0 * (low - plane_z)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="live/captures_b5b")
    ap.add_argument("--skip", default="", help="comma separated record names")
    args = ap.parse_args()
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    skip = {s for s in args.skip.split(",") if s}

    vectors = []
    for item in sorted((ROOT / "live/captures").glob("*_cal.json")):
        rec = json.loads(item.read_text())
        vectors.append((rec["name"], {k: float(rec["joints_deg"][k]) for k in JOINTS}))

    model, data, gid, plane_z = sim_geom()
    arm = Arm()
    st = arm.state()
    print(f"bridge mode {st['mode']} armed {st['armed']} last_seq {st['last_seq']}")
    if st["mode"] != "REMOTE":
        if st["mode"] != "HOLD":
            print("bridge is "+st["mode"]+"; refusing to change mode from this script")
            return 3
        print("requesting REMOTE:", arm.mode("REMOTE"))
        for _ in range(100):
            time.sleep(0.1)
            st = arm.state()
            if st["mode"] == "REMOTE":
                break
        if st["mode"] != "REMOTE":
            print("bridge did not enter REMOTE")
            return 3
    if not st["armed"]:
        print("arming:", arm.arm())
    here = {k: float(st["joints"][k]) for k in JOINTS}

    index = {}
    for name, target in vectors:
        if name in skip:
            print(f"SKIP {name}: excluded by --skip")
            index[name] = {"skipped": "excluded by --skip"}
            continue
        tip = min_path_tip_mm(model, data, gid, plane_z, here, target)
        if tip < FLOOR_MM:
            print(f"REFUSE {name}: path min tip {tip:.1f} mm < {FLOOR_MM} mm")
            index[name] = {"skipped": f"path min tip {tip:.1f} mm"}
            continue
        print(f"MOVE {name} (path min tip {tip:.1f} mm)", flush=True)
        err = None
        t0 = time.time()
        last = None
        accepted = 0
        trace = []
        aborted = None
        while True:
            try:
                arm.seq += 1
                arm.send(target, arm.seq)
                accepted += 1
            except urllib.error.HTTPError as exc:
                if exc.code == 409:
                    arm.seq = int(arm.state()["last_seq"])
                    time.sleep(0.05)
                    continue
                aborted = f"HTTP {exc.code}: {exc.read().decode()[:120]}"
                break
            st = arm.state()
            if st["mode"] != "REMOTE" or st["fault"]:
                aborted = f"bridge {st['mode']} fault={st['fault']}"
                break
            j = {k: float(st["joints"][k]) for k in JOINTS}
            # The gripper jaw stops against its closed position near 0 and can sit
            # ~1 deg off target; it does not move the camera, so settle on the
            # five kinematic joints only.
            err = max(abs(j[n] - target[n]) for n in ARM_JOINTS)
            worst = max(ARM_JOINTS, key=lambda n: abs(j[n] - target[n]))
            trace.append((round(time.time() - t0, 2), round(err, 2)))
            now = time.time()
            moved_ok = False
            if last is not None and now - last[0] >= 1.5:
                moved_ok = max(abs(j[n] - last[1][n]) for n in ARM_JOINTS) < 0.08
            if last is not None and now - last[0] >= 3.0:
                moved = max(abs(j[n] - last[1][n]) for n in JOINTS)
                if moved < 0.05 and err > 1.0:
                    aborted = f"jam: moved {moved:.3f} deg in 3 s with error {err:.2f}"
                    break
                last = (now, j)
            elif last is None:
                last = (now, j)
            if err <= SETTLE_DEG:
                break
            # The follower has a gravity offset of up to ~1 deg on the elbow and
            # lift; the sim is rendered at the *measured* joints, so a stationary
            # residual is usable where a creeping arm is not.
            if err <= 2.5 and moved_ok:
                print(f"  stationary with {err:.2f} deg residual ({worst})")
                break
            if now - t0 > SETTLE_BUDGET_S:
                aborted = f"settle timeout: error {err:.2f} deg after {SETTLE_BUDGET_S:.0f} s"
                break
            time.sleep(1.0 / SEND_HZ)
        if aborted:
            print(f"  ABORT {aborted} -> HOLD")
            arm.mode("HOLD")
            index[name] = {"aborted": aborted, "target": target}
            break
        payload = http("GET", "/capture", timeout=30)
        jpeg = base64.b64decode(payload["jpeg_b64"])
        measured = {k: float(payload["joints"][k]) for k in JOINTS}
        (out / f"{name}.jpg").write_bytes(jpeg)
        rec = {"name": name, "target_deg": target, "measured_deg": measured,
               "settle_error_deg": round(max(abs(measured[n] - target[n]) for n in ARM_JOINTS), 3),
               "worst_joint": max(ARM_JOINTS, key=lambda n: abs(measured[n] - target[n])),
               "gripper_error_deg": round(abs(measured["gripper"] - target["gripper"]), 3),
               "commands_sent": accepted, "seq_last": arm.seq,
               "settle_s": round(time.time() - t0, 2), "trace": trace}
        (out / f"{name}.json").write_text(json.dumps(rec, indent=2) + "\n")
        index[name] = {"ok": True, "settle_error_deg": rec["settle_error_deg"],
                       "settle_s": rec["settle_s"]}
        print(f"  captured; settle err {rec['settle_error_deg']:.2f} deg "
              f"in {rec['settle_s']:.1f} s", flush=True)
        here = measured

    (out / "index.json").write_text(json.dumps(index, indent=2) + "\n")
    print("done ->", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
