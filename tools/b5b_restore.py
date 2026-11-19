#!/usr/bin/env python3
"""Return the arm to the reference pose it was holding before Block 5b.

Commands the documented reference joint vector (operator degrees) through the
bridge in REMOTE, with monotonic seqs above the bridge's last_seq, and leaves
the bridge in REMOTE.
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path("/home/brian/robot-arm-sim")
BRIDGE = "http://192.168.1.167:8100"
TOKEN = (ROOT / "live/bridge_token.txt").read_text().strip()
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
REF = {"shoulder_pan": -0.44, "shoulder_lift": -55.25, "elbow_flex": 74.59,
       "wrist_flex": 38.99, "wrist_roll": 0.0, "gripper": 39.39}


def http(method, path, body=None, timeout=15):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BRIDGE + path, data=data, method=method,
                                 headers={"X-Bridge-Token": TOKEN,
                                          "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def main() -> int:
    st = http("GET", "/state")
    print("mode", st["mode"], "joints", st["joints"])
    if st["mode"] != "REMOTE":
        http("POST", "/mode", {"mode": "REMOTE"})
        for _ in range(100):
            time.sleep(0.1)
            st = http("GET", "/state")
            if st["mode"] == "REMOTE":
                break
    if not st["armed"]:
        print(http("POST", "/arm", {"confirm": True}))
    seq = int(http("GET", "/state")["last_seq"])
    t0 = time.time()
    while True:
        seq += 1
        try:
            http("POST", "/command", {"v": 1, "seq": seq, "ttl_ms": 300,
                                      "joints": [float(REF[n]) for n in JOINTS],
                                      "speed_scale": 0.2})
        except urllib.error.HTTPError as exc:
            if exc.code == 409:
                seq = int(http("GET", "/state")["last_seq"])
                time.sleep(0.05)
                continue
            raise
        st = http("GET", "/state")
        if st["mode"] != "REMOTE" or st["fault"]:
            print("stopped:", st["mode"], st["fault"])
            return 1
        err = max(abs(float(st["joints"][n]) - REF[n]) for n in JOINTS[:5])
        if err <= 0.6 or time.time() - t0 > 25:
            print(f"settled: err {err:.2f} deg after {time.time() - t0:.1f} s; joints {st['joints']}")
            return 0
        time.sleep(0.1)


if __name__ == "__main__":
    raise SystemExit(main())
