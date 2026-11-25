#!/usr/bin/env python3
"""Step 1 discriminator: is the settle shortfall servo-side or mechanical?

Commands a step on one joint and compares, at settle:
  commanded goal  (what we asked for)
  Present_Position (what the servo reports it reached)
  external link   (not measured here - the FK tip is derived from Present, so it
                   cannot answer this; see the note at the end)

If Present lands ~= goal while the *external* arm is short -> slop downstream of
the sensor (horn / printed adapter / gear mesh).
If Present lands short of goal with non-zero load -> the servo loop itself is
holding short (P-gain droop / static friction).

Read-only with respect to configuration: no register writes, small moves only.
"""
import json
import statistics
import sys
import time
import urllib.error
import urllib.request

B = "http://192.168.1.167:8100"
TOK = open("/home/brian/robot-arm-sim/live/bridge_token.txt").read().strip()
NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def state():
    with urllib.request.urlopen(B + "/state", timeout=6) as reply:
        return json.load(reply)


def send(target: dict, seq: int) -> dict:
    body = {"v": 1, "seq": seq, "ttl_ms": 400, "speed_scale": 0.25,
            "joints": [target[name] for name in NAMES]}
    request = urllib.request.Request(
        B + "/command", data=json.dumps(body).encode(), method="POST",
        headers={"Content-Type": "application/json", "x-bridge-token": TOK})
    try:
        with urllib.request.urlopen(request, timeout=10) as reply:
            return json.load(reply)
    except urllib.error.HTTPError as exc:
        return {"http_error": exc.code, "detail": exc.read().decode()[:160]}


def step(joint: str, delta: float, seconds: float = 2.4, settle_s: float = 1.6) -> dict:
    """Command one joint by delta and watch it settle. Returns goal vs reached."""
    start = state()["joints"]
    target = dict(start)
    target[joint] = start[joint] + delta
    seq = int(time.time() * 1000)
    deadline = time.time() + seconds
    while time.time() < deadline:
        seq += 1
        send(target, seq)
        time.sleep(0.15)
    # stop commanding: the bridge holds the last accepted goal, so the servo is
    # still being asked for `target` while we watch it settle
    samples = []
    settle_deadline = time.time() + settle_s
    while time.time() < settle_deadline:
        now = state()["joints"]
        samples.append(now[joint])
        time.sleep(0.1)
    reached = samples[-1]
    spread = max(samples[-4:]) - min(samples[-4:])
    return {"joint": joint, "delta": delta,
            "start": round(start[joint], 2),
            "goal": round(target[joint], 2),
            "reached": round(reached, 2),
            "residual": round(target[joint] - reached, 2),
            "travel": round(reached - start[joint], 2),
            "tail_spread": round(spread, 3)}


def main() -> int:
    mode = state().get("mode")
    if mode != "REMOTE":
        print(f"bridge is {mode}, needs REMOTE")
        return 2
    joints = sys.argv[1].split(",") if len(sys.argv) > 1 else ["shoulder_pan", "elbow_flex"]
    results = []
    for joint in joints:
        for delta in (10.0, -10.0):
            for rep in range(3):
                record = step(joint, delta)
                results.append(record)
                print(f"  {joint:14s} {delta:+5.1f}  goal {record['goal']:8.2f}  "
                      f"reached {record['reached']:8.2f}  residual {record['residual']:+6.2f}  "
                      f"tail spread {record['tail_spread']:.3f}", flush=True)
                time.sleep(0.4)
        for delta in (10.0, -10.0):
            rows = [r for r in results if r["joint"] == joint and r["delta"] == delta]
            if rows:
                print(f"    {joint} {delta:+5.1f}: median residual "
                      f"{statistics.median(r['residual'] for r in rows):+.2f} deg "
                      f"(n={len(rows)})", flush=True)
                # put it back roughly where it started
        step(joint, -10.0, seconds=1.6)
        step(joint, 10.0, seconds=1.6)

    print("\ninterpretation:")
    print("  residual near 0 (<=0.2 deg) with the external arm short  -> mechanical slop")
    print("  residual large (1-4 deg) with the servo loaded            -> servo-loop droop")
    print("  opposite signs for +delta vs -delta                       -> hysteresis/backlash")
    print(json.dumps({"results": results}, indent=1)[:200] + " ...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
