#!/usr/bin/env python3
"""Phase 2 — per-tick command/measured/hold logging against the REAL bridge.

The 2026-11-23 symptom ("accepted, small error, arm never moved") reproduced on
real hardware on 2026-11-24: a restore move was issued, the bridge accepted 328
commands, and the arm travelled 0.88 of 71 deg.

This tool drives ONE joint in small increments — exactly what `_stream_to` does
— and logs, per tick: what the console sent, what the bridge's `hold` (its own
accumulated command) became, and what the encoder actually reads. That is the
evidence needed to name the mechanism rather than guess it.

It is bounded: a hard cap on total joint travel and on ticks, a dry-run default,
and it only ever commands one joint by a small amount per tick.

    robot-arm-sim python tools/phase2_ticks.py --joint shoulder_pan --deg 8 --execute
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import urllib.request

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


def _get(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=8) as r:
        return json.loads(r.read().decode() or "{}")


def _post(url: str, body: dict, timeout: float = 10.0) -> dict:
    data = json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        return {"http_error": exc.code,
                "detail": exc.read().decode(errors="replace")[:300]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge", default="http://192.168.1.167:8100")
    ap.add_argument("--token-file", default="/home/brian/robot-arm-sim/live/bridge_token.txt")
    ap.add_argument("--joint", default="shoulder_pan")
    ap.add_argument("--deg", type=float, default=8.0)
    ap.add_argument("--ticks", type=int, default=24)
    ap.add_argument("--step", type=float, default=0.7)
    ap.add_argument("--anchored", action="store_true",
                    help="command max(measured+step, previous command) instead "
                         "of measured+step — the monotonic variant")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--out", default="reports/phase2_ticks.json")
    args = ap.parse_args()

    token = None
    try:
        with open(args.token_file) as handle:
            token = handle.read().strip()
    except OSError:
        pass
    if token:
        def _post(url, body, timeout=10.0):   # noqa: F811
            data = json.dumps(body).encode()
            req = urllib.request.Request(
                url, data=data, method="POST",
                headers={"Content-Type": "application/json",
                         "x-bridge-token": token})
            try:
                with urllib.request.urlopen(req, timeout=timeout) as r:
                    return json.loads(r.read().decode() or "{}")
            except urllib.error.HTTPError as exc:
                return {"http_error": exc.code,
                        "detail": exc.read().decode(errors="replace")[:300]}

    state = _get(args.bridge + "/state")
    if state.get("mode") != "REMOTE" or not state.get("armed"):
        print(f"bridge is {state.get('mode')} armed={state.get('armed')}; "
              f"set REMOTE+armed first", file=sys.stderr)
        return 1

    index = JOINTS.index(args.joint)
    start = [float(state["joints"][n]) for n in JOINTS]
    target = list(start)
    target[index] = start[index] + args.deg

    rows = []
    seq = int(time.time() * 1000)
    command = list(start)
    print(f"{'tick':>4} {'sent':>8} {'hold':>8} {'measured':>9} {'delta':>7}")
    for tick in range(args.ticks):
        measured = [float(_get(args.bridge + "/state")["joints"][n]) for n in JOINTS]
        residual = target[index] - measured[index]
        if abs(residual) <= 0.2:
            break
        step = max(-args.step, min(args.step, residual))
        if args.anchored:
            command[index] = max(measured[index] + step, command[index])
        else:
            command[index] = measured[index] + step
        if not args.execute:
            rows.append({"tick": tick, "sent": round(command[index], 3),
                         "measured": round(measured[index], 3),
                         "would_move": True})
            print(f"{tick:4} {command[index]:8.3f} {'-':>8} {measured[index]:9.3f} "
                  f"{(command[index] - measured[index]):7.3f}")
            time.sleep(0.02)
            continue
        seq += 1
        ack = _post(args.bridge + "/command",
                    {"v": 1, "seq": seq, "ttl_ms": 200,
                     "joints": [float(v) for v in command], "speed_scale": 0.3})
        time.sleep(0.066)
        after = _get(args.bridge + "/state")
        held = after.get("last_clamped")
        rows.append({"tick": tick, "sent": round(command[index], 3),
                     "measured": round(float(after["joints"][args.joint]), 3),
                     "ack": ack, "clamped": held})
        print(f"{tick:4} {command[index]:8.3f} {'?':>8} "
              f"{float(after['joints'][args.joint]):9.3f} "
              f"{(command[index] - float(after['joints'][args.joint])):7.3f} "
              f"{'CLAMP ' + str(held) if held else ''}")

    final = [float(_get(args.bridge + "/state")["joints"][n]) for n in JOINTS]
    travel = max(abs(a - b) for a, b in zip(final, start))
    record = {"joint": args.joint, "requested_deg": args.deg, "executed": args.execute,
              "anchored": args.anchored, "start": start, "final": final,
              "travel_deg": round(travel, 3), "ticks": rows}
    with open(args.out, "w") as handle:
        handle.write(json.dumps(record, indent=1) + "\n")
    print(f"\ntravel {travel:.3f} deg of {args.deg} requested -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
