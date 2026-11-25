#!/usr/bin/env python3
"""A/B the backlash take-up primitive on the arm, one joint at a time.

The measurement is the same one the deadband discriminator used, so the numbers
are comparable with ``reports/deadband_*.md``: command a +/-10 deg step, hold
the exact goal at the bridge for 1.5 s, and report

    residual = commanded goal - Present_Position at settle

positive meaning the joint stopped SHORT of the goal.  Holding the goal at the
bridge after the move returns is deliberate: the console's take-up ends the
approach, and this tool then asks the servo for the *same* number the
discriminator asks for, so the only difference between the two conditions is
how the joint got there.

Every rep starts from the same anchor pose (a ``/move mode=joint`` back to it
before the step), which also removes the deadband discriminator's known
artifact where three same-direction reps walk a joint into the elbow reach
corner.

Usage (on the BC-250, with the console and bridge running):

    PYTHONPATH=. python tools/backlash_take_up_test.py \
        --out reports/backlash_take_up_on.json --label take-up-on

    PYTHONPATH=. python tools/backlash_take_up_test.py \
        --out reports/backlash_take_up_off.json --label take-up-off

The take-up gate is read from the console's config at request time, so switch
conditions with ``live_config.json`` (``"backlash_take_up_deg": 0``) and a
``robot-live.service`` restart -- the receipt records which condition it saw in
each move response.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")


class ApiError(RuntimeError):
    def __init__(self, status: int, text: str) -> None:
        self.status = status
        self.text = text
        super().__init__(f"HTTP {status}: {text}")


class Api:
    def __init__(self, base: str, token: str) -> None:
        self.base = base.rstrip("/")
        self.token = token

    def _request(self, method: str, path: str, body=None, timeout: float = 120.0):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.base + path, data=data, method=method)
        if data is not None:
            request.add_header("Content-Type", "application/json")
            request.add_header("x-bridge-token", self.token)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as reply:
                return json.loads(reply.read().decode() or "{}")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode(errors="replace")[:400]
            raise ApiError(exc.code, detail) from None

    def state(self) -> dict:
        return self._request("GET", "/state", timeout=10.0)

    def joints(self) -> list[float]:
        bridge = self.state().get("bridge") or {}
        joints = bridge.get("joints") or {}
        if bridge.get("mode") != "REMOTE" or not bridge.get("armed"):
            raise SystemExit(f"bridge not REMOTE/armed: {bridge.get('mode')}")
        return [float(joints[name]) for name in NAMES]

    def move_joint(self, joint: str, value: float) -> dict:
        return self._request("POST", "/move",
                             {"mode": "joint", "joints": {joint: value}})

    def move_pose(self, pose: list[float]) -> dict:
        # The gripper is deliberately NOT sent: an unspecified joint defaults to
        # the measured pose, so the jaws are held instead of commanded.  (The
        # first version sent all six values and moved the jaws on every anchor
        # return - no grasp, but a jaw motion this tool has no business making.)
        return self._request("POST", "/move",
                             {"mode": "joint",
                              "joints": dict(zip(NAMES[:5], pose[:5]))})

    def jog(self, joint: str, delta: float) -> dict:
        return self._request("POST", "/jog", {"joint": joint, "delta": delta})


class Bridge:
    """Direct /command access for the exact-goal hold (what the discriminator does)."""

    def __init__(self, base: str, token: str) -> None:
        self.base = base.rstrip("/")
        self.token = token
        self.seq = int(time.time() * 1000)

    def state(self) -> dict:
        with urllib.request.urlopen(self.base + "/state", timeout=8) as reply:
            return json.load(reply)


def hold_target(bridge: Bridge, target: list[float], index: int,
                seconds: float = 1.5) -> tuple[float, float]:
    """Hold ``target`` at the bridge, then report the settled value and spread."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        bridge.seq += 1
        body = {"v": 1, "seq": bridge.seq, "ttl_ms": 400, "speed_scale": 0.25,
                "joints": [float(v) for v in target]}
        request = urllib.request.Request(
            bridge.base + "/command", data=json.dumps(body).encode(),
            method="POST",
            headers={"Content-Type": "application/json",
                     "x-bridge-token": bridge.token})
        with urllib.request.urlopen(request, timeout=8):
            pass
        time.sleep(0.15)
    samples = []
    for _ in range(6):
        joints = bridge.state().get("joints") or {}
        samples.append(float(joints[NAMES[index]]))
        time.sleep(0.1)
    spread = max(samples[-4:]) - min(samples[-4:])
    return samples[-1], spread


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--console", default="http://127.0.0.1:8080")
    parser.add_argument("--bridge", default="http://192.168.1.167:8100")
    parser.add_argument("--token-file", default=str(ROOT / "live" / "bridge_token.txt"))
    parser.add_argument("--out", default=str(ROOT / "reports" / "backlash_take_up.json"))
    parser.add_argument("--label", default="unlabelled")
    parser.add_argument("--joints", default="elbow_flex,shoulder_pan")
    parser.add_argument("--delta-deg", type=float, default=10.0)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--hold-s", type=float, default=1.5)
    parser.add_argument("--anchor-pose", default="",
                        help="six comma-separated joint values to start from "
                             "(pan,lift,elbow,flex,roll,gripper); the condition "
                             "runs then start from the same anchor")
    args = parser.parse_args()

    token = Path(args.token_file).read_text().strip()
    api = Api(args.console, token)
    bridge = Bridge(args.bridge, token)

    if args.anchor_pose:
        wanted = [float(v) for v in args.anchor_pose.split(",")]
        if len(wanted) != len(NAMES):
            raise SystemExit("--anchor-pose needs six values")
        try:
            api.move_pose(wanted)
        except ApiError as exc:
            print(f"anchor move refused: {exc}", flush=True)
            raise SystemExit(1)

    anchor = api.joints()
    print(f"label {args.label}")
    print("anchor pose: " + " ".join(f"{n} {v:+.2f}" for n, v in zip(NAMES, anchor)))
    records: list[dict] = []
    take_up_seen: list[dict] = []

    for joint in args.joints.split(","):
        index = NAMES.index(joint)
        print(f"\n{joint}")
        for direction in (1.0, -1.0):
            deltas = []
            for rep in range(args.reps):
                # Same anchor for every rep: removes the accumulating-rep artifact.
                back = api.move_pose(anchor)
                if (back.get("take_up") or {}).get("applied"):
                    take_up_seen.append({"leg": "anchor",
                                         "joint": joint, **(back["take_up"])})
                start_pose = api.joints()
                target = [float(v) for v in start_pose]
                target[index] = start_pose[index] + direction * args.delta_deg
                try:
                    moved = api.move_joint(joint, target[index])
                except ApiError as exc:
                    moved = {"http_error": exc.status, "detail": exc.text}
                    print(f"  {direction:+.0f} rep{rep}  REFUSED {exc}",
                          flush=True)
                reach, spread = hold_target(bridge, list(target), index,
                                            seconds=args.hold_s)
                residual = target[index] - reach
                deltas.append(residual)
                receipt = moved.get("take_up") or {}
                if receipt.get("applied"):
                    take_up_seen.append({"leg": "step", "joint": joint, **receipt})
                records.append({
                    "label": args.label, "joint": joint,
                    "direction": direction, "rep": rep,
                    "goal": round(target[index], 3),
                    "reached": round(reach, 3),
                    "residual": round(residual, 3),
                    "tail_spread": round(spread, 3),
                    "move_error_deg": moved.get("error_deg"),
                    "move_commands": moved.get("commands"),
                    "take_up": receipt,
                    "moved_start": round(start_pose[index], 3),
                })
                print(f"  {direction:+.0f} rep{rep}  goal {target[index]:8.2f}  "
                      f"reached {reach:8.2f}  residual {residual:+6.2f}  "
                      f"spread {spread:.3f}  take-up "
                      f"{'on' if receipt.get('applied') else 'off:' + str(receipt.get('reason'))}",
                      flush=True)
            print(f"  {joint} {direction:+.0f}: median residual "
                  f"{statistics.median(deltas):+.2f} deg  "
                  f"({', '.join(f'{v:+.2f}' for v in deltas)})")

    summary: dict = {"label": args.label, "anchor_pose": dict(zip(NAMES, anchor)),
                     "delta_deg": args.delta_deg, "reps": args.reps,
                     "records": records}
    for joint in args.joints.split(","):
        rows = [r for r in records if r["joint"] == joint]
        summary[joint] = {}
        for direction in (1.0, -1.0):
            values = [r["residual"] for r in rows if r["direction"] == direction]
            summary[joint][f"{direction:+.0f}"] = {
                "median": round(statistics.median(values), 3),
                "values": [round(v, 3) for v in values],
                "abs_median": round(abs(statistics.median(values)), 3)}
        summary[joint]["both_directions_le_0_3"] = all(
            summary[joint][f"{d:+.0f}"]["abs_median"] <= 0.3 for d in (1.0, -1.0))
        summary[joint]["consistent"] = (
            abs(summary[joint]["+1"]["median"] - summary[joint]["-1"]["median"])
            <= 0.3)
    summary["take_up_seen"] = take_up_seen
    Path(args.out).write_text(json.dumps(summary, indent=2) + "\n")
    print(f"\nwrote {args.out}")
    print("target: |median residual| <= 0.3 both directions, and the two medians "
          "within 0.3 of each other")
    return 0


if __name__ == "__main__":
    sys.exit(main())
