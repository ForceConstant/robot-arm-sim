"""Post-deploy verification: wait for the new build, then run crowded-v3.

Case failures are data, not a deploy failure; a suite that never completes, a
version mismatch, or a dead health endpoint is. Writes a receipt next to the
deploy artifacts so every release has a recorded outcome.
"""
from __future__ import annotations

import argparse
import json
import math
import zlib
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

from chess_sim import gripper
from chess_sim.dual import DualArmEnv

ROOT = Path(__file__).resolve().parents[1]
SUITE_TIMEOUT = 60 * 25
READY_TIMEOUT = 60 * 8


def call(base, path, payload=None, timeout=30):
    data = None if payload is None else json.dumps(payload).encode()
    request = urllib.request.Request(
        base + path, data=data, method="GET" if payload is None else "POST",
        headers={"Content-Type": "application/json", "X-Robot-Lab": "1"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read()
        return json.loads(body) if body else {}


def download(base, path):
    request = urllib.request.Request(base + path,
        headers={"X-Robot-Lab": "1"})
    with urllib.request.urlopen(request, timeout=30) as response:
        return response.read()


def wait_ready(base, expected_version, timeout=READY_TIMEOUT):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            status = call(base, "/api/status")
        except Exception as exc:                  # container restarting
            last = str(exc)
            time.sleep(4)
            continue
        if status.get("ready") and status.get("version") == expected_version:
            return status
        last = "%s / %s" % (status.get("version"), status.get("phase"))
        time.sleep(4)
    raise RuntimeError(f"Simulator did not become ready as {expected_version}: {last}")


def run_suite(base, speed=8.0, seed=None):
    call(base, "/api/control", {"action": "speed", "speed": speed})
    payload = {"action": "suite", "suite": "start"}
    if seed is not None:
        payload["suite_seed"] = int(seed)
    call(base, "/api/control", payload)
    started = time.time()
    status = call(base, "/api/status")
    while time.time() - started < SUITE_TIMEOUT:
        status = call(base, "/api/status")
        suite = status.get("suite") or {}
        if suite.get("phase") in ("complete", "stopped"):
            return status, time.time() - started
        if status.get("phase") in ("fault", "error"):
            break
        time.sleep(5)
    raise RuntimeError("Suite did not finish: " + str(status.get("message")))


def verify(base, expected_version, speed=8.0, receipt_dir=None, seed=None):
    status = wait_ready(base, expected_version)
    final, elapsed = run_suite(base, speed, seed)
    results = (final.get("suite") or {}).get("results", [])
    suite = final.get("suite") or {}
    nominal_home = DualArmEnv.home_pose().tolist()
    if suite.get("version") != "crowded-v3":
        raise RuntimeError(f"Unexpected suite version: {suite.get('version')}")
    if suite.get("seed") != seed:
        raise RuntimeError(f"Suite seed mismatch: {suite.get('seed')} != {seed}")
    for case in results:
        if seed is not None:
            expected_case_seed = (zlib.crc32(case["case"].encode()) ^ int(seed)) & 0xffffffff
            if case.get("seed") != expected_case_seed:
                raise RuntimeError(f"Case seed mismatch for {case['case']}")
            layout = case.get("layout") or {}
            origin = layout.get("board_origin_m") or []
            start = layout.get("arm_start_rad") or []
            if len(origin) != 2 or any(abs(v) > 0.005001 for v in origin):
                raise RuntimeError(f"Board perturbation escaped bound for {case['case']}: {origin}")
            # Stock and CAD packs intentionally use different nominal gripper
            # angles. Compare all six when this verifier is configured with the
            # deployed CAD pack; otherwise validate the five arm joints and
            # retain/range-check the recorded jaw angle.
            joints_to_check = 6 if gripper.CUSTOM else 5
            if (len(start) != 6 or any(not math.isfinite(v) for v in start)
                    or any(abs(a - b) > 0.100001 for a, b in
                           zip(start[:joints_to_check], nominal_home[:joints_to_check]))
                    or abs(start[5]) > math.pi):
                raise RuntimeError(f"Arm start perturbation escaped bound for {case['case']}: {start}")
    passed = sum(1 for r in results if r["success"])
    receipt = {
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "base": base, "version": status["version"], "renderer": status.get("renderer"),
        "suite_version": (final.get("suite") or {}).get("version"),
        "suite_seed": seed,
        "elapsed_s": round(elapsed, 1), "cases": len(results), "passed": passed,
        "failed": len(results) - passed,
        "results": [{k: r.get(k) for k in ("case", "success", "error_mm", "categories",
                                               "seed", "layout", "recording")}
                    for r in results],
    }
    folder = Path(receipt_dir) if receipt_dir else ROOT / ".private"
    folder.mkdir(parents=True, exist_ok=True)
    name = (f"crowded-v3-seed-{seed}.json" if seed is not None
            else f"post-deploy-{status['version']}.json")
    (folder / name).write_text(json.dumps(receipt, indent=2))
    failed = [r for r in receipt["results"] if not r["success"] and r.get("recording")]
    if failed:
        archive_dir = folder / (f"crowded-v3-seed-{seed}-incidents" if seed is not None
                                else f"post-deploy-{status['version']}-incidents")
        archive_dir.mkdir(parents=True, exist_ok=True)
        for row in failed:
            ident = row["recording"]
            (archive_dir / f"{ident}.zip").write_bytes(
                download(base, f"/api/incidents/{ident}.zip"))
    return receipt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="http://192.168.1.138:8098")
    parser.add_argument("--version", default=None, help="expected version, e.g. 1.4.2")
    parser.add_argument("--speed", type=float, default=8.0)
    parser.add_argument("--receipt-dir", default=None)
    parser.add_argument("--seed", type=int, default=None,
                        help="bounded board/start perturbation seed for a repeated suite")
    args = parser.parse_args()
    version = args.version
    if version is None:
        version = (ROOT / "robot_web" / "__init__.py").read_text().split('"')[1]
    receipt = verify(args.base, version, args.speed, args.receipt_dir, args.seed)
    print(f"{receipt['version']} on {receipt['renderer']} · seed={receipt['suite_seed']}")
    for case in receipt["results"]:
        print("  %-13s %-5s %-8s %s" % (case["case"], "ok" if case["success"] else "FAIL",
                                        case["error_mm"], ", ".join(case["categories"] or [])))
    print(f"{receipt['passed']}/{receipt['cases']} passed in {receipt['elapsed_s']}s")


if __name__ == "__main__":
    main()
