"""Post-deploy verification: wait for the new build, then run crowded-v2.

Case failures are data, not a deploy failure; a suite that never completes, a
version mismatch, or a dead health endpoint is. Writes a receipt next to the
deploy artifacts so every release has a recorded outcome.
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

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


def run_suite(base, speed=8.0):
    call(base, "/api/control", {"action": "speed", "speed": speed})
    call(base, "/api/control", {"action": "suite", "suite": "start"})
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


def verify(base, expected_version, speed=8.0, receipt_dir=None):
    status = wait_ready(base, expected_version)
    final, elapsed = run_suite(base, speed)
    results = (final.get("suite") or {}).get("results", [])
    passed = sum(1 for r in results if r["success"])
    receipt = {
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "base": base, "version": status["version"], "renderer": status.get("renderer"),
        "suite_version": (final.get("suite") or {}).get("version"),
        "elapsed_s": round(elapsed, 1), "cases": len(results), "passed": passed,
        "failed": len(results) - passed,
        "results": [{k: r.get(k) for k in ("case", "success", "error_mm", "categories")}
                    for r in results],
    }
    folder = Path(receipt_dir) if receipt_dir else ROOT / ".private"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / f"post-deploy-{status['version']}.json").write_text(json.dumps(receipt, indent=2))
    return receipt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="http://192.168.1.138:8098")
    parser.add_argument("--version", default=None, help="expected version, e.g. 1.4.2")
    parser.add_argument("--speed", type=float, default=8.0)
    parser.add_argument("--receipt-dir", default=None)
    args = parser.parse_args()
    version = args.version
    if version is None:
        version = (ROOT / "robot_web" / "__init__.py").read_text().split('"')[1]
    receipt = verify(args.base, version, args.speed, args.receipt_dir)
    print(f"{receipt['version']} on {receipt['renderer']}")
    for case in receipt["results"]:
        print("  %-13s %-5s %-8s %s" % (case["case"], "ok" if case["success"] else "FAIL",
                                        case["error_mm"], ", ".join(case["categories"] or [])))
    print(f"{receipt['passed']}/{receipt['cases']} passed in {receipt['elapsed_s']}s")


if __name__ == "__main__":
    main()
