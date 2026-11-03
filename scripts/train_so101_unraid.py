#!/usr/bin/env python3
"""Start a Phase 11a training run on the Unraid host and collect its receipt.

Usage:
  python3 scripts/train_so101_unraid.py download
  python3 scripts/train_so101_unraid.py train -- --camera wrist --episodes 40 --steps 400

Requires the docker socket proxy at DOCKER_PROXY (default 100.76.32.93:2375) and
the named volume `phase11_data`. No host changes; containers are removed after.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMAGE = os.environ.get("PHASE11_IMAGE", "ghcr.io/forceconstant/pdf2epub-nvidia:latest")
PROXY = os.environ.get("DOCKER_PROXY", "http://100.76.32.93:2375")


def call(method, path, payload=None, timeout=120):
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(PROXY + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read()
        return json.loads(body) if body.strip() else {}


def run(script: Path, argv: list[str], name: str, minutes: int = 240):
    code = base64.b64encode(script.read_bytes()).decode()
    launcher = (f"import base64,sys;sys.argv={['run'] + argv!r};"
                f"exec(base64.b64decode('{code}'))")
    payload = {
        "Image": IMAGE,
        "Entrypoint": ["python3", "-c", launcher],
        "Tty": True,
        "HostConfig": {
            "AutoRemove": True,
            "Binds": ["phase11_data:/data"],
            "Runtime": "nvidia",
            "DeviceRequests": [{"Driver": "", "Count": -1, "DeviceIDs": [],
                                "Capabilities": [["gpu"]], "Options": {}}],
            "Memory": 8 * 1024 ** 3,
            "NanoCpus": int(4e9),
            "NetworkMode": "bridge",
        },
    }
    cid = call("POST", f"/containers/create?name={name}", payload)["Id"]
    call("POST", f"/containers/{cid}/start", {})
    deadline = time.time() + minutes * 60
    while time.time() < deadline:
        info = call("GET", f"/containers/{cid}/json")
        if not info["State"]["Running"]:
            break
        time.sleep(20)
    info = call("GET", f"/containers/{cid}/json")
    req = urllib.request.Request(PROXY + f"/containers/{cid}/logs?stdout=1&stderr=1")
    with urllib.request.urlopen(req, timeout=60) as r:
        raw = r.read()
    if raw[:8] and raw[0] in (0, 1, 2) and len(raw) > 8:
        out, i = [], 0
        while i + 8 <= len(raw):
            size = int.from_bytes(raw[i + 4:i + 8], "big")
            out.append(raw[i + 8:i + 8 + size])
            i += 8 + size
        raw = b"".join(out)
    return {"exit": info["State"].get("ExitCode"), "logs": raw.decode("utf-8", "replace")[-8000:]}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=("download", "train"))
    ap.add_argument("rest", nargs=argparse.REMAINDER)
    args = ap.parse_args()
    rest = [a for a in args.rest if a != "--"]
    script = ROOT / ("training/probes/so101_download.py" if args.action == "download"
                     else "training/probes/so101_train.py")
    if not script.exists():
        raise SystemExit(f"missing {script}")
    name = f"phase11-{args.action}"
    print(json.dumps(run(script, rest, name), indent=1))


if __name__ == "__main__":
    main()
