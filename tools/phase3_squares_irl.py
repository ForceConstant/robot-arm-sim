#!/usr/bin/env python3
"""64-square IRL sweep with a camera frame per square.

Moves the real arm through every square at a 60 mm hover through the live
console (`/move`, the same path the dashboard uses), waits for the move to
finish, then captures one frame plus the joint state from the bridge.  The
frames are the measurement: each square yields the PnP camera pose and the FK
camera pose, so the distribution of FK-vs-image error over the whole board can
be computed before and after a calibration change on *identical* images.

    .venv/bin/python tools/phase3_squares_irl.py --base http://127.0.0.1:8080
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim import fiducials  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")
FILES = "abcdefgh"


def targets(hover=60.0, square=28.0):
    half = square * 4.0
    return [(f"{FILES[i]}{r}", (i + 0.5) * square - half,
             (r - 0.5) * square - half, hover)
            for r in range(1, 9) for i in range(8)]


def post(base, path, body, timeout=200):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"},
                                 method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8080")
    ap.add_argument("--out", default=str(ROOT / "live" / "captures_squares_phase3"))
    ap.add_argument("--receipt", default=str(ROOT / "reports" / "phase3_squares_irl.json"))
    ap.add_argument("--hover-mm", type=float, default=60.0)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    token = (ROOT / cfg["token_file"]).read_text().strip()
    from bridge_client import HttpFrames
    frames = HttpFrames(cfg["bridge_url"], token=token, timeout=15.0)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    ts = time.strftime("%Y%m%dT%H%M%S")
    squares = targets(args.hover_mm)
    if args.limit:
        squares = squares[:args.limit]
    for name, x, y, z in squares:
        t0 = time.time()
        body = {"mode": "xyz", "x_mm": x, "y_mm": y, "z_mm": z, "span_deg": 0.0}
        try:
            result = post(args.base, "/move", body)
            ok, detail = True, None
        except urllib.error.HTTPError as exc:
            result = {}
            ok = False
            detail = exc.read().decode()[:200]
        dur = time.time() - t0
        rec = {"square": name, "target": [x, y, z], "ok": ok, "detail": detail,
               "move_s": round(dur, 1),
               "error_deg": (result.get("error_deg")),
               "tip_mm": ((result.get("pose") or {}).get("tip_mm")),
               "frame": None}
        if ok:
            frame = frames.capture()
            ctrl = 0
            for _ in range(3):
                st = json.loads(urllib.request.urlopen(
                    args.base + "/state", timeout=5).read())
                cur = [float(st["bridge"]["joints"][n]) for n in NAMES]
                drift = max(abs(a - b) for a, b in zip(cur, frame.joints_deg))
                if drift <= 1.5:
                    break
                time.sleep(0.3)
                frame = frames.capture()
            dets = fiducials.detect_markers(frame.image)
            markers = [{"id": int(d.marker_id),
                        "corners": [[round(float(c), 2) for c in p]
                                    for p in np.asarray(d.corners).reshape(-1, 2)],
                        "edge_px": round(float(np.mean([
                            np.linalg.norm(d.corners[i] - d.corners[(i + 1) % 4])
                            for i in range(4)])), 1)} for d in dets]
            nm = f"{ts}_{name}"
            cv2.imwrite(str(out / f"{nm}.jpg"),
                        cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR))
            rec["frame"] = nm
            rec["joints_deg"] = {n: float(v) for n, v in zip(NAMES, frame.joints_deg)}
            rec["drift_deg"] = round(drift, 2)
            rec["markers"] = markers
            (out / f"{nm}.json").write_text(json.dumps(rec, indent=2) + "\n")
        rows.append(rec)
        print(f"{name}: ok {ok} {dur:5.1f}s err {rec['error_deg']} "
              f"markers {[m['id'] for m in (rec.get('markers') or [])]}"
              + (f"  {detail}" if detail else ""), flush=True)

    receipt = {"schema": "phase3-squares-irl/1",
               "started": ts, "base": args.base, "hover_mm": args.hover_mm,
               "squares": rows,
               "summary": {"ok": sum(1 for r in rows if r["ok"]),
                           "with_markers": sum(1 for r in rows if r.get("markers")),
                           "total": len(rows)}}
    Path(args.receipt).write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"\n{receipt['summary']}\nreceipt: {args.receipt}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
