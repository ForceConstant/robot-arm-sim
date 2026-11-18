#!/usr/bin/env python3
"""Inventory the cal capture set: which files carry joints and which the
ChArUco detector can actually use."""
from __future__ import annotations

import json
from pathlib import Path

import cv2

from chess_sim import fiducials
from chess_sim.charuco_face import FIRST_ID

caps = Path("live/captures")
rows = []
for item in sorted(caps.glob("*_cal.json")):
    record = json.loads(item.read_text())
    jpg = item.with_suffix(".jpg")
    good = 0
    ids = []
    if jpg.exists():
        image = cv2.imread(str(jpg))
        if image is not None:
            det = [d.marker_id for d in fiducials.detect_markers(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                   if FIRST_ID <= d.marker_id < FIRST_ID + 32]
            good = len(det)
            ids = sorted(det)
    stored = [m["id"] for m in record.get("markers", []) if FIRST_ID <= m["id"] < FIRST_ID + 32]
    rows.append((item.stem, record.get("label"), bool(record.get("joints_deg")), len(stored), good, ids[:3]))

print(f"{'name':34s} label joints storeddet detected ids")
for name, label, joints, stored, good, ids in rows:
    print(f"{name:34s} {str(label):5s} {str(joints):6s} {stored:9d} {good:8d} {ids}")
print("total cal json:", len(rows))
print("with >=1 charuco detection:", sum(1 for r in rows if r[4] >= 1))
print("with >=2:", sum(1 for r in rows if r[4] >= 2))
print("with joints:", sum(1 for r in rows if r[2]))
