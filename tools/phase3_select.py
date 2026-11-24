#!/usr/bin/env python3
"""Pick a balanced, spatially spread subset of the solved roll plan.

The solver lands every pose in a roll on the same tool orientation, so the
useful diversity is *where* on the board the camera sits.  Greedy farthest-point
selection spreads the eight poses per roll over both board halves, and each
pose carries the count of printed markers its predicted view keeps in frame
(occlusion is not modelled -- the capture receipts record what actually decoded).

Also draws the milestone map: the 8x8 board, the selected camera stations
coloured by roll, and a summary line.

    .venv/bin/python tools/phase3_select.py --per-roll 8
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

from chess_sim.charuco_face import corners_mm  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from live.kinematics import frame_at, square_xy  # noqa: E402

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", default=str(ROOT / "reports" / "phase3_plan_20261124.json"))
    ap.add_argument("--out", default=str(ROOT / "reports" / "phase3_plan_selected.json"))
    ap.add_argument("--per-roll", type=int, default=8)
    ap.add_argument("--png", default=str(ROOT / "reports" / "phase3_plan.png"))
    args = ap.parse_args()

    plan = json.loads(Path(args.plan).read_text())
    cfg = json.loads((ROOT / "live/live_config.json").read_text())
    cal = WristCalibration.load(str(ROOT / cfg["calibration"]))
    b = plan["board"]
    board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])

    poses = plan["poses"]
    by_roll = {}
    for p in poses:
        by_roll.setdefault(float(p["roll"]), []).append(p)

    selected = []
    for roll in sorted(by_roll):
        pool = by_roll[roll]
        # board-frame target coordinates for spacing
        for p in pool:
            sx, sy = square_xy(p["target"])
            p["_xy"] = (sx, sy)
        chosen = [max(pool, key=lambda p: (p["_xy"][0] ** 2 + p["_xy"][1] ** 2))]
        while len(chosen) < min(args.per_roll, len(pool)):
            best, best_d = None, -1
            for p in pool:
                if p in chosen:
                    continue
                d = min(math.hypot(p["_xy"][0] - c["_xy"][0],
                                   p["_xy"][1] - c["_xy"][1]) for c in chosen)
                if d > best_d:
                    best, best_d = p, d
            chosen.append(best)
        for p in chosen:
            q = [math.radians(float(v)) for v in p["joints_deg"][:5]] + [0.0]
            cam = cal.camera(tuple(q))
            pos = np.asarray(cam.camera_position)
            rot = np.asarray(cam.camera_rotation).reshape(3, 3)
            visible = []
            for marker_id in range(5, 37):
                xy = np.asarray(corners_mm(marker_id), float) / 1000.0
                centre = board[:3, :3] @ np.array([xy[:, 0].mean(),
                                                   xy[:, 1].mean(), 0.0]) + board[:3, 3]
                cam_xyz = rot.T @ (centre - pos)
                depth = -cam_xyz[2]
                if depth <= 0:
                    continue
                u = cam.cx + cam.fx * cam_xyz[0] / depth
                v = cam.cy - cam.fy * cam_xyz[1] / depth
                if 0 <= u < cam.width and 0 <= v < cam.height:
                    visible.append(marker_id)
            p["predicted_markers"] = len(visible)
            p["predicted_ids"] = visible
            selected.append(p)
    for p in selected:
        p.pop("_xy", None)

    out = dict(plan)
    out["poses"] = selected
    out["selection"] = {"per_roll": args.per_roll,
                        "rule": "greedy farthest-point spread per roll group"}
    Path(args.out).write_text(json.dumps(out, indent=2) + "\n")
    print(f"{len(selected)} poses selected -> {args.out}")
    for roll in sorted(by_roll):
        rows = [p for p in selected if float(p["roll"]) == roll]
        print(f"roll {roll:+6.1f}: " + ", ".join(
            f"{p['target']}({p['predicted_markers']}mk,{p['incidence_deg']:.0f}deg)"
            for p in rows))

    # ----- milestone map
    from PIL import Image, ImageDraw, ImageFont
    scale = 3.4
    pad = 60
    size = int(244 * scale) + 2 * pad
    height = size + 150
    img = Image.new("RGB", (size, height), (250, 250, 248))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 15)
        big = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 21)
    except Exception:
        font = big = None
    colours = {-40: (30, 90, 200), 0: (20, 140, 60), 40: (230, 150, 20),
               68: (180, 40, 160), 87: (200, 40, 40)}
    for rank in range(8):
        for col in range(8):
            x0 = pad + col * 28 * scale
            y0 = pad + (7 - rank) * 28 * scale
            fill = (225, 225, 225) if (col + rank) % 2 else (255, 255, 255)
            d.rectangle([x0, y0, x0 + 28 * scale, y0 + 28 * scale], fill=fill,
                        outline=(180, 180, 180))
    for p in selected:
        roll = int(round(float(p["roll"])))
        sx, sy = square_xy(p["target"])
        cx = pad + (sx / 28.0 + 4) * 28 * scale
        cy = pad + (4 - sy / 28.0) * 28 * scale
        r = 9
        d.ellipse([cx - r, cy - r, cx + r, cy + r],
                  fill=colours.get(roll, (0, 0, 0)), outline=(30, 30, 30))
    d.text((pad, 12), "Phase 3 plan — camera stations by wrist roll", fill=(20, 20, 20), font=big)
    y = pad + 244 * scale + 10
    for i, roll in enumerate(sorted(colours)):
        rows = [p for p in selected if int(round(float(p["roll"]))) == roll]
        d.ellipse([pad + (i % 3) * 250, y + (i // 3) * 44,
                   pad + (i % 3) * 250 + 14, y + (i // 3) * 44 + 14],
                  fill=colours[roll])
        d.text((pad + (i % 3) * 250 + 22, y + (i // 3) * 44 - 2),
               f"roll {roll:+d}: {len(rows)} poses, {min((p['predicted_markers'] for p in rows), default=0)}-{max((p['predicted_markers'] for p in rows), default=0)} markers",
               fill=(40, 40, 40), font=font)
    img.save(args.png)
    print(f"wrote {args.png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
