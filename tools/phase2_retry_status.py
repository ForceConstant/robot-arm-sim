#!/usr/bin/env python3
"""Phase 2 retry — milestone status sheets from a small JSON spec.

One image per milestone the retry brief asks for (elbow probe, IRL gate, flip
applied). Pillow only, runs on the BC-250 (that is where the fonts and Pillow
live).

    .venv/bin/python tools/phase2_retry_status.py --spec m1.json --out reports/retry_m1_elbow.png

Spec::

    {"title": "...", "subtitle": "...",
     "headers": ["probe", "travel", "verdict"],
     "widths": [300, 200, 300],
     "tone_col": 2,
     "rows": [{"cells": ["elbow -2 deg", "0.35 / 2.0", "STALL"], "tone": "bad"}]}
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, PAD = 1240, 26
INK = (24, 24, 28)
MUTED = (108, 112, 120)
RULE = (222, 224, 230)
TONES = {"ok": (28, 138, 76), "bad": (190, 46, 46),
         "warn": (186, 120, 16), "none": INK}


def font(size: int):
    for path in ("/usr/share/fonts/TTF/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def build(spec: dict, out: str) -> None:
    rows = spec["rows"]
    row_h = 32
    height = PAD * 2 + 96 + row_h * (len(rows) + 1) + 44
    width = max(W, PAD * 2 + sum(spec["widths"]))
    image = Image.new("RGB", (width, height), (250, 250, 252))
    draw = ImageDraw.Draw(image)
    f_title, f_sub, f_head, f_cell = font(23), font(14), font(13), font(14)

    draw.text((PAD, PAD), spec["title"], font=f_title, fill=INK)
    draw.text((PAD, PAD + 34), spec["subtitle"], font=f_sub, fill=MUTED)

    y = PAD + 88
    x = PAD
    for head, col_w in zip(spec["headers"], spec["widths"]):
        draw.text((x, y), head, font=f_head, fill=MUTED)
        x += col_w
    y += 22
    draw.line((PAD, y, width - PAD, y), fill=RULE, width=1)
    y += 8

    for row in rows:
        x = PAD
        colour = TONES.get(row.get("tone", "none"), INK)
        for index, (cell, col_w) in enumerate(zip(row["cells"], spec["widths"])):
            ink = colour if index == spec.get("tone_col") else INK
            draw.text((x, y), str(cell), font=f_cell, fill=ink)
            x += col_w
        y += row_h

    footer = spec.get("footer")
    if footer:
        draw.text((PAD, y + 8), footer, font=f_sub, fill=MUTED)
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    image.save(out)
    print(f"wrote {out} ({image.width}x{image.height})")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    build(json.loads(Path(args.spec).read_text()), args.out)


if __name__ == "__main__":
    main()
