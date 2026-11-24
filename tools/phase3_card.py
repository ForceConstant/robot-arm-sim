#!/usr/bin/env python3
"""Render a small status card PNG for a Phase 3 milestone.

Not a calculation: just a readable summary image for the person watching.

    .venv/bin/python tools/phase3_card.py --title "..." --lines "a|b|c" --out x.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--title", required=True)
    ap.add_argument("--subtitle", default="")
    ap.add_argument("--lines", default="", help="pipe-separated")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    lines = [s for s in args.lines.split("|") if s.strip()]

    W, pad = 980, 34
    H = 150 + 30 * len(lines) + (40 if args.subtitle else 0)
    img = Image.new("RGB", (W, H), (252, 251, 248))
    d = ImageDraw.Draw(img)
    try:
        title = ImageFont.truetype(BOLD, 27)
        sub = ImageFont.truetype(FONT, 17)
        body = ImageFont.truetype(FONT, 17)
        mono = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 16)
    except Exception:
        title = sub = body = mono = None
    d.rectangle([0, 0, W, 6], fill=(30, 90, 160))
    d.text((pad, 26), args.title, fill=(20, 20, 25), font=title)
    y = 74
    if args.subtitle:
        d.text((pad, y), args.subtitle, fill=(90, 90, 100), font=sub)
        y += 34
    for line in lines:
        colour = (200, 30, 30) if line.startswith("!") else (35, 35, 40)
        d.text((pad, y), line.lstrip("!"), fill=colour, font=mono)
        y += 30
    img.save(args.out)
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
