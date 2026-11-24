#!/usr/bin/env python3
"""Render a small status image for a Phase 0 milestone.

Pillow only, no matplotlib (the BC-250 venv has PIL 13, and this has to run
there). One panel per run: a per-case row with a PASS/FAIL badge, measured joint
travel, reported error and tip travel. Bars, not curves — there is no time series
worth plotting here.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, ROW, PAD = 1180, 34, 22
INK = (24, 24, 28)
MUTED = (110, 114, 122)
PASS = (28, 138, 76)
FAIL = (190, 46, 46)
AMBER = (186, 120, 16)


def font(size: int):
    for path in ("/usr/share/fonts/TTF/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def panel(run: dict) -> tuple[Image.Image, int]:
    cases = run.get("cases") or []
    height = PAD * 2 + 96 + ROW * max(1, len(cases)) + 52
    image = Image.new("RGB", (W, height), (250, 250, 252))
    draw = ImageDraw.Draw(image)
    title, sub, small = font(23), font(14), font(14)

    draw.text((PAD, PAD), "Phase 0 — move-verification harness", font=title, fill=INK)
    mode = run.get("mode", "?")
    flags = []
    if run.get("stale_readback"):
        flags.append("stale readback (frozen encoders)")
    if run.get("stall_above") is not None:
        flags.append(f"stall above {run['stall_above']} deg")
    if run.get("deadband"):
        flags.append(f"deadband {run['deadband']} deg")
    if run.get("sag"):
        flags.append(f"sag {run['sag']}")
    label = f"{mode.upper()} backend" + ((" · " + " · ".join(flags)) if flags else "")
    draw.text((PAD, PAD + 32), label, font=small, fill=MUTED)

    verdict = "PASS" if run.get("passed") else "FAIL"
    colour = PASS if run.get("passed") else FAIL
    badge = (W - PAD - 96, PAD + 6, W - PAD, PAD + 34)
    draw.rounded_rectangle(badge, radius=5, fill=colour)
    draw.text((badge[0] + 27, badge[1] + 7), verdict, font=small, fill=(255, 255, 255))

    top = PAD + 96
    headers = [("case", 300), ("", 62), ("joint travel", 130),
               ("max joint", 110), ("reported err", 120), ("tip travel", 120)]
    x = PAD
    for text, width in headers:
        if text:
            draw.text((x, top), text.upper(), font=small, fill=MUTED)
        x += width
    draw.line([(PAD, top + 22), (W - PAD, top + 22)], fill=(218, 218, 224), width=1)

    y = top + 32
    for case in cases:
        ok = bool(case.get("ok"))
        mark = PASS if ok else FAIL
        draw.rectangle([PAD, y + 8, PAD + 8, y + 20], fill=mark)
        draw.text((PAD + 18, y), str(case.get("case"))[:30], font=small, fill=INK)
        movement = case.get("movement") or {}
        travel = movement.get("travel_deg")
        maxabs = movement.get("max_abs_deg")
        err = case.get("reported_error_deg")
        tip = case.get("tip_travel_mm")
        cols = [
            (PAD + 300, "moved" if ok else "no move"),
            (PAD + 362, f"{travel:.1f}°" if travel is not None else "—"),
            (PAD + 492, f"{maxabs:.1f}°" if maxabs is not None else "—"),
            (PAD + 602, f"{err:.2f}°" if err is not None else "—"),
            (PAD + 722, f"{tip:.1f} mm" if tip is not None else "—"),
        ]
        for cx, text in cols:
            draw.text((cx, y), text, font=small,
                      fill=INK if ok else AMBER)
        y += ROW

    note = run.get("note") or ""
    if note:
        draw.text((PAD, y + 8), note[:150], font=small, fill=MUTED)
    return image, height


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="+", required=True,
                    help="result JSONs, one panel each, left to right")
    ap.add_argument("--out", required=True)
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    panels = []
    for path in args.runs:
        run = json.loads(Path(path).read_text())
        if args.note:
            run["note"] = args.note
        panels.append(panel(run)[0])
    width = sum(p.width for p in panels)
    height = max(p.height for p in panels)
    sheet = Image.new("RGB", (width, height), (250, 250, 252))
    x = 0
    for p in panels:
        sheet.paste(p, (x, 0))
        x += p.width
        if x < width:
            ImageDraw.Draw(sheet).line([(x, 0), (x, height)], fill=(200, 200, 208), width=2)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.out)
    print(f"wrote {args.out} {sheet.width}x{sheet.height}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
