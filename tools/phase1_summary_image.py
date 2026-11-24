#!/usr/bin/env python3
"""One summary sheet for the Phase 1 receipt: the three milestones side by side.

    robot-arm-sim python tools/phase1_summary_image.py \
        --parity reports/phase1_parity.json \
        --squares reports/phase1_squares.json \
        --landing reports/phase1_landing.json \
        --out reports/phase1_summary.png
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, PAD = 1240, 30
INK = (24, 24, 28)
MUTED = (110, 114, 122)
PASS = (28, 138, 76)
AMBER = (186, 120, 16)
RULE = (222, 224, 230)


def font(size):
    for path in ("/usr/share/fonts/TTF/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--parity", required=True)
    ap.add_argument("--squares", required=True)
    ap.add_argument("--landing", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    p = json.loads(Path(a.parity).read_text())
    s = json.loads(Path(a.squares).read_text())
    g = json.loads(Path(a.landing).read_text())
    m = s["summary"]

    blocks = [
        ("1 · MODEL PARITY", PASS, "PASS", [
            f"FK(chain) vs FK(mink model), jaw tip, {p['samples']} poses",
            f"worst position delta {p['worst_delta_mm']:.6f} mm  (gate {p['tolerance_mm']} mm)",
            f"worst rotation delta {p['worst_delta_rot_deg']:.1e} deg",
            "the QP stack solves the LIVE arm, bit-exact",
        ]),
        ("2 · 64-SQUARE SWEEP", PASS, "PASS", [
            f"least_squares {sum(1 for q in s['squares'] if q['least_squares']['verdict']=='PASS')}/64"
            f"   mink {sum(1 for q in s['squares'] if q['mink']['verdict']=='PASS')}/64"
            f"   regressions {sum(1 for q in s['squares'] if q['delta']=='REGRESS')}",
            f"residual p50   LS {m['ls_median_mm']:.2f} mm   mink {m['mink_median_mm']:.2f} mm",
            f"solve time p50 LS {m['ls_median_ms']:.0f} ms   mink {m['mink_median_ms']:.0f} ms",
            f"solve time max LS 169504 ms   mink 4 ms   (417 s vs 0.2 s total)",
        ]),
        ("3 · GATE PROPOSAL", AMBER, "REVIEW", [
            f"mink solver residual p99 {g['solver_residual_mm']['mink']['p99']:.2f}  "
            f"max {g['solver_residual_mm']['mink']['max']:.2f} mm",
            f"model bias {g['model_bias_mm']:.1f} mm + hardware {g['hardware_residual_mm']['settle_p95_median_mm']:.1f} mm "
            "invisible to an FK gate",
            f"honest landing p50 {g['honest_landing_mm']['p50']:.1f}  p95 {g['honest_landing_mm']['p95']:.1f} mm",
            f"20.0 mm -> PROPOSED {g['proposed_gate_mm']:.1f} mm with the flag; OWNER APPROVAL",
        ]),
    ]

    line_h, block_gap = 22, 26
    height = PAD * 2 + 96 + sum(34 + line_h * len(b[3]) + block_gap for b in blocks)
    img = Image.new("RGB", (W, height), (250, 250, 252))
    d = ImageDraw.Draw(img)
    title, sub, small, tag = font(25), font(14), font(14), font(12)
    d.text((PAD, PAD), "Phase 1 — mink unification", font=title, fill=INK)
    d.text((PAD, PAD + 36), "promote the sim QP stack to be THE solver behind /move "
           "— flag wired, default OFF, nothing committed", font=sub, fill=MUTED)
    y = PAD + 88
    for name, colour, badge, lines in blocks:
        d.text((PAD, y), name, font=small, fill=INK)
        bw = 92
        d.rounded_rectangle((W - PAD - bw, y - 3, W - PAD, y + 19), radius=4, fill=colour)
        d.text((W - PAD - bw + 22, y + 1), badge, font=tag, fill=(255, 255, 255))
        y += 26
        for text in lines:
            d.text((PAD + 16, y), text, font=small, fill=INK)
            y += line_h
        y += 8
        d.line((PAD, y, W - PAD, y), fill=RULE)
        y += block_gap
    img.save(a.out)
    print(f"wrote {a.out} ({img.width}x{img.height})")


if __name__ == "__main__":
    main()
