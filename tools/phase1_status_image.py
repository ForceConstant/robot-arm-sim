#!/usr/bin/env python3
"""Status sheets for the Phase 1 milestones (parity, 64-square, gate proposal).

Pillow only, no matplotlib (the BC-250 venv has PIL, not matplotlib).  Each
sheet is a single panel: a headline verdict, the numbers that decide it, and a
small table.  Deliberately plain — this is a receipt artifact, not a chart.

    robot-arm-sim python tools/phase1_status_image.py parity \
        --json reports/phase1_parity.json --out reports/phase1_parity.png
    robot-arm-sim python tools/phase1_status_image.py squares \
        --compare reports/phase1_squares.json --out reports/phase1_squares.png
    robot-arm-sim python tools/phase1_status_image.py gate \
        --json reports/phase1_landing.json --out reports/phase1_gate.png
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, ROW, PAD = 1240, 30, 26
INK = (24, 24, 28)
MUTED = (110, 114, 122)
PASS = (28, 138, 76)
FAIL = (190, 46, 46)
AMBER = (186, 120, 16)
RULE = (222, 224, 230)


def font(size: int):
    for path in ("/usr/share/fonts/TTF/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def _sheet(title, subtitle, verdict, colour, lines, table_head, table_rows, out):
    height = PAD * 2 + 118 + 24 * len(lines) + 40 + ROW * (len(table_rows) + 1) + 30
    image = Image.new("RGB", (W, max(height, 340)), (250, 250, 252))
    draw = ImageDraw.Draw(image)
    t_font, s_font, m_font = font(24), font(14), font(13)
    draw.text((PAD, PAD), title, font=t_font, fill=INK)
    draw.text((PAD, PAD + 34), subtitle, font=s_font, fill=MUTED)
    badge = (W - PAD - 130, PAD + 2, W - PAD, PAD + 32)
    draw.rounded_rectangle(badge, radius=5, fill=colour)
    draw.text((badge[0] + 16, badge[1] + 7), verdict, font=s_font, fill=(255, 255, 255))

    y = PAD + 74
    for line in lines:
        draw.text((PAD, y), line, font=s_font, fill=INK)
        y += 24
    y += 16
    draw.line((PAD, y, W - PAD, y), fill=RULE)
    y += 12
    columns = len(table_head)
    col_w = (W - 2 * PAD) // columns
    for index, head in enumerate(table_head):
        draw.text((PAD + index * col_w, y), head, font=m_font, fill=MUTED)
    y += ROW
    for row in table_rows:
        for index, cell in enumerate(row):
            fill = INK
            text = str(cell)
            if text in ("PASS", "exact"):
                fill = PASS
            elif text in ("FAIL",):
                fill = FAIL
            elif text in ("REGRESS", "worse"):
                fill = AMBER
            draw.text((PAD + index * col_w, y), text, font=m_font, fill=fill)
        y += ROW
    image.save(out)
    print(f"wrote {out} ({image.width}x{image.height})")


def parity(args):
    r = json.loads(Path(args.json).read_text())
    verdict = "PASS" if r["passed"] else "FAIL"
    colour = PASS if r["passed"] else FAIL
    lines = [
        f"FK(chain) vs FK(mink model), jaw tip, {r['samples']} joint-grid samples "
        f"(step {r['step_deg']:g} deg, rolls 0/±90)",
        f"worst position delta  {r['worst_delta_mm']:.6f} mm   (gate {r['tolerance_mm']} mm)",
        f"worst rotation delta  {r['worst_delta_rot_deg']:.2e} deg",
        f"reference angles all zero: {r['references_zero']}   "
        f"tip in chain {r['tip_in_chain_m'][2] * 1000:.2f} mm",
        "The mink model is built from the calibration chain + TIP_IN_CHAIN, not from the sim scene.",
    ]
    rows = [[f"{s['delta_mm']:.2e} mm", f"{s['delta_rot_deg']:.1e} deg",
             ", ".join(f"{v:g}" for v in s["joints_deg"][:5])]
            for s in r["sample_rows"][:6]]
    _sheet("Phase 1 — model parity", "chain FK vs mink arm model built from the same chain",
           verdict, colour, lines, ["position Δ", "rotation Δ", "joints (deg)"], rows, args.out)


def squares(args):
    r = json.loads(Path(args.compare).read_text())
    rows = []
    for row in r["squares"]:
        rows.append([row["square"], row["least_squares"].get("verdict", "?"),
                     f"{row['least_squares'].get('mm', 0):.1f}",
                     f"{row['least_squares'].get('ms', 0):.0f}",
                     row["mink"].get("verdict", "?"),
                     f"{row['mink'].get('mm', 0):.1f}",
                     f"{row['mink'].get('ms', 0):.0f}",
                     row.get("delta", "same")])
    ls_ok = sum(1 for q in r["squares"] if q["least_squares"].get("verdict") == "PASS")
    mink_ok = sum(1 for q in r["squares"] if q["mink"].get("verdict") == "PASS")
    regress = sum(1 for q in r["squares"] if q.get("delta") == "REGRESS")
    verdict = "PASS" if regress == 0 and mink_ok >= ls_ok else "REVIEW"
    colour = PASS if verdict == "PASS" else AMBER
    lines = [
        f"{len(r['squares'])} squares in sim, identical targets, both solvers",
        f"least_squares {ls_ok}/{len(r['squares'])}   mink {mink_ok}/{len(r['squares'])}   "
        f"regressions {regress}",
        f"median solve ms  least_squares {r['summary']['ls_median_ms']:.0f}   "
        f"mink {r['summary']['mink_median_ms']:.0f}",
        f"median landing mm  least_squares {r['summary']['ls_median_mm']:.2f}   "
        f"mink {r['summary']['mink_median_mm']:.2f}",
        f"hover {r['hover_mm']:g} mm above the board surface",
    ]
    _sheet("Phase 1 — 64-square sweep, least_squares vs mink",
           "same board-frame targets; verdict follows the live gate (20 mm / 8 deg)",
           verdict, colour, lines,
           ["square", "LS", "mm", "ms", "mink", "mm", "ms", "Δ"],
           rows[:64], args.out)


def gate(args):
    r = json.loads(Path(args.json).read_text())
    lines = [
        f"Solver residual bound, {r['squares']*2} solved poses (64 squares x 2 solvers)",
        f"mink solver residual  p50 {r['solver_residual_mm']['mink']['p50']:.2f}  "
        f"p95 {r['solver_residual_mm']['mink']['p95']:.2f}  "
        f"p99 {r['solver_residual_mm']['mink']['p99']:.2f}  "
        f"max {r['solver_residual_mm']['mink']['max']:.2f} mm",
        f"least_squares residual  p50 {r['solver_residual_mm']['least_squares']['p50']:.2f}  "
        f"max {r['solver_residual_mm']['least_squares']['max']:.2f} mm",
        f"model bias {r['model_bias_mm']:.1f} mm (T2 refit) and hardware scatter "
        f"{r['hardware_residual_mm']['settle_p95_median_mm']:.1f} mm are INVISIBLE to an FK-vs-FK gate",
        f"honest physical landing  p50 {r['honest_landing_mm']['p50']:.1f}  "
        f"p95 {r['honest_landing_mm']['p95']:.1f} mm  (why 20 mm never rejects anything)",
        f"today {r['current_gate_mm']:.1f} mm -> PROPOSED {r['proposed_gate_mm']:.1f} mm "
        f"(solver p{r['proposed_percentile']} + headroom); OWNER APPROVAL REQUIRED",
    ]
    rows = [[f"{a['gate_mm']:g} mm", f"{a['mink_pass']}/64", f"{a['least_squares_pass']}/64"]
            for a in r["gate_admission"]]
    _sheet("Phase 1 — proposed move_tolerance_mm",
           "a bound on the SOLVER, not on physical landing accuracy",
           "PROPOSAL", AMBER, lines, ["candidate gate", "mink pass", "LS pass"], rows, args.out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="kind", required=True)
    p = sub.add_parser("parity"); p.add_argument("--json", required=True)
    p.add_argument("--out", required=True); p.set_defaults(fn=parity)
    s = sub.add_parser("squares"); s.add_argument("--compare", required=True)
    s.add_argument("--out", required=True); s.set_defaults(fn=squares)
    g = sub.add_parser("gate"); g.add_argument("--json", required=True)
    g.add_argument("--out", required=True); g.set_defaults(fn=gate)
    args = parser.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
