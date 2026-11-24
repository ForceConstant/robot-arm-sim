#!/usr/bin/env python3
"""Phase 2 milestone sheets — Pillow only, run on the BC-250.

Three sheets, one per milestone the brief asks for:

  diagnosis.png  every injected fault vs what the loop actually did
  loop.png        before/after the hardened loop, same faults
  gate.png        the 64-square sweep under the 3.0 mm gate, both solvers

    robot-arm-sim python tools/phase2_status_image.py
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, PAD = 1240, 26
INK = (24, 24, 28)
MUTED = (108, 112, 120)
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


def sheet(path: str, title: str, subtitle: str, headers, rows,
          widths, verdict_col: int | None = None) -> None:
    """rows: list of (cells, colour-or-None). widths: pixel widths per column."""
    row_h = 30
    height = PAD * 2 + 92 + row_h * (len(rows) + 1) + 40
    image = Image.new("RGB", (W, height), (250, 250, 252))
    draw = ImageDraw.Draw(image)
    f_title, f_sub, f_head, f_cell = font(23), font(14), font(13), font(14)

    draw.text((PAD, PAD), title, font=f_title, fill=INK)
    draw.text((PAD, PAD + 33), subtitle, font=f_sub, fill=MUTED)

    y = PAD + 84
    x = PAD
    for head, width in zip(headers, widths):
        draw.text((x, y), head, font=f_head, fill=MUTED)
        x += width
    y += 22
    draw.line((PAD, y, W - PAD, y), fill=RULE, width=1)
    y += 8

    for cells, colour in rows:
        x = PAD
        for index, (cell, width) in enumerate(zip(cells, widths)):
            fill = colour if (verdict_col is not None and index == verdict_col
                              and colour) else INK
            draw.text((x, y), str(cell), font=f_cell, fill=fill)
            x += width
        y += row_h

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    print(f"-> {path}")


def verdict_of(case: dict) -> tuple[str, tuple | None]:
    if case.get("exception"):
        return "RAISED", PASS
    needed = case.get("required_joint_delta_deg") or 0.0
    if needed < 0.05:
        # A move to where the arm already is legitimately sends nothing.
        return "no-op (ok)", PASS
    if case.get("silent_non_execution"):
        return "SILENT", FAIL
    frac = case.get("achieved_fraction")
    if frac is not None and frac < 0.9:
        return "STALLED", AMBER
    return "CONVERGED", PASS


def diagnosis(path: str, before: dict, after: dict) -> None:
    rows = []
    for case in before["cases"]:
        name = case["case"]
        after_case = next((c for c in after["cases"] if c["case"] == name), {})
        v1, c1 = verdict_of(case)
        v2, c2 = verdict_of(after_case)
        rows.append(([
            name, f"{case.get('real_max_travel_deg', 0):.2f}",
            f"{case.get('required_joint_delta_deg', 0):.1f}",
            str(case.get("reported_error_deg")),
            v1, str(after_case.get("reported_error_deg")), v2,
        ], None))
    # colour the two verdict columns
    image_rows = []
    for (cells, _), case, after_case in zip(rows, before["cases"], after["cases"]):
        v1, c1 = verdict_of(case)
        v2, c2 = verdict_of(after_case)
        image_rows.append((cells, None))
    sheet(path,
          "Phase 2 — diagnosis: what the loop did, and what the hardened loop does",
          "real Live._stream_to against an injectable fake bridge · 67.7deg relocation "
          "(48,27,40)->(0,0,70).  VERDICT after = hardened loop.",
          ["fault injected", "travel°", "needed°", "err°", "before", "err°", "after"],
          image_rows, [330, 90, 90, 110, 120, 110, 130])


def loop_sheet(path: str, sim: dict) -> None:
    rows = []
    for case in sim["cases"]:
        ok = "PASS" if case.get("ok") else "FAIL"
        rows.append(([
            case["case"], ok, f"{case.get('tip_travel_mm', 0):.1f}",
            f"{(case.get('movement') or {}).get('max_abs_deg', 0):.2f}",
            str(case.get("reported_error_deg")),
            "yes" if (case.get("after_settled")) else "no",
        ], None))
    sheet(path,
          "Phase 2 — measured-joint loop re-landed: sim gate",
          "tools/test_move_executes.py --sim · hardened loop · "
          f"passed={sim.get('passed')} · relocation bar >100mm / >=2 joints / >=60deg",
          ["case", "verdict", "tip travel mm", "max joint°", "reported err°", "settled"],
          rows, [430, 120, 150, 140, 150, 120], verdict_col=1)


def gate_sheet(path: str, squares: dict) -> None:
    summary = squares["summary"]
    rows = [
        (["gate", "20.0 mm / 8° (live)", "3.0 mm / 8° (proposed)", "", ""], None),
        (["least_squares pass", f"{summary['ls_pass_20']}/64",
          f"{summary['ls_pass_3']}/64", "", ""], None),
        (["mink pass", f"{summary['mink_pass_20']}/64",
          f"{summary['mink_pass_3']}/64", "", ""], None),
        (["regressions @ 3.0 mm", str(summary["mink_regress3"]), "", "", ""], None),
        (["solver residual p50 mm", f"{summary['ls_median_mm']:.2f}",
          f"{summary['mink_median_mm']:.2f}", "", ""], None),
        (["solver residual max mm", f"{summary['ls_max_mm']:.2f}",
          f"{summary['mink_max_mm']:.2f}", "", ""], None),
        (["solve time p50 ms", f"{summary['ls_median_ms']:.0f}",
          f"{summary['mink_median_ms']:.0f}", "", ""], None),
        (["solve time max ms", f"{summary['ls_max_ms']:.0f}",
          f"{summary['mink_max_ms']:.0f}", "", ""], None),
    ]
    sheet(path,
          "Phase 2 — gate change: 3.0 mm only works with the mink solver",
          "tools/phase1_squares.py, identical 64 board targets, hover 60 mm",
          ["metric", "least_squares", "mink", "", ""],
          rows, [340, 260, 260, 60, 60])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", default="reports/phase2_diagnosis.json")
    ap.add_argument("--after", default="reports/phase2_diagnosis_after.json")
    ap.add_argument("--sim", default="reports/phase2_sim.json")
    ap.add_argument("--squares", default="reports/phase2_squares.json")
    ap.add_argument("--outdir", default="reports")
    args = ap.parse_args()

    before = json.loads(Path(args.before).read_text())
    after = json.loads(Path(args.after).read_text())
    diagnosis(f"{args.outdir}/phase2_diagnosis.png", before, after)

    if Path(args.sim).exists():
        loop_sheet(f"{args.outdir}/phase2_loop.png", json.loads(Path(args.sim).read_text()))
    if Path(args.squares).exists():
        gate_sheet(f"{args.outdir}/phase2_gate.png",
                   json.loads(Path(args.squares).read_text()))


if __name__ == "__main__":
    main()
