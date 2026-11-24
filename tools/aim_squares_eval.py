#!/usr/bin/env python3
"""Evaluation of an ``aim_squares.py`` receipt: tip mode vs aim mode.

The sweep records two protocols on the same squares in one session:

``tip``       ``mode=xyz``  -- tip commanded to the square centre
``aim_point`` ``mode=aim_point`` -- the aim ray commanded to the square centre
``aim_final`` the state after the deadband-compensation refinement, where run

This tool reports all three, the paired comparison, the pixel evidence against
the marker-projected grid, and draws the comparison heatmap.  It performs no
motion and needs only the receipt.

    PYTHONPATH=. python tools/aim_squares_eval.py \\
        --receipt reports/aim_squares_20261124.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import aim_squares as sweep                                  # noqa: E402

FILES = "abcdefgh"


def detail(receipt: dict, leg: str) -> dict[str, dict]:
    out = {}
    for name, record in (receipt.get("squares") or {}).items():
        entry = record.get(leg) or {}
        if entry.get("done") and entry.get("ok"):
            out[name] = entry
    return out


def values(records: dict[str, dict], key: str) -> dict[str, float]:
    out = {}
    for name, entry in records.items():
        value = entry.get(key)
        if value is not None and np.isfinite(float(value)):
            out[name] = float(value)
    return out


def block(vals) -> dict:
    return sweep.summarise(vals.values() if isinstance(vals, dict) else vals)


def grid(receipt: dict, leg: str, key: str) -> np.ndarray:
    array = np.full((8, 8), np.nan)
    rows = receipt.get("squares") or {}
    for rank in range(1, 9):
        for index, file in enumerate(FILES):
            entry = (rows.get(f"{file}{rank}") or {}).get(leg) or {}
            value = entry.get(key)
            if entry.get("done") and entry.get("ok") and value is not None:
                array[rank - 1, index] = float(value)
    return array


def draw(receipt: dict, path: Path) -> bool:
    import cv2
    cell = 62
    width = 2 * (8 * cell) + 120
    height = 2 * (8 * cell) + 230
    canvas = np.full((height, width, 3), 20, np.uint8)
    tip = detail(receipt, "tip")
    aim = detail(receipt, "aim")
    panels = (
        ("tip", "aim_err_mm", "TIP mode: |aim - commanded centre| (mm)", 20.0),
        ("aim_point", "final_error_mm", "AIM mode: |aim - commanded centre| (mm)", 20.0),
        ("tip", "pixel_delta", "TIP mode: aim px vs marker grid centre", 140.0),
        ("aim", "pixel_delta", "AIM mode: aim px vs marker grid centre", 140.0),
    )
    for index, (leg, key, title, vmax) in enumerate(panels):
        column, row = index % 2, index // 2
        origin = (40 + column * (8 * cell + 60), 46 + row * (8 * cell + 95))
        if leg == "aim_point":
            array = np.full((8, 8), np.nan)
            for name, entry in aim.items():
                value = entry.get("final_error_mm")
                if value is None:
                    continue
                rank, file = int(name[1]), FILES.index(name[0])
                array[rank - 1, file] = float(value)
        else:
            array = grid(receipt, leg, key)
        sweep._panel(canvas, origin, cell, array, title, vmax)
    sweep._colourbar(canvas, (40, height - 55), 8 * cell, 20.0)
    sweep._colourbar(canvas, (40 + 8 * cell + 60, height - 55), 8 * cell, 140.0)
    a = block(values(aim, "final_error_mm"))
    t = block(values(tip, "aim_err_mm"))
    header = (f"aim sweep {receipt.get('stamp', '')}   TIP p50 {t.get('p50')} mm   "
              f"AIM (aim_point) p50 {a.get('p50')} mm   "
              f"n tip {t.get('n')} / aim {a.get('n')}")
    cv2.putText(canvas, header, (40, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (225, 232, 240), 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(path), canvas))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--receipt",
                        default=str(ROOT / "reports" / "aim_squares_20261124.json"))
    parser.add_argument("--out",
                        default=str(ROOT / "reports" / "aim_squares_compare.png"))
    parser.add_argument("--report",
                        default=str(ROOT / "reports" / "aim_squares_20261124_report.md"))
    args = parser.parse_args()

    receipt = json.loads(Path(args.receipt).read_text())
    tip = detail(receipt, "tip")
    aim = detail(receipt, "aim")

    tip_err = values(tip, "aim_err_mm")
    aim_point_err = values(aim, "final_error_mm")
    aim_final_err = values(aim, "aim_err_mm")
    tip_px = values(tip, "pixel_delta")
    aim_px = values(aim, "pixel_delta")
    aim_cfg_px = values(aim, "pixel_delta_cfg")
    tip_cfg_px = values(tip, "pixel_delta_cfg")

    paired = [(name, tip_err[name], aim_point_err[name])
              for name in tip_err if name in aim_point_err]
    diffs = np.array([t - a for _, t, a in paired], float) if paired else np.array([])

    # Marker grid vs the console's configured board frame, in pixels, per leg.
    offsets = {"tip": [], "aim": []}
    for leg, records in (("tip", tip), ("aim", aim)):
        for entry in records.values():
            if entry.get("square_uv") and entry.get("square_uv_cfg"):
                offsets[leg].append([entry["square_uv"][0] - entry["square_uv_cfg"][0],
                                     entry["square_uv"][1] - entry["square_uv_cfg"][1]])
    offset_stats = {}
    for leg, rows in offsets.items():
        if not rows:
            continue
        array = np.array(rows, float)
        offset_stats[leg] = {
            "n": len(rows),
            "mean_px": [round(float(v), 1) for v in array.mean(axis=0)],
            "sd_px": [round(float(v), 1) for v in array.std(axis=0)],
            "dist_p50_px": round(float(np.percentile(np.hypot(array[:, 0], array[:, 1]), 50)), 1),
            "dist_max_px": round(float(np.hypot(array[:, 0], array[:, 1]).max()), 1),
        }

    # Deadband evidence: the first aim_point step and what it actually achieved.
    first_steps = []
    for entry in aim.values():
        steps = entry.get("steps") or []
        if len(steps) >= 2 and steps[0].get("step_mm"):
            first_steps.append((float(steps[0]["error_mm"]),
                                float(steps[0]["step_mm"]),
                                float(steps[1]["error_mm"])))
    deadband = None
    if first_steps:
        error0 = np.array([row[0] for row in first_steps])
        error1 = np.array([row[2] for row in first_steps])
        deadband = {"n": len(first_steps),
                    "commanded_p50_mm": round(float(np.median(
                        [row[1] for row in first_steps])), 2),
                    "achieved_p50_mm": round(float(np.median(error0 - error1)), 2),
                    "unchanged_or_worse": int((error1 >= error0 - 0.5).sum())}

    refine = [(name, aim[name].get("final_error_mm"),
               (aim[name].get("refine") or {}).get("final_error_mm"))
              for name in aim if (aim[name].get("refine") or {}).get("ok")]
    refine_counts = {"ran": len(refine),
                     "better": sum(1 for _, a, b in refine if b is not None and b < a - 0.2),
                     "worse": sum(1 for _, a, b in refine if b is not None and b > a + 0.2)}
    refusals = []
    for name, record in (receipt.get("squares") or {}).items():
        for leg in ("tip", "aim"):
            entry = record.get(leg) or {}
            if entry.get("refused"):
                refusals.append((name, leg, entry["refused"]))
            refinement = entry.get("refine") or {}
            if refinement.get("refused"):
                refusals.append((name, leg + "/refine", refinement["refused"]))
        for failure in record.get("plane_failures") or []:
            refusals.append((name, "plane", failure.get("reason", "")))

    summary = {
        "tip_err_mm": block(tip_err),
        "aim_point_err_mm": block(aim_point_err),
        "aim_final_err_mm": block(aim_final_err),
        "tip_px": block(tip_px),
        "aim_px": block(aim_px),
        "tip_cfg_px": block(tip_cfg_px),
        "aim_cfg_px": block(aim_cfg_px),
        "within_5mm": {"tip": sum(1 for v in tip_err.values() if v <= 5.0),
                       "aim": sum(1 for v in aim_point_err.values() if v <= 5.0)},
        "within_10mm": {"tip": sum(1 for v in tip_err.values() if v <= 10.0),
                        "aim": sum(1 for v in aim_point_err.values() if v <= 10.0)},
        "paired": {"n": len(paired),
                   "aim_better": int((diffs > 0).sum()),
                   "mean_improvement_mm": round(float(diffs.mean()), 2) if diffs.size else None,
                   "median_improvement_mm": round(float(np.median(diffs)), 2) if diffs.size else None},
        "deadband": deadband,
        "refine": refine_counts,
        "frame_offset": offset_stats,
        "refusals": refusals,
        "readout_spread_mm": block(values(aim, "aim_board_spread_mm")),
    }

    made = draw(receipt, Path(args.out))

    lines = ["# Aim-ray 64-square sweep — tip mode vs aim mode", "",
             f"Session {receipt.get('started')} → {receipt.get('finished')}; "
             f"z {receipt.get('z_mm')} mm tip height; tolerance "
             f"{receipt.get('tolerance_mm')} mm; {receipt.get('base')}.", "",
             "Board frame (console): " + json.dumps(receipt.get("board_frame")), "",
             "## Columns", "",
             "| column | what was commanded | what is measured |",
             "|---|---|---|",
             "| tip | jaw tip → square centre (mode xyz) | aim readout after the move |",
             "| aim_point | aim ray → square centre (mode aim_point) | the app's own final error |",
             "| aim_final | + deadband-compensated rel refinement | independent 3-sample readout |",
             "", "## Accuracy", "",
             "| metric | n | p50 | p95 | max | mean | ≤5 mm | ≤10 mm |",
             "|---|---|---|---|---|---|---|---|"]
    for label, key, counts in (
            ("tip |aim − centre| mm", "tip_err_mm", summary["within_5mm"]["tip"]),
            ("aim_point |aim − centre| mm", "aim_point_err_mm", summary["within_5mm"]["aim"]),
            ("aim_final |aim − centre| mm", "aim_final_err_mm", None)):
        stats = summary[key]
        lines.append(f"| {label} | {stats['n']} | {stats['p50']} | {stats['p95']} | "
                     f"{stats['max']} | {stats['mean']} | "
                     f"{counts if counts is not None else '—'} | "
                     f"{summary['within_10mm']['tip' if key == 'tip_err_mm' else 'aim'] if counts is not None else '—'} |")
    lines += ["",
              f"Paired (n {summary['paired']['n']}): aim_point better on "
              f"{summary['paired']['aim_better']} squares; median improvement "
              f"{summary['paired']['median_improvement_mm']} mm, mean "
              f"{summary['paired']['mean_improvement_mm']} mm.", "",
              "## Pixel evidence (marker-projected square centre)", "",
              "| metric | n | p50 | p95 | max |", "|---|---|---|---|---|"]
    for label, key in (("tip aim px", "tip_px"), ("aim aim px", "aim_px"),
                       ("tip px vs configured", "tip_cfg_px"),
                       ("aim px vs configured", "aim_cfg_px")):
        stats = summary[key]
        lines.append(f"| {label} | {stats['n']} | {stats['p50']} | {stats['p95']} | {stats['max']} |")
    lines += ["", "Marker grid centre minus console's configured centre (px):", ""]
    for leg, stats in offset_stats.items():
        lines.append(f"- {leg}: n {stats['n']}, mean {stats['mean_px']} px, "
                     f"sd {stats['sd_px']} px, distance p50 {stats['dist_p50_px']} px, "
                     f"max {stats['dist_max_px']} px")
    if deadband:
        lines += ["", "## Servo deadband", "",
                  f"First aim_point step across {deadband['n']} squares: commanded p50 "
                  f"{deadband['commanded_p50_mm']} mm, achieved p50 "
                  f"{deadband['achieved_p50_mm']} mm; {deadband['unchanged_or_worse']} "
                  f"squares moved 0.5 mm or less."]
    lines += ["", "## Refinement", "",
              f"Ran on {refine_counts['ran']} squares: better on {refine_counts['better']}, "
              f"worse on {refine_counts['worse']}.", "",
              "## Refusals and plane failures (verbatim)", ""]
    lines += [f"- {name} {leg}: {reason}" for name, leg, reason in refusals]
    lines += ["", "## Per square", "",
              "| sq | tip mm | aim_point mm | aim_final mm | tip px | aim px |",
              "|---|---|---|---|---|---|"]
    for rank in range(1, 9):
        for file in FILES:
            name = f"{file}{rank}"
            entry_t = tip.get(name) or {}
            entry_a = aim.get(name) or {}
            def fmt(value):
                return "—" if value is None else f"{float(value):.1f}"
            lines.append(f"| {name} | {fmt(entry_t.get('aim_err_mm'))} | "
                         f"{fmt(entry_a.get('final_error_mm'))} | "
                         f"{fmt(entry_a.get('aim_err_mm'))} | "
                         f"{fmt(entry_t.get('pixel_delta'))} | "
                         f"{fmt(entry_a.get('pixel_delta'))} |")
    Path(args.report).write_text("\n".join(lines) + "\n")

    print(json.dumps(summary, indent=2, default=str))
    print(f"\ndrew {args.out} ({made})")
    print(f"wrote {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
