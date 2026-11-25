#!/usr/bin/env python3
"""Compare two aim-squares receipts: the metrics the sweep is judged on.

Both files are ``aim-squares/2`` receipts written by ``tools/aim_squares.py``.
Prints a side-by-side of the summary blocks plus the two counts the summary
does not carry: how many lands were within 5 mm, and how many squares completed
both legs.  Nothing here re-measures; it only reads what the runs wrote.

    python tools/aim_squares_compare.py before.json after.json
"""
from __future__ import annotations

import json
import sys


def _ok(value) -> bool:
    return value is not None


def landed_within(rows: list[float], limit: float) -> tuple[int, int]:
    clean = [v for v in rows if _ok(v)]
    return sum(1 for v in clean if v <= limit), len(clean)


def leg_errors(squares: dict, leg: str) -> list[float]:
    out = []
    for record in squares.values():
        block = record.get(leg) or {}
        if block.get("done") and block.get("error_deg") is not None:
            out.append(abs(float(block["error_deg"])))
    return out


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    before = json.load(open(sys.argv[1]))
    after = json.load(open(sys.argv[2]))

    rows = [
        ("aim board-mm p50", "aim_err_mm", "p50"),
        ("aim board-mm p95", "aim_err_mm", "p95"),
        ("aim board-mm max", "aim_err_mm", "max"),
        ("tip FK-mm p50", "tip_err_mm", "p50"),
        ("tip FK-mm p95", "tip_err_mm", "p95"),
        ("tip FK-mm max", "tip_err_mm", "max"),
        ("aim px p50 (marker)", "aim_px", "p50"),
        ("aim px p95 (marker)", "aim_px", "p95"),
        ("tip px p50 (marker)", "tip_px", "p50"),
        ("aim px p50 (configured)", "aim_px_cfg", "p50"),
        ("aim px p95 (configured)", "aim_px_cfg", "p95"),
    ]
    print(f"{'metric':28s} {'before':>10s} {'after':>10s}")
    for label, key, field in rows:
        b = ((before.get("summary") or {}).get(key) or {}).get(field)
        a = ((after.get("summary") or {}).get(key) or {}).get(field)
        print(f"{label:28s} {str(b):>10s} {str(a):>10s}")

    for label, receipt in (("before", before), ("after", after)):
        summary = receipt.get("summary") or {}
        aim_rows = [float((r.get("aim") or {}).get("executed", {}).get("error_deg"))
                    for r in (receipt.get("squares") or {}).values()
                    if (r.get("aim") or {}).get("executed")]
        tip_rows = leg_errors(receipt.get("squares") or {}, "tip")
        aim_sq = [float((r.get("aim") or {}).get("final_error_mm"))
                  for r in (receipt.get("squares") or {}).values()
                  if (r.get("aim") or {}).get("final_error_mm") is not None]
        within5, total5 = landed_within(aim_sq, 5.0)
        both = sum(1 for r in (receipt.get("squares") or {}).values()
                   if (r.get("tip") or {}).get("done") and (r.get("aim") or {}).get("done"))
        print(f"\n{label}: squares {summary.get('squares')} both-legs {both} "
              f"aim_done {summary.get('aim_done')} aim_ok {summary.get('aim_ok')} "
              f"aim_converged {summary.get('aim_converged')} "
              f"tip_done {summary.get('tip_done')} tip_ok {summary.get('tip_ok')} "
              f"tip_refused {summary.get('tip_refused')} "
              f"plane_failures {summary.get('plane_failures')}")
        print(f"        final aim <=5 mm: {within5}/{total5}   "
              f"tip leg error_deg: n {len(tip_rows)} "
              f"p50 {sorted(tip_rows)[len(tip_rows)//2]:.2f} max {max(tip_rows):.2f}"
              if tip_rows else "        no tip legs")
        failures = summary.get("failures") or []
        print(f"        failures {len(failures)}")
        for item in failures[:12]:
            print(f"          {item.get('square')} {item.get('leg')}: {item.get('reason')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
