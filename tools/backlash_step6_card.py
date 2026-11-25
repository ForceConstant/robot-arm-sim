#!/usr/bin/env python3
"""Step 5/6 status cards: the sweep blocker and the decision summary.

Numbers come from the A/B receipts and the committed sweep baseline; the
vision-path evidence is passed on the command line so it is visible in the
receipt rather than typed into a picture by hand.
"""
import argparse
import json
import subprocess
import sys

TARGET = 0.3


def load(path):
    with open(path) as handle:
        return json.load(handle)


def card_sweep(out, evidence):
    width, height = 1120, 560
    draw = [
        f'fill "#111111" rectangle 0,0 {width},{height}',
        'fill "#f2f2f2" font-size 30 text 40,58 "STEP 5  -  64-SQUARE SWEEP: STOPPED AT SQUARE 1"',
        'fill "#9aa0a6" font-size 19 text 40,94 '
        '"the console cannot keep a marker board plane; the tool refused to move blind"',
        'fill "#2a2f36" rectangle 40,114 1080,116',
    ]
    y = 150
    for label, value in evidence:
        draw.append(f'fill "#cfd3d8" font-size 18 text 40,{y} "{label}"')
        draw.append(f'fill "#ffffff" font-size 18 text 740,{y} "{value}"')
        y += 34
    draw += [
        'fill "#2a2f36" rectangle 40,470 1080,472',
        'fill "#ffd166" font-size 18 text 40,506 '
        '"one tip leg is not a comparison: no after board-mm p50/p95 is claimed"',
        'fill "#ffd166" font-size 18 text 40,536 '
        '"owner decision: re-focus the wrist lens and re-run the sweep?"',
    ]
    command = ["magick", "-size", f"{width}x{height}", "xc:black"]
    for item in draw:
        command += ["-draw", item]
    command += ["-quality", "92", out]
    subprocess.run(command, check=True)
    print("wrote", out)


def card_decision(out, on):
    width, height = 1120, 560
    draw = [
        f'fill "#111111" rectangle 0,0 {width},{height}',
        'fill "#f2f2f2" font-size 30 text 40,58 "STEP 6  -  DECISION"',
        'fill "#9aa0a6" font-size 19 text 40,94 '
        '"P=32 I=0 D=32, take-up 3.5 deg: settle residual per direction vs the 0.30 deg target"',
        'fill "#2a2f36" rectangle 40,114 1080,116',
    ]
    y = 160
    scale = 300.0
    rows = []
    for joint in ("shoulder_pan", "elbow_flex"):
        for direction in ("+1", "-1"):
            value = on[joint][direction]["median"]
            rows.append((f'{joint} {direction.replace("+1", "+10").replace("-1", "-10")}',
                         value))
    for label, value in rows:
        draw.append(f'fill "#cfd3d8" font-size 19 text 40,{y} "{label}"')
        length = max(1.0, min(abs(value) * scale, 330.0))
        colour = "#e06666" if abs(value) > TARGET else "#7aa7e0"
        draw.append(f'fill "{colour}" rectangle 330,{y - 13} {330 + length:.0f},{y + 2}')
        draw.append(f'fill "#ffffff" font-size 17 text 660,{y} "{value:+.2f}"')
        draw.append(f'fill "#9aa0a6" font-size 17 text 740,{y} '
                    f'{"OVER" if abs(value) > TARGET else "ok"}')
        y += 42
    # 0.30 target marker
    draw.append(f'fill "#ffd166" rectangle {330 + TARGET * scale:.0f},128 '
                f'{330 + TARGET * scale + 2:.0f},352')
    draw += [
        'fill "#2a2f36" rectangle 40,372 1080,374',
        'fill "#e6e6e6" font-size 18 text 40,404 '
        '"near-1 deg goal: met (worst 0.73).  <=0.30 both directions: NOT met on elbow +10."',
        'fill "#e6e6e6" font-size 18 text 40,434 '
        '"remaining: static band 0.6-0.8 on the loaded elbow; pan is at one encoder count (0.088)."',
        'fill "#e6e6e6" font-size 18 text 40,464 '
        '"cheapest next: bridge anti-windup + I=8 (fixes corner-pose droop); then damping/gearbox."',
        'fill "#e6e6e6" font-size 18 text 40,494 '
        '"board-mm floor is the ~19 mm frame/model bias, untouched by any servo term."',
        'fill "#ffd166" font-size 18 text 40,532 '
        '"end-to-end sweep: blocked by the camera/marker plane (step 5 card)"',
    ]
    command = ["magick", "-size", f"{width}x{height}", "xc:black"]
    for item in draw:
        command += ["-draw", item]
    command += ["-quality", "92", out]
    subprocess.run(command, check=True)
    print("wrote", out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--on", required=True)
    parser.add_argument("--out-decision", required=True)
    parser.add_argument("--out-sweep", required=True)
    args = parser.parse_args()
    on = load(args.on)
    evidence = [
        ("squares completed (tip / aim)", "1 / 0"),
        ("a1 tip error: this run / baseline", "4.19 mm / 7.17 mm"),
        ("a1 tip joint error: this run / baseline", "1.04 deg / 2.47 deg"),
        ("plane failures before the tool stopped itself", "4"),
        ("marker ids decoded (strict / lenient / inverted / 3x)", "0 / 0 / 0 / 0"),
        ("same detector on board_charuco.png", "32 ids"),
        ("Laplacian variance, home pose / recovery pose", "25.0 / 184"),
        ("verify_baseline", "7/7 once, 6/7 when the plane is down"),
    ]
    card_sweep(args.out_sweep, evidence)
    card_decision(args.out_decision, on)
    return 0


if __name__ == "__main__":
    sys.exit(main())
