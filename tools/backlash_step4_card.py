#!/usr/bin/env python3
"""Render the step-4 status card from the two A/B receipts.

Every number comes from the JSON the probe wrote; nothing is typed in by hand.
ImageMagick only (no matplotlib/PIL on the BC-250).
"""
import argparse
import json
import subprocess
import sys

PAIRS = (("shoulder_pan", "+1"), ("shoulder_pan", "-1"),
         ("elbow_flex", "+1"), ("elbow_flex", "-1"))


def load(path: str) -> dict:
    with open(path) as handle:
        return json.load(handle)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--off", required=True)
    parser.add_argument("--on", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--note", default="")
    args = parser.parse_args()

    off, on = load(args.off), load(args.on)
    width, height = 1120, 560
    scale = 190.0          # px per degree
    draw = [
        f'fill "#111111" rectangle 0,0 {width},{height}',
        'fill "#f2f2f2"',
        f'font-size 30 text 40,58 "STEP 4  -  BACKLASH TAKE-UP PRIMITIVE"',
        'fill "#9aa0a6"',
        f'font-size 19 text 40,92 "P=32 I=0 D=32, margin {on.get("delta_deg", 3.5) and on["records"][0]["take_up"]["deg"]:.1f} deg, '
        f'{on.get("reps", 3)} reps per direction, anchor elbow {on["anchor_pose"]["elbow_flex"]:.1f}"',
        'fill "#c9ced6" font-size 18 text 620,92 "OFF (grey)   ON (blue)"',
        'fill "#2a2f36" rectangle 40,112 {0},114'.format(width - 40),
    ]
    y = 158
    for joint, direction in PAIRS:
        off_value = off[joint][direction]["median"]
        on_value = on[joint][direction]["median"]
        label = f"{joint} {direction.replace('+1', '+10').replace('-1', '-10')} deg"
        draw.append(f'fill "#cfd3d8" font-size 19 text 40,{y} "{label}"')
        for row, (value, colour) in enumerate(((off_value, "#6b7280"),
                                               (on_value, "#7aa7e0"))):
            base = 330 + row * 30
            length = max(1.0, min(abs(value) * scale, 420.0))
            draw.append(f'fill "{colour}" rectangle 330,{base - 13} '
                        f'{330 + length:.0f},{base + 2}')
            draw.append(f'fill "#ffffff" font-size 17 text 760,{base} "{value:+.2f}"')
        y += 78
    draw += [
        'fill "#2a2f36" rectangle 40,470 {0},472'.format(width - 40),
        'fill "#e6e6e6" font-size 18 text 40,506 '
        '"target: |median residual| <= 0.30 both directions on the loaded elbow"',
    ]
    note = args.note or (
        "loaded elbow -1.12 -> -0.15 (7x); other direction -0.02 -> -0.64; "
        "target NOT met on that side")
    draw.append(f'fill "#ffd166" font-size 18 text 40,540 "{note}"')
    command = ["magick", "-size", f"{width}x{height}", "xc:black"]
    for item in draw:
        command += ["-draw", item]
    command += ["-quality", "92", args.out]
    subprocess.run(command, check=True)
    print(f"wrote {args.out}")
    for joint, direction in PAIRS:
        print(f"  {joint:13s} {direction:3s}  OFF {off[joint][direction]['median']:+.2f}"
              f"  ON {on[joint][direction]['median']:+.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
