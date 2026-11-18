#!/usr/bin/env python3
"""Generate a ChArUco calibration target as DXF, with algebraic notation.

An 8x8 board of 28 mm squares matches the chess board exactly, so the target
can be laid on the playing surface and the derived coordinates line up with the
squares.  Markers sit inside the white squares; 32 of them from DICT_4X4_50,
numbered 5..36 so they never collide with the mat tags (0..3) or the loose
centre target (4).

Outputs (next to --out):
    *.dxf          R12 entities: filled SOLID regions plus TEXT labels
    *.json         layout the detector needs (square, marker size, ids, ids'
                   positions) so this is a *custom* ChArUco board, not OpenCV's
                   built-in pattern
    *_preview.png  raster of the same geometry for eyeballing before cutting

    robot-arm-sim python tools/charuco_dxf.py --out charuco_28mm

Everything is millimetres and the origin is the board centre, matching the live
board frame (+x files a->h, +y ranks 1->8, a1 dark).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

SQUARES = 8
SQUARE_MM = 28.0
MARKER_MM = 20.0          # 0.71 of the square: leaves a 4 mm white border
MODULES = 6               # 4 data modules + 1 border each side
DICT = cv2.aruco.DICT_4X4_50
FIRST_ID = 5
FIELD_MM = SQUARES * SQUARE_MM
MARGIN_MM = 10.0          # notation band
SHEET_MM = FIELD_MM + 2 * MARGIN_MM
FRAME_MM = 0.6            # printed line around the playing field

FILES = "abcdefgh"


def marker_bitmap(marker_id: int) -> np.ndarray:
    """6x6 module matrix, True where the module is BLACK."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICT)
    image = cv2.aruco.generateImageMarker(dictionary, marker_id, MODULES * 20)
    small = cv2.resize(image, (MODULES, MODULES), interpolation=cv2.INTER_AREA)
    return small < 128


def build() -> dict:
    """Geometry in board-frame millimetres."""
    half = FIELD_MM / 2.0
    black_squares, markers = [], []
    marker_id = FIRST_ID
    for row in range(SQUARES):          # row 0 == rank 1
        for col in range(SQUARES):      # col 0 == file a
            x0 = -half + col * SQUARE_MM
            y0 = -half + row * SQUARE_MM
            dark = (col + row) % 2 == 0  # a1 is dark
            if dark:
                black_squares.append((x0, y0, x0 + SQUARE_MM, y0 + SQUARE_MM))
                continue
            markers.append({"id": marker_id, "col": col, "row": row,
                            "cx": x0 + SQUARE_MM / 2.0,
                            "cy": y0 + SQUARE_MM / 2.0})
            marker_id += 1
    return {"black_squares": black_squares, "markers": markers,
            "field_mm": FIELD_MM, "sheet_mm": SHEET_MM,
            "square_mm": SQUARE_MM, "marker_mm": MARKER_MM,
            "modules": MODULES, "dictionary": "DICT_4X4_50",
            "first_id": FIRST_ID, "marker_count": len(markers)}


# ------------------------------------------------------------------ DXF writer
def _solid(out: list[str], x0: float, y0: float, x1: float, y1: float) -> None:
    """Axis-aligned filled rectangle.

    DXF SOLID takes its four corners in a Z order (3rd and 4th swapped relative
    to polygon order); getting that wrong draws a bow tie instead of a square.
    """
    out.extend(["0", "SOLID", "8", "BLACK",
                "10", f"{x0:.3f}", "20", f"{y0:.3f}", "30", "0.0",
                "11", f"{x1:.3f}", "21", f"{y0:.3f}", "31", "0.0",
                "12", f"{x0:.3f}", "22", f"{y1:.3f}", "32", "0.0",
                "13", f"{x1:.3f}", "23", f"{y1:.3f}", "33", "0.0"])


def _text(out: list[str], x: float, y: float, height: float, value: str) -> None:
    """Centre-aligned single-line text."""
    out.extend(["0", "TEXT", "8", "TEXT", "10", f"{x:.3f}", "20", f"{y:.3f}",
                "30", "0.0", "40", f"{height:.3f}", "1", value,
                "7", "STANDARD", "72", "1", "11", f"{x:.3f}", "21", f"{y:.3f}",
                "31", "0.0"])


def _polyline(out: list[str], points: list[tuple[float, float]]) -> None:
    out.extend(["0", "POLYLINE", "8", "OUTLINE", "66", "1",
                "10", "0.0", "20", "0.0", "30", "0.0", "70", "1"])
    for x, y in points:
        out.extend(["0", "VERTEX", "8", "OUTLINE",
                    "10", f"{x:.3f}", "20", f"{y:.3f}", "30", "0.0"])
    out.extend(["0", "SEQEND", "8", "OUTLINE"])


def write_dxf(path: Path, geometry: dict) -> None:
    out: list[str] = []
    half_sheet = SHEET_MM / 2.0
    out += ["0", "SECTION", "2", "HEADER",
            "9", "$INSUNITS", "70", "4",            # millimetres
            "9", "$EXTMIN", "10", f"{-half_sheet:.3f}", "20", f"{-half_sheet:.3f}",
            "30", "0.0",
            "9", "$EXTMAX", "10", f"{half_sheet:.3f}", "20", f"{half_sheet:.3f}",
            "30", "0.0",
            "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]

    _polyline(out, [(-half_sheet, -half_sheet), (half_sheet, -half_sheet),
                    (half_sheet, half_sheet), (-half_sheet, half_sheet),
                    (-half_sheet, -half_sheet)])

    # printed frame around the playing field
    # The frame sits strictly OUTSIDE the playing field: any overlap would eat
    # into the outer squares and falsify the 28 mm geometry the board model
    # assumes.
    half = geometry["field_mm"] / 2.0
    edge = half + FRAME_MM
    _solid(out, -edge, half, edge, edge)      # top
    _solid(out, -edge, -edge, edge, -half)    # bottom
    _solid(out, -edge, -half, -half, half)    # left
    _solid(out, half, -half, edge, half)      # right

    for x0, y0, x1, y1 in geometry["black_squares"]:
        _solid(out, x0, y0, x1, y1)

    half_field = geometry["field_mm"] / 2.0
    module_mm = geometry["marker_mm"] / geometry["modules"]
    for marker in geometry["markers"]:
        bits = marker_bitmap(marker["id"])
        origin_x = marker["cx"] - geometry["marker_mm"] / 2.0
        origin_y = marker["cy"] + geometry["marker_mm"] / 2.0   # top edge
        for r in range(MODULES):
            for c in range(MODULES):
                if not bits[r][c]:
                    continue
                x0 = origin_x + c * module_mm
                y1 = origin_y - r * module_mm
                _solid(out, x0, y1 - module_mm, x0 + module_mm, y1)

    # algebraic notation, outside the playing field
    text_h = 7.5
    band = MARGIN_MM - FRAME_MM
    offset = half_field + FRAME_MM + band / 2.0
    for col, letter in enumerate(FILES):
        cx = -half_field + (col + 0.5) * geometry["square_mm"]
        _text(out, cx, -offset, text_h, letter)
        _text(out, cx, offset, text_h, letter)
    for row in range(SQUARES):
        cy = -half_field + (row + 0.5) * geometry["square_mm"]
        _text(out, -offset, cy - text_h / 2.0, text_h, str(row + 1))
        _text(out, offset, cy - text_h / 2.0, text_h, str(row + 1))

    out += ["0", "ENDSEC", "0", "EOF"]
    path.write_text("\n".join(out) + "\n")


def _anchor(x: float, y: float, ppm: float, height_mm: float,
            align: float) -> tuple[int, int]:
    """Pixel position for cv2 text, roughly centred like the DXF TEXT entity."""
    width_px = 0.62 * height_mm * ppm * 4
    return (int(round((x + SHEET_MM / 2) * ppm - width_px * align / 2.0)),
            int(round((SHEET_MM / 2 - y) * ppm + 0.35 * height_mm * ppm)))


def write_preview(path: Path, geometry: dict, pixels_per_mm: float = 4.0) -> None:
    sheet_px = int(round(SHEET_MM * pixels_per_mm))
    image = np.full((sheet_px, sheet_px), 255, np.uint8)
    def to_px(x: float, y: float) -> tuple[int, int]:
        return (int(round((x + SHEET_MM / 2) * pixels_per_mm)),
                int(round((SHEET_MM / 2 - y) * pixels_per_mm)))
    half = geometry["field_mm"] / 2.0
    thickness = max(1, int(round(FRAME_MM * pixels_per_mm)))
    cv2.rectangle(image, to_px(-half - FRAME_MM, half + FRAME_MM),
                  to_px(half + FRAME_MM, -half - FRAME_MM), 0, thickness)
    # inner edge of the frame lines up with the field boundary
    for x0, y0, x1, y1 in geometry["black_squares"]:
        cv2.rectangle(image, to_px(x0, y1), to_px(x1, y0), 0, -1)
    module = geometry["marker_mm"] / geometry["modules"]
    for marker in geometry["markers"]:
        bits = marker_bitmap(marker["id"])
        ox = marker["cx"] - geometry["marker_mm"] / 2.0
        oy = marker["cy"] + geometry["marker_mm"] / 2.0
        for r in range(geometry["modules"]):
            for c in range(geometry["modules"]):
                if not bits[r][c]:
                    continue
                cv2.rectangle(image,
                              to_px(ox + c * module, oy - r * module),
                              to_px(ox + (c + 1) * module, oy - (r + 1) * module),
                              0, -1)
    # notation and scale bar, so the preview shows what the DXF contains
    half_field = geometry["field_mm"] / 2.0
    for col, letter in enumerate(FILES):
        x = -half_field + (col + 0.5) * geometry["square_mm"]
        cv2.putText(image, letter, _anchor(x, -half_field - MARGIN_MM * 0.55,
                                          pixels_per_mm, 5.0, 0.5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, 0, 1, cv2.LINE_AA)
        cv2.putText(image, letter, _anchor(x, half_field + MARGIN_MM * 0.45,
                                          pixels_per_mm, 5.0, 0.5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, 0, 1, cv2.LINE_AA)
    for row in range(SQUARES):
        y = -half_field + (row + 0.5) * geometry["square_mm"]
        cv2.putText(image, str(row + 1), _anchor(-half_field - MARGIN_MM * 0.5, y,
                                                pixels_per_mm, 5.0, 0.5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, 0, 1, cv2.LINE_AA)
        cv2.putText(image, str(row + 1), _anchor(half_field + MARGIN_MM * 0.5, y,
                                                pixels_per_mm, 5.0, 0.5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, 0, 1, cv2.LINE_AA)
    cv2.imwrite(str(path), image)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="charuco_28mm",
                        help="output stem (files get .dxf/.json/_preview.png)")
    parser.add_argument("--verify", action="store_true",
                        help="decode the rendered preview and report marker ids")
    args = parser.parse_args()
    geometry = build()
    stem = Path(args.out)
    write_dxf(stem.with_suffix(".dxf"), geometry)
    write_preview(stem.parent / f"{stem.name}_preview.png", geometry)
    layout = {"kind": "charuco", "dictionary": geometry["dictionary"],
              "squares": SQUARES, "square_mm": SQUARE_MM,
              "marker_mm": MARKER_MM, "modules": MODULES,
              "field_mm": FIELD_MM, "sheet_mm": SHEET_MM,
              "origin": "board centre, +x files a->h, +y ranks 1->8, a1 dark",
              "markers": [{"id": m["id"], "col": m["col"], "row": m["row"],
                           "cx_mm": round(m["cx"], 3), "cy_mm": round(m["cy"], 3)}
                          for m in geometry["markers"]]}
    stem.with_suffix(".json").write_text(json.dumps(layout, indent=2) + "\n")
    if args.verify:
        preview = cv2.imread(str(stem.parent / f"{stem.name}_preview.png"))
        gray = cv2.cvtColor(preview, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(DICT),
            cv2.aruco.DetectorParameters())
        _, ids, _ = detector.detectMarkers(gray)
        found = sorted(int(i) for i in (ids.ravel() if ids is not None else []))
        expected = sorted(m["id"] for m in geometry["markers"])
        print(f"decoded {len(found)}/{len(expected)} markers from the render: {found}")
        missing = [i for i in expected if i not in found]
        print("missing:", missing if missing else "none")
    print(f"{stem.with_suffix('.dxf')}  {geometry['marker_count']} markers, "
          f"{len(geometry['black_squares'])} dark squares, {SHEET_MM:.0f} mm sheet")


if __name__ == "__main__":
    main()
