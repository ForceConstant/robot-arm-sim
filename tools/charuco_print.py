#!/usr/bin/env python3
"""ChArUco target as printable pieces: split, lapped, flush, with notation.

Designed for a printer whose usable bed is smaller than the 244 mm sheet, and
for a multi-colour print where the top face must be flat.

    robot-arm-sim python tools/charuco_print.py --out charuco_28mm

Outputs, per piece, two STLs (white body and black top layer) plus a preview:

    charuco_28mm_L_white.stl / _black.stl
    charuco_28mm_R_white.stl / _black.stl
    charuco_28mm_L_preview.png ...

Geometry choices worth knowing:

*   The split runs along a square boundary (x = 0, between files d and e), so
    no square and no marker is cut in half.  The seam has no geometry error to
    correct for.
*   Halves are 122 x 244 mm: the 244 mm side runs the long way, but only 122 mm
    has to clear the exclusion zone, which is what a corner zone actually
    demands.  Rotate the piece in the slicer if your zone sits the other way.
*   The joint is a half-lap under the top face: the left piece carries a 4 mm
    tongue at 0..1.3 mm, the right piece is inset there, so they interlock and
    glue with the face still flush at 2.6 mm and the black layer at 3.2 mm.
*   Notation is real geometry: each label is rasterised and converted to
    run-length boxes, because an STL cannot carry text.
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

import cv2
import numpy as np

from charuco_dxf import (FILES, FRAME_MM, MARGIN_MM, SHEET_MM, SQUARES,
                         build, marker_bitmap, MODULES)

# Face-down stack: the painted face is printed on the bed, so the colour layer
# occupies z = 0..FACE_Z for BOTH filaments -- that is what makes black and
# white level with each other.  The body above it is one white solid, and the
# half-lap lives at the back, away from the bed.
FACE_Z = 0.6         # colour layer thickness, on the bed
MIRROR_FACE = True   # print face-down mirrors the face; pre-mirror to cancel it
TOTAL = 3.2          # total thickness
LAP_W = 4.0          # tongue width across the seam
LAP_Z0 = 1.9         # lap starts here; 1.3 mm of material to TOTAL
TEXT_H = 8.2          # ink height; the band between frame and edge is 9.4 mm
TEXT_STEP = 0.10     # rasterisation pitch for glyphs, mm
TEXT_STROKE = 0.80   # stroke width: bold enough for a Classic wall generator


# --------------------------------------------------------------------- meshes
def _box(tris: list[np.ndarray], x0, y0, z0, x1, y1, z1) -> None:
    if x1 - x0 <= 1e-9 or y1 - y0 <= 1e-9 or z1 - z0 <= 1e-9:
        return
    c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    for a, b, cc, d in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                        (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        p = [np.array(c[i], dtype=np.float64) for i in (a, b, cc, d)]
        tris.append(np.vstack([p[0], p[1], p[2]]))
        tris.append(np.vstack([p[0], p[2], p[3]]))


def _write_stl(path: Path, tris: list[np.ndarray]) -> None:
    with path.open("wb") as fh:
        fh.write(b"charuco piece".ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(tris)))
        for tri in tris:
            n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            ln = float(np.linalg.norm(n))
            n = n / ln if ln > 1e-12 else np.zeros(3)
            fh.write(struct.pack("<3f", *n))
            for v in tri:
                fh.write(struct.pack("<3f", *v))
            fh.write(struct.pack("<H", 0))


def _volume(tris: list[np.ndarray]) -> float:
    return abs(sum(float(np.dot(t[0], np.cross(t[1], t[2]))) for t in tris) / 6.0)


def _bounds(tris: list[np.ndarray]) -> tuple[list[float], list[float]]:
    pts = np.vstack(tris)
    return ([round(float(v), 2) for v in pts.min(axis=0)],
            [round(float(v), 2) for v in pts.max(axis=0)])


# ---------------------------------------------------------------- glyph boxes
def _glyph_boxes(text: str, x_mm: float, y_mm: float, size_mm: float,
                 align: str = "centre") -> list[tuple[float, float, float, float]]:
    """Rasterise one label and return merged run boxes in millimetres.

    Two passes: render, measure the ink, then rescale so the glyph lands at
    exactly ``size_mm`` tall.  OpenCV's font scale has no fixed relation to cap
    height, so a guessed scale silently renders well under the requested size.
    """
    def render(px_per_mm: float, canvas_mm: float) -> np.ndarray:
        """Rasterise the glyph on a canvas that is deliberately generous.

        Clipping the glyph and then measuring it is how the tops of the digits
        went missing: the measurement was of ink that had already been cut off,
        so the rescale pass kept enlarging a truncated letter.
        """
        span = int(round(canvas_mm * px_per_mm))
        canvas = np.zeros((span, span), np.uint8)
        thick = max(2, int(round(TEXT_STROKE * px_per_mm)))
        # Hershey capital height is about 0.72 of the nominal size; ask for the
        # size and let the measurement pass correct the remainder.
        nominal = size_mm * px_per_mm / 0.72 / 22.0
        cv2.putText(canvas, text, (int(span * 0.15), int(span * 0.80)),
                    cv2.FONT_HERSHEY_SIMPLEX, nominal, 255, thick, cv2.LINE_AA)
        return canvas > 127

    probe = render(1.0 / TEXT_STEP, size_mm * 3.0)
    if not probe.any():
        raise RuntimeError(f"glyph {text!r} rasterised to nothing")
    rows = np.where(probe.any(axis=1))[0]
    if rows.min() < 2 or rows.max() > probe.shape[0] - 3:
        probe = render(1.0 / TEXT_STEP, size_mm * 6.0)     # still not enough room
        rows = np.where(probe.any(axis=1))[0]
    measured = (rows.max() - rows.min() + 1) * TEXT_STEP
    mask = render((1.0 / TEXT_STEP) * size_mm / max(1e-6, measured), size_mm * 3.0)
    rows = np.where(mask.any(axis=1))[0]
    cols = np.where(mask.any(axis=0))[0]
    mask = mask[rows.min():rows.max() + 1, cols.min():cols.max() + 1]

    height_mm = mask.shape[0] * TEXT_STEP
    width_mm = mask.shape[1] * TEXT_STEP
    x0 = x_mm - width_mm / 2.0 if align == "centre" else x_mm
    y0 = y_mm - height_mm / 2.0
    boxes = []
    for r in range(mask.shape[0]):
        y_top = y0 + height_mm - r * TEXT_STEP
        c = 0
        while c < mask.shape[1]:
            if not mask[r][c]:
                c += 1
                continue
            start_c = c
            while c < mask.shape[1] and mask[r][c]:
                c += 1
            boxes.append((x0 + start_c * TEXT_STEP, y_top - TEXT_STEP,
                          x0 + c * TEXT_STEP, y_top))
    return boxes


def notation_boxes(geometry: dict) -> list[tuple[float, float, float, float]]:
    half = geometry["field_mm"] / 2.0
    boxes: list[tuple[float, float, float, float]] = []
    # centre of the free band between the frame's outer edge and the sheet edge
    band = MARGIN_MM - FRAME_MM
    offset = half + FRAME_MM + band / 2.0
    for col, letter in enumerate(FILES):
        cx = -half + (col + 0.5) * geometry["square_mm"]
        boxes += _glyph_boxes(letter, cx, -offset, TEXT_H)
        boxes += _glyph_boxes(letter, cx, offset, TEXT_H)
    for row in range(SQUARES):
        cy = -half + (row + 0.5) * geometry["square_mm"]
        boxes += _glyph_boxes(str(row + 1), -offset, cy, TEXT_H)
        boxes += _glyph_boxes(str(row + 1), offset, cy, TEXT_H)
    return boxes



# --------------------------------------------------------------------- 3MF
CONTENT_TYPES = """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
"""

RELS = """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel0"
   Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"""


def _mesh_xml(tris: list[np.ndarray]) -> tuple[str, int, int]:
    """Vertices and triangles for one object, with duplicate vertices merged."""
    index: dict[tuple[float, float, float], int] = {}
    points: list[tuple[float, float, float]] = []
    faces: list[str] = []
    for tri in tris:
        ids = []
        for vertex in tri:
            key = (round(float(vertex[0]), 4), round(float(vertex[1]), 4),
                   round(float(vertex[2]), 4))
            if key not in index:
                index[key] = len(points)
                points.append(key)
            ids.append(index[key])
        if len(set(ids)) == 3:
            faces.append('   <triangle v1="%d" v2="%d" v3="%d"/>' % tuple(ids))
    body = ["  <vertices>"]
    body += ['   <vertex x="%.4f" y="%.4f" z="%.4f"/>' % p for p in points]
    body.append("  </vertices>")
    body.append("  <triangles>")
    body += faces
    body.append("  </triangles>")
    return "\n".join(body), len(points), len(faces)


def write_3mf(path: Path, objects: list[tuple[str, list[np.ndarray], int]]) -> None:
    """One file, one object per part, each assigned to a base material.

    A slicer opening this sees the parts side by side at their true relative
    positions, so the two colours cannot drift apart -- which is the whole
    reason for a multi-part file rather than two loose meshes.
    """
    import zipfile
    parts = []
    resources = []
    build = []
    next_id = 2
    resources.append('  <basematerials id="1">')
    resources.append('   <base name="White" displaycolor="#FFFFFFFF"/>')
    resources.append('   <base name="Black" displaycolor="#FF000000"/>')
    resources.append("  </basematerials>")
    for name, tris, material in objects:
        mesh, _, _ = _mesh_xml(tris)
        resources.append(f'  <object id="{next_id}" type="model" name="{name}" '
                         f'pid="1" pindex="{material}">')
        resources.append("   <mesh>")
        resources.append(mesh)
        resources.append("   </mesh>")
        resources.append("  </object>")
        build.append(f'  <item objectid="{next_id}"/>')
        next_id += 1
    model = ("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
             '<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
             " <resources>\n" + "\n".join(resources) + "\n </resources>\n"
             " <build>\n" + "\n".join(build) + "\n </build>\n</model>\n")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", CONTENT_TYPES)
        z.writestr("_rels/.rels", RELS)
        z.writestr("3D/3dmodel.model", model)


# ------------------------------------------------------------------- features
def black_rects(geometry: dict) -> list[tuple[float, float, float, float]]:
    rects = list(geometry["black_squares"])
    module = geometry["marker_mm"] / geometry["modules"]
    for marker in geometry["markers"]:
        bits = marker_bitmap(marker["id"])
        ox = marker["cx"] - geometry["marker_mm"] / 2.0
        oy = marker["cy"] + geometry["marker_mm"] / 2.0
        for r in range(MODULES):
            c = 0
            while c < MODULES:
                if not bits[r][c]:
                    c += 1
                    continue
                start = c
                while c < MODULES and bits[r][c]:
                    c += 1
                rects.append((ox + start * module, oy - r * module - module,
                              ox + c * module, oy - r * module))
    half = geometry["field_mm"] / 2.0
    edge = half + FRAME_MM
    rects += [(-edge, half, edge, edge), (-edge, -edge, edge, -half),
              (-edge, -half, -half, half), (half, -half, edge, half)]
    return rects


def _clip(rect, xmin, xmax):
    x0, y0, x1, y1 = rect
    x0, x1 = max(x0, xmin), min(x1, xmax)
    if x1 - x0 <= 1e-9:
        return None
    return (x0, y0, x1, y1)


def complement_boxes(ink, xmin: float, xmax: float,
                     ymin: float, ymax: float) -> list[tuple[float, float, float, float]]:
    """Rectangles covering ``ink``'s region minus the ink itself, exactly.

    A y-sweep over the ink's own coordinates keeps every edge on a real
    coordinate, so marker borders stay straight.  A rasterised complement would
    quantise them, and jagged tag edges are the last thing a calibration target
    needs.
    """
    rects = []
    for x0, y0, x1, y1 in ink:
        cx0, cx1 = max(x0, xmin), min(x1, xmax)
        cy0, cy1 = max(y0, ymin), min(y1, ymax)
        if cx1 - cx0 > 1e-9 and cy1 - cy0 > 1e-9:
            rects.append((cx0, cy0, cx1, cy1))
    cuts = sorted({ymin, ymax} | {y for r in rects for y in (r[1], r[3])})
    out = []
    for lo, hi in zip(cuts, cuts[1:]):
        if hi - lo <= 1e-9:
            continue
        mid = (lo + hi) / 2.0
        spans = sorted((r[0], r[2]) for r in rects if r[1] <= mid <= r[3])
        cursor = xmin
        for a, b in spans:
            if a - cursor > 1e-9:
                out.append((cursor, lo, a, hi))
            cursor = max(cursor, b)
        if xmax - cursor > 1e-9:
            out.append((cursor, lo, xmax, hi))
    return out


def build_piece(geometry: dict, side: str, xmin: float, xmax: float,
                tongue: bool) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """White and black bodies for one piece, printed face down."""
    half = SHEET_MM / 2.0
    white: list[np.ndarray] = []
    black: list[np.ndarray] = []

    # Printing face-down means the face is seen from -z, which mirrors it.  The
    # whole face layer is pre-mirrored here so that after the flip the glyphs
    # read the right way round AND the ArUco patterns decode -- a mirrored tag
    # cannot be read at all.  Mirroring about y keeps each piece's files intact
    # (left piece still carries a-d); only the ranks invert in the model.
    ink = []
    for rect in black_rects(geometry) + notation_boxes(geometry):
        clipped = _clip(rect, xmin, xmax)
        if clipped is None:
            continue
        if MIRROR_FACE:
            x0, y0, x1, y1 = clipped
            clipped = (x0, -y1, x1, -y0)
        ink.append(clipped)

    # colour layer on the bed: black where the pattern is, white everywhere else
    for x0, y0, x1, y1 in ink:
        _box(black, x0, y0, 0.0, x1, y1, FACE_Z)
    for x0, y0, x1, y1 in complement_boxes(ink, xmin, xmax, -half, half):
        _box(white, x0, y0, 0.0, x1, y1, FACE_Z)

    # body above the face layer, with the half-lap at the back
    if tongue:
        _box(white, xmin, -half, FACE_Z, xmax, half, TOTAL)
        _box(white, xmax, -half, LAP_Z0, xmax + LAP_W, half, TOTAL)
    else:
        _box(white, xmin, -half, FACE_Z, xmax, half, LAP_Z0)
        _box(white, xmin + LAP_W, -half, LAP_Z0, xmax, half, TOTAL)
    return white, black


def preview(path: Path, rects, xmin, xmax, pixels_per_mm=5.0) -> None:
    width = int((xmax - xmin) * pixels_per_mm)
    height = int(SHEET_MM * pixels_per_mm)
    image = np.full((height, width), 255, np.uint8)
    for x0, y0, x1, y1 in rects:
        if x1 <= xmin or x0 >= xmax:
            continue
        x0, x1 = max(x0, xmin), min(x1, xmax)
        px0 = int(round((x0 - xmin) * pixels_per_mm))
        px1 = int(round((x1 - xmin) * pixels_per_mm))
        py0 = int(round((SHEET_MM / 2 - y1) * pixels_per_mm))
        py1 = int(round((SHEET_MM / 2 - y0) * pixels_per_mm))
        cv2.rectangle(image, (px0, py0), (px1, py1), 0, -1)
    cv2.imwrite(str(path), image)


def main() -> None:
    global LAP_W
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="charuco_28mm")
    parser.add_argument("--lap", type=float, default=LAP_W)
    args = parser.parse_args()
    LAP_W = args.lap

    geometry = build()
    half = SHEET_MM / 2.0
    stem = Path(args.out)
    pieces = (("L", -half, 0.0, True), ("R", 0.0, half, False))
    for name, xmin, xmax, tongue in pieces:
        white, black = build_piece(geometry, name, xmin, xmax, tongue)
        for kind, tris in (("white", white), ("black", black)):
            path = stem.parent / f"{stem.name}_{name}_{kind}.stl"
            _write_stl(path, tris)
            low, high = _bounds(tris)
            print(f"{path.name:34s} {len(tris):5d} tris  "
                  f"{low} .. {high}  {_volume(tris):10.1f} mm3")
        preview(stem.parent / f"{stem.name}_{name}_preview.png",
                black_rects(geometry) + notation_boxes(geometry), xmin, xmax)
        write_3mf(stem.parent / f"{stem.name}_{name}.3mf",
                  [(f"{name}_white", white, 0), (f"{name}_black", black, 1)])
        print(f"{stem.name}_{name}.3mf  two objects: white + black")


if __name__ == "__main__":
    main()
