#!/usr/bin/env python3
"""ChArUco target as two STL bodies for a multicolour printer.

Split by ink, matching the DXF:

    white_*.stl   244 x 244 x 0.6 mm base plate
    black_*.stl   dark squares, marker modules and the field frame, extruded
                  from 0.6 mm up to the full 3.2 mm

Both are Z-up, millimetres, XY centred on the board origin like the DXF, and
sit on z=0.  Load both as separate parts in the slicer and assign one filament
each; the black features land on the white base, so the tallest surface is
3.2 mm and the white shows through the gaps.

Note: that leaves the white areas 2.6 mm *below* the black surface.  For plain
printing that is fine and looks right.  For calibration it is not ideal --
chessboard corners become 2.6 mm step edges, which biases subpixel corner
refinement.  ``--flush`` writes a flat variant instead (white full thickness,
black as a 0.6 mm top layer) if you would rather have a coplanar face.

    robot-arm-sim python tools/charuco_stl.py --out charuco_28mm
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np

from charuco_dxf import (FRAME_MM, SHEET_MM, build, marker_bitmap, MODULES)

WHITE_MM = 0.6
TOTAL_MM = 3.2


def _box(triangles: list[np.ndarray], x0, y0, z0, x1, y1, z1) -> None:
    """Twelve triangles, outward normals, right-handed."""
    corners = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
               (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2, 1),     # bottom (viewed from below)
             (4, 5, 6, 7),     # top
             (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    for a, b, c, d in faces:
        p0, p1, p2, p3 = (np.array(corners[i], dtype=np.float64)
                          for i in (a, b, c, d))
        triangles.append(np.vstack([p0, p1, p2]))
        triangles.append(np.vstack([p0, p2, p3]))


def _write_stl(path: Path, triangles: list[np.ndarray]) -> None:
    with path.open("wb") as handle:
        handle.write(b"charuco target".ljust(80, b"\0"))
        handle.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            edge1 = tri[1] - tri[0]
            edge2 = tri[2] - tri[0]
            normal = np.cross(edge1, edge2)
            length = np.linalg.norm(normal)
            normal = normal / length if length > 1e-12 else np.zeros(3)
            handle.write(struct.pack("<3f", *normal))
            for vertex in tri:
                handle.write(struct.pack("<3f", *vertex))
            handle.write(struct.pack("<H", 0))


def _measure(triangles: list[np.ndarray]) -> dict:
    points = np.vstack(triangles)
    volume = 0.0
    for tri in triangles:  # signed tetrahedra, valid for closed meshes
        volume += float(np.dot(tri[0], np.cross(tri[1], tri[2]))) / 6.0
    return {"triangles": len(triangles),
            "min": [round(float(v), 2) for v in points.min(axis=0)],
            "max": [round(float(v), 2) for v in points.max(axis=0)],
            "volume_mm3": round(abs(volume), 1)}


def build_black(geometry: dict, z0: float, z1: float) -> list[np.ndarray]:
    triangles: list[np.ndarray] = []
    for x0, y0, x1, y1 in geometry["black_squares"]:
        _box(triangles, x0, y0, z0, x1, y1, z1)
    module = geometry["marker_mm"] / geometry["modules"]
    for marker in geometry["markers"]:
        bits = marker_bitmap(marker["id"])
        ox = marker["cx"] - geometry["marker_mm"] / 2.0
        oy = marker["cy"] + geometry["marker_mm"] / 2.0
        # Merge horizontal runs of black modules into one box per run: touching
        # boxes leave coincident faces behind, which some slicers flag even
        # though the union is closed.
        for r in range(MODULES):
            c = 0
            while c < MODULES:
                if not bits[r][c]:
                    c += 1
                    continue
                start = c
                while c < MODULES and bits[r][c]:
                    c += 1
                x0 = ox + start * module
                x1 = ox + c * module
                y1 = oy - r * module
                _box(triangles, x0, y1 - module, z0, x1, y1, z1)
    half = geometry["field_mm"] / 2.0
    edge = half + FRAME_MM
    for x0, y0, x1, y1 in ((-edge, half, edge, edge), (-edge, -edge, edge, -half),
                           (-edge, -half, -half, half), (half, -half, edge, half)):
        _box(triangles, x0, y0, z0, x1, y1, z1)
    return triangles


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="charuco_28mm")
    parser.add_argument("--flush", action="store_true",
                        help="white full thickness, black as a 0.6 mm top layer")
    args = parser.parse_args()

    geometry = build()
    half = SHEET_MM / 2.0
    stem = Path(args.out)

    white = []
    if args.flush:
        _box(white, -half, -half, 0.0, half, half, TOTAL_MM - WHITE_MM)
        black = build_black(geometry, TOTAL_MM - WHITE_MM, TOTAL_MM)
        white_name, black_name = "white_base", "black_top"
    else:
        _box(white, -half, -half, 0.0, half, half, WHITE_MM)
        black = build_black(geometry, WHITE_MM, TOTAL_MM)
        white_name, black_name = "white", "black"

    for name, triangles in ((white_name, white), (black_name, black)):
        path = stem.parent / f"{stem.name}_{name}.stl"
        _write_stl(path, triangles)
        print(f"{path}: {_measure(triangles)}")


if __name__ == "__main__":
    main()
