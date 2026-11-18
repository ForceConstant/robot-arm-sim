"""ChArUco face for the simulated board.

The physical target prints its markers into the board surface, so the sim needs
the same thing in its texture: 8x8 squares of the board's own size with a
20/28 ratio ArUco in every white square, a frame line around the field, and the
algebraic notation in the border band.

Keeping this as a texture rather than marker geoms matters twice over: the
visual and the collision surface cannot disagree, and a rendered frame carries
exactly the marker pattern the printed board carries, so a detection test in
sim predicts what the camera will see.
"""
from __future__ import annotations

import math
import os

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_4X4_50
FIRST_ID = 5              # never collides with the mat tags 0..3 or centre tag 4
MARKER_RATIO = 20.0 / 28.0
MODULES = 6
FILES = "abcdefgh"


def square_of(marker_id: int) -> tuple[int, int] | None:
    """(col, rank) of a ChArUco marker in board axes, or None if not on the face."""
    index = int(marker_id) - FIRST_ID
    if index < 0:
        return None
    count = 0
    for rank in range(8):
        for col in range(8):
            if (col + rank) % 2 == 0:
                continue
            if count == index:
                return col, rank
            count += 1
    return None


def corners_mm(marker_id: int, square_mm: float = 28.0,
               marker_mm: float = 28.0 * MARKER_RATIO):
    """Four marker corners in board-frame millimetres, top-left first."""
    square = square_of(marker_id)
    if square is None:
        raise ValueError(f"marker {marker_id} is not on the ChArUco face")
    col, rank = square
    half_field = square_mm * 4.0
    cx = (col + 0.5) * square_mm - half_field
    cy = (rank + 0.5) * square_mm - half_field
    h = marker_mm / 2.0
    return ((cx - h, cy + h), (cx + h, cy + h), (cx + h, cy - h), (cx - h, cy - h))


class CharucoLayout:
    """Adapter so the live app's marker maths can use the printed face.

    Exposes the same surface as ``fiducials.MarkerLayout`` (``size_mm`` and
    ``corners_mm``), which is all ``marker_corners_world`` and the derive path
    need.
    """

    def __init__(self, square_mm: float = 28.0, marker_mm: float | None = None) -> None:
        self.square_mm = float(square_mm)
        self.size_mm = float(marker_mm or square_mm * MARKER_RATIO)

    def corners_mm(self, marker_id: int):
        return corners_mm(marker_id, self.square_mm, self.size_mm)


def _marker_black(marker_id: int) -> np.ndarray:
    """6x6 module matrix, True where the module is black."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    image = cv2.aruco.generateImageMarker(dictionary, marker_id, MODULES * 16)
    small = cv2.resize(image, (MODULES, MODULES), interpolation=cv2.INTER_AREA)
    return small < 128


def write_face(path: str, board, px_per_square: int = 224) -> str:
    """Render the ChArUco face to `path`, sized to `board`.

    224 px per 28 mm square is the smallest multiple of 28 that makes the
    20 mm marker exactly 160 px, so every square and marker edge lands on an
    integer pixel.  At 128 px the 20 mm marker rounded to 91 px (19.906 mm),
    a 0.47 % size error against the declared geometry that showed up as a
    ~0.7 px reprojection floor in the block-3 calibration matrix.
    """
    from PIL import Image, ImageDraw, ImageFont

    square_px = px_per_square
    band_px = max(8, int(round(square_px * board.border / board.square)))
    field_px = 8 * square_px
    size = field_px + 2 * band_px

    # Layout is authored in BOARD coordinates exactly like the printed target:
    # row 0 is rank 1, col 0 is file a, a1 is dark, and marker ids start at b1.
    # Image rows run the other way (row 0 is the far edge), so every band is
    # placed with 7 - row.  Getting this wrong silently mismatches ids to
    # squares, which would poison any calibration done against the print.
    img = np.full((size, size, 3), 246, np.uint8)
    light, dark = 236, 26
    for rank in range(8):
        for col in range(8):
            value = dark if (col + rank) % 2 == 0 else light        # a1 dark
            y0 = band_px + (7 - rank) * square_px
            x0 = band_px + col * square_px
            img[y0:y0 + square_px, x0:x0 + square_px] = value

    # frame line just outside the field, as printed
    line = max(2, int(round(square_px * 0.001 * 10)))
    img[band_px - line:band_px, band_px:band_px + field_px] = 0
    img[band_px + field_px:band_px + field_px + line, band_px:band_px + field_px] = 0
    img[band_px:band_px + field_px, band_px - line:band_px] = 0
    img[band_px:band_px + field_px, band_px + field_px:band_px + field_px + line] = 0

    # markers inside the white squares
    marker_px = int(round(square_px * MARKER_RATIO))
    module_px = marker_px / MODULES
    marker_id = FIRST_ID
    for rank in range(8):
        for col in range(8):
            if (col + rank) % 2 == 0:
                continue                       # dark square, no marker
            bits = _marker_black(marker_id)
            marker_id += 1
            cx = band_px + col * square_px + square_px / 2.0
            cy = band_px + (7 - rank) * square_px + square_px / 2.0
            x0 = cx - marker_px / 2.0
            y0 = cy - marker_px / 2.0
            for r in range(MODULES):
                for c in range(MODULES):
                    if not bits[r][c]:
                        continue
                    ya = int(round(y0 + r * module_px))
                    xa = int(round(x0 + c * module_px))
                    yb = int(round(y0 + (r + 1) * module_px))
                    xb = int(round(x0 + (c + 1) * module_px))
                    img[ya:yb, xa:xb] = 0

    image = Image.fromarray(img)
    if band_px >= 14:
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default(size=int(band_px * 0.62))
        centre = band_px / 2.0
        for i, letter in enumerate(FILES):
            x = band_px + (i + 0.5) * square_px
            draw.text((x, centre), letter, fill=0, font=font, anchor="mm")
            draw.text((x, size - centre), letter, fill=0, font=font, anchor="mm")
        for rank in range(8):
            y = band_px + (7 - rank + 0.5) * square_px
            draw.text((centre, y), str(rank + 1), fill=0, font=font, anchor="mm")
            draw.text((size - centre, y), str(rank + 1), fill=0, font=font, anchor="mm")

    # NOTE: the simulator maps this face with V inverted relative to image rows,
    # so the rendered board is the print's mirror in rank.  Flipping the file
    # here also mirrors every marker's bitmap and destroys decoding, so the
    # correction belongs in the consumer: see UV_RANK_FLIP in handeye_bundle.
    os.makedirs(os.path.dirname(path), exist_ok=True)
    image.save(path)
    return path
