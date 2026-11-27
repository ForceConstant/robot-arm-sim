"""tests/test_mount_pin.py — the camera mount is a hardware constant, not a knob.

``camera_in_tool`` is the one calibration value that a board offset must never
absorb (AGENTS.md, 2026-11-27).  A silent re-fit over a single session's board
placement produced a consistent ~15 mm "correction" that was really the mat
being set down by hand (``reports/block5r_wide3_20261127.md``).  This pins the
deployed bench calibration byte-for-byte so such a rewrite fails here.

Pins both copies when they exist:

* ``.private/phase11d/calibration.json`` — the deployed file the console loads
  (present on the bench, absent from a bare checkout);
* ``rig_state/mount_calibration.json`` — the committed snapshot.

The test skips only when neither exists.  Changing the pin is a deliberate
re-baseline (docs/TESTS.md §3b), not a test to waive.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

#: handeye-2026-11-18T11-block4c, the deployed 4c mount and lens.
MOUNT_SHA256 = "70f6c51893d294fdf1512c896f5f7171694b06fce8c888e6c04b00a788b144f4"
MOUNT_ID = "handeye-2026-11-18T11-block4c"
MOUNT_INTRINSICS = (831.897, 833.014)

CANDIDATES = (
    ROOT / ".private" / "phase11d" / "calibration.json",
    ROOT / "rig_state" / "mount_calibration.json",
)


def _existing() -> list[Path]:
    return [path for path in CANDIDATES if path.exists()]


@pytest.mark.parametrize("path", CANDIDATES, ids=lambda p: p.relative_to(ROOT).as_posix())
def test_mount_file_hash_is_pinned(path: Path) -> None:
    if not path.exists():
        pytest.skip(f"{path.relative_to(ROOT)} is not present in this checkout")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert digest == MOUNT_SHA256, (
        f"{path.relative_to(ROOT)} drifted: {digest} != {MOUNT_SHA256}. "
        "A mount re-fit belongs in a deliberate re-baseline, never here.")


def test_mount_identity_matches_the_recorded_calibration() -> None:
    existing = _existing()
    if not existing:
        pytest.skip("no mount calibration file in this checkout")
    for path in existing:
        record = json.loads(path.read_text())
        assert record.get("calibration_id") == MOUNT_ID
        assert tuple(float(v) for v in record["intrinsics"][:2]) == pytest.approx(
            MOUNT_INTRINSICS)
