#!/usr/bin/env python3
"""Re-pin rig state: a deliberate, logged step of the test run -- not an edit.

The frame pins (``tests/test_frame_pins.py``) exist to catch *unexplained*
drift, not to freeze per-session data.  When something legitimately changes --
an accepted wrist zero, a new session board registration -- the change is
re-pinned here, with a reason:

    python tools/repin_rig_state.py --why "board registration 2026-11-27" \\
        --file board.json

What it does, per file: copy the canonical live file into ``rig_state/``,
recompute sha256 + bytes in ``rig_state/MANIFEST.json``, rewrite the matching
literals in ``tests/test_frame_pins.py`` (hash, plus the board pose for
``board.json``) and append one line to ``rig_state/PIN_LOG.md``.

The pinned set is read from ``rig_state/MANIFEST.json``, so the policy choice
(pin the board each session, or drop it from the pinned set) is a manifest
decision and not something this tool hard-codes.

A re-pin without a reason is a failure: ``--why`` is required and must be
non-blank.  See ``docs/TESTS.md`` §0c.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / "rig_state"
MANIFEST = STATE_DIR / "MANIFEST.json"
PINS = ROOT / "tests" / "test_frame_pins.py"
LOG = STATE_DIR / "PIN_LOG.md"
BOARD_AXES = ("x_mm", "y_mm", "z_mm", "yaw_deg")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rewrite_hash(text: str, name: str, new_hash: str) -> str:
    pattern = re.compile(r'("%s"\s*:\s*")[0-9a-f]{64}(")' % re.escape(name))
    text, count = pattern.subn(r"\g<1>%s\g<2>" % new_hash, text)
    if count != 1:
        raise SystemExit(f"REFUSED: {count} MANIFEST_HASHES entries for {name}")
    return text


def rewrite_board_pose(text: str, board: dict) -> str:
    for axis in BOARD_AXES:
        pattern = re.compile(r'(\n\s*"%s":\s*)-?[0-9.]+(,)' % axis)
        text, count = pattern.subn(r"\g<1>%.2f\g<2>" % float(board[axis]), text, count=1)
        if count != 1:
            raise SystemExit(f"REFUSED: {count} BOARD_EXPECTED entries for {axis}")
    return text


def log_line(name: str, old: str, new: str, why: str) -> None:
    stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    if not LOG.exists():
        LOG.write_text("# Rig-state pin log\n\n"
                       "One line per deliberate re-pin: UTC time, file, old -> new "
                       "hash prefix, reason.\n\n")
    with LOG.open("a") as handle:
        handle.write(f"{stamp}  {name}  {old[:10]} -> {new[:10]}  why: {why}\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--why", required=True, help="one sentence: why this re-pin is legitimate")
    ap.add_argument("--file", action="append", dest="files", metavar="NAME",
                    help="pinned file name inside rig_state/ (default: board.json)")
    args = ap.parse_args()
    why = (args.why or "").strip()
    if not why:
        print("REFUSED: --why must not be blank; a re-pin without a reason is a failure")
        return 2
    if not MANIFEST.exists():
        print(f"REFUSED: no manifest at {MANIFEST.relative_to(ROOT)}")
        return 2
    manifest = json.loads(MANIFEST.read_text())
    pinned = manifest["files"]
    names = args.files or ["board.json"]
    unknown = [n for n in names if n not in pinned]
    if unknown:
        print("REFUSED: not a pinned file: " + ", ".join(unknown)
              + " (pinned: " + ", ".join(pinned) + ")")
        return 2
    if not PINS.exists():
        print(f"REFUSED: pins file missing at {PINS.relative_to(ROOT)}")
        return 2

    text = PINS.read_text()
    changed = []
    for name in names:
        entry = pinned[name]
        source = ROOT / entry["canonical"]
        if not source.exists():
            print(f"REFUSED: canonical file missing: {entry['canonical']}")
            return 2
        target = STATE_DIR / name
        old = entry["sha256"]
        shutil.copyfile(source, target)
        new = sha256(target)
        if new == old:
            print(f"  unchanged {name}  {new[:10]}  ({entry['canonical']})")
            continue
        entry["sha256"] = new
        entry["bytes"] = target.stat().st_size
        text = rewrite_hash(text, name, new)
        if name == "board.json":
            text = rewrite_board_pose(text, json.loads(target.read_text()))
        changed.append((name, old, new, entry["bytes"]))
        print(f"  repinned  {name}  {old[:10]} -> {new[:10]}  {entry['bytes']} B"
              f"  ({entry['canonical']} -> rig_state/{name})")

    if not changed:
        print("nothing changed; no manifest, pin or log write")
        return 0
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n")
    PINS.write_text(text)
    for name, old, new, _ in changed:
        log_line(name, old, new, why)
    print(f"manifest: {MANIFEST.relative_to(ROOT)}")
    print(f"pins:     {PINS.relative_to(ROOT)}")
    print(f"log:      {LOG.relative_to(ROOT)}  (+{len(changed)} line(s))")
    print("why:      " + why)
    return 0


if __name__ == "__main__":
    sys.exit(main())
