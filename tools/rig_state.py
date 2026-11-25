#!/usr/bin/env python3
"""Rig state: snapshot, restore and verify the machine-local calibration files.

Why this exists
---------------
The console's behaviour depends on files that are deliberately NOT in git:
``live/board.json``, ``live/live_config.json``, ``live/wrist_calibration.json``
and the camera mount ``.private/phase11d/calibration.json``.  They are ignored
because they describe one bench and because ``.private/`` also holds CAD,
meshes and weights.  The consequence, found the hard way on 2026-11-25: a fresh
clone reproduces the *code* but not the *rig*, and a wrong board frame fails
silently - the arm still moves, it just moves to the wrong place.

So the values are committed as data in ``rig_state/`` with checksums and
provenance, and this tool moves them between the repo and their canonical
paths.  Restoring is explicit: nothing here runs at import time.

    python tools/rig_state.py snapshot      # canonical files -> rig_state/
    python tools/rig_state.py verify        # canonical vs manifest, report drift
    python tools/rig_state.py restore       # manifest -> canonical (backs up first)
    python tools/rig_state.py restore --force   # accept a checksum mismatch

Run it on the machine that owns the files (the BC-250), not on a workstation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / "rig_state"
MANIFEST = STATE_DIR / "MANIFEST.json"

#: repo filename inside rig_state/ -> canonical path on the machine
FILES = {
    "board.json": "live/board.json",
    "live_config.json": "live/live_config.json",
    "wrist_calibration.json": "live/wrist_calibration.json",
    "mount_calibration.json": ".private/phase11d/calibration.json",
}

#: What each file decides, in one line, for the manifest and for a human.
PURPOSE = {
    "board.json": "board pose in the arm frame (x, y, z, yaw) - a wrong value "
                  "moves every commanded square and fails silently",
    "live_config.json": "console settings: solver (mink), move tolerance, auth "
                        "posture, home pose, layout",
    "wrist_calibration.json": "wrist roll zero and the allowed roll window",
    "mount_calibration.json": "camera-in-tool mount and the arm chain - every "
                              "projection and aim point depends on it",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def snapshot(note: str = "") -> int:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    manifest = {"schema": "rig-state/1", "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "note": note, "files": {}}
    missing = []
    for name, canonical in FILES.items():
        source = ROOT / canonical
        if not source.exists():
            missing.append(canonical)
            continue
        target = STATE_DIR / name
        shutil.copy2(source, target)
        manifest["files"][name] = {
            "canonical": canonical,
            "sha256": sha256(target),
            "bytes": target.stat().st_size,
            "purpose": PURPOSE.get(name, ""),
        }
        print(f"  {canonical} -> rig_state/{name}  {manifest['files'][name]['bytes']} B")
    if missing:
        print("MISSING (not snapshotted): " + ", ".join(missing))
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"manifest: {MANIFEST.relative_to(ROOT)}")
    return 0


def verify(quiet: bool = False) -> int:
    if not MANIFEST.exists():
        print("no manifest; run snapshot first")
        return 2
    manifest = json.loads(MANIFEST.read_text())
    bad = 0
    for name, entry in manifest["files"].items():
        canonical = ROOT / entry["canonical"]
        if not canonical.exists():
            print(f"  MISSING   {entry['canonical']}")
            bad += 1
            continue
        actual = sha256(canonical)
        if actual != entry["sha256"]:
            print(f"  DRIFT     {entry['canonical']}  (manifest {entry['sha256'][:12]}, "
                  f"actual {actual[:12]})")
            bad += 1
        elif not quiet:
            print(f"  ok        {entry['canonical']}")
    print("rig state: " + ("MATCHES the manifest" if not bad else f"{bad} file(s) drifted"))
    return 0 if not bad else 1


def restore(force: bool) -> int:
    if not MANIFEST.exists():
        print("no manifest to restore from")
        return 2
    manifest = json.loads(MANIFEST.read_text())
    stamp = time.strftime("%Y%m%dT%H%M%S")
    problems = []
    # Verify everything before writing anything: a half-applied rig is worse
    # than an untouched one.
    for name, entry in manifest["files"].items():
        stored = STATE_DIR / name
        if not stored.exists():
            problems.append(f"rig_state/{name} is missing")
            continue
        if sha256(stored) != entry["sha256"] and not force:
            problems.append(f"rig_state/{name} does not match its checksum")
    if problems:
        for line in problems:
            print("  REFUSED: " + line)
        if not force:
            print("nothing written; re-snapshot or pass --force")
            return 1
    for name, entry in manifest["files"].items():
        stored = STATE_DIR / name
        canonical = ROOT / entry["canonical"]
        canonical.parent.mkdir(parents=True, exist_ok=True)
        if canonical.exists():
            backup = canonical.with_name(canonical.name + f".bak-restore-{stamp}")
            shutil.copy2(canonical, backup)
            print(f"  backed up {entry['canonical']} -> {backup.name}")
        shutil.copy2(stored, canonical)
        print(f"  restored  {entry['canonical']}")
    print("rig state restored. Restart the console so it reloads board/config:")
    print("  systemctl --user restart robot-live")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("snapshot", "verify", "restore"))
    ap.add_argument("--note", default="", help="why this snapshot was taken")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    if args.action == "snapshot":
        return snapshot(args.note)
    if args.action == "verify":
        return verify()
    return restore(args.force)


if __name__ == "__main__":
    sys.exit(main())
