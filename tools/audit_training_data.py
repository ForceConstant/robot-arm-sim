"""Fail-closed audit of explicit episode splits before export or training.

Manifest: {"schema": 1, "episodes": [{"path": "relative/episode_0000",
"origin": "simulator", "session": "collection-id", "layout_group": "layout-id",
"split": "train"}]}. Paths are relative to the manifest. Never infer provenance
from pixels or directory names. The audit does not upload or modify recordings.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import imageio.v2 as imageio
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from chess_sim import JOINTS, ROBOT_CAMERAS
from chess_sim.conventions import SO101_DEGREES


def audit(manifest_path: Path, *, check_video: bool = True) -> dict:
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema") != 1 or not manifest.get("episodes"):
        raise ValueError("need schema 1 and nonempty episodes")
    errors, rows = [], []
    groups = {"session": {}, "layout_group": {}, "content": {}}
    origins = set()
    for entry in manifest["episodes"]:
        path = (manifest_path.parent / entry["path"]).resolve()
        try:
            origin, split = entry["origin"], entry["split"]
            if origin not in {"simulator", "real_camera"}:
                raise ValueError("unknown data origin")
            if split not in {"train", "validation", "test"}:
                raise ValueError("unknown split")
            origins.add(origin)
            for key in ("session", "layout_group"):
                if not isinstance(entry.get(key), str) or not entry[key].strip():
                    raise ValueError(f"missing {key}")
                group = (origin, entry[key])
                previous = groups[key].setdefault(group, split)
                if previous != split:
                    raise ValueError(f"{key} leaks across {previous}/{split}")
            meta = json.loads((path / "meta.json").read_text())
            if meta.get("success") is not True:
                raise ValueError("not a verified success; quarantine from imitation positives")
            if not isinstance(meta.get("instruction"), str) or not meta["instruction"].strip():
                raise ValueError("missing instruction")
            if meta.get("joints") != [str(j) for j in JOINTS]:
                raise ValueError("joint order differs from six-joint SO101 contract")
            if set(meta.get("cameras", [])) != {str(c) for c in ROBOT_CAMERAS}:
                raise ValueError("camera contract differs")
            if not isinstance(meta.get("fps"), int) or meta["fps"] <= 0 or meta["fps"] % 3:
                raise ValueError("fps must be positive and divisible by stride 3")
            with np.load(path / "data.npz", allow_pickle=False) as data:
                states, actions = data["observation_state"], data["action"]
            if states.shape != actions.shape or states.ndim != 2 or states.shape[1] != 6 or len(states) < 2:
                raise ValueError("need aligned nonempty Nx6 state/action arrays")
            if meta.get("steps") != len(states):
                raise ValueError("metadata step count differs")
            if not np.isfinite(states).all() or not np.isfinite(actions).all():
                raise ValueError("nonfinite state/action label")
            if not np.allclose(SO101_DEGREES.to_radians(SO101_DEGREES.from_radians(actions)), actions):
                raise ValueError("action convention round-trip failed")
            if np.max(np.ptp(actions, axis=0)) < 1e-5:
                raise ValueError("constant action labels")
            # Hash canonical arrays AND media bytes, not NPZ container metadata.
            digest = hashlib.sha256(states.tobytes() + actions.tobytes())
            for camera in sorted(meta["cameras"]):
                video = path / f"{camera}.mp4"
                with video.open("rb") as handle:
                    for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                        digest.update(chunk)
                if check_video:
                    reader = imageio.get_reader(video)
                    try:
                        if reader.count_frames() != len(states):
                            raise ValueError(f"{camera}: video/state frame mismatch")
                        reader.get_data(0)
                        reader.get_data(len(states) - 1)
                    finally:
                        reader.close()
            content = digest.hexdigest()
            if content in groups["content"]:
                raise ValueError(f"duplicate recording (already in {groups['content'][content]})")
            groups["content"][content] = split
            rows.append({"path": entry["path"], "split": split, "origin": origin,
                         "frames": len(states), "sha256": content,
                         "bytes": sum(p.stat().st_size for p in path.iterdir() if p.is_file())})
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append({"path": entry["path"], "error": str(exc)})
    if len(origins) > 1:
        errors.append({"error": "simulator and real-camera data require separate manifests/exports"})
    return {"schema": 1, "passed": not errors, "episodes": rows, "errors": errors,
            "video_checked": check_video, "action_contract": "pre-step observed radians -> commanded absolute joint targets; export so101 degrees, stride 3"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.manifest)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"{len(report['episodes'])} accepted, {len(report['errors'])} errors -> {args.out}")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
