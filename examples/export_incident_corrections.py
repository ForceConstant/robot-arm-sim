"""Export linked, successful incident corrections as state/action LeRobot episodes.

This deliberately exports the executed parameter-correction trace, not the
failed source trace as if it were a policy correction. A JSONL sidecar preserves
the source incident, source fingerprint, task and actual simulator verdicts.

Run in the VLA environment (Python 3.12 with lerobot installed):

    python examples/export_incident_corrections.py \
      --incidents /path/to/private/incidents --repo-id local/robot-corrections \
      --root datasets/lerobot/robot-corrections
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

import numpy as np
from chess_sim import JOINTS
from chess_sim.conventions import CONVENTIONS, SO101_DEGREES


ID = re.compile(r"^[0-9a-f]{32}$")
def read_incidents(root: Path) -> dict[str, tuple[dict, Path, list[dict]]]:
    incidents = {}
    for folder in root.iterdir():
        manifest_path = folder / "manifest.json"
        if not folder.is_dir() or not manifest_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text())
        ident = manifest.get("id", folder.name)
        if ID.fullmatch(ident):
            trace_path = folder / "trace.json"
            trace = json.loads(trace_path.read_text()) if trace_path.is_file() else []
            incidents[ident] = (manifest, folder, trace)
    return incidents


def correction_task(meta: dict) -> str:
    task = meta.get("task") or {}
    if task.get("kind") == "move":
        return f"Move the piece from {task['from']} to {task['to']}"
    if task.get("kind") == "capture":
        return f"Capture the piece on {task['square']}"
    raise ValueError("correction has no supported task description")


def episode_frames(trace: list[dict], task: str, convention):
    for row in trace:
        action = row.get("action")
        joints = row.get("joints")
        # The initial snapshot precedes the first command and has no action;
        # it is not a supervised transition.
        if action is None or joints is None:
            continue
        state = convention.from_radians(np.asarray(joints, dtype=np.float32)).astype(np.float32)
        action = convention.from_radians(np.asarray(action, dtype=np.float32)).astype(np.float32)
        if state.shape != (6,) or action.shape != (6,) or not np.isfinite(state).all() or not np.isfinite(action).all():
            raise ValueError("incident contains a malformed or non-finite joint transition")
        yield {"observation.state": state, "action": action, "task": task}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--incidents", type=Path, required=True,
                        help="private incident store directory (contains incident-ID folders)")
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--include-failures", action="store_true",
                        help="also export linked corrections whose actual result failed")
    parser.add_argument("--convention", choices=list(CONVENTIONS), default=SO101_DEGREES.name,
                        help="joint units used by the LeRobot feature vectors")
    args = parser.parse_args()

    incidents = read_incidents(args.incidents)
    candidates = []
    for ident, (meta, folder, trace) in incidents.items():
        source_id = meta.get("correction_of")
        if not source_id:
            continue
        if not ID.fullmatch(source_id) or source_id not in incidents:
            raise SystemExit(f"{ident}: correction source {source_id!r} is missing")
        source, _, _ = incidents[source_id]
        if source.get("success") is not False:
            raise SystemExit(f"{ident}: source incident {source_id} is not a recorded failure")
        # A correction is useful only with its own measured outcome. Never infer
        # success from a sweep's proposed parameters or its UI status message.
        if meta.get("success") is not True and not args.include_failures:
            continue
        fingerprint = meta.get("source_revision") or meta.get("runtime_signature")
        if not fingerprint:
            raise SystemExit(f"{ident}: no source revision/runtime fingerprint")
        candidates.append((ident, meta, folder, trace, source_id, fingerprint))
    if not candidates:
        raise SystemExit("no eligible linked incident corrections found")

    convention = CONVENTIONS[args.convention]
    if not any(next(episode_frames(c[3], correction_task(c[1]), convention), None)
               is not None for c in candidates):
        raise SystemExit("eligible correction has no recorded state/action transitions")
    # Import only at execution so the incident reader/validator remains usable
    # in the simulator image, which intentionally does not carry training deps.
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    features = {
        "observation.state": {"dtype": "float32", "shape": (6,), "names": list(JOINTS)},
        "action": {"dtype": "float32", "shape": (6,), "names": list(JOINTS)},
    }
    config = candidates[0][1].get("config") or {}
    control = config.get("control") or {}
    fps = int(control.get("hz", 30))
    dataset = LeRobotDataset.create(repo_id=args.repo_id, root=args.root, fps=fps,
                                    robot_type="so101", features=features)
    sidecar = args.root / "incident-corrections.jsonl"
    args.root.mkdir(parents=True, exist_ok=True)
    exported = 0
    with sidecar.open("w") as index:
        for ident, meta, folder, trace, source_id, fingerprint in candidates:
            frames = list(episode_frames(trace, correction_task(meta), convention))
            if not frames:
                continue
            for frame in frames:
                dataset.add_frame(frame)
            dataset.save_episode()
            index.write(json.dumps({
                "episode": exported,
                "correction_of": source_id,
                "incident_id": ident,
                "source_revision": fingerprint,
                "task": correction_task(meta),
                "success": bool(meta.get("success")),
                "failure_label": None if meta.get("success") else meta.get("categories"),
                "source_success": bool(incidents[source_id][0].get("success")),
                "source_failure_label": incidents[source_id][0].get("categories"),
                "frames": len(frames),
                "model_signature": meta.get("model_signature"),
                "runtime_signature": meta.get("runtime_signature"),
            }) + "\n")
            exported += 1
    dataset.finalize()
    print(f"exported {exported} linked correction episodes to {args.root}")
    print(f"labels and correction/source provenance: {sidecar}")


if __name__ == "__main__":
    main()
