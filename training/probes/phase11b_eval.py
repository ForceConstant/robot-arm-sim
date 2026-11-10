"""Per-bucket held-out evaluation for a Phase 11b ACT checkpoint.

Reads the checkpoint written by phase11b_train.py and reports L1 and MSE against
the action chunk, with the hold-position baseline, for every
piece/colour/family/crowding bucket. Evaluation only; no robot commands.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from act_runtime import load_act, config  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--batch", type=int, default=32)
    args = parser.parse_args()

    import torch
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    with np.load(args.dataset, allow_pickle=False) as loaded:
        data = {key: loaded[key] for key in loaded.files}
    saved = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    proc = saved["processor"]
    model = load_act(args.source)(config())
    model.load_state_dict(saved["model"])
    model.eval()

    image_mean = torch.tensor(proc["image_mean"])[None, :, None, None]
    image_std = torch.tensor(proc["image_std"])[None, :, None, None]
    state_mean, state_std = torch.tensor(proc["state_mean"]), torch.tensor(proc["state_std"])
    action_mean, action_std = torch.tensor(proc["action_mean"]), torch.tensor(proc["action_std"])
    test = np.flatnonzero(data["split"] == "test")

    def batch(indices):
        images = torch.from_numpy(data["image"][indices]).permute(0, 3, 1, 2).float().div_(255)
        return {"observation.images": [(images - image_mean) / image_std],
                "observation.state": (torch.from_numpy(data["state"][indices]) - state_mean) / state_std,
                "observation.environment_state": torch.from_numpy(data["goal"][indices])}

    prediction = []
    with torch.inference_mode():
        for start in range(0, len(test), args.batch):
            part = test[start:start + args.batch]
            raw = model(batch(part))[0]
            prediction.append((raw * action_std + action_mean).numpy())
    prediction = np.concatenate(prediction)
    valid = ~data["pad"][test]
    error = prediction - data["action"][test]
    hold = data["state"][test, None, :] - data["action"][test]
    with args.checkpoint.open("rb") as handle:
        checkpoint_sha = hashlib.file_digest(handle, "sha256").hexdigest()
    result = dict(schema="phase11b-bucket-eval/1", checkpoint_sha256=checkpoint_sha,
                  samples=int(len(test)), buckets={})
    for bucket in sorted(set(data["bucket"][test])):
        mask = data["bucket"][test] == bucket
        bucket_error = error[mask][valid[mask]]
        bucket_hold = hold[mask][valid[mask]]
        l1 = float(np.abs(bucket_error).mean())
        hold_l1 = float(np.abs(bucket_hold).mean())
        result["buckets"][bucket] = dict(
            samples=int(mask.sum()), l1_rad=l1, hold_position_l1_rad=hold_l1,
            improvement_over_hold=float(1 - l1 / hold_l1),
            mse_rad2=float((bucket_error ** 2).mean()),
            hold_position_mse_rad2=float((bucket_hold ** 2).mean()))
    args.out.write_text(json.dumps(result, indent=2))
    print(json.dumps({bucket: round(row["improvement_over_hold"], 3)
                      for bucket, row in result["buckets"].items()}))


if __name__ == "__main__":
    main()
