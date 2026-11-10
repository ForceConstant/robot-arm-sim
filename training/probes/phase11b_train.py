"""Phase 11b: train full-size explicit-goal ACT locally on the BC-250 CPU.

Consumes the audited corpus from tools/export_phase11b.py and writes, per seed,
a JSON record with the held-out learning curve, the hold-position baseline,
throughput, peak RSS, peak SoC temperature and an exact checkpoint reload. This
is a training/quality receipt for the control-policy gate; it never commands a
robot.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import resource
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from act_runtime import load_act, config, loss  # noqa: E402


def sha(path: Path) -> str:
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def temperature() -> float | None:
    for hwmon in Path("/sys/class/hwmon").glob("hwmon*"):
        try:
            name = (hwmon / "name").read_text().strip()
            if name in ("k10temp", "zenpower"):
                return int((hwmon / "temp1_input").read_text()) / 1000
        except OSError:
            continue
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--steps", type=int, default=2500)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--eval-every", type=int, default=250)
    parser.add_argument("--eval-samples", type=int, default=768)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--lr", type=float, default=1e-5)
    parser.add_argument("--deterministic", action="store_true",
                        help="force deterministic algorithms (slower on CPU)")
    parser.add_argument("--final-samples", type=int, default=0,
                        help="0 uses the whole test split")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)

    import torch
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    torch.manual_seed(args.seed)
    if args.deterministic:
        torch.use_deterministic_algorithms(True)

    started = time.monotonic()
    with np.load(args.dataset, allow_pickle=False) as loaded:
        data = {key: loaded[key] for key in loaded.files}
    train = np.flatnonzero(data["split"] == "train")
    test = np.flatnonzero(data["split"] == "test")
    if not len(train) or not len(test):
        raise ValueError("empty split")
    if set(data["episode"][train]) & set(data["episode"][test]):
        raise ValueError("episode leaked across the split")
    rng = np.random.default_rng(args.seed)

    proc = dict(state_mean=data["state"][train].mean(0).tolist(),
                state_std=np.maximum(data["state"][train].std(0), 0.01).tolist(),
                action_mean=data["action"][train][~data["pad"][train]].mean(0).tolist(),
                action_std=np.maximum(data["action"][train][~data["pad"][train]].std(0), 0.01).tolist(),
                image_mean=[0.485, 0.456, 0.406], image_std=[0.229, 0.224, 0.225])
    image_mean = torch.tensor(proc["image_mean"])[None, :, None, None]
    image_std = torch.tensor(proc["image_std"])[None, :, None, None]
    state_mean, state_std = torch.tensor(proc["state_mean"]), torch.tensor(proc["state_std"])
    action_mean, action_std = torch.tensor(proc["action_mean"]), torch.tensor(proc["action_std"])

    def batch(indices):
        images = torch.from_numpy(data["image"][indices]).permute(0, 3, 1, 2).float().div_(255)
        return {
            "observation.images": [(images - image_mean) / image_std],
            "observation.state": (torch.from_numpy(data["state"][indices]) - state_mean) / state_std,
            "observation.environment_state": torch.from_numpy(data["goal"][indices]),
            "action": (torch.from_numpy(data["action"][indices]) - action_mean) / action_std,
            "action_is_pad": torch.from_numpy(data["pad"][indices]),
        }

    def infer(indices, size=32):
        model.eval()
        chunks = []
        with torch.inference_mode():
            for start in range(0, len(indices), size):
                part = indices[start:start + size]
                raw = model({k: v for k, v in batch(part).items() if k not in ("action", "action_is_pad")})[0]
                chunks.append(raw * action_std + action_mean)
        return torch.cat(chunks).numpy()

    def evaluate(indices):
        prediction = infer(indices)
        valid = ~data["pad"][indices]
        error = prediction - data["action"][indices]
        hold = data["state"][indices, None, :] - data["action"][indices]
        return dict(l1_rad=float(np.abs(error[valid]).mean()),
                    mse_rad2=float((error[valid] ** 2).mean()),
                    hold_position_l1_rad=float(np.abs(hold[valid]).mean()),
                    hold_position_mse_rad2=float((hold[valid] ** 2).mean()),
                    improvement_over_hold=float(1 - np.abs(error[valid]).mean()
                                                / np.abs(hold[valid]).mean()))

    ACT = load_act(args.source)
    cfg = config()
    model = ACT(cfg)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4, foreach=False)
    parameters = sum(p.numel() for p in model.parameters())

    # Fixed held-out slice for the curve; the final score uses every test sample.
    curve_idx = rng.choice(test, size=min(args.eval_samples, len(test)), replace=False)
    curve_idx.sort()
    curve = [dict(step=0, samples=0, **evaluate(curve_idx))]
    print("EVAL", 0, json.dumps(curve[-1]), flush=True)

    train_started = time.monotonic()
    samples = 0
    peak_temp = temperature()
    log = (args.out / "metrics.jsonl").open("w")
    while curve[-1]["step"] < args.steps:
        target = min(curve[-1]["step"] + args.eval_every, args.steps)
        while samples < target * args.batch:
            indices = rng.choice(train, size=args.batch, replace=False)
            begin = time.monotonic()
            model.train()
            opt.zero_grad(set_to_none=True)
            value = loss(model, batch(indices))
            value.backward()
            if not torch.isfinite(value) or any(p.grad is None or not torch.isfinite(p.grad).all()
                                                for p in model.parameters() if p.requires_grad):
                raise RuntimeError("missing/nonfinite gradient")
            opt.step()
            samples += args.batch
            step = samples // args.batch
            row = dict(step=step, loss=float(value.detach()), seconds=round(time.monotonic() - begin, 4),
                       samples_per_s=round(samples / (time.monotonic() - train_started), 2),
                       rss_mb=round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024),
                       temp_c=temperature())
            log.write(json.dumps(row) + "\n")
            peak_temp = max([peak_temp] + ([row["temp_c"]] if row["temp_c"] is not None else []))
            if step % 100 == 0:
                print("STEP", step, json.dumps(row), flush=True)
        point = evaluate(curve_idx)
        curve.append(dict(step=target, samples=samples, **point))
        print("EVAL", target, json.dumps(point), flush=True)
    log.close()
    train_seconds = time.monotonic() - train_started

    final_idx = test
    if 0 < args.final_samples < len(test):
        final_idx = rng.choice(test, size=args.final_samples, replace=False)
        final_idx.sort()
    final = evaluate(final_idx)
    checkpoint = args.out / "checkpoint.pt"
    torch.save(dict(model=model.state_dict(), processor=proc, source_sha256=sha(args.source),
                    dataset_sha256=sha(args.dataset), seed=args.seed), checkpoint)
    del model
    reload = ACT(cfg)
    saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
    reload.load_state_dict(saved["model"])
    model = reload
    reload_index = test[:min(128, len(test))]
    first, second = infer(reload_index), infer(reload_index)
    reload_error = float(np.abs(first - second).max())

    report = dict(
        schema="phase11b-training/1", mode="local CPU training", device="cpu", precision="float32",
        seed=args.seed, steps=args.steps, batch=args.batch, threads=args.threads, lr=args.lr,
        parameters=parameters, source_sha256=sha(args.source), dataset_sha256=sha(args.dataset),
        dataset=str(args.dataset), python=platform.python_version(), torch=torch.__version__,
        train_samples=int(len(train)), test_samples=int(len(test)),
        train_episodes=int(len(set(data["episode"][train]))),
        test_episodes=int(len(set(data["episode"][test]))),
        curve=curve, final=final, final_samples=int(len(final_idx)), checkpoint=str(checkpoint), checkpoint_sha256=sha(checkpoint),
        reload_max_abs_diff_rad=reload_error,
        train_seconds=round(train_seconds, 1),
        throughput_samples_per_s=round(samples / train_seconds, 2),
        peak_rss_mb=round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024),
        start_temp_c=temperature(), peak_temp_c=peak_temp,
        wall_seconds=round(time.monotonic() - started, 1),
        scope="simulated product-distribution corpus; held-out error is not a physical-task result")
    (args.out / "report.json").write_text(json.dumps(report, indent=2))
    print("DONE", json.dumps({k: report[k] for k in ("seed", "final", "throughput_samples_per_s",
                                                     "peak_rss_mb", "peak_temp_c", "reload_max_abs_diff_rad")}))


if __name__ == "__main__":
    main()
