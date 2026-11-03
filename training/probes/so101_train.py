"""Turnkey ACT training harness for the public SO-101 chess dataset (Phase 11a).

Runs inside the GPU container; reads a LeRobot v3 dataset from /data, trains
full-size ACT on one camera, evaluates a held-out split, checkpoints and can
resume. Writes JSONL metrics and a summary. Samples GPU temperature and aborts
above the configured limit. Offline only: never sends robot commands.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

CHUNK = 30
SQ = "abcdefgh"


def bootstrap(libs: Path):
    sys.path.insert(0, str(libs))
    try:
        import av  # noqa: F401
        import pyarrow  # noqa: F401
    except ModuleNotFoundError:
        subprocess.run([sys.executable, "-m", "pip", "install", "--quiet",
                        "--disable-pip-version-check", "--target", str(libs),
                        "pandas", "pyarrow", "av"], check=True)
        sys.path.insert(0, str(libs))


def gpu_sample():
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu,memory.used,memory.total,clocks_throttle_reasons.active",
                              "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=10)
        parts = [p.strip() for p in out.stdout.strip().splitlines()[0].split(",")]
        return {"temp_c": float(parts[0]), "mem_used_mib": float(parts[1]),
                "mem_total_mib": float(parts[2]), "throttle": parts[3] if len(parts) > 3 else None}
    except Exception:
        return {}


class Monitor(threading.Thread):
    def __init__(self, out: Path, limit_c: float, interval: float = 5.0):
        super().__init__(daemon=True)
        self.out, self.limit_c, self.interval = out, limit_c, interval
        self.stop_flag, self.peak, self.abort = threading.Event(), 0.0, False

    def run(self):
        with (self.out / "gpu.jsonl").open("w") as f:
            while not self.stop_flag.is_set():
                s = gpu_sample()
                if s:
                    s["t"] = round(time.time(), 3)
                    f.write(json.dumps(s) + "\n")
                    f.flush()
                    self.peak = max(self.peak, s["temp_c"])
                    if s["temp_c"] >= self.limit_c:
                        self.abort = True
                        return
                self.stop_flag.wait(self.interval)


def goal_from_tasks(tasks):
    text = tasks[0] if isinstance(tasks, (list, tuple)) and len(tasks) else str(tasks)
    try:
        src = text.split(" on ")[1].split(" ")[0]
        dst = text.split(" on ")[-1].split(" ")[0].strip(" .")
        out = []
        for sq in (src, dst):
            out += [(SQ.index(sq[0]) - 3.5) * 0.028, (int(sq[1]) - 1 - 3.5) * 0.028, 0.0]
        return out
    except Exception:
        return [0.0] * 6


def load_index(root: Path, camera: str):
    import pyarrow.parquet as pq
    ep_files = sorted((root / "meta/episodes").rglob("*.parquet"))
    episodes = []
    for f in ep_files:
        episodes += pq.read_table(str(f)).to_pylist()
    data_files = sorted((root / "data").rglob("*.parquet"))
    rows = 0
    for f in data_files:
        rows += pq.read_table(str(f)).num_rows
    episodes = [e for e in episodes if e["dataset_to_index"] <= rows
                and e["length"] >= CHUNK + 5]
    key = f"videos/observation.images.{camera}"
    for e in episodes:
        if key + "/file_index" not in e:
            raise ValueError(f"camera {camera!r} missing from dataset metadata")
    return episodes, rows, data_files


def read_states(data_files, lo, hi):
    import pyarrow.parquet as pq
    import numpy as np
    out = []
    have = 0
    for f in data_files:
        t = pq.read_table(str(f), columns=["observation.state", "action"]).to_pylist()
        if have + len(t) <= lo:
            have += len(t)
            continue
        a = max(lo, have)
        b = min(hi, have + len(t))
        out += t[a - have:b - have]
        have += len(t)
        if have >= hi:
            break
    st = np.asarray([r["observation.state"] for r in out], dtype=np.float32)
    ac = np.asarray([r["action"] for r in out], dtype=np.float32)
    return st, ac


def collect_frames(root: Path, camera: str, episodes, per_ep: int):
    import av
    import numpy as np
    from PIL import Image

    key = f"videos/observation.images.{camera}"
    by_file = {}
    for e in episodes:
        by_file.setdefault((int(e[key + "/chunk_index"]), int(e[key + "/file_index"])), []).append(e)

    frames = {e["episode_index"]: [] for e in episodes}
    for (chunk_index, file_index), group in by_file.items():
        path = (root / f"videos/observation.images.{camera}"
                / f"chunk-{chunk_index:03d}" / f"file-{file_index:03d}.mp4")
        if not path.exists():
            continue
        ranges = [(float(e[key + "/from_timestamp"]), float(e[key + "/to_timestamp"]),
                   e["episode_index"]) for e in group]
        with av.open(str(path)) as c:
            stream = c.streams.video[0]
            for frame in c.decode(stream):
                t = frame.time
                if t is None:
                    continue
                for start, stop, eid in ranges:
                    if start - 1e-2 <= t <= stop + 1e-2:
                        frames[eid].append(np.asarray(frame.to_image().resize((224, 224), Image.Resampling.BOX)))
                        break
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path, default=Path("/data/so101/full"))
    ap.add_argument("--camera", default="wrist", choices=("top", "wrist"))
    ap.add_argument("--device", default="cuda", choices=("cuda", "cpu"))
    ap.add_argument("--episodes", type=int, default=40)
    ap.add_argument("--selection", choices=("random", "first"), default="random",
                    help="first = lowest episode indices (already-downloaded smoke tests)")
    ap.add_argument("--per-episode", type=int, default=16)
    ap.add_argument("--test-episodes", type=int, default=5)
    ap.add_argument("--steps", type=int, default=400)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--minutes", type=float, default=0.0, help="run until this wall time instead of --steps")
    ap.add_argument("--lr", type=float, default=1e-5)
    ap.add_argument("--temp-limit", type=float, default=85.0)
    ap.add_argument("--duty-train", type=float, default=0.0,
                    help="seconds of training per duty cycle (0 = continuous)")
    ap.add_argument("--duty-cool", type=float, default=60.0,
                    help="seconds of idle cooling per duty cycle")
    ap.add_argument("--resume", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    if not 0 < args.episodes <= 6095 or not 1 <= args.per_episode <= 128:
        ap.error("episode/sample bounds")

    args.out.mkdir(parents=True, exist_ok=False)
    (args.out / "summary.json").write_text(json.dumps({"status": "running", "args": vars(args) | {"out": str(args.out), "root": str(args.root), "resume": str(args.resume) if args.resume else None}}, indent=1))

    bootstrap(Path("/data/pylibs"))
    import numpy as np
    import torch

    sys.path.insert(0, "/tmp")
    ACT_URL = ("https://raw.githubusercontent.com/huggingface/lerobot/"
               "2774d9bddcbbda50e697e162e89e7eaada8d7105/src/lerobot/policies/act/modeling_act.py")
    RT_URL = ("https://raw.githubusercontent.com/ForceConstant/robot-arm-sim/"
              "main/training/probes/act_runtime.py")
    for url, name in ((ACT_URL, "/tmp/modeling_act.py"), (RT_URL, "/tmp/act_runtime.py")):
        with urllib.request.urlopen(url, timeout=60) as r, open(name, "wb") as f:
            f.write(r.read())
    from act_runtime import load_act, config, loss

    dev = torch.device(args.device)
    if dev.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("cuda requested but unavailable")

    episodes, rows, data_files = load_index(args.root, args.camera)
    episodes.sort(key=lambda e: e["episode_index"])
    rng = np.random.default_rng(20261103)
    if args.selection == "first":
        episodes = episodes[:args.episodes]
    else:
        episodes = [episodes[i] for i in rng.permutation(len(episodes))[:args.episodes]]
        episodes.sort(key=lambda e: e["episode_index"])

    t0 = time.monotonic()
    frames = collect_frames(args.root, args.camera, episodes, args.per_episode)
    decode_s = time.monotonic() - t0

    samples = []
    for e in episodes:
        lo, hi = e["dataset_from_index"], e["dataset_to_index"]
        st, ac = read_states(data_files, lo, hi)
        fr = frames.get(e["episode_index"], [])
        n = min(len(fr), len(st))
        if n < CHUNK + 2:
            continue
        goal = np.asarray(goal_from_tasks(e["tasks"]), dtype=np.float32)
        for idx in np.linspace(0, n - 2, args.per_episode, dtype=int):
            end = min(idx + CHUNK, n)
            chunk = ac[idx:end]
            pad = np.zeros(CHUNK, dtype=bool)
            pad[len(chunk):] = True
            if len(chunk) < CHUNK:
                chunk = np.concatenate([chunk, np.repeat(chunk[-1:], CHUNK - len(chunk), axis=0)])
            samples.append({"image": fr[idx], "state": st[idx], "action": chunk,
                            "pad": pad, "goal": goal, "episode": e["episode_index"]})
    if not samples:
        key = f"videos/observation.images.{args.camera}"
        wanted = {(int(e[key + "/chunk_index"]), int(e[key + "/file_index"])) for e in episodes}
        present = {(int(p.parent.name.split("-")[1]), int(p.stem.split("-")[1]))
                   for p in (args.root / f"videos/observation.images.{args.camera}").rglob("*.mp4")}
        raise RuntimeError(f"no samples built; video files present={len(present)} "
                           f"needed={sorted(wanted)} missing={sorted(wanted - present)}")

    eps = sorted({s["episode"] for s in samples})
    test_eps = set(eps[-args.test_episodes:]) if len(eps) > args.test_episodes else set(eps[-1:])
    train = [s for s in samples if s["episode"] not in test_eps]
    test = [s for s in samples if s["episode"] in test_eps]
    if not train or not test:
        raise RuntimeError("empty split")

    st_mean = np.mean([s["state"] for s in train], 0)
    st_std = np.maximum(np.std([s["state"] for s in train], 0), 0.01)
    ac_all = np.concatenate([s["action"][~s["pad"]] for s in train])
    ac_mean, ac_std = ac_all.mean(0), np.maximum(ac_all.std(0), 0.01)
    img_mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    img_std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def to_batch(rows_):
        imgs = torch.from_numpy(np.stack([r["image"] for r in rows_]).astype(np.float32) / 255).permute(0, 3, 1, 2)
        imgs = (imgs - torch.tensor(img_mean)[None, :, None, None]) / torch.tensor(img_std)[None, :, None, None]
        return {
            "observation.images": [imgs.to(dev)],
            "observation.state": ((torch.from_numpy(np.stack([r["state"] for r in rows_])) - torch.tensor(st_mean)) / torch.tensor(st_std)).to(dev),
            "observation.environment_state": torch.from_numpy(np.stack([r["goal"] for r in rows_])).to(dev),
            "action": ((torch.from_numpy(np.stack([r["action"] for r in rows_])) - torch.tensor(ac_mean)) / torch.tensor(ac_std)).to(dev),
            "action_is_pad": torch.from_numpy(np.stack([r["pad"] for r in rows_])).to(dev),
        }

    torch.manual_seed(20261103)
    if dev.type == "cuda":
        torch.cuda.manual_seed_all(20261103)
    model = load_act("/tmp/modeling_act.py")(config()).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4, foreach=False)
    start_step = 0
    if args.resume:
        ckpt = torch.load(args.resume, map_location=dev, weights_only=False)
        model.load_state_dict(ckpt["model"])
        if "optimizer" in ckpt:
            opt.load_state_dict(ckpt["optimizer"])
        start_step = int(ckpt.get("step", 0))
        st_mean, st_std = ckpt["state_mean"], ckpt["state_std"]
        ac_mean, ac_std = ckpt["action_mean"], ckpt["action_std"]

    monitor = Monitor(args.out, args.temp_limit)
    if dev.type == "cuda":
        monitor.start()
    metrics = (args.out / "metrics.jsonl").open("w")
    history, t0 = [], time.monotonic()
    stop_at = t0 + args.minutes * 60 if args.minutes else None
    step = start_step
    phase_start = time.monotonic()
    while True:
        if stop_at is not None:
            if time.monotonic() >= stop_at:
                break
        elif step - start_step >= args.steps:
            break
        if monitor.abort:
            raise RuntimeError(f"GPU temperature reached {args.temp_limit} C")
        if args.duty_train > 0 and time.monotonic() - phase_start >= args.duty_train:
            cool_until = time.monotonic() + args.duty_cool
            print(json.dumps({"duty_cool_s": args.duty_cool,
                              "step": step, "temp_c": gpu_sample().get("temp_c")}), flush=True)
            while time.monotonic() < cool_until:
                time.sleep(5)
            phase_start = time.monotonic()
        idx = rng.choice(len(train), size=min(args.batch, len(train)), replace=False)
        b = to_batch([train[i] for i in idx])
        opt.zero_grad(set_to_none=True)
        value = loss(model, b)
        value.backward()
        if not torch.isfinite(value):
            raise RuntimeError("nonfinite loss")
        opt.step()
        step += 1
        history.append(float(value.detach()))
        if len(history) % 25 == 0:
            row = {"step": step, "loss": round(history[-1], 4),
                   "samples_s": round(len(idx) * len(history) / (time.monotonic() - t0), 2),
                   "elapsed_s": round(time.monotonic() - t0, 1)}
            metrics.write(json.dumps(row) + "\n")
            metrics.flush()
            print(json.dumps(row), flush=True)

    monitor.stop_flag.set()
    model.eval()

    def evaluate(rows_):
        preds, errs, holds = [], [], []
        with torch.inference_mode():
            for i in range(0, len(rows_), args.batch):
                chunk = rows_[i:i + args.batch]
                b = to_batch(chunk)
                out = model({k: v for k, v in b.items() if k not in ("action", "action_is_pad")})[0]
                p = out.cpu().numpy() * ac_std + ac_mean
                a = np.stack([r["action"] for r in chunk])
                st = np.stack([r["state"] for r in chunk])[:, None, :]
                valid = ~np.stack([r["pad"] for r in chunk])
                preds.append(p)
                errs.append(((p - a) ** 2)[valid])
                holds.append(((st - a) ** 2)[valid])
        return float(np.mean(np.concatenate(errs))), float(np.mean(np.concatenate(holds)))

    mse, hold = evaluate(test)
    ckpt_path = args.out / "checkpoint.pt"
    torch.save({"model": model.state_dict(), "optimizer": opt.state_dict(), "step": step,
                "state_mean": st_mean, "state_std": st_std,
                "action_mean": ac_mean, "action_std": ac_std,
                "camera": args.camera, "dataset": "XvKuoMing/so101_chess",
                "dataset_revision": "969ae3a886b2038dfa349a768cbf8a9d57dde583"}, ckpt_path)

    reloaded = load_act("/tmp/modeling_act.py")(config()).to(dev)
    rck = torch.load(ckpt_path, map_location=dev, weights_only=False)
    reloaded.load_state_dict(rck["model"])
    reloaded.eval()

    def predict(m):
        with torch.inference_mode():
            b = to_batch(test[:min(5, len(test))])
            return m({k: v for k, v in b.items() if k not in ("action", "action_is_pad")})[0]

    reload_exact = bool(torch.equal(predict(model), predict(reloaded)))
    summary = {
        "status": "completed", "camera": args.camera, "episodes_used": len({s["episode"] for s in samples}),
        "samples": len(samples), "train": len(train), "test": len(test),
        "decode_s": round(decode_s, 1), "steps": step, "start_step": start_step,
        "elapsed_s": round(time.monotonic() - t0, 1),
        "samples_per_s": round(len(train) * max(1, step - start_step) / max(1e-6, time.monotonic() - t0), 2),
        "loss_first": round(history[0], 4) if history else None,
        "loss_last": round(history[-1], 4) if history else None,
        "test_mse_deg2": round(mse, 3), "hold_mse_deg2": round(hold, 3),
        "beats_hold": bool(mse < hold),
        "gpu_peak_c": monitor.peak if dev.type == "cuda" else None, "temp_limit_c": args.temp_limit,
        "duty_train_s": args.duty_train, "duty_cool_s": args.duty_cool,
        "reload_exact": reload_exact,
        "checkpoint_MB": round(ckpt_path.stat().st_size / 1e6, 1),
        "dataset_rows": rows, "selection": args.selection, "device": args.device,
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
