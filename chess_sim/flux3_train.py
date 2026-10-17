"""FLUX 3 Action training on one 32 GB card: text contexts from disk, not from Qwen.

The frozen Qwen3-VL text encoder takes 8.3 GB of the card, and at batch 1 the video
VAE's activations on top of it run the card out of memory as soon as instructions
stop repeating - which in chess is at once: every square pair is its own sentence.
The instructions of a dataset are few, though (986 here), so each is encoded once,
ahead of training (`build_cache`), and training reads the cached context instead.
The encoder is replaced by an empty module, which frees the 8.3 GB and its load time.

Run in place of `lerobot.scripts.lerobot_train`, with the same flags:

    accelerate launch -m chess_sim.flux3_train --config_path=... --policy.path=...

The cache directory is FLUX3_TEXT_CACHE (default outputs/flux3_text_cache). A caption
missing from it is still encoded - by a CPU copy of the encoder, slowly, and logged.
"""
from __future__ import annotations

import hashlib
import logging
import os
import runpy
import sys

import torch
from torch import nn

CACHE_ENV = "FLUX3_TEXT_CACHE"
DEFAULT_CACHE = "outputs/flux3_text_cache"


def cache_dir() -> str:
    return os.environ.get(CACHE_ENV, DEFAULT_CACHE)


def cache_file(directory: str, encoder_id: str, fixed_length: int | None, caption: str) -> str:
    key = hashlib.sha1(f"{encoder_id}\n{fixed_length}\n{caption}".encode()).hexdigest()
    return os.path.join(directory, f"{key}.pt")


def build_cache(captions: list[str], encoder_id: str, fixed_length: int | None,
                directory: str, device: str = "cuda") -> int:
    """Encode every caption not yet cached, plus the empty one guidance uses. Returns how many."""
    from lerobot.policies.flux3.f3.text_encoder import load_text_encoder, text_context

    os.makedirs(directory, exist_ok=True)
    todo = [c for c in dict.fromkeys(["", *captions])
            if not os.path.exists(cache_file(directory, encoder_id, fixed_length, c))]
    if not todo:
        return 0
    encoder = load_text_encoder(encoder_id, device=device)
    for i, caption in enumerate(todo):
        ctx = text_context(encoder, caption, "cpu", fixed_length=fixed_length)
        path = cache_file(directory, encoder_id, fixed_length, caption)
        torch.save(ctx, path + ".tmp")
        os.replace(path + ".tmp", path)          # a killed build leaves no half-written entry
        if (i + 1) % 100 == 0:
            print(f"  {i + 1}/{len(todo)} captions encoded", flush=True)
    return len(todo)


class _NoEncoder(nn.Module):
    """Stands in for the text encoder: nothing to load, nothing to move."""


def install() -> None:
    """Point a Flux3Policy at the cache instead of its text encoder."""
    from lerobot.policies.flux3 import modeling_flux3
    from lerobot.policies.flux3.f3 import packing
    from lerobot.policies.flux3.f3.text_encoder import VEC_DIM, load_text_encoder, text_context

    policy_cls = modeling_flux3.Flux3Policy
    directory = cache_dir()
    fallback: dict[str, nn.Module] = {}

    def build_text_encoder(self, config):
        return _NoEncoder()

    def context(self, caption: str, device: torch.device):
        hit = self._ctx_cache.get(caption)
        if hit is not None and hit[0].device == device:
            return hit
        encoder_id, length = self.config.text_encoder_id, self.config.text_fixed_length
        path = cache_file(directory, encoder_id, length, caption)
        if os.path.exists(path):
            ctx = torch.load(path, map_location="cpu")
        else:
            logging.warning("caption not in %s, encoding on the CPU: %r", directory, caption)
            if "encoder" not in fallback:
                fallback["encoder"] = load_text_encoder(encoder_id, device="cpu")
            ctx = text_context(fallback["encoder"], caption, "cpu", fixed_length=length)
        ctx = ctx.to(device)
        if len(self._ctx_cache) > 256:
            self._ctx_cache.clear()
        self._ctx_cache[caption] = (ctx.to(self.dtype_), packing.pack_text(ctx, VEC_DIM)["ctx_ids"])
        return self._ctx_cache[caption]

    policy_cls._build_text_encoder = build_text_encoder
    policy_cls._context = context


def main() -> None:
    install()
    sys.argv[0] = "lerobot_train"
    runpy.run_module("lerobot.scripts.lerobot_train", run_name="__main__", alter_sys=True)


if __name__ == "__main__":
    main()
