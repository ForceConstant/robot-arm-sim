"""Encode a dataset's instructions for FLUX 3 training (see chess_sim/flux3_train.py).

    python tools/flux3_text_cache.py --dataset-root datasets/lerobot/chess_flux_wide30 \
        --checkpoint black-forest-labs/flux-3-action-so101
"""
import argparse

import pandas as pd

from chess_sim.flux3_train import build_cache, cache_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--checkpoint", default="black-forest-labs/flux-3-action-so101",
                    help="the policy whose text encoder and text length the cache is for")
    ap.add_argument("--cache", default=None, help="default: $FLUX3_TEXT_CACHE or outputs/flux3_text_cache")
    args = ap.parse_args()
    import lerobot.policies  # noqa: F401 - registers every policy config
    from lerobot.configs.policies import PreTrainedConfig

    config = PreTrainedConfig.from_pretrained(args.checkpoint)
    captions = pd.read_parquet(f"{args.dataset_root}/meta/tasks.parquet").index.tolist()
    directory = args.cache or cache_dir()
    n = build_cache(captions, config.text_encoder_id, config.text_fixed_length, directory)
    print(f"{len(captions)} instructions; {n} newly encoded into {directory}")


if __name__ == "__main__":
    main()
