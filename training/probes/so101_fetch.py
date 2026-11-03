"""Fetch the pinned public SO-101 chess dataset with hf_transfer (Phase 11a).

Resumable and content-addressed by the pinned revision. Downloads meta, data and
both camera streams. No uploads, no host changes.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

os.environ["HF_HUB_ENABLE_HF_TRANSFER"] = "1"
# Xet hashing is CPU-bound on this host and throttles the transfer well
# below the link rate; prefer plain ranged transfers.
os.environ["HF_HUB_DISABLE_XET"] = "1"
try:
    import hf_transfer  # noqa: F401
except ModuleNotFoundError:
    subprocess.run([sys.executable, "-m", "pip", "install", "--quiet",
                    "--disable-pip-version-check", "hf_transfer"], check=True)
    import hf_transfer  # noqa: F401

from huggingface_hub import snapshot_download

REPO = "XvKuoMing/so101_chess"
REVISION = "969ae3a886b2038dfa349a768cbf8a9d57dde583"


def main() -> None:
    path = snapshot_download(
        REPO,
        repo_type="dataset",
        revision=REVISION,
        local_dir=os.environ.get("SO101_DIR", "/data/so101/full"),
        max_workers=int(os.environ.get("SO101_WORKERS", "16")),
        allow_patterns=["meta/*", "data/*", "videos/*"],
    )
    print(json.dumps({"path": path, "repo": REPO, "revision": REVISION}))


if __name__ == "__main__":
    main()
