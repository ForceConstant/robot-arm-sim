#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLCHAIN="$ROOT/.toolchain"
UV_VERSION="0.13.6"
UV_BIN="$TOOLCHAIN/uv"
BASE_PYTHON="${PYTHON311:-/usr/bin/python3.11}"
VENV="$ROOT/.venv"

mkdir -p "$TOOLCHAIN"

if [[ ! -x "$UV_BIN" ]]; then
    archive="$TOOLCHAIN/uv.tar.gz"
    curl --fail --location --retry 3 \
        "https://github.com/astral-sh/uv/releases/download/${UV_VERSION}/uv-x86_64-unknown-linux-gnu.tar.gz" \
        --output "$archive"
    tar -xzf "$archive" -C "$TOOLCHAIN" --strip-components=1 "uv-x86_64-unknown-linux-gnu/uv"
    rm -f "$archive"
fi

export UV_LINK_MODE=copy
if [[ ! -x "$BASE_PYTHON" ]] || ! "$BASE_PYTHON" -c 'import sys; assert sys.version_info[:2] == (3, 11)' >/dev/null 2>&1; then
    echo "Python 3.11 is required; expected executable: $BASE_PYTHON" >&2
    exit 1
fi
EXPECTED_BASE="$($BASE_PYTHON -c 'import sys; print(sys.base_prefix)')"

valid_venv=false
if [[ -x "$VENV/bin/python" ]]; then
    if "$VENV/bin/python" -c 'import sys; assert sys.version_info[:2] == (3, 11)' >/dev/null 2>&1; then
        venv_base="$($VENV/bin/python -c 'import sys; print(sys.base_prefix)')"
        [[ "$venv_base" == "$EXPECTED_BASE" ]] && valid_venv=true
    fi
fi

if [[ "$valid_venv" != true ]]; then
    echo "Rebuilding stale or non-system Python 3.11 venv: $VENV"
    "$UV_BIN" venv --clear --python "$BASE_PYTHON" "$VENV"
fi

"$UV_BIN" pip sync --python "$VENV/bin/python" \
    "$ROOT/requirements.lock" "$ROOT/requirements-test.lock"
"$VENV/bin/python" -c 'import sys; print("Project venv:", sys.executable); print("Python:", sys.version.split()[0]); print("Base:", sys.base_prefix)'
