#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLCHAIN="$ROOT/.toolchain"
UV_VERSION="0.13.6"
UV_BIN="$TOOLCHAIN/uv"
PYTHON_INSTALL_DIR="$TOOLCHAIN/python"
PYTHON_BIN_DIR="$TOOLCHAIN/bin"
VENV="$ROOT/.venv"

mkdir -p "$TOOLCHAIN" "$PYTHON_BIN_DIR"

if [[ ! -x "$UV_BIN" ]]; then
    archive="$TOOLCHAIN/uv.tar.gz"
    curl --fail --location --retry 3 \
        "https://github.com/astral-sh/uv/releases/download/${UV_VERSION}/uv-x86_64-unknown-linux-gnu.tar.gz" \
        --output "$archive"
    tar -xzf "$archive" -C "$TOOLCHAIN" --strip-components=1 "uv-x86_64-unknown-linux-gnu/uv"
    rm -f "$archive"
fi

export UV_PYTHON_INSTALL_DIR="$PYTHON_INSTALL_DIR"
export UV_PYTHON_BIN_DIR="$PYTHON_BIN_DIR"
export UV_LINK_MODE=copy
export PATH="$PYTHON_BIN_DIR:$PATH"
"$UV_BIN" python install 3.12
PYTHON="$($UV_BIN python find 3.12)"

valid_venv=false
if [[ -x "$VENV/bin/python" ]]; then
    if "$VENV/bin/python" -c 'import sys; assert sys.version_info[:2] == (3, 12)' >/dev/null 2>&1; then
        venv_base="$($VENV/bin/python -c 'import sys; print(sys.base_prefix)')"
        case "$venv_base/" in
            "$PYTHON_INSTALL_DIR"/*) valid_venv=true ;;
        esac
    fi
fi

if [[ "$valid_venv" != true ]]; then
    echo "Rebuilding stale or non-project Python 3.12 venv: $VENV"
    "$UV_BIN" venv --clear --python "$PYTHON" "$VENV"
fi

"$UV_BIN" pip sync --python "$VENV/bin/python" \
    "$ROOT/requirements.lock" "$ROOT/requirements-test.lock"
"$VENV/bin/python" -c 'import sys; print("Project venv:", sys.executable); print("Python:", sys.version.split()[0]); print("Base:", sys.base_prefix)'
