"""Unit tests for ``tools/repin_rig_state.py`` (docs/TESTS.md §0c).

Everything runs against a throwaway copy of the pinned set and the pins file,
so the real ``rig_state/`` and ``tests/test_frame_pins.py`` are never touched.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "repin_rig_state.py"

NEW_BOARD = {"x_mm": -11.50, "y_mm": -37.72, "z_mm": 423.77, "yaw_deg": -165.50}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture()
def rig(tmp_path: Path) -> Path:
    (tmp_path / "tools").mkdir()
    shutil.copyfile(TOOL, tmp_path / "tools" / "repin_rig_state.py")
    (tmp_path / "rig_state").mkdir()
    for item in (ROOT / "rig_state").iterdir():
        if item.is_file():
            shutil.copyfile(item, tmp_path / "rig_state" / item.name)
    (tmp_path / "tests").mkdir()
    shutil.copyfile(ROOT / "tests" / "test_frame_pins.py",
                    tmp_path / "tests" / "test_frame_pins.py")
    (tmp_path / "live").mkdir()
    (tmp_path / "live" / "board.json").write_text(json.dumps(NEW_BOARD, indent=2) + "\n")
    (tmp_path / "live" / "live_config.json").write_text('{"unchanged": true}\n')
    return tmp_path


def run(rig: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(rig / "tools" / "repin_rig_state.py"), *args],
                          cwd=rig, capture_output=True, text=True)


def manifest(rig: Path) -> dict:
    return json.loads((rig / "rig_state" / "MANIFEST.json").read_text())


def test_refuses_without_a_reason(rig: Path) -> None:
    no_flag = run(rig, "--file", "board.json")
    assert no_flag.returncode != 0
    blank = run(rig, "--why", "   ", "--file", "board.json")
    assert blank.returncode != 0
    assert "REFUSED" in blank.stdout
    # Nothing may have been written by a refused run.
    assert manifest(rig)["files"]["board.json"]["sha256"] != digest(rig / "live" / "board.json")


def test_refuses_a_file_outside_the_pinned_set(rig: Path) -> None:
    result = run(rig, "--why", "should not work", "--file", "tool_tip.json")
    assert result.returncode != 0
    assert "not a pinned file" in result.stdout


def test_repin_updates_manifest_snapshot_pins_and_log(rig: Path) -> None:
    why = "board registration 2026-11-27 (session mat placement)"
    old_hash = manifest(rig)["files"]["board.json"]["sha256"]
    result = run(rig, "--why", why, "--file", "board.json")
    assert result.returncode == 0, result.stdout + result.stderr

    new_hash = digest(rig / "live" / "board.json")
    assert new_hash != old_hash
    entry = manifest(rig)["files"]["board.json"]
    assert entry["sha256"] == new_hash
    assert entry["bytes"] == (rig / "live" / "board.json").stat().st_size
    assert digest(rig / "rig_state" / "board.json") == new_hash
    # Manifest formatting and the other entries survive the rewrite.
    raw = (rig / "rig_state" / "MANIFEST.json").read_text()
    assert raw.startswith('{\n  "schema": "rig-state/1",')
    assert raw.endswith("}\n")
    assert manifest(rig)["files"]["wrist_calibration.json"]["sha256"] != new_hash

    pins = (rig / "tests" / "test_frame_pins.py").read_text()
    assert f'"board.json": "{new_hash}"' in pins
    assert old_hash not in pins
    for axis, value in NEW_BOARD.items():
        assert f'"{axis}": {value:.2f}' in pins

    log = (rig / "rig_state" / "PIN_LOG.md").read_text()
    assert why in log
    assert old_hash[:10] in log and new_hash[:10] in log


def test_repin_of_a_non_board_file_leaves_the_board_pose_alone(rig: Path) -> None:
    before = (rig / "tests" / "test_frame_pins.py").read_text()
    assert run(rig, "--why", "config touch", "--file", "live_config.json").returncode == 0
    after = (rig / "tests" / "test_frame_pins.py").read_text()
    board_block = before.split("BOARD_EXPECTED = {")[1].split("}")[0]
    assert board_block in after
    assert digest(rig / "rig_state" / "live_config.json") == digest(rig / "live" / "live_config.json")


def test_second_run_is_a_no_op(rig: Path) -> None:
    assert run(rig, "--why", "first pin", "--file", "board.json").returncode == 0
    log_after_first = (rig / "rig_state" / "PIN_LOG.md").read_text()
    again = run(rig, "--why", "second pin", "--file", "board.json")
    assert again.returncode == 0
    assert "unchanged" in again.stdout
    assert (rig / "rig_state" / "PIN_LOG.md").read_text() == log_after_first
