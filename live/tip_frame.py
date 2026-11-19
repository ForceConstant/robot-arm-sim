"""The one chain(tool) -> jaw-tip frame used by the live solver.

The chain ends at the ``so101:gripper`` body (the arm's wrist-roll output).
Cartesian targets mean the *jaw tip*, so every target, every guard check and
every reported position goes through this constant.  The axial extent is not a
literal any more: it is derived from the tool that is physically installed and
stamped into ``live/tool_tip.json`` by ``tools/derive_tool_tip.py``.

Why a stamp and not a number in this file: the constant belongs to the tool, not
to the solver.  The stock SO-101 finger tip sits 104.425 mm below the body
origin; the installed CAD parallel finger sits 110.450 mm below it.  Those are
6 mm apart, and a wrong constant is invisible to the move gate (the solver and
the gate use the same transform, so the bias cancels) while it puts the real tip
6 mm into the board.  Reading the pack's own measurement removes the chance to
drift.

Two ways to resolve:

* **stock** - no ``gripper`` in ``live_config.json``.  The stock literal is used
  and the resolution records that; nothing is silent.
* **a configured pack** - ``live/tool_tip.json`` must exist for *that same pack*
  and be stamped ``ok``.  Missing, stale, or ``ok: false`` is a startup error,
  not a fallback.  Re-run ``tools/derive_tool_tip.py`` after any pack change.

Orientation is unchanged: the tip frame's +z points back up the fingers, so the
approach is its -z, and +y spans the jaws.  The origin stays on the chain axis -
an off-axis origin turns the solver's weak in-plane rotation into millimetres of
tip error, which is exactly the a1 24 mm failure.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "live_config.json"
DEFAULT_STAMP = HERE / "tool_tip.json"

#: The stock SO-101 finger tip, audited in ``docs/TIP_FRAME_CONVENTION.md``.
STOCK_TIP_M = 0.104425


def _resolve_pack(config_path: Path, stamp_path: Path) -> dict:
    config = json.loads(Path(config_path).read_text())
    kind = str(config.get("gripper") or "stock")

    if kind == "stock":
        print("tip_frame: no pack configured (gripper=stock); using the stock "
              f"finger tip {STOCK_TIP_M * 1000:.3f} mm", file=sys.stderr)
        return {"gripper_origin_to_tip_m": STOCK_TIP_M,
                "pack": "stock", "asset_root": None,
                "source_method": "stock SO-101 literal (tools/audit_tip_frame.py)",
                "ok": True, "used_stock": True,
                "stamp_path": None, "measured_at": None, "git_head": None,
                "cross_check_delta_m": None}

    raw = config.get("gripper_assets")
    if not raw:
        raise RuntimeError(
            f"live_config.json sets gripper={kind!r} but no gripper_assets; "
            "cannot resolve the installed tool")
    root = Path(raw)
    if not root.is_absolute():
        root = repo_root() / root
    root = root.resolve()

    if not Path(stamp_path).exists():
        raise RuntimeError(
            f"no tool-tip stamp at {stamp_path} for pack {kind!r}; "
            "run tools/derive_tool_tip.py")
    stamp = json.loads(Path(stamp_path).read_text())

    if not stamp.get("ok"):
        raise RuntimeError(
            "tool-tip stamp is not ok "
            f"(measured {stamp.get('gripper_origin_to_tip_m')}, profile "
            f"{stamp.get('profile_expected_m')}, delta "
            f"{stamp.get('cross_check_delta_m')}); fix the pack and re-run "
            "tools/derive_tool_tip.py")
    if stamp.get("pack") != kind:
        raise RuntimeError(
            f"stale tool-tip stamp: it describes pack {stamp.get('pack')!r}, "
            f"live_config.json selects {kind!r}; re-run tools/derive_tool_tip.py")
    stamped_root = stamp.get("asset_root")
    if stamped_root and Path(stamped_root).name != root.name:
        raise RuntimeError(
            f"stale tool-tip stamp: it measured {stamped_root}, live_config.json "
            f"points at {root}; re-run tools/derive_tool_tip.py")

    return {"gripper_origin_to_tip_m": float(stamp["gripper_origin_to_tip_m"]),
            "pack": kind, "asset_root": str(root),
            "source_method": stamp.get("source_method"),
            "ok": bool(stamp["ok"]), "used_stock": False,
            "stamp_path": str(stamp_path),
            "measured_at": stamp.get("measured_at"),
            "git_head": stamp.get("git_head"),
            "cross_check_delta_m": stamp.get("cross_check_delta_m")}


def repo_root() -> Path:
    """Repository root: ``live/`` is one level down."""
    return HERE.parent


def resolve(config_path: Path | None = None,
            stamp_path: Path | None = None) -> dict:
    """Resolve the tip constant for the configured tool (raises if unusable)."""
    return _resolve_pack(Path(config_path or DEFAULT_CONFIG),
                         Path(stamp_path or DEFAULT_STAMP))


RESOLVED = resolve()


def describe() -> dict:
    """What the solver thinks it is holding, in millimetres, for ``/state``."""
    out = dict(RESOLVED)
    out["gripper_origin_to_tip_mm"] = round(
        float(RESOLVED["gripper_origin_to_tip_m"]) * 1000.0, 4)
    out["gripper_origin_to_tip_m"] = float(RESOLVED["gripper_origin_to_tip_m"])
    return out


#: ``T_chain_tip``: tip-frame origin in the ``so101:gripper`` body frame.
#: On the wrist axis at the fingertip plane of the installed tool.
TIP_IN_CHAIN = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, -float(RESOLVED["gripper_origin_to_tip_m"])],
    [0.0, 0.0, 0.0, 1.0],
])
