#!/usr/bin/env python3
"""Derive the chain->tip axial constant from the installed tool pack.

The live solver holds one number: how far the jaw tip sits below the
``so101:gripper`` body origin along the body -z axis.  It must follow the tool
that is actually bolted on, so this tool measures the compiled model and writes
``live/tool_tip.json``; ``live/tip_frame.py`` reads that stamp and nothing else.

Measurement (same rule as ``tools/audit_tip_frame.py``): compile the arm with
the configured pack, put every vertex of every geom on the jaw bodies into the
``so101:gripper`` frame, and take ``max(-z)``.  Three apertures, so an open-jaw
corner cannot masquerade as the tip plane.

Two independent profile cross-checks, both recorded:

* **site** - ``cad_tool.install`` puts the site at ``tcp_position`` and the tip
  ``tip_reach_m`` beyond it, so the pack promises ``|tcp_z| + tip_reach``.  This
  is the check that decides ``ok``.
* **mount** - composing ``mount_position`` / ``mount_rotation`` with
  ``tcp_position`` yields ~173 mm, because the mount translation is applied
  twice (the base body already sits at the mount).  That number is the rejected
  ``-173.027`` from ``TIP_FRAME_CONVENTION.md``; it is recorded, never used.

    python tools/derive_tool_tip.py            # write the stamp
    python tools/derive_tool_tip.py --check    # exit 1 if not ok
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
APERTURES_DEG = (10.0, 38.4, 70.0)
TOLERANCE_M = 0.05e-3
STOCK_TIP_M = 0.104425


# --------------------------------------------------------------------- helpers
def _git_head() -> str:
    try:
        return subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, check=True).stdout.strip()
    except Exception:  # noqa: BLE001 - a stamp without a sha is still usable
        return "unknown"


def _config(path: Path) -> dict:
    return json.loads(Path(path).read_text())


def _resolve_root(config: dict, config_path: Path) -> Path:
    raw = config.get("gripper_assets")
    if not raw:
        raise SystemExit("no gripper_assets in the config; nothing to derive")
    path = Path(raw)
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


# ------------------------------------------------------------------ measuring
def _vertices(model, gid: int) -> np.ndarray:
    import mujoco
    kind = model.geom_type[gid]
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mid = int(model.geom_dataid[gid])
        adr = int(model.mesh_vertadr[mid])
        count = int(model.mesh_vertnum[mid])
        return np.asarray(model.mesh_vert[adr:adr + count], dtype=float).reshape(-1, 3)
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        size = model.geom_size[gid]
        return np.array([[i * size[0], j * size[1], k * size[2]]
                         for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return np.zeros((0, 3))


def measure_pack(asset_root: Path, kind: str = "cad_parallel",
                 home: dict | None = None) -> dict:
    """Compile one pack and measure the jaw tip in the gripper frame."""
    os.environ["ROBOT_GRIPPER"] = kind
    os.environ["ROBOT_GRIPPER_ASSETS"] = str(asset_root)
    os.environ.setdefault("MUJOCO_GL", "egl")
    sys.path.insert(0, str(ROOT))

    import mujoco  # noqa: E402
    from chess_sim import gripper, scene  # noqa: E402

    if not gripper.CUSTOM:
        raise SystemExit("pack not loaded: ROBOT_GRIPPER did not select cad_parallel")

    model = scene.build_arm().compile()
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "so101:gripper")
    home = home or {"shoulder_pan": 0.1, "shoulder_lift": -55.1, "elbow_flex": 72.0,
                    "wrist_flex": 39.3, "wrist_roll": 22.4, "gripper": 38.4}

    rows, verts_total = [], 0
    for aperture in (home["gripper"],) + tuple(
            a for a in APERTURES_DEG if abs(a - home["gripper"]) > 1e-9):
        pose = dict(home, gripper=aperture)
        for index, name in enumerate(JOINTS):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "so101:" + name)
            data.qpos[model.jnt_qposadr[jid]] = np.radians(float(pose[name]))
        mujoco.mj_forward(model, data)
        rotation = np.asarray(data.body(body).xmat, dtype=float).reshape(3, 3)
        origin = np.asarray(data.body(body).xpos, dtype=float)

        deepest, where, count = -1e9, None, 0
        for gid in range(model.ngeom):
            bname = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or ""
            if "jaw" not in bname:            # stock "moving_jaw_*" and CAD "parallel_jaw_*"
                continue
            corners = _vertices(model, gid)
            if not len(corners):
                continue
            world = (np.asarray(data.geom_xpos[gid], dtype=float)
                     + corners @ np.asarray(data.geom_xmat[gid], dtype=float).reshape(3, 3).T)
            local = (world - origin) @ rotation
            depth = float(-local[:, 2].min())
            count += len(corners)
            if depth > deepest:
                deepest, where = depth, [round(float(v), 3) for v in
                                      local[int(np.argmin(local[:, 2]))] * 1000.0]
        rows.append({"aperture_deg": aperture, "deepest_mm": round(deepest * 1000, 4),
                     "deepest_point_mm": where})
        verts_total = count

    depths = [r["deepest_mm"] for r in rows]
    return {"deepest_m": float(np.max(depths)) / 1000.0,
            "spread_mm": round(max(depths) - min(depths), 4),
            "poses": rows, "vertex_count": verts_total,
            "profile": json.loads((asset_root / "profile.json").read_text())}


def profile_checks(profile: dict, measured_m: float) -> dict:
    """Expected tip from the profile, plus the mount composition red herring."""
    tcp = np.asarray(profile["tcp_position"], dtype=float)
    mount_t = np.asarray(profile["mount_position"], dtype=float)
    # profile stores a rotation *matrix* here (cad_tool passes it to mju_mat2Quat)
    mount_r = np.asarray(profile["mount_rotation"], dtype=float).reshape(3, 3)
    reach = float(profile.get("tip_reach_m") or 0.0)

    expected = abs(float(tcp[2])) + reach
    composed = -float((mount_r @ tcp + mount_t)[2])   # positive depth, same sign as the tip
    return {"profile_tcp_z_m": float(tcp[2]),
            "profile_tip_reach_m": reach,
            "profile_expected_m": expected,
            "profile_composed_m": composed,
            "mount_z_m": float(mount_t[2]),
            "cross_check_delta_m": measured_m - expected,
            "composition_delta_m": measured_m - composed}


# ---------------------------------------------------------------------- stamp
def build_stamp(config: dict, config_path: Path) -> dict:
    kind = str(config.get("gripper") or "stock")
    if kind == "stock":
        return {"pack": "stock", "asset_root": None,
                "gripper_origin_to_tip_m": STOCK_TIP_M,
                "source_method": "stock SO-101 literal (tools/audit_tip_frame.py)",
                "profile_composed_m": None, "cross_check_delta_m": None,
                "ok": True, "measured_at": datetime.now(timezone.utc).isoformat(),
                "git_head": _git_head(), "spread_mm": 0.0, "poses": [],
                "note": "no pack configured; the stock model is the tool"}

    root = _resolve_root(config, config_path)
    if not (root / "profile.json").exists():
        raise SystemExit(f"pack at {root} has no profile.json")

    measurement = measure_pack(root, kind)
    checks = profile_checks(measurement["profile"], measurement["deepest_m"])
    ok = (abs(checks["cross_check_delta_m"]) <= TOLERANCE_M
          and measurement["spread_mm"] <= 0.05)
    return {"pack": kind,
            "asset_root": str(root),
            "gripper_origin_to_tip_m": round(measurement["deepest_m"], 9),
            "source_method": "compiled MJCF, max(-z) of jaw-body vertices in so101:gripper frame",
            "profile_composed_m": round(checks["profile_composed_m"], 9),
            "cross_check_delta_m": round(checks["cross_check_delta_m"], 9),
            "ok": bool(ok),
            "measured_at": datetime.now(timezone.utc).isoformat(),
            "git_head": _git_head(),
            "profile_expected_m": round(checks["profile_expected_m"], 9),
            "profile_tcp_z_m": round(checks["profile_tcp_z_m"], 9),
            "profile_tip_reach_m": round(checks["profile_tip_reach_m"], 9),
            "composition_delta_m": round(checks["composition_delta_m"], 9),
            "mount_z_m": round(checks["mount_z_m"], 9),
            "spread_mm": measurement["spread_mm"],
            "vertex_count": measurement["vertex_count"],
            "poses": measurement["poses"],
            "note": ("profile_composed_m composes the mount with tcp_position and is "
                     "~62.6 mm too deep by construction: the base body already sits at "
                     "the mount, so the translation is applied twice. Recorded for "
                     "audit only; the tip is |tcp_z| + tip_reach.")}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "live/live_config.json"))
    parser.add_argument("--out", default=str(ROOT / "live/tool_tip.json"))
    parser.add_argument("--check", action="store_true",
                        help="exit nonzero unless the stamp is ok")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    stamp = build_stamp(_config(config_path), config_path)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(stamp, indent=2) + "\n")

    print(f"pack            {stamp['pack']}")
    print(f"asset_root      {stamp['asset_root']}")
    print(f"tip             {stamp['gripper_origin_to_tip_m'] * 1000:.4f} mm "
          f"(spread {stamp['spread_mm']:.4f} mm over {len(stamp['poses'])} apertures)")
    if stamp["profile_composed_m"] is not None:
        print(f"profile expects {stamp['profile_expected_m'] * 1000:.4f} mm, "
              f"delta {stamp['cross_check_delta_m'] * 1000:+.4f} mm")
        print(f"mount-composed  {stamp['profile_composed_m'] * 1000:.4f} mm "
              f"(red herring, delta {stamp['composition_delta_m'] * 1000:+.4f} mm)")
    print(f"ok              {stamp['ok']}   -> {out}")
    if args.check and not stamp["ok"]:
        print("derive_tool_tip: stamp is NOT ok", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
