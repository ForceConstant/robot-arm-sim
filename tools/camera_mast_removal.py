#!/usr/bin/env python3
"""Before/after render: the retired yellow/black overhead camera mast.

The mast was removed from the scene builder on Brian's 2026-11-25 request.  The
BEFORE frame is reconstructed by adding the four retired geoms to a fresh spec;
the AFTER frame is the current default scene.  The lens position is identical
either way (mast_lens(board)), so only the visible structure changes.

Run on the BC-250:

    ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \\
      MUJOCO_GL=egl .venv/bin/python tools/camera_mast_removal.py

Outputs reports/camera_mast_removal/: before.png, after.png, comparison.png,
manifest.json.  Pixel statistics and a changed-pixel check are printed so a
blank render cannot be mistaken for a successful removal.
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from chess_sim.config import AppearanceConfig, BoardConfig  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.scene import build_scene  # noqa: E402

OUT = ROOT / "reports" / "camera_mast_removal"
W, H = 640, 480
MAST_GEOMS = ("mast_plate", "mast_lower", "mast_upper", "mast_camera")


def git_head() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"],
                                       cwd=ROOT, text=True).strip()
    except Exception:
        return "unknown"


def stats(frame: np.ndarray) -> dict:
    return {"mean": round(float(frame.mean()), 2),
            "std": round(float(frame.std()), 2),
            "distinct_colors": int(len(np.unique(frame.reshape(-1, 3), axis=0))),
            "size": [int(frame.shape[1]), int(frame.shape[0])]}


def pose_joints() -> dict:
    recs = json.loads((ROOT / "reports" / "block5m_frames.json").read_text())
    return next(r for r in recs if r["pose"] == "marker14")["joints_deg"]


def add_retired_mast(spec, board) -> None:
    """Re-add the four geoms removed from scene.py, for the BEFORE frame only."""
    ax, ay, az = board.arm_base
    plate, mx, lower = 0.010, ax + 0.16, 0.20
    for name, size, pos, rgba in [
        ("mast_plate", [0.14, 0.07, plate / 2], [ax + 0.08, ay, az + plate / 2],
         [0.78, 0.86, 0.20, 1]),
        ("mast_lower", [0.015, 0.015, lower / 2], [mx, ay, az + plate + lower / 2],
         [0.78, 0.86, 0.20, 1]),
        ("mast_upper", [0.015, 0.015, (0.60 - lower) / 2],
         [mx, ay, az + plate + lower + (0.60 - lower) / 2], [0.08, 0.08, 0.08, 1]),
        ("mast_camera", [0.015, 0.03, 0.02], [mx, ay + 0.02, az + plate + 0.60 + 0.02],
         [0.08, 0.08, 0.08, 1]),
    ]:
        spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX,
                                size=size, pos=pos, rgba=rgba,
                                contype=0, conaffinity=0)


def build(mast: bool, joint_qpos, joints: dict):
    board = BoardConfig()
    spec = build_scene(board, AppearanceConfig(aim_laser=False))
    if mast:
        add_retired_mast(spec, board)
    m = spec.compile()
    d = mujoco.MjData(m)
    d.qpos[joint_qpos[:5]] = np.radians([joints[k] for k in
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")])
    d.qpos[joint_qpos[5]] = np.radians(joints["gripper"])
    mujoco.mj_forward(m, d)
    geoms = [m.geom(i).name for i in range(m.ngeom)]
    mast_geoms = sorted(g for g in geoms if g in MAST_GEOMS)
    return m, d, mast_geoms


def render_orbit(renderer, m, d, orbit) -> np.ndarray:
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.lookat[:] = np.asarray(orbit["lookat"], float)
    cam.distance = float(orbit["distance"])
    cam.azimuth = float(orbit["azimuth"])
    cam.elevation = float(orbit["elevation"])
    renderer.update_scene(d, camera=cam)
    return renderer.render()


def changed(a: np.ndarray, b: np.ndarray) -> dict:
    diff = np.max(np.abs(a.astype(np.int16) - b.astype(np.int16)), axis=2) > 12
    ys, xs = np.nonzero(diff)
    if len(xs) == 0:
        return {"pixels": 0, "bbox": None}
    return {"pixels": int(len(xs)),
            "bbox": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    board = BoardConfig()
    head = git_head()
    cfg_sha = hashlib.sha256((ROOT / "chess_sim" / "config.py").read_bytes()).hexdigest()
    scene_sha = hashlib.sha256((ROOT / "chess_sim" / "scene.py").read_bytes()).hexdigest()
    joints = pose_joints()

    # One env just to recover the joint qpos indices. Both compiled models come
    # from the same base spec (plus the four mast geoms), so indices match.
    env = ChessSimEnv()
    try:
        joint_qpos = env._joint_qpos
    finally:
        env.close()

    views = {
        "wide": {"lookat": [0.0, 0.0, board.top], "azimuth": 135.0,
                 "elevation": -32.0, "distance": 1.05},
        "mast": {"lookat": [0.12, -0.20, 0.78], "azimuth": 160.0,
                 "elevation": -12.0, "distance": 0.55},
    }

    bm, bd, before_geoms = build(True, joint_qpos, joints)
    am, ad, after_geoms = build(False, joint_qpos, joints)
    print(f"git {head[:12]}  config.py {cfg_sha[:12]}  scene.py {scene_sha[:12]}")
    print(f"BEFORE mast geoms: {before_geoms}")
    print(f"AFTER  mast geoms: {after_geoms}")
    if not before_geoms or after_geoms:
        raise SystemExit("mast toggle did not take: expected all before, none after")

    br = mujoco.Renderer(bm, height=H, width=W)
    ar = mujoco.Renderer(am, height=H, width=W)
    frames: dict[str, dict[str, np.ndarray]] = {}
    manifest = {"tool": "tools/camera_mast_removal.py", "git_head": head,
                "config_sha256": cfg_sha, "scene_sha256": scene_sha,
                "mast_geoms_before": before_geoms, "mast_geoms_after": after_geoms,
                "aim_laser": False, "pose": joints, "views": {}, "images": {}}

    for name, orbit in views.items():
        b = render_orbit(br, bm, bd, orbit)
        a = render_orbit(ar, am, ad, orbit)
        diff = changed(b, a)
        print(f"{name}: before {stats(b)}; after {stats(a)}; changed {diff}")
        if stats(b)["std"] < 5 or stats(a)["std"] < 5:
            raise SystemExit(f"{name}: near-uniform frame, render path is wrong")
        if diff["pixels"] < 100:
            raise SystemExit(f"{name}: mast not visible in frame ({diff['pixels']} changed px)")
        frames[name] = {"before": b, "after": a}
        manifest["views"][name] = {**orbit, "changed": diff}
        Image.fromarray(b).save(OUT / f"{name}_before.png")
        Image.fromarray(a).save(OUT / f"{name}_after.png")
        manifest["images"][f"{name}_before.png"] = stats(b)
        manifest["images"][f"{name}_after.png"] = stats(a)

    Image.fromarray(frames["wide"]["before"]).save(OUT / "before.png")
    Image.fromarray(frames["wide"]["after"]).save(OUT / "after.png")

    h, w = frames["wide"]["before"].shape[:2]
    banner = 74
    sheet = Image.new("RGB", (w * 2, (h + 34) * 2 + banner), (14, 14, 16))
    dr = ImageDraw.Draw(sheet)
    for line_num, line in enumerate([
        "OVERHEAD CAMERA MAST - BEFORE / AFTER",
        f"git {head[:12]}   retired mast restored / scene default   {w}x{h} per tile",
        "same pose and lens; only the yellow/black mast geometry is removed",
    ]):
        dr.text((10, 5 + line_num * 15), line, fill=(235, 235, 210))
    for row, name in enumerate(("wide", "mast")):
        y = banner + row * (h + 34)
        for col, state in enumerate(("before", "after")):
            x = col * w
            dr.text((x + 8, y + 6),
                    f"{name.upper()}  /  {state.upper()}  "
                    f"({'WITH' if state == 'before' else 'WITHOUT'} mast)",
                    fill=(255, 235, 120))
            sheet.paste(Image.fromarray(frames[name][state]), (x, y + 24))
    sheet.save(OUT / "comparison.png")
    manifest["images"]["comparison.png"] = stats(np.asarray(sheet))
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {OUT}: before.png after.png comparison.png manifest.json")


if __name__ == "__main__":
    main()
