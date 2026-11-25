#!/usr/bin/env python3
"""Multi-view sheet of the current DEFAULT scene: no mast, aim laser visible.

Uses ChessSimEnv with the stock EnvConfig/AppearanceConfig — i.e. exactly what
a fresh baseline render gets after the mast removal: no yellow/black mast, and
the baked aim laser ON. No render-time pixel drawing: the red ray and yellow
impact are scene geoms driven by chess_sim.aim_laser.

Run on the BC-250:

    ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \\
      MUJOCO_GL=egl .venv/bin/python tools/mast_free_views.py

Outputs reports/mast_free_views/: one PNG per view + a 2x3 sheet + manifest.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import mujoco  # noqa: E402
from chess_sim.config import Camera, EnvConfig  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402

OUT = ROOT / "reports" / "mast_free_views"
RECOVERY_JOINTS = (-12.37, -13.02, 4.80, 98.34, 0.00, 38.4)
JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper")


def git_head() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"],
                                       cwd=ROOT, text=True).strip()
    except Exception:
        return "unknown"


def stats(frame: np.ndarray) -> dict:
    flat = frame.reshape(-1, 3).astype(int)
    red = int(((flat[:, 0] > 170) & (flat[:, 1] < 80) & (flat[:, 2] < 80)).sum())
    yellow = int(((flat[:, 0] > 190) & (flat[:, 1] > 160) & (flat[:, 2] < 90)).sum())
    return {"mean": round(float(frame.mean()), 2),
            "std": round(float(frame.std()), 2), "red_px": red, "yellow_px": yellow,
            "size": [int(frame.shape[1]), int(frame.shape[0])]}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    head = git_head()
    env = ChessSimEnv(EnvConfig())  # DEFAULT appearance: mast gone, laser ON
    try:
        print(f"git {head[:12]}  aim_laser={env.config.appearance.aim_laser}  "
              f"mast_geoms={[g for g in [env.model.geom(i).name for i in range(env.model.ngeom)] if g.startswith('mast_')]}")
        assert env._laser_on, "default scene must have the laser on"
        assert not any(env.model.geom(i).name.startswith("mast_")
                       for i in range(env.model.ngeom)), "default scene must have no mast"

        # Recovery pose: tool vertical above board centre, laser lands on a square.
        env.data.qpos[env._joint_qpos] = np.radians(RECOVERY_JOINTS)
        mujoco.mj_forward(env.model, env.data)

        board_top = float(env.board.top)
        views = [
            ("third_person", "orbit:135:-28:0.95"),
            ("close_gripper", "orbit:45:-22:0.34"),
            ("side", "orbit:90:-6:0.85"),
            ("top", Camera.TOP),
            ("external", Camera.EXTERNAL),
            ("wrist", Camera.WRIST),
        ]

        frames = {}
        manifest = {"tool": "tools/mast_free_views.py", "git_head": head,
                    "aim_laser": True, "mast_geoms": [],
                    "pose": dict(zip(JOINT_ORDER, RECOVERY_JOINTS)),
                    "images": {}}
        for name, spec in views:
            if isinstance(spec, str) and spec.startswith("orbit:"):
                az, el, dist = spec.split(":")[1:]
                orbit = {"lookat": [0.0, 0.0, board_top], "azimuth": float(az),
                         "elevation": float(el), "distance": float(dist)}
                frame = env.render(orbit=orbit)
            else:
                frame = env.render(camera=spec)
            frames[name] = frame
            st = stats(frame)
            manifest["images"][f"{name}.png"] = st
            Image.fromarray(frame).save(OUT / f"{name}.png")
            print(f"{name}: {st}")

        # 2x3 sheet with labels.
        h, w = next(iter(frames.values())).shape[:2]
        banner = 46
        cell_h = h + 24
        sheet = Image.new("RGB", (w * 3, cell_h * 2 + banner), (14, 14, 16))
        dr = ImageDraw.Draw(sheet)
        dr.text((10, 6), f"DEFAULT SIM SCENE  git {head[:12]}  -  no mast, aim laser ON",
                fill=(235, 235, 210))
        names = [v[0] for v in views]
        for i, name in enumerate(names):
            x, y = (i % 3) * w, banner + (i // 3) * cell_h
            dr.text((x + 8, y + 4), name.upper(), fill=(255, 235, 120))
            sheet.paste(Image.fromarray(frames[name]), (x, y + 20))
        sheet.save(OUT / "views_sheet.png")
        manifest["images"]["views_sheet.png"] = stats(np.asarray(sheet))
        (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"wrote {OUT}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
