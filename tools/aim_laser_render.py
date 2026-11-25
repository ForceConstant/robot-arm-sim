#!/usr/bin/env python3
"""Aim-laser render — red ray from jaw origin through tip to the board.

Spec (Brian, 2026-11-25):
  * the red laser **emanates from the jaw origin** (the so101:gripper body
    origin, on the wrist axis), passes **through the tip centre** at 110.45 mm
    down that axis, and continues to the board plane;
  * a **yellow marker** sits at the board impact;
  * a clean third-person view with the gripper and board both visible.

This is the tool-axis ray documented in tip-center.md §1 (jaw origin -> tip ->
straight out), intersected with the sim board plane (§4a). The camera pose is
taken from MuJoCo itself via mjv_cameraFrame so the drawn line lands exactly on
the rendered pixels. No IRL data, no arm motion.

Run on the BC-250:

    ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \
      MUJOCO_GL=egl .venv/bin/python tools/aim_laser_render.py
"""
from __future__ import annotations

import json
import math
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
import sys  # noqa: E402
sys.path.insert(0, str(ROOT))
OUT = ROOT / "reports" / "full_parity_samples"

# Recovery pose (AGENTS.md standing rule): tip above the middle of the board,
# tool vertical. This is the only pose where the aim ray lands cleanly on a
# board square, and it is the pose the laser sketch is about.
RECOVERY_JOINTS = (-12.37, -13.02, 4.80, 98.34, 0.00, 38.4)


def git_head() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"],
                                       cwd=ROOT, text=True).strip()
    except Exception:
        return "unknown"


def frame_camera(m, d, lookat, azimuth, elevation, distance):
    """MuJoCo free-camera pose + fov, matching env.render(orbit=...)."""
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = np.asarray(lookat, float)
    cam.distance = float(distance)
    cam.azimuth = float(azimuth)
    cam.elevation = float(elevation)
    head = np.zeros(3); fwd = np.zeros(3); up = np.zeros(3); right = np.zeros(3)
    mujoco.mjv_cameraFrame(head, fwd, up, right, d, cam)
    # env.render() uses the model's global camera fov for free-Camera renders.
    fovy = float(np.asarray(m.vis.global_.fovy).reshape(-1)[0])
    return head, np.column_stack([right, up, -fwd]), fovy


def projector(cam_pos, cam_R, fovy, w, h):
    fy = (h / 2.0) / np.tan(np.radians(fovy) / 2.0)

    def proj(point):
        v = np.asarray(point, float) - cam_pos
        z = -float(v @ cam_R[:, 2])            # camera forward is -back
        if z <= 1e-6:
            return None
        # Image v grows downward; camera up axis grows upward, hence the minus.
        return (w / 2.0 + fy * float(v @ cam_R[:, 0]) / z,
                h / 2.0 - fy * float(v @ cam_R[:, 1]) / z, z)
    return proj


def render_shot(m, d, env, stamp, name, az, el, dist, board_z):
    """Render one view with the laser overlays, return image + facts."""
    depth = float(stamp["gripper_origin_to_tip_m"])
    gid = m.body("so101:gripper").id
    origin = np.asarray(d.xpos[gid], float)
    R = np.asarray(d.xmat[gid], float).reshape(3, 3)
    axis = -R[:, 2]                             # tool +z points back up the jaws
    tip = origin + depth * axis
    s = (board_z - origin[2]) / axis[2]
    hit = origin + s * axis
    lookat = np.array([origin[0], origin[1], board_z])

    frame = env.render(orbit={"lookat": lookat, "distance": dist,
                              "azimuth": az, "elevation": el})
    h, w = frame.shape[:2]
    cam_pos, cam_R, fovy = frame_camera(m, d, lookat, az, el, dist)
    proj = projector(cam_pos, cam_R, fovy, w, h)

    samples = [origin + t * axis for t in np.linspace(0.0, s, 81)]
    pixels = [proj(p) for p in samples]
    line = [(int(u), int(v)) for u, v, _ in pixels
            if u is not None and 0 <= u < w and 0 <= v < h]
    hit_px = proj(hit)

    img = Image.fromarray(frame).convert("RGB")
    dr = ImageDraw.Draw(img)
    if line:
        dr.line(line, fill=(255, 0, 0), width=2, joint="curve")
    if hit_px and 0 <= hit_px[0] < w and 0 <= hit_px[1] < h:
        x, y = int(hit_px[0]), int(hit_px[1])
        dr.ellipse((x - 6, y - 6, x + 6, y + 6), fill=(255, 220, 0),
                   outline=(90, 70, 0), width=1)
        dr.line((x - 11, y, x + 11, y), fill=(255, 220, 0), width=1)
        dr.line((x, y - 11, x, y + 11), fill=(255, 220, 0), width=1)
    dr.text((10, 10), "aim laser: jaw origin -> tip -> board", fill=(255, 255, 255))
    dr.text((10, 26), "red = ray, yellow = impact", fill=(255, 255, 0))

    stats = {"mean": round(float(frame.mean()), 2),
             "std": round(float(frame.std()), 2),
             "colors": int(len(np.unique(frame.reshape(-1, 3), axis=0))),
             "size": [w, h]}
    facts = {
        "name": name, "azimuth": az, "elevation": el, "distance": dist,
        "camera_pos_mm": [round(float(v) * 1000, 1) for v in cam_pos],
        "ray_pixels": len(line),
        "hit_pixel": [round(float(hit_px[0]), 1), round(float(hit_px[1]), 1)]
        if hit_px and np.isfinite(hit_px[0]) else None,
        "origin_mm": [round(float(v) * 1000, 2) for v in origin],
        "tip_mm": [round(float(v) * 1000, 2) for v in tip],
        "hit_mm": [round(float(v) * 1000, 2) for v in hit],
        "standoff_mm": round(s * 1000, 2),
        "axis": [round(float(v), 5) for v in axis],
        "stats": stats,
    }
    return img, facts


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--elbow", default="",
                    help="comma-separated elbow angles (deg); sweep if given")
    ap.add_argument("--sweep", action="store_true",
                    help="render -20,0,10,20 elbow contact sheet")
    ap.add_argument("--angle", action="store_true",
                    help="render an azimuth sweep of the arm at the recovery pose")
    ap.add_argument("--az", default="-150,-90,-30,30",
                    help="comma-separated azimuths for --angle")
    ap.add_argument("--dist", type=float, default=0.80,
                    help="orbit distance for --angle (default 0.80 m)")
    args = ap.parse_args(sys.argv[1:])
    OUT.mkdir(parents=True, exist_ok=True)
    from chess_sim.config import EnvConfig, START_FEN
    from chess_sim.env import ChessSimEnv

    env = ChessSimEnv(EnvConfig())
    try:
        env.reset(START_FEN)
        m, d = env.model, env.data
        board_z = float(env.board.top)
        stamp = json.loads((ROOT / "live" / "tool_tip.json").read_text())

        if args.angle:
            azs = [float(x) for x in args.az.split(",")]
            d.qpos[env._joint_qpos[:5]] = np.radians(RECOVERY_JOINTS[:5])
            d.qpos[env._joint_qpos[5]] = np.radians(RECOVERY_JOINTS[5])
            mujoco.mj_forward(m, d)
            tiles = []
            manifest = {"tool": "tools/aim_laser_render.py",
                        "git_head": git_head(), "sweep": "azimuth", "shots": []}
            for az in azs:
                img, facts = render_shot(m, d, env, stamp, f"az_{az:+.0f}",
                                         az, -30.0, args.dist, board_z)
                manifest["shots"].append(facts)
                tiles.append((az, img))
                img.save(OUT / f"aim_laser_az_{az:+.0f}.png")
                print(f"az {az:+.0f}: ray px {facts['ray_pixels']} "
                      f"hit {facts['hit_pixel']} stats {facts['stats']}")
            th, tw = tiles[0][1].size[1], tiles[0][1].size[0]
            sheet = Image.new("RGB", (tw * 2, th * 2), (12, 12, 12))
            dr = ImageDraw.Draw(sheet)
            for i, (az, tile) in enumerate(tiles):
                x, y = (i % 2) * tw, (i // 2) * th
                sheet.paste(tile, (x, y))
                dr.rectangle([x, y, x + 170, y + 20], fill=(0, 0, 0))
                dr.text((x + 6, y + 4), f"azimuth {az:+.0f} deg", fill=(255, 255, 255))
            sheet.save(OUT / "aim_laser_angle_sweep.png")
            (OUT / "aim_laser_angle_manifest.json").write_text(
                json.dumps(manifest, indent=2))
            return

        if args.elbow or args.sweep:
            elbows = ([float(x) for x in args.elbow.split(",")] if args.elbow
                      else [-20.0, 0.0, 10.0, 20.0])
            tiles = []
            manifest = {"tool": "tools/aim_laser_render.py",
                        "git_head": git_head(), "sweep": "elbow", "shots": []}
            for elbow in elbows:
                joints = list(RECOVERY_JOINTS)
                joints[2] = elbow
                d.qpos[env._joint_qpos[:5]] = np.radians(joints[:5])
                d.qpos[env._joint_qpos[5]] = np.radians(joints[5])
                mujoco.mj_forward(m, d)
                img, facts = render_shot(m, d, env, stamp, f"elbow_{elbow:+.0f}",
                                         -150.0, -30.0, 0.80, board_z)
                facts["elbow_deg"] = elbow
                manifest["shots"].append(facts)
                tiles.append((elbow, img))
                img.save(OUT / f"aim_laser_elbow_{elbow:+.0f}.png")
                print(f"elbow {elbow:+.0f}: standoff {facts['standoff_mm']:.1f} mm, "
                      f"hit {facts['hit_mm']}, ray px {facts['ray_pixels']}")
            # 2-column contact sheet
            th, tw = tiles[0][1].size[1], tiles[0][1].size[0]
            sheet = Image.new("RGB", (tw * 2, th * 2), (12, 12, 12))
            dr = ImageDraw.Draw(sheet)
            for i, (elbow, tile) in enumerate(tiles):
                x, y = (i % 2) * tw, (i // 2) * th
                sheet.paste(tile, (x, y))
                dr.rectangle([x, y, x + 150, y + 20], fill=(0, 0, 0))
                dr.text((x + 6, y + 4), f"elbow {elbow:+.0f} deg", fill=(255, 255, 255))
            sheet.save(OUT / "aim_laser_elbow_sweep.png")
            (OUT / "aim_laser_elbow_manifest.json").write_text(
                json.dumps(manifest, indent=2))
            return

        # Default: the two verified views at the recovery pose.
        d.qpos[env._joint_qpos[:5]] = np.radians(RECOVERY_JOINTS[:5])
        d.qpos[env._joint_qpos[5]] = np.radians(RECOVERY_JOINTS[5])
        mujoco.mj_forward(m, d)
        manifest = {"tool": "tools/aim_laser_render.py",
                    "git_head": git_head(), "pose": RECOVERY_JOINTS,
                    "spec": "jaw origin -> tip centre (110.45 mm) -> board impact; red line, yellow impact marker",
                    "pack": stamp["pack"], "tip_depth_m": stamp["gripper_origin_to_tip_m"],
                    "shots": []}
        for name, az, el, dist in (("close", -135.0, -38.0, 0.42),
                                   ("wide", -150.0, -30.0, 0.80)):
            img, facts = render_shot(m, d, env, stamp, name, az, el, dist, board_z)
            img.save(OUT / f"aim_laser_{name}.png")
            manifest["shots"].append(facts)
            print(f"{name}: ray px {facts['ray_pixels']} | hit {facts['hit_pixel']} "
                  f"| {facts['stats']}")
        (OUT / "aim_laser_manifest.json").write_text(json.dumps(manifest, indent=2))
    finally:
        env.close()


if __name__ == "__main__":
    main()
