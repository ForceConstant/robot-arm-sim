#!/usr/bin/env python3
"""FULL-PARITY-01 T8 sample images — sim side only.

Renders the shapes the human gate needs, with provenance printed on every
sheet so a sample can never be mistaken for evidence from a different commit:

  * external orthogonal views (front, side, top, iso), full arm + board;
  * a jaw/base close-up with a millimetre scale callout;
  * the wrist-camera view as the sim camera actually sees it.

The joint pose comes from the committed `reports/block5m_frames.json`
(marker14), the same pose the historical orientation sheet uses, so these
samples are comparable to earlier receipts. No IRL photo is used or required:
this is a sim-side sample set for Brian to review while IRL work is underway.

Run on the BC-250:

    ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \
      MUJOCO_GL=egl .venv/bin/python tools/full_parity_sample_images.py

Outputs (reports/full_parity_samples/):
    sim_external.png      4-up front/side/top/iso + provenance
    sim_jaw_closeup.png   jaw seam + mm scale bar
    sim_wrist.png         wrist camera view
    manifest.json         git head, constants, pixel statistics
"""
from __future__ import annotations

import hashlib
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import mujoco  # noqa: E402
from chess_sim.config import Camera, EnvConfig, sim_wrist_roll  # noqa: E402
from chess_sim.scene import wrist_camera_fovy  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from live.tip_frame import TIP_IN_CHAIN  # noqa: E402

OUT = ROOT / "reports" / "full_parity_samples"


def git_head() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"],
                                       cwd=ROOT, text=True).strip()
    except Exception:
        return "unknown"


def provenance() -> list[str]:
    config = json.loads((ROOT / "live" / "live_config.json").read_text())
    stamp = json.loads((ROOT / "live" / "tool_tip.json").read_text())
    board = json.loads((ROOT / "rig_state" / "board.json").read_text())
    from chess_sim import tool_reference
    return [
        f"FULL-PARITY-01 sim sample  git {git_head()[:12]}",
        f"tool {stamp['pack']} / {config.get('gripper_assets', 'n/a')}",
        f"tip z {-float(TIP_IN_CHAIN[2, 3]):.5f} m  solver {config.get('ik_solver')}",
        f"board x{board['x_mm']} y{board['y_mm']} z{board['z_mm']} yaw{board['yaw_deg']}",
        f"camera: sim B5 mount, fovy {tool_reference.fovy_deg():.3f} deg, "
        f"roll {sim_wrist_roll(0.0):+.2f} deg offset (live->sim)",
    ]


def caption(draw: ImageDraw.ImageDraw, lines: list[str], width: int,
            y: int, *, fill=(240, 240, 210)) -> int:
    for line in lines:
        draw.text((10, y), line, fill=fill)
        y += 14
    return y


def _projector(cam_pos, cam_R, fovy, w, h):
    """Return a world-point -> (u, v, depth) projector for a camera pose.

    cam_R columns are the camera right/up/back axes in world coordinates
    (MuJoCo cam_xmat / MjvCamera.mat convention).  Camera looks down its -z.
    """
    fy = (h / 2.0) / np.tan(np.radians(fovy) / 2.0)
    cx, cy = w / 2.0, h / 2.0

    def project(point):
        v = np.asarray(point, float) - cam_pos
        z = -float(v @ cam_R[:, 2])          # forward depth
        if z <= 1e-6:
            return None
        return (cx + fy * float(v @ cam_R[:, 0]) / z,
                cy + fy * float(v @ cam_R[:, 1]) / z, z)

    return project


def draw_laser(frame, project, world_pts, board_hit, labels=True):
    """Draw the aim beam through any camera, from the tip to the board hit."""
    from PIL import ImageDraw as _Draw

    h, w = frame.shape[:2]
    pts = [p for p in (project(v) for v in world_pts) if p is not None]
    beam = [(int(u), int(v)) for u, v, _ in pts if 0 <= u < w and 0 <= v < h]
    out = Image.fromarray(frame).convert("RGB")
    dr = _Draw.Draw(out)
    # Match the prior tipcheck 'laser' look: one thin bright-red debug line
    # and a small red dot at the board, no glow or cylinder.
    if beam:
        dr.line(beam, fill=(255, 0, 0), width=2, joint="curve")
    hit = project(board_hit)
    if hit and 0 <= hit[0] < w and 0 <= hit[1] < h:
        x, y = int(hit[0]), int(hit[1])
        dr.ellipse((x - 4, y - 4, x + 4, y + 4), fill=(255, 0, 0))
        if labels:
            dr.text((x + 8, y - 16), "aim", fill=(255, 60, 60))
    return np.asarray(out), (len(beam), hit)


def wrist_laser(m, d, frame):
    """Legacy wrist-camera laser (kept for the sim_wrist sample)."""
    h, w = frame.shape[:2]
    cam = m.camera(str(Camera.WRIST))
    cam_pos = np.asarray(d.cam_xpos[cam.id], float)
    cam_R = np.asarray(d.cam_xmat[cam.id], float).reshape(3, 3)
    fovy = float(np.asarray(cam.fovy).reshape(-1)[0])
    project = _projector(cam_pos, cam_R, fovy, w, h)
    proj = _tool_geometry(m, d, project)
    out, diag = draw_laser(frame, project, proj["world"], proj["hit"])
    print(f"wrist laser: beam pixels {diag[0]}, hit uv "
          f"{tuple(round(float(v), 1) for v in diag[1][:2]) if diag[1] else None}")
    return out


def _tool_geometry(m, d, project=None, board_z=None):
    """Jaw origin, tool axis, tip, and the board-plane impact of the ray."""
    gid = m.body("so101:gripper").id
    origin = np.asarray(d.xpos[gid], float)
    R = np.asarray(d.xmat[gid], float).reshape(3, 3)
    axis = -R[:, 2]                          # tool +z points back up the jaws
    depth = float(json.loads((ROOT / "live" / "tool_tip.json").read_text())
                  ["gripper_origin_to_tip_m"])
    tip = origin + depth * axis
    if board_z is None:
        from chess_sim.config import BoardConfig
        board_z = BoardConfig().top
    # ray: origin + s*axis; solve z = board_z
    s = (board_z - origin[2]) / axis[2] if abs(axis[2]) > 1e-6 else 0.0
    hit = origin + s * axis
    world = [origin + t * axis for t in np.linspace(0.0, max(s, 0.05), 61)]
    return {"origin": origin, "axis": axis, "tip": tip, "hit": hit,
            "world": world, "standoff_mm": s * 1000.0, "board_z": board_z}


def third_person_laser(m, d, env, geo):
    """Render the aim beam from an external orbit camera."""
    # Frame the whole scene like the prior tipcheck 3rd-person renders:
    # look at the board centre, elevated ~45 deg, wide enough to show the arm.
    from chess_sim.config import BoardConfig
    lookat = np.array([0.0, 0.0, BoardConfig().top])
    az, el, dist = 135.0, -35.0, 1.20
    frame = env.render(orbit={"lookat": lookat, "distance": dist,
                              "azimuth": az, "elevation": el})
    h, w = frame.shape[:2]
    # Ask MuJoCo itself for the free-camera pose so the projected ray agrees
    # exactly with the rendered pixels (my hand-rolled geometry did not).
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = np.asarray(lookat, float)
    cam.distance = float(dist)
    cam.azimuth = float(az)
    cam.elevation = float(el)
    head = np.zeros(3); fwd = np.zeros(3); upv = np.zeros(3); rightv = np.zeros(3)
    mujoco.mjv_cameraFrame(head, fwd, upv, rightv, d, cam)
    # head is the camera position; cam_xmat columns are right, up, back=-fwd.
    cam_pos = np.asarray(head, float)
    cam_R = np.column_stack([rightv, upv, -fwd])
    fovy = 45.0
    project = _projector(cam_pos, cam_R, fovy, w, h)
    out_arr, diag = draw_laser(frame, project, geo["world"], geo["hit"])
    out = Image.fromarray(out_arr).convert("RGB")
    # label
    dr = ImageDraw.Draw(out)
    dr.text((12, 12), "3rd person + laser", fill=(255, 255, 255))
    dr.text((12, 30), f"ray origin -> tip -> board; standoff {geo['standoff_mm']:.0f} mm, "
            f"hit x{(geo['hit'][0]-lookat[0])*1000:.0f} y{(geo['hit'][1]-lookat[1])*1000:.0f} mm",
            fill=(255, 255, 255))
    print(f"third-person laser: azimuth {az} el {el} dist {dist}; "
          f"standoff {geo['standoff_mm']:.1f} mm; beam pixels {diag[0]}; "
          f"hit uv {tuple(round(float(v), 1) for v in diag[1][:2]) if diag[1] else None}")
    return np.asarray(out), {"azimuth": az, "elevation": el, "distance": dist,
                 "lookat": lookat.tolist(),
                 "standoff_mm": round(geo["standoff_mm"], 2),
                 "board_hit_world_mm": [round(float(v) * 1000, 1) for v in geo["hit"]]}


def stats(img: np.ndarray) -> dict:
    return {"mean": round(float(img.mean()), 2),
            "std": round(float(img.std()), 2),
            "distinct_colors": int(len(np.unique(img.reshape(-1, img.shape[-1]), axis=0))),
            "size": [int(img.shape[1]), int(img.shape[0])]}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joints", default=None,
                        help="live joint degrees pan,lift,elbow,flex,roll,gripper; "
                             "default is the committed block5m marker14 pose")
    args = parser.parse_args()
    if args.joints:
        values = [float(v) for v in args.joints.split(",")]
        if len(values) != 6:
            parser.error("--joints needs six values")
        joints = dict(zip(("shoulder_pan", "shoulder_lift", "elbow_flex",
                           "wrist_flex", "wrist_roll", "gripper"), values))
    else:
        rec = next(r for r in json.loads((ROOT / "reports" / "block5m_frames.json").read_text())
                   if r["pose"] == "marker14")
        joints = rec["joints_deg"]
    env = ChessSimEnv(EnvConfig())
    try:
        m, d = env.model, env.data
        # The live wrist_roll and the sim wrist_roll are different zeroes:
        # skipping sim_wrist_roll() renders the pose ~87 deg out (tip-center.md
        # §3.3, Brian 2026-11-24).  This render used to skip it.
        sim_joints = [joints[k] for k in
                      ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")]
        sim_joints.append(sim_wrist_roll(joints["wrist_roll"]))
        d.qpos[env._joint_qpos[:5]] = np.radians(sim_joints)
        d.qpos[env._joint_qpos[5]] = np.radians(joints["gripper"])
        mujoco.mj_forward(m, d)

        # Render the wrist at the measured lens for pixel comparison, not the
        # 47 deg marker-size fudge used by the scene (tip-center.md §3.4).
        from chess_sim import tool_reference
        cam_id = m.camera(str(Camera.WRIST)).id
        m.cam_fovy[cam_id] = tool_reference.fovy_deg()
        print(f"wrist pixel-comparison fovy {tool_reference.fovy_deg():.4f} deg "
              f"(scene default {wrist_camera_fovy():.4f})")

        gid = m.body("so101:gripper").id
        lookat = d.xpos[gid] + d.xmat[gid].reshape(3, 3) @ np.array([0, 0, -0.070])
        views = [("front  az0 el0", 0, 0, 0.45),
                 ("side   az90 el0", 90, 0, 0.45),
                 ("top    az0 el-89", 0, -89, 0.45),
                 ("iso    az35 el-25", 35, -25, 0.45)]
        tiles = []
        for label, az, el, dist in views:
            frame = env.render(orbit={"lookat": lookat, "distance": dist,
                                      "azimuth": az, "elevation": el})
            tiles.append((label, frame))
        banner = 14 * (len(provenance()) + 1)
        # The aim ray, from jaw origin through the tip to the board impact.
        geo = _tool_geometry(m, d)
        laser, laser_meta = third_person_laser(m, d, env, geo)
        laser_sheet = Image.new("RGB", (laser.shape[1], laser.shape[0] + banner),
                                (14, 14, 16))
        laser_sheet.paste(Image.fromarray(laser), (0, banner))
        caption(ImageDraw.Draw(laser_sheet), provenance(), laser_sheet.width, 4)
        laser_sheet.save(OUT / "sim_aim_laser_3p.png")

        wrist = env.render(camera=Camera.WRIST)
        wrist = wrist_laser(m, d, wrist)

        h, w = tiles[0][1].shape[:2]
        sheet = Image.new("RGB", (w * 2, (h + 22) * 2 + banner), (14, 14, 16))
        dr = ImageDraw.Draw(sheet)
        y = caption(dr, provenance(), sheet.width, 4)
        for i, (label, frame) in enumerate(tiles):
            x, ty = (i % 2) * w, banner + (i // 2) * (h + 22)
            sheet.paste(Image.fromarray(frame), (x, ty + 20))
            dr.text((x + 8, ty + 3), label, fill=(255, 235, 120))
        sheet.save(OUT / "sim_external.png")

        # Jaw close-up: tighter orbit plus a drawn 20 mm scale bar.
        close = env.render(orbit={"lookat": lookat, "distance": 0.22,
                                  "azimuth": 35, "elevation": -25})
        ci = Image.fromarray(close)
        cd = ImageDraw.Draw(ci)
        ch, cw = close.shape[:2]
        bar_px = int(round(0.020 / 0.22 * (cw / (2 * np.tan(np.radians(45) / 2)))))
        bar_px = max(8, min(bar_px, cw // 3))
        x0, y0 = 20, ch - 30
        cd.line([(x0, y0), (x0 + bar_px, y0)], fill=(120, 255, 130), width=3)
        cd.text((x0, y0 - 16), "20 mm scale bar (approx framing)", fill=(120, 255, 130))
        cd.text((20, 8), f"jaw seam / fingertip plane - tip depth {stamp_depth()} mm",
                fill=(255, 235, 120))
        close_sheet = Image.new("RGB", (cw, ch + banner), (14, 14, 16))
        close_sheet.paste(ci, (0, banner))
        caption(ImageDraw.Draw(close_sheet), provenance(), cw, 4)
        close_sheet.save(OUT / "sim_jaw_closeup.png")

        wi = Image.fromarray(wrist)
        wsheet = Image.new("RGB", (wrist.shape[1], wrist.shape[0] + banner), (14, 14, 16))
        wsheet.paste(wi, (0, banner))
        caption(ImageDraw.Draw(wsheet), provenance(), wsheet.width, 4)
        wsheet.save(OUT / "sim_wrist.png")

        manifest = {
            "tool": "tools/full_parity_sample_images.py",
            "git_head": git_head(),
            "pose": joints,
            "note": "sim-only samples for T8; no IRL photo used; wrist view carries a simulated aim laser from the tool axis",
            "laser": {
                "third_person": laser_meta,
                "wrist": "tool -z axis projected through the model's wrist camera",
            },
            "images": {
                name: stats(np.asarray(Image.open(OUT / name).convert("RGB")))
                for name in ("sim_external.png", "sim_jaw_closeup.png",
                             "sim_wrist.png", "sim_aim_laser_3p.png")
            },
        }
        (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2))
        for name, info in manifest["images"].items():
            print(f"{name}: {info}")
    finally:
        env.close()


def stamp_depth() -> str:
    stamp = json.loads((ROOT / "live" / "tool_tip.json").read_text())
    return f"{float(stamp['gripper_origin_to_tip_m']) * 1000:.3f}"


if __name__ == "__main__":
    main()
