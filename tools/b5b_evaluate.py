#!/usr/bin/env python3
"""Block 5b pixel-parity gate: does the sim camera put the corners where the
real camera put them?

For every capture in the run:
  * detect the ChArUco markers on the RAW 640x480 frame -- that is the frame the
    measured intrinsics belong to -- and undistort to ideal pinhole pixels with
    the measured lens;
  * pose the sim camera with the *measured* joint vector (wrist_roll through
    config.sim_wrist_roll) and the scene's own mount, and project the same
    board corners with the same lens;
  * rotate both sets 90 deg CW into the portrait 480x640 grid the sensor is
    mounted in, and compare corner for corner.

Both sides use the same lens and the same rotation, so focal length, principal
point and the portrait/landscape question cancel; what remains is geometry.

Valid-view rule (Block 4c, unchanged): >= 2 detected markers and no tag clipped
by the frame border (a corner closer than edge_px/6 + 1 px).  Excluded frames
are listed with their border distance and get an _excluded overlay.

Gate: median <= 2.0 px, max <= 4.0 px on the valid set.

A self-check renders the sim at the measured lens (fovy 42.028 deg, f 833 px)
and compares the detector's corners against the same projection in the portrait
frame, so a projection or rotation error cannot masquerade as parity.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from chess_sim import config, fiducials, scene  # noqa: E402
from handeye_bundle import board_corners  # noqa: E402

WRIST_MODE = "calibrated"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")
W_RAW, H_RAW = 640, 480          # capture as stored
W, H = 480, 640                  # portrait grid the sensor is mounted in
LENS = {"fx": 831.897, "fy": 833.014, "cx": 320.0, "cy": 240.0,
        "distortion": [-0.102251, 0.685733, 0.001112, 0.003239, 0.0]}
K = np.array([[LENS["fx"], 0.0, LENS["cx"]], [0.0, LENS["fy"], LENS["cy"]], [0.0, 0.0, 1.0]])
DIST = np.asarray(LENS["distortion"], float).reshape(-1, 1)
MEASURED_FOVY = 2.0 * math.degrees(math.atan((H_RAW / 2.0) / LENS["fy"]))
GATE = {"median_px": 2.0, "max_px": 4.0}


def rotate_cw(px: np.ndarray) -> np.ndarray:
    """640x480 landscape pixels -> 480x640 portrait pixels."""
    return np.column_stack([(H_RAW - 1) - px[:, 1], px[:, 0]])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", default="live/captures_b5b")
    ap.add_argument("--out", default="reports/b5b")
    ap.add_argument("--limit", type=int, default=40, help="overlays to write")
    ap.add_argument("--wrist", choices=("calibrated", "verbatim"),
                    default="calibrated",
                    help="how the measured wrist_roll maps into the sim")
    args = ap.parse_args()
    global WRIST_MODE
    WRIST_MODE = args.wrist
    cap = ROOT / args.captures
    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)

    board = json.loads((ROOT / "live/board.json").read_text())
    bc = config.BoardConfig(face="charuco",
                            origin=(board["x_mm"] / 1e3, board["y_mm"] / 1e3),
                            yaw=math.radians(board["yaw_deg"]), mat=0.0,
                            table_top=board["z_mm"] / 1e3 - 0.0032)
    model = scene.build_scene(board=bc,
                              appearance=config.AppearanceConfig()).compile()
    data = mujoco.MjData(model)
    cid = model.camera("wrist").id
    baked_fovy = float(model.cam_fovy[cid])
    th = math.radians(board["yaw_deg"])
    Rz = np.array([[math.cos(th), -math.sin(th), 0.0],
                   [math.sin(th), math.cos(th), 0.0], [0.0, 0.0, 1.0]])
    origin = np.array([board["x_mm"] / 1e3, board["y_mm"] / 1e3, board["z_mm"] / 1e3])
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
    renderer = mujoco.Renderer(model, height=H, width=W)

    def set_joints(joints):
        for n in JOINTS:
            v = joints[n]
            if n == "wrist_roll" and WRIST_MODE == "calibrated":
                v = config.sim_wrist_roll(v)
            data.qpos[model.jnt_qposadr[model.joint("so101:" + n).id]] = math.radians(v)
        mujoco.mj_forward(model, data)

    # Portrait camera matrix: rotating a 640x480 capture 90 deg CW puts the
    # landscape vertical on the portrait horizontal, so the focal lengths swap
    # and the principal point moves to (479 - cy, cx).  The rotated IRL pixels
    # live in exactly this frame; projecting the sim in the landscape frame and
    # rotating afterwards is a different camera (x/y swapped) and is wrong.
    K_PORTRAIT = np.array([[LENS["fy"], 0.0, (H_RAW - 1) - LENS["cy"]],
                           [0.0, LENS["fx"], LENS["cx"]],
                           [0.0, 0.0, 1.0]])

    def project_portrait(ids, scale, k=K_PORTRAIT):
        obj = np.vstack([board_corners(i) for i in ids])
        p_world = obj @ Rz.T + origin
        Rc = data.cam_xmat[cid].reshape(3, 3)
        t = data.cam_xpos[cid]
        p_cam = (p_world - t) @ Rc
        z = -p_cam[:, 2]
        return np.column_stack([k[0, 2] + scale * k[0, 0] * p_cam[:, 0] / z,
                                k[1, 2] - scale * k[1, 1] * p_cam[:, 1] / z])

    rows, valid_err, all_err, overlays = [], [], [], []
    for item in sorted(cap.glob("*.json")):
        if item.name == "index.json":
            continue
        rec = json.loads(item.read_text())
        if "measured_deg" not in rec:
            continue
        name = rec["name"]
        raw = cv2.imread(str(cap / f"{name}.jpg"))
        if raw is None:
            continue
        dets = fiducials.detect_markers(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB))
        # The detector returns markers in detection order, not id order; the
        # projection is built from the same id list, so sort the corners the
        # same way or every marker is compared against another's corners.
        dets = sorted(dets, key=lambda d: int(d.marker_id))
        ids = [int(d.marker_id) for d in dets]
        if not ids:
            rows.append({"name": name, "markers": 0, "valid": False,
                         "excluded_because": "no markers"})
            continue
        sides = [float(np.mean([np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]))
                 for c in (np.asarray(d.corners, float).reshape(4, 2) for d in dets)]
        obs_raw = np.vstack([np.asarray(d.corners, float).reshape(4, 2) for d in dets])
        obs_portrait = rotate_cw(obs_raw)
        border = float(min(obs_portrait[:, 0].min(), obs_portrait[:, 1].min(),
                           W - 1 - obs_portrait[:, 0].max(), H - 1 - obs_portrait[:, 1].max()))
        margin = min(sides) / 6.0 + 1.0
        clipped = border < margin
        single = len(ids) < 2
        valid = not (clipped or single)
        reasons = [r for r, flag in (("single tag", single),
                                     ("tag clipped at frame border", clipped)) if flag]

        # IRL: undistort in the raw frame, back to ideal pinhole pixels, rotate.
        n = cv2.undistortPoints(obs_raw.reshape(-1, 1, 2), K, DIST).reshape(-1, 2)
        on = cv2.projectPoints(np.column_stack([n, np.ones(len(n))]).reshape(-1, 1, 3),
                               np.zeros(3), np.zeros(3), K, np.zeros(5))[0].reshape(-1, 2)
        irl_portrait = rotate_cw(on)

        set_joints(rec["measured_deg"])
        pred = project_portrait(ids, 1.0, K_PORTRAIT)
        d = np.linalg.norm(pred - irl_portrait, axis=1)

        # Self-check: same projection against a render at the measured lens.
        model.cam_fovy[cid] = MEASURED_FOVY
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera="wrist", scene_option=opt)
        simframe = renderer.render().copy()
        model.cam_fovy[cid] = baked_fovy
        sim_dets = sorted(fiducials.detect_markers(cv2.cvtColor(simframe, cv2.COLOR_RGB2BGR)),
                          key=lambda x: int(x.marker_id))
        shared = [int(x.marker_id) for x in sim_dets if int(x.marker_id) in ids]
        self_max = None
        if len(shared) >= 2:
            f_self = H / 2 / math.tan(math.radians(MEASURED_FOVY) / 2)
            pred_s = project_portrait(shared, f_self / LENS["fy"])
            det_s = np.vstack([np.asarray(x.corners, float).reshape(4, 2)
                               for x in sim_dets if int(x.marker_id) in shared])
            self_max = float(np.linalg.norm(pred_s - det_s, axis=1).max())

        # Decompose the residual: a 2D similarity absorbs a range error (scale),
        # a mount/board yaw (rotation) and a board shift (translation).  What is
        # left after it is the part that is neither.
        fit = {}
        if len(ids) >= 2:
            M, _ = cv2.estimateAffinePartial2D(pred, irl_portrait)
            if M is not None:
                sc = float(np.hypot(M[0, 0], M[1, 0]))
                ang = float(math.degrees(math.atan2(M[1, 0], M[0, 0])))
                warped = (pred @ M[:, :2].T) + M[:, 2]
                fit = {"fit_scale": round(sc, 4), "fit_rot_deg": round(ang, 2),
                       "fit_t_px": [round(float(v), 1) for v in M[:, 2]],
                       "resid_after_fit_px": round(float(np.median(
                           np.linalg.norm(warped - irl_portrait, axis=1))), 2)}

        all_err.extend(d.tolist())
        if valid:
            valid_err.extend(d.tolist())
        rows.append({"name": name, "markers": len(ids), "ids": ids,
                     "corners": int(len(d)), "median_px": round(float(np.median(d)), 3),
                     "max_px": round(float(d.max()), 3), "mean_px": round(float(d.mean()), 3),
                     "border_px": round(border, 1), "clip_margin_px": round(margin, 1),
                     "valid": valid, "excluded_because": ", ".join(reasons),
                     "settle_error_deg": rec.get("settle_error_deg"),
                     "sim_selfcheck_max_px": None if self_max is None else round(self_max, 2),
                     **fit})

        if len(overlays) < args.limit:
            img = cv2.rotate(raw, cv2.ROTATE_90_CLOCKWISE)
            for mid, quad in zip(ids, (np.asarray(m.corners, float).reshape(4, 2) for m in dets)):
                cv2.polylines(img, [np.round(rotate_cw(quad)).astype(int).reshape(-1, 1, 2)],
                              True, (0, 255, 0), 1)
            start = 0
            for mid in ids:
                p = np.round(pred[start:start + 4]).astype(int)
                r = np.round(irl_portrait[start:start + 4]).astype(int)
                cv2.polylines(img, [p.reshape(-1, 1, 2)], True, (0, 0, 255), 1)
                cv2.putText(img, str(mid), tuple(p[0]), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 0), 1)
                for j in range(4):
                    e = float(d[start + j])
                    col = (0, 200, 0) if e <= GATE["max_px"] else (0, 0, 255)
                    cv2.putText(img, f"{e:.1f}", tuple(r[j]), cv2.FONT_HERSHEY_SIMPLEX,
                                0.35, col, 1)
                start += 4
            tag = "valid" if valid else "excluded"
            cv2.putText(img, f"{name} {tag} n={len(ids)} med {np.median(d):.2f} max {d.max():.2f}",
                        (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            path = out / f"{name}_overlay{'' if valid else '_excluded'}.png"
            cv2.imwrite(str(path), img)
            overlays.append(path)

    renderer.close()
    v = np.asarray(valid_err) if valid_err else np.asarray([np.nan])
    a = np.asarray(all_err) if all_err else np.asarray([np.nan])
    valid_views = [r for r in rows if r["valid"]]
    summary = {
        "captures_dir": args.captures,
        "wrist_mode": WRIST_MODE,
        "valid_view_rule": ">=2 detected markers and no tag clipped at the frame border "
                           "(corner closer than edge_px/6 + 1 px)",
        "gate_thresholds_px": GATE,
        "lens": LENS,
        "sim": {"mount": "reference (tool_reference.camera_in_gripper)",
                "baked_fovy_deg": round(baked_fovy, 4),
                "measured_fovy_deg": round(MEASURED_FOVY, 4),
                "board": board},
        "views": len(rows),
        "valid": {"views": len(valid_views), "corners": int(v.size),
                  "median_px": round(float(np.median(v)), 3),
                  "mean_px": round(float(np.mean(v)), 3),
                  "p90_px": round(float(np.percentile(v, 90)), 3),
                  "max_px": round(float(v.max()), 3),
                  "gate": {"median_pass": bool(np.median(v) <= GATE["median_px"]),
                           "max_pass": bool(v.max() <= GATE["max_px"]),
                           "pass": bool(np.median(v) <= GATE["median_px"]
                                        and v.max() <= GATE["max_px"])}},
        "all": {"corners": int(a.size), "median_px": round(float(np.median(a)), 3),
                "max_px": round(float(a.max()), 3)},
        "excluded": [r for r in rows if not r.get("valid")],
        "per_view": sorted(rows, key=lambda r: (not r.get("valid"), -float(r.get("max_px") or 0))),
        "overlays": [str(p.relative_to(ROOT)) for p in overlays],
    }
    (out / "pixel_parity.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(f"{'view':<26} {'mk':>2} {'cor':>3} {'median':>7} {'max':>7} "
          f"{'border':>7} {'clip':>6} {'self':>5}  verdict")
    for r in summary["per_view"]:
        print(f"{r['name']:<26} {r.get('markers', 0):2d} {r.get('corners', 0):3d} "
              f"{str(r.get('median_px', '-')):>7} {str(r.get('max_px', '-')):>7} "
              f"{str(r.get('border_px', '-')):>7} {str(r.get('clip_margin_px', '-')):>6} "
              f"{str(r.get('sim_selfcheck_max_px', '-')):>5}  "
              f"{'VALID' if r.get('valid') else 'excluded: ' + str(r.get('excluded_because'))}")
    print(f"\nvalid set: {summary['valid']['views']} views / {summary['valid']['corners']} corners")
    print(f"  median {summary['valid']['median_px']:.3f} px (gate {GATE['median_px']})"
          f"  p90 {summary['valid']['p90_px']:.3f}  max {summary['valid']['max_px']:.3f} px"
          f" (gate {GATE['max_px']})")
    print(f"  -> {'PASS' if summary['valid']['gate']['pass'] else 'FAIL'}")
    print("wrote", out / "pixel_parity.json")
    return 0 if summary["valid"]["gate"]["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
