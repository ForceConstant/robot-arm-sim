#!/usr/bin/env python3
"""T2 - decompose the FK-vs-PnP error field into yaw + translation (+ scale).

Why this exists: the escalated diagnosis says the ~5 mm Cartesian error is two
stacked faults, not one. A pure +x command producing -0.138 in y is a ~22 deg
*rotation*, and gravity sag cannot rotate axes - only a frame error can. This
tool tests that directly, offline, with no arm motion:

  for every real captured view:
      PnP camera position in the marker board frame   (image only)
      FK  camera position from the recorded measured joints, taken into the
          same frame with the deployed board.json transform
  fit  fk_xy = R(theta) @ pnp_xy + t   (rigid, and rigid+scale)

  ~22 deg with a small residual  -> one board/hand-eye constant is wrong
  unstructured residual          -> not a rigid frame error; look at joints

Secondary metric: the jaw-tip constant. FK tip vs the tip pixel unprojected at
the FK height. A constant offset in tool z shows up here and not in the camera
fit, which is how a wrong TIP_IN_CHAIN is separated from a wrong board frame.

Read-only: reads manifests and images, opens no sockets, moves nothing.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/home/brian/robot-arm-sim")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import cv2  # noqa: E402

from chess_sim.board_target import Board  # noqa: E402

sys.path.insert(0, str(ROOT / "live"))
from tip_frame import TIP_IN_CHAIN  # noqa: E402


def frame_at(x_mm, y_mm, z_mm, yaw_deg):
    """Board frame in base coordinates: the same convention as live/app.py."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    m = np.eye(4)
    m[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = [x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0]
    return m


def rigid_fit(src, dst, allow_scale=False):
    """Best R, t (, s): dst ~= s*R@src + t.  Kabsch/Umeyama, 3D."""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    cov = b.T @ a / len(src)
    u, d, vt = np.linalg.svd(cov)
    s = np.ones(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[-1] = -1.0
    rot = u @ np.diag(s) @ vt
    scale = 1.0
    if allow_scale:
        var = (a ** 2).sum() / len(src)
        scale = float((d * s).sum() / var) if var > 0 else 1.0
    t = mu_d - scale * rot @ mu_s
    pred = (scale * (rot @ a.T)).T + mu_d
    resid = dst - pred
    return rot, t, scale, resid


def yaw_of(rot):
    return math.degrees(math.atan2(rot[1, 0], rot[0, 0]))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=str(ROOT))
    ap.add_argument("--datasets", nargs="*", default=None,
                    help="dataset subdirectories to use (default: all with a manifest)")
    ap.add_argument("--board", default="live/board.json")
    ap.add_argument("--calibration", default=".private/phase11d/calibration.json")
    ap.add_argument("--tip-pixel", nargs=2, type=float, default=(320.0, 250.0))
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()

    root = Path(args.root)
    calib = json.loads((root / args.calibration).read_text())
    board = Board(calib)
    spec = json.loads((root / args.board).read_text())
    board_frame = frame_at(float(spec.get("x_mm", 0.0)), float(spec.get("y_mm", 0.0)),
                           float(spec.get("z_mm", 0.0)), float(spec.get("yaw_deg", 0.0)))
    board_inv = np.linalg.inv(board_frame)

    print(f"board.json     x {spec.get('x_mm')} y {spec.get('y_mm')} "
          f"z {spec.get('z_mm')} yaw {spec.get('yaw_deg')}")
    print(f"tip pixel      {args.tip_pixel[0]:.0f}, {args.tip_pixel[1]:.0f}\n")

    base = root / ".private/real_piece_views"
    dirs = sorted(p.parent for p in base.glob("*/manifest.json"))
    if args.datasets:
        keep = set(args.datasets)
        dirs = [d for d in dirs if d.name in keep]

    rows, seen, skipped = [], set(), {"no_image": 0, "no_pnp": 0, "no_joints": 0, "dupe": 0}

    for d in dirs:
        try:
            manifest = json.loads((d / "manifest.json").read_text())
        except Exception as exc:
            print(f"  ! {d.name}: unreadable manifest ({exc})")
            continue
        label = manifest.get("label") or d.name
        for placement in manifest.get("placements", []):
            piece = placement.get("piece")
            square = placement.get("square")
            for view in placement.get("views", []):
                sha = view.get("sha256")
                if sha and sha in seen:
                    skipped["dupe"] += 1
                    continue
                if sha:
                    seen.add(sha)
                name = f"{piece}_{square}_{label}_{view.get('view')}.jpg"
                img_path = d / name
                if not img_path.exists():
                    cands = sorted(d.glob(f"{piece}_{square}_{label}_{view.get('view')}*"))
                    if not cands:
                        skipped["no_image"] += 1
                        continue
                    img_path = cands[0]

                meta = view.get("capture_meta") or {}
                joints = meta.get("state_joints_deg") or meta.get("joints_deg")
                if not joints:
                    skipped["no_joints"] += 1
                    continue

                raw = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
                if raw is None:
                    skipped["no_image"] += 1
                    continue
                sol, dets = board.solve(board.rectify(raw))
                if sol is None:
                    skipped["no_pnp"] += 1
                    continue

                ordered = [float(joints[k]) for k in
                           ("shoulder_pan", "shoulder_lift", "elbow_flex",
                            "wrist_flex", "wrist_roll", "gripper")]
                cam_base = board.camera_from_joints(ordered)
                cam_fk = (board_inv @ np.array([*cam_base[:3, 3], 1.0]))[:3] * 1000.0
                cam_pnp = np.asarray(sol["camera_in_board_mm"], float)

                rows.append({
                    "dataset": d.name, "piece": piece, "square": square,
                    "view": view.get("view"), "file": str(img_path.relative_to(root)),
                    "markers": len(dets), "rms_px": round(float(sol["rms_px"]), 2),
                    "cam_pnp": cam_pnp.round(1).tolist(),
                    "cam_fk": cam_fk.round(1).tolist(),
                    "err": (cam_fk - cam_pnp).round(1).tolist(),
                    "err_norm": round(float(np.linalg.norm(cam_fk - cam_pnp)), 1),
                    "shoulder_lift": ordered[1], "wrist_roll": ordered[4],
                    "joints_ordered": ordered,
                    "dist_mm": round(float(sol["range_mm"]), 1),
                })

    print(f"views used {len(rows)}   skipped {skipped}\n")
    if len(rows) < 6:
        print("not enough views to fit - stopping")
        return 1

    pnp = np.array([r["cam_pnp"] for r in rows], float)
    fk = np.array([r["cam_fk"] for r in rows], float)
    err = fk - pnp

    print("BEFORE any frame correction")
    print(f"  |err| mean {np.linalg.norm(err, axis=1).mean():6.2f} mm   "
          f"median {np.median(np.linalg.norm(err, axis=1)):6.2f}   "
          f"max {np.linalg.norm(err, axis=1).max():6.2f}")
    print(f"  err mean vector  {err.mean(0).round(1)}  (ddx, ddy, ddz board mm)\n")

    for allow_scale in (False, True):
        rot, t, scale, resid = rigid_fit(pnp, fk, allow_scale=allow_scale)
        rn = np.linalg.norm(resid, axis=1)
        rn_xy = np.linalg.norm(resid[:, :2], axis=1)
        tag = "rigid + scale" if allow_scale else "rigid (yaw + t)"
        print(f"FIT {tag}")
        print(f"  yaw       {yaw_of(rot):+7.2f} deg")
        print(f"  scale     {scale:7.4f}" if allow_scale else "  scale     1.0000")
        print(f"  t         {t.round(1)} mm")
        print(f"  residual  mean {rn.mean():5.2f} mm   xy-rms {np.sqrt((rn_xy**2).mean()):5.2f}"
              f"   max {rn.max():5.2f}")
        print()

    # Per-dataset yaw, to show whether one constant explains all of them.
    print("PER-DATASET (rigid yaw + t)")
    print(f"  {'dataset':<14}{'n':>4}{'yaw':>8}{'resid':>8}{'max':>7}")
    per_ds = {}
    for name in sorted({r["dataset"] for r in rows}):
        sub = [r for r in rows if r["dataset"] == name]
        if len(sub) < 4:
            continue
        a = np.array([r["cam_pnp"] for r in sub], float)
        b = np.array([r["cam_fk"] for r in sub], float)
        rot, t, _, resid = rigid_fit(a, b)
        rn = np.linalg.norm(resid, axis=1)
        per_ds[name] = {"n": len(sub), "yaw": round(yaw_of(rot), 2),
                        "resid": round(float(rn.mean()), 2), "max": round(float(rn.max()), 2)}
        print(f"  {name:<14}{len(sub):>4}{yaw_of(rot):>8.2f}{rn.mean():>8.2f}{rn.max():>7.2f}")
    print()

    # Does the error track pose? Sag would follow shoulder_lift, a frame error would not.
    lifts = np.array([r["shoulder_lift"] for r in rows], float)
    if lifts.std() > 1e-6:
        dz = np.array([r["err"][2] for r in rows], float)
        dxy = np.linalg.norm(err[:, :2], axis=1)
        print(f"corr(shoulder_lift, err_z)  {np.corrcoef(lifts, dz)[0, 1]:+.3f}")
        print(f"corr(shoulder_lift, |err_xy|) {np.corrcoef(lifts, dxy)[0, 1]:+.3f}")

    # Secondary: the jaw tip itself, which is the quantity a grasp depends on.
    # FK tip vs the tip pixel unprojected at the FK tip height. This assumes the
    # tip sits at a fixed pixel (the mount is rigid) - stated, not verified.
    tip_px = tuple(args.tip_pixel)
    tip_rows = []
    for row in rows:
        raw = cv2.imread(str(root / row["file"]), cv2.IMREAD_COLOR)
        if raw is None:
            continue
        sol, _ = board.solve(board.rectify(raw))
        if sol is None:
            continue
        joints = row.get("joints_ordered")
        if not joints:
            continue
        flange = board.camera_from_joints(joints) @ np.linalg.inv(board.camera_in_tool)
        tip_base = (flange @ TIP_IN_CHAIN)[:3, 3]
        tip_board = (board_inv @ np.array([*tip_base, 1.0]))[:3] * 1000.0
        img_xy = board.unproject(sol, tip_px, float(tip_board[2]))
        if not np.all(np.isfinite(img_xy)):
            continue
        tip_rows.append({"fk_xy": tip_board[:2], "img_xy": np.asarray(img_xy, float),
                         "fk_z": float(tip_board[2]), "square": row["square"]})

    if len(tip_rows) >= 6:
        t_fk = np.array([t["fk_xy"] for t in tip_rows], float)
        t_img = np.array([t["img_xy"] for t in tip_rows], float)
        t_err = t_fk - t_img
        tn = np.linalg.norm(t_err, axis=1)
        print(f"\nTIP (FK tip vs tip-pixel unprojection, {len(tip_rows)} frames)")
        print(f"  |err| mean {tn.mean():5.2f} mm   median {np.median(tn):5.2f}   max {tn.max():6.2f}")
        print(f"  err mean {t_err.mean(0).round(1)} mm")
        r2, t2, _, resid2 = rigid_fit(np.column_stack([t_img, np.zeros(len(t_img))]),
                                      np.column_stack([t_fk, np.zeros(len(t_fk))]))
        rn2 = np.linalg.norm(resid2[:, :2], axis=1)
        print(f"  best rigid yaw for the tip: {yaw_of(r2):+.2f} deg  "
              f"t {t2[:2].round(1)}  resid {rn2.mean():.2f} mm")
    elif tip_rows:
        print(f"\ntip check ran on only {len(tip_rows)} frames - not enough to fit")

    if args.json_out:
        Path(args.json_out).write_text(json.dumps(
            {"board": spec, "views": rows, "per_dataset": per_ds}, indent=1))
        print(f"\nwrote {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
