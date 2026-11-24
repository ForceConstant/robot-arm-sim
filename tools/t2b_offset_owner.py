#!/usr/bin/env python3
"""T2b - is the ~19 mm offset a board-origin error or a hand-eye error?

T2 found the FK-vs-PnP camera error is a nearly constant translation with ~1 mm
scatter inside a dataset, not the ~22 deg yaw that was predicted. A constant
offset in board coordinates has two plausible owners, and they are fixed in
different files:

  A) board.json origin is wrong by Delta         (one JSON edit, reversible)
  B) camera_in_tool translation is wrong by delta (re-derive hand-eye)

Both are linear in their 3 unknowns, so each is a plain least-squares fit over
every view. Whichever leaves the smaller residual owns the bug.

Read-only: reads the manifest data, fits, prints. No sockets, no motion.
"""
from __future__ import annotations

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

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
         "wrist_roll", "gripper")


def frame_at(x_mm, y_mm, z_mm, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    m = np.eye(4)
    m[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = [x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0]
    return m


def collect(board, base_inv, dirs):
    rows, seen = [], set()
    for d in dirs:
        manifest = json.loads((d / "manifest.json").read_text())
        label = manifest.get("label") or d.name
        for placement in manifest.get("placements", []):
            for view in placement.get("views", []):
                sha = view.get("sha256")
                if sha and sha in seen:
                    continue
                if sha:
                    seen.add(sha)
                meta = view.get("capture_meta") or {}
                joints = meta.get("state_joints_deg") or meta.get("joints_deg")
                if not joints:
                    continue
                name = f"{placement.get('piece')}_{placement.get('square')}_{label}_{view.get('view')}.jpg"
                path = d / name
                if not path.exists():
                    cands = sorted(d.glob(f"{placement.get('piece')}_{placement.get('square')}_{label}_{view.get('view')}*"))
                    if not cands:
                        continue
                    path = cands[0]
                raw = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if raw is None:
                    continue
                sol, dets = board.solve(board.rectify(raw))
                if sol is None:
                    continue
                ordered = [float(joints[n]) for n in NAMES]
                cam = board.camera_from_joints(ordered)
                rows.append({"dataset": d.name, "ordered": ordered,
                             "p": cam[:3, 3].copy(), "R": cam[:3, :3].copy(),
                             "pnp": np.asarray(sol["camera_in_board_mm"], float),
                             "rms_px": float(sol["rms_px"]), "markers": len(dets),
                             "drift": float(meta.get("joint_drift_deg") or 0.0)})
    return rows


def main():
    root = ROOT
    calib = json.loads((root / ".private/phase11d/calibration.json").read_text())
    board = Board(calib)
    spec = json.loads((root / "live/board.json").read_text())
    board_frame = frame_at(spec.get("x_mm", 0), spec.get("y_mm", 0),
                           spec.get("z_mm", 0), spec.get("yaw_deg", 0))
    b_inv = np.linalg.inv(board_frame)
    b_rot = b_inv[:3, :3]
    r_ct = np.asarray(calib["camera_in_tool"], float)[:3, :3]

    dirs = sorted(p.parent for p in (root / ".private/real_piece_views").glob("*/manifest.json"))
    rows_all = collect(board, b_inv, dirs)
    print(f"views (deduped) {len(rows_all)}")
    for rms_cap in (5.0, 3.0, 2.0, 1.5):
        rows = [r for r in rows_all if r["rms_px"] <= rms_cap and r["markers"] >= 3]
        if len(rows) < 20:
            print(f"  rms<={rms_cap}: only {len(rows)} views - stop")
            break
        _fit(rows, b_inv, b_rot, r_ct, tag=f"rms<={rms_cap}, markers>=3, n={len(rows)}")


def _fit(rows, b_inv, b_rot, r_ct, tag=""):
    print(f"\n===== {tag} =====")
    pred0 = np.array([b_inv[:3, :3] @ r["p"] + b_inv[:3, 3] for r in rows]) * 1000.0
    pnp = np.array([r["pnp"] for r in rows])
    e0 = pred0 - pnp
    print(f"baseline |err| mean {np.linalg.norm(e0, axis=1).mean():.2f} mm  "
          f"mean vec {e0.mean(0).round(1)}  sd {e0.std(0).round(1)}")

    # --- A: board origin offset Delta (base frame metres)
    # coords = b_inv @ (p - Delta)  =>  residual = (coords0 - pnp) - b_rot @ Delta
    # solve b_rot @ Delta = mean(coords0 - pnp)
    delta_a = np.linalg.solve(b_rot, e0.mean(0) / 1000.0)
    res_a = e0 - (np.tile(b_rot @ delta_a * 1000.0, (len(rows), 1)))
    # refine with all rows (identical to mean solve, kept explicit)
    na = np.linalg.norm(res_a, axis=1)

    # --- B: hand-eye translation delta (tool frame metres)
    # p' = p + R_flange @ delta ; residual = (coords0 - pnp) + b_rot @ R_flange @ delta
    a_mat, b_vec = [], []
    for r, e in zip(rows, e0):
        r_flange = r["R"] @ r_ct.T
        a_mat.append(-(b_rot @ r_flange))
        b_vec.append(e / 1000.0)
    a_mat = np.vstack(a_mat)
    b_vec = np.concatenate(b_vec)
    delta_b, *_ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
    pred_b, pred_a = [], []
    for r in rows:
        r_flange = r["R"] @ r_ct.T
        pred_b.append((b_inv[:3, :3] @ (r["p"] + r_flange @ delta_b) + b_inv[:3, 3]) * 1000.0)
        pred_a.append((b_inv[:3, :3] @ (r["p"] - delta_a) + b_inv[:3, 3]) * 1000.0)
    nb = np.linalg.norm(np.array(pred_b) - pnp, axis=1)
    nb_a = np.linalg.norm(np.array(pred_a) - pnp, axis=1)

    print("\nMODEL A - board.json origin offset (base frame)")
    print(f"  Delta      {(delta_a * 1000.0).round(1)} mm")
    print(f"  residual   mean {na.mean():5.2f}  rms {np.sqrt((na**2).mean()):5.2f}  max {na.max():5.2f}")
    print("\nMODEL B - camera_in_tool translation offset (tool frame)")
    print(f"  delta      {(delta_b * 1000.0).round(1)} mm")
    print(f"  residual   mean {nb.mean():5.2f}  rms {np.sqrt((nb**2).mean()):5.2f}  max {nb.max():5.2f}")

    # Robust refit: datasets disagree, so down-weight the views that do not fit.
    # A constant that only holds for most views is still a usable constant; the
    # point is to see how tight it gets, and which datasets refuse it.
    w = np.ones(len(rows))
    for _ in range(6):
        sw = np.sqrt(w)
        am = np.vstack([-(b_rot @ (r["R"] @ r_ct.T)) * s for r, s in zip(rows, sw)])
        bv = (e0 * sw[:, None] / 1000.0).reshape(-1)
        delta_r, *_ = np.linalg.lstsq(am, bv, rcond=None)
        pred_r = np.array([(b_inv[:3, :3] @ (r["p"] + (r["R"] @ r_ct.T) @ delta_r)
                            + b_inv[:3, 3]) * 1000.0 for r in rows])
        nr = np.linalg.norm(pred_r - pnp, axis=1)
        w = 1.0 / (1.0 + (nr / 3.0) ** 2)
    keep = nr < 6.0
    print(f"\nROBUST (weights, 6 iters): kept {keep.sum()}/{len(rows)} views under 6 mm")
    print(f"  delta      {(delta_r * 1000.0).round(1)} mm (tool frame)")
    print(f"  residual   mean {nr.mean():5.2f}  median {np.median(nr):5.2f}  max {nr.max():5.2f}")

    print("\nper dataset (mean residual, mm):  ds            A      B")
    for name in sorted({r["dataset"] for r in rows}):
        idx = [i for i, r in enumerate(rows) if r["dataset"] == name]
        if len(idx) < 3:
            continue
        print(f"  {name:<14} {na[idx].mean():6.2f} {nb[idx].mean():6.2f}")

    # What would each fix imply for a board target? Displacement of a commanded square.
    print("\nsanity: a commanded board point (0,0,0) moves by")
    print(f"  under A  {(b_rot @ delta_a * 1000.0).round(1)} mm in board coords")
    print(f"  under B  depends on pose (tool rotation), not a constant")
    json.dump({"tag": tag,
               "n": len(rows),
               "model_a_delta_mm": (delta_a * 1000).round(2).tolist(),
               "model_a_resid_mean": round(float(na.mean()), 2),
               "model_b_delta_mm": (delta_b * 1000).round(2).tolist(),
               "model_b_resid_mean": round(float(nb.mean()), 2)},
              open(ROOT / ".private/t2b_owner.json", "w"), indent=1)
    print("wrote .private/t2b_owner.json")


if __name__ == "__main__":
    main()
