# FK/hand-eye recovery — steps 1–2: print audit + datum inventory (2026-11-26)

Read-only. No arm motion, no gripper, no Docker, no writes to `.private/`, `rig_state/`,
`live/` or tracked files. All project execution on the BC-250 (`brian@192.168.1.171`),
`/home/brian/robot-arm-sim`, HEAD `32b241e` (local clone byte-identical: md5 of
`fiducials.py`, `board_target.py`, `charuco_face.py`, `config.py`, `wrist_calibration.py`).

## 1. Print audit

Capture: 3 passive frames, `GET http://192.168.1.167:8100/capture` (landscape 640x480, no
rotation needed; frame age 0.48 s). Joints at capture: pan −12.40, lift −12.35, elbow 5.85,
flex 97.98, roll 0.18 (near home).

| item | value |
|---|---|
| markers / corners (raw frame, `Board.solve`) | 6 (ids 18,19,22,27,30,34) / 24 |
| PnP rms / max | 2.75 px / 11.39 px |
| per-marker px | 18:1.15 19:1.51 22:1.41 27:1.48 30:**3.78** 34:1.61 |
| same view, 5 markers (id 30 fails after rectify) | rms **1.34 px**, max 2.24 px |
| camera range / in-board position | 216.7 mm / (+29.2, −66.5, 204.2) mm |
| rectified @4 px/mm (1024²) | 5 markers (18,19,22,27,34), 20 corners |
| fit vs `charuco_face.corners_mm` | dx +0.06, dy −0.03 mm, scale 1.00055, rot −0.19° |
| corner residual | rms **0.379 mm (1.51 px)**, max 0.708 mm (2.83 px) |
| centroid residual | rms 0.21 mm, max 0.28 mm |
| square pitch (col / rank) | 28.015 / 28.030 mm (mean 28.023, +0.08 %) |
| field extent (8 × pitch) | **224.18 mm** (+0.08 %) |
| skew (col vs rank axis) | 0.18° |
| reference `board_charuco.png` 1952², 8 px/mm, 10 mm border | 32 markers / 128 corners, vs nominal **0.01 mm rms**, pitch 28.000, extent 224.000 (method validation) |
| capture vs reference print (5 common) | dx 0.12, dy −0.10 mm, scale 1.0006, rot −0.19°, rms 0.371 mm, max 0.698 mm |
| apparent marker black edge | 78.1–78.7 px = 19.5–19.7 mm vs 20.0 nominal (−1.8 %) |

Per-marker centroid offset vs nominal (mm): 18 (−0.14, −0.02), 19 (+0.22, −0.17),
22 (−0.14, −0.02), 27 (+0.29, +0.21), 34 (+0.44, +0.13). No ramp in either axis.

Marker edge: the −1.8 % is consistent with a symmetric 0.19 mm/side corner bias, which
cancels in the lattice (pitch reads nominal) but not in the edge; only a caliper resolves
print size. Cell-polarity texture test discarded — markers fill the square centres, so the
sampled patches measured marker ink (dark 129 / light 164, populations overlapping);
it carries no information.

**Verdict:** in the observed region (cols b–e, ranks 3–8; 21/64 cells fully observed,
10 partial, 33 not seen) the printed face matches the generated face within **0.38 mm rms /
0.71 mm max**, pitch 28.02 mm, extent 224.18 mm, rotation 0.19°, translation ≤0.13 mm, skew
0.18°. No print defect. Absolute scale (invisible to marker-based measurement — it scales
the 20 mm marker and the pitch together) and the unobserved two-thirds need the checklist.
The 2.75 px PnP is one bad tag (id 30: 3.78 px mean, border distance 154 px, not clipped —
jaw/shadow), not a print error: the same view is 1.34 px without it.

## 2. Datum inventory

| datum | live value | other value | inconsistency |
|---|---|---|---|
| playing surface z | 431.35 mm (`live_config.board_z_m`, `marker_z_m`; block4c fit 431.35) | `live/board.json` + `rig_state/board.json` **432.65** (+1.30) | 4 values for one surface: 431.35 / 432.65 / 433.2 (`BoardConfig` defaults: 430 + mat 0.0 + 3.2) / 433.7 (`hardware/current_layout`: mat 0.5 + 3.2) — 2.35 mm spread |
| marker z | 431.35 = board z | `BoardConfig.marker_above_table` returns `mat` (0.5 mm) for `marker_plane="mat"`; Phase-9 note quotes a 0.5 mm marker face | coplanar is right for `face="charuco"`, but the config path and the live file disagree on whether markers are raised |
| board x / y | −27.62 / −31.63 mm (aim sweep 2026-11-25 06:43) | block4c joint solve −9.92 / −3.13 mm | Δ 17.7 / 28.5 mm; yaw 184.32 vs 185.67 = 1.35° |
| camera mount | (63.59, 11.10, −13.15) mm | block4c (67.66, −2.92, −10.89) mm | Δ 14.77 mm, rotation 23.4°, view axis 17.05°; file still says `calibration_id: handeye-2026-11-18T11-block4c` (false provenance). Came from `calibration.json.bak-restore-20261125T071815` (identical mount) |
| MANIFEST integrity | all four sha256 match (`9928f2`, `8c8303`, `021bc9`, `70f6c5`), sizes match | — | pinned state is self-consistent; the *label* on the mount is wrong |
| wrist_roll limits | chain joint_index 4 (`so101:gripper`) −157.2°…+162.8° | bridge `/state` −89.98…+90.07; `wrist_calibration.json` ±90 | range 1.8× too wide in the chain |
| chain base y | −217 mm | `current_layout` [0, −212, 462.4] | 5 mm; z 462.4 agrees |
| 5y jaw pack | `.private/gripper-assets-5y`, `source_microversion 4dca0986f24304c301118aa4` | `current_layout.tool.revision` same | identity agrees; but `model.xml` (19 Nov 08:10) is newer than `profile.json`/`model.xml.bak-2026-11-19-prebaset` (18 Nov 15:51) and has no MANIFEST entry |

## 3. Physical checklist for the owner

Expected value and the tolerance that matters; one sitting.

1. Slab thickness, 4 points (calipers) — **3.20 mm ±0.10**.
2. Mat thickness — **0.50 mm ±0.05**.
3. Board top above table, centre + 4 corners — **3.70 mm ±0.20**; corner spread **≤0.30 mm**.
4. Square pitch: 8 squares along x and along y, ink line to ink line — **28.00 ±0.15 mm**; x vs y within **0.10 mm**.
5. Field extent, outer frame line edge to edge — **224.0 ±0.30 mm**; full face **244.0 ±0.40 mm**.
6. Field diagonal (a1 corner to h8 corner) — **316.8 ±0.30 mm**.
7. All four border widths, frame line to slab edge — **10.0 mm each ±0.30**. If two opposite borders differ by ≈5 mm, `FIELD_OFFSET_MM = (0.18, −5.1)` is physical, not a texture artefact.
8. One marker's printed black edge, edge to edge (calipers) — **20.00 ±0.15 mm**. Settles the 19.6 mm image reading as detector bias or real print scale.
9. Board edge to base-frame front face — **90 ±1 mm**; board centre to the arm's Y axis — **0 ±1 mm**.
10. Board yaw against the base front face — **0.0° ±0.30°** (current datum implies 4.32° off the 180° flip; check whether the board is really rotated).
11. Lens front face to nearest jaw face along the view axis — record to 0.5 mm; must reconcile with the mount: camera centre sits **63.6 mm** from the tool-frame origin and the jaw tip at **83.0 mm** (`BoardConfig.jaw_tip_mm`).
12. Camera centre to board plane at the home pose — **216.7 mm** by tape, **±2 mm** of the audit value.
13. Printed field rotation in a rectified view at `wrist_roll = 0` — **0.0° ±0.30°** (measured −0.19°).

## Commands run

```
ssh brian@192.168.1.171 'cd /home/brian/robot-arm-sim && git rev-parse --short HEAD; git status --porcelain'
curl -s http://192.168.1.167:8100/state
curl -s -o /tmp/cap.json -w '%{http_code} %{size_download}' http://192.168.1.167:8100/capture
md5sum chess_sim/{fiducials,board_target,charuco_face,config,wrist_calibration}.py   # local vs remote
ssh brian@192.168.1.171 'sha256sum live/{board,live_config,wrist_calibration}.json .private/phase11d/calibration.json'
~/.local/bin/robot-arm-sim python /tmp/fk_step12_audit.py           # capture + Board.solve + rectify
~/.local/bin/robot-arm-sim python /tmp/fk_step12_fixup.py           # corrected reference comparison
~/.local/bin/robot-arm-sim python /tmp/fk_step12_closing.py         # raw-frame clipping, cell test
~/.local/bin/robot-arm-sim python /tmp/fk_step12_mount_polarity.py  # mount diff + observed-cell test
~/.local/bin/robot-arm-sim python /tmp/fk_step12_cells.py           # threshold recheck (discarded metric)
```
Artifacts: `/tmp/fk_step12/{audit.json,audit_fixed.json,closing.json,mount_polarity.json,rectified_4ppmm*.png,valid_mask.png}`.

## Physical measurements and root cause (Brian, 2026-11-26)

Brian measured the checklist and found the real fault while doing it:

- **Root cause: the wrist-to-jaw connection was loose.** Both the camera and
  the jaw moved together relative to the wrist. The mount had been checked and
  was tight; the joint below it was not. It has been fixed.
- Slab 3.2 mm, **no mat** (so the playing surface is 3.2 mm above the table,
  not the 3.7 mm used by the Phase 9/board config).
- Items 4-8 (pitch, extent, diagonal, borders, marker edge) confirmed.
- Item 11 (lens front to nearest jaw face along the view axis): **97 mm ±0.5**
  — different from the 63.6 mm camera centre / 83.0 mm tip implicitly assumed
  by the block4c mount and the current calibration.
- Item 12 (camera centre to board plane at home): confirmed.
- Items 9 (board edge to base frame) and 10 (board yaw) are still unknown.
- Item 13: at wrist_roll servo 0.0 the image is slightly rotated, so a camera
  yaw offset control is being added to the wrist calibration page.

Consequence: every prior mount/board fit in this investigation was made with a
moving camera, which explains the pose-dependent scatter that no mount, board,
joint-offset or chain solve could explain. A full recalibration (wide-volume
capture) is required after the yaw control lands; do not reuse any pre-fix
calibration receipt as truth.
