# Hand-eye session probes (2026-11-21)

One-off diagnostic scripts from the camera-to-wrist re-derivation attempts. They
are kept for the record, not as maintained tools: each hardcodes
`ROOT = /home/brian/robot-arm-sim`, was run on the BC-250 with
`~/.local/bin/robot-arm-sim python <script>`, and assumes the live console is up
on `127.0.0.1:8080` with the Pi bridge reachable.

Read `reports/HANDEYE_PROBLEM_REVIEW_20261121.md` first — it explains what each
measurement means and which ones are trustworthy.

| script | what it does |
|---|---|
| `p4_board_clear_check.py` | exposure settle on a high-exposure grid, then per-square Otsu blobs around the tool tip |
| `p4_square_stats.py` | rectified top-down board, per-square inner-region brightness stats |
| `p4_rectified_view.py` | saves a rectified board image with the square grid and the projected tool tip |
| `p4_cam_debug.py` | FK camera vs PnP camera position per view, in the base frame |
| `p4_scan_preflight.py` | FK every plan pose, check limits/keep-in and the transit tip height |
| `p4_jaw_test.py` | hold the pose, move only the gripper 0→100, count decoded markers |
| `p4_tone_sweep.py` | brightness/contrast grid vs decoded markers and clipping |
| `p4_handeye_capture.py` | first 13-pose capture pass (settles exposure at pose 1) |
| `p4_handeye_capture2.py` | second 13-pose pass with a fixed, measured tone and the plan jaw |
| `p4_handeye_capture3.py` | the good poses again with wrist roll +46.3° |
| `p4_handeye_eval.py` | staged solve, per-view board-pose spread, PnP-vs-FK, leave-one-out |
| `p4_final_metrics.py` | per-view PnP-vs-FK disagreement in the base frame, before/after |
| `p4_view_axis.py` | FK vs PnP optical-axis comparison (superseded by `p4_final_metrics.py`) |
| `p4_update_plan.py` | rebuilds the scan plan from the two measured capture passes |

Note: `p4_handeye_eval.py` also prints a *board-frame* PnP-vs-FK figure that is
inconsistent with the base-frame one; the base-frame numbers are the valid ones.
