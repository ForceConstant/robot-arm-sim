# Robot Arm Chess Lab

## Current scope

Current delivery authority: `docs/ROADMAP.md` (Brian, 2026-11-03). Completed
Phases 0–8 are bounded foundations; **Phase 9 is next**. Phase 11 is in
progress: a CPU ACT execution-runtime receipt exists, but local training paths
for both the control and vision models are still required. Develop and qualify core
software in simulation before real autonomous integration. Product target is
one follower with wrist-only vision, Pi 4 direct camera/servo I/O and watchdog,
and BC-250 central processing/model inference/training where qualified. Existing
two-arm simulation remains a debug/regression fixture, not proof of single-arm
reach or permission to buy another follower. Follow the phase requested by the
user; the roadmap is not authorization to implement all future phases at once.

## Invariants

- Current-mount Phase 8 rerun: `docs/PHASE8_UPDATED_OFFSETS.md` and
  `reports/phase8_updated_*.json`. The unchanged CAD jaw passes 64/64 source
  paths after retries; a span-only candidate passes 25/25 crowded cases. Keep
  geometry; qualify finer span selection before redesigning fingers. Candidate
  overrides are experiments, not live defaults. Four cardinal spans and a
  conservative 2-D clearance heuristic do not prove mechanical impossibility.
- Tray qualification must assert distinct actual target coordinates. A reset
  clears capture bookkeeping and otherwise sends every capture to slot zero.
- Phase 9 (complete 2026-11-03 for a stated envelope; 94.0% turn resolution
  accepted by Brian, self-model occlusion queued to Phase 10): wrist-only
  registration uses the printed 4-marker mat, with all board measurements
  relative to the table (surface = mat + slab = 3.7 mm, marker face = 0.5 mm,
  arm datum = 30 mm)
  (`chess_sim/fiducials.py`, mat thickness `BoardConfig.mat`), a saved
  hand-eye/lens calibration (`chess_sim/wrist_calibration.py`) and measured
  joints. Never feed simulator camera or board truth to inference; the checker
  fit stays only as a marker-free ablation. Empty references are epoch-bound and
  must be re-captured after any re-registration. Measured: registration 8/8
  within 2 mm/1°, 0 false confirmations in 200 turns, turn resolution 94.0%
  (95% target, gap named) — see `docs/PHASE9_WRIST_ONLY.md`.

- Phase 11 runtime evidence: `docs/PHASE11_RUNTIME_QUALIFICATION.md`. This is a
  sub-gate receipt: Phase 11 also requires local training of control **and**
  vision models (scope change 2026-11-03). Full ACT execution is qualified on
  BC-250 **CPU** only. Tinygrad/WebGPU ACT, the 450M SmolVLA
  action artifact and the 5.44B MolmoAct2 artifact are rejected; do not cite the
  old tiny-MLP or 35B-VLM receipts as ACT/VLA execution evidence. A fresh HTTP
  JPEG is not a verified capture timestamp, so probe latency is not a
  motion-safety claim.

- Current interactive layout: `hardware/current_layout.json` / `docs/LAYOUT_OPTIMIZATION.md`
  (one follower via `ROBOT_ARM_COUNT=1`, 90 mm board-edge-to-base-frame gap,
  30 mm riser, measured 3.2 mm slab,
  latest custom jaw, position-priority IK). The 2026-11-02 stock-tool
  `product_twin.json` is a historical Phase 8 baseline, not current runtime
  layout. Keep nominal reach separate from crowded/perturbed reliability.

- Keep upstream provenance and history. `upstream` is
  `qalby-tech/embryo_lab_chess_so101`; `origin` is `ForceConstant/robot-arm-sim`.
- Physics/rendering are owned by one worker thread. HTTP must not touch MuJoCo.
- Default to simulation only. Do not import hardware drivers, open serial ports
  or add Docker/socket/host mounts to the application.
- The expert uses oracle simulator state. Wrist-only observation selection is
  NOT a claim that camera-only perception or control works.
- Preserve real contact grasps. Do not hide failures with welds, teleportation,
  loose success thresholds or unreported resets.
- Pause resumes the active controller stack. Stop cancels it; a subsequent Start
  resets an interrupted world. Reset is an explicit scene reset.
- Do not stop other GPU consumers or unrelated containers.

## Checks / release

- Runtime dependencies are pinned in `requirements.lock`; test-only dependencies
  are pinned separately in `requirements-test.lock`.
- At the start of a session that needs project Python, run
  `scripts/ensure_venv.sh`. It checks/rebuilds `.venv` against the container's
  system Python 3.11; never point it at an interpreter under `/tmp`. Then run
  `MUJOCO_GL=egl .venv/bin/python -m pytest -q tests`.
- Exercise real browser loading, camera images, start/pause/resume/stop/reset,
  wrist-only observation contract and mobile overflow. Prefer Playwright from
  the shared Dockhand checkout when available.
- `git diff --check`; stage only intended source, never `.venv` or `.private`.
- Commit/push and publish a versioned runtime bundle, compose and checksums.
- Deploy only the authorized `robot_arm` stack using `scripts/deploy.py`;
  collect the Dockhand job result and verify the running renderer/controls.
- Update the project notes at `/home/node/obsidian/Chess-Robot-Arm*.md` when
  architecture or measured environment readiness changes. Physical gates remain
  independent of simulator-environment readiness.

## BC-250 CachyOS simulation workstation

Brian requested that future robot-arm simulation and test-case runs use the
BC-250 on CachyOS rather than Unraid. The workstation is `cachyos`
(`brian@192.168.1.171`), with the project at `/home/brian/robot-arm-sim` and
the user launcher at `~/.local/bin/robot-arm-sim`.

- Before running project simulations or tests, synchronize the current working
  tree to the BC-250 and check its Git revision/status; do not test an old clean
  clone when the active source has uncommitted changes. Exclude `.private`,
  generated datasets/runs, `dist`, virtualenv/toolchain directories and private
  CAD assets. Preserve the BC-250's own generated data; do not use destructive
  mirroring.
- Use `robot-arm-sim test` for the suite, `robot-arm-sim python <script>` for
  project tools, or `robot-arm-sim serve` for a localhost-only simulator. The
  launcher selects the user-local Python 3.11 environment and sets
  `MUJOCO_GL=egl`, `OMP_NUM_THREADS=1` and `OPENBLAS_NUM_THREADS=1`.
- EGL rendering has been verified on `AMD BC-250 (radeonsi, gfx1013)`. MuJoCo
  physics remains CPU-executed; this workstation setup does **not** mean the
  simulation/test physics or PyTorch training uses GPU compute. Do not add ROCm,
  alter kernel/GPU settings, start a persistent service, or change containers as
  part of routine test execution.
- Keep workload duration and scope explicit. A renderer smoke check is not a
  full test-suite or long-run qualification; preserve that distinction in
  reports. If a GPU fault appears, stop further GPU work and report it rather
  than trusting subsequent results without recovery.
