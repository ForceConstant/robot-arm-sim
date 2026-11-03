# Robot Arm Chess Lab

## Current scope

Current delivery authority: `docs/ROADMAP.md` (Brian, 2026-11-02). Completed
Phases 0–8 are bounded foundations; Phase 9 is next. Develop and qualify core
software in simulation before real autonomous integration. Product target is
one follower with wrist-only vision, Pi 4 direct camera/servo I/O and watchdog,
and BC-250 central processing/model inference/training where qualified. Existing
two-arm simulation remains a debug/regression fixture, not proof of single-arm
reach or permission to buy another follower. Follow the phase requested by the
user; the roadmap is not authorization to implement all future phases at once.

## Invariants

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
