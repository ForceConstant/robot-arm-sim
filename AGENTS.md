# Robot Arm Chess Lab

## Current scope

Brian's explicit priority (2026-10-27): **set up the environment first; do not
optimize pick-and-place or full-game reliability yet.** The initial environment
is a two-follower-arm MuJoCo scene, browser controls/views and a Dockhand stack.
These simulated arms do not imply buying a second physical follower.

## Invariants

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

- Runtime dependencies are pinned in `requirements.lock`.
- `MUJOCO_GL=egl .venv/bin/python -m pytest -q tests`
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
