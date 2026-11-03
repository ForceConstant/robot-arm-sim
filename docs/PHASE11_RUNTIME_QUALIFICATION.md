# Phase 11 — BC-250 training and policy-execution qualification

Measured 2026-11-03 on Brian's Bazzite BC-250. Scope is **platform/runtime
compatibility**, not trained-policy quality, not physical readiness and not a
whole-board controller result. No robot, servo, camera or simulator command was
issued by any probe in this phase; the simulator was only read over LAN.

## Outcome

**Selected runtime: pinned LeRobot ACT at full default size, FP32, CPU fallback
on the BC-250 for bounded-qualification work.** The GPU routes are rejected for
this pinned stack, and the generic 450M SmolVLA artifact is rejected on the
declared action-replenishment deadline.

| Route | Result |
| --- | --- |
| CPU, full ACT (51,565,446 parameters) | **qualified** for training/reload mechanics and scheduled inference latency |
| Tinygrad/WebGPU ACT torch bridge | **rejected** — fails inside the forward graph, after full-model transfer |
| Patched ROCm/PyTorch | **not attempted** — requires kernel, firmware and driver changes; only available as a separate, explicitly authorized host-change decision |
| SmolVLA `lerobot_base` 450M action model | loads and runs, **rejected on latency**: 5.40–5.49 s per chunk against a 1.0 s deadline |
| MolmoAct2 5.44B | **rejected on memory** before allocation: 21.77 GB F32 artifact, ≥10.88 GB even at hypothetical FP16, versus 8.02 GB total CPU memory |

Nothing in this document claims a usable chess policy, a physical gate, or
sustained GPU operation. The previously reported small MLP and 35B VLM results
remain separate receipts and do not qualify ACT.

## Pinned implementation

- LeRobot commit `2774d9bddcbbda50e697e162e89e7eaada8d7105`
  (archive SHA-256 `794aa3761cc44e5ac3071241e274df34d582cac1a4b481f3adce753b079c8fbc`).
- `src/lerobot/policies/act/modeling_act.py`, SHA-256
  `8d622935319af7d03f1efd95d51e726af2921010f7e4f5ec3742f62280493111`.
  Only the neural classes/functions are executed by the harness (AST-selected),
  so the policy registry, dataset stack, drivers and publication code are not
  imported. Arithmetic is untouched.
- Architecture: ResNet-18 backbone, `dim_model` 512, 8 heads, feedforward 3200,
  4 encoder + 1 decoder layer, VAE enabled (`latent_dim` 32), chunk size 30,
  10 executed action steps, 224×224 single wrist RGB, 6 joint state, explicit
  source/destination goal.
- Precision: FP32. Device: CPU, 4 threads. Torch 2.8.0+cpu, torchvision 0.23.0+cpu.

## Data path

`tools/export_act_pilot.py` re-runs the existing fail-closed media/provenance
audit, then exports a deliberately small historical fixture:

- 48 sampled observations from 3 simulator episodes (32 train / 16 test), whole
  episodes and layouts kept disjoint; **1 held-out episode**.
- Inputs: wrist RGB at 224×224, six observed joint radians, and goal coordinates
  derived only from the instructed source/destination squares in board metres
  (z = 0 surface). No FEN, piece pose, task phase or success label enters the
  observation.
- Targets: 30 future absolute joint targets at 10 Hz (every third 30 Hz frame),
  with an explicit padding mask after episode end.
- Dataset SHA-256 `7c82202bd14d71374610482fa9f4199dda8d160977354141bac39f139068c96f`.

This is an ingestion/mechanism fixture, not a training corpus and not the
current 90/30 product layout.

## Measured gates

### Initialized graph and optimizer

- Full initialized forward/backward/AdamW step passed at batch 1 and batch 2
  with synthetic targets (graph-shape subgate only; those targets say nothing
  about quality).
- 20 AdamW updates (`lr=1e-5`, `weight_decay=1e-4`) on the audited data, ~0.53 s
  per batch-2 step, finite loss and finite gradients for every trainable
  parameter.
- Checkpoint (model + optimizer + RNG) reload reproduces identical predictions.
  The resumed AdamW update is **bit-identical** to an uninterrupted branch,
  including the stochastic VAE/dropout sample.
- Independent-process inference-only reload of the exported artifact reproduces
  every held-out prediction exactly. Artifact 206,487,563 bytes, SHA-256
  `134231589b64dbdfae65e153e3638c9541cafe5deb79d7411dd61a33e3c83107`.
- Peak process RSS 1.80 GB. Guard peak 52 °C, minimum CPU availability 4.30 GB,
  maximum swap growth 156 MB during this bounded run.

### Held-out quality (not a selection result)

Held-out MSE 1.158 rad² versus 0.163 rad² for holding position, on a single
held-out episode after 20 steps from random initialization. This is exactly what
a 20-step pilot should look like and is **not** evidence for or against ACT
quality; Phase 12 owns training, held-out scoring and controller selection.

### Scheduled inference under expected load

The soak drives the same preprocess → inference → decode path against live
simulator JSON status and wrist JPEG retrieval over LAN, on the declared
schedule (1.5 s replan period, 1.0 s deadline, 30-action chunks = 3 s of motion
per chunk), with a **zero swap-growth** gate.

- Duration **30.0 min**, **1201** scheduled cycles at a 1.5 s replan
  period, **0** deadline misses and **0** guard aborts.
- End-to-end per cycle (LAN status+JPEG retrieval, preprocess, inference, decode):
  **p50 79 ms**, **p95 101 ms**, max **166 ms**
  against the pre-declared 1.0 s deadline.
- Guard: peak GPU edge **50 °C**, minimum CPU availability
  **4.80 GiB**, maximum swap growth **0.0 MB**,
  peak process RSS **700.40 GiB**. The zero-swap-growth gate passed
  for every sample.
- **Capture-age admission: 1201/1201 cycles
  returned `hold_replan:unverified_capture_age`.** Unique simulator frame IDs
  observed: **1**. The endpoint serves a cached image
  and publishes no capture timestamp, so this run qualifies *transport+inference
  timing*, not a controllable observation pipeline. Phase 15 owns that contract.

### Command boundary

A fresh HTTP 200 for a cached JPEG is not evidence that the image was newly
captured. Every soak cycle is therefore also scored through
`motion_verdict(..., capture_verified=False)` and returns
`hold_replan:unverified_capture_age`; the simulator exposes no capture timestamp
to certify. This is why the measured latency is reported as a **runtime timing
gate only** and cannot be promoted to a motion-safety claim. Phase 15 owns
capture timestamps, sequence IDs, expiry and the Pi-local watchdog.

## Rejected routes

- **Tinygrad/WebGPU ACT port.** All 51.6M parameters transferred to the
  `tiny` device, then the first forward pass failed in
  `ACTSinusoidalPositionEmbedding2d` at `cumsum(1, dtype=torch.float32)`, which
  the experimental torch bridge does not implement. Rejected as-is; the earlier
  synthetic 1.06M-parameter Tinygrad pass does not qualify this network.
- **SmolVLA (`lerobot/smolvla_base`, revision
  `5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6`).** 450,046,176 parameters,
  906,712,520-byte artifact, strict full-state reload, 512×512, chunk 50, 10
  diffusion steps. Four forwards produced bit-identical actions (fixed noise),
  so the graph is deterministic and functional — but 5.404–5.492 s per chunk
  on CPU, about 5.4× the deadline. Peak RSS 2.50 GB. Quality was deliberately
  not evaluated; this upstream generic artifact is not a chess controller.
- **MolmoAct2 (`allenai/MolmoAct2-SO100_101`, revision
  `152569fe57914d97be91055800035f54e250d009`).** 5,442,196,272 F32 parameters =
  21.77 GB of weights. Even a hypothetical FP16 copy (10.88 GB) exceeds the
  board's 8.02 GB total CPU memory, before activations or the action expert
  runtime. Rejected on a nonallocating preflight; no allocation was attempted.
  This is an artifact/CPU-runtime rejection, not a hardware-wide impossibility
  claim: patched ROCm with more of the shared 16 GB pool is a different question.

## Environment and host state

- Kernel `7.2.11-ogc4.1.fc44.x86_64`, Mesa RADV/Vulkan as previously recorded.
- No `amdgpu.bc250_*` module parameters are present; no HIP libraries are
  installed; `/dev/dri/renderD128` and `/dev/kfd` exist.
- Everything ran from user-space virtualenvs under
  `~/.local/share/robot-arm-bc250-investigation/`. No packages, drivers,
  firmware, kernel arguments, services, fans or power settings were changed, and
  the running desktop/gaming session stayed up.
- Two earlier soak attempts are retained: the first stopped at a strict
  zero-swap-growth gate while unrelated model staging was in progress, and the
  second was intentionally interrupted to separate cached-image retrieval
  latency from verified capture-age admission.

## Reproduction

```bash
# on the BC-250, in the user-owned stage
python3 prepare_phase11.py --vla            # pinned public sources/artifacts only
python3 bc250_policy_preflight.py --out large-policy-preflight
python3 bc250_guard.py --out train-guard --seconds 900 -- \
  ../phase11-venv/bin/python -u bc250_act.py --mode train \
  --source modeling_act.py --dataset dataset.npz --out train --steps 20
python3 bc250_guard.py --out reload-guard --seconds 180 -- \
  ../phase11-venv/bin/python -u bc250_act.py --mode reload \
  --source modeling_act.py --dataset dataset.npz \
  --artifact train/inference.pt --out reload
python3 bc250_guard.py --out soak-guard --seconds 1900 --swap-growth-mib 0 -- \
  ../phase11-venv/bin/python -u bc250_act.py --mode soak \
  --source modeling_act.py --dataset dataset.npz \
  --artifact train/inference.pt --out soak --seconds 1800
```

Local regression: `113 passed, 5 skipped` (clean worktree at HEAD plus the Phase 11 files) and `117 passed`
(custom tool + printed pieces) — the custom/printed invocation needs
`ROBOT_GRIPPER_ASSETS=.private/gripper-assets-v3` and
`ROBOT_PIECE_ASSETS=.private/piece-assets-v5`.

## Limits carried forward

- The CPU ACT path is a **bounded qualification/development** fallback. No
  long-duration training throughput, multi-hour soak, or 10 Hz GPU inference is
  claimed. Training-time estimates require measured throughput that this phase
  deliberately did not extrapolate.
- Latency covers LAN status/JPEG retrieval, preprocessing, inference and decode.
  It excludes model load, physical camera capture, Pi transport and servo
  execution.
- The historical three-episode fixture is not a Phase 12 dataset and does not
  cover the current layout, crowding, all piece types, captures or trays.
- Adopting a learned controller still requires the Phase 10 and Phase 14 gates
  plus a measured benefit over scripted control. A justified rejection keeps the
  scripted controller.
