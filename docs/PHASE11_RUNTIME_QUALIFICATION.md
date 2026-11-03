# Phase 11 — BC-250 training and policy-execution qualification

Measured 2026-11-03 on Brian's Bazzite BC-250. Scope is **platform/runtime
compatibility**, not trained-policy quality, not physical readiness and not a
whole-board controller result. No robot, servo, camera or simulator command was
issued by any probe in this phase; the simulator was only read over LAN.

> **Scope update 2026-11-03 (Brian): this document is a sub-gate receipt, not
> Phase 11 completion.** Phase 11 is reopened and additionally requires a
> locally runnable training path for both the control policy and the vision
> model, on our own services. See the roadmap's Phase 11 deliverables. Cloud
> training is out of scope; a route that cannot train locally is a rejected
> path, not a pass.

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
  peak process RSS **700 MiB** (0.68 GiB). The zero-swap-growth gate passed
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

## Small-GPU fit test — GTX 1060 3 GB (2026-11-03)

Run on the Unraid host in a throwaway container (image
`ghcr.io/forceconstant/pdf2epub-nvidia:latest`, `torch 2.9.1+cu126`, CUDA 12.6,
`--gpus all` via the nvidia runtime, 8 GiB host RAM cap, 4 CPUs), with the live
`robot_arm` simulator still owning its share of the card. **Synthetic tensors**
were used: this measures graph fit and throughput, not learning.

| Batch | Peak allocated | Peak reserved | Warm step | Samples/s |
| --- | --- | --- | --- | --- |
| 1 | 835.6 MiB | 922 MiB | 0.055 s | 18.2 |
| 2 | 835.7 MiB | 928 MiB | 0.059 s | 33.8 |
| 4 | 836.3 MiB | 942 MiB | 0.075 s | 53.2 |
| 8 | 914.1 MiB | 1072 MiB | 0.107 s | 74.9 |

- Full-size ACT (51,565,446 parameters, 196.7 MiB weights) fits at **batch 8
  under 1.1 GiB reserved**, on a card reporting 2.94 GiB total and 2.0 GiB free
  at start because the simulator was rendering.
- Throughput at batch 8 is **~75 samples/s**, about **20× the BC-250 CPU**
  measurement (3.8 samples/s at batch 2). A 70k-step run at batch 8 is roughly
  **2.1 hours** of optimiser time, before data loading.
- Contention is real but not fatal at these sizes; larger batches or a second
  process would need the simulator paused.

Limits: torch 2.9.1 is not the pinned 2.8.0, so this is a fit/throughput result
rather than a reproduction of the CPU receipt; synthetic inputs bypass the
dataloader and video decode, which will be the next bottleneck; no convergence,
held-out quality or checkpoint reload was tested here; and only the control model
was exercised — the vision model still needs its own fit test.

## Public-dataset training pilot on the GTX 1060 (2026-11-03)

First end-to-end training run on real data, in a throwaway GPU container on the
Unraid host (`torch 2.9.1+cu126`, nvidia runtime, 8 GiB host cap, 4 CPUs).
Dataset: `XvKuoMing/so101_chess` at revision
`969ae3a886b2038dfa349a768cbf8a9d57dde583` (LeRobot v3.0, 10 fps, 6,095
episodes, 497,565 frames, 640×480), fetched into a named volume
`phase11_data` (~280 MB: meta, one data shard, one top-camera video).

- Slice used: **25 episodes**, 16 sampled observations each → **400 samples**
  (320 train / 80 test, 5 whole held-out episodes). Top camera only; the wrist
  stream was not downloaded.
- Video decode: **one sequential pass**, 15.7 s for 12,627 frames, bucketed by
  each episode's `from_timestamp`/`to_timestamp`. Seeking per episode returns
  zero frames past the first episode in this file, so the loader must scan
  sequentially.
- Training: full ACT, FP32, batch 8, 200 AdamW steps in **25.4 s** →
  **62.6 samples/s** (loss 85.0 → 4.09, min 3.51). Checkpoint 206.5 MB.
- **Independent reload exact:** a fresh model loaded from the checkpoint
  reproduces the trained model's predictions bit-for-bit.
- **Held-out quality is not claimed:** 944.5 deg² versus 465.6 deg² for
  hold-position on 5 unseen episodes. Twenty episodes and 200 steps overfit the
  train split; this is a data-path/training-loop proof, not a policy result.

Limits: top view only, 0.4% of the dataset, no wrist stream, no crowding or
capture/tray tasks, torch 2.9.1 (not the pinned 2.8.0), and one seed. The next
run needs the full dataset (18 GB), the wrist camera, far more steps, and a
held-out learning curve before any quality claim.

## GTX 1060 thermal envelope — measured (2026-11-03)

Brian observed the fan turning. Direct sampling shows the fan **does** ramp, but
late and weakly, and the card cannot hold a continuous training load:

- **Idle:** 56 °C, fan **0 %**, 26 W.
- **Ramp under load (1024² FP32 matmul, ~0.7 s bursts):** fan stays at **0 %**
  from 56 °C through **83 °C** (about 4.4 s), then jumps to **71 %** at ~5 s and
  settles around **58–61 %**. Temperature then sits at **81–83 °C** at only
  ~40 W, with occasional spikes (one reached **87 °C** at 57 W, aborting the
  test).
- **Training soak (batch 8, ACT):** ~64 samples/s, 81–85 °C within the first
  minutes, throttle reason `0x20` (SW thermal slowdown); the 85 °C guard aborted.

Interpretation: the fan and heatsink are functioning, but the curve has a large
deadband (0 % until ~80 °C) and tops out near 60 %, so the card stabilises at
82–83 °C at low power and spikes above 85 °C under bursts. That is a marginal
cooling path — dust, aged paste, restricted airflow, or a conservative fan
profile — not a dead fan.

**Fan-control feasibility (2026-11-03):** there is no fan control surface
available from the container. `nvidia-settings` is absent and there is no X
server; the driver exposes no NVIDIA `hwmon` PWM node (only the CPU `k10temp`
hwmon exists), and `nvidia-smi` can report fan speed but not set it. The only
software lever is a power cap (`power.limit` 60–140 W, default 120 W), which
needs root on the Unraid host and affects the simulator's GPU too. It is also
unlikely to be sufficient: the card already sits at 82–83 °C at only ~40 W with
the fan at ~60 %, which points at degraded die-to-heatsink contact rather than
insufficient fan speed.

**Batch-1 test (2026-11-03):** even at batch 1 the card pins at the 83 °C guard
after ~35 s (8 samples, peak 83 °C, 15.0 samples/s, step 525) and the run exits
on the temperature guard. There is no throughput/heat trade left to make: the
card is at its thermal ceiling at one-third of the batch-8 rate.

**Consequence for Phase 11a:** a *continuous* batch-8 training soak is not
achievable within an 85 °C guard on this card, and neither is a reduced-batch or
duty-cycled envelope.

**Post-service retest (2026-11-03 18:10):** after cleaning the heatsink and
replacing the thermal paste, the same guarded ramp test is transformed — idle
**43 °C** (was 56), and 20 s at ~60 W reaches only **62 °C** (previously 80 °C at
1.5 s). The fan still sits at 0 % below its threshold, so the improvement is the
die-to-heatsink path, not fan speed. A 30-minute batch-8 soak is now the
acceptance run.

**Ingest complete (2026-11-03 18:1x):** all 82 pinned files verified present
against the HF listing — 75 videos (39 top, 36 wrist), meta and data, **0
missing**, 16.1 GB in `phase11_data`. Four stale `.part` files from an earlier
interrupted transfer are being removed.

**Post-service soak (in progress):** batch 8 sustained at **73–74 °C peak** after
40 s with 64.5 samples/s — down from 85 °C+ before servicing. The 30-minute run
is the acceptance gate.

**Service task (Brian, 2026-11-03):** clean the heatsink/fan and replace the
thermal paste. Post-service acceptance: steady state at or below 78 °C under the
batch-8 training load with the fan below 100 %, and a clean 30-minute soak
without the 85 °C guard. The current readings (82–83 °C at ~40 W; 56 → 84 °C in
under 2 s) are consistent with dried paste and blocked fins.

**Wrist stream validated (2026-11-03):** as the wrist videos arrived, a CPU smoke
run (2 episodes, 8 samples, 2 steps, batch 2) built samples from
`observation.images.wrist`, trained, and reloaded the checkpoint with exact
predictions. Both camera paths now exercise the same harness. Ingest reached
11.56 of 15.7 GB with 13 wrist files present; the remainder is still
transferring. Options, in order of preference:
physical cooling maintenance (clean/paste/airflow), a host-side fan curve or power
cap (needs operator access), or a duty-cycled training envelope measured to stay
under the limit. A duty-cycled run (batch 4, 60 s train / 90 s cool, 84 °C guard)
is the bounded test of that last option. CPU training remains available as a
fallback at ~3.8 samples/s.
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
