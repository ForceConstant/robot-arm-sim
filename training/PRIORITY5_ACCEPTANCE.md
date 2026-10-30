# Priority 5 acceptance — local platform and data boundary

2026-10-30. **Complete for a bounded local platform/data qualification**, not
for an autonomous chess robot or a production learned motor policy.

## Architecture and scope decision

Following Brian's discussion of reusing controls without large-network training,
use **frozen pretrained perception + calibrated geometry/IK/scripted motion** as
the implementation direction. BC-250 is the local ML host; GTX 1060 remains the
simulator host. One wrist camera is the physical observation requirement.
Large-network fine-tuning is not a prerequisite for this architecture.

The prior Priority 5 text mixed platform readiness with final trained-controller
quality. This acceptance explicitly separates them: the audited real-task
optimizer/reload pilot is now measured, while usable perception is Priority 6,
state-change inference is Priority 7, optional learned motor-policy qualification
is Priority 8, and physical calibration is Priority 10. **None of those gates is
passed or waived by this document.** In particular, the small pilot below is
rejected as a controller, not selected as a substitute for an unqualified VLA.

## Completed platform gates

- **Hosts/runtime:** existing Bazzite BC-250 gfx1013, existing RADV Vulkan;
  portable llama.cpp b12002 for frozen Qwen vision; pinned Tinygrad/Dawn for
  optional small FP32 experiments. No ROCm/PyTorch/LeRobot training compatibility
  is claimed. No kernel/firmware/driver, fan/power or system-package changes.
- **Frozen inference artifact:** Qwen3.5-35B-A3B UD-IQ2_M, 11,391,613,568 bytes,
  SHA-256 `187b5bbb81d7ec1dc228cb04120302cc0d486cb3c988501af87e5f02e15ace0e`;
  F16 projector 899,283,648 bytes,
  `a516ab92e8240da4734d68352bdfba84c16e830ee40010b8fac80d69c77272ff`.
  Largest passing tested VLM, not a hardware maximum or qualified chess recognizer.
- **Bounded operating mode:** one request at a time, 128 image-token budget,
  strict JSON, 1024 context, optional caches disabled, >=5 s idle after each
  response, sampled memory/temperature guards. Prior 20/20 paced requests pass
  at p50/p95 1.463/1.469 s, peak 79 C. Unpaced runs fail at 85 C; no continuous
  service or hours-long reliability claim. Images/commands can be incomplete:
  uncertainty must cause abstention, not invented board state.
- **Storage:** BC-250 user-owned
  `~/.local/share/robot-arm-bc250-investigation/`; public model/runtime staging,
  private task exports and receipts remain under that local experiment tree.
  Latest free-space probe: about 890 GiB. Source recordings and original audits
  remain under simulator checkout `.private/priority5-pilot/`; new export,
  checkpoints and copied receipts under `.private/priority5-task-pilot/`.
  No services, pruning or unattended training jobs were enabled.
- **Data boundaries:** all 3 original successful physical-contact simulator
  recordings re-audited, including full videos, 648 frames; sessions/layouts
  disjoint across train/test, simulator/real origins separate. No private data
  leaves the LAN. Real physical capture sessions are not yet available.
- **Actual-task pilot:** 32 training observations from 2 episodes and 16 held-out
  observations from 1 independent episode, sampled across each episode. Only
  wrist RGB resized to 8x8, six pre-step joint observations divided by pi, and
  four instructed square coordinates enter the 202-feature model. No top view,
  FEN, oracle piece pose, task phase or success label enters observations.
  Targets are the six commanded joint deltas in radians. This is a deliberately
  tiny ingestion/operator pilot, not a representative training corpus.
- **Optimizer/checkpoint:** 202→64→64→6 tanh MLP, 17,542 FP32 parameters,
  20 AdamW updates. Finite gradients and loss; training MSE
  0.0117493→0.000443648 rad². Checkpoint reload and resumed update match exactly.
  Initial guarded run 10.87 s, peak 53 C,  >=5.046 GiB CPU availability and zero
  swap growth. A deterministic accounting repeat also passed, with identical
  exported model bytes; driver-total peaks 476,377,088 VRAM bytes and 87,199,744
  GTT bytes, peak 52 C. These are sampled totals including existing workloads,
  not dedicated model allocation; VRAM/GTT are not independent physical pools.
- **Inference-only transfer/reload:** exported 70,632-byte safetensors model plus
  processor/action contract; weights SHA-256
  `814d228378498f69c2c85a991ed992b42b2fff877d7f474ceae399f53263a8cf`.
  A new process loads that artifact without optimizer state and reproduces all
  held-out predictions exactly.16 serialized GPU predictions: p50 13.39 ms,
  p95 270.44 ms (includes first batch-size-one compilation; not a warmed 10 Hz
  qualification). CPU list conversion/synchronization included; camera capture,
  networking and motion execution are not part of this small-model timing.

## Held-out result — model rejected for control

Held-out joint-delta MSE **0.0157158 rad²**, versus **0.00286711 rad²** for
predicting zero delta (holding position). Initial random-model MSE was
0.0113778 rad². Training loss improved while held-out MSE worsened. Held-out MAE
was 0.08116 rad. No test labels were used for gradient updates or model selection;
no extra training/search was done to improve this reported test score.

This is a successful **platform/data-path pilot** and a failed **generalization
candidate**. No rollout success is inferred from finite predictions, no learned
commands were sent to a simulator or physical arm, and this model must not be
activated as a controller. Conventional control remains the selected direction.

## Evidence and reproduction

Dataset export SHA-256:
`f2fc8a27ca3d6c5b3365ad8e7d53dd8c6c3518fe54e6b8aca778764afae2b049`.
Private receipts: `task-pilot`, `task-pilot-guard`, `task-pilot-accounted`,
`task-pilot-accounted-guard`, `task-pilot-reload`, `task-pilot-reload-guard` beneath
`.private/priority5-task-pilot/`; dataset/checkpoint/prediction files remain private.

```bash
.venv/bin/python tools/export_bc250_task_pilot.py \
  .private/priority5-pilot/manifest.json --out .private/new-task-pilot/dataset.json
# Copy dataset and the probe/guard to the existing BC-250 user-space stage over LAN.
python3 bc250_guard.py --out receipts/new-task-guard --seconds 300 -- \
  python3 bc250_task_pilot.py --dataset dataset.json --width 64 --steps 20 \
  --out receipts/new-task
python3 bc250_guard.py --out receipts/new-reload-guard --seconds 300 -- \
  python3 bc250_task_pilot.py --dataset dataset.json --width 64 \
  --checkpoint receipts/new-task/model.safetensors --out receipts/new-reload
```

The exporter audits media/provenance before selecting samples. The probe requires
fresh output paths. Existing [pinned preparation](probes/README.md) applies;
there is no package installation or new model download for this pilot.

## Read-only simulator-to-BC-250 LAN check

After training stopped, the frozen 35B configuration fetched the current
`white_wrist.jpg` from the existing simulator on the GTX 1060 host, while its
normal renderer remained available. Four serialized, paced requests completed
as strict JSON: gripper visible, board visible, complete state unknown.
JPEG retrieval took 6.16–9.17 ms; fetch+inference took 1.445–1.498 s. Model load
was 19.05 s, peak 72 C, minimum CPU availability 1.323 GiB, maximum swap growth
51.1 MiB. Driver-total peaks 8,465,629,184 VRAM bytes and 4,504,338,432 GTT bytes;
these share physical memory and must not be added as independent capacities.
This is current simulator JPEG retrieval over LAN, **not physical-camera capture,
frame freshness, four independent scenes or board-state accuracy**. No motion or
simulator configuration was changed. Probe/server exited; receipt:
`.private/priority5-task-pilot/qwen35-single-wrist-lan4/`.
