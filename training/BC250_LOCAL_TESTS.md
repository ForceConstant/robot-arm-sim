# BC-250 local inference and training tests — 2026-10-30

## Scope

Requested continuation on Brian's existing Bazzite BC-250, targeting **one
wrist-camera image per inference request**. The camera input here is a saved
800×600 simulator fixture, not a live physical camera. The test prompt asks for
gripper visibility, board visibility and whether a complete board state is
available. The fixture shows the gripper and only part of the board; returning
`unknown` for the full state is appropriate. Repeated requests are stability
tests, **not 20 independent accuracy examples**.

Existing Qwen3.5-35B-A3B IQ2_M weights/projector and llama.cpp b12002 provenance
are recorded in [BC250_CAPABILITY.md](BC250_CAPABILITY.md). Each run uses one
slot, 1024 context, batch/microbatch 32, Q8 KV, no thinking, full requested GPU
offload, non-mmap loading and disabled optional prompt/context caches. The
server binds localhost only. No camera/model data left the LAN.

## Single-camera inference

- **256 image-token budget, unpaced:** seven responses completed, median
  approximately 2.12 s, then the resource guard stopped the run at **85 C**.
  Requesting JSON in the prompt alone produced Markdown fences, so strict raw
  JSON consumption did not pass. No memory guard was breached.
- **256 budget, strict schema, 5 s idle after each response:** **20/20**
  completed. Median **2.115 s**, nearest-rank p95 **2.149 s**, peak **81 C**,
  minimum CPU MemAvailable **1.164 GiB**, maximum swap growth **28.4 MiB**.
  Every raw response parses as JSON and gives the same visible-gripper,
  visible-board, unknown-complete-state result. Model load was **19.04 s**;
  the whole bounded run including load and pauses was **161.78 s**.
- **128 budget, strict schema, unpaced:** **15** responses completed, median
  **1.413 s**, then the guard again stopped at **85 C**. Prompt tokens fell
  from 295 to 178. Lowering the image budget improves this coarse fixture's
  latency but does **not** by itself solve continuous-operation thermals.

- **128 budget, strict schema, 5 s idle:** **20/20** completed with identical,
  valid JSON on the repeated fixture. Median **1.463 s**, nearest-rank p95
  **1.469 s**, peak **79 C**, minimum CPU MemAvailable **1.211 GiB**, maximum
  swap growth **12.8 MiB**. Load **18.04 s**, whole bounded run **147.38 s**.
  About 31% lower request latency than the paced 256-budget run, but only for
  the coarse visibility/unknown output tested, not piece recognition.

Request times include local request serialization, image/prompt processing and
response generation. They exclude model loading, the explicit idle interval,
live camera capture and simulator-to-inference-host network transfer. A 5 s
idle period means a roughly 7.1 s cycle at the 256 budget, not 2.1 s sustained
sampling (roughly 6.5 s per cycle at the 128 budget). No physical IRL capture/transport latency or piece/square accuracy is
qualified. Lower resolution/token budgets need held-out visual-quality checks.

The temperature guard is an experiment limit, not a specification of a safe
continuous operating temperature. Keep both failed continuous runs as evidence;
do not describe the paced result as an unpaced or hours-long reliability pass.

## GPU training without ROCm

A **user-space Tinygrad → Dawn WebGPU → existing RADV Vulkan** path completed
real GPU forward/backward and optimizer work. Adapter identity was checked as
`AMD BC-250 (RADV GFX1013)`, AMD vendor ID `0x1002`, Vulkan backend; the RADV
ICD was selected explicitly, with no CPU fallback.

| Synthetic FP32 model | AdamW steps | First → final-step loss | Total elapsed | Peak GPU edge |
| --- | --- | --- | --- | --- |
| 838 parameters, 6→64→6 | 20 | 0.18035 → 0.004689 | 6.43 s | 55 C |
| 1,062,918 parameters, 6→1024→1024→6 | 20 | 0.13176 → 0.04864 | 25.99 s | 58 C |

Both runs checked finite losses/gradients, saved model **and AdamW state**, and
required **exact prediction agreement after reload**. They then compared the
next resumed optimizer update with an uninterrupted branch; both matched
exactly. Loss is not monotonic, especially in the larger probe. These are
synthetic fixed-batch optimization tests, not a convergence or generalization
benchmark. Total elapsed includes initialization/compilation and checkpoint
checks; it is not a large-model training-throughput estimate.

- Both guards reported **zero swap growth**; minimum CPU availability was
  approximately **5.09 GiB** and **4.98 GiB**, respectively. Framework memory
  counters report current bytes, not peak total GPU memory.
- Checkpoint sizes: **14,844 bytes** and **17,008,996 bytes**. They include
  optimizer and duplicate parameter references, not just inference weights.
- The published parameterized probe was separately run at width 64 with the
  final guard and artifact checks; it passed. The guard's one-second timeout
  self-check terminated its child as expected. Project regression:
  **49 passed, 4 skipped**; all three probe scripts compile.
- No kernel warnings were returned by the bounded recent `journalctl -k`
  query. That is not proof that every device event or transient was observed.
  Worker logs preserve a pre-existing optional LSFG implicit-layer loader
  warning that Vulkan skipped; no gaming layer was changed.

### What this does and does not establish

**GPU training on this BC-250 is no longer wholly unproven:** small dense FP32
models can train through the existing Vulkan stack without installing ROCm.
This does **not** establish PyTorch/LeRobot compatibility, convolution/attention
operator coverage, low-bit or mixed-precision training, 35B Qwen fine-tuning,
MolmoAct2/action-expert training, dataset ingestion or an actual-task pilot.
The CUDA-oriented trainer cannot simply be pointed at Tinygrad. A compatible
real architecture/operator path and measured footprint remain required.

## Reproduction and state

See [user-space training probes](probes/README.md) for the pinned preparation,
guard and parameterized training commands. Tinygrad source and Dawn are
confined to a dedicated home-directory tree. No OS packages, driver, kernel,
firmware, BIOS, memory split, CU masks, fan/power settings, services or Docker
containers were changed. The existing gaming session remained running.
Only public source/runtime artifacts were downloaded; no new model weights.

Private receipts under `.private/bc250-investigation/receipts/`:

- `qwen35-single-wrist-20`
- `qwen35-single-wrist-20-paced-json`
- `qwen35-single-wrist-128-unpaced`
- `qwen35-single-wrist-128-paced-json`
- `webgpu-train-smoke` / `webgpu-train-guard`
- `webgpu-train-1m` / `webgpu-train-1m-guard`
- `webgpu-final-smoke` / `webgpu-final-guard`
- `webgpu-guard-timeout-check`

**Priority 5 remains in progress.** Next gates: varied single-camera scene
quality plus capture/LAN timing; sustainable thermal/duty-cycle envelope; and
a compatible real training/action-policy path followed by an audited-task
optimizer/checkpoint pilot. No physical robot is controlled by these probes.
