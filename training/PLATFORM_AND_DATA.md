# Training platform and data boundary

Priority 5 acceptance record, 2026-10-30. **Not yet qualified for a training run.**

## Deployment-first constraint — Brian, 2026-10-30

**Required:** trained models must run locally on the existing GTX 1060 3 GB
host or BC-250 APU. Cloud inference and a hardware purchase are not the default.
Colab is only a candidate training accelerator; a cloud training pass does not
qualify deployment. Training memory and inference memory are different: the
historical ~30 GB training peak alone neither proves nor disproves inference
fit, but no MolmoAct2 local-fit measurement exists. Treat that recipe as
historical research, not the selected production architecture.

**Superseding direction from Brian:** start big and find the largest practical
model that can run locally, rather than selecting small perception first.
Investigate BC-250 hardware/runtime limits before shrinking the architecture.
Keep MolmoAct2 and larger supported VLM/VLA candidates under consideration;
there is no evidence yet of their final local compatibility or latency.
Small models and calibrated/scripted control remain comparators/fallbacks, not
the chosen ceiling. HIL-SERL is a learning workflow, not proof of model fit.
Keep oracle labels out of inference inputs. See
[BC-250 research and qualification path](BC250_CAPABILITY.md).

Before substantial task training, select the local deployment host, framework,
model architecture, input resolution and action contract; exercise an exact-size
initialized/exported model there to catch unsupported operators and gross
capacity failures. Then run the trained checkpoint through these gates:

- **Memory/concurrency:** measure peak process RAM and total GPU usage with
  actual camera inputs, preprocessing, model, action buffers, simulator rendering
  and normal concurrent services active. Do not use the full 3 GB as the model
  budget or stop other consumers to pass. The previous <=1 GiB incremental
  NVIDIA target was an assistant-proposed budget, not Brian's requirement, and
  is withdrawn: search the available capacity with measured host/rendering
  headroom instead. On BC-250 account for one shared physical pool, not
  VRAM+GTT addition. Qualify GPU backends rather than assuming CPU-only use.
  Large-model search may use BC-250 inference with simulation on the GTX host;
  test network/camera/action latency and bounded-command handling separately.

- **Latency:** report warm-up, p50/p95 end-to-end preprocessing+inference+decode
  latency and sustained throughput. Provisional settled-board perception target
  is p95 <=500 ms (proposal, not a model exclusion rule). An unchunked
  10 Hz motor policy needs p95 <=100 ms per replenishment cycle. Larger models
  may use slower stop-observe-plan-execute or chunked/async control, with
  measured task completion time and no action-buffer underruns. Interpolate bounded targets locally; late/invalid output must stop
  or abstain, not replay stale commands. These targets still need measurement.
- **Artifact compatibility:** save/reload the inference-only checkpoint and
  processor configs on the actual local runtime; compare outputs within a
  documented tolerance. GTX 1060 is Pascal: do not assume modern CUDA wheels,
  BF16, FlashAttention or low-bit kernels work. Start with supported ordinary
  operators/FP32; FP16, quantization and alternate export backends need separate
  numerical, latency and held-out-quality qualification, not parameter-count
  estimates. Quantization is an option, not a promise of fit.
- **Task quality:** run the final local artifact on untouched held-out sessions
  and layouts; report perception confusion/unknowns or motor right-piece,
  grasp/carry/place, disturbance and abstention metrics as appropriate. Compare
  against the matching scripted baseline; physical deployment remains gated
  by independent calibration and guarded hardware trials.

Priority 5 now includes local deployment qualification, not just a remote
optimizer/checkpoint pilot. Priority 6 supplies the perception training/evaluation;
Priority 5 must establish the selected model/runtime's measured local feasibility
without claiming Priority 6 task quality or physical readiness complete.

## Platform decision and measured preflight

- The RTX 5090 32 GB / Python 3.12 / CUDA 12.8 setup in the inherited training
  README and experiment log describes historical upstream experiments. It is
  **not evidence that Brian owns or can access that machine**. Brian has been
  asked to select/confirm a training host and its access method.
- Measured in this workspace: GTX 1060 3 GB, driver 580.159.03; 62 GiB system
  RAM (21 GiB available at probe), 929 GiB filesystem free. Shared GPU, not
  reserved. This is the simulator/data-audit host, **not the selected MolmoAct2
  training host**. No BC-250 CUDA compatibility is assumed.
- `PYTHON=.venv/bin/python bash training/preflight.sh` failed: torch and the
  dataset/LeRobot imports are absent. The 3 GB GPU is also below the documented
  approximately 30 GB training peak. Do not install the heavy training stack in
  the simulator venv or allocate the shared GPU to a large training run.
- Candidate reproducible runtime: `training/Dockerfile` pins CUDA 12.8, torch
  2.8.0, LeRobot commit `2774d9bddcbbda50e697e162e89e7eaada8d7105`, and
  transformers/accelerate/peft. This is a recipe, not a built/qualified image.
  Capture the actual image digest, `pip freeze`, driver, accelerator, available
  VRAM/RAM, and filesystem free space on the selected host before qualifying it.
- Storage on that host must be chosen explicitly. Keep separate raw recordings,
  split manifests, exported datasets, model cache, checkpoints and evaluation
  receipts. Budget from measured bytes/episode and model size: allow input +
  export + cache + at least two full checkpoints + 20% free-space reserve.
  Historical full checkpoints were about 18 GB each. Do not auto-prune an
  existing run or enable the inherited user services.

## Bazzite / BC-250 candidate (live read-only probe)

Brian powered it on and asked whether it could provide training GPU memory.
SSH verified Cyan Skillfish BC-250 (`gfx_target_version=100103`, gfx1013),
Bazzite 44.20261026 / kernel 7.2.11-ogc4.1.fc44, 8 GiB reserved VRAM,
7.5 GiB CPU-visible RAM (4.7 GiB available), and 913 GiB disk free.
`/dev/kfd` and a render node exist, but torch, numpy, ROCm/HIP packages and
BC-250 compute-flush module parameters were absent in the inspected system.
The driver also reports about 7.47 GiB GTT; **do not add this to VRAM as
independent physical capacity**. Both consume the same 16 GB pool, including
OS/application needs. No large-allocation or GPU training test was run.

[Community BC-250 ROCm work](https://github.com/akandr/bc250-rocm) reports
small-transformer training using a native gfx1013 PyTorch/rocBLAS build,
patched amdgpu/KFD and firmware adjustments. It also documents possible host
lockups; stock PyTorch wheels omit this architecture. That is evidence of
feasibility on the author's board, not qualification of Brian's Bazzite box.
[AMD's supported platform matrix](https://rocm.docs.amd.com/en/latest/compatibility/compatibility-matrix.html)
is not a support commitment for BC-250/Bazzite.

**Recommendation:** candidate for bounded small occupancy/color perception
experiments, not the inherited approximately 30 GB MolmoAct2 recipe. Keep its
working gaming OS/kernel/firmware unchanged while choosing scope; a container
cannot supply missing host-driver fixes. Brian now requires local inference on existing hardware and explicitly
requests the largest practical model; investigate BC-250 capacity first and
consider Colab for training. The exact model/runtime
and training host are still unselected. No training-host selection, memory-split change, install or reboot
has been assumed from the power-on.

## Local-only boundary

- Default: recordings, camera frames, oracle labels, user-supplied CAD/printed
  assets, datasets, caches and checkpoints stay local/private. No automatic Hub
  push, dataset publication, cloud upload, or download of large model weights.
  A selected remote/cloud host needs a separately explicit transfer scope.
- Public Git contains code, manifest **schema**, documentation and aggregate
  measurements only. Raw paths, receipts and generated data belong in ignored
  `.private/`, `datasets/` and `outputs/`. Do not publish private mesh packs in
  a training image or dataset.
- Keep simulator and real-camera datasets in separate manifests/exports.
  Every recording needs explicit origin and collection-session/layout IDs.
  A model trained on simulator data is not physically calibrated or qualified.
- Hold out **entire sessions and layout groups**, not adjacent video frames.
  Allocate train/validation/test before collection/export; all recolors,
  corrections and retries of a held-out scene stay held out. Use a dedicated
  untouched final test set; checkpoint selection uses validation only.
  IDs must describe actual grouping, not new IDs invented per duplicate.
- Oracle FEN/layout/pose/verdict may generate labels and score evaluation, but
  must not enter camera-policy input features. `EpisodeRecorder` records the
  pre-step six-joint state and the commanded absolute six-joint target in
  radians, plus the two `ROBOT_CAMERAS` streams and task instruction. These
  action labels are not achieved next-state labels. Export converts to SO101
  degrees-with-offset; stride 3 turns 30 Hz recordings into 10 Hz data. Undo
  that convention and interpolate targets at execution.
- Only verified successes are imitation-positive examples. Preserve failures
  separately with their actual outcomes. Incident corrections also retain their
  failure links and runtime provenance; joint-only traces are not visual data.

## Fail-closed dataset audit

Create a private JSON manifest next to the recording directories:

```json
{"schema": 1, "episodes": [
  {"path": "train/shard_0000/episode_0000", "origin": "simulator",
   "session": "collection-A", "layout_group": "scene-A", "split": "train"},
  {"path": "test/shard_0000/episode_0000", "origin": "simulator",
   "session": "collection-B", "layout_group": "scene-B", "split": "test"}
]}
```

```bash
.venv/bin/python tools/audit_training_data.py .private/run/manifest.json \
  --out .private/run/audit.json
```

The audit rejects cross-split sessions/layouts, duplicate state/action/media
content, mixed origins, unsuccessful episodes, missing instructions, wrong
joint/camera contracts, malformed/nonfinite/constant action labels, convention
round-trip errors and video/state length mismatches. It decodes first/last
frames and checks frame counts. It cannot verify a human's provenance claim,
prove that differently encoded videos depict different scenes, or certify
visual synchronization/calibration from frame counts alone. Export **only the
explicit train paths**; the legacy recursive exporter does not enforce this
manifest. Do not point it at a common parent containing held-out recordings.

## Free-tier Colab only — Brian, 2026-10-30

**Allocation update:** Brian's first attempt received no GPU. Free Colab is
therefore an unqualified candidate, not a secured training host. See the
[Kaggle free-tier investigation](KAGGLE_FREE_TIER.md) for the recommended next
candidate and the distinction between two T4s and one 32 GB device.

Brian confirmed **no paid Colab account and no compute units**; free tier is the
training host. Consequences, from the current Colab FAQ and the measured model
size:

- **Hardware is not chosen.** Free tier typically allocates a **T4 16 GB**, but
  GPU type, session length and availability vary and are not guaranteed.
  **T4 is Turing: fp16 works, bf16 does not.** Any recipe written for
  bf16/A100 needs an fp16 + loss-scaling path instead.
- **Session limits.** Roughly 12 h maximum and idle disconnection; a long run
  will be interrupted. Every pilot must checkpoint to persistent storage and
  resume, not assume one continuous session.
- **No remote or distributed workers.** SSH shells, remote desktops, web-UI
  driving, distributed workers and similar patterns are disallowed on free
  managed runtimes. HIL-SERL's separate actor/learner servers cannot be the free
  tier design; an in-process, single-session loop is required.
- **Ephemeral disk.** The VM filesystem is deleted with the runtime. Datasets,
  checkpoints and receipts must live in Drive or another persistent store, and
  Drive is slow with many small files (archive, do not mount-and-stream).

### Memory arithmetic: the current recipe cannot run on a T4

`allenai/MolmoAct2-SO100_101` reports **5,442,196,272 parameters**. Weights only:

| Precision | Weights | On a 16 GB T4 |
| --- | --- | --- |
| FP32 | 20.27 GiB | does not fit |
| bf16 | 10.14 GiB | fits, but T4 cannot compute bf16 |
| fp16 | 10.14 GiB | fits; needs loss scaling |
| 4-bit | ~2.5 GiB | fits, quantization unvalidated |

Full fine-tuning adds AdamW state (two FP32 moments ≈ 40.5 GiB) plus gradients,
master weights and activations. **Full-parameter fine-tuning of this checkpoint
is impossible on a 16 GB T4 at any batch size.** The inherited batch-8,
~30 GB recipe assumed a 32 GB RTX 5090 and is not portable to free Colab.

**Correction (2026-10-30):** an earlier version of this note said the trainer
had no adapter option. That was wrong. The MolmoAct2 recipe already runs
`--policy.train_mode_vlm=lora` — LeRobot's own adapter mode, `lora_rank` 64,
about **737M trainable of 5.6B** — and `chess_sim/policies.py` loads those
adapters at inference. A second `--peft` path was added and then removed: it
duplicated the policy's own mechanism and would have wrapped the model twice.

What is actually missing for a 16 GB T4 is **footprint**, not existence of
adapters:

| Component | Approx. |
| --- | --- |
| Frozen fp16 weights | 10.1 GiB |
| Optimizer state for 737M trainable (fp32 master + 2 moments) | ~8.3 GiB |
| Gradients (fp16) | ~1.4 GiB |
| Activations at micro-batch 1 (gradient checkpointing on) | 1–3 GiB |
| **Total before any batch scaling** | **~21–23 GiB** |

So the default `lora_rank` still does not fit a T4. The supported levers are a
lower `--policy.lora_rank`, or a smaller policy. There is **no** upstream
`train_expert_only` flag at the pinned commit — the comment in
`chess_sim/training.py` that mentions one is stale and should not be relied on.

Only two things were added to the trainer, both verified at command-construction
level: a `--mixed-precision fp16|bf16|no` override (a T4 is Turing and cannot
compute bf16 at all) and `training/settings.free_tier_colab.env`, a 20-step
micro-batch-1 pilot using the policy's own LoRA at rank 8. Full regression:
**49 passed, 4 skipped**. No GPU run has happened, so fit remains arithmetic.

### What a free-tier pilot would have to be

1. **Parameter-efficient training**: LoRA/QLoRA adapters on attention
   projections (and optionally the action expert) with the backbone frozen, or a
   frozen-backbone action-expert-only run. LoRA in fp16 on a T4 is the realistic
   ceiling; 4-bit QLoRA is the roomier option but adds bitsandbytes and its own
   numerical risk.
2. **Micro-batch 1 with gradient accumulation** to a small effective batch, and
   gradient checkpointing.
3. **Checkpoint every few dozen steps to Drive**, resumable, with truncation
   validation; treat each session as disposable.
4. **A genuinely bounded pilot**: ~20 optimizer steps on audited task data,
   recording loss, gradient finiteness, peak allocated/reserved memory, step
   time, checkpoint save and reload agreement.
5. **Held-out evaluation after reload** — the current evaluator runs in one
   process with a single policy copy, which suits a 16 GB card.

### Data-transfer scope

- The **public** `XvKuoMing/so101_chess` dataset is already published; using it
  discloses nothing new.
- Our **private** recordings, camera frames, CAD/printed assets and checkpoints
  are LAN-only by default. Uploading them to Drive/Colab needs an explicit
  decision from Brian; this document records the requirement, not consent.

**Priority 5 stays in progress.** The policy already has adapter training;
the next gate is an actual GPU allocation followed by a measured footprint
and optimizer/checkpoint pilot, not adding another adapter mechanism.

## Bounded pilots and completion gates

1. Local collection/data pilot: two `e2e4` episodes, seed 20261030, plus one
   separately held-out `d2d4`, seed 912407; 160×120 two-camera recordings, one
   worker each. This measures recording integrity/storage only, not diversity,
   policy quality, full-resolution throughput or a training-platform pass.
2. Selected-host preflight: verify storage, driver/kernel-log visibility,
   pinned stack imports, CUDA numerical forward/backward and EGL rendering.
   Run the existing allocation probe only with enough free VRAM and no other
   workload; it attempts allocations up to 31 GiB. An unreadable kernel log
   is unknown, not proof of no driver faults.
3. After an audited split export and explicit local model availability, run a
   bounded **20 optimizer-step** pilot (single worker, effective batch 8;
   decrease microbatch/use accumulation as necessary). Record elapsed time,
   peak VRAM/RAM, finite loss/gradients, and successful checkpoint save/reload.
   Abort on OOM, nonfinite values, capacity/reserve breach or stalled progress;
   do not auto-resume into a long run.
4. Run a small held-out inference/servo round-trip after checkpoint reload.
   Report right-piece, grasp/carry, placement, collateral disturbance and
   abstention outcomes. A smoke pilot is not a policy-quality benchmark.
5. Transfer to the simulator only a reviewed inference checkpoint plus processor
   configs, joint/action convention, camera contract, source/image hashes and
   validation report; verify checksums and test in simulation before activation.
   Keep optimizer state/raw data on training storage. Never connect a new
   checkpoint directly to the eventual physical arm; physical calibration and
   bounded hardware gates are independent.

**Priority 5 remains in progress until the host/storage/access are confirmed,
selected-host preflight and optimizer/checkpoint pilot pass, the selected
artifact passes local deployment gates above, and receipts are recorded.** No long run, purchases, cloud resources or deployment are implied.

## Local pilot receipt

All **3/3** physical-contact moves succeeded (two train recordings, one held-out
recording). The full media audit accepted **3/3**, **648 frames**,
**620,193 bytes** including both camera streams and state/action metadata;
no audit errors. Train collection: 363.4 s total (181.7 s/episode); held-out:
177.2 s. Input render size was 160×120; FFmpeg padded recordings to 160×128.
Private receipts: `.private/priority5-pilot/manifest.json`, `audit.json`, and
`.private/priority5-preflight.log`. These tiny recordings do not estimate
full-resolution dataset storage or GPU training throughput. No LeRobot export,
optimizer step, checkpoint or learned-policy evaluation was run.

## Colab and HIL-SERL candidates — 2026-10-30

Brian suggested Colab and the official HIL-SERL simulation guide.

**Colab recommendation:** a managed NVIDIA runtime is more practical for
CUDA-oriented experiments than modifying the BC-250 gaming host. Actual GPU,
free VRAM, Python/torch versions, disk and session budget must be measured;
no specific GPU is guaranteed even on paid plans. A T4 16 GB or L4 24 GB does
not fit the unchanged approximately 30 GB batch-8 recipe; an allocated A100
40 GB is a plausible candidate, not a measured training pass. Smaller models
or changed microbatch/accumulation need their own pilot. High-RAM runtime
settings refer to CPU RAM, not VRAM.

The [official Colab FAQ](https://research.google.com/colaboratory/faq.html)
explains variable resources, idle/session limits, ephemeral VMs, and paid
compute-unit limits. Free-tier restrictions specifically include remote
control, distributed workers and “chess training.” Our task is robot
manipulation, not chess-engine training, but do not assume an exemption or
plan a free remote actor/learner worker. Paid-tier/compute-unit availability
and budget are pending Brian's answer. No subscription or resources purchased.
**Updated 2026-10-30:** Brian confirmed free tier only — see "Free-tier Colab only" below.

[`notebooks/colab_preflight.ipynb`](notebooks/colab_preflight.ipynb) reports
GPU/VRAM, versions and disk; runs 20 tiny synthetic optimizer steps, tests exact
checkpoint reload and one resumed step, and writes a JSON receipt. It uses
preinstalled torch and neither mounts Drive nor installs packages or uploads
private data. Notebook JSON/Python syntax was checked locally; **it has not
been executed in Colab**. It does not replace the actual-task training pilot,
LeRobot imports, EGL qualification or persistent-storage selection. Brian must
open it in his Colab account and return its JSON report to qualify allocated
resources. Persistent checkpoint storage and a specific cloud-data transfer
manifest must be selected before real training data leaves the LAN.

**HIL-SERL:** the [official simulation guide](https://huggingface.co/docs/lerobot/main/en/hilserl_sim)
provides MuJoCo `gym_hil` environments for Franka Panda cube pickup, keyboard/
gamepad interventions and separate actor/learner processes on NVIDIA/CUDA.
It is not a ready-made SO101 chess environment or a MolmoAct2 training recipe.
Our scene needs a compatible environment/processor adapter, observation and
action contract, physical-success rewards/termination/reset, replay/intervention
collection and held-out baseline comparison. Preserve contact-based verdicts,
keep oracle labels out of vision inputs, and disable Hub upload (the tutorial
example enables it). The guide follows LeRobot `main`; qualify a separately
pinned HIL-SERL runtime rather than assume compatibility with historical
commit `2774d9b`. Headless colocated actor/learner may be investigated after
the platform gate; interactive LAN-to-Colab HIL networking is unverified.
The link informs a later bounded RL comparison, not completion of Priority 9.
