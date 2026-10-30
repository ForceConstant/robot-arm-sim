# Training platform and data boundary

Priority 5 acceptance record, 2026-10-30. **Not yet qualified for a training run.**

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
cannot supply missing host-driver fixes. A direction question is pending:
qualify small perception models here versus select a separate NVIDIA host for
MolmoAct2. No training-host selection, memory-split change, install or reboot
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
selected-host preflight and optimizer/checkpoint pilot pass, and receipts are
recorded.** No long run, purchases, cloud resources or deployment are implied.

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
