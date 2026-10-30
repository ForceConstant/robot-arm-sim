# BC-250 user-space training probes

These are **synthetic framework probes**, not robot-policy training. They do
not change the simulator, kernel, firmware, driver, services or memory split.
The host must already have a working AMD RADV/Vulkan driver and Python 3.11+
with standard-library archive filtering support. The tested host uses Python
3.14.8. No pip installation or system package change is required.

Copy the three Python files to a user-owned directory on the BC-250. Then run:

```bash
python3 prepare_bc250_webgpu.py
python3 bc250_guard.py --out receipts/guard-width1024 -- \
  python3 bc250_webgpu_smoke.py --width 1024 --steps 20 \
  --out receipts/train-width1024
```

Use a fresh output directory for each guard invocation. Preparation stages
Tinygrad source and Dawn under
`~/.local/share/robot-arm-bc250-investigation/training-webgpu/`; it downloads
only public source/runtime artifacts and checks the exact tested hashes. It
does not download model weights or upload data. Training runs locally on
synthetic six-dimensional inputs, with no robot images or commands.

## What is checked

- Vulkan is forced to the existing RADV ICD; the WebGPU adapter must identify
  as AMD hardware. There is no CPU fallback.
- FP32 forward/backward, finite gradients, 20 AdamW steps and lower final loss.
- A checkpoint includes model and optimizer tensors. Predictions must reload
  exactly, and the next resumed AdamW update must match an uninterrupted update.
- The wrapper samples CPU memory, swap and GPU edge temperature once per
  second. It terminates its own child process group at 85 C, more than 512 MiB
  swap growth, three samples below 768 MiB CPU availability, or its time limit.
  Sampling is not protection from every transient or driver hang. It does not
  alter fan speed, power limits or other workloads.

The reported framework-current-bytes value is not peak device usage. Use the
sampled driver/system receipts for total-resource accounting. The wrapper
refuses to overwrite an existing receipt directory.

## Provenance

- Tinygrad commit `0dd490c19492e606aadde8dc25e10b9223e06dbf`, source archive
  SHA-256 `52103076081ff065f29046ce23cb15505dca8615ca1927cf2fb7fc640a3fa36f`.
- Dawn `wpmed92/pydawn` v0.1.6 Linux shared library, SHA-256
  `cf36091d266a32c9d5080f14662de44cece241987939713282ea0ff558db81c6`.
  This is the version used by Tinygrad's CI at that commit. Its GitHub release
  has no publisher digest; the pinned hash records the tested artifact identity,
  not an independent audit or signature.

Existing files are checksum-checked, not silently upgraded. This is a pinned
experiment, not a recommendation to change system libraries or use arbitrary
future releases.

## Scope limits

A million-parameter dense network is not a billion-parameter VLM/VLA. These
probes do not qualify convolutions, transformer attention, low-bit gradients,
mixed precision, a PyTorch/LeRobot backend, or MolmoAct2's action expert. The
existing CUDA-oriented trainer cannot use Tinygrad without implementation and
operator-compatibility work. Do not extrapolate capacity or tokens/actions per
second from these results.
