# Phase 11 — local training paths: investigation notes

Recorded 2026-11-03 from Brian's four questions. These are **research notes and
open decisions**, not completed gates. Phase 11 is in progress; the execution
runtime receipt is separate (`PHASE11_RUNTIME_QUALIFICATION.md`).

## 1. Has a vision policy been chosen for the future tasks?

**No.** The measured floor and the gaps:

| Task | Current approach | Status |
| --- | --- | --- |
| Board pose / registration | Geometric: checker-saddle coarse pose, edge-consistency phase fix, Nelder–Mead refinement | Measured; 1.42 mm median, 1.92 mm max origin error, 5/8 held-out sessions accepted |
| Occupancy / colour | 32-unit NumPy MLP over 34 patch features; trained locally on CPU | Measured; 99.4% answered accuracy at 96% coverage; synthetic-occluder abstention only 22.5% |
| Opponent-move detection | Legal-successor comparison (`move_inference.py`); no learning | Measured; 13/13 accepted correct, 0 false confirmations, 37.1% coverage |
| Piece type + in-square position/orientation (incl. knight yaw) | Not selected; Phase 10 names segmentation/keypoints only | **Open** |
| Wrist-only registration under real occlusion | Phase 9 in progress | **Open** |

Worst-case work therefore means a small segmentation / keypoint model (or a
small VLM) covering occlusion-robust move changes, per-piece type plus
position/orientation, and wrist-only registration. Requirement: train it
locally with measured throughput, memory, thermal and held-out error against the
Phase 6/9/10 budgets, or record a measured frozen-model + calibrated-geometry
decision meeting the same budgets. The existing CPU-trained MLP is the floor,
not the answer.

## 2. Is the Vulkan compute path a possibility?

**Yes, but bounded — it is a ladder rung, not a PyTorch replacement.**

- `llama.cpp` Vulkan is measured working on this board (the 35B VLM probes).
- Tinygrad → WebGPU/Vulkan works: a 1.06M-parameter FP32 training run passed with
  exact checkpoint resume; the full ACT graph reached the device and failed on
  one operator (`cumsum(1, dtype=torch.float32)` in the 2-D positional
  embedding).
- PyTorch's own Vulkan backend is inference-only and unmaintained, with no BLAS,
  so it cannot serve as a training backend for ACT.

Implication: Tinygrad/Vulkan is the cheapest GPU route for a **small vision
model** and for inference. It is not an autograd path for ACT without either
implementing the missing operators or writing dedicated compute kernels.

## 3. Is the ROCm 9221 fix in the kernel or in ROCm?

**Kernel.** `ROCm/TheRock#9221` is a user freeze report (ROCm 7.2.3, gfx1013,
system freeze after any compute workload, still open, 26 comments) asking for an
explanation rather than specifying a patch. The community diagnosis places the
fault in the **`amdgpu`/`amdkfd` TLB/PASID flush path** — a kernel-driver bug;
without the runlist-flush fix, compute returned wrong elements with no error.

Separate from that, ROCm **userspace** still needs architecture enablement: a
native gfx1013 rocBLAS build and (on ROCm 7.1.1) the comgr table fix. Both
layers are required; neither substitutes for the other.

## 4. Is SDMA a config setting or an issue?

**Neither a config setting nor a tracked upstream issue — it is two more kernel
patches plus a firmware selection.**

Measured evidence (`cachenetics/project-ariel#13`, closed): the board's
`cyan_skillfish2` SDMA firmware never drives user queues — 0 of 4 MiB copied on
**both** engines, signal never drops — while another SDMA 5.0 blob works
(navi12 copies 4 MiB in 0.04 s, +20% H2D). The boot "fence fallback timer
expired" lines are an init-order artifact, not dead silicon. Their fix:

- `bc250-sdma-firmware-override` — module parameter `amdgpu.bc250_sdma_fw=navi12`
- `bc250-early-sdma-trap` — writes TRAP_ENABLE before `ip_init`

On Bazzite (immutable root) this means layering another kernel build, not a
modprobe/sysctl line. Navi12 blobs are already installed locally
(`navi12_sdma.bin.xz`, `navi12_sdma1.bin.xz`), but firmware loads early and
`firmware_class.path=/usr/local/lib/firmware` may lose to the initramfs copy —
unverified.

## Cost-ordered ladder (the actual plan)

1. **CPU threads** — already trains ACT; re-measure at 8 threads rather than 4.
2. **GTX 1060 3 GB** — fit test at batch 1/2/4/8 in FP32 (Pascal FP16 compute is
   1/32–1/64 rate), with live-simulator VRAM/SM contention stated. Needs a
   throwaway CUDA container (approval outstanding).
3. **Tinygrad/Vulkan** — small vision model first; price the missing ACT
   operators separately.
4. **Patched ROCm** — four required pieces: kernel patch set (PASID/runlist
   flush, SDMA override), navi12 firmware, native gfx1013 rocBLAS (+comgr or
   ROCm 7.2), PyTorch built for gfx1013. No host change without a written change
   set and rollback, in a maintenance window.

Stop at the first rung that trains **both** the control and vision models
end-to-end. Every rejected rung keeps a receipt with its measured blocker.
