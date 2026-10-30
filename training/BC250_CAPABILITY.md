# BC-250: largest practical local model investigation

Research/inventory: 2026-10-30. Brian's direction is **start big, determine the
largest practical locally runnable model**. This supersedes the assistant's
small-perception-first recommendation and arbitrary 1 GiB inference budget.
Model size is the search objective; working robot observations/actions, quality,
latency and stability remain acceptance gates. No new GPU purchase is selected.

## Bottom line

BC-250 is a credible large **quantized inference** candidate and deserves testing.
It is not a drop-in RTX 5080, CUDA accelerator, or equivalent training platform.
Matching the headline 16 GB does not mean equal usable memory or performance.
There is no measured largest robot-policy model on Brian's machine yet.

## Hardware and live configuration

- Whole computer/APU, not an add-in GPU card: Zen 2 CPU plus Cyan Skillfish
  `gfx1013` graphics. Commonly called RDNA 1/1.5; do not assume modern RDNA
  matrix/AI acceleration from its PS5 lineage. Stock boards expose 6 CPU cores
  and 24 GPU CUs; unlocks and actual working silicon vary.
- Read-only SSH today: **8 cores / 16 threads**, Bazzite kernel
  `7.2.11-ogc4.1.fc44`, Mesa RADV **26.2.4**, Vulkan device API **1.4.354**.
  `/dev/kfd` and `/dev/dri/renderD128` exist.
- Saved WGP masks `0x1d,0x1f,0x1f,0x1f`: 18 CUs on SE0, 20 on SE1, **38 total**.
  Boot service log confirms dispatch registers updated to 38/40. This proves
  the apply operation, not sustained compute stability or a speedup. Uneven
  shader-engine topology may limit scaling; benchmark rather than treating
  a reported CU count as proportional useful ML throughput. No masks changed.
- **8 GiB reserved VRAM**, **7.5 GiB CPU-visible RAM**, about **4.8 GiB available**
  in this sample; GTT accounting ~7.47 GiB. One physical **16 GB GDDR6** pool:
  these categories are not independent/additive. Disk swap is not extra fast VRAM.
- Boot `ttm.pages_limit=3014656` corresponds to an 11.5 GiB page-count limit.
  This alone is not the usable model-allocation limit: reservation, GTT/KFD,
  fragmentation and runtime allocations all matter. Community larger-model
  configurations raise limits, which has not been done here.
- No `llama-cli`, `llama-bench` or `rocminfo` on PATH, no GGUF found in the bounded
  home-directory scan, no BC-250 ROCm flush module parameters. Earlier system
  Python probe lacked torch/numpy. Alternate environments may exist; no model
  inference, allocation stress or optimizer benchmark was run on this host.

## RTX 5080 comparison (desktop card, not laptop)

| Dimension | BC-250 | RTX 5080 |
| --- | --- | --- |
| Memory | 16 GB shared with Linux/CPU/display | 16 GB dedicated GDDR7, host RAM separate |
| Matrix hardware | No comparable dedicated Tensor Cores | Fifth-generation Tensor Cores |
| Software | Working Vulkan graphics; application-specific compute; experimental patched ROCm | CUDA/PyTorch ecosystem with supported Blackwell builds |
| Bandwidth evidence | Community measured ~432 GB/s streaming read on one 40-CU board | NVIDIA specifies 960 GB/s peak |
| General GPU math evidence | Community PyTorch FP32 GEMM ~4.61 TFLOP/s on that setup | Published FP32 peak ~56.3 TFLOP/s |
| Training | Source-built gfx1013 stack, operator/numerical/stability work | Much more practical mainstream mixed-precision training |

Measured application/GEMM figures versus theoretical peaks are **not** an
apples-to-apples speed ratio. Do not compare NVIDIA sparse/low-bit AI TOPS to
BC-250 FP32 TFLOPS. Need identical model, quantization, context, image resolution,
backend and power/thermal conditions for an honest application comparison.

**Can it replace a 5080?** Functionally for some models at lower throughput,
possibly yes; as a general training/CUDA/performance replacement, no. Even a
5080's 16 GB does not fit the unchanged ~30 GB historical training recipe.

## What the strongest community evidence actually demonstrates

Source: akandr/bc250-rocm, commit
`9a2abb17834ffefbadd3312926fa83571f3f7b08`, README benchmark section. One author's
40-CU board, 1500 MHz cap, Fedora 44, patched llama.cpp `7ba604f`, flash attention
on, median of 18 samples, empty-context decode (`tg64`):

| Model/artifact | File size | Vulkan decode |
| --- | --- | --- |
| Qwen3 14B Q4_K_M | 8.63 GiB | 34.7 tokens/s |
| Qwen3.8 27B UD-IQ3_XXS | 11.09 GiB | 17.6 tokens/s |
| Qwen3.6 35B-A3B MoE IQ2_M | 10.72 GiB | 86.9 tokens/s |

These are **text-model** receipts, not Brian's results or robot-control rates.
MoE uses only a subset of weights per token but must still store the full model.
Aggressive low-bit quantization can lose quality; parameter count alone does not
make a model preferable. Vision adds projector/image-token memory and compute;
VLA adds an action expert and often iterative denoising. Long-context throughput
is different from empty-context decode. Community documentation includes logs
and correctness probes, but is not a vendor support promise or independent
replication. Its measurements also show thermal throttling and GPU-reset failure.

### Runtime paths

1. **Vulkan / llama.cpp:** strongest initial route for supported GGUF language
   and vision models without rebuilding the host kernel. Upstream libmtmd lists
   Gemma 3 12B/27B, Qwen2.5-VL 7B/32B and other vision families. This is upstream
   model support, not proof every Vulkan operator, quantization or memory size
   works on this board. Reserve projector, image tokens, KV cache and workspace.
2. **ROCm / PyTorch:** now demonstrated on gfx1013, not categorically impossible.
   Community setup requires patched amdgpu/KFD, firmware changes, native rocBLAS,
   comgr/runtime fixes and source-built PyTorch. Stock wheels are not sufficient.
   Its small-transformer training runs at 33.8 ktokens/s, but that does not
   predict VLA training. FP16 autocast was slower than FP32 in its tests. A GPU
   reset can take down the host; this is a separate host-change project.
3. **CPU / split offload:** potential fallback for unsupported parts or memory
   pressure, but measure full latency. BC-250 CPU shares the same memory pool;
   it does not acquire additional memory by moving layers to its CPU. Running
   the simulator on GTX 1060 and inference on BC-250 can isolate workloads;
   measure LAN/camera/action transport without assuming WAN/cloud inference.

## MolmoAct2 is worth investigating, not ruled out by training memory

Hugging Face reports **5,442,196,272 parameters**, all stored as F32, for
`allenai/MolmoAct2-SO100_101`, revision
`152569fe57914d97be91055800035f54e250d009`. Idealized weights-only arithmetic:

- FP32: **20.27 GiB** — exceeds the BC-250 physical pool.
- FP16: **10.14 GiB** — potentially near total-memory limits once everything
  else is included; not proof of fit on today's split/runtime.
- 8-bit: **5.07 GiB**; 4-bit: **2.53 GiB**, **before** scales, unquantized layers,
  vision/action components' working memory, attention cache and framework overhead.

This is arithmetic, not an available quantized checkpoint or working backend.
Custom MolmoAct2 architecture/action expert is not established to work via GGUF
or llama.cpp; ordinary VLM support does not imply VLA action-output support.
AMD's September 30 tutorial runs MolmoAct2 on **Ryzen AI Max+ 395 / Strix Halo**
using ROCm/Ryzers. Valuable AMD implementation evidence, but different hardware,
architecture, memory capacity and support: not a BC-250 compatibility claim.
Investigate its dependencies and action-chunk timing before selecting a port.

## Qualification sequence: pursue the upper boundary, then step down if needed

1. Inventory above is complete. Pick a pinned, user-space Vulkan build path and
   inspect build/dependency requirements before changing the gaming host.
2. Establish actual allocatable memory and numerical correctness using bounded
   probes, retain OS/display headroom, monitor temperatures and avoid blind
   allocation-to-OOM. Do not change BIOS, CU masks, firmware or kernel to write
   this research report; propose necessary changes concretely for review.
3. Search **large compatible** artifacts first: approximately 12–14B 4-bit VLM
   candidates, then 24–32B lower-bit candidates if total memory permits. These
   are search tiers, not demonstrated fits. Use 27B/35B text receipts only as
   capacity/backend controls, not substitutes for robot-task evaluation.
   Investigate MolmoAct2 quantization/operator support in parallel as a VLA path.
4. Record exact weights/revision, all non-text components, quantization, image
   resolution/count, context/action horizon, runtime build, offload and peak
   physical RAM/GPU usage. Benchmark image-to-decision/action-chunk p50/p95,
   output agreement and held-out robot-task quality, not text tokens/s alone.
5. Keep the largest **useful, stable, locally runnable** model. If a large model
   needs seconds per observation, test stop-observe-plan-execute or safe action
   chunks; do not force every model into a small-policy 10 Hz synchronous loop.
   Shrink only when a measured gate fails or a smaller artifact performs better.
6. Select Colab/local training only after deployment feasibility. Priority 5
   remains in progress pending model/runtime selection and measured local plus
   training pilots. No host changes, installs, model downloads or training were
   performed for this research pass.

## Sources

- [Community BC-250 ROCm measurements and logs](https://github.com/akandr/bc250-rocm/tree/9a2abb17834ffefbadd3312926fa83571f3f7b08)
- [Community Vulkan investigation](https://github.com/akandr/bc250/tree/fac2867d948ac97f7a44c08f054d618fab8922ca)
- [BC-250 field guide](https://bc250.info/) and [AI caveats](https://bc250.info/ai/)
- [NVIDIA desktop RTX 5080 specs](https://www.nvidia.com/en-us/geforce/graphics-cards/50-series/rtx-5080/)
- [NVIDIA Blackwell architecture whitepaper](https://images.nvidia.com/aem-dam/Solutions/geforce/blackwell/nvidia-rtx-blackwell-gpu-architecture.pdf)
- [RTX 5080 FP32/spec summary](https://geeks3d.com/20250129/nvidia-geforce-rtx-5080-blackwell-reviews/)
- [llama.cpp multimodal support](https://github.com/ggml-org/llama.cpp/blob/master/docs/multimodal.md)
- [MolmoAct2 parameter metadata](https://huggingface.co/api/models/allenai/MolmoAct2-SO100_101)
- [AMD MolmoAct2 on Strix Halo tutorial](https://rocm.blogs.amd.com/artificial-intelligence/molmoact2-ryzers-strix/README.html)
- [Reddit first-hand discussion](https://www.reddit.com/r/LocalLLaMA/comments/1mqjdmn/did_anyone_tried_to_use_amd_bc250_for_inference/) (older ROCm conclusions superseded by native-build work)
