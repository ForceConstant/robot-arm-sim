# Kaggle free-tier investigation — 2026-10-30

**Superseded:** Brian ended cloud-host exploration and selected BC-250 as the
primary local ML target. The steps below are reference only, not pending actions.
See [current direction](BC250_CAPABILITY.md#local-optimization-order).

Brian's first free Colab attempt received **no GPU allocation**. This is an
allocation failure, not a model-fit or notebook-compute failure. No paid Colab
account or compute units are available. Kaggle is a candidate, not yet a
qualified or selected training host.

## Findings and recommendation

- Standard free Kaggle notebook choices commonly include **T4 ×2**, with
  16 GB dedicated VRAM per GPU, or **P100 ×1**, with 16 GB. Availability varies;
  inspect the account's actual accelerator menu before assuming either exists.
- The commonly reported allowance is **30 GPU notebook hours per week**, with
  **up to 12 hours per CPU/GPU execution**. Treat the displayed account quota
  and current documentation as authoritative, not this planning allowance.
  Unused quota does not guarantee an allocation. Phone verification may be
  required to unlock accelerators/Internet access.
- Two T4s are **not one transparent 32 GB GPU**. Ordinary data-parallel/DDP
  training replicates the model, and does not make a too-large single replica
  fit. Memory savings require compatible model/optimizer sharding or model
  parallelism, including a tested checkpoint/reload path. Our existing recipe
  has not been qualified for that topology.
- Prefer a T4 allocation for the first compatibility check. T4 has Tensor
  Cores and a mainstream CUDA path; P100 is older Pascal, lacks Tensor Cores,
  and requires checking that the installed PyTorch wheel and model kernels
  actually support it. Neither card provides native bf16 compute. Use the
  explicit fp16 path, with appropriate loss scaling, and qualify model ops.
- The BC-250's 16 GB is shared by CPU, GPU and OS, unlike dedicated GPU VRAM
  plus separate host RAM. Its measured GGUF/Vulkan 35B vision result does not
  qualify the custom PyTorch action-policy training path. A T4 therefore offers
  a meaningful **training compatibility** advantage despite similar nominal
  memory. No same-workload training speed comparison has been measured.
- Competition pages advertise upgraded L4×4 resources, but those are not
  evidence of unrestricted free capacity for this unrelated robot project.
  Do not build the plan around competition-specific hardware.

**Recommendation:** try Kaggle **T4 ×2** next, initially using one T4 for the
synthetic preflight and the policy's existing rank-8 LoRA pilot if the actual
trainable-parameter/activation budget permits. Rank 8 is not a fit guarantee:
the action expert and other unfrozen parameters also consume optimizer memory.
The default rank-64 recipe remains an estimated 21–23 GiB and does not fit one
16 GB card. Investigate sharding only after checking pinned-policy support;
do not claim two devices alone unblock MolmoAct2.

## Concrete next check

1. Create a private notebook at <https://www.kaggle.com/code>, complete any
   requested account verification, and import
   [`notebooks/colab_preflight.ipynb`](notebooks/colab_preflight.ipynb).
   Select **GPU T4 ×2** if offered. The notebook uses preinstalled torch,
   installs nothing, downloads no weights and uploads no private project data.
2. Run all cells and return the final JSON. The existing notebook smoke-tests
   **cuda:0 only**; its `nvidia_smi` field lists all visible cards. It does not
   exercise the second GPU, distributed training, fp16 model operators,
   LeRobot or actual-task fit. Relative output paths work on Kaggle even
   though the filename still contains `colab`.
3. Check available Internet access before later public model/dependency
   downloads. Keep the notebook private and deliberately save/download its
   outputs; runtime files are not a durable checkpoint plan. Confirm storage
   capacity for model cache, two checkpoints and headroom before real training.
4. If compute smoke passes, check the pinned stack's architecture/kernel
   support, actual trainable parameter count and peak memory in a bounded
   policy pilot. No long run or private-data upload is authorized by this note.

Public dataset use and uploading our LAN-only recordings are separate
decisions. Local deployment of a resulting robot-policy artifact remains an
independent unresolved gate. **Priority 5 remains in progress.**

## Sources and evidence limits

- [Kaggle notebook documentation](https://www.kaggle.com/docs/notebooks):
  search-indexed documentation confirms the 12-hour CPU/GPU execution limit;
  direct fetch returned a reCAPTCHA page. Full current account-specific terms
  and quota were not independently read in an authenticated account.
- [Kaggle GPU quota announcement](https://www.kaggle.com/general/108481)
  and [reported GPU availability problems](https://www.kaggle.com/discussions/product-feedback/743898).
- [Kaggle 2026 competition accelerator page](https://www.kaggle.com/competitions/arc-prize-2026-arc-agi-2/overview/upgraded-accelerators):
  indexed text distinguishes newer L4×4 from older T4×2/P100 resources and
  signals restricted use; not a general-purpose entitlement.
- [User reports comparing free Kaggle with Colab](https://www.reddit.com/r/GoogleColab/comments/1noduls/how_long_are_the_cooldowns_for_free_tier_gpu/)
  corroborate T4×2 and 30 hours/week; these are user reports, not an SLA.
- [Colab FAQ](https://research.google.com/colaboratory/faq.html), directly
  fetched: free GPU access is heavily restricted, resources are not guaranteed,
  and accelerator types vary over time. T4 is a sensible planning assumption,
  not an official promise that free Colab always and only allocates T4.
