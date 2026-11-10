# Phase 11b — local control-policy training

**Status: complete 2026-11-10, gate met in aggregate, with a named move-family
shortfall.** Phase 11 remains open only for **11c** (vision model). This receipt
is a simulated training-path qualification, not a controller acceptance and not
a physical result. No robot commands, drivers, serial ports or hardware were
touched; everything ran in user space on the BC-250 CPU.

## Question

Can a full-size goal-conditioned ACT policy be trained **locally** on the
product distribution — Phase 9's wrist observation contract and Phase 10's
grasp/task buckets — with measured throughput, memory, thermals, exact reload
and a held-out learning curve that improves on hold-position, stably across
seeds?

## Build and host

- Host: BC-250, CachyOS `7.3.1-1.401-cachyos-bc250`, 16 cores, 14 GB RAM.
- Source: `robot-arm-sim` Release **1.13.0**, commit `2711162`.
- Product assets: `gripper-assets-v5-tipthin` (tip-thin parallel jaw) and
  `piece-assets-v9-brian-parallel-knight` (Brian's measured 4.7 g
  parallel-sided knight; lower-8 king retained).
- Pinned ACT: LeRobot commit `2774d9bddcbbda50e697e162e89e7eaada8d7105`,
  `modeling_act.py` SHA-256
  `8d622935319af7d03f1efd95d51e726af2921010f7e4f5ec3742f62280493111`.
- CPU-only FP32. GPU acceleration is not claimed; the BC-250 GPU remains
  rendering-only under the current ROCm decision.

## Corpus

Collected by `tools/collect_phase11b.py`, exported by
`tools/export_phase11b.py`; both were added for this phase. The recorder now
stores the task's explicit `goal_xy_m` (source and destination in the board
frame) in `meta.json`, so the policy never has to infer task intent from FEN or
object poses.

- **48 buckets** = 6 piece types × 2 colours × 2 task families × 2 crowding
  levels (sparse 1–2 extra pieces, crowded 7–8).
- **768 episodes recorded, 756 verified successes (98.4%)**: 379/384 moves and
  377/384 captures. The 12 refusals are scripted-expert guard failures and are
  excluded; no failed demonstration is used as a positive.
- Wrist camera only, 224×224 RGB at 30 Hz, fresh appearance/lighting, randomized
  board placement and arm start per episode.
- **18,144 samples**: 13,608 train (567 episodes) / 4,536 held out (189
  episodes). Every fourth episode of each bucket is held out, so no episode or
  layout appears in both splits.
- Action contract: 30-step absolute-joint chunks at 10 Hz (every third 30 Hz
  frame), padded at the end of an episode; loss and evaluation ignore padding.
- Goal contract: `[source_x, source_y, 0, destination_x, destination_y, 0]` in
  board metres.
- Dataset SHA-256 `189c42fcfb7683667d92d0284ff02c1ce9ff7b0017f063f59d07da699f78162a`.
- Collection cost: 12,922 worker-seconds (~54 minutes wall at 4 workers).

Public evidence: [corpus summary](../reports/phase11b_corpus.json),
[export manifest](../reports/phase11b_export.json).

## Training

Full LeRobot ACT, from scratch except the ImageNet ResNet-18 backbone:

- 51,565,446 parameters; 512-dim, 4 encoder / 1 decoder layer, VAE, 30-step
  chunks, dropout 0.1.
- AdamW: lr 1e-5, weight decay 1e-4, batch 4, 8 CPU threads, FP32.
- 3,000 steps per seed = 12,000 samples (0.88 epoch of the training split).
- Three seeds (1, 2, 3); identical data, split and schedule.
- Throughput **7.40–7.41 samples/s**; training wall **1,620 s (~27 min)** per
  seed.
- Peak RSS **4.70–4.71 GB**.
- Peak SoC temperature **76.6 / 77.1 / 77.5 °C**, below the owner-accepted
  79–80 °C envelope from Phase 11a; no throttle or guard trip.
- Checkpoint reload into a fresh model: **max absolute difference 0.0 rad** for
  all three seeds.

## Held-out result

L1 and MSE are over valid (non-padded) action steps, in radians². The
hold-position baseline repeats the current joint vector for the whole 30-step
chunk.

| Seed | Held-out L1 | Hold L1 | L1 improvement | Held-out MSE | Hold MSE | MSE improvement |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0.1486 | 0.1608 | **+7.6%** | 0.0624 | 0.0996 | **+37.3%** |
| 2 | 0.1332 | 0.1608 | **+17.2%** | 0.0578 | 0.0996 | **+42.0%** |
| 3 | 0.1574 | 0.1608 | **+2.1%** | 0.0672 | 0.0996 | **+32.5%** |

Mean L1 improvement **9.0%** (population SD 6.2 percentage points); mean MSE
improvement **37.3%**. All three seeds improve in the same direction, which is
the stated stability gate; the spread is recorded rather than smoothed.

### Learning curve (fixed 1,024-sample held-out slice)

| Step | Seed 1 L1 (impr.) | Seed 2 L1 (impr.) | Seed 3 L1 (impr.) |
| ---: | ---: | ---: | ---: |
| 0 | 0.6028 (−284%) | 0.5202 (−220%) | 0.5672 (−246%) |
| 500 | 0.1724 (−9.8%) | 0.1859 (−14.3%) | 0.1869 (−14.0%) |
| 1,000 | 0.1582 (−0.7%) | 0.1693 (−4.1%) | 0.1834 (−11.9%) |
| 1,500 | 0.1574 (−0.2%) | 0.1619 (+0.4%) | 0.1505 (+8.2%) |
| 2,000 | 0.1443 (+8.1%) | 0.1419 (+12.7%) | 0.1520 (+7.3%) |
| 2,500 | 0.1368 (+12.9%) | 0.1439 (+11.5%) | 0.1640 (−0.0%) |
| 3,000 | 0.1466 (+6.6%) | 0.1318 (+18.9%) | 0.1605 (+2.1%) |

The curve is not monotone; 2,000 steps is the most consistent point across
seeds (+7.3% to +12.7%), while the declared 3,000-step endpoint is noisier.
The final table above is the declared endpoint on the full 4,536-sample test
split.

## Per-bucket result — the named shortfall

The aggregate improvement is **concentrated in capture/tray tasks**:

| Family | Seed 1 | Seed 2 | Seed 3 | Mean |
| --- | ---: | ---: | ---: | ---: |
| Capture | +23.3% | +28.1% | +19.7% | **+23.7%** |
| Move | −10.5% | +4.6% | −18.1% | **−8.0%** |

Every move bucket is worse than hold-position in seeds 1 and 3; seed 2 is
slightly better on most move buckets but not all. The aggregate L1 is dominated
by the larger capture motions, so it passes while quiet moves do not. Crowding
does not separate the result: sparse +7.8/+17.9/+3.1% and crowded
+7.4/+16.5/+1.1%.

This is the Phase 12 input, not a hidden exclusion. **Do not adopt this policy
for quiet moves.** Phase 12 must evaluate move and capture families separately
against scripted control, and may retain scripted control for moves. No task
rollout, grasp success, placement accuracy or physical behaviour is claimed
here.

Per-bucket JSON: `reports/phase11b_summary.json`,
`reports/phase11b_buckets_seed{1,2,3}.json`.

## Gate verdict

- **Training path:** local, reproducible, full-size ACT on BC-250 CPU. **Pass.**
- **Throughput / memory / thermal / reload:** measured and within the accepted
  envelope. **Pass.**
- **Held-out error improves on hold-position, all seeds:** L1 +7.6/+17.2/+2.1%,
  MSE +37.3/+42.0/+32.5%. **Pass in aggregate.**
- **Stability:** same sign in all three seeds; variance and the move-family
  shortfall are recorded. **Conditional pass with named limitation.**
- **Controller quality / task success:** **not claimed.** Phase 12 owns the
  paired comparison and the adopt/reject decision.

## Reproduction

```bash
cd ~/robot-arm-sim
ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=.private/gripper-assets-v5-tipthin \
ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS=.private/piece-assets-v9-brian-parallel-knight \
MUJOCO_GL=egl PYTHONPATH=$PWD .venv/bin/python tools/collect_phase11b.py \
  --out .private/phase11b/data --episodes 16 --workers 4
PYTHONPATH=$PWD .venv/bin/python tools/export_phase11b.py \
  --root .private/phase11b/data --out .private/phase11b/export --frames 24
PYTHONPATH=$PWD .venv/bin/python training/probes/phase11b_train.py \
  --source .private/lerobot-pinned/modeling_act.py \
  --dataset .private/phase11b/export/phase11b_224.npz \
  --out .private/phase11b/run_seed1 --seed 1 --steps 3000 --batch 4 \
  --eval-every 500 --eval-samples 1024 --threads 8
PYTHONPATH=$PWD .venv/bin/python training/probes/phase11b_eval.py \
  --source .private/lerobot-pinned/modeling_act.py \
  --dataset .private/phase11b/export/phase11b_224.npz \
  --checkpoint .private/phase11b/run_seed1/checkpoint.pt \
  --out .private/phase11b/run_seed1/buckets.json
```

The private corpus, checkpoints and raw shards stay under `.private/phase11b/`
and are not committed or uploaded.

Related: [ROADMAP](ROADMAP.md) · [Phase 11 runtime qualification](PHASE11_RUNTIME_QUALIFICATION.md) ·
[local training paths](PHASE11_LOCAL_TRAINING_PATHS.md) · [Phase 10](PHASE10_GRASP_CONTACTS.md).
