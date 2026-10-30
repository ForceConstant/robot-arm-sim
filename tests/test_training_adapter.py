"""Adapter-training flags: a full fine-tune cannot fit a 16 GB card.

These check command construction only - no torch, no dataset, no GPU.
"""
from chess_sim.training import RECIPES, TrainConfig


def config(**overrides):
    settings = dict(recipe=RECIPES["molmoact2"], dataset_root="datasets/lerobot/x",
                    output_dir="outputs/x", total_steps=20)
    settings.update(overrides)
    return TrainConfig(**settings)


def test_full_finetune_emits_no_adapter_flags():
    cmd = config().command(20)
    assert not any(flag.startswith("--peft.") for flag in cmd)
    assert "--mixed_precision=bf16" in cmd


def test_adapter_flags_are_emitted():
    cmd = config(peft=True, peft_r=32, peft_alpha=64.0).command(20)
    assert "--peft.method_type=LORA" in cmd
    assert "--peft.r=32" in cmd
    assert "--peft.lora_alpha=64" in cmd


def test_optional_adapter_flags_only_when_set():
    bare = config(peft=True).command(20)
    assert not any(f.startswith("--peft.target_modules") for f in bare)
    assert not any(f.startswith("--peft.full_training_modules") for f in bare)

    tuned = config(peft=True, peft_target_modules="all-linear",
                   peft_full_training_modules=("action_in_proj", "action_out_proj")).command(20)
    assert "--peft.target_modules=all-linear" in tuned
    assert "--peft.full_training_modules=[action_in_proj,action_out_proj]" in tuned


def test_resume_repeats_adapter_flags_and_precision():
    run = config(peft=True, peft_r=8, mixed_precision="fp16")
    cmd = run.resume_command(40)
    assert "--mixed_precision=fp16" in cmd
    assert "--peft.r=8" in cmd
    assert "--resume=true" in cmd


def test_mixed_precision_override_reaches_the_launcher():
    assert "--mixed_precision=no" in config(mixed_precision="no").command(20)
