"""Training-command plumbing for small cards.

MolmoAct2 already trains adapters through its own `--policy.train_mode_vlm`
(default `lora`); these tests check that the precision override and verbatim
policy flags reach the launcher, and that nothing invents a second adapter path.
"""
from chess_sim.training import RECIPES, TrainConfig


def config(**overrides):
    settings = dict(recipe=RECIPES["molmoact2"], dataset_root="datasets/lerobot/x",
                    output_dir="outputs/x", total_steps=20)
    settings.update(overrides)
    return TrainConfig(**settings)


def test_default_precision_is_bf16():
    assert "--mixed_precision=bf16" in config().command(20)


def test_precision_override_reaches_the_launcher():
    run = config(mixed_precision="fp16")
    assert "--mixed_precision=fp16" in run.command(20)
    assert "--mixed_precision=fp16" in run.resume_command(40)


def test_recipe_keeps_its_own_lora_mode():
    """The VLM adapter is the policy's flag, not a duplicate peft block."""
    cmd = config().command(20)
    assert "--policy.train_mode_vlm=lora" in cmd
    assert not any(flag.startswith("--peft.") for flag in cmd)


def test_verbatim_policy_flags_still_pass_through():
    """A lower LoRA rank is the supported memory lever for a 16 GB card."""
    cmd = config(extra_flags=("--policy.lora_rank=8",)).command(20)
    assert "--policy.lora_rank=8" in cmd
