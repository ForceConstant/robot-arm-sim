"""The free-tier pilot settings must describe a run that could fit a T4."""
from pathlib import Path

SETTINGS = Path(__file__).resolve().parents[1] / "training" / "settings.free_tier_colab.env"


def values():
    out = {}
    for line in SETTINGS.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            key, _, value = line.partition("=")
            out[key.strip()] = value.strip().strip('"')
    return out


def test_uses_a_reduced_rank_and_fp16():
    v = values()
    assert "--policy.lora_rank=" in v["POLICY_FLAGS"]
    assert v["MIXED_PRECISION"] == "fp16"      # a T4 cannot compute bf16


def test_does_not_repeat_a_flag_the_recipe_already_sets():
    """The recipe emits train_mode_vlm itself; repeating it duplicates the flag."""
    v = values()
    assert "train_mode_vlm" not in v["POLICY_FLAGS"]


def test_is_a_bounded_pilot_not_a_long_run():
    v = values()
    assert int(v["TOTAL_STEPS"]) <= 100
    assert int(v["BLOCK"]) <= int(v["TOTAL_STEPS"])
    assert int(v["BATCH_SIZE"]) == 1
    assert int(v["SAVE_EVERY"]) <= int(v["TOTAL_STEPS"])


def test_train_sh_forwards_precision_and_policy_flags():
    script = (SETTINGS.parent / "train.sh").read_text()
    assert "--mixed-precision" in script
    assert "--policy-flags" in script
