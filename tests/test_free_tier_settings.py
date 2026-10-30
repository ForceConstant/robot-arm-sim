"""The free-tier pilot settings must describe an adapter run that can fit a T4."""
from pathlib import Path

SETTINGS = Path(__file__).resolve().parents[1] / "training" / "settings.free_tier_colab.env"


def values():
    out = {}
    for line in SETTINGS.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            key, _, value = line.partition("=")
            out[key.strip()] = value.strip()
    return out


def test_settings_exist_and_are_an_adapter_run():
    v = values()
    assert v["PEFT"] == "1"
    assert v["MIXED_PRECISION"] == "fp16"      # a T4 cannot compute bf16
    assert v["PEFT_R"].isdigit() and int(v["PEFT_R"]) > 0


def test_settings_are_a_bounded_pilot_not_a_long_run():
    v = values()
    assert int(v["TOTAL_STEPS"]) <= 100
    assert int(v["BLOCK"]) <= int(v["TOTAL_STEPS"])
    assert int(v["BATCH_SIZE"]) == 1
    assert int(v["GRAD_ACCUM"]) >= 1
    assert int(v["SAVE_EVERY"]) <= int(v["TOTAL_STEPS"])


def test_train_sh_forwards_the_adapter_flags():
    script = (SETTINGS.parent / "train.sh").read_text()
    for flag in ("--peft", "--peft-r", "--peft-alpha", "--peft-target-modules",
                 "--mixed-precision"):
        assert flag in script, flag
