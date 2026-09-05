"""Configuration: config/default.toml, then the environment on top."""
import os
import tomllib
from pathlib import Path

DEFAULTS = Path(__file__).resolve().parent.parent / "config" / "default.toml"


def load_config() -> dict:
    cfg = tomllib.loads(DEFAULTS.read_text())
    if "HOPPER_RATE_LIMIT" in os.environ:
        cfg["rate_limit"] = os.environ["HOPPER_RATE_LIMIT"] == "1"
    return cfg
