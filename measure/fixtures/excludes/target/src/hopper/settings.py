"""Settings: config/default.toml, then the environment on top."""
import os
import tomllib
from pathlib import Path

DEFAULTS = Path(__file__).resolve().parents[2] / "config" / "default.toml"


def load() -> dict:
    cfg = tomllib.loads(DEFAULTS.read_text())
    for section, keys in cfg.items():
        for key in keys:
            env = f"HOPPER_{section}_{key}".upper()
            if env in os.environ:
                raw = os.environ[env]
                cur = keys[key]
                keys[key] = type(cur)(raw) if not isinstance(cur, bool) else raw == "1"
    return cfg
