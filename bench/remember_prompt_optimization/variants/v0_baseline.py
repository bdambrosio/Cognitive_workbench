"""v0_baseline — current production prompt verbatim.

Re-exports `chat.remember._build_system_prompt` as `build` so the harness
treats it like any other variant. Reference number for comparing
v1+ variants.
"""

from chat.remember import _build_system_prompt as build

__all__ = ["build"]
