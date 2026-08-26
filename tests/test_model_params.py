"""Every model and scenario must name a model with a configured temperature.

This is the guard that catches the failure of 2026-08-24: an model added without
a sampling setting, silently inheriting a default nobody chose. Runs offline —
no GPU, no network, no API key.
"""

from __future__ import annotations

import glob
import os
import sys

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src"))

from chat.model_params import (  # noqa: E402
    MODEL_TEMPERATURE, RETIRED, TOP_P, UnknownModel, is_configured,
    resolve_temperature)


def _arms():
    return sorted(glob.glob(os.path.join(REPO, "measure", "models", "*.yaml")))


def _scenario_models():
    """(file, where, model) for every llm_config that names a model."""
    out = []
    for f in sorted(glob.glob(os.path.join(REPO, "scenarios", "*.yaml"))):
        try:
            d = yaml.safe_load(open(f, encoding="utf-8")) or {}
        except yaml.YAMLError:
            continue                       # not this test's job to police syntax
        blocks = []
        if isinstance(d.get("llm_config"), dict):
            blocks.append(("top-level", d["llm_config"]))
        for cn, cc in (d.get("characters") or {}).items():
            if isinstance(cc, dict) and isinstance(cc.get("llm_config"), dict):
                blocks.append((cn, cc["llm_config"]))
        for where, lc in blocks:
            m = (lc.get("model") or "").strip()
            if m:                          # model:"" resolves at runtime
                out.append((os.path.basename(f), where, m))
    return out


def test_top_p_is_the_house_value():
    assert TOP_P == 0.95


@pytest.mark.parametrize("path", _arms(), ids=os.path.basename)
def test_every_arm_resolves(path):
    """An model names a model with a temperature, or declares expects_served_model.

    A local model legitimately says model:"" — it takes whatever is served — but
    it must then declare what it expects, so the row can name its own backend
    and the served id can be checked against the table.
    """
    d = yaml.safe_load(open(path, encoding="utf-8")) or {}
    lc = d.get("llm_config") or {}
    model = (lc.get("model") or "").strip()
    expects = (d.get("expects_served_model") or "").strip()

    if not model:
        assert expects, (
            f"{os.path.basename(path)}: declares model:\"\" and no "
            f"expects_served_model, so nothing can say which model answers "
            f"or what temperature it should use.")
        model = expects

    assert is_configured(model), (
        f"{os.path.basename(path)}: no temperature configured for {model!r}. "
        f"See docs/model-settings.md — adding one needs the publisher's "
        f"agentic recommendation and explicit confirmation.")


@pytest.mark.parametrize("f,where,model", _scenario_models(),
                         ids=lambda v: str(v)[:40])
def test_every_scenario_model_resolves(f, where, model):
    assert is_configured(model), (
        f"{f} [{where}]: no temperature configured for {model!r}.")


def test_unknown_model_raises_rather_than_defaulting():
    """The whole point. A fallback here is the bug this module was written after."""
    with pytest.raises(UnknownModel):
        resolve_temperature("some-model-nobody-configured")


def test_empty_model_is_an_error_not_a_wildcard():
    with pytest.raises(UnknownModel):
        resolve_temperature("")


@pytest.mark.parametrize("retired", sorted(RETIRED))
def test_retired_models_fail_with_a_reason(retired):
    with pytest.raises(UnknownModel) as e:
        resolve_temperature(retired)
    assert "retired" in str(e.value)


def test_local_qwen_resolves_from_the_served_id():
    """The server reports Qwen/Qwen3.8-27B; the table key is the substring."""
    assert resolve_temperature("Qwen/Qwen3.8-27B") == 0.25


def test_no_key_is_a_prefix_of_another_with_a_different_value():
    """Substring matching is only safe while this holds."""
    for a in MODEL_TEMPERATURE:
        for b in MODEL_TEMPERATURE:
            if a != b and a.lower() in b.lower():
                assert MODEL_TEMPERATURE[a] == MODEL_TEMPERATURE[b], (
                    f"{a!r} is a substring of {b!r} with a different "
                    f"temperature; match order would decide the value.")
