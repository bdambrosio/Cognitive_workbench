"""Sampling parameters, keyed by model. The single source of truth.

WHY THIS FILE EXISTS. On 2026-08-24 twelve benchmark runs were made at
temperature 0.7 and thrown away. Nothing was misconfigured: no scenario and no
model set a temperature, so every one of them inherited the code default, and
`run.py --help` reported that default as though it were a decision. The three
DeepInfra models are the sharp case — their yaml states the publisher's 1.0,
explains that the runner must pass it, and warns in as many words about "the
silent second variable this exists to avoid". All true, all in the file, none
of it executed.

**Documenting a setting in a file the runner does not read is not setting it.**

So the values live here, in code, on the path every call takes, and
`resolve_temperature` RAISES for a model it does not know. There is
deliberately no fallback: a model absent from the table cannot run at all,
which is what forces the conversation instead of producing a mislabelled row.

ADDING A MODEL IS NOT A CODE CHANGE YOU MAKE ALONE. Search for the
publisher's recommended temperature for agentic/tool-use work, and get
explicit confirmation before adding a line. Where no recommendation exists
(xAI publishes none for grok-4.6), say so and ask rather than picking.
Provenance for every value below is in `docs/model-settings.md`.
"""

from __future__ import annotations

from typing import Dict, List

# ---------------------------------------------------------------------------
# GLOBAL. Every model, every call site: runs, scoring, the grader, chat,
# subagents. Not a per-model setting and not overridable by convention — the
# grader's old `top_p=1.0` pin was removed rather than kept as an exception.
# Route 1 (Anthropic) omits top_p entirely because Opus 4.7 rejects it; that
# is a protocol limit, not an exemption, and no Anthropic model is currently
# configured.
# ---------------------------------------------------------------------------
TOP_P: float = 0.95

# ---------------------------------------------------------------------------
# PER-MODEL TEMPERATURE. Settled 2026-08-24. Sources in docs/model-settings.md.
#
# Keys are matched against the served / configured model id: exact first, then
# unique case-insensitive substring. `Qwen3.8-27B` is a substring key because
# the local server reports `Qwen/Qwen3.8-27B` while models declare
# `model: ""`. It is spelled out in full rather than as `Qwen3.8`, which would
# also match every cloud sibling of that family and hand each one a
# temperature nobody chose.
# ---------------------------------------------------------------------------
MODEL_TEMPERATURE: Dict[str, float] = {
    # Cloud, closed.
    "gpt-5.6-luna": 0.2,
    "gpt-5.6-terra": 0.1,      # the pinned grader; NOT exempt from TOP_P
    "grok-4.6": 0.5,           # xAI publishes no agentic recommendation
    # Local.
    #
    # NARROWED 2026-08-26 from the bare `Qwen3.8`. That key matched every
    # cloud sibling — qwen3.8-flash, qwen3.8-max — which then resolved to this
    # 0.25 and never raised. A temperature nobody chose, arriving through a
    # coincidence of spelling, is the exact failure this module exists to
    # prevent, and a matching key does not trip the guard the way a missing
    # one does. Qwen's own card recommends 1.0 for thinking mode, so it was
    # not a near miss either.
    #
    # Still a substring, so it covers both the id the local server reports
    # and the OpenRouter spelling of the same weights (`qwen/qwen3.8-27b`).
    "Qwen3.8-27B": 0.25,
    "gemma-4-31B": 0.25,
    # OSS cloud, at their publishers' agentic recommendation.
    #
    # KEYS ARE PROVIDER-SPELLING-AGNOSTIC ON PURPOSE. The same weights ship
    # under different ids depending on the route — DeepInfra serves
    # `nvidia/NVIDIA-Nemotron-3-Ultra-550B-A55B`, OpenRouter serves
    # `nvidia/nemotron-3-ultra-550b-a55b`. Keying on the full id meant an
    # model that had a temperature under one route had none under another,
    # which the guard test caught on the first run.
    "DeepSeek-V4-Flash": 1.0,
    "nemotron-3-ultra": 1.0,
    "nemotron-3-super": 1.0,
    # MiniMax publishes 1.0 / top_p 0.95 as general guidance rather than as a
    # benchmark config. top_p is not recorded here — theirs happens to equal
    # the global, and TOP_P stays a single value with no per-model override.
    "minimax-m3": 1.0,
    # NO PUBLISHER RECOMMENDATION EXISTS for this one. The card gives only
    # per-benchmark configurations and they disagree (1.0/0.95 HLE-with-tools,
    # 1.0/1.0 NL2Repo and Terminal-Bench, 0.95/1.0 DeepSWE). Bruce chose the
    # 1.0 / top_p 0.95 pair on 2026-08-26 — the same category of decision as
    # grok-4.6's 0.5, not a publisher value.
    #
    # The key must stay `glm-5.3-flash` and not shorten to `glm-5.3`: a
    # `glm-5.3` key would ALSO match this model id, and two matching keys with
    # different values raise. Adding the full GLM-5.3 later means keeping both
    # keys spelled out.
    "glm-5.3-flash": 1.0,
    # Qwen3.8-Flash on OpenRouter (`qwen/qwen3.8-flash`). Bruce confirmed 1.0
    # on 2026-08-28, for BOTH the reasoning-on and reasoning-off arms.
    #
    # PROVENANCE IS WEAKER THAN THE OTHER OSS ROWS AND THE DIFFERENCE MATTERS.
    # Qwen publishes 1.0 (thinking) and 0.7 (non-thinking) on the
    # Qwen3.8-Flash-Next open-weights card. It is NOT established that the
    # hosted route serves those weights: OpenRouter's entry declares
    # image+video input and the card describes a text stack, and no
    # recommendation for the hosted model was found. So 1.0 is the publisher's
    # number for a model that is probably, not provably, this one.
    #
    # WHY ONE VALUE ACROSS BOTH ARMS rather than the published pair. The arms
    # exist to measure what turning reasoning off costs. Running them at 1.0
    # and 0.7 would vary two things at once and attribute nothing. The
    # reasoning-off arm therefore runs above its recommended 0.7, and that is
    # a deliberate trade recorded in docs/model-settings.md.
    #
    # The key must stay `qwen3.8-flash` and not shorten to `qwen3.8`: that
    # would also match Qwen3.8-27B and qwen3.8-max, which is the exact
    # coincidence-of-spelling failure the 2026-08-26 narrowing above fixed.
    # NARROWED 2026-08-28 from the bare `qwen3.8-flash`, which is a substring
    # of the local `primitive-ai/Qwen3.8-Flash-Next-NVFP4` below and silently
    # captured it. Both wanted 1.0, so nothing raised — which is the worse
    # failure: it would have worked by accident until the day the two models
    # wanted different values. Spelled with the `qwen/` prefix so it matches
    # the OpenRouter id and nothing else.
    "qwen/qwen3.8-flash": 1.0,
    # Qwen3.8-Flash-Next, run locally in vLLM as primitive-ai's NVFP4 repack.
    # Bruce confirmed 1.0 on 2026-08-28.
    #
    # THIS IS THE ONE ROW WHOSE PUBLISHER RECOMMENDATION ACTUALLY APPLIES TO
    # THE MODEL IT NAMES. Qwen's Qwen3.8-Flash-Next card publishes 1.0 /
    # top_p 0.95 / top_k 20 for thinking mode, and these are those weights.
    # The hosted `qwen/qwen3.8-flash` row borrows the same card without it
    # being established that the two serve the same artifact; this one does
    # not have to borrow. The quantization is a third party's, the weights
    # are Qwen's.
    #
    # 1.0 is the THINKING-mode value, which is the mode we run: the audit
    # yaml sets reasoning_effort low, not off. Qwen publishes 0.7 for
    # non-thinking, so a config that ever turns thinking off is at the wrong
    # operating point and needs its own conversation, not a silent reuse of
    # this number.
    "qwen3.8-flash-next": 1.0,
}

# Retired deliberately, so a stale config naming one fails with a reason
# rather than a bare "unknown model".
RETIRED: Dict[str, str] = {
    "Qwen/Qwen3.8-2.4T-A95B":
        "dropped 2026-08-24: 2,902 completion tokens for a one-line answer; "
        "too slow for a call-heavy agent",
    "claude-opus-4-7":   "dropped 2026-08-24; revisit at Sonnet 5",
    "claude-sonnet-4-6": "dropped 2026-08-24; revisit at Sonnet 5",
    "mimo-v2.5":         "dropped 2026-08-24",
}


class UnknownModel(RuntimeError):
    """Raised for a model with no configured temperature.

    Deliberately fatal. The alternative — falling back to some default — is
    exactly the failure this module was written after.
    """


def _candidates(model_id: str) -> List[str]:
    m = (model_id or "").strip()
    if not m:
        return []
    if m in MODEL_TEMPERATURE:
        return [m]
    low = m.lower()
    # Longest key first so a specific key beats a general one.
    return [k for k in sorted(MODEL_TEMPERATURE, key=len, reverse=True)
            if k.lower() in low]


def resolve_temperature(model_id: str) -> float:
    """Temperature for `model_id`. Raises UnknownModel if it is not configured.

    `model_id` must be a real identity — the configured model for a cloud
    endpoint, or the id the local server reports for `model: ""`. Passing the
    empty string is itself an error: "whatever is being served" is not a model
    and cannot carry a setting.
    """
    m = (model_id or "").strip()
    if not m:
        raise UnknownModel(
            "no model id: a local config with model:\"\" must be resolved "
            "against the server's /v1/models before a temperature can be "
            "chosen. 'Whatever is served' is not a model.")
    for retired, why in RETIRED.items():
        if retired.lower() in m.lower():
            raise UnknownModel(f"{m!r} is retired — {why}")
    hits = _candidates(m)
    if not hits:
        raise UnknownModel(
            f"no temperature configured for {m!r}. Adding one requires the "
            f"publisher's recommended agentic temperature and Bruce's "
            f"confirmation — see src/chat/model_params.py and "
            f"docs/model-settings.md. Known: {sorted(MODEL_TEMPERATURE)}")
    if len(hits) > 1 and len({MODEL_TEMPERATURE[h] for h in hits}) > 1:
        raise UnknownModel(
            f"{m!r} matches several keys with different temperatures "
            f"({ {h: MODEL_TEMPERATURE[h] for h in hits} }). Make the key "
            f"unambiguous rather than letting match order decide.")
    return MODEL_TEMPERATURE[hits[0]]


def is_configured(model_id: str) -> bool:
    """True if `model_id` resolves. For tests and pre-flight checks."""
    try:
        resolve_temperature(model_id)
        return True
    except UnknownModel:
        return False
