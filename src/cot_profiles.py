"""
Structured Chain-of-Thought (CoT) profile registry.

GBNF grammars that constrain the <think>…</think> block of reasoning models
per call site, keeping deliberation tight while leaving (most) answer
channels free.

Background: https://andthattoo.dev/blog/structured_cot — Qwen3.6-class
reasoning models over-deliberate by default; constraining the <think>
block via guided sampling cuts thinking tokens 22–43× with neutral or
positive accuracy effect. Per-domain shapes work better than a single
universal skeleton, so each call site picks a profile.

Usage from a backend driver:

    from cot_profiles import resolve_profile, is_reasoning_model

    if is_reasoning_model(model_name):
        grammar = resolve_profile(profile_name)   # may be None for 'none'
        if grammar:
            body['grammar'] = grammar             # llama.cpp / llama-server
            # body.setdefault('extra_body', {})['ebnf'] = grammar  # SGLang
"""

from typing import List, Optional


# ---------------------------------------------------------------------------
# Model gate
# ---------------------------------------------------------------------------

# Substrings matched (case-insensitive) against the model identifier. Add
# new reasoning-model families here. Scenarios may also force this on/off
# via llm_config.is_reasoning_model.
_REASONING_PATTERNS = (
    'qwen3.6',
    'qwen3-thinking',
    'qwen3-235b',
    'deepseek-r1',
)


def is_reasoning_model(model_name: Optional[str]) -> bool:
    """Return True if the model is known to emit <think>…</think> blocks."""
    if not model_name:
        return False
    name = str(model_name).lower()
    return any(p in name for p in _REASONING_PATTERNS)


# ---------------------------------------------------------------------------
# Grammar fragments (GBNF, llama.cpp dialect)
# ---------------------------------------------------------------------------
#
# Notes on the JSON sub-grammar: adapted from llama.cpp's stock json.gbnf
# (grammars/json.gbnf). Permissive enough for arbitrary JSON objects while
# still preventing prose-where-JSON-is-required.
#
# IMPORTANT — llama.cpp GBNF identifier syntax is [a-z][a-z0-9-]*. UNDERSCORES
# ARE NOT ACCEPTED. Earlier names (j_ws, j_object, think_body) silently failed
# to parse and llama-server dropped the grammar entirely without rejecting the
# request. Stick to lowercase letters, digits, and hyphens.

# Permissive answer rule: tab, newline, CR, printable ASCII, plus all
# bytes 0x80-0xFF so UTF-8 continuation/lead bytes pass through. Without
# this, replies containing em-dash, smart quotes, non-English characters,
# or emoji deadlock the constrained sampler.
_FREE_ANSWER_RULE = r'answer ::= [\x09\x0A\x0D\x20-\x7E\x80-\xFF]+' + '\n'

# Byte-budget cap for the <think> interior. The body is any non-`<`
# byte (newlines and UTF-8 included), repeated up to the per-profile
# limit; once the cap is hit, the constrained sampler can only emit
# `</think>`. This bounds reasoning *length* without dictating its
# *shape* — the previous label-skeleton approach (one labeled line per
# field) deadlocked Qwen3.6 because the model wanted to think
# multi-paragraph and the no-newline rule forced it to spiral until
# max_tokens.
#
# Byte → token rough conversion: ~4 bytes/token for English BPE, so
# 256 bytes ≈ 64 tokens, 800 bytes ≈ 200 tokens. UTF-8 multi-byte
# characters cost more bytes per token; budgets are bytes, not tokens.
def _think_body_rule(max_bytes: int) -> str:
    return f'tbody ::= [^<]{{0,{max_bytes}}}\n'

_JSON_RULES = r"""object ::= "{" jws ( pair ( jws "," jws pair )* )? jws "}" jws
pair   ::= string jws ":" jws value
value  ::= object | array | string | number | ("true" | "false" | "null") jws
array  ::= "[" jws ( value ( jws "," jws value )* )? jws "]" jws
string ::= "\"" ( [^"\\\x00-\x1F] | "\\" (["\\/bfnrt] | "u" [0-9a-fA-F][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F]) )* "\"" jws
number ::= ("-"? ([0-9] | [1-9] [0-9]*)) ("." [0-9]+)? ([eE] [-+]? [0-9]+)? jws
jws    ::= ([ \t\n])*
"""


def _profile_freeform(max_body_bytes: int) -> str:
    """Length-bounded thinking, then </think>, then freeform answer.

    Note: grammar does NOT include the opening "<think>" literal. Qwen3.6
    via llama.cpp --jinja auto-prefixes the assistant turn with `<think>\\n`
    in the chat template, so model output starts INSIDE the think block.
    The grammar bounds the thinking body, requires `</think>` to close the
    template-opened block, then captures the answer.
    """
    return (
        'root  ::= tbody "</think>" answer\n'
        + _think_body_rule(max_body_bytes)
        + _FREE_ANSWER_RULE
    )


def _profile_json(max_body_bytes: int) -> str:
    """Length-bounded thinking, then </think>, then a JSON object answer."""
    return (
        'root  ::= tbody "</think>" jws object\n'
        + _think_body_rule(max_body_bytes)
        + _JSON_RULES
    )


# ---------------------------------------------------------------------------
# Public registry
# ---------------------------------------------------------------------------
#
# Per-profile thinking budget (bytes inside <think>…</think>). Tuned
# loosely to call-site complexity — chat replies need almost no
# deliberation, discourse extraction does. Adjust based on observed
# behavior; the gate refuses to forward grammar at all when the model
# isn't classed as a reasoning model.

PROFILES = {
    'default':       _profile_freeform(max_body_bytes=400),   # ~100 tokens
    'chat_reply':    _profile_freeform(max_body_bytes=256),   # ~64 tokens — minimal
    'extract':       _profile_freeform(max_body_bytes=800),   # ~200 tokens
    'revise_belief': _profile_freeform(max_body_bytes=800),   # ~200 tokens
    'plan':          _profile_freeform(max_body_bytes=600),   # ~150 tokens
    'triage':        _profile_json(max_body_bytes=200),       # ~50 tokens then JSON
    'none':          None,
}


def resolve_profile(profile: Optional[str]) -> Optional[str]:
    """Return the GBNF grammar for a profile name.

    - None / unset           → 'default'
    - 'none'                 → None (caller skips grammar attachment)
    - unknown name           → 'default' (with no warning; callers may log)
    """
    if profile is None:
        profile = 'default'
    if profile == 'none':
        return None
    return PROFILES.get(profile, PROFILES['default'])


def list_profiles() -> List[str]:
    return sorted(PROFILES.keys())
