"""
ChatLoop — lightweight per-character chat agent.

Mirrors ZenohExecutiveNode's user-facing wire protocol so existing clients
(cli.py) work unchanged:

    in:  cognitive/{character}/sense_data
    out: cognitive/{character}/action       (type="say")

Each turn:
    1. record_incoming   → ConversationStore
    2. orientation pass  → character_evaluator.evaluate (concerns/goals empty)
    3. LLM reply         → OpenAI-compatible chat/completions
    4. record_outgoing   → ConversationStore
    5. publish /action
    6. discourse update  → DiscourseTracker (analyze_segment + companion)
"""

from __future__ import annotations

import json
import logging
import os
import queue
import re
import sys
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

import requests


# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from cot_profiles import is_reasoning_model, resolve_profile  # noqa: E402
from affect.publisher import AffectPublisher  # noqa: E402
from canvas.publisher import CanvasPublisher, default_key as canvas_default_key  # noqa: E402
from utils.json_utils import repair_json_string  # noqa: E402


logger = logging.getLogger('chat_loop')


# ReAct loop: maximum iterations per turn before forcing a fallback
# synthesis. Empirically (263 chat turns audited 2026-05-03) p95 is 2
# iters and the cap had never been hit — raised 8→12 to add headroom
# for hypothetical multi-tool sequences and more ambitious autonomous
# concerns without introducing room for runaway loops to wander. 12
# also matches inspect's inner cap, removing the asymmetry where the
# outer loop had fewer iters than each subagent it could invoke.
REACT_MAX_ITERS = 12

# Concern-instruction authoring rubric. Each fired concern dispatches
# one ReAct loop capped at REACT_MAX_ITERS — an instruction that can't
# be completed in that budget hits the max-iters fallback (which spawns
# a successor concern). The fallback exists as a safety net, not a
# crutch: authoring should default to narrow, single-step instructions.
# This canonical wording is referenced from the reflection prompt so
# the rubric and the budget stay in sync.
_CONCERN_INSTRUCTION_NARROWNESS_RULE = (
    f"Each `instruction` must be narrow enough to complete in roughly "
    f"{REACT_MAX_ITERS} ReAct iterations (one tool call per iter, plus a "
    f"final respond). Write single concrete actions (\"look up X and "
    f"summarize the result\"), not omnibus directives (\"investigate the "
    f"entire topic of Y\"). If the natural work is broader, pick the "
    f"most-immediate slice — successor concerns can carry the rest."
)

# Tools the model can emit. Validated structurally in _parse_react_action;
# the dispatcher in _run_react_loop knows how to run each.
_REACT_TOOLS = ('process_text', 'search', 'fetch_text', 'recall', 'inspect', 'inspect_external', 'security', 'display', 'respond')

# Per-character collection holding episodic specifics across conversations.
# Auto-RAG searches this on every turn; post-turn reflection writes to it.
_MEMORIES_COLLECTION_NAME = "memories"

# Memory subtype, stored on the note's `properties.category`. Distinct from
# the existing `properties.kind="memory"` discriminator (which says what
# class of note this is — memory vs companion_state vs web_search etc).
# Reflection picks one per memory; default is `fact`. Unknown values from
# the model are coerced to `fact` rather than dropped.
_MEMORY_CATEGORIES = ('fact', 'preference', 'commitment')

# Frame values the reflection LLM may return. Only `none` permits writes.
# Anything else (or unparseable / missing) suppresses the turn's memories
# entirely — the asymmetric cost of a poisoned memory is worse than
# missing one we'll re-encounter.
_REFLECT_FRAME_OK = 'none'

# ===== Concerns: two collections, asymmetric dynamics =====
#
# Concerns split into two collections that share infrastructure but
# carry different semantics and different update rhythms.
#
#   user_concerns: "what the user cares about." Models the user's current
#       preoccupations. Strength decays each user turn (recall-driven —
#       if it doesn't come up, we stop tracking it). Bumped when user
#       input semantic-matches the concern. Pruned below threshold.
#       Surfaces in the prompt adjacent to companion. Does NOT fire
#       autonomously — its job is to shape Jill's responses to requests
#       by being recalled when relevant.
#
#   agent_concerns: "what I (Jill) want to act on." Pressure-driven.
#       Activation grows each tick at a rate derived from rhythm_hours;
#       decremented on service (autonomous fire). Fires when activation
#       >= threshold AND the concern carries an instruction. Seeds live
#       here as architectural baseline (seed=True) and source derived
#       agent_concerns via reflection.
#
# The asymmetry — strength decays, activation grows — is the real point:
# user models are recall-driven (silence implies disinterest); agent action
# is pressure-driven (silence implies untended work).
_AGENT_CONCERNS_COLLECTION_NAME = "agent_concerns"
_USER_CONCERNS_COLLECTION_NAME  = "user_concerns"
_AGENT_THREADS_COLLECTION_NAME  = "agent_threads"
_CONCERN_STATUSES = ('active', 'satisfied', 'abandoned')

# ----- user_concerns dynamics -----
# strength ∈ [0, 1]. Decay applied at user-turn entry. Bump applied
# when input similarity >= bump threshold. Pruned below prune threshold.
_USER_CONCERN_DECAY_PER_TURN  = 0.05   # strength lost per user turn
_USER_CONCERN_BUMP_THRESHOLD  = 0.5    # similarity ≥ this counts as a hit
_USER_CONCERN_BUMP_AMOUNT     = 0.30   # gained per hit (capped at 1.0)
_USER_CONCERN_PRUNE_THRESHOLD = 0.10   # delete below this
_USER_CONCERN_PROMPT_BUDGET   = 5      # top-K by strength surfaced

# ----- agent_concerns dynamics -----
# activation ∈ [0, 1]. Grows each tick proportional to elapsed wall-
# clock time. Decremented on service per exit_reason. Fires when
# activation ≥ fire threshold AND instruction is non-null AND status
# is active. Floored at 0; capped at 1.
_AGENT_CONCERN_FIRE_THRESHOLD   = 0.70 # activation ≥ this allows fire
_AGENT_CONCERN_SERVICE_FULL     = 0.60 # decrement on respond exit
_AGENT_CONCERN_SERVICE_PARTIAL  = 0.25 # decrement on max_iters exit
_AGENT_CONCERN_PROMPT_BUDGET    = 5    # top-M by activation surfaced

# rhythm_hours: declared target fire interval. Used at concern creation
# to derive activation growth-per-elapsed-hour:
#   growth_per_hour = (FIRE_THRESHOLD - POST_SERVICE_FLOOR) / rhythm_hours
# where POST_SERVICE_FLOOR ≈ FIRE_THRESHOLD - SERVICE_FULL.
# A concern with rhythm_hours=24 fires roughly daily after full service.
#
# Allowlist matches the legacy cadence values. Default is weekly (168h)
# — chosen to err toward not firing too much when reflection lacks
# rhythm signal (typical for seed-derived self-orientation concerns).
# rhythm_source provenance ('external'|'urgency'|'default') is recorded
# on the note so we can audit how often the default fires versus
# reflection-extracted rhythms.
_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED = (1, 2, 4, 8, 12, 24, 168)
_AGENT_CONCERN_DEFAULT_RHYTHM_HOURS = 168    # weekly default

# Successor-concern depth cap. When a fired concern's ReAct loop hits
# REACT_MAX_ITERS, the fallback path can spawn a successor concern with
# the narrowed remainder of the work. To prevent slow-motion infinite
# loops, depth is capped — a chain of original → successor → successor
# is the most we'll ever produce automatically. Beyond that, the fallback
# response just stands and the user can re-engage manually.
_CONCERN_SUCCESSOR_MAX_DEPTH = 2
# Similarity threshold for recurrence detection on creation. A candidate
# concern whose top match in its collection exceeds this value is treated
# as the same concern: we refresh / promote the existing note rather
# than create a near-duplicate. Tuned high vs the 0.5 used for surfacing
# recall because a false merge silently loses specificity while a missed
# merge just creates a sibling that can be merged manually.
_CONCERN_RECURRENCE_THRESHOLD = 0.8


# --- Legacy aliases (in-flight rename; will be removed once dynamics rewrite lands)
_CONCERN_CATEGORIES = ('one_shot', 'durable', 'derived')
_CONCERN_CADENCE_HOURS_ALLOWED = _AGENT_CONCERN_RHYTHM_HOURS_ALLOWED
_CONCERN_DEFAULT_CADENCE_HOURS = {'one_shot': 1, 'derived': 24, 'durable': 24}
_CONCERN_DEFAULT_LIFETIME_DAYS = {'one_shot': 0.5, 'derived': 2.0, 'durable': 120.0}
_CONCERN_SATISFIED_THRESHOLD = 0.1
_CONCERN_LIFETIME_MIN_DAYS, _CONCERN_LIFETIME_MAX_DAYS = 0.1, 3650.0
_CONCERN_ALWAYS_ON_BUDGET = _AGENT_CONCERN_PROMPT_BUDGET


def _agent_concern_growth_for_elapsed(rhythm_hours: float, elapsed_hours: float) -> float:
    """Activation growth proportional to elapsed wall-clock hours.

    A concern with rhythm_hours=24 (daily target) grows from POST_SERVICE_FLOOR
    to FIRE_THRESHOLD over 24 elapsed hours. Independent of tick frequency —
    uses real elapsed time, so changing the tick schedule doesn't shift the
    firing rhythm.
    """
    if rhythm_hours <= 0 or elapsed_hours <= 0:
        return 0.0
    floor = max(0.0, _AGENT_CONCERN_FIRE_THRESHOLD - _AGENT_CONCERN_SERVICE_FULL)
    span = _AGENT_CONCERN_FIRE_THRESHOLD - floor
    return span * (elapsed_hours / float(rhythm_hours))


def _snap_rhythm_hours(value) -> int:
    """Snap a rhythm_hours value to the nearest allowed bucket. Returns
    the default when value is None / unparseable / out of range."""
    try:
        v = float(value) if value is not None else None
    except (TypeError, ValueError):
        v = None
    if v is None or v <= 0:
        return _AGENT_CONCERN_DEFAULT_RHYTHM_HOURS
    return min(_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED, key=lambda b: abs(b - v))
# Trim observation text from fetch_text before it lands in the working log.
# Bigger than search synthesis (which is already digested) since fetch_text
# is raw page text — but bounded so a single fetch can't blow the prompt.
_FETCH_TEXT_OBS_CAP = 8000

# Reasoning history (awareness feed): per-turn ReAct trace persisted as a
# Note in the `reasoning_history` collection. Last N entries surface in the
# user-message prefix between conversation history and current input. Most
# recent _REASONING_HISTORY_FULL render in full; older ones render as a
# compressed action-sequence digest. Ring-bounded — older traces beyond
# _REASONING_HISTORY_RING_SIZE are pruned at write time.
_REASONING_HISTORY_COLLECTION_NAME = "reasoning_history"
_REASONING_HISTORY_RING_SIZE = 50    # on-disk cap
_REASONING_HISTORY_RECENT = 6        # surfaced in the prompt
_REASONING_HISTORY_FULL = 3          # of those, how many in full vs compressed
_REASONING_HISTORY_OBS_CAP = 1000    # per-iter observation cap in stored trace

# Per-iteration auto-binding: $step1, $step2, ... names the result of each
# action so subsequent actions can reference it. Scoped to the current turn
# only — does not leak into conversation history or the next turn's loop.
_REACT_BINDING_RE = re.compile(r'^\$step\d+$')


# ─── LLM backend ────────────────────────────────────────────────────────────

class _ChatBackend:
    """Thin OpenAI-compatible chat client with structured-CoT support.

    Routes (checked in order):
      1. server == 'anthropic' → POST {base_url}/v1/messages with
         `x-api-key` + `anthropic-version` headers. system is a top-level
         string field; messages contains only user/assistant. temperature
         and top_p are omitted (Opus 4.7 rejects them; defaults are fine
         for other Claude models). stops → stop_sequences. Requires api_key.
      2. `api_key` set → unified OpenAI-compat path. POST to
         {base_url}/v1/chat/completions with `Authorization: Bearer
         <env[api_key]>`. Grammar / chat_template_kwargs are NOT attached
         (cloud endpoints reject them). The api_key field is the NAME of
         an environment variable, not the key itself.
      3. server in ('openrouter', 'openai') → utils.llm_api.LLM (legacy
         cloud shortcut, kept for back-compat).
      4. anything else (local, vllm, llama.cpp, sglang_api_server, lmstudio,
         unset) → POST {base_url}/v1/chat/completions directly, no auth.
         SGLangAPIServer accepts both `grammar` (GBNF, forwarded as ebnf)
         and `ebnf`; llama-server accepts `grammar`.

    `cot_profile` selects a GBNF skeleton for the <think> block. Attached
    only when the model gates as reasoning (auto-detected by name or
    forced via `is_reasoning=True`) AND we're on path 3 (no api_key).
    """

    def __init__(self, server: str, model: str, base_url: str,
                 is_reasoning: Optional[bool] = None,
                 api_key: Optional[str] = None,
                 reasoning_effort: Optional[str] = None):
        self.server = (server or 'local').lower()
        self.model = model or ''
        self.base_url = (base_url or 'http://127.0.0.1:5000').rstrip('/')
        if self.base_url.endswith('/v1'):
            self.base_url = self.base_url[:-3]
        # OpenAI-style reasoning effort dial. gpt-oss harmony serving
        # accepts "low"|"medium"|"high"; vLLM with the gpt-oss reasoning
        # parser respects the field. Forwarded only on the local OAI path
        # below — Anthropic and the legacy cloud_llm route ignore it.
        eff = (reasoning_effort or '').strip().lower() or None
        if eff is not None and eff not in ('low', 'medium', 'high'):
            raise RuntimeError(
                f"_ChatBackend: reasoning_effort must be low|medium|high, "
                f"got {reasoning_effort!r}")
        self.reasoning_effort = eff
        # New unified path: api_key field is the NAME of an env var. We
        # resolve it once at init so a missing env var fails loudly here
        # (with the var name) instead of silently 401-ing on first call.
        self._api_key_value: Optional[str] = None
        self._api_key_var: Optional[str] = None
        if api_key:
            api_key = str(api_key).strip()
            if api_key:
                self._api_key_var = api_key
                resolved = os.environ.get(api_key)
                if not resolved:
                    raise RuntimeError(
                        f"_ChatBackend: api_key env var {api_key!r} is not set. "
                        f"Either export {api_key} or remove the api_key field.")
                self._api_key_value = resolved
        # Anthropic native route requires api_key. Validate up front so
        # the failure is loud at scenario load instead of on first turn.
        if self.server == 'anthropic' and self._api_key_value is None:
            raise RuntimeError(
                "_ChatBackend: server='anthropic' requires api_key "
                "(name of env var holding the Anthropic API key).")

        # Legacy cloud shortcut — only used when api_key is NOT set, so
        # the new unified path takes precedence for any config that
        # specifies api_key (even if server happens to be openrouter/openai).
        self._cloud_llm = None
        if self._api_key_value is None and self.server in ('openrouter', 'openai'):
            from utils.llm_api import LLM
            self._cloud_llm = LLM(server_name=self.server, model_name=self.model)
        # Cloud routing flag, exposed for callers that need to vary
        # behavior by backend kind (e.g. ChatLoop's per-profile token
        # floors, which must be looser on local engines that lump
        # reasoning + visible output into one budget).
        self.is_cloud = (self._api_key_value is not None
                         or self._cloud_llm is not None
                         or self.server == 'anthropic')
        if is_reasoning is None:
            self.is_reasoning = is_reasoning_model(self.model)
        else:
            self.is_reasoning = bool(is_reasoning)

    @property
    def supports_image_input(self) -> bool:
        """True when the active route can carry image_url content parts.
        Route 4 (unified OpenAI-compat: vLLM, llama-server, xAI, OpenAI
        with api_key) handles them natively. Anthropic uses a different
        content-block shape; the legacy cloud_llm path may not handle
        list content. Both rejected for v1."""
        if self.server == 'anthropic':
            return False
        if self._cloud_llm is not None:
            return False
        return True

    def chat(self, messages: List[Dict[str, Any]],
             max_tokens: int = 600,
             temperature: float = 0.7,
             top_p: float = 1.0,
             stops: Optional[List[str]] = None,
             is_json: bool = False,
             cot_profile: Optional[str] = None,
             enable_thinking: Optional[bool] = None,
             reasoning_effort: Optional[str] = None) -> str:
        # Per-call reasoning_effort override. Only takes effect when the
        # scenario already declared a baseline (self.reasoning_effort is
        # not None) — the field is a reasoning-model concept; we don't
        # send it to non-reasoning backends. Validation matches __init__.
        if reasoning_effort is not None:
            eff = str(reasoning_effort).strip().lower()
            if eff not in ('low', 'medium', 'high'):
                raise RuntimeError(
                    f"_ChatBackend.chat: reasoning_effort must be "
                    f"low|medium|high, got {reasoning_effort!r}")
            reasoning_effort = eff
        stops = stops or []
        if self._cloud_llm is not None:
            from Messages import SystemMessage, UserMessage, AssistantMessage
            cls_by_role = {'system': SystemMessage, 'user': UserMessage, 'assistant': AssistantMessage}
            prompt_msgs = [cls_by_role.get(m['role'], UserMessage)(content=m['content']) for m in messages]
            response = self._cloud_llm.ask(
                input={}, prompt_msgs=prompt_msgs,
                temp=temperature, top_p=top_p, max_tokens=max_tokens,
                stops=stops if stops else None, is_json=is_json, log=False, trace=False,
            )
            if response is None:
                raise RuntimeError(f'cloud LLM ({self.server}) returned None')
            return response if isinstance(response, str) else json.dumps(response)

        # Anthropic native Messages API. Different endpoint, headers, and
        # body shape than OpenAI chat completions. temperature/top_p are
        # omitted: Opus 4.7 rejects temperature outright, and other Claude
        # models work fine on defaults. cot_profile / enable_thinking /
        # is_json are local-engine concepts and don't apply here.
        if self.server == 'anthropic':
            sys_parts: List[str] = []
            convo: List[Dict[str, str]] = []
            for m in messages:
                role = m.get('role')
                content = m.get('content', '')
                if role == 'system':
                    if content:
                        sys_parts.append(content)
                else:
                    convo.append({'role': role, 'content': content})
            body = {
                'model': self.model,
                'max_tokens': max_tokens,
                'messages': convo,
            }
            if sys_parts:
                # Mark the system block as cacheable. Cuts per-call input
                # cost ~90% on cache hits (5-min ephemeral TTL); pays a 25%
                # write premium on the first call. Wins are largest for
                # the judge (12 sequential calls share an identical rubric)
                # and the ReAct inner loop (same system across 2-3 iters
                # within a turn). Below ~1024 tokens the marker is ignored
                # and we just pay base price, so no downside on short calls.
                body['system'] = [{
                    'type': 'text',
                    'text': "\n\n".join(sys_parts),
                    'cache_control': {'type': 'ephemeral'},
                }]
            if stops:
                body['stop_sequences'] = stops
            headers = {
                'Content-Type': 'application/json',
                'x-api-key': self._api_key_value,
                'anthropic-version': '2023-06-01',
            }
            resp = requests.post(
                f'{self.base_url}/v1/messages',
                headers=headers, json=body, timeout=120,
            )
            resp.raise_for_status()
            data = resp.json()
            blocks = data.get('content') or []
            text = ''.join(
                b.get('text', '') for b in blocks
                if isinstance(b, dict) and b.get('type') == 'text'
            )
            try:
                usage = data.get('usage') or {}
                logger.info(
                    "<llm-raw> route=anthropic stop=%s "
                    "in=%s cache_write=%s cache_read=%s out=%s text=%s",
                    data.get('stop_reason'),
                    usage.get('input_tokens'),
                    usage.get('cache_creation_input_tokens'),
                    usage.get('cache_read_input_tokens'),
                    usage.get('output_tokens'),
                    json.dumps(text, ensure_ascii=False),
                )
            except Exception:
                pass
            return text

        url = f'{self.base_url}/v1/chat/completions'
        body: Dict[str, Any] = {
            'messages': messages,
            'temperature': temperature,
            'top_p': top_p,
            'max_tokens': max_tokens,
        }
        if self.model:
            body['model'] = self.model
        # Wire-level `stop` field. xAI grok-4.3 hard-rejects this
        # parameter (400 "Model grok-4.3 does not support parameter
        # stop"); the rest of the OpenAI-compat ecosystem accepts it.
        # Skip on cloud paths and rely on (a) prompt-level discipline —
        # the reflection templates explicitly require an "</end>" marker
        # and forbid postscripts — plus (b) the client-side stop-marker
        # truncation just before return below, which replicates the
        # server-side cutoff for any model whose compliance slips.
        # Local engines (llama-server, SGLang) keep the safety belt.
        if stops and not self.is_cloud:
            body['stop'] = stops
        # Reasoning effort: scenario-declared baseline, optionally
        # overridden per call. Only sent when scenario declared a baseline
        # (self.reasoning_effort is not None) — non-reasoning models
        # would just ignore it but no point shipping it.
        if self.reasoning_effort is not None:
            effective = (reasoning_effort if reasoning_effort is not None
                         else self.reasoning_effort)
            body['reasoning_effort'] = effective

        # Skip grammar / chat_template_kwargs when going to a cloud endpoint
        # (signaled by api_key being set). Cloud providers reject those
        # fields; locally-served engines (llama-server, SGLangAPIServer)
        # accept them.
        if not self.is_cloud:
            grammar = resolve_profile(cot_profile) if self.is_reasoning else None
            if grammar:
                # llama-server accepts `grammar`; SGLangAPIServer (our wrapper)
                # accepts both `grammar` and `ebnf`. Send `grammar` for both.
                body['grammar'] = grammar

            # Per-request thinking suppression for Qwen3.x and similar jinja
            # templates that auto-prefix <think>. Setting enable_thinking=False
            # tells the chat template NOT to open the thinking block, so the
            # model goes straight to the answer. Soft `/think` `/nothink`
            # directives don't work on Qwen3.6 — only this template kwarg does.
            if enable_thinking is False:
                body['chat_template_kwargs'] = {'enable_thinking': False}

        headers: Dict[str, str] = {'Content-Type': 'application/json'}
        if self._api_key_value:
            headers['Authorization'] = f'Bearer {self._api_key_value}'

        resp = requests.post(url, headers=headers, json=body, timeout=120)
        if not resp.ok:
            # Surface the provider's actual error reason (xAI/OpenAI return
            # JSON like {"error":{"message":"...","type":"..."}}); requests'
            # default HTTPError only carries the status line.
            raise RuntimeError(
                f'{resp.status_code} {resp.reason} for {url}: '
                f'{resp.text[:1000]}')
        data = resp.json()
        choices = data.get('choices') or []
        if not choices:
            raise RuntimeError(f'no choices in chat response: {data}')

        # Raw-response diagnostic. Captures content + reasoning_content
        # (the latter only present when llama-server has
        # --reasoning-format set, or SGLang has reasoning_parser set).
        # Lets us see what the engine actually emitted before any
        # client-side <think> stripping or grammar-related contortions.
        choice = choices[0]
        try:
            logger.info(
                "<llm-raw> profile=%s grammar=%s finish=%s message=%s",
                cot_profile or '(none)',
                'on' if grammar else 'off',
                choice.get('finish_reason'),
                json.dumps(choice.get('message') or {}, ensure_ascii=False),
            )
        except Exception:
            pass

        msg = choice.get('message') or {}
        text = msg.get('content', '')
        if isinstance(text, list):
            text = ''.join(p.get('text', '') for p in text if isinstance(p, dict))
        # Strip leftover reasoning blocks if the server didn't already
        # (SGLangAPIServer with reasoning_parser does; llama-server does not).
        if isinstance(text, str):
            if '</think>' in text:
                text = text.split('</think>', 1)[1].lstrip()
            elif text.lstrip().startswith('<think>'):
                text = ''
        # Client-side stop emulation. On cloud paths we suppressed
        # body['stop'] above (xAI grok-4.3 rejects it); on local paths
        # the server already truncated and the markers won't appear
        # here — so this loop is idempotent. Replicates the server's
        # truncate-at-first-stop behavior so the prompt's "</end>"
        # anchor still drops trailing prose regardless of whether the
        # provider honored the wire field.
        if stops and isinstance(text, str):
            earliest = -1
            for s in stops:
                if not s:
                    continue
                idx = text.find(s)
                if idx != -1 and (earliest == -1 or idx < earliest):
                    earliest = idx
            if earliest != -1:
                text = text[:earliest]
        return text or ''


# ─── ChatLoop ───────────────────────────────────────────────────────────────

class ChatLoop:
    def __init__(
        self,
        character_name: str,
        character_config: dict,
        shutdown_event: Optional[threading.Event] = None,
    ):
        self.character_name = character_name
        self.config = character_config
        self.shutdown_event = shutdown_event or threading.Event()
        self.shutdown_requested = False

        # ---- LLM backend ----
        llm_cfg = (character_config.get('llm_config') or {})
        self.backend = _ChatBackend(
            server=llm_cfg.get('server', 'local'),
            model=llm_cfg.get('model', ''),
            base_url=llm_cfg.get('vllm_url') or llm_cfg.get('base_url') or 'http://127.0.0.1:5000',
            is_reasoning=llm_cfg.get('is_reasoning_model'),
            api_key=llm_cfg.get('api_key'),
            reasoning_effort=llm_cfg.get('reasoning_effort'),
        )

        # ---- Persona ----
        self.persona = str(character_config.get('character', '')).strip()
        self.capabilities = str(character_config.get('capabilities', '')).strip()
        self.setting = str(character_config.get('setting', '')).strip()
        # Self-model: structural account of what the agent is (machinery,
        # access boundary, roster of inbound reflection products). Distinct
        # from persona (voice/stance) — voice and architecture evolve
        # independently and pinning them in the same field would couple them.
        self.self_model = str(character_config.get('self_model', '')).strip()

        # ---- Feature flags (on by default per project decision) ----
        self.discourse_enabled = bool((character_config.get('discourse') or {}).get('enabled', True))
        self.orientation_enabled = bool((character_config.get('orientation') or {}).get('enabled', True))
        self.history_limit = int((character_config.get('chat') or {}).get('history_limit', 20))
        # Benchmark mode: run post-turn reflection inline (rather than on the
        # background executor) so probe-time state snapshots see fully-resolved
        # state. Off by default; opt-in via scenario YAML for harnesses like
        # bench/introspective_fidelity that need deterministic ordering.
        self.benchmark_mode = bool((character_config.get('chat') or {}).get('benchmark_mode', False))
        # Autonomous concern firing (Phase C). Off by default so existing
        # benchmarks and chat scenarios behave identically — only flips on
        # when the launcher is invoked with --autonomy. Gated at the
        # _handle_tick entry; everything else (cadence math, concern
        # types) is unaffected by the flag.
        self._autonomy_enabled = bool(character_config.get('autonomy_enabled', False))
        # Schema-level affordance gate. Tool names listed here are filtered
        # out of the ReAct tool catalog (and any related guidance) so the
        # agent's affordance representation matches what's actually
        # available. Used by the cspred bench's cf-cells to enforce
        # tool-axis perturbations at the schema level rather than via
        # contradictory prose. Empty/absent → no filtering.
        self._omitted_tools: List[str] = list(
            (character_config.get('chat') or {}).get('omitted_tools') or []
        )

        # ---- Resource manager + conversation store ----
        from infospace_resource_manager import InfospaceResourceManager
        from conversation_store import ConversationStore
        world_config = character_config.get('world_config') or {}
        self.resource_manager = InfospaceResourceManager(
            world_name=world_config.get('world_name') or 'chat',
            world_config=world_config,
            agent_name=character_name,
        )
        # Restore prior session: notes, collections, FAISS indexes (load_resources
        # internally calls load_indexes + reindex). Idempotent if file is missing.
        try:
            self.resource_manager.load_from_file()
        except Exception as e:
            logger.warning(f"[{character_name}] load_from_file failed (starting fresh): {e}")

        self.store = ConversationStore(self.resource_manager, character_name, logger)
        self.store.initialize()
        # Mark the conversation Collection persistent. After this, every Note
        # added to it (record_incoming/record_outgoing) auto-persists per the
        # resource manager's persistence model.
        try:
            conv_id = self.resource_manager.named_collections.get(self.store.collection_name)
            if conv_id:
                self.resource_manager.mark_persistent(conv_id, character_name)
        except Exception as e:
            logger.warning(f"[{character_name}] mark_persistent(conversation) failed: {e}")

        # ---- Discourse / Companion (lazy; one tracker per other-entity) ----
        # ToM is intentionally NOT run in chat mode: its trust-assessment
        # framing (Competence / Intentionality / Reliability / Transparency)
        # is from a multi-agent collaboration scenario and adds no value in
        # single-user chat. Companion covers the substantive overlap (state
        # of mind, what matters, cognitive style, useful-mode).
        self._discourse_trackers: Dict[str, Any] = {}
        self._discourse_state: Dict[str, str] = {}
        self._companion_state: Dict[str, str] = {}
        # External-repo binding for the inspect_external tool. Sticky for
        # the session (and across restarts via Note). Single bound dir at
        # a time; switching wipes prior binding. None when no repo is
        # bound — in that state, inspect_external is not advertised in
        # the ReAct system prompt and calls return ERROR.
        self._external_repo: Optional[Path] = None
        self._restore_chat_state_from_notes()
        # YAML default — applied only if no Note-restored binding took
        # effect. Note wins on conflict (the user's last in-session set
        # is more authoritative than the scenario default).
        if self._external_repo is None:
            yaml_repo = (character_config.get('external_repo') or '').strip()
            if yaml_repo:
                self._set_external_repo(yaml_repo, persist=True)

        # ---- Concurrency primitives (must precede anything FAISS-touching) ----
        # _faiss_lock serializes the one cross-thread FAISS race: main
        # thread `_recall` reading the memories collection vs background
        # thread `_remember` writing to it. FAISS is not thread-safe and
        # releases the GIL during its C calls.
        # _post_turn_executor: single-worker pool for discourse + reflection
        # so the main thread returns to the inbox immediately after publishing.
        self._faiss_lock = threading.Lock()
        self._post_turn_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix=f'chat-{character_name}-postturn',
        )

        # ---- Turn-state tracking (for /status) ----
        # _current_turn is set at _process_user_turn entry and cleared
        # immediately after _publish_say — so /status reports "ready" as
        # soon as the user has seen the reply, even though trace writes
        # and post-turn reflection may still be in progress. Reads are
        # not locked: writes are single-field assigns and a momentarily
        # stale read is acceptable for a status surface.
        self._current_turn: Optional[Dict[str, Any]] = None
        self._post_turn_busy: bool = False
        self._last_reply_at: Optional[str] = None

        # ---- Long-term memory (cross-conversation, per-character) ----
        # A 'memories' Collection holds episodic specifics that should
        # survive past the rolling Companion summary. Auto-RAG queries it
        # at turn start; a post-turn reflection step decides what to write.
        self._memories_collection_id: Optional[str] = None
        self._init_memories()

        # ---- Concerns: two collections, asymmetric dynamics ----
        # agent_concerns: pressure-driven action drivers. Activation grows
        # per-tick at a rhythm-derived rate, decremented on service. Fires
        # when activation crosses threshold AND has an instruction. Seeded
        # from the YAML `concerns:` block.
        # user_concerns: model of what user cares about. Strength decays
        # per turn, bumped on input similarity. Never fires; surfaces in
        # the prompt to inform responses.
        self._agent_concerns_collection_id: Optional[str] = None
        self._user_concerns_collection_id: Optional[str] = None
        # agent_threads: durable activity-level anchors. Each thread
        # carries a centroid embedding (activation-weighted mean of
        # constituent turn embeddings), a name, a summary, and pointers
        # to attached concerns. Seeded by an offline bootstrap pass over
        # conversation history (tools/threads_bootstrap.py +
        # tools/threads_install.py), then maintained incrementally by
        # the reflection pipeline (Stage 5 — not yet wired).
        self._agent_threads_collection_id: Optional[str] = None
        # Per-turn cache of the thread activation distribution (computed
        # once at user-turn entry, read by _build_system_prompt and the
        # remember subagent context). List of (thread_dict, weight)
        # tuples sorted by weight desc; empty when no threads or no
        # turn in flight.
        self._current_thread_activation: List[Tuple[Dict[str, Any], float]] = []
        # Per-turn cache of the user turn's L2-normalized embedding,
        # populated alongside _current_thread_activation. Reused by
        # _update_thread_centroids in _post_turn_work to avoid a
        # second embed pass. None if no embedding was computed this
        # turn.
        self._current_turn_embedding: Optional[Any] = None  # np.ndarray or None
        self._init_agent_concerns()
        self._init_user_concerns()
        self._init_agent_threads()
        self._seed_concerns_from_config(character_config)

        # ---- Reasoning history (awareness feed) ----
        # Per-turn ReAct traces persisted as one JSON record per line in
        # <memory>/reasoning_trace.jsonl. Append-only. Each record's
        # `prefix_trace_refs` lists the turn_seq values that were
        # surfaced in this turn's ## Recent reasoning block, giving a
        # faithful record of awareness window per turn without
        # duplicating content. The Notes-based reasoning_history
        # collection is no longer used; _reasoning_history_collection_id
        # stays None so legacy snapshot helpers short-circuit.
        self._reasoning_history_collection_id: Optional[str] = None
        self._turn_seq: int = 0
        self._last_inject_trace_seqs: List[int] = []

        # ---- Zenoh wiring ----
        self._inbox: "queue.Queue[dict]" = queue.Queue()
        self._zenoh_session = None
        self._action_pub = None
        self._sense_sub = None

        # Processing-state publisher for the affect visualization.
        # Headless until _open_zenoh attaches the session — safe to call
        # mutators before then; they're no-ops on the wire.
        self._affect = AffectPublisher()

        # Rich-display surface for the `display` ReAct tool. Per-character
        # Zenoh key, latest-wins. Headless until session attached.
        self._canvas = CanvasPublisher(
            key=canvas_default_key(self.character_name))

    # ------------------------------------------------------------------
    # LLM helper (used by character_evaluator and DiscourseTracker)
    # ------------------------------------------------------------------

    def _llm_generate(self, messages, bindings=None, max_tokens=400,
                      temperature=0.7, stops=None, is_json=False,
                      cot_profile: Optional[str] = None,
                      enable_thinking: Optional[bool] = None,
                      reasoning_effort: Optional[str] = None) -> SimpleNamespace:
        """Callable conforming to the convention used by character_evaluator
        and discourse: list-of-strings positional roles. Returns
        SimpleNamespace(success, text, error).
        """
        try:
            chat_msgs: List[Dict[str, str]] = []
            if messages and all(isinstance(m, str) for m in messages):
                if len(messages) == 1:
                    chat_msgs = [{'role': 'user', 'content': messages[0]}]
                else:
                    chat_msgs.append({'role': 'system', 'content': messages[0]})
                    next_role = 'user'
                    for m in messages[1:]:
                        chat_msgs.append({'role': next_role, 'content': m})
                        next_role = 'assistant' if next_role == 'user' else 'user'
            else:
                for m in messages or []:
                    if isinstance(m, dict) and 'role' in m and 'content' in m:
                        chat_msgs.append({'role': m['role'], 'content': m['content']})
                    elif hasattr(m, 'role') and hasattr(m, 'content'):
                        chat_msgs.append({'role': m.role, 'content': m.content})
                    else:
                        chat_msgs.append({'role': 'user', 'content': str(m)})

            text = self.backend.chat(
                chat_msgs,
                max_tokens=max_tokens,
                temperature=temperature,
                stops=stops,
                is_json=is_json,
                cot_profile=cot_profile,
                enable_thinking=enable_thinking,
                reasoning_effort=reasoning_effort,
            )
            if not text:
                return SimpleNamespace(success=False, text='', error='empty response')
            if is_json and isinstance(text, str):
                parsed = repair_json_string(text)
                if parsed is not None:
                    return SimpleNamespace(success=True, text=parsed, error=None)
                return SimpleNamespace(success=True, text=text.strip(), error=None)
            return SimpleNamespace(success=True, text=text, error=None)
        except Exception as e:
            logger.warning(f'[{self.character_name}] _llm_generate failed: {e}')
            return SimpleNamespace(success=False, text='', error=str(e))

    # Per-profile max_tokens floor. Caller-supplied max_tokens may be tuned
    # for non-reasoning models (e.g. character_evaluator hardcodes 256 for a
    # ~150-token JSON envelope); reasoning models burn far more on the
    # analysis channel before producing the final-channel answer.
    #
    # Local floor (LOCAL): max_tokens budgets BOTH reasoning and visible
    # output on locally-served reasoning engines (vLLM, llama-server,
    # SGLang), so the cap has to absorb thinking-channel burn:
    #   - Qwen3.6: ~1000-3000 free-thinking tokens
    #   - gpt-oss-120B at reasoning_effort=medium on long reflection
    #     prompts (companion/discourse): commonly 5000-10000 analysis
    #     tokens before the final-channel structured output begins
    # Combined with the 800-1500 tokens of structured output the
    # reflection templates ask for, the worst-case budget is ~12-15K.
    # Floor set above that to avoid finish_reason=length truncating
    # mid-analysis — when that happens, vLLM's reasoning parser leaves
    # `content` empty (analysis went to reasoning_content but no final
    # channel emitted), the backend returns '', and the reflection pass
    # drops the update with "Discourse/Companion update failed".
    #
    # Cloud floor (CLOUD): xAI / OpenAI / Anthropic meter reasoning
    # tokens *separately* from visible output — `max_tokens` (or
    # `max_completion_tokens`) caps visible output only. The reflection
    # templates target ~2-3K plain-text output, so a tight cloud cap
    # bounds blast radius if the model ignores the prompt's "no
    # postscript" rule and runs on (we already drop wire-level `stop`
    # on cloud paths).
    _PROFILE_TOKEN_FLOOR_LOCAL = {
        'triage': 16384,
        'none': 16384,   # covers discourse extract + revise_belief callables
    }
    _PROFILE_TOKEN_FLOOR_CLOUD = {
        'triage': 2048,  # triage emits a small JSON envelope
        'none': 4096,    # discourse / companion target ~2-3K visible output
    }

    def _make_llm_callable(self, cot_profile: Optional[str],
                           enable_thinking: Optional[bool] = None,
                           reasoning_effort: Optional[str] = None):
        """Return an llm_generate-shaped callable bound to a CoT profile.

        Used to hand profile-tagged callables to consumers (character_evaluator,
        DiscourseTracker) that don't know about profiles or thinking themselves.
        `enable_thinking=False` suppresses <think> auto-prefix at the chat
        template level (Qwen3.6 and similar). `reasoning_effort` overrides
        the scenario's baseline reasoning_effort for every call routed
        through this callable — used to set reflection passes (companion/
        discourse) to `low` while leaving the ReAct main loop at the
        scenario's baseline (typically `medium`).
        """
        floor_table = (self._PROFILE_TOKEN_FLOOR_CLOUD
                       if self.backend.is_cloud
                       else self._PROFILE_TOKEN_FLOOR_LOCAL)
        floor = floor_table.get(cot_profile or '', 0)

        def _gen(messages, bindings=None, max_tokens=400, temperature=0.7,
                 stops=None, is_json=False):
            return self._llm_generate(messages, bindings=bindings,
                                      max_tokens=max(max_tokens, floor),
                                      temperature=temperature,
                                      stops=stops, is_json=is_json,
                                      cot_profile=cot_profile,
                                      enable_thinking=enable_thinking,
                                      reasoning_effort=reasoning_effort)
        return _gen

    # ------------------------------------------------------------------
    # Persistence — discourse_state and companion_state via named Notes,
    # save/load via the resource manager's standard mechanism.
    # ------------------------------------------------------------------

    @staticmethod
    def _state_note_name(kind: str, entity: str) -> str:
        # Single namespace for chat-loop-owned state notes.
        return f"chat:{kind}:{entity}"

    def _save_state_note(self, kind: str, entity: str, text: str) -> None:
        """Create-or-replace a named Note holding chat state, and mark it
        persistent so save_to_file() includes it."""
        try:
            success, note_id, err, _ = self.resource_manager.create_note(
                self.character_name, str(text), "text", "chat-loop", "",
                self._state_note_name(kind, entity),
                {"kind": "chat_state", "entity": entity},
            )
            if success and note_id:
                self.resource_manager.mark_persistent(note_id, self.character_name)
            elif err:
                logger.warning(f"[{self.character_name}] state note create failed: {err}")
        except Exception as e:
            logger.warning(f"[{self.character_name}] _save_state_note failed: {e}")

    def _delete_state_note(self, kind: str, entity: str) -> None:
        """Remove a chat-state Note by name, if present. Idempotent."""
        try:
            name = self._state_note_name(kind, entity)
            note_id = self.resource_manager.named_notes.get(name)
            if not note_id:
                return
            self.resource_manager.remove_resource(note_id, self.character_name)
        except Exception as e:
            logger.warning(f"[{self.character_name}] _delete_state_note failed: {e}")

    # ------------------------------------------------------------------
    # External-repo binding for the inspect_external tool.
    # Single sticky binding per character; persisted as a chat-state Note
    # (kind=external_repo, entity=default) so it survives restarts.
    # ------------------------------------------------------------------

    _EXTERNAL_REPO_ENTITY = "default"

    def _set_external_repo(self, path: str, *, persist: bool) -> Tuple[bool, str]:
        """Bind an external repo for the inspect_external tool. Validates
        that the path exists and is a directory. Returns (success, msg).
        When persist=True, writes the Note so the binding survives
        restart (used for both YAML defaults and slash-command sets)."""
        candidate = Path(str(path or '').strip()).expanduser()
        if not candidate.is_dir():
            return (False, f"path is not a directory: {candidate}")
        try:
            resolved = candidate.resolve()
        except Exception as e:
            return (False, f"resolve failed: {e}")
        self._external_repo = resolved
        if persist:
            self._save_state_note("external_repo",
                                  self._EXTERNAL_REPO_ENTITY, str(resolved))
            self._persist_to_disk()
        logger.info(f"[{self.character_name}] external_repo bound: {resolved}")
        return (True, str(resolved))

    def _clear_external_repo(self) -> bool:
        """Clear the external-repo binding. Returns True if a binding was
        cleared, False if nothing was bound. Idempotent."""
        had = self._external_repo is not None
        self._external_repo = None
        self._delete_state_note("external_repo", self._EXTERNAL_REPO_ENTITY)
        self._persist_to_disk()
        if had:
            logger.info(f"[{self.character_name}] external_repo cleared")
        return had

    def _get_external_repo(self) -> Optional[Path]:
        """Return the current binding, validating that the path still
        exists. Auto-clears the binding if it's gone stale (the directory
        was deleted or moved between sessions/calls)."""
        if self._external_repo is None:
            return None
        if not self._external_repo.is_dir():
            logger.warning(
                f"[{self.character_name}] external_repo {self._external_repo} "
                f"no longer exists; auto-clearing binding")
            self._clear_external_repo()
            return None
        return self._external_repo

    def _restore_chat_state_from_notes(self) -> None:
        """Re-populate _discourse_state and _companion_state from named notes
        that were persisted in a previous session. Stale chat:tom_state:* notes
        from older sessions are ignored — ToM is no longer run in chat mode."""
        try:
            for name, note_id in self.resource_manager.named_notes.items():
                if not name.startswith("chat:"):
                    continue
                parts = name.split(":", 2)
                if len(parts) != 3:
                    continue
                _, kind, entity = parts
                note = self.resource_manager.get_resource(note_id)
                if not note:
                    continue
                content = note.get("properties", {}).get("content", "")
                if not isinstance(content, str) or not content:
                    continue
                if kind == "discourse_state":
                    self._discourse_state[entity] = content
                elif kind == "companion_state":
                    self._companion_state[entity] = content
                elif kind == "external_repo":
                    # Validate at restore time: if the bound path no
                    # longer exists, drop the Note and leave _external_repo
                    # unset rather than carry a broken binding into the
                    # session.
                    candidate = Path(content.strip())
                    if candidate.is_dir():
                        self._external_repo = candidate.resolve()
                    else:
                        logger.warning(
                            f"[{self.character_name}] bound external_repo "
                            f"{candidate} no longer exists; clearing")
                        self._delete_state_note("external_repo", entity)
            if self._discourse_state or self._companion_state:
                logger.info(
                    f"[{self.character_name}] restored chat state: "
                    f"{len(self._discourse_state)} discourse, "
                    f"{len(self._companion_state)} companion entries"
                )
            if self._external_repo is not None:
                logger.info(
                    f"[{self.character_name}] restored external_repo "
                    f"binding: {self._external_repo}")
        except Exception as e:
            logger.warning(f"[{self.character_name}] _restore_chat_state_from_notes failed: {e}")

    def _persist_to_disk(self) -> None:
        """Flush the resource manager (notes/collections/relations + indexes)
        to disk. Safe to call frequently; the resource manager only writes the
        files that have changed contents."""
        try:
            self.resource_manager.save_to_file()
        except Exception as e:
            logger.warning(f"[{self.character_name}] save_to_file failed: {e}")

    # ------------------------------------------------------------------
    # Long-term memory — per-character "memories" Collection.
    # Notes carry kind=memory + entity; the Collection is FAISS-indexed so
    # `search_collection` returns ranked chunks. add_to_collection
    # auto-updates the index for indexed Collections.
    # ------------------------------------------------------------------

    def _init_memories(self) -> None:
        """Get-or-create the memories Collection, mark it persistent, ensure
        it has a semantic index. Idempotent: safe across restarts because
        load_from_file restores named_collections, and index_collection is
        a no-op for already-indexed collections."""
        try:
            cid = self.resource_manager.named_collections.get(_MEMORIES_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _MEMORIES_COLLECTION_NAME,
                    {"kind": "memories"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create memories collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._memories_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_memories failed: {e}")

    def _remember(self, text: str, entity: str = "",
                  category: str = 'fact',
                  polarity: str = 'positive') -> Optional[str]:
        """Persist one memory string into the memories Collection. Returns
        the new note_id, or None on failure. `category` is one of
        _MEMORY_CATEGORIES; unknown values coerce to 'fact'. `polarity`
        is 'positive' (default) or 'negative' — negatives are explicit
        rejections / dispreferences.

        Held under _faiss_lock for the FAISS-touching span (create_note
        indexes the new note, add_to_collection updates the memories
        collection's vector store). Without the lock, a concurrent
        _recall in the main thread could race a background _remember
        and corrupt the FAISS index.
        """
        if not self._memories_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        if category not in _MEMORY_CATEGORIES:
            category = 'fact'
        if polarity not in ('positive', 'negative'):
            polarity = 'positive'
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, text, "text", "chat-loop", entity or "",
                    "",
                    {
                        "kind": "memory",
                        "category": category,
                        "polarity": polarity,
                        "entity": entity,
                        "created_at": datetime.now(timezone.utc).isoformat(),
                    },
                )
                if not success or not note_id:
                    logger.warning(f"[{self.character_name}] memory create failed: {err}")
                    return None
                self.resource_manager.mark_persistent(note_id, self.character_name)
                ok, _, add_err = self.resource_manager.add_to_collection(
                    self._memories_collection_id, note_id, self.character_name)
                if not ok:
                    logger.warning(f"[{self.character_name}] memory add_to_collection failed: {add_err}")
                # Trace: append to memories.jsonl for provenance audit and
                # remember-subagent visibility. Trigger is hardcoded
                # 'reflection' since that's currently the sole writer; if
                # other writers appear (manual /recall, autonomous), pass
                # trigger as a parameter instead.
                self._write_memory_event({
                    'event': 'write',
                    'note_id': note_id,
                    'text': text,
                    'category': category,
                    'polarity': polarity,
                    'entity': entity,
                    'source_turn_seq': getattr(self, '_turn_seq', None),
                    'trigger': 'reflection',
                })
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _remember failed: {e}")
            return None

    # Recency-bias parameters for _recall. The penalty is multiplicative
    # and saturates: a brand-new memory gets factor 1.0; an ancient memory
    # asymptotes to factor (1 - _RECALL_RECENCY_MAX_PENALTY). With the
    # current values (max=0.20, tau=14d), a 100%-relevant 30-day-old
    # memory loses ~17% to age — enough that two memories tied within
    # ~0.05 similarity flip toward the newer, but not enough to demote a
    # strong topical match in favor of an irrelevant fresh one.
    _RECALL_RECENCY_MAX_PENALTY = 0.20
    _RECALL_RECENCY_TAU_DAYS = 14.0

    def _recency_adjust(self, score: float, created_at_iso: Optional[str]) -> float:
        """Apply a saturating age penalty so recall ties break toward
        newer memories. Stable facts older than ~6 weeks lose at most
        ~20% of score; iteration-accumulation stops promoting outdated
        playbook revisions over fresh ones. Failure-tolerant: any
        parse/arithmetic error returns the raw score unchanged."""
        if not created_at_iso:
            return score
        try:
            import math
            t = datetime.fromisoformat(str(created_at_iso))
            if t.tzinfo is None:
                t = t.replace(tzinfo=timezone.utc)
            age_days = max(
                0.0,
                (datetime.now(timezone.utc) - t).total_seconds() / 86400.0)
            factor = 1.0 - self._RECALL_RECENCY_MAX_PENALTY * (
                1.0 - math.exp(-age_days / self._RECALL_RECENCY_TAU_DAYS))
            return score * factor
        except Exception:
            return score

    def _recall(self, query: str, k: int = 3, threshold: float = 0.5
                ) -> List[Tuple[str, str, str]]:
        """Semantic search over the memories Collection. Returns ranked
        (text, category, polarity) tuples, highest score first. Category
        is read from the source note's properties; pre-taxonomy memories
        without a category default to 'fact'. Polarity is 'positive' or
        'negative'; pre-J memories without a polarity default to
        'positive'. Empty list on miss / not yet indexed / any error.

        Re-ranks by recency-adjusted score: fetch up to 2*k candidates
        from FAISS, apply a saturating age penalty (see _recency_adjust),
        then return the top-k. Raw similarity still dominates — recency
        only flips ties within the same topical band."""
        if not self._memories_collection_id or not query:
            return []
        try:
            fetch = max(k * 2, 6)
            with self._faiss_lock:
                ok, results, err = self.resource_manager.search_collection(
                    self.character_name, self._memories_collection_id, query,
                    mode='semantic', limit=fetch, threshold=threshold)
            if not ok or not results:
                return []
            scored: List[Tuple[float, str, str, str]] = []
            for r in results:
                if not isinstance(r, dict):
                    continue
                doc = r.get('document')
                if not isinstance(doc, str) or not doc.strip():
                    continue
                # Map chunk back to source note for category + polarity +
                # created_at. Chunks store source_note_id in metadata (per
                # _index_note_chunks). Missing note → fall back to 'fact'
                # category, 'positive' polarity, no recency adjustment.
                # No lock needed for this lookup — get_resource is a dict
                # read, not a FAISS operation.
                cat = 'fact'
                pol = 'positive'
                created_at: Optional[str] = None
                meta = r.get('metadata') or {}
                note_id = meta.get('source_note_id')
                if note_id:
                    note = self.resource_manager.get_resource(note_id)
                    if note:
                        props = note.get('properties') or {}
                        raw = props.get('category')
                        if isinstance(raw, str) and raw in _MEMORY_CATEGORIES:
                            cat = raw
                        raw_pol = props.get('polarity')
                        if isinstance(raw_pol, str) and raw_pol in ('positive', 'negative'):
                            pol = raw_pol
                        ca = props.get('created_at')
                        if isinstance(ca, str):
                            created_at = ca
                base_score = float(r.get('score') or 0.0)
                adj = self._recency_adjust(base_score, created_at)
                scored.append((adj, doc.strip(), cat, pol))
            scored.sort(key=lambda x: x[0], reverse=True)
            return [(text, cat, pol) for _adj, text, cat, pol in scored[:k]]
        except Exception as e:
            logger.warning(f"[{self.character_name}] _recall failed: {e}")
            return []

    # ------------------------------------------------------------------
    # Concerns — actionable directives Jill should keep ready to advance.
    # Distinct from memories (stable specifics): a concern is something
    # she can reasonably expect to act on as an instruction; a preference
    # ("be brief") is a modifier that stays in memories.
    #
    # Lifecycle is lazy: weight = exp(-(now - last_engaged_at) / tau)
    # is computed at read time. Recall hits refresh last_engaged_at.
    # When weight drops below _CONCERN_SATISFIED_THRESHOLD the concern
    # transitions active → satisfied at the next read, except for seed
    # concerns (architectural baseline from YAML, immune to decay).
    # ------------------------------------------------------------------

    def _init_agent_concerns(self) -> None:
        """Get-or-create the agent_concerns Collection, mark it persistent,
        ensure semantic index exists. Idempotent across restarts."""
        try:
            cid = self.resource_manager.named_collections.get(_AGENT_CONCERNS_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _AGENT_CONCERNS_COLLECTION_NAME,
                    {"kind": "agent_concerns"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create agent_concerns collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._agent_concerns_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_agent_concerns failed: {e}")

    def _init_user_concerns(self) -> None:
        """Get-or-create the user_concerns Collection, mark it persistent,
        ensure semantic index exists. Distinct from agent_concerns:
        decays per turn, bumped by user-input similarity, never fires
        autonomously. Idempotent across restarts."""
        try:
            cid = self.resource_manager.named_collections.get(_USER_CONCERNS_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _USER_CONCERNS_COLLECTION_NAME,
                    {"kind": "user_concerns"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create user_concerns collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._user_concerns_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_user_concerns failed: {e}")

    def _init_agent_threads(self) -> None:
        """Get-or-create the agent_threads Collection, mark it persistent,
        ensure semantic index exists. Threads are activity-level anchors;
        each note carries a centroid_embedding (activation-weighted mean
        of constituent turn embeddings) in its properties for direct
        cosine-similarity activation against new turns. The note's
        text field holds the LLM-generated summary, which the semantic
        index will pick up — useful for thread-similarity queries during
        consolidation. Idempotent across restarts."""
        try:
            cid = self.resource_manager.named_collections.get(_AGENT_THREADS_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _AGENT_THREADS_COLLECTION_NAME,
                    {"kind": "agent_threads"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create agent_threads collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._agent_threads_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_agent_threads failed: {e}")

    def _reasoning_trace_path(self) -> 'Path':
        """Path to <memory>/reasoning_trace.jsonl — one JSON record per
        ReAct turn, append-only. Line N of the file is record N
        (= turn_seq N, 1-indexed). The active-recall subagent reads
        this file directly with line-range semantics where each line is
        one parseable JSON object."""
        return self._memory_dir() / 'reasoning_trace.jsonl'

    def _autonomy_log_path(self) -> 'Path':
        """Path to <memory>/autonomy.jsonl — one JSON record per
        autonomous-fire event (fire/deferred). Append-only, separate
        from reasoning_trace.jsonl so 'what has Jill been doing on her
        own?' is grep-able as a clean stream."""
        return self._memory_dir() / 'autonomy.jsonl'

    def _write_autonomy_event(self, event: Dict[str, Any]) -> None:
        """Append one event record to autonomy.jsonl. Stamps `ts` and
        `character` if not already set. Best-effort — failures log a
        warning but don't disrupt the autonomous turn."""
        path = self._autonomy_log_path()
        record = dict(event)
        record.setdefault('ts', datetime.now(timezone.utc).isoformat())
        record.setdefault('character', self.character_name)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        except Exception as e:
            logger.warning(f"[{self.character_name}] autonomy.jsonl write failed: {e}")

    def _memories_log_path(self) -> 'Path':
        """Path to <memory>/memories.jsonl — one JSON record per memory
        write. Append-only, separate from the live memories collection so
        provenance audit + temporal queries don't require iterating the
        Notes store. The `remember` subagent can grep this file for
        questions like 'when did I learn X?' or 'what memories were
        written from yesterday's turns?'."""
        return self._memory_dir() / 'memories.jsonl'

    def _write_memory_event(self, event: Dict[str, Any]) -> None:
        """Append one event record to memories.jsonl. Stamps `ts` and
        `character` if not already set. Best-effort — failures log a
        warning but don't disrupt the write that triggered the event."""
        path = self._memories_log_path()
        record = dict(event)
        record.setdefault('ts', datetime.now(timezone.utc).isoformat())
        record.setdefault('character', self.character_name)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        except Exception as e:
            logger.warning(f"[{self.character_name}] memories.jsonl write failed: {e}")

    def _load_reasoning_records(self) -> List[Dict[str, Any]]:
        """Read all per-turn records from reasoning_trace.jsonl.
        Returns [] if missing or unreadable. Lines are 1:1 with
        records (one JSON object per line, in turn order)."""
        path = self._reasoning_trace_path()
        if not path.is_file():
            return []
        out: List[Dict[str, Any]] = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for line in f:
                    s = line.strip()
                    if not s:
                        continue
                    try:
                        out.append(json.loads(s))
                    except json.JSONDecodeError:
                        continue
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] reasoning trace read failed: {e}")
        return out

    @staticmethod
    def _seed_concern_name(idx: int) -> str:
        return f"chat:agent_concern:seed:{idx}"

    def _seed_concerns_from_config(self, character_config: dict) -> None:
        """Instantiate seed concerns listed under YAML key `concerns:`.
        Seeds become agent_concerns with seed=True. They source derived
        agent_concerns via reflection and also carry their own instruction
        (if any) for direct firing. Idempotent — each seed gets a stable
        named-note slot.

        YAML may specify `rhythm_hours` (preferred) or legacy
        `cadence_hours` per seed; default is weekly (168h). `instruction`
        is optional — without it the concern accumulates activation but
        never fires (it's a source for derived concerns)."""
        seeds = character_config.get('concerns') or []
        if not isinstance(seeds, list) or not self._agent_concerns_collection_id:
            return
        for idx, seed in enumerate(seeds):
            if not isinstance(seed, dict):
                continue
            text = str(seed.get('text', '') or '').strip()
            if not text:
                continue
            entity = str(seed.get('entity', 'User') or 'User')
            name = self._seed_concern_name(idx)
            if name in self.resource_manager.named_notes:
                continue   # Already seeded; preserve any edits
            rhythm_h = seed.get('rhythm_hours')
            if rhythm_h is None:
                rhythm_h = seed.get('cadence_hours')   # legacy field
            if rhythm_h is None and seed.get('cadence_days') is not None:
                try:
                    rhythm_h = float(seed.get('cadence_days')) * 24.0
                except (TypeError, ValueError):
                    rhythm_h = None
            self._add_agent_concern(
                text, entity=entity, provenance='asserted', seed=True,
                name=name, rhythm_hours=rhythm_h, rhythm_source='external',
                instruction=seed.get('instruction'))

    # ------------------------------------------------------------------
    # Concern creation: shared note-create path + per-collection helpers.
    # ------------------------------------------------------------------

    def _find_similar_concern(self, text: str, collection_id: Optional[str],
                              threshold: float = _CONCERN_RECURRENCE_THRESHOLD
                              ) -> Optional[str]:
        """Find the top semantically-similar existing concern in
        `collection_id` whose similarity meets `threshold`. Returns the
        source note_id, or None. Excludes abandoned concerns. Caller
        decides what to do with active vs satisfied matches (per-class
        promote/bump logic differs)."""
        if not collection_id or not text:
            return None
        try:
            with self._faiss_lock:
                ok, results, _ = self.resource_manager.search_collection(
                    self.character_name, collection_id, text,
                    mode='semantic', limit=1, threshold=threshold)
            if not ok or not results:
                return None
            r = results[0] if isinstance(results[0], dict) else None
            if not r:
                return None
            meta = r.get('metadata') or {}
            note_id = meta.get('source_note_id')
            if not note_id:
                return None
            note = self.resource_manager.get_resource(note_id)
            if not note:
                return None
            status = (note.get('properties') or {}).get('status', 'active')
            if status == 'abandoned':
                return None
            return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _find_similar_concern failed: {e}")
            return None

    def _promote_existing_agent_concern(self, note_id: str) -> str:
        """Recurrence: revive a near-twin agent_concern rather than
        creating a duplicate. status: satisfied → active. Activation
        left as-is so existing pressure is preserved; the user's
        re-engagement is for the topic, not against the queue."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return note_id
        props = note.setdefault('properties', {})
        if props.get('status') == 'satisfied':
            props['status'] = 'active'
        return note_id

    def _bump_existing_user_concern(self, note_id: str) -> str:
        """Recurrence: bump strength on a near-twin user_concern rather
        than creating a duplicate."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return note_id
        props = note.setdefault('properties', {})
        s = float(props.get('strength', 0.0) or 0.0)
        props['strength'] = min(1.0, s + _USER_CONCERN_BUMP_AMOUNT)
        props['last_bumped_at'] = datetime.now(timezone.utc).isoformat()
        if props.get('status') != 'active':
            props['status'] = 'active'
        return note_id

    @staticmethod
    def _clamp_optional(value: Any, lo: float, hi: float) -> Optional[float]:
        """Clamp a numeric value into [lo, hi]; pass None through."""
        if value is None:
            return None
        try:
            v = float(value)
        except (TypeError, ValueError):
            return None
        return max(lo, min(hi, v))

    @classmethod
    def _resolve_rhythm_hours(cls, properties: Dict[str, Any]) -> int:
        """Read rhythm_hours from an agent_concern note. Falls back to
        legacy `cadence_hours` (current data) and `cadence_days` (older
        migration). Always returns a value from the allowed bucket;
        defaults to weekly when nothing usable is on the note."""
        raw = properties.get('rhythm_hours')
        if raw is None:
            raw = properties.get('cadence_hours')
        if raw is None and properties.get('cadence_days') is not None:
            try:
                raw = float(properties['cadence_days']) * 24.0
            except (TypeError, ValueError):
                raw = None
        return _snap_rhythm_hours(raw)

    def _create_concern_note(self, text: str, name: str, entity: str,
                             properties: Dict[str, Any],
                             collection_id: str) -> Optional[str]:
        """Shared note creation path. create_note + mark_persistent +
        add_to_collection, all under _faiss_lock."""
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, text, "text", "chat-loop",
                    entity or "", name or "", properties)
                if not success or not note_id:
                    logger.warning(
                        f"[{self.character_name}] concern create failed: {err}")
                    return None
                self.resource_manager.mark_persistent(note_id, self.character_name)
                ok, _, add_err = self.resource_manager.add_to_collection(
                    collection_id, note_id, self.character_name)
                if not ok:
                    logger.warning(
                        f"[{self.character_name}] concern add_to_collection failed: {add_err}")
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _create_concern_note failed: {e}")
            return None

    def _add_agent_concern(self, text: str, entity: str = 'User',
                           provenance: str = 'asserted', seed: bool = False,
                           name: str = '',
                           rhythm_hours: Optional[int] = None,
                           rhythm_source: str = 'default',
                           instruction: Optional[str] = None,
                           skip_recurrence: bool = False,
                           extra_properties: Optional[Dict[str, Any]] = None
                           ) -> Optional[str]:
        """Create an agent_concern. Activation starts at 0; per-tick
        growth is proportional to elapsed wall-clock / rhythm_hours.
        Fires when activation ≥ _AGENT_CONCERN_FIRE_THRESHOLD AND
        instruction is non-null. Seeds carry seed=True; their activation
        still grows but they won't fire without an instruction.

        skip_recurrence bypasses similarity merge — used by successor
        concerns. extra_properties merges into the note's properties
        (e.g. successor_of, successor_depth)."""
        if not self._agent_concerns_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        rhythm_hours = _snap_rhythm_hours(rhythm_hours)
        instruction = (str(instruction).strip() if instruction else '') or None
        if rhythm_source not in ('external', 'urgency', 'default'):
            rhythm_source = 'default'
        if provenance not in ('asserted', 'inferred'):
            provenance = 'asserted'
        if not seed and not skip_recurrence:
            existing = self._find_similar_concern(
                text, self._agent_concerns_collection_id)
            if existing:
                return self._promote_existing_agent_concern(existing)
        now_iso = datetime.now(timezone.utc).isoformat()
        properties: Dict[str, Any] = {
            "kind": "agent_concern",
            "status": "active",
            "entity": entity,
            "provenance": provenance,
            "seed": bool(seed),
            "instruction": instruction,
            "rhythm_hours": rhythm_hours,
            "rhythm_source": rhythm_source,
            "activation": 0.0,
            "last_activation_update_at": now_iso,
            "last_fired_at": None,
        }
        if extra_properties:
            properties.update(extra_properties)
        return self._create_concern_note(
            text, name, entity, properties,
            self._agent_concerns_collection_id)

    def _add_user_concern(self, text: str, entity: str = 'User',
                          name: str = '',
                          initial_strength: float = 1.0,
                          skip_recurrence: bool = False,
                          extra_properties: Optional[Dict[str, Any]] = None
                          ) -> Optional[str]:
        """Create a user_concern. Strength starts at initial_strength
        (1.0 default). Decays each user turn; bumped on similarity hit;
        pruned below threshold. Never fires."""
        if not self._user_concerns_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        if not skip_recurrence:
            existing = self._find_similar_concern(
                text, self._user_concerns_collection_id)
            if existing:
                return self._bump_existing_user_concern(existing)
        now_iso = datetime.now(timezone.utc).isoformat()
        s = max(0.0, min(1.0, float(initial_strength)))
        properties: Dict[str, Any] = {
            "kind": "user_concern",
            "status": "active",
            "entity": entity,
            "strength": s,
            "last_bumped_at": now_iso,
        }
        if extra_properties:
            properties.update(extra_properties)
        return self._create_concern_note(
            text, name, entity, properties,
            self._user_concerns_collection_id)

    # ------------------------------------------------------------------
    # Thread anchors. Threads are activity-level state surfaces with a
    # centroid_embedding stored as a property (NOT in the FAISS index;
    # the index covers the summary text for thread-similarity queries
    # during consolidation, but activation matching uses the centroid
    # directly via cosine similarity over the active-thread set).
    # No firing, no decay, no autonomous behavior. Read-only at this
    # stage — install populated by tools/threads_install.py; reflection-
    # side updates land in Stage 5.
    # ------------------------------------------------------------------

    def _add_thread(self, name: str, summary: str,
                    centroid_embedding: List[float],
                    constituent_turn_count: int = 0,
                    creation_provenance: str = 'discovered',
                    status: str = 'active',
                    exemplar_pairs: Optional[List[Dict[str, str]]] = None,
                    extra_properties: Optional[Dict[str, Any]] = None
                    ) -> Optional[str]:
        """Create a thread note in the agent_threads Collection. The
        note's text field gets the summary; centroid_embedding is
        stored as a list[float] in properties. No similarity-based
        dedup at this layer (callers are expected to enforce
        idempotency by checking name).

        exemplar_pairs is a render-only payload — list of
        {"user": str, "agent": str} dicts captured at bootstrap as
        representative exchanges. Surfaced inside the active-threads
        block to give Jill concrete touchstones for what the thread is.
        Never read by activation, classification, or centroid drift."""
        if not self._agent_threads_collection_id:
            return None
        name = (name or "").strip()
        summary = (summary or "").strip()
        if not name or not summary:
            return None
        if not centroid_embedding:
            logger.warning(f"[{self.character_name}] _add_thread: centroid_embedding is empty for {name!r}")
            return None
        if creation_provenance not in ('bootstrap', 'discovered'):
            creation_provenance = 'discovered'
        if status not in ('active', 'dormant', 'archived'):
            status = 'active'
        now_iso = datetime.now(timezone.utc).isoformat()
        # Note: `name` is stored as `note_name` via create_note's positional
        # arg (which lands in properties.note_name); `created_at` is set
        # automatically by create_note. Don't duplicate them here.
        clean_exemplars: List[Dict[str, str]] = []
        for ex in (exemplar_pairs or []):
            u = str((ex or {}).get("user") or "").strip()
            a = str((ex or {}).get("agent") or "").strip()
            if u and a:
                clean_exemplars.append({"user": u, "agent": a})
        properties: Dict[str, Any] = {
            "kind": "thread",
            "status": status,
            "summary": summary,
            "centroid_embedding": list(map(float, centroid_embedding)),
            "constituent_turn_count": int(constituent_turn_count),
            "creation_provenance": creation_provenance,
            "last_activated_at": now_iso,
            "last_centroid_update_at": now_iso,
            "attached_concern_ids": [],
            "attached_rule_ids": [],
            "exemplar_pairs": clean_exemplars,
        }
        if extra_properties:
            properties.update(extra_properties)
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, summary, "text", "chat-loop",
                    "", name, properties)
                if not success or not note_id:
                    logger.warning(f"[{self.character_name}] thread create failed: {err}")
                    return None
                self.resource_manager.mark_persistent(note_id, self.character_name)
                ok, _, add_err = self.resource_manager.add_to_collection(
                    self._agent_threads_collection_id, note_id, self.character_name)
                if not ok:
                    logger.warning(f"[{self.character_name}] thread add_to_collection failed: {add_err}")
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _add_thread failed: {e}")
            return None

    def _get_threads(self, statuses: Optional[Tuple[str, ...]] = ('active',)
                     ) -> List[Dict[str, Any]]:
        """Return thread records matching any of the given statuses
        (default: active only). Each record is the resource dict with
        properties merged in for convenience."""
        if not self._agent_threads_collection_id:
            return []
        try:
            coll = self.resource_manager.get_resource(self._agent_threads_collection_id)
            if not coll:
                return []
            note_ids = (coll.get("properties") or {}).get("content", []) or []
            out: List[Dict[str, Any]] = []
            status_set = set(statuses) if statuses else None
            for nid in note_ids:
                note = self.resource_manager.get_resource(nid)
                if not note:
                    continue
                props = note.get("properties") or {}
                if props.get("kind") != "thread":
                    continue
                if status_set is not None and props.get("status") not in status_set:
                    continue
                # Human-readable name lives in properties.note_name (set
                # via create_note's positional arg); created_at is set by
                # create_note automatically.
                out.append({
                    "id": nid,
                    "name": props.get("note_name", ""),
                    "summary": props.get("summary", ""),
                    "status": props.get("status", "active"),
                    "centroid_embedding": props.get("centroid_embedding") or [],
                    "constituent_turn_count": int(props.get("constituent_turn_count", 0)),
                    "creation_provenance": props.get("creation_provenance"),
                    "created_at": props.get("created_at"),
                    "last_activated_at": props.get("last_activated_at"),
                    "last_centroid_update_at": props.get("last_centroid_update_at"),
                    "attached_concern_ids": props.get("attached_concern_ids") or [],
                    "attached_rule_ids": props.get("attached_rule_ids") or [],
                    "exemplar_pairs": props.get("exemplar_pairs") or [],
                })
            return out
        except Exception as e:
            logger.warning(f"[{self.character_name}] _get_threads failed: {e}")
            return []

    def _find_thread_by_name(self, name: str) -> Optional[Dict[str, Any]]:
        """Return the active thread record with the given name, or None."""
        for t in self._get_threads(statuses=None):
            if t.get("name") == name:
                return t
        return None

    def _compute_thread_activation(self, text: str, temperature: float = 4.0
                                   ) -> List[Tuple[Dict[str, Any], float]]:
        """Compute the activation distribution over active threads for
        the given text. Embeds the text with the same model used
        elsewhere (bge-small-en-v1.5, L2-normalized), computes cosine
        similarity to each active thread's centroid (also L2-normalized),
        and applies softmax with the given temperature.

        Higher temperature → flatter distribution (more threads share
        weight). temperature=1.0 is "honest softmax" but tends to
        produce winner-take-all assignments because cosine similarities
        in this embedding space cluster within a narrow range.
        temperature=4.0 (default) gives a more usable distribution where
        secondary threads carry meaningful weight.

        Returns a list of (thread_record, weight) tuples sorted by
        weight descending. Empty list if no active threads or text is
        empty. The list represents a probability distribution; weights
        sum to 1.0 (within float precision).

        Cheap — single embedding call + N cosine sims for N active
        threads. With <100 active threads this is sub-millisecond on
        the GPU."""
        text = (text or "").strip()
        if not text:
            return []
        threads = self._get_threads(statuses=('active',))
        if not threads:
            return []

        try:
            self.resource_manager._init_embedder()
            embedder = self.resource_manager.embedder
            if embedder is None:
                logger.warning(
                    f"[{self.character_name}] _compute_thread_activation: "
                    f"embedder unavailable")
                return []
            import numpy as np
            turn_emb = embedder.encode(
                text, normalize_embeddings=True, convert_to_numpy=True,
                show_progress_bar=False)
            # Cache for reuse by _update_thread_centroids in
            # _post_turn_work — same embedding feeds both activation
            # readout and post-turn centroid drift.
            self._current_turn_embedding = turn_emb
            # Cosine similarity = dot product on L2-normalized vectors.
            sims: List[float] = []
            valid_threads: List[Dict[str, Any]] = []
            for t in threads:
                c = t.get("centroid_embedding") or []
                if not c:
                    continue
                cv = np.asarray(c, dtype=np.float32)
                if cv.shape != turn_emb.shape:
                    logger.warning(
                        f"[{self.character_name}] thread {t.get('name')!r} "
                        f"centroid dim {cv.shape} != turn emb dim "
                        f"{turn_emb.shape} — skipping")
                    continue
                sims.append(float(np.dot(turn_emb, cv)))
                valid_threads.append(t)
            if not sims:
                return []
            # Softmax with temperature. We expect cos sim in roughly
            # [-0.2, 0.9] range for bge-small; temperature scales these
            # before exp.
            arr = np.asarray(sims, dtype=np.float64) * float(temperature)
            arr -= arr.max()  # numerical stability
            exp = np.exp(arr)
            weights = exp / exp.sum()
            paired = list(zip(valid_threads, [float(w) for w in weights]))
            paired.sort(key=lambda x: -x[1])
            return paired
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _compute_thread_activation failed: {e}")
            return []

    def _render_active_threads_block(
            self,
            activation: List[Tuple[Dict[str, Any], float]],
            primary_threshold: float = 0.30,
            secondary_threshold: float = 0.20,
            secondary_cap: int = 2,
            ) -> str:
        """Render the activation-weighted thread block for the system
        prompt. Returns empty string when no thread has meaningful
        activation — caller should skip the block in that case.

        Two thresholds, both load-bearing:

        - primary_threshold: minimum weight for the top thread to
          count as a primary. Below this, the distribution is too
          uniform to claim any thread is "what we're working on" —
          render nothing rather than a misleading attribution. With
          5 active threads, uniform random would give 0.20 each;
          threshold 0.30 means the primary is at least 1.5× uniform.

        - secondary_threshold: minimum weight for a thread to appear
          in "also touching on". Below this, the secondary is too
          weak to be informative.

        secondary_cap bounds the secondaries list (top-N after
        filtering). Activation numbers themselves are not surfaced
        as floats — prominence in the prompt encodes the
        distribution."""
        if not activation:
            return ""
        primary, primary_w = activation[0]
        if primary_w < primary_threshold:
            # No thread is clearly active; suppress the block rather
            # than mislead with a low-confidence "primary" assignment.
            return ""

        primary_name = primary.get("name", "")
        primary_summary = (primary.get("summary") or "").strip()

        secondaries: List[Tuple[Dict[str, Any], float]] = []
        for t, w in activation[1:]:
            if w < secondary_threshold:
                break
            secondaries.append((t, w))
            if len(secondaries) >= secondary_cap:
                break

        def _render_exemplars(thread: Dict[str, Any], k: int,
                              indent: str = "") -> List[str]:
            """Format up to k exemplar pairs as 'User: … / Jill: …'
            lines. Returns empty list when the thread has no exemplars
            (e.g. installed before exemplars existed). Render-only —
            never feeds back into activation or drift."""
            pairs = thread.get("exemplar_pairs") or []
            if not pairs:
                return []
            out: List[str] = []
            for ex in pairs[:k]:
                u = str((ex or {}).get("user") or "").strip()
                a = str((ex or {}).get("agent") or "").strip()
                if not u or not a:
                    continue
                out.append(f"{indent}- User: {u}")
                out.append(f"{indent}  Jill: {a}")
            return out

        lines: List[str] = []
        lines.append("## Current activity context (from session threads)")
        lines.append(
            "Threads are activity-level anchors inferred from the "
            "shape of recent conversation. The primary thread reflects "
            "what we're most engaged with right now; secondary threads "
            "are activities the current turn also touches. Exemplar "
            "exchanges (when present) are representative past pairs "
            "from each thread, captured at bootstrap.")
        lines.append("")
        lines.append(f"**Primary:** `{primary_name}` — {primary_summary}")
        primary_ex = _render_exemplars(primary, k=2)
        if primary_ex:
            lines.append("Exemplar exchanges:")
            lines.extend(primary_ex)
        if secondaries:
            lines.append("")
            lines.append("**Also touching on:**")
            for t, _w in secondaries:
                name = t.get("name", "")
                summary = (t.get("summary") or "").strip()
                lines.append(f"- `{name}` — {summary}")
                lines.extend(_render_exemplars(t, k=1, indent="  "))
        return "\n".join(lines)

    # ------------------------------------------------------------------
    # Per-tick / per-turn dynamics. Pure arithmetic — no LLM in these
    # paths. Called from _handle_tick (growth, fire-check) and
    # _process_user_turn (decay, bump). Cheap enough to run every
    # tick / every turn without throttling.
    # ------------------------------------------------------------------

    def _grow_agent_concerns_per_tick(self) -> None:
        """Apply elapsed-time growth to every active agent_concern.
        activation += growth_for_elapsed(rhythm_hours, Δhours). Caps
        at 1.0; updates last_activation_update_at."""
        if not self._agent_concerns_collection_id:
            return
        coll = self.resource_manager.get_resource(self._agent_concerns_collection_id)
        if not coll:
            return
        note_ids = (coll.get('properties') or {}).get('content', []) or []
        now = datetime.now(timezone.utc)
        for nid in note_ids:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.setdefault('properties', {})
            if props.get('status') != 'active':
                continue
            rhythm_h = self._resolve_rhythm_hours(props)
            last_str = (props.get('last_activation_update_at')
                        or props.get('created_at'))
            try:
                last = datetime.fromisoformat(str(last_str)) if last_str else now
                if last.tzinfo is None:
                    last = last.replace(tzinfo=timezone.utc)
            except (TypeError, ValueError):
                last = now
            elapsed_h = max(0.0, (now - last).total_seconds() / 3600.0)
            growth = _agent_concern_growth_for_elapsed(rhythm_h, elapsed_h)
            a = float(props.get('activation', 0.0) or 0.0)
            props['activation'] = min(1.0, a + growth)
            props['last_activation_update_at'] = now.isoformat()

    def _decay_user_concerns_per_turn(self) -> None:
        """Apply per-turn strength decay to active user_concerns. Hard-
        deletes any concern that falls below the prune threshold (a
        topic the user hasn't engaged with for ~18 turns at default
        rates)."""
        if not self._user_concerns_collection_id:
            return
        coll = self.resource_manager.get_resource(self._user_concerns_collection_id)
        if not coll:
            return
        note_ids = list((coll.get('properties') or {}).get('content', []) or [])
        pruned: List[str] = []
        for nid in note_ids:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.setdefault('properties', {})
            if props.get('status') != 'active':
                continue
            s = float(props.get('strength', 1.0) or 0.0)
            s = max(0.0, s - _USER_CONCERN_DECAY_PER_TURN)
            props['strength'] = s
            if s < _USER_CONCERN_PRUNE_THRESHOLD:
                ok, err = self.resource_manager.delete_resource(nid)
                if ok:
                    pruned.append(nid)
                else:
                    logger.warning(
                        f"[{self.character_name}] user_concern prune failed for "
                        f"{nid}: {err}")
        if pruned:
            logger.info(
                f"[{self.character_name}] pruned {len(pruned)} user_concern(s) "
                f"below strength threshold")

    def _bump_user_concerns_on_input(self, text: str) -> None:
        """Semantic-search user_concerns for input similarity; bump
        strength on each hit above threshold. Cheap (one FAISS query)
        and called once per user-turn entry."""
        if not self._user_concerns_collection_id or not text:
            return
        try:
            with self._faiss_lock:
                ok, results, _ = self.resource_manager.search_collection(
                    self.character_name, self._user_concerns_collection_id,
                    text, mode='semantic',
                    limit=_USER_CONCERN_PROMPT_BUDGET * 2,
                    threshold=_USER_CONCERN_BUMP_THRESHOLD)
            if not ok or not results:
                return
            now_iso = datetime.now(timezone.utc).isoformat()
            for r in results:
                if not isinstance(r, dict):
                    continue
                meta = r.get('metadata') or {}
                nid = meta.get('source_note_id')
                if not nid:
                    continue
                note = self.resource_manager.get_resource(nid)
                if not note:
                    continue
                props = note.setdefault('properties', {})
                if props.get('status') != 'active':
                    continue
                s = float(props.get('strength', 0.0) or 0.0)
                props['strength'] = min(1.0, s + _USER_CONCERN_BUMP_AMOUNT)
                props['last_bumped_at'] = now_iso
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _bump_user_concerns_on_input failed: {e}")

    def _service_agent_concern(self, note_id: str, exit_reason: str) -> None:
        """Decrement activation on service. Called after autonomous fire
        completes. exit_reason determines decrement size:
          'respond'   → full service (ReAct ran to completion)
          'max_iters' → partial (work continued via successor concern)
          others      → no decrement (fire didn't really happen)
        Activation floors at 0; last_fired_at recorded so we have a
        fire history independent of the activation curve."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return
        props = note.setdefault('properties', {})
        if exit_reason == 'respond':
            decrement = _AGENT_CONCERN_SERVICE_FULL
        elif exit_reason == 'max_iters':
            decrement = _AGENT_CONCERN_SERVICE_PARTIAL
        else:
            decrement = 0.0
        if decrement > 0:
            a = float(props.get('activation', 0.0) or 0.0)
            props['activation'] = max(0.0, a - decrement)
        props['last_fired_at'] = datetime.now(timezone.utc).isoformat()

    # ------------------------------------------------------------------
    # Surfacing helpers — top-K iteration over each collection for
    # prompt rendering and resource_browser queries.
    # ------------------------------------------------------------------

    def _iter_active_agent_concerns(self) -> List[Tuple[str, Dict[str, Any], float]]:
        """Iterate active agent_concerns: (note_id, note, activation)."""
        if not self._agent_concerns_collection_id:
            return []
        coll = self.resource_manager.get_resource(self._agent_concerns_collection_id)
        if not coll:
            return []
        out: List[Tuple[str, Dict[str, Any], float]] = []
        for nid in (coll.get('properties') or {}).get('content', []) or []:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get('properties') or {}
            if props.get('status') != 'active':
                continue
            a = float(props.get('activation', 0.0) or 0.0)
            out.append((nid, note, a))
        return out

    def _iter_active_user_concerns(self) -> List[Tuple[str, Dict[str, Any], float]]:
        """Iterate active user_concerns: (note_id, note, strength)."""
        if not self._user_concerns_collection_id:
            return []
        coll = self.resource_manager.get_resource(self._user_concerns_collection_id)
        if not coll:
            return []
        out: List[Tuple[str, Dict[str, Any], float]] = []
        for nid in (coll.get('properties') or {}).get('content', []) or []:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get('properties') or {}
            if props.get('status') != 'active':
                continue
            s = float(props.get('strength', 0.0) or 0.0)
            out.append((nid, note, s))
        return out

    def _top_active_agent_concerns(self, n: int = _AGENT_CONCERN_PROMPT_BUDGET
                                   ) -> List[Tuple[str, str, float, Dict[str, Any]]]:
        """Top-n agent_concerns by activation, descending. Tuple:
        (note_id, text, activation, props)."""
        active = self._iter_active_agent_concerns()
        active.sort(key=lambda t: t[2], reverse=True)
        out: List[Tuple[str, str, float, Dict[str, Any]]] = []
        for nid, note, a in active[:max(0, n)]:
            props = note.get('properties') or {}
            text = str(props.get('content', '') or note.get('text', '') or '').strip()
            if not text:
                continue
            out.append((nid, text, a, props))
        return out

    def _top_active_user_concerns(self, n: int = _USER_CONCERN_PROMPT_BUDGET
                                  ) -> List[Tuple[str, str, float, Dict[str, Any]]]:
        """Top-n user_concerns by strength, descending. Tuple:
        (note_id, text, strength, props)."""
        active = self._iter_active_user_concerns()
        active.sort(key=lambda t: t[2], reverse=True)
        out: List[Tuple[str, str, float, Dict[str, Any]]] = []
        for nid, note, s in active[:max(0, n)]:
            props = note.get('properties') or {}
            text = str(props.get('content', '') or note.get('text', '') or '').strip()
            if not text:
                continue
            out.append((nid, text, s, props))
        return out

    def _set_concern_status(self, concern_id: str, new_status: str
                            ) -> Tuple[bool, Optional[str]]:
        """Manual status transition (browser-driven abandon/close).
        Accepts notes from either collection — kind check covers
        agent_concern, user_concern, and legacy 'concern'."""
        if new_status not in _CONCERN_STATUSES:
            return False, f"invalid status {new_status!r}"
        note = self.resource_manager.get_resource(concern_id)
        if not note:
            return False, f"concern {concern_id} not found"
        props = note.get('properties') or {}
        if props.get('kind') not in ('concern', 'agent_concern', 'user_concern'):
            return False, f"{concern_id} is not a concern"
        props['status'] = new_status
        return True, None

    # ------------------------------------------------------------------
    # Fire gate — deterministic, no LLM. Walks active agent_concerns
    # and returns those whose activation has crossed _AGENT_CONCERN_
    # FIRE_THRESHOLD AND have a non-null instruction. Service decrement
    # is applied separately via _service_agent_concern after the
    # autonomous run completes.
    # ------------------------------------------------------------------

    def _check_and_fire_agent_concerns(self) -> List[Tuple[str, str, str]]:
        """Identify agent_concerns ready to fire. Pure arithmetic over
        activation + instruction presence + status. Returns (note_id,
        text, instruction) tuples sorted by activation desc so the
        most-pressured concerns fire first when the per-tick cap
        bites."""
        active = self._iter_active_agent_concerns()
        active.sort(key=lambda t: t[2], reverse=True)
        fired: List[Tuple[str, str, str]] = []
        for nid, note, a in active:
            if a < _AGENT_CONCERN_FIRE_THRESHOLD:
                continue
            props = note.get('properties') or {}
            instruction = props.get('instruction')
            if not instruction:
                continue
            text = str(props.get('content', '') or note.get('text', '') or '').strip()
            fired.append((nid, text, str(instruction)))
        return fired

    # Reflection prompt: extract durable episodic specifics from the latest
    # exchange. Two-stage decision:
    #   (a) Frame check. If the exchange is a hypothetical / role-play /
    #       counterfactual / instructional frame, suppress all writes. The
    #       cost of poisoning memory with frame-bound content is asymmetric
    #       — a bad memory taints every future recall hit on that topic;
    #       a missed real fact just gets re-asserted next time.
    #   (b) For real exchanges, extract memories AND tag each with a
    #       category: fact (default) / preference / commitment.
    # Companion already absorbs personality/style/mood, so the bar for
    # memory is "would NOT be recoverable from the companion model on a
    # fresh re-read" — names, places, commitments, stable preferences.
    _REFLECT_SYS = (
        "You watch chats between {character} and {entity}. Your job has "
        "four stages.\n\n"
        "STAGE 1 — Frame check. Classify the latest exchange as one of:\n"
        "- `hypothetical`: 'imagine that…', 'suppose…', 'what if…'\n"
        "- `roleplay`: user asked {character} to take on a persona, character, or voice\n"
        "- `counterfactual`: discussion of an alternate world / past / scenario\n"
        "- `instructional`: user is teaching {character} how to behave, not stating facts\n"
        "- `none`: a real exchange where statements about {entity}, the world, or "
        "agreements between you carry their literal weight\n"
        "If the frame is anything other than `none`, return all three lists "
        "empty (memories, user_concerns, agent_concerns). When in doubt, "
        "prefer the more conservative classification.\n\n"
        "STAGE 2 — Memories. If frame is `none`, extract stable specifics "
        "that should survive into FUTURE conversations.\n"
        "CAPTURE as memories:\n"
        "- Personal facts the user shared (names, places, relationships) → category=`fact`\n"
        "- Long-running project/work context → category=`fact`\n"
        "- Stable preferences expressed plainly (modifiers, not actions) → category=`preference`\n"
        "- Specific commitments or follow-ups agreed → category=`commitment`\n"
        "- Things {entity} explicitly REJECTED, ruled out, or said they "
        "do NOT want → same categories as above, but set "
        "`polarity`=`negative`. Phrase the text negatively (e.g. "
        "\"User rejected ChromaDB as the vector store\" / \"Does NOT "
        "want emoji in replies\"). Default polarity is `positive`; only "
        "set `negative` for explicit dispreference.\n"
        "SKIP from memories:\n"
        "- Pleasantries, mood, conversational tone (companion handles these).\n"
        "- Anything already in the companion model verbatim.\n"
        "- Anything semantically equivalent to an item shown in "
        "\"## Existing memories\" below — do not re-emit a memory we "
        "already hold. Emit only what is genuinely new or refines a prior "
        "fact in a way the existing wording does not capture.\n"
        "- One-off questions with no stable signal.\n\n"
        "STAGE 3 — User concerns. Topics {entity} is currently preoccupied "
        "with, has surfaced repeatedly, or has explicitly asked {character} "
        "to attend to. DISTINCT from memories: a memory is a stable fact "
        "({entity}'s brother is named Joe); a user_concern is an active "
        "preoccupation ({entity} is investigating concerns redesign).\n"
        "User concerns DO NOT fire autonomously. Their job is to inform "
        "{character}'s responses by surfacing in the prompt when relevant. "
        "Strength decays each turn unless reinforced by the user touching "
        "the topic again — so capture liberally; the runtime prunes what's "
        "not engaged.\n"
        "CAPTURE as user_concerns:\n"
        "- Active investigations / current preoccupations evidenced this turn\n"
        "- Topics user keeps returning to across exchanges\n"
        "- Things user explicitly said they're tracking or thinking about\n"
        "SKIP from user_concerns:\n"
        "- Stable identity facts (those go to memories)\n"
        "- One-off questions with no preoccupation signal\n"
        "- Items already covered by an existing user_concern in the prompt\n"
        "Schema per user_concern: just `text` (≤120 chars). Strength is "
        "set to 1.0 by the runtime — don't include strength in your output.\n\n"
        "STAGE 4 — Agent concerns. {character}'s OWN action queue: things "
        "{character} should be ready to advance. Authored from user "
        "requests with explicit deferred action, OR from seed concerns + "
        "observed context where {character} would benefit from periodic "
        "attention. DISTINCT from user_concerns: agent_concerns drive "
        "{character}'s autonomous action; user_concerns shape responses.\n"
        "Agent concerns FIRE autonomously when their activation grows past "
        "threshold AND they carry an instruction. Activation grows on a "
        "per-concern rhythm (rhythm_hours); each fire decrements activation.\n"
        "CAPTURE as agent_concerns:\n"
        "- User explicitly asked {character} to do/track something ongoing\n"
        "- User stated a deferred action ('remind me about X tomorrow')\n"
        "- {character} noticed a pattern that genuinely warrants recurring "
        "attention (use sparingly — most observations belong in memories or "
        "user_concerns, not as agent_concerns)\n"
        "SKIP from agent_concerns:\n"
        "- Modifiers like 'be brief' / 'don't use emoji' (memories→preferences)\n"
        "- Items already covered by an existing agent_concern in the prompt\n"
        "- Speculative inferences without textual support\n"
        "- Requests the user-driven turn already fulfilled this exchange — "
        "skip them entirely rather than logging an unfireable record\n"
        "Schema per agent_concern:\n"
        "- `text` (≤120 chars): short summary of what the concern is about.\n"
        "- `instruction` (string|null): the procedure {character} executes "
        "when this concern fires. null = the concern is logged for context "
        "but never fires. Usually a one-line imperative, but when the user "
        "DICTATES a multi-step procedure (URL templates, ranges, output "
        "rules, edge cases), capture the FULL spec verbatim — multi-paragraph "
        "instructions are encouraged in that case. The autonomous fire path "
        "passes this string straight to the ReAct loop, so anything you write "
        "here is what {character} will see when she executes. Examples:\n"
        "    \"Search for today's S&P 500 close and summarize the move.\"\n"
        "    \"Pull recent papers on multi-agent coordination from arxiv.\"\n"
        "    \"Query http://192.168.68.56:8086/query?db=pv&q=SELECT last(\\\"value\\\"),"
        " time FROM \\\"voltage\\\" WHERE time > now() - 15m GROUP BY *. Healthy "
        "range 51.5–57.5V. Flag if stale (>10m) or out of range. Silent on "
        "healthy for auto-checks; manual checks always report raw values.\"\n"
        "- `rhythm_hours` (int|null): target fire interval in hours. MUST be "
        "one of {{1, 2, 4, 8, 12, 24, 168}} or null. Pick from the underlying "
        "signal, not how often you'd nag the user:\n"
        "    hourly events (breaking news, intraday): 1 or 2\n"
        "    several-times-a-day work / project: 4 or 8\n"
        "    daily event (S&P close, daily roundup): 12 or 24\n"
        "    weekly check-in (project, hobby): 168\n"
        "    null = no autonomous fire (concern still logs).\n"
        "- `rhythm_source` (\"external\"|\"urgency\"|\"default\"):\n"
        "    `external` if user specified rhythm or topic has natural cadence\n"
        "    `urgency` if user signaled importance ('track this closely')\n"
        "    `default` if you guessed (default to 168 / weekly when guessing)\n"
        "{narrowness_rule}\n\n"
        "Output ONLY this JSON shape — nothing else, no prose:\n"
        "  {{\"frame\": \"<hypothetical|roleplay|counterfactual|instructional|none>\",\n"
        "   \"memories\": [{{\"text\": \"...\", \"category\": \"fact|preference|commitment\", \"polarity\": \"positive|negative\"}}, ...],\n"
        "   \"user_concerns\": [{{\"text\": \"...\"}}, ...],\n"
        "   \"agent_concerns\": [{{\n"
        "     \"text\": \"...\",\n"
        "     \"instruction\": \"<imperative>|null\",\n"
        "     \"rhythm_hours\": <int|null>,\n"
        "     \"rhythm_source\": \"external|urgency|default\"\n"
        "   }}, ...]}}\n\n"
        "WORKED EXAMPLE 1. {entity}: \"Please keep an eye on S&P 500 "
        "closes — I want to hear about them every day.\"\n"
        "Output:\n"
        "{{\"frame\": \"none\", \"memories\": [],\n"
        "  \"user_concerns\": [{{\"text\": \"S&P 500 daily performance\"}}],\n"
        "  \"agent_concerns\": [{{\n"
        "    \"text\": \"Track S&P 500 closing price daily.\",\n"
        "    \"instruction\": \"Search for today's S&P 500 close and summarize the day's move.\",\n"
        "    \"rhythm_hours\": 24,\n"
        "    \"rhythm_source\": \"external\"\n"
        "  }}]}}\n\n"
        "WORKED EXAMPLE 2. {entity}: \"I'm thinking about how concerns and "
        "tasks differ.\" (Just thinking aloud — no action requested.)\n"
        "Output (user_concern only; no agent_concern since no action):\n"
        "{{\"frame\": \"none\", \"memories\": [],\n"
        "  \"user_concerns\": [{{\"text\": \"concerns vs tasks distinction\"}}],\n"
        "  \"agent_concerns\": []}}\n\n"
        "If frame≠none or nothing qualifies: return the envelope with all "
        "three lists empty."
    )

    def _reflect_and_remember(self, source: str) -> Tuple[List[str], List[str], List[str]]:
        """Run a single reflection LLM call over the latest exchange; persist
        memories, user_concerns, agent_concerns. Returns the three written-text
        lists. Failure-tolerant: any error path returns three empty lists."""
        if not self._memories_collection_id:
            return ([], [], [])
        try:
            dialog = self._build_dialog(source, limit=4)
            if not dialog:
                return ([], [], [])
            convo = "\n".join(f"{t['source']}: {t['text']}" for t in dialog)
            companion = self._companion_state.get(source, '').strip()
            # Show the LLM the existing concerns from BOTH collections so it
            # doesn't re-derive ones we already track. Two compact sections,
            # one per collection.
            existing_agent = self._top_active_agent_concerns(n=10)
            existing_user = self._top_active_user_concerns(n=10)
            # Memory neighbors near the dialog topic — used by the LLM to
            # avoid re-emitting a memory we already hold (write-time
            # dedupe). Threshold lower than auto-RAG (0.4 vs 0.5) and
            # k larger (8 vs 3) because false-positives here suppress a
            # write that *might* be redundant; false-negatives produce a
            # duplicate. Suppression is the cheaper failure mode.
            existing_memories = self._recall(convo, k=8, threshold=0.4)
            sys_msg = self._REFLECT_SYS.format(
                character=self.character_name, entity=source,
                narrowness_rule=_CONCERN_INSTRUCTION_NARROWNESS_RULE)
            user_parts = []
            if companion:
                user_parts.append(
                    "## Existing companion model (do NOT re-extract from this; "
                    "use only to avoid duplicates)\n" + companion)
            if existing_memories:
                mem_lines = []
                for text, cat, pol in existing_memories:
                    marker = '[avoid] ' if pol == 'negative' else ''
                    mem_lines.append(f"- ({cat}) {marker}{text}")
                user_parts.append(
                    "## Existing memories (do NOT re-emit anything "
                    "semantically equivalent to these; emit only NEW or "
                    "genuinely-refining memories)\n" + "\n".join(mem_lines))
            if existing_user:
                lines = [f"- {text}" for _nid, text, _s, _p in existing_user]
                user_parts.append(
                    "## Existing user_concerns (do NOT re-emit; emit only "
                    "NEW user_concerns this exchange surfaced)\n" + "\n".join(lines))
            if existing_agent:
                lines = [f"- {text}" for _nid, text, _a, _p in existing_agent]
                user_parts.append(
                    "## Existing agent_concerns (do NOT re-emit; emit only "
                    "NEW agent_concerns this exchange surfaced)\n" + "\n".join(lines))
            user_parts.append("## Latest exchange\n" + convo)
            user_parts.append(
                "Return the JSON object now (keys: frame, memories, "
                "user_concerns, agent_concerns). All lists empty if frame≠none "
                "or nothing qualifies.")
            result = self._llm_generate(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': "\n\n".join(user_parts)}],
                max_tokens=8192, temperature=0.3, is_json=True,
                cot_profile='none')
            if not result.success:
                return ([], [], [])
            payload = result.text
            try:
                _preview = json.dumps(payload) if not isinstance(payload, str) else payload
            except Exception:
                _preview = repr(payload)
            logger.info(f"[{self.character_name}] reflection raw: {_preview[:800]}")
            if isinstance(payload, str):
                payload = repair_json_string(payload)
                if payload is None:
                    return ([], [], [])

            frame, raw_memories, raw_user_concerns, raw_agent_concerns = (
                self._parse_reflection_payload(payload))

            if frame != _REFLECT_FRAME_OK:
                logger.info(
                    f"[{self.character_name}] reflection suppressed "
                    f"(frame={frame!r}) — nothing written")
                return ([], [], [])

            mems_written: List[str] = []
            for text, category, polarity in raw_memories:
                if len(text) > 240:
                    text = text[:240].rstrip()
                if self._remember(text, entity=source, category=category,
                                  polarity=polarity):
                    mems_written.append(text)

            user_cons_written: List[str] = []
            for text in raw_user_concerns:
                if len(text) > 240:
                    text = text[:240].rstrip()
                if self._add_user_concern(text, entity=source):
                    user_cons_written.append(text)

            agent_cons_written: List[str] = []
            for c in raw_agent_concerns:
                text = c.get('text', '')
                if len(text) > 240:
                    text = text[:240].rstrip()
                if self._add_agent_concern(
                        text, entity=source,
                        provenance='asserted',
                        seed=False,
                        rhythm_hours=c.get('rhythm_hours'),
                        rhythm_source=c.get('rhythm_source') or 'default',
                        instruction=c.get('instruction')):
                    agent_cons_written.append(text)

            if mems_written or user_cons_written or agent_cons_written:
                logger.info(
                    f"[{self.character_name}] reflection wrote "
                    f"{len(mems_written)} memory(s), "
                    f"{len(user_cons_written)} user_concern(s), "
                    f"{len(agent_cons_written)} agent_concern(s) from {source}")
            return (mems_written, user_cons_written, agent_cons_written)
        except Exception as e:
            logger.warning(f"[{self.character_name}] _reflect_and_remember failed: {e}")
            return ([], [], [])

    @staticmethod
    def _parse_reflection_payload(
            payload: Any
    ) -> Tuple[str, List[Tuple[str, str, str]], List[str], List[Dict[str, Any]]]:
        """Normalize reflection output to (frame, memories, user_concerns,
        agent_concerns).
        memories: list of (text, category, polarity) tuples. Polarity is
                  'positive' (default) or 'negative'.
        user_concerns: list of text strings (no fields beyond text).
        agent_concerns: list of dicts with keys text, instruction,
                        rhythm_hours, rhythm_source (any may be missing/None).

        Accepts:
          - New envelope: {"frame", "memories", "user_concerns", "agent_concerns"}
          - Legacy envelope with single "concerns" key (single-channel design):
            all concerns are routed to agent_concerns; cadence_hours field is
            read as rhythm_hours.
          - Bare list of strings: assumed frame=none, treated as memories.
        Anything else returns ('unknown', [], [], []) — caller treats
        non-`none` frame as suppression so this fails safe.
        """

        def _normalize_memories(raw: Any) -> List[Tuple[str, str, str]]:
            out: List[Tuple[str, str, str]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append((item.strip(), 'fact', 'positive'))
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    c = str(item.get('category', 'fact') or 'fact').strip().lower()
                    if c not in _MEMORY_CATEGORIES:
                        c = 'fact'
                    p = str(item.get('polarity', 'positive') or 'positive').strip().lower()
                    if p not in ('positive', 'negative'):
                        p = 'positive'
                    if t:
                        out.append((t, c, p))
            return out

        def _normalize_user_concerns(raw: Any) -> List[str]:
            out: List[str] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append(item.strip())
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    if t:
                        out.append(t)
            return out

        def _normalize_agent_concerns(raw: Any) -> List[Dict[str, Any]]:
            out: List[Dict[str, Any]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append({'text': item.strip(), 'instruction': None,
                                'rhythm_hours': None, 'rhythm_source': 'default'})
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    if not t:
                        continue
                    instr = item.get('instruction')
                    rhythm_h = item.get('rhythm_hours')
                    if rhythm_h is None:
                        rhythm_h = item.get('cadence_hours')   # legacy field
                    if rhythm_h is None and item.get('cadence_days') is not None:
                        try:
                            rhythm_h = float(item.get('cadence_days')) * 24.0
                        except (TypeError, ValueError):
                            rhythm_h = None
                    src = str(item.get('rhythm_source', 'default') or 'default').strip().lower()
                    if src not in ('external', 'urgency', 'default'):
                        src = 'default'
                    out.append({
                        'text': t,
                        'instruction': str(instr).strip() if instr else None,
                        'rhythm_hours': rhythm_h,
                        'rhythm_source': src,
                    })
            return out

        # Old shape — bare list. Assume frame=none, all memories.
        if isinstance(payload, list):
            return (_REFLECT_FRAME_OK, _normalize_memories(payload), [], [])

        if isinstance(payload, dict):
            frame = str(payload.get('frame', '') or '').strip().lower() or 'unknown'
            mems = _normalize_memories(payload.get('memories', []))
            user_cons = _normalize_user_concerns(payload.get('user_concerns', []))
            # Prefer the new agent_concerns field; fall back to legacy
            # single-channel `concerns` (route to agent collection).
            agent_cons_raw = payload.get('agent_concerns')
            if agent_cons_raw is None:
                agent_cons_raw = payload.get('concerns', [])
            agent_cons = _normalize_agent_concerns(agent_cons_raw)
            return (frame, mems, user_cons, agent_cons)

        return ('unknown', [], [], [])

    # ------------------------------------------------------------------
    # Zenoh wiring
    # ------------------------------------------------------------------

    def _open_zenoh(self) -> None:
        import zenoh
        from utils.zenoh_utils import make_localhost_config

        self._zenoh_session = zenoh.open(make_localhost_config())
        self._affect.attach_session(self._zenoh_session)
        self._canvas.attach_session(self._zenoh_session)
        self._action_pub = self._zenoh_session.declare_publisher(
            f"cognitive/{self.character_name}/action"
        )
        self._sense_sub = self._zenoh_session.declare_subscriber(
            f"cognitive/{self.character_name}/sense_data",
            self._on_sense_data,
        )

        # Resource queryables for resource_browser. Chat mode now exposes
        # Notes, Collections, AND concerns (graph still executive-only).
        self._resources_list_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resources",
            self._handle_resources_list_query,
        )
        self._resource_remove_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resource/remove/*",
            self._handle_resource_remove_query,
        )
        self._resource_update_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resource/update/*",
            self._handle_resource_update_query,
        )
        # Single-chunk wildcard — does not match resource/remove/* or
        # resource/update/* (those have an extra path segment).
        self._resource_by_id_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/resource/*",
            self._handle_resource_by_id_query,
        )
        # Concerns queryables. The browser frontend already expects the
        # executive's shape (user_concerns + derived_concerns + activations);
        # _handle_concerns_query adapts chat's flat concerns collection to
        # that shape so the existing UI works unchanged.
        self._concerns_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/concerns",
            self._handle_concerns_query,
        )
        self._concern_manage_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/control/concern_manage",
            self._handle_concern_manage_query,
        )
        # Direct subagent invocation — bypasses Jill's ReAct loop. Used by
        # `/recall <query>` in the CLI to test/inspect the active-recall
        # subagent without spending a full Jill turn.
        self._remember_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/recall",
            self._handle_remember_query,
        )
        # External-repo binding control. CLI emits set / clear / get;
        # response carries the resolved path or an error.
        self._external_repo_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/control/external_repo",
            self._handle_external_repo_query,
        )
        # Status (/status): is Jill ready for new input, currently
        # processing a turn, or running autonomous work?
        self._status_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/status",
            self._handle_status_query,
        )

        logger.info(
            f"[{self.character_name}] chat zenoh ready "
            f"(in=cognitive/{self.character_name}/sense_data, "
            f"out=cognitive/{self.character_name}/action, "
            f"backend={self.backend.server}@{self.backend.base_url}, "
            f"discourse={self.discourse_enabled}, orientation={self.orientation_enabled})"
        )

    def _on_sense_data(self, sample) -> None:
        try:
            raw = bytes(sample.payload).decode('utf-8', errors='replace')
            envelope = json.loads(raw)
        except Exception as e:
            logger.warning(f"[{self.character_name}] sense_data decode failed: {e}")
            return

        if envelope.get('mode') != 'text':
            return

        content = envelope.get('content')
        source = "User"
        text = ""
        close = False
        image_url: Optional[str] = None
        if isinstance(content, str):
            try:
                inner = json.loads(content)
                source = inner.get('source', source)
                text = inner.get('text', '')
                close = bool(inner.get('close', False))
                img = inner.get('image')
                if isinstance(img, dict):
                    url = img.get('url')
                    if isinstance(url, str) and url:
                        image_url = url
            except Exception:
                text = content
        elif isinstance(content, dict):
            source = content.get('source', source)
            text = content.get('text', '')
            close = bool(content.get('close', False))
            img = content.get('image')
            if isinstance(img, dict):
                url = img.get('url')
                if isinstance(url, str) and url:
                    image_url = url

        # Sensor dispatch: source is e.g. 'sensor:tick'. Recognized sensors
        # become typed events on the inbox; unknown sensors fall through to
        # the empty-text drop below.
        if isinstance(source, str) and source == 'sensor:tick':
            self._inbox.put({'kind': 'tick'})
            return

        if not text and not close and not image_url:
            return

        msg: Dict[str, Any] = {'kind': 'user', 'source': source, 'text': text, 'close': close}
        if image_url:
            msg['image_url'] = image_url
        self._inbox.put(msg)

    def _publish_say(self, text: str) -> None:
        if not self._action_pub:
            return
        payload = {
            'type': 'say',
            'source': 'assistant',
            'text': text,
            'timestamp': datetime.now(timezone.utc).isoformat(),
        }
        try:
            self._action_pub.put(json.dumps(payload).encode('utf-8'))
        except Exception as e:
            logger.warning(f"[{self.character_name}] publish failed: {e}")

    # ------------------------------------------------------------------
    # Web search — backend for the ReAct `search` tool. Persists each
    # synthesis as a named Note so it appears in /resources.
    # ------------------------------------------------------------------

    @staticmethod
    def _search_note_name(query: str) -> str:
        slug = re.sub(r'[^a-z0-9]+', '-', query.lower()).strip('-')
        if len(slug) > 60:
            slug = slug[:60].rstrip('-')
        return f"search:{slug}" if slug else "search:result"

    def _get_llm_search(self):
        """Lazy-load llm_search from src/tools/search-web/tool.py. Cached on
        the instance after first load. Hyphenated package directory rules
        out plain `import`, so we use importlib by file path — same pattern
        executive_node uses for tool dispatch."""
        if hasattr(self, '_llm_search_cached'):
            return self._llm_search_cached
        try:
            import importlib.util
            tool_path = os.path.join(_SRC_DIR, "tools", "search-web", "tool.py")
            spec = importlib.util.spec_from_file_location(
                "_chat_search_web_tool", tool_path)
            if spec is None or spec.loader is None:
                self._llm_search_cached = None
                return None
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._llm_search_cached = getattr(mod, 'llm_search', None)
        except Exception as e:
            logger.warning(f"[{self.character_name}] could not load search tool: {e}")
            self._llm_search_cached = None
        return self._llm_search_cached

    def _run_web_search(self, query: str) -> Optional[Dict[str, Any]]:
        """Run a single web search and persist the synthesis as a named
        Note. Returns {synthesis, sources, query} on success, None on
        failure (missing API key, no results, network error, etc)."""
        llm_search = self._get_llm_search()
        if llm_search is None:
            return None
        try:
            result = llm_search(query=query, wall_time_limit=90.0)
        except Exception as e:
            logger.warning(f"[{self.character_name}] web search raised: {e}")
            return None
        if not result or not result.get('synthesis'):
            return None

        synthesis = str(result['synthesis'])
        sources = result.get('sources', []) or []

        try:
            success, note_id, err, _ = self.resource_manager.create_note(
                self.character_name, synthesis, "text",
                "search-web", query, self._search_note_name(query),
                {
                    'kind': 'web_search',
                    'query': query,
                    'sources': sources,
                    'source_count': len(sources),
                    'model': result.get('_model', ''),
                },
            )
            if success and note_id:
                self.resource_manager.mark_persistent(note_id, self.character_name)
            elif err:
                logger.warning(f"[{self.character_name}] persist search note failed: {err}")
        except Exception as e:
            logger.warning(f"[{self.character_name}] persist search note failed: {e}")

        return {'synthesis': synthesis, 'sources': sources, 'query': query}

    # ------------------------------------------------------------------
    # fetch_text — backend for the ReAct `fetch_text` tool. Loaded the
    # same way as search-web (importlib by file path, hyphenated dir).
    # ------------------------------------------------------------------

    def _get_fetch_text_tool(self):
        """Lazy-load the fetch-text tool's `tool` callable. Cached on the
        instance after first load. The executive_node tool expects an
        `executor` kwarg with `_create_uniform_return`; we pass a stub
        since chat doesn't run an executor."""
        if hasattr(self, '_fetch_text_cached'):
            return self._fetch_text_cached
        try:
            import importlib.util
            tool_path = os.path.join(_SRC_DIR, "tools", "fetch-text", "tool.py")
            spec = importlib.util.spec_from_file_location(
                "_chat_fetch_text_tool", tool_path)
            if spec is None or spec.loader is None:
                self._fetch_text_cached = None
                return None
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._fetch_text_cached = getattr(mod, 'tool', None)
        except Exception as e:
            logger.warning(f"[{self.character_name}] could not load fetch-text tool: {e}")
            self._fetch_text_cached = None
        return self._fetch_text_cached

    class _FetchTextStubExecutor:
        """Minimal stand-in for InfospaceExecutor — fetch-text only calls
        `_create_uniform_return`, so this is sufficient."""
        @staticmethod
        def _create_uniform_return(status, value=None, reason=None,
                                   resource_id=None, extra=None, data=None):
            return {'status': status, 'value': value, 'reason': reason,
                    'resource_id': resource_id, 'extra': extra, 'data': data}

    def _run_fetch_text(self, url: str) -> str:
        """Fetch+extract text from a URL (or Note/Collection ID, or local
        absolute path). Returns an OK:/EMPTY:/ERROR: prefixed observation
        per the ReAct tool-observation convention (see _run_react_loop
        comment block). Successful text is capped to _FETCH_TEXT_OBS_CAP
        chars for the ReAct working log."""
        if not url:
            return 'EMPTY: fetch_text url was empty'
        fetch = self._get_fetch_text_tool()
        if fetch is None:
            return 'ERROR: fetch_text tool unavailable (failed to load — see warning log)'
        try:
            result = fetch(url, executor=self._FetchTextStubExecutor,
                           resource_manager=self.resource_manager)
        except Exception as e:
            logger.warning(f"[{self.character_name}] fetch_text raised: {e}")
            return f'ERROR: fetch_text raised: {e}'
        if not isinstance(result, dict):
            return 'ERROR: fetch_text returned unexpected shape'
        if result.get('status') != 'success':
            return f"ERROR: fetch_text failed: {result.get('reason') or 'unknown'}"
        # The tool returns a JSON-encoded blob in `value` and a parsed dict
        # in `extra`. Prefer `extra` for the text field.
        extra = result.get('extra') or {}
        text = ''
        if isinstance(extra, dict):
            text = str(extra.get('text') or '').strip()
        if not text:
            value = result.get('value') or ''
            try:
                parsed = json.loads(value) if isinstance(value, str) else value
                if isinstance(parsed, dict):
                    text = str(parsed.get('text') or '').strip()
                elif isinstance(parsed, str):
                    text = parsed.strip()
            except Exception:
                text = str(value).strip()
        if not text:
            return f'EMPTY: fetch_text extracted no text from {url}'
        if len(text) > _FETCH_TEXT_OBS_CAP:
            text = text[:_FETCH_TEXT_OBS_CAP].rstrip() + f"\n…[truncated at {_FETCH_TEXT_OBS_CAP} chars]"
        return 'OK: ' + text

    # ------------------------------------------------------------------
    # ReAct loop — single-action-per-iteration tool use for chat.
    #
    # Tools: process_text (LLM transformation), search (web), respond (exit).
    # Each emit is a single JSON object; results auto-bind to $step1, $step2…
    # Variable scope is per-turn; nothing leaks into conversation history.
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_react_action(raw: str) -> Optional[Dict[str, Any]]:
        """Extract a single JSON action object from LLM output via the
        shared tolerant parser (fences, missing braces, bareword keys,
        etc). Returns None if no dict can be recovered."""
        if not raw or not isinstance(raw, str):
            return None
        obj = repair_json_string(raw)
        return obj if isinstance(obj, dict) else None

    @staticmethod
    def _resolve_react_value(val: Any, log: List[Tuple[str, str]]) -> str:
        """Literal string passes through; `$stepN` looks up the log.

        Strips the OK:/EMPTY:/ERROR: observation prefix from bound values
        on resolve. Rationale: the prefix is informational signal for the
        LLM reading the working log (success vs failure discrimination),
        but it should not appear in downstream substitutions — when the
        agent does `respond.text = "$step2"`, the user shouldn't see
        "OK: <content>" leaking through. The LLM has already discriminated
        in the log; substitution gives clean content."""
        if not isinstance(val, str):
            return str(val) if val is not None else ''
        if _REACT_BINDING_RE.match(val):
            content = ''
            for label, c in log:
                if label == val:
                    content = c
                    break
            for tag in ('OK: ', 'EMPTY: ', 'ERROR: '):
                if content.startswith(tag):
                    return content[len(tag):]
            return content
        return val

    def _diagnose_process_text_args(self, raw_src: Any, resolved_src: str,
                                    instruction: Any,
                                    log: List[Tuple[str, str]]
                                    ) -> Optional[str]:
        """Return a diagnostic reason string when process_text args look
        malformed — section-name placeholder as `source`, unresolved
        `$stepN` binding, or empty fields. Returns None when args are
        usable. Caller (the ReAct dispatch) wraps the returned reason
        with the ERROR: prefix, so emit naked sentences here. Each
        diagnostic names the failure AND points at the recovery path —
        the model has been observed to recover cleanly when given a
        legible error."""
        if not isinstance(instruction, str) or not instruction.strip():
            return ("requires a non-empty `instruction` field; "
                    "no instruction was supplied.")
        # Unresolved $stepN: model passed a binding that doesn't exist.
        if isinstance(raw_src, str) and _REACT_BINDING_RE.match(raw_src) and not resolved_src:
            bound = sorted({lab for lab, _ in log
                            if _REACT_BINDING_RE.match(lab)})
            avail = ", ".join(bound) if bound else "(none yet this turn)"
            return (f"`source` is `{raw_src}`, an unresolved binding. "
                    f"Available `$stepN` bindings this turn: {avail}. "
                    f"Pass an existing binding or literal inline text instead.")
        if not resolved_src:
            return ("`source` is empty. Pass either literal text content "
                    "(the actual material to process) or a `$stepN` binding "
                    "from a prior tool call this turn.")
        # Heading-shaped placeholder: starts with `##` and is short.
        # Real content the model wants processed is typically much
        # longer; a section heading is rarely a reasonable source.
        stripped = resolved_src.lstrip()
        if stripped.startswith('##') and len(resolved_src) < 500:
            return ("`source` appears to be a section heading like "
                    "`## Conversation history` rather than content. "
                    "Section bodies are already in your system prompt — "
                    "read them directly in `respond` rather than passing "
                    "the heading as `source`. Section names do not "
                    "resolve to their bodies; `source` accepts only "
                    "literal inline text or a `$stepN` binding.")
        return None

    def _run_process_text(self, source_text: str, instruction: str) -> str:
        """Apply a focused LLM pass to source_text using the given instruction.
        Used by the ReAct process_text tool.

        The persona block is passed as system context so output respects
        Jill's voice (fair witness, no sycophancy, grounded specifics). The
        citation rule is added explicitly because the persona doesn't
        codify it: when the source contains a 'Sources:' block (the format
        web-search results use), the output must cite by domain or
        publication name rather than 'the source' / 'the writeup'.
        """
        has_sources = isinstance(source_text, str) and 'Sources:' in source_text
        sys_parts = [
            f"You are {self.character_name}. The user has asked you to apply an "
            "instruction to a source text. Output ONLY the transformed result — "
            "no preamble, no commentary, no headers, no markdown fences."
        ]
        if self.persona:
            sys_parts.append("## Persona (governs voice and stance)\n" + self.persona)
        if has_sources:
            sys_parts.append(
                "## Citation requirement\n"
                "The source contains a 'Sources:' block. Cite by real "
                "publication / domain when available (e.g. \"per weather.com\", "
                "\"according to NOAA\", \"per ir.tesla.com\"). Never use "
                "generic phrases like \"the source\" or \"the writeup\". If "
                "a source line lacks a domain or carries '(no public URL)', "
                "do NOT invent a domain and do NOT forward the raw URL — say "
                "something honest like \"per the weather service consulted\" "
                "or attribute by the source title. Never paste an opaque "
                "internal reference (e.g. weather://turn0forecast0) into "
                "your output. If the source did not address the instruction, "
                "say so plainly rather than improvise."
            )
        sys_msg = "\n\n".join(sys_parts)
        user_msg = f"INSTRUCTION:\n{instruction}\n\nSOURCE:\n{source_text}"
        try:
            result = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=4096, temperature=0.4, cot_profile='none',
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] process_text failed: {e}")
            return f"ERROR: process_text raised: {e}"
        text = (result or '').strip()
        if not text:
            return 'EMPTY: process_text produced no output'
        return 'OK: ' + text

    @staticmethod
    def _format_react_search_observation(search_result: Dict[str, Any]) -> str:
        """Render a search result (synthesis + sources) into the scratchpad
        observation text. Sources capped to keep prompt size bounded.

        Filters opaque internal references (non-http URLs like OpenAI
        Responses API's `weather://turn0forecast0` grounding refs) — the
        URL is dropped and the scheme-as-domain (`weather`) is suppressed
        so downstream citation doesn't forward garbage."""
        synthesis = search_result.get('synthesis', '') or ''
        sources = search_result.get('sources', []) or []
        src_lines = []
        for s in sources[:8]:
            domain = s.get('domain') or ''
            url = s.get('url') or ''
            title = s.get('title') or ''
            is_real_url = url.startswith(('http://', 'https://'))
            if not is_real_url:
                url = ''
                # A "domain" with no dot is usually just the URL scheme
                # of an opaque ref (e.g. "weather"); not a real publication.
                if domain and '.' not in domain:
                    domain = ''
            if url and domain:
                src_lines.append(f"- {domain}: {title} ({url})")
            elif domain:
                src_lines.append(f"- {domain}: {title}")
            elif title:
                src_lines.append(f"- {title} (no public URL)")
        if not src_lines:
            return synthesis
        return f"{synthesis}\n\nSources:\n" + "\n".join(src_lines)

    # ------------------------------------------------------------------
    # Resource queryable handlers (chat-mode subset of executive_node)
    # ------------------------------------------------------------------

    @staticmethod
    def _json_safe_resource(resource: Dict[str, Any]) -> Dict[str, Any]:
        rc = resource.copy()
        if 'type' in rc:
            rc['type'] = str(rc['type'])
        if 'location' in rc and isinstance(rc['location'], tuple):
            rc['location'] = list(rc['location'])
        if 'properties' in rc and isinstance(rc['properties'], dict):
            props = rc['properties'].copy()
            for k, v in props.items():
                if isinstance(v, datetime):
                    props[k] = v.isoformat()
            rc['properties'] = props
        return rc

    def _reply(self, query, payload: Dict[str, Any]) -> None:
        try:
            query.reply(query.key_expr, json.dumps(payload).encode('utf-8'))
        except Exception as e:
            logger.warning(f"[{self.character_name}] query reply failed: {e}")

    def _handle_resources_list_query(self, query) -> None:
        try:
            resources = self.resource_manager.get_resource_list()
            self._reply(query, {
                'success': True,
                'resources': [self._json_safe_resource(r) for r in resources],
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] resources list query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_resource_by_id_query(self, query) -> None:
        try:
            resource_id = str(query.key_expr).split('/')[-1]
            r = self.resource_manager.get_resource(resource_id)
            if not r:
                self._reply(query, {'success': False, 'error': f'Resource {resource_id} not found'})
                return
            self._reply(query, {'success': True, 'resource': self._json_safe_resource(r)})
        except Exception as e:
            logger.error(f"[{self.character_name}] resource by id query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_resource_remove_query(self, query) -> None:
        try:
            resource_id = str(query.key_expr).split('/')[-1]
            success, err = self.resource_manager.delete_resource(resource_id)
            if success:
                self._reply(query, {'success': True, 'message': f'Resource {resource_id} deleted'})
                self._persist_to_disk()
            else:
                self._reply(query, {'success': False, 'error': err or f'Failed to delete {resource_id}'})
        except Exception as e:
            logger.error(f"[{self.character_name}] resource remove query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_resource_update_query(self, query) -> None:
        try:
            resource_id = str(query.key_expr).split('/')[-1]
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            new_content = params.get('content', '')
            success, err = self.resource_manager.update_note_content(resource_id, new_content)
            if success:
                self._reply(query, {'success': True, 'message': f'Note {resource_id} updated'})
                self._persist_to_disk()
            else:
                self._reply(query, {'success': False, 'error': err or f'Failed to update {resource_id}'})
        except Exception as e:
            logger.error(f"[{self.character_name}] resource update query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    # ------------------------------------------------------------------
    # Concern queryables (browser-facing).
    # The Resource Browser frontend expects the executive's shape
    # ({user_concerns: [...], derived_concerns: [...], activations: {}});
    # we adapt chat's single concerns collection to that split:
    #   - user_concerns  = category in (one_shot, durable). Status remap:
    #                      active→open, satisfied→closed, abandoned→abandoned
    #                      so the UI's user-concern color/label code (which
    #                      checks "open"/"closed") works unchanged.
    #   - derived_concerns = category == derived. Status passthrough.
    # ------------------------------------------------------------------

    _USER_CONCERN_STATUS_MAP = {'active': 'open', 'satisfied': 'closed',
                                'abandoned': 'abandoned'}

    def _serialize_concern(self, note_id: str, note: Dict[str, Any],
                           is_user_kind: bool) -> Dict[str, Any]:
        """Build a dict matching the field names the resource_browser
        frontend looks for. The browser falls back gracefully through
        concern_description || description, concern_label || name, etc.,
        so we populate both forms to keep the UI working."""
        props = note.get('properties') or {}
        text = str(props.get('content', '') or '')
        status = props.get('status', 'active')
        if is_user_kind:
            status = self._USER_CONCERN_STATUS_MAP.get(status, status)
        # Compact label for the list-row heading: first line, capped.
        first_line = text.split('\n', 1)[0].strip()
        if len(first_line) > 60:
            label = first_line[:60].rstrip() + '…'
        else:
            label = first_line or note_id
        return {
            # IDs (browser checks concern_id first, falls back to id)
            'concern_id': note_id,
            'id': note_id,
            # Headings (browser checks concern_label || name)
            'concern_label': label,
            'name': label,
            # Body text (browser checks concern_description || description)
            'concern_description': text,
            'description': text,
            # State + lifecycle
            'category': props.get('category', 'durable'),  # legacy field; may be empty for new notes
            'kind': props.get('kind', 'concern'),
            'status': status,
            # Browser displays a single 'weight' bar — pick activation
            # for agent_concerns, strength for user_concerns. Both are
            # in [0,1] so the visual scale is consistent.
            'weight': float(
                (props.get('activation') if props.get('kind') == 'agent_concern'
                 else props.get('strength') if props.get('kind') == 'user_concern'
                 else 0.0) or 0.0),
            'activation': props.get('activation'),
            'strength': props.get('strength'),
            'provenance': props.get('provenance', 'asserted'),
            'origin': 'seed' if props.get('seed') else 'reflection',
            'seed': bool(props.get('seed', False)),
            'entity': props.get('entity', ''),
            'created_at': props.get('created_at', ''),
            'created': props.get('created_at', ''),
            'last_engaged_at': props.get('last_engaged_at', ''),
            'last_bumped_at': props.get('last_bumped_at', ''),
            'recency': (props.get('last_bumped_at')
                        or props.get('last_engaged_at', '')),
            # Firing parameters (agent_concerns only; user_concerns ignore).
            # rhythm_hours editable via concern_manage set_rhythm_hours.
            'rhythm_hours': self._resolve_rhythm_hours(props),
            'rhythm_source': props.get('rhythm_source', 'default'),
            'rhythm_hours_allowed': list(_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED),
            # Legacy aliases retained so the browser UI keeps rendering
            # while we refactor it in item 7.
            'cadence_hours': self._resolve_rhythm_hours(props),
            'cadence_hours_allowed': list(_AGENT_CONCERN_RHYTHM_HOURS_ALLOWED),
            'lifetime_days': None,
            'instruction': props.get('instruction'),
            'last_acted_at': (props.get('last_fired_at')
                              or props.get('last_acted_at')   # legacy
                              or None),
            'last_fired_at': props.get('last_fired_at'),
        }

    def _all_concerns_split(self) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """Iterate both concern collections for the browser API. Returns
        (user_concerns, agent_concerns) — keyed at the API layer as
        user_concerns / derived_concerns for back-compat with the
        existing UI tabs (rename to come in item 7). No status filter:
        browser shows all states."""
        user_out: List[Dict[str, Any]] = []
        agent_out: List[Dict[str, Any]] = []
        if self._user_concerns_collection_id:
            coll = self.resource_manager.get_resource(self._user_concerns_collection_id)
            if coll:
                for nid in (coll.get('properties') or {}).get('content', []) or []:
                    note = self.resource_manager.get_resource(nid)
                    if not note:
                        continue
                    user_out.append(self._serialize_concern(nid, note, is_user_kind=True))
        if self._agent_concerns_collection_id:
            coll = self.resource_manager.get_resource(self._agent_concerns_collection_id)
            if coll:
                for nid in (coll.get('properties') or {}).get('content', []) or []:
                    note = self.resource_manager.get_resource(nid)
                    if not note:
                        continue
                    agent_out.append(self._serialize_concern(nid, note, is_user_kind=False))
        return (user_out, agent_out)

    def _handle_remember_query(self, query) -> None:
        """Direct invocation of the active-recall subagent. Payload:
        {"query": "<natural-language question>"}. Returns {"success": bool,
        "answer": "<synthesized answer>", "trace_dir": "<path>"}. Bypasses
        Jill's ReAct loop — used by `/recall` in the CLI for test/inspect."""
        try:
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            q_text = str(params.get('query', '') or '').strip()
            if not q_text:
                self._reply(query, {'success': False, 'error': 'empty query'})
                return
            answer = self._run_remember(q_text)
            self._reply(query, {
                'success': True,
                'answer': answer,
                'trace_dir': str(self._subagent_traces_dir()),
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] remember query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_status_query(self, query) -> None:
        """Report whether Jill is ready for new input. Payload is empty
        (or {}). Response shape:
          {
            success: True,
            ready: bool,                # True iff no turn currently in flight
            action: str,                # human-readable description
            current_turn: dict | null,  # kind/source/text_preview/started_at
            post_turn_busy: bool,       # discourse + reflection still running
            inbox_backlog: int,         # queued user/tick items
            last_reply_at: str | null,  # ISO timestamp of last published reply
            character: str,
          }
        ready=True does NOT require post-turn reflection to have finished —
        Jill accepts new input while reflection runs in parallel. The
        /status caller can render post_turn_busy as informational context."""
        try:
            ct = self._current_turn  # snapshot the dict reference
            ready = ct is None
            if ct is not None:
                kind = ct.get('kind')
                src = ct.get('source', '?')
                if kind == 'autonomous':
                    cid = ct.get('autonomous_concern_id') or '(no concern_id)'
                    action = f"running autonomous turn ({cid})"
                else:
                    action = f"processing user input from {src}"
            elif self._post_turn_busy:
                action = "idle (post-turn reflection still running)"
            else:
                action = "idle"
            try:
                backlog = self._inbox.qsize()
            except Exception:
                backlog = 0
            # Canonical log directory. All in-process loggers and
            # subprocess loggers (resource_browser, fastapi_action_display,
            # discourse, llm_api, executive_node via templates) are pinned
            # to <repo>/logs/ — see launcher.py / discourse.py / etc.
            # Single line, one place to look.
            log_dir = str(
                Path(__file__).resolve().parent.parent.parent / 'logs')
            self._reply(query, {
                'success': True,
                'ready': ready,
                'action': action,
                'current_turn': ct,
                'post_turn_busy': self._post_turn_busy,
                'inbox_backlog': backlog,
                'last_reply_at': self._last_reply_at,
                'character': self.character_name,
                'log_dir': log_dir,
                'backend': {
                    'server': self.backend.server,
                    'base_url': self.backend.base_url,
                    'model': self.backend.model,
                },
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] status query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_external_repo_query(self, query) -> None:
        """Bind / unbind / inspect the external-repo binding for this
        chat session. Payload: {"action": "set|clear|get", "path": "..."}.
        Responses:
          set  → {success, path}                  on success
                 {success: false, error: "..."}  on failure (e.g. bad path)
          clear → {success, was_bound: bool, path: <prior or null>}
          get   → {success, path: <str or null>}"""
        try:
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            action = str(params.get('action') or '').strip().lower()
            if action == 'set':
                path_arg = str(params.get('path') or '').strip()
                if not path_arg:
                    self._reply(query, {'success': False, 'error': 'missing path'})
                    return
                ok, msg = self._set_external_repo(path_arg, persist=True)
                if ok:
                    self._reply(query, {'success': True, 'path': msg})
                else:
                    self._reply(query, {'success': False, 'error': msg})
            elif action == 'clear':
                prior = self._external_repo
                was_bound = self._clear_external_repo()
                self._reply(query, {
                    'success': True,
                    'was_bound': was_bound,
                    'path': str(prior) if prior else None,
                })
            elif action == 'get':
                bound = self._get_external_repo()
                self._reply(query, {
                    'success': True,
                    'path': str(bound) if bound else None,
                })
            else:
                self._reply(query, {
                    'success': False,
                    'error': f'unknown action {action!r}; expected set|clear|get',
                })
        except Exception as e:
            logger.error(f"[{self.character_name}] external_repo query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_concerns_query(self, query) -> None:
        try:
            user_concerns, derived_concerns = self._all_concerns_split()
            self._reply(query, {
                'success': True,
                'user_concerns': user_concerns,
                'derived_concerns': derived_concerns,
                'activations': {},  # chat has no activation system
            })
        except Exception as e:
            logger.error(f"[{self.character_name}] concerns query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    def _handle_concern_manage_query(self, query) -> None:
        """Browser → chat: status transitions, cadence edit, hard delete,
        bulk wipe.
        Payload: {"concern_id": "Note_X", "action": "close|reopen|satisfy|
                  abandon|activate|delete|set_cadence_hours|wipe_non_seed",
                  "type": "user|derived",
                  "value": <int or null>}  # required for set_cadence_hours

        The action vocabulary mirrors what resource_browser.py emits.
        `delete` is a hard delete (the browser confirms with a dialog
        first); status actions go through _set_concern_status;
        set_cadence_hours snaps the value to the allowed bucket and
        writes to the concern note. `wipe_non_seed` hard-deletes every
        concern whose `seed` property is False and does not require a
        concern_id."""
        try:
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            action = str(params.get('action', '') or '').strip().lower()
            if not action:
                self._reply(query, {'success': False,
                                    'error': "missing action"})
                return

            # Bulk wipe — does not require concern_id. Walks the concerns
            # collection and hard-deletes every note whose `seed` flag is
            # False. Seeds stay (they're architectural baseline).
            if action == 'wipe_non_seed':
                deleted: List[str] = []
                kept_seed: List[str] = []
                errors: List[str] = []
                # Walk both collections. Agent_concerns: keep seeds.
                # User_concerns: no seeds exist (none authored from YAML),
                # so all get wiped.
                for coll_id, keep_seeds in (
                        (self._agent_concerns_collection_id, True),
                        (self._user_concerns_collection_id, False)):
                    if not coll_id:
                        continue
                    coll = self.resource_manager.get_resource(coll_id)
                    note_ids = list(((coll or {}).get('properties') or {}).get('content', []) or [])
                    for nid in note_ids:
                        note = self.resource_manager.get_resource(nid)
                        if not note:
                            continue
                        props = note.get('properties') or {}
                        if keep_seeds and bool(props.get('seed')):
                            kept_seed.append(nid)
                            continue
                        ok, err = self.resource_manager.delete_resource(nid)
                        if ok:
                            deleted.append(nid)
                        else:
                            errors.append(f"{nid}: {err}")
                if deleted:
                    self._persist_to_disk()
                logger.info(
                    f"[{self.character_name}] wipe_non_seed: deleted "
                    f"{len(deleted)}, kept {len(kept_seed)} seed, "
                    f"{len(errors)} errors")
                self._reply(query, {
                    'success': True,
                    'deleted_ids': deleted,
                    'deleted_count': len(deleted),
                    'kept_seed_ids': kept_seed,
                    'kept_seed_count': len(kept_seed),
                    'errors': errors,
                })
                return

            concern_id = str(params.get('concern_id', '') or '').strip()
            if not concern_id:
                self._reply(query, {'success': False,
                                    'error': "missing concern_id"})
                return

            # Hard delete — the browser confirms first.
            if action == 'delete':
                ok, err = self.resource_manager.delete_resource(concern_id)
                if ok:
                    self._persist_to_disk()
                    self._reply(query, {'success': True,
                                        'message': f'{concern_id} deleted'})
                else:
                    self._reply(query, {'success': False,
                                        'error': err or f'delete failed for {concern_id}'})
                return

            # Instruction edit from the browser textarea. Empty / whitespace-
            # only input → null so the concern stops firing, matching the
            # reflection-side null convention (a concern with no instruction
            # is logged for context but never fires). Whitespace-trimming is
            # surface-only; intentional internal blank lines are preserved.
            if action == 'set_instruction':
                note = self.resource_manager.get_resource(concern_id)
                kind = (note.get('properties') or {}).get('kind') if note else None
                if not note or kind not in ('concern', 'agent_concern'):
                    self._reply(query, {'success': False,
                                        'error': f"{concern_id} is not an agent_concern"})
                    return
                raw = params.get('value')
                if raw is None:
                    new_instr: Optional[str] = None
                else:
                    s = str(raw)
                    new_instr = s if s.strip() else None
                note['properties']['instruction'] = new_instr
                self._persist_to_disk()
                preview = (new_instr or '')[:80].replace('\n', ' ')
                logger.info(
                    f"[{self.character_name}] {concern_id} instruction set "
                    f"({len(new_instr or '')} chars): {preview!r}")
                self._reply(query, {
                    'success': True,
                    'message': f'{concern_id} instruction updated',
                    'instruction_chars': len(new_instr or ''),
                })
                return

            # Rhythm edit from the browser combo-box. Accepts both the
            # new action name and the legacy 'set_cadence_hours' so the
            # browser UI keeps working through item-7 refactor.
            if action in ('set_rhythm_hours', 'set_cadence_hours'):
                note = self.resource_manager.get_resource(concern_id)
                kind = (note.get('properties') or {}).get('kind') if note else None
                if not note or kind not in ('concern', 'agent_concern'):
                    self._reply(query, {'success': False,
                                        'error': f"{concern_id} is not an agent_concern"})
                    return
                snapped = _snap_rhythm_hours(params.get('value'))
                note['properties']['rhythm_hours'] = snapped
                # Reset the activation update anchor so the new rhythm
                # takes effect from now (otherwise a long-stalled note
                # would jump immediately).
                note['properties']['last_activation_update_at'] = (
                    datetime.now(timezone.utc).isoformat())
                self._persist_to_disk()
                self._reply(query, {'success': True,
                                    'message': f'{concern_id} rhythm_hours → {snapped}',
                                    'rhythm_hours': snapped,
                                    'cadence_hours': snapped})  # legacy alias
                return

            # Status transitions — map browser vocabulary to chat statuses.
            action_to_status = {
                'close': 'satisfied', 'closed': 'satisfied',
                'satisfy': 'satisfied', 'satisfied': 'satisfied',
                'abandon': 'abandoned', 'abandoned': 'abandoned',
                'reopen': 'active', 'reactivate': 'active',
                'activate': 'active', 'active': 'active',
            }
            new_status = action_to_status.get(action)
            if not new_status:
                self._reply(query, {'success': False,
                                    'error': f"unknown action {action!r}"})
                return
            ok, err = self._set_concern_status(concern_id, new_status)
            if ok:
                self._persist_to_disk()
                self._reply(query, {'success': True,
                                    'message': f'{concern_id} → {new_status}'})
            else:
                self._reply(query, {'success': False, 'error': err})
        except Exception as e:
            logger.error(f"[{self.character_name}] concern_manage query failed: {e}")
            self._reply(query, {'success': False, 'error': str(e)})

    # ------------------------------------------------------------------
    # Discourse / ToM
    # ------------------------------------------------------------------

    def _get_tracker(self, other_name: str):
        from discourse import DiscourseTracker
        tracker = self._discourse_trackers.get(other_name)
        if tracker is None:
            tracker = DiscourseTracker(self._llm_generate, self.character_name, other_name)
            self._discourse_trackers[other_name] = tracker
        return tracker

    def _build_dialog(self, entity: str, limit: int) -> List[Dict[str, str]]:
        """Build a list of {source, text} from the store, oldest-first."""
        turns = self.store.get_recent_turns(entity, limit=limit, scope='all')
        return [{'source': t.get('source') or t.get('entity') or '?',
                 'text': str(t.get('text', ''))} for t in turns]

    def _update_discourse_async(self, entity: str) -> None:
        """Run discourse + companion updates after a turn. Synchronous for now."""
        if not self.discourse_enabled:
            return
        self._affect.set_memory_op('companion_update')
        try:
            dialog = self._build_dialog(entity, limit=self.history_limit)
            if not dialog:
                return
            tracker = self._get_tracker(entity)
            prev_disc = self._discourse_state.get(entity, '')
            # Discourse and companion calls run with generous token budgets and
            # no grammar gate. Grammar adds no value here, and the byte-cap +
            # permissive-answer pattern leaks thinking into the output, which
            # then contaminates state notes and re-injects into chat_reply's
            # system prompt on subsequent turns.
            #
            # reasoning_effort=low: these reflection passes produce
            # structured-text output (commitments / agreements / decisions
            # / companion sections) from a long prompt — synthesis, not
            # novel reasoning. Medium effort burns analysis tokens that
            # don't improve output but DO push the final-channel output
            # past max_tokens, leaving content empty and the update
            # marked as failed. Override to low; ReAct main loop keeps
            # the scenario's medium baseline.
            tracker.llm_generate = self._make_llm_callable(
                'none', reasoning_effort='low')
            new_disc = tracker.analyze_segment(
                dialog, start=0, end=len(dialog) - 1,
                previous_discourse_state=prev_disc, tom='',
                narrator_persona=self.persona,
                narrator_self_model=self.self_model,
            )
            if new_disc:
                self._discourse_state[entity] = str(new_disc)
                self._save_state_note('discourse_state', entity, str(new_disc))
                self._write_state_snapshot('discourse_state', entity, str(new_disc))

            # Companion model — fair-witness texture for the chat reply.
            # Runs per-turn here (rather than only at dialog close as in
            # executive_node) because chat-mode sessions rarely emit `close`,
            # and the template is built to self-prune slow-moving fields.
            prev_comp = self._companion_state.get(entity, '')
            tracker.llm_generate = self._make_llm_callable(
                'none', reasoning_effort='low')
            new_comp = tracker.update_companion_from_discourse_segment(
                dialog, character_name=entity, start=0, end=len(dialog) - 1,
                discourse_state=self._discourse_state.get(entity, ''),
                previous_companion_state=prev_comp,
                narrator_persona=self.persona,
                narrator_self_model=self.self_model,
            )
            if new_comp and len(str(new_comp).strip()) > 20:
                self._companion_state[entity] = str(new_comp)
                self._save_state_note('companion_state', entity, str(new_comp))
                self._write_state_snapshot('companion_state', entity, str(new_comp))
        except Exception as e:
            logger.warning(f'[{self.character_name}] discourse update failed: {e}')
        finally:
            self._affect.set_memory_op('none')

    # ------------------------------------------------------------------
    # Orientation pass (character_evaluator)
    # ------------------------------------------------------------------

    def _orientation_summary(self, source: str, text: str) -> str:
        if not self.orientation_enabled:
            return ''
        try:
            from character_evaluator import evaluate, build_orientation_summary
            recent = self._build_dialog(source, limit=6)
            recent_str = '\n'.join(f"{t['source']}: {t['text']}" for t in recent[:-1])
            event = {
                'event_type': 'user_text',
                'source': source,
                'content': text,
                'disposition': 'inform',
                'event_id': uuid.uuid4().hex[:12],
                'timestamp': datetime.now(timezone.utc).isoformat(),
            }
            # Pass current active concerns so the evaluator can do real
            # relevance assessment. Concern text serves double duty as id
            # (truncated) and description — the evaluator emits matches as
            # `<id>:<level>` and the rendered orientation surfaces those
            # ids verbatim, so a meaningful slug lets Jill see *which*
            # concern the input activates.
            char_concerns: List[Dict[str, str]] = []
            for _nid, note, _a in self._iter_active_agent_concerns():
                ctext = str((note.get('properties') or {}).get('content', '') or '').strip()
                if not ctext:
                    continue
                slug = ctext[:60].replace('\n', ' ').strip()
                char_concerns.append({'id': slug, 'description': ctext})
            companion = self._companion_state.get(source, '').strip()
            assessment = evaluate(
                event=event,
                character_concerns=char_concerns,
                user_concerns=[],
                goals_compact=[],
                recent_context=recent_str,
                activity_state='chat-only (no autonomous activity)',
                llm_generate=self._make_llm_callable('triage'),
                narrator_persona=self.persona,
                narrator_self_model=self.self_model,
                companion_state=companion,
            )
            return build_orientation_summary(assessment, event_content=text)
        except Exception as e:
            logger.warning(f'[{self.character_name}] orientation eval failed: {e}')
            return ''

    # ------------------------------------------------------------------
    # System prompt + turn assembly
    # ------------------------------------------------------------------

    # Order in which categories appear in the rendered memories block.
    # Operationally most-relevant first: preferences shape every reply,
    # commitments may need follow-up, facts are background context.
    _CATEGORY_RENDER_ORDER = ('preference', 'commitment', 'fact')
    _CATEGORY_HEADERS = {
        'preference': 'Preferences',
        'commitment': 'Commitments',
        'fact': 'Facts',
    }

    def _build_system_prompt(self, source: str, orientation: str,
                             recall: Optional[List[Tuple[str, str, str]]] = None,
                             agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                             user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None) -> str:
        """Build the persona/state portion of the system prompt — shared base
        for ReAct mode. The prose-only directive that used to live here was
        moved out: ReAct supplies its own JSON-emit directive in
        _build_react_system_prompt. There is no non-ReAct chat path now."""
        parts: List[str] = []
        parts.append(f"You are {self.character_name}, speaking in first person.")
        if self.persona:
            parts.append("## Persona (from character config)\n" + self.persona)
        if self.self_model:
            parts.append("## Self-model (from character config; what I am, not who)\n" + self.self_model)
        if self.capabilities:
            parts.append("## Capabilities (from character config; chat-only mode)\n" + self.capabilities)
        # When an external repo is bound for the session, append a single
        # capabilities line so the persona-level account reflects what the
        # ReAct surface actually exposes. Self-model is intentionally left
        # alone — external code is environment, not substrate.
        external_repo = self._get_external_repo()
        if external_repo is not None:
            parts.append(
                f"## External repo (bound for this session)\n"
                f"She can also navigate the project repo at `{external_repo}` "
                f"via the inspect_external tool — same list/read/grep "
                f"primitives as inspect, different geofence. This is reading "
                f"an external codebase as documentation, not introspection.")
        if self.setting:
            parts.append("## Setting (from character config)\n" + self.setting)
        companion = self._companion_state.get(source, '').strip()
        if companion:
            parts.append(
                f"## Companion model of {source} (rolling LLM reflection; fair-witness texture, not a brief to flatter)\n"
                f"{companion}"
            )
        # User concerns sit adjacent to companion — they're a structured
        # part of the user model. Decay each turn; bumped on similarity
        # match with input. Surface what user has been tracking, ranked
        # by current strength.
        if user_concerns:
            uc_lines: List[str] = []
            for _nid, text, strength, _props in user_concerns:
                uc_lines.append(f"- [{strength:.2f}] {text}")
            parts.append(
                f"## What {source} has been tracking (user_concerns, ranked by strength)\n"
                "Topics user has surfaced or returned to recently. Strength "
                "decays each turn unless user touches the topic again. Use "
                "to inform responses; do not act on these autonomously.\n\n"
                + "\n".join(uc_lines)
            )
        # Agent concerns sit before memories: directives to advance, not
        # background context. Each item renders with current activation
        # so Jill can see how close to firing each one is. Seeds are
        # constitutional (sources for derived concerns), distinguished
        # by tag. instruction-bearing concerns fire when activation
        # crosses threshold.
        if agent_concerns:
            ac_lines: List[str] = []
            for _nid, text, activation, props in agent_concerns:
                tags = []
                if props.get('seed'):
                    tags.append('seed')
                if props.get('successor_of'):
                    tags.append(f"successor d{props.get('successor_depth', 1)}")
                tag_str = f", {','.join(tags)}" if tags else ''
                ac_lines.append(f"- [{activation:.2f}{tag_str}] {text}")
                instr = (props.get('instruction') or '').strip()
                rhythm = props.get('rhythm_hours')
                if instr and rhythm:
                    ac_lines.append(
                        f"    fires every ~{rhythm}h when activation crosses "
                        f"{_AGENT_CONCERN_FIRE_THRESHOLD:.2f}: {instr[:120]}")
                elif not instr:
                    ac_lines.append(
                        "    standing concern, no instruction (won't fire)")
            parts.append(
                f"## My active concerns (agent_concerns, ranked by activation)\n"
                "Pressure-driven: activation grows over wall-clock time at "
                "each concern's rhythm; firing decrements it. Concerns "
                "without an instruction don't fire — they shape what I "
                "attend to without driving action.\n\n"
                + "\n".join(ac_lines)
            )
        # Threads: activity-level anchors. Computed once per turn in
        # _process_user_turn_inner via _compute_thread_activation; the
        # rendered block names the primary active thread plus any
        # secondary threads carrying meaningful weight. Activations
        # themselves are not surfaced as numbers — prominence in the
        # prompt encodes the distribution.
        threads_block = self._render_active_threads_block(
            self._current_thread_activation)
        if threads_block:
            parts.append(threads_block)
        if recall:
            # Episodic specifics retrieved from prior conversations. Distinct
            # from the rolling Companion summary: these are durable items
            # that should not decay with style. Rendered grouped by
            # category so the model can treat each group on its own terms.
            # Negative-polarity items (explicit rejections / dispreferences)
            # are prefixed `[avoid] ` so the model treats them as boundaries
            # rather than positive facts.
            grouped: Dict[str, List[str]] = {c: [] for c in _MEMORY_CATEGORIES}
            for text, cat, pol in recall:
                if cat not in grouped:
                    cat = 'fact'
                marker = '[avoid] ' if pol == 'negative' else ''
                grouped[cat].append(f"{marker}{text}")

            body_lines: List[str] = []
            for cat in self._CATEGORY_RENDER_ORDER:
                items = grouped.get(cat) or []
                if not items:
                    continue
                if body_lines:
                    body_lines.append('')  # blank line between groups
                body_lines.append(f"{self._CATEGORY_HEADERS[cat]}:")
                for t in items:
                    body_lines.append(f"- {t}")

            if body_lines:
                parts.append(
                    f"## Recalled memories (from prior conversations with {source})\n"
                    "Preferences shape how to respond; commitments are open "
                    "agreements that may need follow-up; facts are background "
                    "specifics about " + source + ". Items marked `[avoid]` "
                    "are explicit rejections — do not act on them as if "
                    "they were positive preferences.\n\n"
                    + "\n".join(body_lines)
                )
        disc = self._discourse_state.get(source, '').strip()
        if disc:
            parts.append(
                "## Shared premises and standing decisions (from periodic reflection)\n"
                "These are the operating premises and decisions both parties "
                "have accepted across this conversation. Treat as load-bearing — "
                "disagreeing with one reopens prior discussion rather than "
                "introducing a new topic.\n\n"
                + disc)
        if orientation:
            parts.append(orientation)
        return "\n\n".join(parts)

    # ------------------------------------------------------------------
    # ReAct loop — continuing-computation style.
    # System prompt: persona + companion + tool catalog (stable).
    # User message: conversation history + current input + working log +
    # "Emit next action:" trailer (rebuilt each iteration).
    # The user input is part of the log text, NOT a separate user-role
    # message — that's what stops chat-trained models from defaulting to
    # direct prose reply.
    # ------------------------------------------------------------------

    def _build_react_tool_catalog(self) -> str:
        """Build the numbered tool list shown in the ReAct system prompt.
        Conditionally includes inspect_external — and the bound external
        repo path — only when an external_repo is currently bound. Tools
        whose name appears in self._omitted_tools (set per scenario via
        chat.omitted_tools, used by the cspred bench cf-cells) are
        filtered out so the affordance representation matches what is
        actually invokable. Numbers shift accordingly; the model handles
        all layouts fine."""
        tools: List[Tuple[str, str]] = [
            ("process_text",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"process_text\", \"source\": <string|$stepN>, \"instruction\": <string>}` — "
             "LLM pass over text in context. Use to formulate queries, render results in your voice, extract info."),
            ("search",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"search\", \"query\": <string|$stepN>}` — "
             "web search (digested synthesis + sources)."),
            ("fetch_text",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"fetch_text\", \"url\": <string|$stepN>}` — "
             "full text from a single URL (or local file path). Use when a search hit looks promising and the "
             "snippet isn't enough; always pass the result through process_text before responding."),
            ("recall",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"recall\", \"query\": <string>}` — "
             "READ-ONLY active recall over your own memory: full reasoning trace, conversation history, "
             "current companion model, current discourse state, log of persisted memories. Use when the "
             "user asks about prior turns ('what did we discuss?', 'you mentioned X earlier'), your "
             "earlier reasoning, or persistent state that may not be in your current prompt. The query "
             "is opaque to you — a subagent reads the files and returns a synthesized answer in `$stepN`. "
             "Cannot write: if the user says 'remember X' / 'note Y' / 'save Z', acknowledge in `respond` "
             "without claiming persistence — memory writes happen via background reflection, not from "
             "inside this loop."),
            ("inspect",
             "`{\"thought\": \"<one terse sentence>\", \"tool\": \"inspect\", \"query\": <string>}` — "
             "query your own codebase. A separate subagent (geofenced read-only to `src/`, list/read/grep "
             "primitives) navigates the source and returns a synthesized answer with file:line citations. Use "
             "when the user asks how you work, where something is implemented, what a module does, or to verify "
             "a claim about your own code. The query is opaque to you — phrase it as a natural-language question "
             "(e.g. \"where is the ReAct dispatch defined?\", \"what tools does the chat loop wire up?\")."),
        ]
        external_repo = self._get_external_repo()
        if external_repo is not None:
            tools.append(("inspect_external",
                "`{\"thought\": \"<one terse sentence>\", \"tool\": \"inspect_external\", \"query\": <string>}` — "
                f"query the external project repo currently bound to this session: `{external_repo}`. "
                "Same primitives as `inspect` (list/read/grep), different geofence — this is reading an external "
                "codebase as documentation, not introspection of your own substrate. Use when the user asks "
                "questions about THAT project (its code, README, structure, behavior). Phrase as a "
                "natural-language question (e.g. \"how does this project structure its modules?\", "
                "\"what does the README say about installation?\", \"where is the main entry point?\")."))
        tools.append(("security",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"security\", \"query\": <string>}` — "
            "investigate LAN / local-system network state. A separate subagent (read-only typed primitives: "
            "nmap host discovery + service scan, ss/ip for sockets/routes/arp/interfaces; targets restricted to "
            "RFC1918 ranges) runs the probes and returns a synthesized answer. Use when the user asks about LAN "
            "hosts, what's listening locally, what services a host exposes, suspicious activity on the network, "
            "or routing/interface state. v0.1 does NOT do traffic capture or IDS log inspection. Phrase as a "
            "natural-language question (e.g. \"what hosts are on my LAN at 192.168.1.0/24?\", \"what TCP ports "
            "am I listening on?\", \"what's my default gateway?\")."))
        tools.append(("display",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"display\", "
            "\"content\": <string|$stepN>, \"format\": \"markdown\"|\"html\"}` — "
            "**USE THIS** whenever the user asks to *show / display / draw / render / visualize / "
            "put on screen*, OR whenever the answer is fundamentally structured (multi-row tables, "
            "diagrams via inline SVG, side-by-side comparisons, multi-section walk-throughs). "
            "Pushes rich content to the canvas window so the user can SEE it, not just read it. "
            "Non-terminal — the loop continues; immediately follow with a brief `respond` so the "
            "chat conversation gets a textual reply too (e.g. \"posted it on screen — let me know if "
            "you want X\"). "
            "**Even if your data is incomplete or imperfect, render what you DO have** — the user "
            "asked to see it; don't decline because the picture isn't full. Acknowledge the gap "
            "verbally in `respond`. "
            "Default `format=markdown` (safer, easier to author). Use `format=html` for inline "
            "SVG (diagrams, drawings, illustrations), multi-column layout, or custom styling. "
            "**Line breaks matter**: markdown needs real `\\n` newlines in the content string "
            "for headings, lists, and paragraphs to render — don't collapse your source to a "
            "single line, or `### Title` and `- bullet` show up as literal text. "
            "**Drawings / illustrations**: when the user says *draw / sketch / illustrate / "
            "diagram*, generate inline `<svg>...</svg>` yourself — cheap, immediate, no "
            "external dependency. Image search is for *photos* or specific existing images "
            "the user named — not for whimsical drawings you can produce yourself. "
            "**Photos from the web**: `format=html` with "
            "`<img src=\"http://127.0.0.1:8789/proxy?url=<url>\">`. The URL must be a "
            "**direct image-file URL** (e.g. `cdn.example.com/photo.jpg`), NOT a webpage that "
            "contains the image. Page URLs through the proxy produce broken-image icons — "
            "the proxy faithfully returns whatever bytes the URL yields, and `text/html` "
            "cannot render inside `<img>`. If you only have a page URL (a search hit, a "
            "stock-photo landing page, an article), `fetch_text` it first and use "
            "`metadata.html_metadata.images.og_image` — that's what that field is for."))
        tools.append(("respond",
            "`{\"thought\": \"<one terse sentence>\", \"tool\": \"respond\", \"text\": <string|$stepN>}` — "
            "final reply, exits loop. Must be in your voice; pass search/fetch results through process_text "
            "first or write the reply yourself."))

        omitted = set(self._omitted_tools or [])
        if omitted:
            kept = [(name, descr) for name, descr in tools if name not in omitted]
            dropped = [name for name, _ in tools if name in omitted]
            logger.info(f"[{self.character_name}] _build_react_tool_catalog: omitted_tools active, dropped={dropped}")
            tools = kept

        numbered = "\n".join(f"{i}. {descr}" for i, (_, descr) in enumerate(tools, start=1))
        return "Tools (each emission picks ONE):\n" + numbered + "\n"

    def _build_react_system_prompt(self, source: str, orientation: str,
                                   now_str: str,
                                   recall: Optional[List[Tuple[str, str, str]]] = None,
                                   agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                                   user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None) -> str:
        base = self._build_system_prompt(
            source, orientation, recall=recall,
            agent_concerns=agent_concerns, user_concerns=user_concerns)
        search_omitted = 'search' in (self._omitted_tools or [])
        if search_omitted:
            fresh_info_guidance = (
                "You do NOT know: current weather, recent news, current "
                "prices, or anything requiring fresh information. You have "
                "no live-data tool available this session; if asked for "
                "fresh information, say so plainly rather than guessing.\n"
            )
        else:
            fresh_info_guidance = (
                "You do NOT know: current weather, recent news, current prices, "
                "or anything requiring fresh information. For time-sensitive or "
                "fact-specific questions, your first action is `search`.\n"
            )
        react = (
            f"\n\n## Now (system clock)\n{now_str}\n"
            "\n"
            "## ReAct Tool Loop — READ FIRST\n"
            "Each emission is ONE JSON action object — nothing else. Prose "
            "around the JSON is discarded and the loop will retry. Output "
            "begins with `{` and ends with `}`.\n"
            "\n"
            "Every emission MUST include a `thought` field: ONE TERSE "
            "SENTENCE supporting your action choice. Not narration of the "
            "user's intent, not a summary of the conversation, not "
            "throwaway filler — the actual one-line reason for picking "
            "THIS action over the alternatives. The thought is preserved "
            "verbatim into your future-turn awareness feed.\n"
            "\n"
            + fresh_info_guidance
            + "\n"
            + self._build_react_tool_catalog()
            + "\n"
            "## Observation format\n"
            "Each tool observation in the working log starts with one of three tags:\n"
            "  `OK: <content>`     — tool succeeded; content follows\n"
            "  `EMPTY: <reason>`   — tool ran cleanly but produced no usable result\n"
            "  `ERROR: <reason>`   — tool failed (unavailable / raised / malformed args)\n"
            "Treat ERROR as a hard signal: the tool is currently broken — do NOT retry the "
            "same call. Treat EMPTY as a soft signal: reformulating the call (different query, "
            "different URL) may help, but do not loop blindly. The tag is informational for your "
            "reading of the log; it is automatically stripped when `$stepN` is substituted into "
            "downstream actions or into `respond.text`, so you do not need to strip it yourself.\n"
            "\n"
            "Three sources of content in this loop, each attributable when asked about provenance: "
            "(a) Substrate output (your own): text you write inline within an action — `thought`, "
            "`process_text` `instruction`, or inline `respond.text` — is produced by you in this "
            "iteration. You can report THAT you produced it, not WHY a particular phrasing or "
            "formulation arose. (b) Bound inputs: each action's result auto-binds to `$step1, "
            "$step2, …` (per-turn scope) — observable outputs of prior tool calls. (c) Prompt-given "
            "inputs: `## Conversation history`, `## Active concerns`, `## Recalled memories`, "
            "`## Recent reasoning` are observable input from external state. Reason over them "
            "directly in `respond` rather than echoing a section name into `process_text`'s "
            "`source`; `source` resolves only literal inline text or a `$stepN` binding, never a "
            "section header.\n"
            "\n"
            + ("" if search_omitted else
               "Worked example. User: 'what's the weather in Berkeley tomorrow?'\n"
               "  Iter 1: `{\"thought\": \"Need fresh weather data — search first.\", \"tool\": \"search\", \"query\": \"Berkeley CA weather forecast tomorrow\"}` → $step1\n"
               "  Iter 2: `{\"thought\": \"Search synthesis is decent; render in my voice with source.\", \"tool\": \"process_text\", \"source\": \"$step1\", \"instruction\": \"answer the user in your voice in 1-2 sentences, citing the source domain\"}` → $step2\n"
               "  Iter 3: `{\"thought\": \"Processed answer is ready — send it.\", \"tool\": \"respond\", \"text\": \"$step2\"}` → loop exits.\n"
               "\n"
               "Worked example with display. User: 'show me the S&P 500 daily closes for the last week.'\n"
               "  Iter 1: `{\"thought\": \"Need fresh price data.\", \"tool\": \"search\", \"query\": \"S&P 500 daily close last 7 trading days\"}` → $step1\n"
               "  Iter 2: `{\"thought\": \"Format extracted prices as a markdown table for the canvas.\", \"tool\": \"process_text\", \"source\": \"$step1\", \"instruction\": \"extract the daily closes as a markdown table with Date and Close columns; nothing else\"}` → $step2\n"
               "  Iter 3: `{\"thought\": \"Push table to canvas — it's tabular, not prose.\", \"tool\": \"display\", \"content\": \"$step2\", \"format\": \"markdown\"}` → $step3 (non-terminal)\n"
               "  Iter 4: `{\"thought\": \"Acknowledge in the chat with a brief note pointing at the canvas.\", \"tool\": \"respond\", \"text\": \"Posted the last week's closes on screen — let me know if you want a longer window or a chart.\"}` → loop exits.\n"
               "\n")
            + "Output ONLY one JSON object. No prose, no apology, no explanation."
        )
        return base + react

    def _build_react_user_prefix(self, source: str, user_text: str) -> str:
        """Build the user-message PREFIX for the ReAct loop. Constructed once
        per loop; the working-log entries and the "Emit next action:" trailer
        are appended by the caller (store-and-append: the prefix is byte-stable
        across iterations so KV cache hits, only the log grows). Ends with
        '## Working log\\n' so the first appended entry sits below the header."""
        parts: List[str] = []
        # On autonomous turns, source is the character itself (the
        # originator), but the conversation we want surfaced is still the
        # User dialogue — fall back to 'User' when source is self so the
        # ReAct loop sees the same context the user-driven path would.
        history_entity = 'User' if source == self.character_name else source
        history = self.store.get_recent_turns(history_entity, limit=self.history_limit, scope='all')
        if history and history[-1].get('direction') == 'in' and str(history[-1].get('text', '')) == user_text:
            history = history[:-1]
        if history:
            parts.append("## Conversation history (verbatim session turns)")
            for t in history:
                who = history_entity if t.get('direction') == 'in' else self.character_name
                parts.append(f"{who}: {t.get('text', '')}")
            parts.append("")
        # Awareness feed: prior ReAct traces (Jill's own thinking from
        # recent turns) sit between conversation history and current
        # input. Read once per loop entry — same store-and-append
        # discipline as conversation history: byte-stable across
        # iterations within the loop, fresh on the next loop.
        reasoning_block = self._get_reasoning_history_block()
        if reasoning_block:
            parts.append(reasoning_block)
            parts.append("")
        parts.append("## Current user input")
        parts.append(user_text)
        parts.append("")
        parts.append("## Working log (this loop's actions and observations)")
        return "\n".join(parts) + "\n"

    # --- ReAct status line (CLI feedback during the loop) -----------------
    # Subsequent _emit_status calls overwrite the previous one via \r, so the
    # user sees a single line that mutates as the loop progresses
    # ("thinking…" → "using search…" → "thinking…" → "using process_text…").
    # _clear_status wipes the line before the response is published, so the
    # CLI's response print starts on a clean line.

    _STATUS_DIM = '\033[2m'
    _STATUS_RESET = '\033[0m'
    _STATUS_PAD = 80  # enough to overwrite any reasonable previous status

    def _emit_status(self, msg: str) -> None:
        """Overwrite the current terminal line with a transient status.
        No-op when stdout isn't a TTY (logs / piped output) — printed
        per-iteration log lines via logger.info already cover that case."""
        if not sys.stdout.isatty():
            return
        try:
            line = f"\r{self._STATUS_DIM}[{self.character_name}] {msg}{self._STATUS_RESET}"
            # Pad with spaces so a shorter status overwrites leftover chars
            # from a longer previous one. The \r places the cursor back at
            # column 0; the next write (or _clear_status) replaces this.
            sys.stdout.write(line.ljust(self._STATUS_PAD))
            sys.stdout.flush()
        except Exception:
            pass  # status is non-essential; never raise

    def _clear_status(self) -> None:
        """Clear the status line and return cursor to column 0."""
        if not sys.stdout.isatty():
            return
        try:
            sys.stdout.write('\r' + ' ' * self._STATUS_PAD + '\r')
            sys.stdout.flush()
        except Exception:
            pass

    def _run_react_loop(self, source: str, user_text: str, orientation: str,
                        recall: Optional[List[Tuple[str, str, str]]] = None,
                        agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                        user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                        image_url: Optional[str] = None
                        ) -> Tuple[str, List[Tuple[str, str]], List[Dict[str, Any]], str]:
        """Run the ReAct loop. Returns (reply, log, iters, exit_reason).
        The trace is NOT written here — caller writes it after post-turn
        reflection so the trace can include memories written."""
        log: List[Tuple[str, str]] = []
        # Per-iteration record:
        #   system   — full system prompt sent (constant across iters,
        #              captured here for trace fidelity)
        #   user     — full user message sent (grows with the working log)
        #   raw      — model's full raw emission (thought + tool call)
        #   appended — log entries this iter's parsed action contributed to
        #              the working log for the NEXT iter to see. May be 0
        #              entries (respond / loop-exit) or 2 entries (action
        #              + observation) or 1 entry (NOTE on parse error).
        # The model's raw emission and `appended` differ — the difference
        # is exactly the reasoning that got "compressed away" before the
        # next iter saw the context.
        iters: List[Dict[str, Any]] = []
        # Store-and-append architecture for KV-cache stability:
        #   • system_prompt_str and user_prefix_str are built ONCE at loop
        #     start and reused literally on every iteration — never rebuilt.
        #   • log_appendage_str grows by literal string append in lockstep
        #     with the log list. We never reformat past entries.
        #   • Each iter's user message = user_prefix_str + log_appendage_str
        #     + trailer. All three are stored strings; the only operation
        #     between iterations is concatenation of stored pieces.
        # This guarantees a byte-stable prefix across iterations so the
        # backend's KV cache hits on every iter past the first.
        now_str = datetime.now().astimezone().strftime("%A, %B %d %Y, %I:%M %p %Z")
        system_prompt_str = self._build_react_system_prompt(
            source, orientation, now_str, recall=recall,
            agent_concerns=agent_concerns, user_concerns=user_concerns)
        user_prefix_str = self._build_react_user_prefix(source, user_text)
        log_appendage_str = ""
        trailer = "\nEmit next action:"

        def _append_log(label: str, content: str) -> None:
            """Lockstep append: list and string grow together. The string
            format must match what _build_react_user_prefix would have
            rendered for these entries (LABEL:\\nCONTENT\\n)."""
            nonlocal log_appendage_str
            log.append((label, content))
            log_appendage_str += f"{label}:\n{content}\n"

        # Track whether the agent explicitly pushed something to the canvas
        # this turn. If she did, leave it alone — `display` content is richer
        # than the prose reply. If she didn't, auto-mirror the respond text
        # to canvas as markdown so the surface always reflects the latest
        # reply rather than going stale.
        did_display = False

        self._affect.enter_loop()
        for i in range(REACT_MAX_ITERS):
            self._affect.set_react_iter(i + 1)
            pre_log_len = len(log)
            usr_msg = user_prefix_str + log_appendage_str + trailer
            # When the turn carries an image, every iter sends a
            # multimodal content array (text first per vLLM's expected
            # ordering, then image_url). The image must persist across
            # iters: ReAct's parse-failure retry path drops back into
            # iter 2+, and if the image were missing there, the model
            # confabulates against its own prior text. Byte-stable image
            # tokens across iters still let vLLM prefix-cache the text.
            # No-image turns keep the plain-string content shape exactly
            # as before — wire format byte-identical to pre-multimodal.
            if image_url:
                user_content: Any = [
                    {'type': 'text', 'text': usr_msg},
                    {'type': 'image_url', 'image_url': {'url': image_url}},
                ]
            else:
                user_content = usr_msg
            prompt = [
                {'role': 'system', 'content': system_prompt_str},
                {'role': 'user', 'content': user_content},
            ]
            self._emit_status('thinking…')
            self._affect.set_waiting_for_llm(True)
            try:
                raw = self.backend.chat(prompt, max_tokens=8192, temperature=0.7,
                                        cot_profile='none')
            except Exception as e:
                logger.error(f"[{self.character_name}] ReAct iter {i+1} LLM failed: {e}")
                iters.append({'system': system_prompt_str, 'user': usr_msg,
                              'raw': f'(LLM call failed: {e})', 'appended': []})
                reply = self._react_fallback_synthesis(log, str(e))
                self._clear_status()
                self._affect.exit_loop()
                return reply, log, iters, 'llm_error'
            finally:
                self._affect.set_waiting_for_llm(False)

            iters.append({'system': system_prompt_str, 'user': usr_msg, 'raw': raw or '',
                          'appended': []})

            action = self._parse_react_action(raw)
            if action is None:
                logger.warning(f"[{self.character_name}] ReAct iter {i+1}: unparseable: {raw[:160]!r}")
                self._emit_status('parse failed, retrying…')
                self._affect.incr_llm_retry()
                _append_log('NOTE', "Previous output was prose, not JSON. The user's task above is "
                            "unanswered. Do NOT apologize — emit ONE JSON action now to address it.")
                iters[-1]['appended'] = log[pre_log_len:]
                continue

            tool = action.get('tool')
            if tool == 'respond':
                text = self._resolve_react_value(action.get('text', ''), log)
                logger.info(f"[{self.character_name}] ReAct iter {i+1}: respond ({len(text)} chars)")
                # respond appends nothing to the log (loop exits); appended stays []
                self._clear_status()
                # Auto-mirror: when the agent didn't push richer content
                # via `display` this turn, render the respond text on the
                # canvas as markdown. Keeps the surface in sync with the
                # latest reply for free.
                if not did_display and text and text.strip():
                    try:
                        self._canvas.set_content(text, fmt='markdown')
                    except Exception as e:
                        logger.warning(
                            f"[{self.character_name}] canvas auto-mirror failed: {e}")
                self._affect.mark_completion()
                self._affect.exit_loop()
                return text, log, iters, 'respond'

            self._emit_status(f'using {tool}…')
            # Tool-call signature for repeat detection. Truncated to avoid
            # giant payloads in the publisher state. Sort keys so equivalent
            # actions hash the same regardless of dict ordering.
            _tool_args_sig = json.dumps(
                {k: v for k, v in action.items() if k != 'tool'},
                sort_keys=True, default=str)[:200]
            self._affect.set_tool(tool, args_sig=_tool_args_sig)
            if tool == 'recall':
                self._affect.set_memory_op('recall')
            binding = f'$step{i+1}'
            _append_log(f'ACTION {i+1}', json.dumps(action))

            # ----------------------------------------------------------
            # Tool-observation convention (KISS, prose-only, no runtime
            # layering). Every observation string emitted by a tool starts
            # with one of three prefixes so the ReAct LLM can discriminate
            # success from failure without keyword-spotting:
            #   OK: <content>     — tool succeeded, content follows
            #   EMPTY: <reason>   — tool ran without error but produced no
            #                       usable result (no search hits; remember
            #                       found nothing; empty argument)
            #   ERROR: <reason>   — tool failed (unavailable, raised,
            #                       returned malformed). Treat as "this
            #                       tool is currently broken" — do not
            #                       retry the same call.
            # New tools must follow the same convention. See src/chat/
            # AGENTS.md for the author-facing rule.
            # ----------------------------------------------------------
            if tool == 'process_text':
                raw_src = action.get('source', '')
                ins = action.get('instruction', '')
                src = self._resolve_react_value(raw_src, log)
                diag = self._diagnose_process_text_args(raw_src, src, ins, log)
                if diag is not None:
                    logger.warning(f"[{self.character_name}] process_text "
                                   f"dispatch rejected (iter {i+1}): {diag}")
                    # Diagnose returns a free-form reason — wrap as ERROR
                    # so the agent treats it the same as any other tool
                    # failure rather than mistaking it for content.
                    obs = f'ERROR: process_text rejected: {diag}'
                else:
                    obs = self._run_process_text(src, ins)
            elif tool == 'search':
                q = self._resolve_react_value(action.get('query', ''), log)
                if not q:
                    obs = 'EMPTY: search query was empty'
                elif self._get_llm_search() is None:
                    obs = 'ERROR: search-web tool unavailable (failed to load — see warning log)'
                else:
                    result = self._run_web_search(q)
                    if result:
                        obs = 'OK: ' + self._format_react_search_observation(result)
                    else:
                        obs = (f'EMPTY: search returned no results for "{q}" '
                               '(or transient API failure — see warning log)')
            elif tool == 'fetch_text':
                u = self._resolve_react_value(action.get('url', ''), log)
                obs = self._run_fetch_text(u)
            elif tool == 'recall':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_remember(q)
            elif tool == 'inspect':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_inspect(q)
            elif tool == 'inspect_external':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_inspect_external(q)
            elif tool == 'security':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_security(q)
            elif tool == 'display':
                content = self._resolve_react_value(action.get('content', ''), log)
                fmt = (action.get('format') or 'markdown').strip().lower()
                obs = self._run_display(content, fmt)
                # Suppress auto-mirror on the final respond — the agent
                # has already chosen what belongs on the canvas this turn.
                if isinstance(obs, str) and obs.startswith('OK'):
                    did_display = True
            else:
                # Tool list shown to the model on bad emission. Don't
                # advertise inspect_external when nothing is bound — keeps
                # the surface honest about what's actually available.
                avail = ("process_text, search, fetch_text, recall, "
                         "inspect, security, display, respond")
                if self._get_external_repo() is not None:
                    avail = avail.replace("inspect, ", "inspect, inspect_external, ")
                obs = f"ERROR: unknown tool {tool!r}; available: {avail}"

            _append_log(binding, obs)
            iters[-1]['appended'] = log[pre_log_len:]
            # Tool observations follow the documented OK:/EMPTY:/ERROR: prefix
            # protocol (see 4170–4184). ERROR: is the only failure signal and
            # is a literal prefix, not a keyword classification.
            if isinstance(obs, str) and obs.startswith('ERROR'):
                self._affect.incr_tool_error()
            self._affect.set_memory_op('none')
            self._affect.set_tool(None)
            logger.info(f"[{self.character_name}] ReAct iter {i+1}: {tool} → {binding} ({len(obs)} chars)")

        logger.warning(f"[{self.character_name}] ReAct hit max iters ({REACT_MAX_ITERS})")
        reply = self._react_fallback_synthesis(log)
        self._clear_status()
        self._affect.exit_loop()
        return reply, log, iters, 'max_iters'

    def _memory_dir(self) -> 'Path':
        """Per-world per-agent memory directory — substrate the active-recall
        subagent will read. Layout: scenarios/<world>/<agent>/memory/.
        Derived from resource_manager.base_dir (which is
        scenarios/<world>/resources/<agent>/ once the resource manager
        applies its per-agent scoping) by going up two levels and over.
        Created on first write by callers."""
        from pathlib import Path
        return (
            self.resource_manager.base_dir.parent.parent
            / self.character_name / 'memory'
        )

    def _subagent_traces_dir(self) -> 'Path':
        """Where the active-recall subagent writes its per-call traces.
        Peer of memory/, NOT under it — keeps subsequent recall calls from
        reading their own prior traces as substrate. Layout:
        scenarios/<world>/<agent>/subagent_traces/."""
        return self._memory_dir().parent / 'subagent_traces'

    _DATA_URI_RE = re.compile(r'^data:(image/[a-zA-Z0-9.+-]+);base64,(.+)$', re.DOTALL)

    def _persist_image_url(self, image_url: str) -> Optional[str]:
        """Write a data-URI-encoded image to <memory>/images/<sha>.<ext>
        once (deduped by sha256) and return the relative path string.
        Returns None for http(s) URLs (the URL itself is the trace
        identifier; we don't fetch). Raises on a malformed data URI."""
        if image_url.startswith(('http://', 'https://')):
            return None
        m = self._DATA_URI_RE.match(image_url)
        if not m:
            raise ValueError("image_url is neither http(s) URL nor a base64 data URI")
        mime, b64 = m.group(1), m.group(2)
        import base64, hashlib
        try:
            raw = base64.b64decode(b64, validate=False)
        except Exception as e:
            raise ValueError(f"data URI base64 decode failed: {e}")
        sha = hashlib.sha256(raw).hexdigest()
        ext = {
            'image/png': 'png', 'image/jpeg': 'jpg', 'image/jpg': 'jpg',
            'image/gif': 'gif', 'image/webp': 'webp',
        }.get(mime.lower(), 'bin')
        img_dir = self._memory_dir() / 'images'
        img_dir.mkdir(parents=True, exist_ok=True)
        out = img_dir / f'{sha}.{ext}'
        if not out.exists():
            out.write_bytes(raw)
        return str(out)

    def _update_thread_centroids(
            self,
            min_activation: float = 0.20,
            learning_rate: float = 0.05,
            ) -> int:
        """Stage 5 — incremental centroid evolution. Drift each active
        thread's centroid toward the current turn's embedding,
        weighted by how much that thread participated in the turn.

        Algorithm: for each active thread t with activation_t above
        min_activation:

            c_new  = c_old + lr * activation_t * (turn_emb - c_old)
            c_norm = c_new / ||c_new||
            n_new  = n_old + activation_t

        With lr=0.05 and activation=0.5 a single turn shifts the
        centroid by 2.5%; the constituent count grows by 0.5 (real-
        valued count, since membership is graded).

        min_activation defaults to 0.20, matching the secondary-render
        threshold in _render_active_threads_block: a thread that's
        below the 'meaningful enough to surface' threshold shouldn't
        drift toward the current turn either. The bge-small embedding
        space has a ~13-17% noise floor on cosine activation across
        unrelated threads; updating below the secondary threshold
        would smear all centroids on every off-topic query.

        Mutates the underlying note properties in-place; persistence
        is handled by the post-turn _persist_to_disk call. Returns
        the number of thread centroids actually updated."""
        activation = self._current_thread_activation
        turn_emb = self._current_turn_embedding
        if not activation or turn_emb is None:
            return 0
        try:
            import numpy as np
        except ImportError:
            logger.warning(
                f"[{self.character_name}] _update_thread_centroids: numpy unavailable")
            return 0

        now_iso = datetime.now(timezone.utc).isoformat()
        updated = 0
        for thread, weight in activation:
            if weight < min_activation:
                continue
            note_id = thread.get("id")
            if not note_id:
                continue
            note = self.resource_manager.get_resource(note_id)
            if not note:
                continue
            props = note.get("properties") or {}
            old_c = props.get("centroid_embedding") or []
            if not old_c:
                continue
            try:
                old_arr = np.asarray(old_c, dtype=np.float32)
                if old_arr.shape != turn_emb.shape:
                    logger.warning(
                        f"[{self.character_name}] centroid update: dim "
                        f"mismatch for thread {props.get('note_name')!r}, skipping")
                    continue
                step = float(learning_rate) * float(weight)
                new_arr = old_arr + step * (turn_emb - old_arr)
                norm = float(np.linalg.norm(new_arr))
                if norm > 0:
                    new_arr = new_arr / norm
                old_count = float(props.get("constituent_turn_count") or 0)
                new_count = old_count + float(weight)
                props["centroid_embedding"] = [float(x) for x in new_arr.tolist()]
                props["constituent_turn_count"] = new_count
                props["last_centroid_update_at"] = now_iso
                # last_activated_at marks the most recent turn this
                # thread had non-trivial weight on — useful for
                # consolidation / archival logic later.
                props["last_activated_at"] = now_iso
                updated += 1
            except Exception as e:
                logger.warning(
                    f"[{self.character_name}] centroid update for "
                    f"{props.get('note_name')!r} failed: {e}")
                continue
        if updated:
            logger.info(
                f"[{self.character_name}] thread centroids updated: "
                f"{updated}/{len(activation)} threads")
        return updated

    def _render_threads_for_subagent(
            self, threads: List[Dict[str, Any]]) -> str:
        """Render the full active-threads list as a system-prompt
        section for subagents (currently: remember). Distinct from
        _render_active_threads_block, which surfaces only the
        currently-activated threads to Jill's main turn — subagents
        operating over the whole memory benefit from seeing the full
        thread inventory regardless of current activation, since
        retrospective queries may target dormant-but-active threads."""
        if not threads:
            return ""
        lines: List[str] = []
        lines.append("## Session threads (activity-level structure of the conversation)")
        lines.append(
            "Threads are activity-level anchors inferred from the "
            "shape of the user's conversation. Each thread groups "
            "turns that participated in a coherent activity. Use these "
            "to (a) understand the topical structure of the user's "
            "interactions, (b) scope retrospective queries to the "
            "relevant thread when the question is about a specific "
            "activity, and (c) answer direct enumeration questions "
            "(\"what threads do you have?\") without needing to read "
            "memory files. Numbers are constituent turn counts.")
        lines.append("")
        for t in threads:
            name = t.get("name", "")
            n = int(t.get("constituent_turn_count", 0))
            summary = (t.get("summary") or "").strip()
            lines.append(f"- `{name}` ({n} turns): {summary}")
        return "\n".join(lines)

    def _run_remember(self, query: str) -> str:
        """Backend for the ReAct `recall` tool. Delegates to the
        active-recall subagent (chat.remember.remember), which runs its
        own thin ReAct loop over the memory dir. Returns an OK:/EMPTY:/
        ERROR: prefixed observation per the ReAct tool-observation
        convention. Subagent's full per-call trace lands under
        subagent_traces/ for debugging.

        Stage 4b: passes the active-threads inventory to the subagent
        as a system-prompt extension. The subagent gets activity-level
        structure for free without needing to grep files for it."""
        if not query or not str(query).strip():
            return "EMPTY: remember query was empty"
        try:
            from chat.remember import remember as _remember
            threads = self._get_threads(statuses=('active',))
            thread_context = self._render_threads_for_subagent(threads)
            answer = _remember(
                query=str(query),
                memory_dir=self._memory_dir(),
                llm_backend=self.backend,
                trace_dir=self._subagent_traces_dir(),
                thread_context=thread_context,
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] remember subagent raised: {e}")
            return f"ERROR: remember subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: remember subagent produced no answer'
        return 'OK: ' + text

    # Matches any <img ... src="http://127.0.0.1:8789/proxy?url=..."> regardless
    # of attribute order or surrounding tag content. Compiled once; used by the
    # preflight to validate proxy URLs before they reach the browser, where a
    # bad URL would render as a silent broken-image icon.
    _PROXY_IMG_RE = re.compile(
        r'''<img\b[^>]*?\bsrc\s*=\s*["'](http://127\.0\.0\.1:8789/proxy\?[^"']+)["']''',
        re.IGNORECASE,
    )

    def _preflight_proxy_imgs(self, content: str) -> Optional[str]:
        """If `content` references the local image proxy, GET each URL
        ahead of canvas publish. The proxy's Content-Type guard returns
        415 for non-image upstreams (most commonly a webpage URL the
        model mistakenly piped into <img>); preflight relays that to the
        ReAct loop as an ERROR observation so the next iteration can
        re-strategize instead of leaving a broken-icon on the canvas.

        Returns None if all proxy URLs validated, or a short reason
        string if any failed. Caller wraps with `ERROR:` prefix.
        """
        for m in self._PROXY_IMG_RE.finditer(content):
            url = m.group(1)
            try:
                r = requests.get(url, timeout=5)
            except requests.RequestException as e:
                return f"proxy unreachable for {url}: {e}"
            if r.status_code != 200:
                # The proxy emits an actionable hint in the body
                # (e.g. "upstream returned 'text/html', not an image…").
                # Relay it verbatim so Jill sees what to do next.
                hint = (r.text or '').strip().replace('\n', ' ')[:400]
                return (
                    f"proxy rejected {url} (HTTP {r.status_code})"
                    + (f" — {hint}" if hint else '')
                )
        return None

    def _run_display(self, content: str, fmt: str) -> str:
        """Backend for the ReAct `display` tool. Publishes the content to
        the canvas Zenoh key; the display window (launched separately or
        via launcher --canvas) renders markdown or HTML. Non-terminal:
        the ReAct loop continues after this — Jill should follow up with
        a `respond` so the conversation gets a textual reply too."""
        if content is None:
            return "ERROR: display content was None"
        body = str(content)
        if not body.strip():
            return "EMPTY: display content was empty"
        if fmt not in ('markdown', 'html'):
            fmt = 'markdown'
        # Validate any proxy URLs before publish so a page-URL-as-image
        # bug surfaces here, not as a broken-icon on the user's screen.
        if fmt == 'html':
            err = self._preflight_proxy_imgs(body)
            if err is not None:
                return f"ERROR: display preflight: {err}"
        try:
            n_bytes = self._canvas.set_content(body, fmt=fmt)
        except Exception as e:
            logger.warning(f"[{self.character_name}] canvas publish failed: {e}")
            return f"ERROR: canvas publish failed: {e}"
        return f"OK: rendered to canvas ({n_bytes} bytes, format={fmt})"

    def _inspect_traces_dir(self) -> 'Path':
        """Where the inspect (codebase-query) subagent writes its per-call
        traces. Sibling of subagent_traces/ — kept separate so per-tool
        debugging stays clean. Layout:
        scenarios/<world>/<agent>/inspect_traces/."""
        return self._memory_dir().parent / 'inspect_traces'

    def _run_inspect(self, query: str) -> str:
        """Backend for the ReAct `inspect` tool. Delegates to the
        codebase-inspection subagent (chat.code_subagent.inspect), which
        runs its own thin ReAct loop over src/ with read-only primitives
        (list, read, grep via ripgrep). Returns an OK:/EMPTY:/ERROR:
        prefixed observation per the ReAct tool-observation convention.

        Uses Jill's main backend (self.backend) — the same model the
        outer ReAct loop runs on. Per-scenario YAML decides the model;
        no per-subagent backend overrides. Per-call traces under
        inspect_traces/."""
        if not query or not str(query).strip():
            return "EMPTY: inspect query was empty"
        try:
            from chat.code_subagent import inspect as _inspect
            answer = _inspect(
                query=str(query),
                repo_root=Path(_SRC_DIR),
                llm_backend=self.backend,
                trace_dir=self._inspect_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] inspect subagent raised: {e}")
            return f"ERROR: inspect subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: inspect subagent produced no answer'
        return 'OK: ' + text

    def _run_inspect_external(self, query: str) -> str:
        """Backend for the ReAct `inspect_external` tool. Delegates to the
        same subagent loop as `inspect`, geofenced to the bound external
        repo (see _set_external_repo / _get_external_repo). Returns ERROR
        if no repo is currently bound — the system prompt only advertises
        this tool when a binding exists, so a call here without binding
        means the model fabricated the tool."""
        if not query or not str(query).strip():
            return "EMPTY: inspect_external query was empty"
        repo = self._get_external_repo()
        if repo is None:
            return ("ERROR: no external repo is bound for this session "
                    "(use /set-external-repo <path> first)")
        try:
            from chat.code_subagent import inspect_external as _inspect_external
            answer = _inspect_external(
                query=str(query),
                repo_root=repo,
                llm_backend=self.backend,
                trace_dir=self._inspect_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] inspect_external subagent raised: {e}")
            return f"ERROR: inspect_external subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: inspect_external subagent produced no answer'
        return 'OK: ' + text

    def _security_traces_dir(self) -> 'Path':
        """Where the security (network-investigation) subagent writes its
        per-call traces. Sibling of inspect_traces/ — kept separate so
        per-tool debugging stays clean. Layout:
        scenarios/<world>/<agent>/security_traces/."""
        return self._memory_dir().parent / 'security_traces'

    def _run_security(self, query: str) -> str:
        """Backend for the ReAct `security` tool. Delegates to the
        network-security investigation subagent (chat.security.security),
        which runs its own thin ReAct loop with read-only typed primitives
        (discover, scan_services, system_state via nmap + iproute2).
        Returns an OK:/EMPTY:/ERROR: prefixed observation per the ReAct
        tool-observation convention.

        Uses Jill's main backend (self.backend) — see _run_inspect for
        rationale. Per-call traces under security_traces/."""
        if not query or not str(query).strip():
            return "EMPTY: security query was empty"
        try:
            from chat.security import security as _security
            answer = _security(
                query=str(query),
                llm_backend=self.backend,
                trace_dir=self._security_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] security subagent raised: {e}")
            return f"ERROR: security subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: security subagent produced no answer'
        return 'OK: ' + text

    def _append_conversation_entry(self, direction: str, entity: str,
                                   text: str, meta: str = '') -> None:
        """Append one conversation entry (incoming or outgoing) to
        <memory>/conversation.txt as plain timestamped prose. One section
        per message. The active-recall subagent reads this for queries
        about what was said this session and prior."""
        try:
            mem_dir = self._memory_dir()
            mem_dir.mkdir(parents=True, exist_ok=True)
            path = mem_dir / 'conversation.txt'
            ts = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
            arrow = '->' if direction == 'in' else '<-'
            header = f'[{ts}] {entity} {arrow} {self.character_name}'
            if meta:
                header += f' ({meta})'
            block = '\n'.join([
                '',
                '=' * 80,
                header,
                '=' * 80,
                (text or '').rstrip(),
                '',
            ])
            with open(path, 'a', encoding='utf-8') as f:
                f.write(block + '\n')
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] conversation log append "
                f"({direction}/{entity}) failed: {e}"
            )

    def _write_state_snapshot(self, kind: str, entity: str, content: str) -> None:
        """Write a current-value state snapshot to memory as raw .txt,
        overwriting any prior version. Used for companion_state and
        discourse_state — LLM-emitted prose with section headers, kept
        as plain text so the subagent can grep them naturally. mtime
        provides the 'last updated' timestamp."""
        try:
            mem_dir = self._memory_dir()
            mem_dir.mkdir(parents=True, exist_ok=True)
            path = mem_dir / f'{kind}_{entity}.txt'
            path.write_text(content, encoding='utf-8')
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] state snapshot write "
                f"({kind}/{entity}) failed: {e}"
            )

    def _write_react_trace(self, source: str, user_text: str,
                           log: List[Tuple[str, str]],
                           iters: List[Dict[str, Any]],
                           final_response: str,
                           exit_reason: str,
                           recall: Optional[List[Tuple[str, str, str]]] = None,
                           image_ref: Optional[str] = None) -> None:
        """Append one ReAct session to <memory>/chat_trace.txt as the
        literal byte-stream that was sent to the LLM, with no editorial
        annotation. Each iteration is one (USER, ASSISTANT) pair; the
        SYSTEM message is shown once (it's byte-stable across iterations).
        A single dim header line per turn carries minimal metadata
        (timestamp / source / iters / exit) so multiple turns in the same
        file are separable, and that's it."""
        try:
            mem_dir = self._memory_dir()
            mem_dir.mkdir(parents=True, exist_ok=True)
            trace_path = mem_dir / 'chat_trace.txt'
            ts = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
            n_iters = len(iters)
            header = f'[{ts}] source={source} iters={n_iters} exit={exit_reason}'
            if image_ref:
                header += f' image={image_ref}'
            lines: List[str] = [
                '',
                '=' * 80,
                header,
                '=' * 80,
                '',
            ]
            if iters:
                first = iters[0]
                lines.append('SYSTEM:')
                lines.append(first.get('system', ''))
                lines.append('')
                for it in iters:
                    lines.append('USER:')
                    lines.append(it.get('user', ''))
                    lines.append('')
                    lines.append('ASSISTANT:')
                    lines.append(it.get('raw', ''))
                    lines.append('')
            with open(trace_path, 'a', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
        except Exception as e:
            logger.warning(f"[{self.character_name}] react trace write failed: {e}")

    # ------------------------------------------------------------------
    # Reasoning history (awareness feed)
    # ------------------------------------------------------------------

    def _write_reasoning_history(self, source: str, user_text: str,
                                  iters: List[Dict[str, Any]],
                                  final_response: str,
                                  exit_reason: str,
                                  orientation: str = '',
                                  recall: Optional[List[Tuple[str, str, str]]] = None,
                                  agent_concerns: Optional[List[Any]] = None,
                                  user_concerns: Optional[List[Any]] = None,
                                  autonomous: bool = False,
                                  image_ref: Optional[str] = None) -> None:
        """Append one structured ReAct turn record to
        <memory>/reasoning_trace.jsonl. Per-turn-unique content (orientation,
        active concerns at this turn, recall hits at this turn, user input,
        working log, raw response, companion/discourse state at this turn)
        is stored inline. The list of trace turn_seqs that were visible in
        this turn's ## Recent reasoning block is captured in
        prefix_trace_refs (set as side effect by
        _get_reasoning_history_block at prompt build time) — this gives a
        faithful record of awareness window per turn without duplicating
        prior trace content."""
        try:
            self._turn_seq += 1

            # Build working_log as literal text from iters: per-iter raw
            # action emission + bound $stepN observation, untruncated.
            log_lines: List[str] = []
            for i, it in enumerate(iters):
                raw = (it.get('raw') or '').strip()
                log_lines.append(f"--- iter {i+1} ---")
                log_lines.append(f"ACTION: {raw}")
                for label, content in (it.get('appended') or []):
                    if isinstance(label, str) and label.startswith('$step'):
                        log_lines.append(f"{label}: {content}")
            working_log = "\n".join(log_lines)

            # Snapshot per-turn state — flatten concerns and recall to
            # readable strings; companion/discourse stored inline (small,
            # slowly-varying). Per-turn snapshot of these is what makes
            # the trace faithful for "what was I operating under at T11"
            # probes.
            # Concerns snapshot: flatten each list to "[kind activation/strength] text"
            # so the trace stays human-grep-able without losing the asymmetry.
            concerns_at_turn: List[str] = []
            for item in (agent_concerns or []):
                if isinstance(item, tuple) and len(item) >= 3:
                    _nid, text, weight = item[0], item[1], item[2]
                    concerns_at_turn.append(f"[agent {weight:.2f}] {text}")
                else:
                    concerns_at_turn.append(f"[agent] {item}")
            for item in (user_concerns or []):
                if isinstance(item, tuple) and len(item) >= 3:
                    _nid, text, weight = item[0], item[1], item[2]
                    concerns_at_turn.append(f"[user {weight:.2f}] {text}")
                else:
                    concerns_at_turn.append(f"[user] {item}")
            recall_at_turn: List[str] = []
            for item in (recall or []):
                if isinstance(item, tuple) and len(item) >= 2:
                    recall_at_turn.append(f"[{item[0]}] {item[1]}")
                else:
                    recall_at_turn.append(str(item))

            record = {
                'turn_seq': self._turn_seq,
                'ts': datetime.now(timezone.utc).isoformat(),
                'source': source,
                'autonomous': bool(autonomous),
                'exit_reason': exit_reason,
                'iters': len(iters or []),
                'user_input': user_text,
                'orientation': orientation or '',
                'active_concerns': concerns_at_turn,
                'recall_hits': recall_at_turn,
                'companion_state': str(self._companion_state.get(source, '') or ''),
                'discourse_state': str(self._discourse_state.get(source, '') or ''),
                'prefix_trace_refs': list(self._last_inject_trace_seqs),
                'working_log': working_log,
                'raw_response': final_response or '',
            }
            if image_ref:
                record['image_ref'] = image_ref

            path = self._reasoning_trace_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _write_reasoning_history failed: {e}")

    def _render_trace_record(self, rec: Dict[str, Any], full: bool) -> str:
        """Render a single per-turn record from reasoning_trace.jsonl
        for inclusion in the user-prompt's ## Recent reasoning block.
        full=True renders all per-turn-unique fields (used for recent
        traces and all traces in benchmark_mode); full=False renders a
        one-line digest (used for older traces in non-benchmark mode)."""
        seq = int(rec.get('turn_seq', 0))
        if not full:
            user_short = (rec.get('user_input') or '')[:60].replace('\n', ' ')
            resp_short = (rec.get('raw_response') or '').replace('\n', ' ')[:200]
            return f"### trace #{seq}: '{user_short}' → \"{resp_short}\""
        parts: List[str] = [f"### trace #{seq}"]
        ts = rec.get('ts')
        if ts:
            parts.append(f"TIMESTAMP: {ts}")
        if rec.get('orientation'):
            parts.append(f"ORIENTATION:\n{rec['orientation']}")
        if rec.get('active_concerns'):
            parts.append("ACTIVE CONCERNS:")
            for c in rec['active_concerns']:
                parts.append(f"  - {c}")
        if rec.get('recall_hits'):
            parts.append("RECALL HITS:")
            for r in rec['recall_hits']:
                parts.append(f"  - {r}")
        prefix_refs = rec.get('prefix_trace_refs') or []
        if prefix_refs:
            parts.append(
                f"PRIOR TRACES VISIBLE TO ME AT THIS TURN: {prefix_refs} "
                "(by turn_seq; resolve via reasoning_trace.jsonl line N)")
        parts.append(f"USER INPUT: {rec.get('user_input', '')}")
        if rec.get('image_ref'):
            parts.append(f"IMAGE ATTACHED: {rec['image_ref']}")
        if rec.get('working_log'):
            parts.append(f"WORKING LOG:\n{rec['working_log']}")
        if rec.get('raw_response'):
            parts.append(f"RAW RESPONSE: {rec['raw_response']}")
        return "\n".join(parts)

    def _get_reasoning_history_block(self) -> str:
        """Build the ## Recent reasoning block for the user-message prefix.
        Reads structured per-turn records from reasoning_trace.jsonl and
        renders per-turn-unique content (orientation, concerns at the
        time, recall, user input, working log, raw response, plus
        prefix_trace_refs as a faithful awareness-window record).

        Side effect: sets self._last_inject_trace_seqs to the list of
        turn_seqs that were rendered, used by the next
        _write_reasoning_history call to populate prefix_trace_refs.

        In benchmark_mode: surfaces ALL session records in full.
        In normal chat mode: last _REASONING_HISTORY_RECENT records,
        most recent _REASONING_HISTORY_FULL in full, older as one-line
        digest (token budget)."""
        self._last_inject_trace_seqs = []
        records = self._load_reasoning_records()
        if not records:
            return ''
        bench = bool((self.config.get('chat') or {}).get('benchmark_mode'))
        selected = records if bench else records[-_REASONING_HISTORY_RECENT:]
        n = len(selected)
        full_threshold = 0 if bench else max(0, n - _REASONING_HISTORY_FULL)
        sections: List[str] = []
        for idx, rec in enumerate(selected):
            seq = int(rec.get('turn_seq', 0))
            if seq:
                self._last_inject_trace_seqs.append(seq)
            sections.append(self._render_trace_record(
                rec, full=(idx >= full_threshold)))
        if not sections:
            return ''
        return ("## Recent reasoning (my own ReAct traces from prior turns — "
                "per-turn orientation, concerns, recall, user input, working "
                "log, and raw response. The static prefix and current state "
                "above also conditioned each turn; only per-turn-varying "
                "content is shown here.)\n"
                + "\n\n".join(sections))

    def _react_fallback_synthesis(self, log: List[Tuple[str, str]], fail_reason: str = '') -> str:
        """Force a Jill-voice reply when respond never fired."""
        if not log:
            return f"(I couldn't formulate a response.{(' — ' + fail_reason) if fail_reason else ''})"
        summary = "\n".join(f"{label}: {content[:400]}" for label, content in log)
        sys_msg = (f"You are {self.character_name}, speaking in first person. The ReAct "
                   "loop did not exit cleanly. Synthesize the log into a reply, in your voice. "
                   "If incomplete, say so honestly rather than fabricate.")
        user_msg = (f"Log:\n{summary}"
                    + (f"\n\n(Failure: {fail_reason})" if fail_reason else "")
                    + "\n\nWrite the response now. No preamble.")
        try:
            result = self.backend.chat(
                [{'role': 'system', 'content': sys_msg}, {'role': 'user', 'content': user_msg}],
                max_tokens=4096, temperature=0.7, cot_profile='none')
            return (result or '').strip() or "(could not synthesize)"
        except Exception as e:
            return f"(trouble formulating a response: {e})"

    def _maybe_spawn_successor_concern(self, parent_id: str, parent_instruction: str,
                                       log: List[Tuple[str, str]]) -> Optional[str]:
        """When an autonomous ReAct loop hits max_iters, decide whether
        to spawn a successor concern carrying the narrowed remainder of
        the work. Returns the new concern's id, or None if no successor
        was spawned (work complete, depth cap reached, parse/LLM failure,
        or LLM judged the remainder not worth pursuing).

        Successor depth is capped at _CONCERN_SUCCESSOR_MAX_DEPTH so
        chains can't run away. The successor is created with skip_recurrence
        so it doesn't fold back into the parent via similarity merge.
        """
        parent = self.resource_manager.get_resource(parent_id)
        if not parent:
            return None
        parent_props = parent.get('properties') or {}
        parent_depth = int(parent_props.get('successor_depth', 0) or 0)
        if parent_depth >= _CONCERN_SUCCESSOR_MAX_DEPTH:
            logger.info(
                f"[{self.character_name}] successor cap reached for {parent_id} "
                f"(depth={parent_depth}); standing on fallback response.")
            return None
        # Show the synthesizer enough log to judge what's left, but not
        # so much it drowns in detail. Last 10 entries, content trimmed.
        tail = log[-10:] if len(log) > 10 else log
        summary = "\n".join(f"{label}: {content[:300]}" for label, content in tail)
        sys_msg = (
            f"You evaluate whether a {self.character_name}-side ReAct loop "
            "completed its instruction. The loop ran to its iteration cap "
            "without emitting a final `respond` action. Decide: did the "
            "work substantively complete (just missing the wrap-up), or "
            "is real work left? If left, what is the narrow next slice "
            "(one ReAct loop's worth, ~12 tool calls) that should run "
            "next?\n\nRespond ONLY with JSON, no prose, no markdown."
        )
        user_msg = (
            f"Original instruction: {parent_instruction}\n\n"
            f"ReAct working log (last entries):\n{summary}\n\n"
            "JSON shapes:\n"
            "  {\"complete\": true}                                  ← work substantively done\n"
            "  {\"complete\": false, \"next_slice\": \"<imperative>\"}  ← real work remains"
        )
        try:
            raw = self.backend.chat(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': user_msg}],
                max_tokens=4096, temperature=0.2, cot_profile='none')
        except Exception as e:
            logger.warning(f"[{self.character_name}] successor synth LLM failed: {e}")
            return None
        data = repair_json_string(raw or '')
        if data is None:
            logger.warning(
                f"[{self.character_name}] successor JSON parse failed; "
                f"raw={(raw or '')[:200]!r}")
            return None
        if not isinstance(data, dict) or data.get('complete'):
            return None
        next_slice = str(data.get('next_slice') or '').strip()
        if not next_slice:
            return None
        # Successor inherits the parent's surface text + a depth tag so it
        # reads coherently in the active-concerns surface.
        parent_text = str(parent_props.get('content', '') or parent.get('text', '') or '').strip()
        new_depth = parent_depth + 1
        succ_text = (f"{parent_text} — continuation (depth {new_depth})"
                     if parent_text else f"continuation of {parent_id} (depth {new_depth})")
        # Successor: same kind as parent (agent_concern). Fast rhythm
        # so the remainder runs promptly. Pre-loaded activation so it
        # fires on the next tick without waiting for full growth from
        # 0 — successors carry the parent's not-quite-finished work.
        new_id = self._add_agent_concern(
            text=succ_text, entity='User', provenance='inferred',
            seed=False, name='',
            rhythm_hours=1, rhythm_source='urgency',
            instruction=next_slice,
            skip_recurrence=True,
            extra_properties={
                'successor_of': parent_id,
                'successor_depth': new_depth,
                # Prime activation just below threshold so the next tick's
                # growth pushes it over without making the spawn itself
                # eligible (avoids re-firing in the same tick window).
                'activation': max(0.0,
                                  _AGENT_CONCERN_FIRE_THRESHOLD - 0.1),
            },
        )
        if not new_id:
            return None
        logger.info(
            f"[{self.character_name}] spawned successor concern {new_id} for "
            f"{parent_id} (depth {new_depth}): {next_slice[:120]!r}")
        return new_id

    # ------------------------------------------------------------------
    # Per-turn handling
    # ------------------------------------------------------------------

    def _post_turn_work_tracked(self, source: str, close: bool) -> None:
        """Wrapper around _post_turn_work that toggles _post_turn_busy
        for /status visibility. Always clears the flag on exit, success
        or failure. _post_turn_busy is set EITHER here at entry OR by
        the caller right before submit() — both flag-on points are
        idempotent. Clear is centralized here in finally so the flag
        can't get stuck if _post_turn_work raises."""
        self._post_turn_busy = True
        try:
            self._post_turn_work(source, close)
        finally:
            self._post_turn_busy = False

    def _post_turn_work(self, source: str, close: bool) -> None:
        """Background side-effect job for one turn: discourse update,
        reflection (memory writes), optional dialog close, autosave.

        Runs on the post-turn executor (single worker) so jobs are
        sequential per character. Failure-tolerant: any exception is
        logged but does not propagate out of the executor (the main
        thread is no longer here to receive it anyway).
        """
        try:
            self._update_discourse_async(source)
            # Reflection runs after discourse so it can see the latest
            # companion state and avoid duplicating it.
            self._reflect_and_remember(source)
            # Stage 5: drift active thread centroids toward the
            # current turn's embedding, weighted by thread activation.
            # No-op if no threads, no embedding, or all activations
            # below threshold. In-place property mutation; the
            # _persist_to_disk below handles save.
            try:
                self._update_thread_centroids()
            except Exception as e:
                logger.warning(
                    f"[{self.character_name}] thread centroid update failed: {e}")
            if close:
                self.store.close_dialog(source)
            # Autosave. Memories written above land on disk here; without
            # this, a SIGINT mid-job loses the turn's memory updates.
            self._persist_to_disk()
        except Exception as e:
            logger.warning(f"[{self.character_name}] post-turn work failed: {e}")

    def _process_user_turn(self, source: str, text: str, close: bool,
                           autonomous: bool = False,
                           autonomous_concern_id: Optional[str] = None,
                           image_url: Optional[str] = None) -> None:
        """Drive one turn through the ReAct loop. The autonomous path
        (autonomous=True) reuses the same prompt construction so Jill's
        voice stays consistent and traces share format. Divergences are
        narrow: skip record_incoming (no fake user turn), suppress recall
        refresh (read-only — autonomous engagement is NOT user engagement),
        skip post-turn reflection/discourse (no user side to model from),
        and bump last_acted_at on the fired concern after success.
        """
        # Mark the turn in flight for /status. Cleared immediately after
        # _publish_say so a fresh user input isn't blocked-on-status by
        # the trace-write / post-turn reflection that follows. Use a
        # try/finally so a crash mid-turn doesn't leave _current_turn
        # set forever.
        self._current_turn = {
            'kind': 'autonomous' if autonomous else 'user',
            'source': source,
            'text_preview': (text or '')[:120],
            'started_at': datetime.now(timezone.utc).isoformat(),
            'autonomous_concern_id': autonomous_concern_id,
        }
        self._affect.set_trigger('autonomous' if autonomous else 'user')
        self._affect.set_mode('thinking')
        try:
            self._process_user_turn_inner(
                source, text, close,
                autonomous=autonomous,
                autonomous_concern_id=autonomous_concern_id,
                image_url=image_url)
        finally:
            self._current_turn = None
            self._affect.set_mode('idle')

    def _process_user_turn_inner(self, source: str, text: str, close: bool,
                                 autonomous: bool = False,
                                 autonomous_concern_id: Optional[str] = None,
                                 image_url: Optional[str] = None) -> None:
        # Reject image input early when the active backend route can't
        # carry it (Anthropic native, legacy cloud_llm). Bail before any
        # state mutation; let the user retry without the image.
        if image_url and not self.backend.supports_image_input:
            warn = (f"backend {self.backend.server!r} does not currently "
                    f"support image input — switch backends or omit the image.")
            logger.warning(f"[{self.character_name}] {warn}")
            self._publish_say(f"[{self.character_name}] {warn}")
            return

        # Empty-caption default: if an image arrived with no caption,
        # supply a directive so iter 1 has something to react to. CLI
        # already injects this; the safety net covers other transports.
        if image_url and not text:
            text = "Describe this image."

        # Persist image once on first arrival (data URIs only — http(s)
        # URLs are referenced as-is). Returns either a sha-keyed local
        # path (for the trace) or None.
        image_ref: Optional[str] = None
        if image_url:
            try:
                image_ref = self._persist_image_url(image_url)
            except Exception as e:
                logger.warning(f"[{self.character_name}] image persist failed: {e}")
                image_ref = None
            if image_ref is None and image_url.startswith(('http://', 'https://')):
                # URL passthrough — trace stores the URL itself.
                image_ref = image_url

        if not autonomous:
            self.store.record_incoming(source, text, close=close)
            self._append_conversation_entry(
                'in', source, text, meta=f'close={close}')

        if not text:
            if close:
                self.store.close_dialog(source)
            return

        # User-turn dynamics for user_concerns: decay all strengths,
        # then bump those that semantic-match this input. Skipped on
        # autonomous turns (the agent talking to itself doesn't shift
        # the user model). Order: decay first so a fresh mention's
        # full bump prevails over the same-turn decay step.
        if not autonomous:
            self._decay_user_concerns_per_turn()
            self._bump_user_concerns_on_input(text)

        # Compute thread activation distribution for the current turn.
        # Read by _build_system_prompt for the "current activity context"
        # block and (Stage 4b) by the remember subagent. Cheap — single
        # embedding + N cosine sims for N active threads. Skipped on
        # autonomous fires: those are agent-initiated (a concern's
        # instruction firing), not user activity, and shouldn't drive
        # thread surfacing or centroid drift.
        if not autonomous:
            self._current_thread_activation = self._compute_thread_activation(text)
        else:
            self._current_thread_activation = []
            self._current_turn_embedding = None

        orientation = self._orientation_summary(source, text)
        # Auto-RAG: pull top-k durable memories that match this turn's input.
        # Injected next to the Companion block in the system prompt; no ReAct
        # tool call required. Cheap miss (one embedding query).
        recall = self._recall(text, k=3)
        # Concerns surface: user_concerns (top by strength) + agent_concerns
        # (top by activation). Two separate lists — the prompt builder
        # renders each in its own section. No firing on user turns;
        # autonomous fires happen on tick via _handle_tick.
        agent_concerns = self._top_active_agent_concerns()
        user_concerns = self._top_active_user_concerns()

        log: List[Tuple[str, str]] = []
        iters: List[Dict[str, Any]] = []
        exit_reason = 'crashed'
        try:
            reply, log, iters, exit_reason = self._run_react_loop(
                source, text, orientation, recall=recall,
                agent_concerns=agent_concerns, user_concerns=user_concerns,
                image_url=image_url)
        except Exception as e:
            logger.error(f"[{self.character_name}] ReAct loop crashed: {e}")
            import traceback
            traceback.print_exc()
            reply = f"[{self.character_name}] I had trouble generating a reply: {e}"

        reply = (reply or '').strip()
        # Autonomous silence is a first-class outcome: a fired concern with a
        # "Silent on healthy" instruction (e.g. PV monitor) intentionally
        # produces no reply on the happy path. Skip the Telegram/Zenoh push
        # in that case — there's no user waiting for an ack — but keep the
        # "(no reply)" rewrite for the trace/store/CLI coda so /status and
        # autonomy logs still show that the concern fired and ran clean.
        intentionally_silent = autonomous and not reply
        if not reply:
            reply = "(no reply)"

        act_type = 'auto_say' if autonomous else 'say'
        self.store.record_outgoing(source, reply, act_type=act_type, close=close)
        self._append_conversation_entry(
            'out', source, reply, meta=f'act={act_type} close={close}')
        if not intentionally_silent:
            self._publish_say(reply)
        self._last_reply_at = datetime.now(timezone.utc).isoformat()
        logger.info(f"[{self.character_name}] -> {source} ({act_type}): {reply!r}")

        # Service the fired agent_concern AFTER successful reply
        # publication. Decrements activation per exit_reason and stamps
        # last_fired_at — the autonomous run has now actually executed.
        if autonomous and autonomous_concern_id:
            self._service_agent_concern(autonomous_concern_id, exit_reason)

        # Successor-concern spawn. When an autonomous ReAct loop hits its
        # iter cap, the work likely isn't done — synthesize what's left
        # into a narrow successor concern so the next tick can pick up
        # where this one left off (capped by _CONCERN_SUCCESSOR_MAX_DEPTH).
        if autonomous and autonomous_concern_id and exit_reason == 'max_iters':
            try:
                succ_id = self._maybe_spawn_successor_concern(
                    autonomous_concern_id, text, log)
                if succ_id:
                    self._write_autonomy_event({
                        'event': 'successor_spawned',
                        'parent_concern_id': autonomous_concern_id,
                        'successor_concern_id': succ_id,
                    })
            except Exception as e:
                logger.warning(
                    f"[{self.character_name}] successor spawn failed for "
                    f"{autonomous_concern_id}: {e}")

        # Trace write goes immediately after publish, before any slow
        # post-turn LLM work, so format_prompt can read the reasoning
        # trace without waiting on discourse/reflection. Skipped if we
        # never entered the ReAct loop (pre-loop crash above).
        if iters:
            self._write_react_trace(
                source, text, log, iters, reply, exit_reason, recall=recall,
                image_ref=image_ref)
            # Awareness feed: persist this turn's trace into
            # reasoning_history so the next turn's prompt prefix can
            # surface it. Same iters list — different store, different
            # render. Both autonomous and user-driven turns write here:
            # all reasoning is Jill's.
            self._write_reasoning_history(
                source, text, iters, reply, exit_reason,
                orientation=orientation, recall=recall,
                agent_concerns=agent_concerns, user_concerns=user_concerns,
                autonomous=autonomous, image_ref=image_ref)

        # Post-turn work (discourse + reflection + close + persist) is
        # ~5-15s of LLM calls and conceptually a side effect of the turn.
        # Skipped on autonomous turns — there's no user side to update
        # discourse/companion from, and reflection over a Jill-only
        # monologue would pollute the memory store. We still need a
        # disk persist so last_acted_at survives a restart.
        if autonomous:
            try:
                self._persist_to_disk()
            except Exception as e:
                logger.warning(f"[{self.character_name}] autonomous persist failed: {e}")
            return

        if self.benchmark_mode:
            # Run reflection inline so a benchmark harness can snapshot
            # state immediately after the call returns and see fully-resolved
            # memory/concern writes from this turn.
            self._post_turn_work_tracked(source, close)
        else:
            try:
                self._post_turn_busy = True
                self._post_turn_executor.submit(
                    self._post_turn_work_tracked, source, close)
            except RuntimeError:
                # Executor already shut down (process tearing down).
                # Fall back to synchronous so nothing is silently dropped.
                self._post_turn_work_tracked(source, close)

    # ------------------------------------------------------------------
    # Tick handler — Phase C autonomy entry point.
    # Triggered by the `tick` sensor's heartbeat. Gated by
    # self._autonomy_enabled (launcher --autonomy); when off, this is a
    # no-op so existing benchmarks and chat scenarios behave identically.
    # When on: grows agent_concern activations, identifies any whose
    # activation has crossed _AGENT_CONCERN_FIRE_THRESHOLD AND have an
    # instruction, then dispatches up to _AUTONOMOUS_FIRE_CAP concerns
    # through the standard ReAct pipeline. Surplus concerns stay above
    # threshold and surface on the next tick.
    #
    # Both the growth pass and the fire-check are deliberately LLM-free
    # — pure activation arithmetic — so this path stays cheap on ticks
    # where nothing is due. Do not introduce LLM calls into the
    # tick→fire decision; reasoning happens inside the ReAct loop a
    # fired concern dispatches, gated by activation threshold.
    # ------------------------------------------------------------------

    _AUTONOMOUS_FIRE_CAP = 2

    def _handle_tick(self) -> None:
        """Per-tick autonomy pass. No-op when --autonomy is off.
        When on: grows activations, identifies fire-eligible concerns,
        runs up to _AUTONOMOUS_FIRE_CAP ReAct loops on their instructions,
        prints a CLI preamble + coda per fire, a deferral note if any are
        dropped, and appends one JSONL record per event to
        <memory>/autonomy.jsonl.
        """
        if not self._autonomy_enabled:
            return
        # Grow first, then check — ensures activations reflect elapsed
        # time before the fire decision.
        self._grow_agent_concerns_per_tick()
        fired = self._check_and_fire_agent_concerns()
        if not fired:
            return
        # Cap dispatch; the rest stay due (last_acted_at unchanged) and
        # surface on the next tick.
        to_run = fired[:self._AUTONOMOUS_FIRE_CAP]
        deferred = fired[self._AUTONOMOUS_FIRE_CAP:]

        if deferred:
            try:
                if sys.stdout.isatty():
                    sys.stdout.write(
                        f"\n{self._STATUS_DIM}[{self.character_name}] "
                        f"{len(deferred)} additional concern(s) deferred to next tick.{self._STATUS_RESET}\n"
                    )
                    sys.stdout.flush()
                else:
                    logger.info(
                        f"[{self.character_name}] {len(deferred)} concern(s) deferred to next tick")
            except Exception as e:
                logger.warning(f"[{self.character_name}] deferred-print failed: {e}")
            for nid, text, instruction in deferred:
                self._write_autonomy_event({
                    'event': 'deferred',
                    'concern_id': nid,
                    'concern_text': text,
                    'instruction': instruction,
                })

        for nid, text, instruction in to_run:
            # CLI preamble: tell the user what's about to happen before
            # the autonomous reply lands.
            preamble = (
                f"auto-firing concern: {text}\n"
                f"        → {instruction}"
            )
            try:
                if sys.stdout.isatty():
                    sys.stdout.write(
                        f"\n{self._STATUS_DIM}[{self.character_name}] "
                        f"{preamble}{self._STATUS_RESET}\n"
                    )
                    sys.stdout.flush()
                else:
                    logger.info(f"[{self.character_name}] {preamble}")
            except Exception as e:
                logger.warning(f"[{self.character_name}] preamble-print failed: {e}")

            started_at = datetime.now(timezone.utc)
            outcome: Dict[str, Any] = {
                'event': 'fire',
                'concern_id': nid,
                'concern_text': text,
                'instruction': instruction,
                'started_at': started_at.isoformat(),
            }
            # Imperative wrapper. Without it, instruction bodies written
            # as reference docs ("InfluxDB lives at...") get acknowledged
            # rather than executed. The framing forces "act now" and
            # supplies the firing mode for instructions that branch on it.
            wrapped_instruction = (
                f"A concern of mine has fired: {text}\n"
                f"Mode: autonomous\n\n"
                f"Execute the following procedure now and produce the "
                f"appropriate output. If the procedure specifies silence "
                f"under some condition, stay silent.\n\n"
                f"{instruction}"
            )
            try:
                # source=self.character_name marks this turn as agent-
                # originated; `_process_user_turn` skips record_incoming
                # on autonomous=True so no fake user message is logged.
                # The conversation history surfaced into the prompt is
                # still the User dialogue (see _build_react_user_prefix).
                self._process_user_turn(
                    source=self.character_name, text=wrapped_instruction, close=False,
                    autonomous=True, autonomous_concern_id=nid)
                outcome['terminated'] = 'ok'
            except Exception as e:
                logger.error(f"[{self.character_name}] autonomous fire failed for {nid}: {e}")
                import traceback
                traceback.print_exc()
                outcome['terminated'] = 'crashed'
                outcome['error'] = str(e)

            finished_at = datetime.now(timezone.utc)
            outcome['finished_at'] = finished_at.isoformat()
            outcome['duration_s'] = round((finished_at - started_at).total_seconds(), 2)

            # Pull the just-written reasoning trace tail to enrich the
            # autonomy record (iters, exit_reason, response). Best effort
            # — if the trace can't be read, the JSONL line still has the
            # core fields.
            try:
                recs = self._load_reasoning_records()
                if recs:
                    last = recs[-1]
                    outcome['react_iters'] = int(last.get('iters', 0) or 0)
                    outcome['react_exit_reason'] = last.get('exit_reason', '')
                    # Trace stores the synthesized response under
                    # `raw_response`; previously this read `reply` and
                    # always came back empty.
                    reply = str(last.get('raw_response', '') or '')
                    outcome['response_chars'] = len(reply)
                    outcome['response_brief'] = reply[:200]
            except Exception as e:
                logger.warning(f"[{self.character_name}] autonomy-trace read failed: {e}")

            self._write_autonomy_event(outcome)

            # Post-fire CLI coda: closes the visual loop the preamble opens.
            iters_n = outcome.get('react_iters', '?')
            term = outcome.get('react_exit_reason') or outcome.get('terminated', '?')
            dur = outcome.get('duration_s', '?')
            brief = (outcome.get('response_brief') or '').replace('\n', ' ')
            if len(brief) > 80:
                brief = brief[:77] + '…'
            coda = f"auto-fire complete: {iters_n} iters, {dur}s, {term} — {brief}"
            try:
                if sys.stdout.isatty():
                    sys.stdout.write(
                        f"{self._STATUS_DIM}[{self.character_name}] "
                        f"{coda}{self._STATUS_RESET}\n"
                    )
                    sys.stdout.flush()
                else:
                    logger.info(f"[{self.character_name}] {coda}")
            except Exception as e:
                logger.warning(f"[{self.character_name}] coda-print failed: {e}")

    # ------------------------------------------------------------------
    # Sensor wiring — mirrors executive_node._start_sensors. Reads the
    # launcher-populated _sensor_metadata_by_name + per-character
    # `sensors:` list, spawns one daemon thread per sensor.
    # ------------------------------------------------------------------

    def _start_sensors(self) -> None:
        all_sensors = self.config.get('_sensor_metadata_by_name') or {}
        char_sensors = self.config.get('sensors', []) or []
        if not all_sensors or not char_sensors:
            return

        try:
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
            from sensor_runner import SensorRunner
            from utils.sensor_loader import parse_schedule
        except ImportError as e:
            logger.warning(f"[{self.character_name}] sensor module import failed: {e}")
            return

        if not hasattr(self, '_sensor_threads'):
            self._sensor_threads: List[threading.Thread] = []

        for sensor_cfg in char_sensors:
            s_name = sensor_cfg.get('name', '')
            if not s_name or s_name not in all_sensors:
                if s_name:
                    logger.warning(
                        f"[{self.character_name}] sensor '{s_name}' declared but not found in src/sensors/")
                continue
            sensor_meta = all_sensors[s_name]

            overrides: Dict[str, Any] = {}
            if 'schedule' in sensor_cfg:
                secs = parse_schedule(sensor_cfg['schedule'])
                if secs is not None:
                    overrides['schedule_seconds'] = secs
            if 'parameters' in sensor_cfg:
                overrides['parameters'] = sensor_cfg['parameters']
            if 'gate' in sensor_cfg:
                overrides['gate'] = sensor_cfg['gate']
            if 'disposition' in sensor_cfg:
                overrides['disposition'] = sensor_cfg['disposition']

            try:
                runner = SensorRunner(
                    sensor_name=s_name,
                    sensor_meta=sensor_meta,
                    character_name=self.character_name,
                    scenario_overrides=overrides,
                    resource_manager=self.resource_manager,
                    zenoh_session=self._zenoh_session,
                    available_tools={},
                    shutdown_event=self.shutdown_event,
                )
                t = threading.Thread(
                    target=runner.run,
                    name=f"sensor-{self.character_name}-{s_name}",
                    daemon=True,
                )
                t.start()
                self._sensor_threads.append(t)
                logger.info(
                    f"[{self.character_name}] started sensor {s_name} "
                    f"(interval={runner.schedule_seconds}s)")
            except Exception as e:
                logger.error(f"[{self.character_name}] failed to start sensor {s_name}: {e}")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._open_zenoh()
        # Sensors start AFTER zenoh — they publish to sense_data on the
        # same session.
        try:
            self._start_sensors()
        except Exception as e:
            logger.warning(f"[{self.character_name}] _start_sensors failed: {e}")

        def _watch_shutdown():
            self.shutdown_event.wait()
            self.shutdown_requested = True
            self._inbox.put({'__shutdown__': True})

        threading.Thread(target=_watch_shutdown, daemon=True).start()

        try:
            while not self.shutdown_requested:
                try:
                    msg = self._inbox.get(timeout=1.0)
                except queue.Empty:
                    continue

                if msg.get('__shutdown__'):
                    break

                kind = msg.get('kind', 'user')
                if kind == 'tick':
                    logger.debug(f"[{self.character_name}] <- tick")
                    try:
                        self._handle_tick()
                    except Exception as e:
                        logger.error(f"[{self.character_name}] tick handling crashed: {e}")
                        import traceback
                        traceback.print_exc()
                    continue

                # Default: user turn (back-compat with messages that
                # don't carry a 'kind' field).
                source = msg.get('source', 'User')
                text = msg.get('text', '')
                close = bool(msg.get('close', False))
                image_url = msg.get('image_url')
                img_tag = ' [+image]' if image_url else ''
                logger.info(f"[{self.character_name}] <- {source}: {text!r} (close={close}){img_tag}")

                try:
                    self._process_user_turn(source, text, close, image_url=image_url)
                except Exception as e:
                    logger.error(f"[{self.character_name}] turn handling crashed: {e}")
                    import traceback
                    traceback.print_exc()
        finally:
            # Drain any in-flight post-turn jobs so their memory writes
            # and discourse updates land on disk before we tear down.
            # wait=True blocks until the current job (if any) completes;
            # queued jobs also drain in order.
            try:
                self._post_turn_executor.shutdown(wait=True)
            except Exception as e:
                logger.warning(f"[{self.character_name}] executor shutdown failed: {e}")
            # Final flush after the executor drains so any work it did
            # in its last job (or that we did synchronously) is on disk.
            self._persist_to_disk()
            self._shutdown_zenoh()

    def _shutdown_zenoh(self) -> None:
        try:
            self._affect.close()
        except Exception as e:
            logger.warning(f"[{self.character_name}] affect close failed: {e}")
        try:
            self._canvas.close()
        except Exception as e:
            logger.warning(f"[{self.character_name}] canvas close failed: {e}")
        try:
            if self._sense_sub:
                self._sense_sub.undeclare()
        except Exception:
            pass
        try:
            if self._action_pub:
                self._action_pub.undeclare()
        except Exception:
            pass
        try:
            if self._zenoh_session:
                self._zenoh_session.close()
        except Exception:
            pass
        logger.info(f"[{self.character_name}] chat loop exited")
