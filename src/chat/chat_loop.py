"""
ChatLoop — lightweight per-character chat agent.

Mirrors ZenohExecutiveNode's user-facing wire protocol so existing clients
(cli.py, tests/chat_cli.py) work unchanged:

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
from types import SimpleNamespace
from typing import Any, Dict, List, Optional

import requests


# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from cot_profiles import is_reasoning_model, resolve_profile  # noqa: E402


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
_REACT_TOOLS = ('process_text', 'search', 'fetch_text', 'remember', 'inspect', 'respond')

# Shared Anthropic Sonnet model id. Used by tools and subagents that
# require a strong backend independent of Jill's per-scenario llm_config
# (e.g. the inspect codebase-query subagent). Bench-side judges
# (introspective_fidelity, memory_recall) keep their own JUDGE_MODEL
# literals for now — consolidation is a trivial follow-up.
CLAUDE_SONNET_MODEL = "claude-sonnet-4-6"

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
                 api_key: Optional[str] = None):
        self.server = (server or 'local').lower()
        self.model = model or ''
        self.base_url = (base_url or 'http://127.0.0.1:5000').rstrip('/')
        if self.base_url.endswith('/v1'):
            self.base_url = self.base_url[:-3]
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
        if is_reasoning is None:
            self.is_reasoning = is_reasoning_model(self.model)
        else:
            self.is_reasoning = bool(is_reasoning)

    def chat(self, messages: List[Dict[str, str]],
             max_tokens: int = 600,
             temperature: float = 0.7,
             top_p: float = 1.0,
             stops: Optional[List[str]] = None,
             is_json: bool = False,
             cot_profile: Optional[str] = None,
             enable_thinking: Optional[bool] = None) -> str:
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
        if stops:
            body['stop'] = stops

        # Skip grammar / chat_template_kwargs when going to a cloud endpoint
        # (signaled by api_key being set). Cloud providers reject those
        # fields; locally-served engines (llama-server, SGLangAPIServer)
        # accept them.
        is_cloud = self._api_key_value is not None
        if not is_cloud:
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
        resp.raise_for_status()
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
        self._restore_chat_state_from_notes()

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
        self._init_agent_concerns()
        self._init_user_concerns()
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

    # ------------------------------------------------------------------
    # LLM helper (used by character_evaluator and DiscourseTracker)
    # ------------------------------------------------------------------

    def _llm_generate(self, messages, bindings=None, max_tokens=400,
                      temperature=0.7, stops=None, is_json=False,
                      cot_profile: Optional[str] = None,
                      enable_thinking: Optional[bool] = None) -> SimpleNamespace:
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
            )
            if not text:
                return SimpleNamespace(success=False, text='', error='empty response')
            if is_json and isinstance(text, str):
                stripped = text.strip()
                if stripped.startswith('```'):
                    lines = [l for l in stripped.split('\n') if not l.strip().startswith('```')]
                    stripped = '\n'.join(lines).strip()
                try:
                    parsed = json.loads(stripped)
                    return SimpleNamespace(success=True, text=parsed, error=None)
                except Exception:
                    return SimpleNamespace(success=True, text=stripped, error=None)
            return SimpleNamespace(success=True, text=text, error=None)
        except Exception as e:
            logger.warning(f'[{self.character_name}] _llm_generate failed: {e}')
            return SimpleNamespace(success=False, text='', error=str(e))

    # Per-profile max_tokens floor. Caller-supplied max_tokens may be tuned
    # for non-reasoning models (e.g. character_evaluator hardcodes 256 for a
    # ~150-token JSON envelope); Qwen3.6-class reasoning models burn 1000-3000
    # tokens on free thinking before producing the answer, so floors are
    # generous to avoid finish_reason=length truncating mid-thought.
    _PROFILE_TOKEN_FLOOR = {
        'triage': 4096,
        'none': 4096,    # covers discourse extract + revise_belief callables
    }

    def _make_llm_callable(self, cot_profile: Optional[str],
                           enable_thinking: Optional[bool] = None):
        """Return an llm_generate-shaped callable bound to a CoT profile.

        Used to hand profile-tagged callables to consumers (character_evaluator,
        DiscourseTracker) that don't know about profiles or thinking themselves.
        `enable_thinking=False` suppresses <think> auto-prefix at the chat
        template level (Qwen3.6 and similar).
        """
        floor = self._PROFILE_TOKEN_FLOOR.get(cot_profile or '', 0)

        def _gen(messages, bindings=None, max_tokens=400, temperature=0.7,
                 stops=None, is_json=False):
            return self._llm_generate(messages, bindings=bindings,
                                      max_tokens=max(max_tokens, floor),
                                      temperature=temperature,
                                      stops=stops, is_json=is_json,
                                      cot_profile=cot_profile,
                                      enable_thinking=enable_thinking)
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
            if self._discourse_state or self._companion_state:
                logger.info(
                    f"[{self.character_name}] restored chat state: "
                    f"{len(self._discourse_state)} discourse, "
                    f"{len(self._companion_state)} companion entries"
                )
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
                  category: str = 'fact') -> Optional[str]:
        """Persist one memory string into the memories Collection. Returns
        the new note_id, or None on failure. `category` is one of
        _MEMORY_CATEGORIES; unknown values coerce to 'fact'.

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
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, text, "text", "chat-loop", entity or "",
                    "",
                    {
                        "kind": "memory",
                        "category": category,
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
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _remember failed: {e}")
            return None

    def _recall(self, query: str, k: int = 3, threshold: float = 0.5
                ) -> List[Tuple[str, str]]:
        """Semantic search over the memories Collection. Returns ranked
        (text, category) tuples, highest score first. Category is read
        from the source note's properties; pre-taxonomy memories without
        a category default to 'fact'. Empty list on miss / not yet
        indexed / any error."""
        if not self._memories_collection_id or not query:
            return []
        try:
            with self._faiss_lock:
                ok, results, err = self.resource_manager.search_collection(
                    self.character_name, self._memories_collection_id, query,
                    mode='semantic', limit=k, threshold=threshold)
            if not ok or not results:
                return []
            out: List[Tuple[str, str]] = []
            for r in results:
                if not isinstance(r, dict):
                    continue
                doc = r.get('document')
                if not isinstance(doc, str) or not doc.strip():
                    continue
                # Map chunk back to source note to read its category.
                # Chunks store source_note_id in metadata (per
                # _index_note_chunks). Missing note → fall back to 'fact'.
                # No lock needed for this lookup — get_resource is a dict
                # read, not a FAISS operation.
                cat = 'fact'
                meta = r.get('metadata') or {}
                note_id = meta.get('source_note_id')
                if note_id:
                    note = self.resource_manager.get_resource(note_id)
                    if note:
                        raw = (note.get('properties') or {}).get('category')
                        if isinstance(raw, str) and raw in _MEMORY_CATEGORIES:
                            cat = raw
                out.append((doc.strip(), cat))
            return out
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

    def _emit_impulse(self, concern_text: str, instruction: str) -> None:
        """Print a concern-fired impulse. Retained for possible future
        Phase B-style use; the autonomous-fire path uses preamble +
        coda directly rather than this helper."""
        msg = f"concern: {concern_text} — impulse: {instruction}"
        if not sys.stdout.isatty():
            logger.info(f"[{self.character_name}] {msg}")
            return
        try:
            line = (f"\n{self._STATUS_DIM}[{self.character_name}] "
                    f"{msg}{self._STATUS_RESET}\n")
            sys.stdout.write(line)
            sys.stdout.flush()
        except Exception as e:
            logger.warning(f"[{self.character_name}] _emit_impulse failed: {e}")

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
        "three stages.\n\n"
        "STAGE 1 — Frame check. Classify the latest exchange as one of:\n"
        "- `hypothetical`: 'imagine that…', 'suppose…', 'what if…'\n"
        "- `roleplay`: user asked {character} to take on a persona, character, or voice\n"
        "- `counterfactual`: discussion of an alternate world / past / scenario\n"
        "- `instructional`: user is teaching {character} how to behave, not stating facts\n"
        "- `none`: a real exchange where statements about {entity}, the world, or "
        "agreements between you carry their literal weight\n"
        "If the frame is anything other than `none`, return memories=[] AND "
        "concerns=[]. When in doubt, prefer the more conservative classification.\n\n"
        "STAGE 2 — Memories. If frame is `none`, extract stable specifics "
        "that should survive into FUTURE conversations.\n"
        "CAPTURE as memories:\n"
        "- Personal facts the user shared (names, places, relationships) → category=`fact`\n"
        "- Long-running project/work context → category=`fact`\n"
        "- Stable preferences expressed plainly (modifiers, not actions) → category=`preference`\n"
        "- Specific commitments or follow-ups agreed → category=`commitment`\n"
        "SKIP from memories:\n"
        "- Pleasantries, mood, conversational tone (companion handles these).\n"
        "- Anything already in the companion model verbatim.\n"
        "- One-off questions with no stable signal.\n\n"
        "STAGE 3 — Concerns. Distinct from memories. A concern is an "
        "ACTIONABLE INSTRUCTION {character} should be ready to advance — "
        "something she can reasonably take action on, not a fact or modifier. "
        "Capture concerns at THREE granularities:\n"
        "- `one_shot`: a specific request with a clear completion (\"look up "
        "X\", \"draft me an email about Y\"). Decays fast. If the request "
        "was already fulfilled in this exchange, leave `instruction` null "
        "so it logs without re-firing; if the request explicitly defers "
        "work for later, set `instruction` and it will fire on the next "
        "tick.\n"
        "- `durable`: an ongoing directive {entity} expects {character} to "
        "uphold over time (\"keep me posted on X\", \"help me think through "
        "my dissertation\"). FIRES per its cadence; provide cadence_hours "
        "and instruction.\n"
        "- `derived`: something {character} noticed worth tracking that the "
        "user did NOT explicitly request. Use sparingly. Provide instruction "
        "ONLY if recurring action is genuinely warranted; otherwise leave "
        "instruction null so it stays as background context without firing.\n"
        "SKIP from concerns:\n"
        "- Modifiers like \"be brief\" / \"don't use emoji\" — those go to "
        "memories as preferences, not concerns.\n"
        "- Items already covered by an existing concern in the system prompt.\n"
        "- Speculative inferences without textual support.\n\n"
        "For EVERY concern, emit these three additional fields:\n"
        "- `cadence_hours` (int|null): firing rhythm in hours. MUST be one of "
        "the allowed values {{1, 2, 4, 8, 12, 24, 168}}. Pick the rhythm of "
        "the underlying signal, not how often you'd nag the user. Examples:\n"
        "    hourly check (intraday market move, breaking news): 1 or 2\n"
        "    several-times-a-day (active project, ongoing task): 4 or 8\n"
        "    twice-daily / daily event (S&P close, daily roundup): 12 or 24\n"
        "    weekly check-in (project, hobby): 168\n"
        "    null is allowed for any category — means \"don't fire on a cadence.\"\n"
        "- `lifetime_days` (number|null): decay tau in DAYS — how long "
        "{entity}'s interest in this topic plausibly persists without "
        "re-engagement. Should be several times the cadence. null = "
        "immortal. Examples:\n"
        "    daily-fire concern (cadence_hours=24): 14-30\n"
        "    weekly-fire concern (cadence_hours=168): 60-180\n"
        "    annual-fire concern: null\n"
        "- `instruction` (string|null): what {character} does when this "
        "concern fires. A directive she could execute, in the imperative. "
        "Examples:\n"
        "    \"Search for today's S&P 500 close and summarize the move.\"\n"
        "    \"Check in on dissertation prep — ask if anything's blocking.\"\n"
        "    \"Pull recent papers on multi-agent coordination from arxiv.\"\n"
        "{narrowness_rule}\n\n"
        "**REQUIRED for category=durable**: cadence_hours MUST be one of the "
        "allowed values, lifetime_days MUST be a number, instruction MUST be "
        "a non-empty string. A durable concern without these fields cannot "
        "fire and is operationally useless. Do not skip them.\n"
        "**For one_shot and derived**: any of cadence_hours, instruction, and "
        "lifetime_days may be null. A null `instruction` means the concern "
        "is logged but never fires — the right choice when the request was "
        "already handled or no recurring action is warranted.\n\n"
        "Output ONLY this JSON shape — nothing else:\n"
        "  {{\"frame\": \"<hypothetical|roleplay|counterfactual|instructional|none>\",\n"
        "   \"memories\": [{{\"text\": \"...\", \"category\": \"fact|preference|commitment\"}}, ...],\n"
        "   \"concerns\": [{{\"text\": \"...\", \"category\": \"one_shot|durable|derived\",\n"
        "                  \"cadence_hours\": <int from {{1,2,4,8,12,24,168}}|null>,\n"
        "                  \"lifetime_days\": <number|null>,\n"
        "                  \"instruction\": \"<imperative directive>|null\"}}, ...]}}\n\n"
        "WORKED EXAMPLE. {entity} just said: \"Please keep an eye on S&P 500 "
        "closes — I want to hear about them every day.\"\n"
        "Correct output:\n"
        "{{\"frame\": \"none\", \"memories\": [], \"concerns\": [{{\n"
        "  \"text\": \"Track the S&P 500 closing price daily.\",\n"
        "  \"category\": \"durable\",\n"
        "  \"cadence_hours\": 24,\n"
        "  \"lifetime_days\": 14,\n"
        "  \"instruction\": \"Search for today's S&P 500 close and summarize the day's move.\"\n"
        "}}]}}\n"
        "If nothing qualifies (or frame≠none): "
        "{{\"frame\": \"<value>\", \"memories\": [], \"concerns\": []}}. No prose."
    )

    def _reflect_and_remember(self, source: str) -> Tuple[List[str], List[str]]:
        """Run a single reflection LLM call over the latest exchange; for any
        memories AND concerns returned, persist them. Returns (memories_written,
        concerns_written). Failure-tolerant: any error path returns ([], [])."""
        if not self._memories_collection_id:
            return ([], [])
        try:
            dialog = self._build_dialog(source, limit=4)
            if not dialog:
                return ([], [])
            convo = "\n".join(f"{t['source']}: {t['text']}" for t in dialog)
            companion = self._companion_state.get(source, '').strip()
            # Show the LLM the active concerns so it doesn't re-derive ones
            # we already track. Compact: text + category badge.
            existing_concerns = self._top_active_agent_concerns(n=10)
            sys_msg = self._REFLECT_SYS.format(
                character=self.character_name, entity=source,
                narrowness_rule=_CONCERN_INSTRUCTION_NARROWNESS_RULE)
            user_parts = []
            if companion:
                user_parts.append(
                    "## Existing companion model (do NOT re-extract from this; "
                    "use only to avoid duplicates)\n" + companion)
            if existing_concerns:
                lines = []
                for _nid, text, activation, props in existing_concerns:
                    cat = props.get('category', 'agent') or 'agent'
                    lines.append(f"- [{cat}] {text}")
                user_parts.append(
                    "## Existing active concerns (do NOT re-emit; emit only "
                    "NEW concerns this exchange surfaced)\n" + "\n".join(lines))
            user_parts.append("## Latest exchange\n" + convo)
            user_parts.append(
                "Return the JSON object now (keys: frame, memories, concerns). "
                "Both lists empty if frame≠none or nothing qualifies.")
            result = self._llm_generate(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': "\n\n".join(user_parts)}],
                max_tokens=4096, temperature=0.3, is_json=True,
                cot_profile='none')
            if not result.success:
                return ([], [])
            payload = result.text
            # Diagnostic: log the raw payload so we can see whether the LLM
            # is emitting cadence_days / lifetime_days / instruction or
            # silently dropping them (which forces category-default fallback
            # and an unfireable concern).
            try:
                _preview = json.dumps(payload) if not isinstance(payload, str) else payload
            except Exception:
                _preview = repr(payload)
            logger.info(f"[{self.character_name}] reflection raw: {_preview[:800]}")
            # Salvage: cloud LLMs may return text instead of parsed JSON.
            if isinstance(payload, str):
                stripped = payload.strip()
                if stripped.startswith(('{', '[')):
                    try:
                        payload = json.loads(stripped)
                    except Exception:
                        return ([], [])
                else:
                    return ([], [])

            frame, raw_memories, raw_concerns = self._parse_reflection_payload(payload)

            if frame != _REFLECT_FRAME_OK:
                logger.info(
                    f"[{self.character_name}] reflection suppressed "
                    f"(frame={frame!r}) — no memories/concerns written")
                return ([], [])

            mems_written: List[str] = []
            for text, category in raw_memories:
                if len(text) > 240:
                    text = text[:240].rstrip()
                if self._remember(text, entity=source, category=category):
                    mems_written.append(text)

            cons_written: List[str] = []
            for c in raw_concerns:
                text = c.get('text', '')
                if len(text) > 240:
                    text = text[:240].rstrip()
                category = c.get('category', 'durable')
                # Provisional adapter for the legacy reflection schema:
                # treat all reflection-emitted concerns as agent_concerns.
                # The reflection prompt rewrite (item 6) will replace this
                # with explicit user_concerns / agent_concerns channels.
                provenance = 'inferred' if category == 'derived' else 'asserted'
                rhythm_h = c.get('rhythm_hours')
                if rhythm_h is None:
                    rhythm_h = c.get('cadence_hours')
                if self._add_agent_concern(
                        text, entity=source, provenance=provenance,
                        seed=False, rhythm_hours=rhythm_h,
                        rhythm_source='external' if rhythm_h else 'default',
                        instruction=c.get('instruction')):
                    cons_written.append(text)

            if mems_written or cons_written:
                logger.info(
                    f"[{self.character_name}] reflection wrote "
                    f"{len(mems_written)} memory(s), "
                    f"{len(cons_written)} concern(s) from {source}")
            return (mems_written, cons_written)
        except Exception as e:
            logger.warning(f"[{self.character_name}] _reflect_and_remember failed: {e}")
            return ([], [])

    @staticmethod
    def _parse_reflection_payload(payload: Any
                                  ) -> Tuple[str, List[Tuple[str, str]], List[Dict[str, Any]]]:
        """Normalize reflection output to (frame, memories, concerns).
        memories: list of (text, category) tuples.
        concerns: list of dicts with keys text, category, cadence_hours,
                  lifetime_days, instruction (any may be missing/None).

        Accepts:
          - Full envelope: {"frame": "...", "memories": [...], "concerns": [...]}
            with each concern dict carrying text, category, cadence_hours,
            lifetime_days, instruction.
          - Older envelope where concerns lack the cadence/lifetime/
            instruction fields — fields default to None.
          - Legacy envelope where concerns carried cadence_days instead
            of cadence_hours — value is converted (×24) and snapped to
            the allowed bucket downstream.
          - Bare list of strings: assumed frame=none, treated as memories.
        Anything else returns ('unknown', [], []) — caller treats non-`none`
        frame as a suppression signal so this fails safe.
        """

        def _normalize_memories(raw: Any) -> List[Tuple[str, str]]:
            out: List[Tuple[str, str]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append((item.strip(), 'fact'))
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    c = str(item.get('category', 'fact') or 'fact').strip().lower()
                    if c not in _MEMORY_CATEGORIES:
                        c = 'fact'
                    if t:
                        out.append((t, c))
            return out

        def _normalize_concerns(raw: Any) -> List[Dict[str, Any]]:
            out: List[Dict[str, Any]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append({'text': item.strip(), 'category': 'durable',
                                'cadence_hours': None, 'lifetime_days': None,
                                'instruction': None})
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    if not t:
                        continue
                    c = str(item.get('category', 'durable') or 'durable').strip().lower()
                    if c not in _CONCERN_CATEGORIES:
                        c = 'durable'
                    # Accept new cadence_hours or legacy cadence_days (×24);
                    # snapping to the allowed bucket happens in _add_concern.
                    cad_h = item.get('cadence_hours')
                    if cad_h is None and item.get('cadence_days') is not None:
                        try:
                            cad_h = float(item.get('cadence_days')) * 24.0
                        except (TypeError, ValueError):
                            cad_h = None
                    life = item.get('lifetime_days')
                    instr = item.get('instruction')
                    out.append({
                        'text': t, 'category': c,
                        'cadence_hours': cad_h if cad_h is not None else None,
                        'lifetime_days': life if life is not None else None,
                        'instruction': str(instr).strip() if instr else None,
                    })
            return out

        # Old shape — bare list. Assume frame=none, all memories.
        if isinstance(payload, list):
            return (_REFLECT_FRAME_OK, _normalize_memories(payload), [])

        # New envelope.
        if isinstance(payload, dict):
            frame = str(payload.get('frame', '') or '').strip().lower() or 'unknown'
            mems = _normalize_memories(payload.get('memories', []))
            cons = _normalize_concerns(payload.get('concerns', []))
            return (frame, mems, cons)

        return ('unknown', [], [])

    # ------------------------------------------------------------------
    # Zenoh wiring
    # ------------------------------------------------------------------

    def _open_zenoh(self) -> None:
        import zenoh
        from utils.zenoh_utils import make_localhost_config

        self._zenoh_session = zenoh.open(make_localhost_config())
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
        # `/remember <query>` in the CLI to test/inspect the active-recall
        # subagent without spending a full Jill turn.
        self._remember_q = self._zenoh_session.declare_queryable(
            f"cognitive/{self.character_name}/remember",
            self._handle_remember_query,
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
        if isinstance(content, str):
            try:
                inner = json.loads(content)
                source = inner.get('source', source)
                text = inner.get('text', '')
                close = bool(inner.get('close', False))
            except Exception:
                text = content
        elif isinstance(content, dict):
            source = content.get('source', source)
            text = content.get('text', '')
            close = bool(content.get('close', False))

        # Sensor dispatch: source is e.g. 'sensor:tick'. Recognized sensors
        # become typed events on the inbox; unknown sensors fall through to
        # the empty-text drop below.
        if isinstance(source, str) and source == 'sensor:tick':
            self._inbox.put({'kind': 'tick'})
            return

        if not text and not close:
            return

        self._inbox.put({'kind': 'user', 'source': source, 'text': text, 'close': close})

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
        """Extract a single JSON action object from LLM output. Tolerates
        ```json fences and surrounding prose. Returns None if no balanced
        JSON object can be found."""
        if not raw or not isinstance(raw, str):
            return None
        text = raw.strip()
        text = re.sub(r'^```(?:json)?\s*', '', text)
        text = re.sub(r'\s*```\s*$', '', text)
        text = text.strip()
        if not text:
            return None
        start = text.find('{')
        if start < 0:
            return None
        depth = 0
        in_str = False
        esc = False
        end = -1
        for i in range(start, len(text)):
            c = text[i]
            if in_str:
                if esc:
                    esc = False
                elif c == '\\':
                    esc = True
                elif c == '"':
                    in_str = False
                continue
            if c == '"':
                in_str = True
            elif c == '{':
                depth += 1
            elif c == '}':
                depth -= 1
                if depth == 0:
                    end = i
                    break
        if end < 0:
            return None
        try:
            obj = json.loads(text[start:end + 1])
        except Exception:
            return None
        if not isinstance(obj, dict):
            return None
        return obj

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
                max_tokens=2048, temperature=0.4, cot_profile='none',
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
        Jill's ReAct loop — used by `/remember` in the CLI for test/inspect."""
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
            tracker.llm_generate = self._make_llm_callable('none')
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
            tracker.llm_generate = self._make_llm_callable('none')
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
                             recall: Optional[List[Tuple[str, str]]] = None,
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
        if recall:
            # Episodic specifics retrieved from prior conversations. Distinct
            # from the rolling Companion summary: these are durable items
            # that should not decay with style. Rendered grouped by
            # category so the model can treat each group on its own terms.
            grouped: Dict[str, List[str]] = {c: [] for c in _MEMORY_CATEGORIES}
            for text, cat in recall:
                if cat not in grouped:
                    cat = 'fact'
                grouped[cat].append(text)

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
                    "specifics about " + source + ".\n\n"
                    + "\n".join(body_lines)
                )
        disc = self._discourse_state.get(source, '').strip()
        if disc:
            parts.append("## Outstanding discourse objects (from periodic reflection)\n" + disc)
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

    def _build_react_system_prompt(self, source: str, orientation: str,
                                   now_str: str,
                                   recall: Optional[List[Tuple[str, str]]] = None,
                                   agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                                   user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None) -> str:
        base = self._build_system_prompt(
            source, orientation, recall=recall,
            agent_concerns=agent_concerns, user_concerns=user_concerns)
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
            "You do NOT know: current weather, recent news, current prices, "
            "or anything requiring fresh information. For time-sensitive or "
            "fact-specific questions, your first action is `search`.\n"
            "\n"
            "Tools (each emission picks ONE):\n"
            "1. `{\"thought\": \"<one terse sentence>\", \"tool\": \"process_text\", \"source\": <string|$stepN>, \"instruction\": <string>}` — "
            "LLM pass over text in context. Use to formulate queries, render results in your voice, extract info.\n"
            "2. `{\"thought\": \"<one terse sentence>\", \"tool\": \"search\", \"query\": <string|$stepN>}` — web search (digested synthesis + sources).\n"
            "3. `{\"thought\": \"<one terse sentence>\", \"tool\": \"fetch_text\", \"url\": <string|$stepN>}` — full text from a single URL "
            "(or local file path). Use when a search hit looks promising and the snippet isn't enough; "
            "always pass the result through process_text before responding.\n"
            "4. `{\"thought\": \"<one terse sentence>\", \"tool\": \"remember\", \"query\": <string>}` — active recall over your "
            "own memory: full reasoning trace, conversation history, current companion model, current discourse "
            "state. Use when the user asks about prior turns, your earlier reasoning, or persistent state that "
            "may not be in your current prompt. The query is opaque to you — a subagent reads the files and "
            "returns a synthesized answer in `$stepN`.\n"
            "5. `{\"thought\": \"<one terse sentence>\", \"tool\": \"inspect\", \"query\": <string>}` — query your own "
            "codebase. A separate subagent (geofenced read-only to `src/`, list/read/grep primitives, "
            "Sonnet-backed) navigates the source and returns a synthesized answer with file:line citations. "
            "Use when the user asks how you work, where something is implemented, what a module does, or "
            "to verify a claim about your own code. The query is opaque to you — phrase it as a natural-language "
            "question (e.g. \"where is the ReAct dispatch defined?\", \"what tools does the chat loop wire up?\").\n"
            "6. `{\"thought\": \"<one terse sentence>\", \"tool\": \"respond\", \"text\": <string|$stepN>}` — final reply, exits loop. "
            "Must be in your voice; pass search/fetch results through process_text first or write the reply yourself.\n"
            "\n"
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
            "Worked example. User: 'what's the weather in Berkeley tomorrow?'\n"
            "  Iter 1: `{\"thought\": \"Need fresh weather data — search first.\", \"tool\": \"search\", \"query\": \"Berkeley CA weather forecast tomorrow\"}` → $step1\n"
            "  Iter 2: `{\"thought\": \"Search synthesis is decent; render in my voice with source.\", \"tool\": \"process_text\", \"source\": \"$step1\", \"instruction\": \"answer the user in your voice in 1-2 sentences, citing the source domain\"}` → $step2\n"
            "  Iter 3: `{\"thought\": \"Processed answer is ready — send it.\", \"tool\": \"respond\", \"text\": \"$step2\"}` → loop exits.\n"
            "\n"
            "Output ONLY one JSON object. No prose, no apology, no explanation."
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
                        recall: Optional[List[Tuple[str, str]]] = None,
                        agent_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None,
                        user_concerns: Optional[List[Tuple[str, str, float, Dict[str, Any]]]] = None
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

        for i in range(REACT_MAX_ITERS):
            pre_log_len = len(log)
            usr_msg = user_prefix_str + log_appendage_str + trailer
            prompt = [
                {'role': 'system', 'content': system_prompt_str},
                {'role': 'user', 'content': usr_msg},
            ]
            self._emit_status('thinking…')
            try:
                raw = self.backend.chat(prompt, max_tokens=2048, temperature=0.7,
                                        cot_profile='none')
            except Exception as e:
                logger.error(f"[{self.character_name}] ReAct iter {i+1} LLM failed: {e}")
                iters.append({'system': system_prompt_str, 'user': usr_msg,
                              'raw': f'(LLM call failed: {e})', 'appended': []})
                reply = self._react_fallback_synthesis(log, str(e))
                self._clear_status()
                return reply, log, iters, 'llm_error'

            iters.append({'system': system_prompt_str, 'user': usr_msg, 'raw': raw or '',
                          'appended': []})

            action = self._parse_react_action(raw)
            if action is None:
                logger.warning(f"[{self.character_name}] ReAct iter {i+1}: unparseable: {raw[:160]!r}")
                self._emit_status('parse failed, retrying…')
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
                return text, log, iters, 'respond'

            self._emit_status(f'using {tool}…')
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
            elif tool == 'remember':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_remember(q)
            elif tool == 'inspect':
                q = self._resolve_react_value(action.get('query', ''), log)
                obs = self._run_inspect(q)
            else:
                obs = (f"ERROR: unknown tool {tool!r}; available: "
                       "process_text, search, fetch_text, remember, "
                       "inspect, respond")

            _append_log(binding, obs)
            iters[-1]['appended'] = log[pre_log_len:]
            logger.info(f"[{self.character_name}] ReAct iter {i+1}: {tool} → {binding} ({len(obs)} chars)")

        logger.warning(f"[{self.character_name}] ReAct hit max iters ({REACT_MAX_ITERS})")
        reply = self._react_fallback_synthesis(log)
        self._clear_status()
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

    def _run_remember(self, query: str) -> str:
        """Backend for the ReAct `remember` tool. Delegates to the
        active-recall subagent (chat.remember.remember), which runs its
        own thin ReAct loop over the memory dir. Returns an OK:/EMPTY:/
        ERROR: prefixed observation per the ReAct tool-observation
        convention. Subagent's full per-call trace lands under
        subagent_traces/ for debugging."""
        if not query or not str(query).strip():
            return "EMPTY: remember query was empty"
        try:
            from chat.remember import remember as _remember
            answer = _remember(
                query=str(query),
                memory_dir=self._memory_dir(),
                llm_backend=self.backend,
                trace_dir=self._subagent_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] remember subagent raised: {e}")
            return f"ERROR: remember subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: remember subagent produced no answer'
        return 'OK: ' + text

    def _inspect_traces_dir(self) -> 'Path':
        """Where the inspect (codebase-query) subagent writes its per-call
        traces. Sibling of subagent_traces/ — kept separate so per-tool
        debugging stays clean. Layout:
        scenarios/<world>/<agent>/inspect_traces/."""
        return self._memory_dir().parent / 'inspect_traces'

    def _get_inspect_backend(self):
        """Lazy-load the Sonnet backend used by the inspect subagent.
        Cached per-instance after first use. The inspect subagent
        deliberately uses Sonnet rather than Jill's configured backend —
        codebase reasoning is harder than memory recall and benefits
        from a strong model. Returns None when CLAUDE_API_KEY is absent;
        the caller surfaces ERROR-tagged observation."""
        if hasattr(self, '_inspect_backend_cached'):
            return self._inspect_backend_cached
        if not os.environ.get('CLAUDE_API_KEY'):
            logger.warning(
                f"[{self.character_name}] inspect tool unavailable: "
                "CLAUDE_API_KEY not set")
            self._inspect_backend_cached = None
            return None
        try:
            self._inspect_backend_cached = _ChatBackend(
                server='anthropic',
                model=CLAUDE_SONNET_MODEL,
                base_url='https://api.anthropic.com',
                is_reasoning=False,
                api_key='CLAUDE_API_KEY',
            )
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] inspect backend init failed: {e}")
            self._inspect_backend_cached = None
        return self._inspect_backend_cached

    def _run_inspect(self, query: str) -> str:
        """Backend for the ReAct `inspect` tool. Delegates to the
        codebase-inspection subagent (chat.inspect.inspect), which runs
        its own thin ReAct loop over src/ with read-only primitives
        (list, read, grep via ripgrep). Returns an OK:/EMPTY:/ERROR:
        prefixed observation per the ReAct tool-observation convention.
        Sonnet-backed; per-call traces under inspect_traces/."""
        if not query or not str(query).strip():
            return "EMPTY: inspect query was empty"
        backend = self._get_inspect_backend()
        if backend is None:
            return ("ERROR: inspect subagent unavailable "
                    "(CLAUDE_API_KEY not set or backend init failed — "
                    "see warning log)")
        try:
            from chat.inspect import inspect as _inspect
            from pathlib import Path as _Path
            answer = _inspect(
                query=str(query),
                repo_root=_Path(_SRC_DIR),
                llm_backend=backend,
                trace_dir=self._inspect_traces_dir(),
            )
        except Exception as e:
            logger.warning(f"[{self.character_name}] inspect subagent raised: {e}")
            return f"ERROR: inspect subagent raised: {e}"
        text = str(answer or '').strip()
        if not text:
            return 'EMPTY: inspect subagent produced no answer'
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
                           recall: Optional[List[Tuple[str, str]]] = None) -> None:
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
            lines: List[str] = [
                '',
                '=' * 80,
                f'[{ts}] source={source} iters={n_iters} exit={exit_reason}',
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
                                  recall: Optional[List[Tuple[str, str]]] = None,
                                  agent_concerns: Optional[List[Any]] = None,
                                  user_concerns: Optional[List[Any]] = None,
                                  autonomous: bool = False) -> None:
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
                max_tokens=1024, temperature=0.7, cot_profile='none')
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
                max_tokens=400, temperature=0.2, cot_profile='none')
        except Exception as e:
            logger.warning(f"[{self.character_name}] successor synth LLM failed: {e}")
            return None
        # Parse: tolerate code-fenced JSON.
        s = (raw or '').strip()
        if s.startswith('```'):
            s = s.strip('`').strip()
            if s.lower().startswith('json'):
                s = s[4:].strip()
        try:
            data = json.loads(s)
        except json.JSONDecodeError as e:
            logger.warning(
                f"[{self.character_name}] successor JSON parse failed: {e}; "
                f"raw={raw[:200]!r}")
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
            if close:
                self.store.close_dialog(source)
            # Autosave. Memories written above land on disk here; without
            # this, a SIGINT mid-job loses the turn's memory updates.
            self._persist_to_disk()
        except Exception as e:
            logger.warning(f"[{self.character_name}] post-turn work failed: {e}")

    def _process_user_turn(self, source: str, text: str, close: bool,
                           autonomous: bool = False,
                           autonomous_concern_id: Optional[str] = None) -> None:
        """Drive one turn through the ReAct loop. The autonomous path
        (autonomous=True) reuses the same prompt construction so Jill's
        voice stays consistent and traces share format. Divergences are
        narrow: skip record_incoming (no fake user turn), suppress recall
        refresh (read-only — autonomous engagement is NOT user engagement),
        skip post-turn reflection/discourse (no user side to model from),
        and bump last_acted_at on the fired concern after success.
        """
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
                agent_concerns=agent_concerns, user_concerns=user_concerns)
        except Exception as e:
            logger.error(f"[{self.character_name}] ReAct loop crashed: {e}")
            import traceback
            traceback.print_exc()
            reply = f"[{self.character_name}] I had trouble generating a reply: {e}"

        reply = (reply or '').strip()
        if not reply:
            reply = "(no reply)"

        act_type = 'auto_say' if autonomous else 'say'
        self.store.record_outgoing(source, reply, act_type=act_type, close=close)
        self._append_conversation_entry(
            'out', source, reply, meta=f'act={act_type} close={close}')
        self._publish_say(reply)
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
                source, text, log, iters, reply, exit_reason, recall=recall)
            # Awareness feed: persist this turn's trace into
            # reasoning_history so the next turn's prompt prefix can
            # surface it. Same iters list — different store, different
            # render. Both autonomous and user-driven turns write here:
            # all reasoning is Jill's.
            self._write_reasoning_history(
                source, text, iters, reply, exit_reason,
                orientation=orientation, recall=recall,
                agent_concerns=agent_concerns, user_concerns=user_concerns,
                autonomous=autonomous)

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
            self._post_turn_work(source, close)
        else:
            try:
                self._post_turn_executor.submit(self._post_turn_work, source, close)
            except RuntimeError:
                # Executor already shut down (process tearing down).
                # Fall back to synchronous so nothing is silently dropped.
                self._post_turn_work(source, close)

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
            # the autonomous reply lands. Same dim-text style as Phase B
            # impulses so the visual signature is consistent.
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
            try:
                # source=self.character_name marks this turn as agent-
                # originated; `_process_user_turn` skips record_incoming
                # on autonomous=True so no fake user message is logged.
                # The conversation history surfaced into the prompt is
                # still the User dialogue (see _build_react_user_prefix).
                self._process_user_turn(
                    source=self.character_name, text=instruction, close=False,
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
                logger.info(f"[{self.character_name}] <- {source}: {text!r} (close={close})")

                try:
                    self._process_user_turn(source, text, close)
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
