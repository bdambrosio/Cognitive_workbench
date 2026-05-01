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
# synthesis. Typical turns are 1-4 iters; cap exists to bound runaway loops.
REACT_MAX_ITERS = 8

# Tools the model can emit. Validated structurally in _parse_react_action;
# the dispatcher in _run_react_loop knows how to run each.
_REACT_TOOLS = ('process_text', 'search', 'fetch_text', 'respond')

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

# Per-character collection holding actionable directives to keep on hand —
# distinct from memories (which are stable specifics to recall). A concern
# is something Jill can reasonably expect to act on as an instruction;
# preferences (modifiers like "be brief") stay in memories.
_CONCERNS_COLLECTION_NAME = "concerns"
_CONCERN_CATEGORIES = ('one_shot', 'durable', 'derived')
_CONCERN_STATUSES = ('active', 'satisfied', 'abandoned')

# Two independent per-concern parameters drive the lifecycle:
#
#   cadence_hours — firing rhythm in hours, drawn from a fixed allowlist
#                   ({1,2,4,8,12,24,168}). When (now - last_engaged_at) >=
#                   cadence_hours, the concern wants attention and fires
#                   once (until engagement resets the cycle). null →
#                   doesn't fire autonomously.
#   lifetime_days — decay tau. Effective weight = exp(-Δdays / lifetime).
#                   Below _CONCERN_SATISFIED_THRESHOLD → status flips to
#                   satisfied (lazy, on read). null → immortal.
#
# Cadence and lifetime are NOT the same. A daily concern (S&P 500) wants
# to fire every day but may legitimately persist for weeks if the user
# takes a vacation. Lifetime should generally be several times the cadence
# (note unit mismatch: cadence is hours, lifetime is days).
#
# Defaults below apply at creation when reflection doesn't emit explicit
# values — and at read time as fallback for legacy concerns missing the
# field. Seed concerns get cadence=null + lifetime=null (immortal, no fire).
# cadence_hours allowlist: 1h, 2h, 4h, 8h, 12h, 1d, 1wk.
_CONCERN_CADENCE_HOURS_ALLOWED = (1, 2, 4, 8, 12, 24, 168)
_CONCERN_DEFAULT_CADENCE_HOURS = {
    'one_shot': None,    # no autonomous fire
    'derived':  None,    # no autonomous fire
    'durable':  24,      # daily fire by default
}
_CONCERN_DEFAULT_LIFETIME_DAYS = {
    'one_shot': 0.5,     # ~12 hours to satisfied
    'derived':  2.0,     # ~5 days to satisfied
    'durable':  120.0,   # ~10 months without engagement
}
_CONCERN_SATISFIED_THRESHOLD = 0.1
# Lifetime sanity bounds (still in days).
_CONCERN_LIFETIME_MIN_DAYS, _CONCERN_LIFETIME_MAX_DAYS = 0.1, 3650.0
# Top-N active concerns surfaced into the system prompt always-on (in
# addition to anything semantic-recall fishes out). Bounded so the prompt
# can't blow up as the corpus grows.
_CONCERN_ALWAYS_ON_BUDGET = 5
# Similarity threshold for recurrence detection. A candidate concern
# whose top match in the concerns collection exceeds this value is
# treated as the same concern: we refresh + possibly promote/revive
# the existing note rather than create a near-duplicate. Tuned high
# (vs the 0.5 used for surfacing recall) because a false merge silently
# loses specificity, while a missed merge just creates a sibling that
# can be merged manually.
_CONCERN_RECURRENCE_THRESHOLD = 0.8
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
                body['system'] = "\n\n".join(sys_parts)
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
                logger.info(
                    "<llm-raw> route=anthropic stop=%s text=%s",
                    data.get('stop_reason'),
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

        # ---- Concerns (actionable directives, separate from memories) ----
        # A 'concerns' Collection holds standalone instructions Jill should
        # be ready to advance — distinct from preferences (modifiers) and
        # facts (specifics). Lifecycle: active → satisfied via lazy decay,
        # → abandoned only via explicit user revocation. Seeded from the
        # YAML `concerns:` block on first launch; reflection adds derived
        # concerns at turn end; recall surfaces them in the system prompt.
        self._concerns_collection_id: Optional[str] = None
        self._init_concerns()
        self._seed_concerns_from_config(character_config)

        # ---- Reasoning history (awareness feed) ----
        # Per-turn ReAct traces persisted here. Last N render in the
        # user-message prefix as Jill's awareness of her own prior thinking.
        self._reasoning_history_collection_id: Optional[str] = None
        self._init_reasoning_history()

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

    def _init_concerns(self) -> None:
        """Get-or-create the concerns Collection, mark it persistent,
        ensure semantic index exists. Idempotent across restarts."""
        try:
            cid = self.resource_manager.named_collections.get(_CONCERNS_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _CONCERNS_COLLECTION_NAME,
                    {"kind": "concerns"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create concerns collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self.resource_manager.index_collection(self.character_name, cid, index_type='semantic')
            self._concerns_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_concerns failed: {e}")

    def _init_reasoning_history(self) -> None:
        """Get-or-create the reasoning_history Collection. Per-turn ReAct
        traces land here as Notes; the last N surface in the user-message
        prefix as Jill's awareness of her own prior thinking. No semantic
        index for v1 — read pattern is recency-only."""
        try:
            cid = self.resource_manager.named_collections.get(_REASONING_HISTORY_COLLECTION_NAME)
            if not cid:
                success, cid, err, _ = self.resource_manager.create_collection(
                    self.character_name, [], "list", "chat-loop", "",
                    _REASONING_HISTORY_COLLECTION_NAME,
                    {"kind": "reasoning_history"},
                )
                if not success or not cid:
                    logger.warning(f"[{self.character_name}] create reasoning_history collection failed: {err}")
                    return
            self.resource_manager.mark_persistent(cid, self.character_name)
            self._reasoning_history_collection_id = cid
        except Exception as e:
            logger.warning(f"[{self.character_name}] _init_reasoning_history failed: {e}")

    @staticmethod
    def _seed_concern_name(idx: int) -> str:
        return f"chat:concern:seed:{idx}"

    def _seed_concerns_from_config(self, character_config: dict) -> None:
        """Instantiate concerns listed under YAML key `concerns:` if they
        don't already exist. Each seed gets a stable named-note slot so
        re-init is idempotent. Seed concerns carry seed=True and are
        immune to decay-induced satisfaction (they're architectural
        baseline, not ephemeral user-derived content). YAML may specify
        cadence_hours, lifetime_days, instruction per seed; otherwise
        seed concerns default to immortal + no fire."""
        seeds = character_config.get('concerns') or []
        if not isinstance(seeds, list) or not self._concerns_collection_id:
            return
        for idx, seed in enumerate(seeds):
            if not isinstance(seed, dict):
                continue
            text = str(seed.get('text', '') or '').strip()
            if not text:
                continue
            category = str(seed.get('category', 'durable') or 'durable').lower()
            if category not in _CONCERN_CATEGORIES:
                category = 'durable'
            entity = str(seed.get('entity', 'User') or 'User')
            name = self._seed_concern_name(idx)
            if name in self.resource_manager.named_notes:
                continue  # Already seeded; preserve any edits
            # Accept either cadence_hours (new) or legacy cadence_days
            # (×24) from seed YAML — same fallback logic _resolve_cadence_hours
            # uses at read time.
            cad_h = seed.get('cadence_hours')
            if cad_h is None and seed.get('cadence_days') is not None:
                try:
                    cad_h = float(seed.get('cadence_days')) * 24.0
                except (TypeError, ValueError):
                    cad_h = None
            self._add_concern(text, category=category, entity=entity,
                              provenance='asserted', seed=True, name=name,
                              cadence_hours=cad_h,
                              lifetime_days=seed.get('lifetime_days'),
                              instruction=seed.get('instruction'))

    def _find_similar_concern(self, text: str,
                              threshold: float = _CONCERN_RECURRENCE_THRESHOLD
                              ) -> Optional[str]:
        """Find the top semantically-similar existing concern whose
        similarity meets `threshold`. Returns the source note_id, or None
        if no match. Excludes abandoned concerns (the user explicitly
        revoked them — recurrence shouldn't auto-revive). Active and
        satisfied are both eligible; the caller decides what to do."""
        if not self._concerns_collection_id or not text:
            return None
        try:
            with self._faiss_lock:
                ok, results, _ = self.resource_manager.search_collection(
                    self.character_name, self._concerns_collection_id, text,
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

    def _promote_existing_concern(self, note_id: str) -> str:
        """Apply recurrence-promotion rules in place:
          - last_engaged_at always refreshed (the recurrence IS user engagement —
            drives decay)
          - last_acted_at refreshed too — user-driven recurrence also resets
            the cadence clock so we don't autonomously re-fire something the
            user just engaged with
          - status: satisfied → active (revival)
          - category: one_shot → durable (intensity promotion)
          - durable / derived: category unchanged (no de-promotion)
        Returns the same note_id so callers have a uniform shape."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return note_id
        props = note.setdefault('properties', {})
        now_iso = datetime.now(timezone.utc).isoformat()
        props['last_engaged_at'] = now_iso
        props['last_acted_at'] = now_iso
        if props.get('status') == 'satisfied':
            props['status'] = 'active'
        if props.get('category') == 'one_shot':
            props['category'] = 'durable'
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

    @staticmethod
    def _snap_cadence_hours(value: Any) -> Optional[int]:
        """Coerce a cadence value to the nearest allowed hours bucket
        ({1,2,4,8,12,24,168}). null/None/non-numeric → None (no fire)."""
        if value is None:
            return None
        try:
            v = float(value)
        except (TypeError, ValueError):
            return None
        if v <= 0:
            return None
        return min(_CONCERN_CADENCE_HOURS_ALLOWED, key=lambda b: abs(b - v))

    @classmethod
    def _resolve_cadence_hours(cls, properties: Dict[str, Any]) -> Optional[int]:
        """Read cadence_hours from a concern note, falling back to (a)
        legacy cadence_days converted to hours and snapped to allowed
        bucket, then (b) category default. Seed concerns default to null
        (no fire) regardless of category."""
        if 'cadence_hours' in properties and properties.get('cadence_hours') is not None:
            return cls._snap_cadence_hours(properties.get('cadence_hours'))
        # Legacy migration: pre-rename notes carry cadence_days (float).
        if 'cadence_days' in properties and properties.get('cadence_days') is not None:
            try:
                hours = float(properties['cadence_days']) * 24.0
                return cls._snap_cadence_hours(hours)
            except (TypeError, ValueError):
                pass
        if properties.get('seed'):
            return None
        return _CONCERN_DEFAULT_CADENCE_HOURS.get(
            properties.get('category', 'durable'))

    @staticmethod
    def _resolve_lifetime_days(properties: Dict[str, Any]) -> Optional[float]:
        """Read lifetime_days from a concern note, falling back to category
        default for legacy notes. Seed concerns default to null (immortal)."""
        if 'lifetime_days' in properties:
            raw = properties.get('lifetime_days')
            if raw is None:
                return None
            try:
                return float(raw)
            except (TypeError, ValueError):
                pass
        if properties.get('seed'):
            return None
        return _CONCERN_DEFAULT_LIFETIME_DAYS.get(
            properties.get('category', 'durable'), 120.0)

    def _add_concern(self, text: str, category: str = 'durable',
                     entity: str = 'User', provenance: str = 'asserted',
                     seed: bool = False, name: str = '',
                     cadence_hours: Optional[int] = None,
                     lifetime_days: Optional[float] = None,
                     instruction: Optional[str] = None) -> Optional[str]:
        """Create a new concern note and add it to the concerns Collection,
        OR — if a near-twin already exists — refresh and promote/revive
        the existing note instead. Returns the resulting note_id (newly
        created or pre-existing), or None on failure. Held under
        _faiss_lock for the FAISS-touching span; pre-add similarity check
        also serializes through the same lock.

        cadence_hours and lifetime_days are independent: cadence drives
        firing rhythm (in hours, snapped to {1,2,4,8,12,24,168}), lifetime
        drives decay-to-satisfied (in days). Either or both may be None.
        instruction (the action to take when this concern fires) is
        required for fireable concerns; null instruction means the
        concern never fires regardless of cadence."""
        if not self._concerns_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        if category not in _CONCERN_CATEGORIES:
            category = 'durable'
        if provenance not in ('asserted', 'inferred'):
            provenance = 'inferred' if category == 'derived' else 'asserted'

        # Apply category defaults if caller didn't specify; seed concerns
        # default to null on both axes (immortal, no autonomous fire).
        if cadence_hours is None and not seed:
            cadence_hours = _CONCERN_DEFAULT_CADENCE_HOURS.get(category)
        if lifetime_days is None and not seed:
            lifetime_days = _CONCERN_DEFAULT_LIFETIME_DAYS.get(category)
        # Snap cadence to allowed bucket; clamp lifetime into sane range.
        cadence_hours = self._snap_cadence_hours(cadence_hours)
        lifetime_days = self._clamp_optional(lifetime_days,
                                             _CONCERN_LIFETIME_MIN_DAYS,
                                             _CONCERN_LIFETIME_MAX_DAYS)
        instruction = (str(instruction).strip() if instruction else '') or None

        # Recurrence check. Skipped for seed concerns (those go in
        # deterministic named-note slots and shouldn't get folded into
        # arbitrary user-derived twins).
        if not seed:
            existing = self._find_similar_concern(text)
            if existing:
                return self._promote_existing_concern(existing)

        now_iso = datetime.now(timezone.utc).isoformat()
        try:
            with self._faiss_lock:
                success, note_id, err, _ = self.resource_manager.create_note(
                    self.character_name, text, "text", "chat-loop", entity or "",
                    name or "",
                    {
                        "kind": "concern",
                        "category": category,
                        "status": "active",
                        "entity": entity,
                        "provenance": provenance,
                        "seed": bool(seed),
                        "last_engaged_at": now_iso,
                        "last_acted_at": None,
                        "cadence_hours": cadence_hours,
                        "lifetime_days": lifetime_days,
                        "instruction": instruction,
                    },
                )
                if not success or not note_id:
                    logger.warning(f"[{self.character_name}] concern create failed: {err}")
                    return None
                self.resource_manager.mark_persistent(note_id, self.character_name)
                ok, _, add_err = self.resource_manager.add_to_collection(
                    self._concerns_collection_id, note_id, self.character_name)
                if not ok:
                    logger.warning(f"[{self.character_name}] concern add_to_collection failed: {add_err}")
                return note_id
        except Exception as e:
            logger.warning(f"[{self.character_name}] _add_concern failed: {e}")
            return None

    @classmethod
    def _concern_decayed_weight(cls, properties: Dict[str, Any]) -> float:
        """Effective weight = exp(-Δdays / lifetime_days). Δ is now −
        last_engaged_at. lifetime_days=None means immortal (returns 1.0).
        Falls back to category default for legacy notes lacking the field.
        Returns 0.0 for malformed timestamps."""
        import math
        try:
            lifetime = cls._resolve_lifetime_days(properties)
            if lifetime is None:
                return 1.0   # immortal (seed concerns, etc.)
            if lifetime <= 0:
                return 1.0
            last_iso = properties.get('last_engaged_at') or properties.get('created_at')
            if not last_iso:
                return 0.0
            last = datetime.fromisoformat(str(last_iso))
            if last.tzinfo is None:
                last = last.replace(tzinfo=timezone.utc)
            delta_days = (datetime.now(timezone.utc) - last).total_seconds() / 86400.0
            return math.exp(-max(0.0, delta_days) / lifetime)
        except Exception:
            return 0.0

    def _maybe_transition_to_satisfied(self, note: Dict[str, Any], weight: float) -> bool:
        """Lazy transition: if weight is below threshold and this isn't a
        seed concern, mark active → satisfied in place. Returns True if
        the transition fired (caller should skip surfacing this concern)."""
        props = note.get('properties') or {}
        if props.get('status') != 'active':
            return props.get('status') != 'active'
        if props.get('seed'):
            return False
        if weight >= _CONCERN_SATISFIED_THRESHOLD:
            return False
        props['status'] = 'satisfied'
        return True

    def _refresh_concern_engagement(self, note_id: str) -> None:
        """Bump last_engaged_at AND last_acted_at to now. Called when a
        concern surfaces via semantic recall on a USER-driven turn —
        engagement is both a decay reset (last_engaged_at) and a
        cadence reset (last_acted_at: the user just brought it up,
        treat it as addressed for this cycle so we don't autonomously
        re-fire it)."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return
        props = note.setdefault('properties', {})
        now_iso = datetime.now(timezone.utc).isoformat()
        props['last_engaged_at'] = now_iso
        props['last_acted_at'] = now_iso

    def _iter_active_concerns(self) -> List[Tuple[str, Dict[str, Any], float]]:
        """Iterate the concerns collection and return (note_id, note,
        weight) for every active concern. Performs lazy decay transitions
        as a side effect — after this call, any concern whose decayed
        weight fell below the threshold has its status flipped to
        satisfied (except seed concerns)."""
        if not self._concerns_collection_id:
            return []
        coll = self.resource_manager.get_resource(self._concerns_collection_id)
        if not coll:
            return []
        note_ids = (coll.get('properties') or {}).get('content', []) or []
        out: List[Tuple[str, Dict[str, Any], float]] = []
        for nid in note_ids:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get('properties') or {}
            if props.get('status') != 'active':
                continue
            w = self._concern_decayed_weight(props)
            if self._maybe_transition_to_satisfied(note, w):
                continue
            out.append((nid, note, w))
        return out

    def _top_active_concerns(self, n: int = _CONCERN_ALWAYS_ON_BUDGET
                             ) -> List[Tuple[str, str, str, float]]:
        """Return up to n active concerns ranked by current decayed
        weight, descending. Tuple shape: (note_id, text, category, weight).
        No reinforcement — top-N surfacing alone doesn't refresh engagement."""
        active = self._iter_active_concerns()
        active.sort(key=lambda t: t[2], reverse=True)
        out: List[Tuple[str, str, str, float]] = []
        for nid, note, w in active[:max(0, n)]:
            props = note.get('properties') or {}
            text = str(props.get('content', '') or '').strip()
            if not text:
                continue
            cat = str(props.get('category', 'durable') or 'durable')
            out.append((nid, text, cat, w))
        return out

    def _recall_concerns(self, query: str, k: int = 3,
                         threshold: float = 0.5,
                         refresh: bool = True
                         ) -> List[Tuple[str, str, str, float]]:
        """Semantic recall over the concerns collection. Returns (note_id,
        text, category, weight) tuples for active matches.

        refresh=True (default): hits refresh last_engaged_at and
        last_acted_at — semantics for USER-driven turns where the user
        engaging with a topic is both decay reset and cadence reset.
        refresh=False: read-only — for autonomous turns (tick-driven),
        which must NOT be conflated with user engagement.
        """
        if not self._concerns_collection_id or not query:
            return []
        try:
            with self._faiss_lock:
                ok, results, err = self.resource_manager.search_collection(
                    self.character_name, self._concerns_collection_id, query,
                    mode='semantic', limit=k, threshold=threshold)
            if not ok or not results:
                return []
            out: List[Tuple[str, str, str, float]] = []
            for r in results:
                if not isinstance(r, dict):
                    continue
                doc = r.get('document')
                if not isinstance(doc, str) or not doc.strip():
                    continue
                meta = r.get('metadata') or {}
                note_id = meta.get('source_note_id')
                if not note_id:
                    continue
                note = self.resource_manager.get_resource(note_id)
                if not note:
                    continue
                props = note.get('properties') or {}
                if props.get('status') != 'active':
                    continue
                # Reinforce on user-driven turns only.
                if refresh:
                    self._refresh_concern_engagement(note_id)
                w = self._concern_decayed_weight(note.get('properties') or {})
                cat = str(props.get('category', 'durable') or 'durable')
                out.append((note_id, doc.strip(), cat, w))
            return out
        except Exception as e:
            logger.warning(f"[{self.character_name}] _recall_concerns failed: {e}")
            return []

    def _get_concerns_for_prompt(self, query: str,
                                 refresh: bool = True
                                 ) -> List[Tuple[str, str, str, bool]]:
        """Combined concern surface for the system prompt. Union of:
          - semantic recall on the user's input (reinforces matches)
          - top-N active concerns by current weight (always-on budget)
        Deduped by note_id; ordering: recall hits first (most relevant
        to this turn), then top-N filler. Returns (text, category,
        status_text, is_seed) where status_text is the rendered
        operational sub-line ('' for categories with no firing
        semantics worth surfacing). Cadence/instruction/anchor logic
        is resolved here so the LLM doesn't have to do timestamp
        arithmetic against `## Now`; anchor is last_acted_at — the same
        anchor used by _check_and_fire_concerns — so this surface and
        the firing logic agree."""
        seen: set = set()
        out: List[Tuple[str, str, str, bool]] = []
        now = datetime.now(timezone.utc)

        def _status(note_id: str, category: str) -> Tuple[str, bool]:
            """Return (status_text, is_seed). status_text is the indented
            sub-line content (no leading whitespace), or '' to omit."""
            note = self.resource_manager.get_resource(note_id)
            if not note:
                return '', False
            props = note.get('properties') or {}
            is_seed = bool(props.get('seed'))
            if category in ('derived', 'one_shot'):
                return '', is_seed
            cadence_h = self._resolve_cadence_hours(props)
            instruction = (props.get('instruction') or '').strip()
            if cadence_h is None:
                return 'standing directive, no firing rhythm', is_seed
            if not instruction:
                return (f"won't run — {cadence_h}h cadence set but no instruction",
                        is_seed)
            # Has cadence + instruction: time-based status.
            anchor_str = props.get('last_acted_at') or props.get('last_fired_at')
            anchor_label = 'since last run'
            if not anchor_str:
                anchor_str = props.get('created_at')
                anchor_label = 'since creation'
            if not anchor_str:
                return f"runnable every {cadence_h}h, due now", is_seed
            try:
                anchor = datetime.fromisoformat(str(anchor_str))
                if anchor.tzinfo is None:
                    anchor = anchor.replace(tzinfo=timezone.utc)
                elapsed_h = (now - anchor).total_seconds() / 3600.0
            except Exception:
                return f"runnable every {cadence_h}h, due now", is_seed
            if elapsed_h >= float(cadence_h):
                return f"runnable every {cadence_h}h, due now", is_seed
            return (f"runnable every {cadence_h}h, {int(elapsed_h)}h {anchor_label}",
                    is_seed)

        for nid, text, cat, _w in self._recall_concerns(query, k=3, refresh=refresh):
            if nid in seen:
                continue
            seen.add(nid)
            status_text, is_seed = _status(nid, cat)
            out.append((text, cat, status_text, is_seed))
        for nid, text, cat, _w in self._top_active_concerns():
            if nid in seen:
                continue
            seen.add(nid)
            status_text, is_seed = _status(nid, cat)
            out.append((text, cat, status_text, is_seed))
        return out

    def _set_concern_status(self, concern_id: str, new_status: str
                            ) -> Tuple[bool, Optional[str]]:
        """Manual status transition (for browser-driven abandon/close).
        Returns (ok, error_message)."""
        if new_status not in _CONCERN_STATUSES:
            return False, f"invalid status {new_status!r}"
        note = self.resource_manager.get_resource(concern_id)
        if not note:
            return False, f"concern {concern_id} not found"
        props = note.get('properties') or {}
        if props.get('kind') != 'concern':
            return False, f"{concern_id} is not a concern"
        props['status'] = new_status
        return True, None

    def _check_and_fire_concerns(self, source: str,
                                  emit_impulse: bool = True
                                  ) -> List[Tuple[str, str, str]]:
        """Identify concerns whose cadence has elapsed since last_acted_at.
        Returns a list of (note_id, text, instruction) tuples for each due
        concern with a non-null instruction.

        Cadence anchor is last_acted_at (with last_fired_at as legacy
        fallback for migration, then created_at). Recall hits and
        recurrence promotion bump last_acted_at, so user-driven engagement
        also resets the cadence clock.

        Side effects:
          - emit_impulse=True: print a Phase B-display impulse line per
            fired concern, then bump last_acted_at to now (the impulse
            surfacing counts as a cadence reset so the same impulse
            doesn't repeat on every subsequent turn).
          - emit_impulse=False: caller (e.g. tick handler) will decide
            when to bump last_acted_at — typically only after the
            autonomous ReAct run succeeds.
        """
        if not self._concerns_collection_id:
            return []
        coll = self.resource_manager.get_resource(self._concerns_collection_id)
        if not coll:
            return []
        note_ids = (coll.get('properties') or {}).get('content', []) or []
        now = datetime.now(timezone.utc)
        fired: List[Tuple[str, str, str]] = []
        for nid in note_ids:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get('properties') or {}
            if props.get('status') != 'active':
                continue
            cadence_hours = self._resolve_cadence_hours(props)
            if cadence_hours is None:
                continue   # no autonomous fire (one_shot/derived/seed)
            instruction = props.get('instruction')
            if not instruction:
                continue   # no action to take; skip
            anchor_str = (props.get('last_acted_at')
                          or props.get('last_fired_at')   # legacy migration
                          or props.get('created_at'))
            if not anchor_str:
                continue
            try:
                anchor = datetime.fromisoformat(str(anchor_str))
                if anchor.tzinfo is None:
                    anchor = anchor.replace(tzinfo=timezone.utc)
            except Exception:
                continue
            elapsed_hours = (now - anchor).total_seconds() / 3600.0
            if elapsed_hours < float(cadence_hours):
                continue
            text = str(props.get('content', '') or '').strip()
            fired.append((nid, text, str(instruction)))
            if emit_impulse:
                # Phase B-display path: bump last_acted_at on impulse so
                # we don't reprint the same line on every subsequent user
                # turn within the same cadence cycle.
                props['last_acted_at'] = now.isoformat()
        if emit_impulse:
            for _nid, text, instruction in fired:
                self._emit_impulse(text, instruction)
        return fired

    def _emit_impulse(self, concern_text: str, instruction: str) -> None:
        """Print a concern-fired impulse to the CLI as a permanent line
        (not transient — distinct from the ReAct status helpers, which
        overwrite themselves). Falls back to logger.info when stdout
        isn't a TTY so the signal is preserved in non-interactive runs."""
        msg = f"concern: {concern_text} — impulse: {instruction}"
        if not sys.stdout.isatty():
            logger.info(f"[{self.character_name}] {msg}")
            return
        try:
            line = (f"\n{self._STATUS_DIM}[{self.character_name}] "
                    f"{msg}{self._STATUS_RESET}\n")
            sys.stdout.write(line)
            sys.stdout.flush()
        except Exception:
            pass

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
        "- `one_shot`: a specific request the user made that has a clear "
        "completion (\"look up X\", \"draft me an email about Y\"). Decays "
        "fast. Does NOT fire autonomously — fulfilled this turn.\n"
        "- `durable`: an ongoing directive {entity} expects {character} to "
        "uphold over time (\"keep me posted on X\", \"help me think through "
        "my dissertation\"). FIRES per its cadence; provide cadence_hours "
        "and instruction.\n"
        "- `derived`: something {character} noticed worth tracking that the "
        "user did NOT explicitly request. Does NOT fire autonomously — "
        "lives as background context. Use sparingly.\n"
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
        "    one_shot or derived: null (no autonomous fire)\n"
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
        "    \"Pull recent papers on multi-agent coordination from arxiv.\"\n\n"
        "**REQUIRED for category=durable**: cadence_hours MUST be one of the "
        "allowed values, lifetime_days MUST be a number, instruction MUST be "
        "a non-empty string. A durable concern without these fields cannot "
        "fire and is operationally useless. Do not skip them.\n"
        "**For one_shot and derived**: cadence_hours and instruction may be "
        "null (those categories don't fire autonomously). lifetime_days "
        "should still be a number to drive decay; null is also acceptable.\n\n"
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
            existing_concerns = self._top_active_concerns(n=10)
            sys_msg = self._REFLECT_SYS.format(
                character=self.character_name, entity=source)
            user_parts = []
            if companion:
                user_parts.append(
                    "## Existing companion model (do NOT re-extract from this; "
                    "use only to avoid duplicates)\n" + companion)
            if existing_concerns:
                lines = [f"- [{cat}] {text}" for _nid, text, cat, _w in existing_concerns]
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
                # Concerns derived by reflection: provenance follows category.
                # one_shot/durable typically come from explicit user requests
                # (asserted); derived is by definition inferred.
                provenance = 'inferred' if category == 'derived' else 'asserted'
                if self._add_concern(text, category=category, entity=source,
                                     provenance=provenance, seed=False,
                                     cadence_hours=c.get('cadence_hours'),
                                     lifetime_days=c.get('lifetime_days'),
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
        absolute path). Returns text capped to _FETCH_TEXT_OBS_CAP chars
        for the ReAct working log. Returns a `(fetch_text …)` failure
        marker on any error so the model sees the failure cleanly."""
        if not url:
            return '(fetch_text: empty url)'
        fetch = self._get_fetch_text_tool()
        if fetch is None:
            return '(fetch_text: tool unavailable)'
        try:
            result = fetch(url, executor=self._FetchTextStubExecutor,
                           resource_manager=self.resource_manager)
        except Exception as e:
            logger.warning(f"[{self.character_name}] fetch_text raised: {e}")
            return f'(fetch_text error: {e})'
        if not isinstance(result, dict):
            return '(fetch_text: unexpected return shape)'
        if result.get('status') != 'success':
            return f"(fetch_text failed: {result.get('reason') or 'unknown'})"
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
            return '(fetch_text: no text extracted)'
        if len(text) > _FETCH_TEXT_OBS_CAP:
            text = text[:_FETCH_TEXT_OBS_CAP].rstrip() + f"\n…[truncated at {_FETCH_TEXT_OBS_CAP} chars]"
        return text

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
        """Literal string passes through; `$stepN` looks up the log."""
        if not isinstance(val, str):
            return str(val) if val is not None else ''
        if _REACT_BINDING_RE.match(val):
            for label, content in log:
                if label == val:
                    return content
            return ''
        return val

    def _diagnose_process_text_args(self, raw_src: Any, resolved_src: str,
                                    instruction: Any,
                                    log: List[Tuple[str, str]]
                                    ) -> Optional[str]:
        """Return a diagnostic observation when process_text args look
        malformed — section-name placeholder as `source`, unresolved
        `$stepN` binding, or empty fields. Returns None when args are
        usable. The model has been observed to recover cleanly when
        given a legible error, so each diagnostic names the failure
        AND points at the recovery path."""
        if not isinstance(instruction, str) or not instruction.strip():
            return ("(process_text requires a non-empty `instruction` "
                    "field; no instruction was supplied.)")
        # Unresolved $stepN: model passed a binding that doesn't exist.
        if isinstance(raw_src, str) and _REACT_BINDING_RE.match(raw_src) and not resolved_src:
            bound = sorted({lab for lab, _ in log
                            if _REACT_BINDING_RE.match(lab)})
            avail = ", ".join(bound) if bound else "(none yet this turn)"
            return (f"(process_text `source` is `{raw_src}`, an unresolved "
                    f"binding. Available `$stepN` bindings this turn: "
                    f"{avail}. Pass an existing binding or literal "
                    f"inline text instead.)")
        if not resolved_src:
            return ("(process_text `source` is empty. Pass either "
                    "literal text content (the actual material to "
                    "process) or a `$stepN` binding from a prior tool "
                    "call this turn.)")
        # Heading-shaped placeholder: starts with `##` and is short.
        # Real content the model wants processed is typically much
        # longer; a section heading is rarely a reasonable source.
        stripped = resolved_src.lstrip()
        if stripped.startswith('##') and len(resolved_src) < 500:
            return ("(process_text `source` appears to be a section "
                    "heading like `## Conversation history` rather "
                    "than content. Section bodies are already in your "
                    "system prompt — read them directly in `respond` "
                    "rather than passing the heading as `source`. "
                    "Section names do not resolve to their bodies; "
                    "`source` accepts only literal inline text or a "
                    "`$stepN` binding.)")
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
            return f"(process_text error: {e})"
        return (result or '').strip()

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
            'category': props.get('category', 'durable'),
            'status': status,
            'weight': self._concern_decayed_weight(props),
            'provenance': props.get('provenance', 'asserted'),
            'origin': 'seed' if props.get('seed') else 'reflection',
            'seed': bool(props.get('seed', False)),
            'entity': props.get('entity', ''),
            'created_at': props.get('created_at', ''),
            'created': props.get('created_at', ''),
            'last_engaged_at': props.get('last_engaged_at', ''),
            'recency': props.get('last_engaged_at', ''),
            # Phase B firing parameters. cadence_hours is editable from the
            # browser via the concern_manage `set_cadence_hours` action.
            'cadence_hours': self._resolve_cadence_hours(props),
            'cadence_hours_allowed': list(_CONCERN_CADENCE_HOURS_ALLOWED),
            'lifetime_days': self._resolve_lifetime_days(props),
            'instruction': props.get('instruction'),
            'last_acted_at': (props.get('last_acted_at')
                              or props.get('last_fired_at')   # legacy migration
                              or None),
        }

    def _all_concerns_split(self) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
        """Iterate the concerns collection and split by category for the
        browser API. No status filtering — browser shows all states."""
        if not self._concerns_collection_id:
            return ([], [])
        coll = self.resource_manager.get_resource(self._concerns_collection_id)
        if not coll:
            return ([], [])
        note_ids = (coll.get('properties') or {}).get('content', []) or []
        user_concerns: List[Dict[str, Any]] = []
        derived_concerns: List[Dict[str, Any]] = []
        for nid in note_ids:
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            cat = (note.get('properties') or {}).get('category', 'durable')
            if cat == 'derived':
                derived_concerns.append(self._serialize_concern(nid, note, is_user_kind=False))
            else:
                user_concerns.append(self._serialize_concern(nid, note, is_user_kind=True))
        return (user_concerns, derived_concerns)

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
        """Browser → chat: status transitions, cadence edit, hard delete.
        Payload: {"concern_id": "Note_X", "action": "close|reopen|satisfy|
                  abandon|activate|delete|set_cadence_hours",
                  "type": "user|derived",
                  "value": <int or null>}  # required for set_cadence_hours

        The action vocabulary mirrors what resource_browser.py emits.
        `delete` is a hard delete (the browser confirms with a dialog
        first); status actions go through _set_concern_status;
        set_cadence_hours snaps the value to the allowed bucket and
        writes to the concern note."""
        try:
            payload_bytes = query.payload.to_bytes() if query.payload else b'{}'
            params = json.loads(payload_bytes.decode('utf-8')) if payload_bytes else {}
            concern_id = str(params.get('concern_id', '') or '').strip()
            action = str(params.get('action', '') or '').strip().lower()
            if not concern_id or not action:
                self._reply(query, {'success': False,
                                    'error': "missing concern_id or action"})
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

            # Cadence edit from the browser combo-box.
            if action == 'set_cadence_hours':
                note = self.resource_manager.get_resource(concern_id)
                if not note or (note.get('properties') or {}).get('kind') != 'concern':
                    self._reply(query, {'success': False,
                                        'error': f"{concern_id} is not a concern"})
                    return
                snapped = self._snap_cadence_hours(params.get('value'))
                note['properties']['cadence_hours'] = snapped
                self._persist_to_disk()
                self._reply(query, {'success': True,
                                    'message': f'{concern_id} cadence_hours → {snapped}',
                                    'cadence_hours': snapped})
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
            for _nid, note, _w in self._iter_active_concerns():
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
                             concerns: Optional[List[Tuple[str, str, str, bool]]] = None) -> str:
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
        if concerns:
            # Active concerns sit ABOVE memories: concerns are directives
            # to advance, memories are background to draw on. Each item
            # renders as `- [category(, seed)] text` with an optional
            # indented sub-line carrying the operational status (cadence
            # + run state, or a consequence-leading note when the concern
            # can't run). Sub-lines are computed in _get_concerns_for_prompt
            # against the same anchor used by _check_and_fire_concerns.
            con_lines: List[str] = []
            for item in concerns:
                text, cat, status_text, is_seed = item
                tag = f"{cat}, seed" if is_seed else cat
                con_lines.append(f"- [{tag}] {text}")
                if status_text:
                    con_lines.append(f"    {status_text}")
            parts.append(
                f"## Active concerns (from YAML seeds + post-turn reflection + semantic recall)\n"
                f"Three categories: `one_shot` (do once), `durable` "
                f"(standing or recurring), `derived` (Jill's own "
                f"observation). Where relevant, an indented sub-line "
                f"states the operational status. Recall reinforces "
                f"concerns on engagement; without it they decay.\n\n"
                + "\n".join(con_lines)
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
                                   concerns: Optional[List[Tuple[str, str, str, bool]]] = None) -> str:
        base = self._build_system_prompt(source, orientation, recall=recall, concerns=concerns)
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
            "4. `{\"thought\": \"<one terse sentence>\", \"tool\": \"respond\", \"text\": <string|$stepN>}` — final reply, exits loop. "
            "Must be in your voice; pass search/fetch results through process_text first or write the reply yourself.\n"
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
        history = self.store.get_recent_turns(source, limit=self.history_limit, scope='all')
        if history and history[-1].get('direction') == 'in' and str(history[-1].get('text', '')) == user_text:
            history = history[:-1]
        if history:
            parts.append("## Conversation history (verbatim session turns)")
            for t in history:
                who = source if t.get('direction') == 'in' else self.character_name
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
                        concerns: Optional[List[Tuple[str, str, str, bool]]] = None
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
            source, orientation, now_str, recall=recall, concerns=concerns)
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

            if tool == 'process_text':
                raw_src = action.get('source', '')
                ins = action.get('instruction', '')
                src = self._resolve_react_value(raw_src, log)
                diag = self._diagnose_process_text_args(raw_src, src, ins, log)
                if diag is not None:
                    logger.warning(f"[{self.character_name}] process_text "
                                   f"dispatch rejected (iter {i+1}): {diag}")
                    obs = diag
                else:
                    obs = self._run_process_text(src, ins)
            elif tool == 'search':
                q = self._resolve_react_value(action.get('query', ''), log)
                result = self._run_web_search(q) if q else None
                obs = self._format_react_search_observation(result) if result else '(search failed or empty query)'
            elif tool == 'fetch_text':
                u = self._resolve_react_value(action.get('url', ''), log)
                obs = self._run_fetch_text(u)
            else:
                obs = f"unknown tool {tool!r}; available: process_text, search, fetch_text, respond"

            _append_log(binding, obs)
            iters[-1]['appended'] = log[pre_log_len:]
            logger.info(f"[{self.character_name}] ReAct iter {i+1}: {tool} → {binding} ({len(obs)} chars)")

        logger.warning(f"[{self.character_name}] ReAct hit max iters ({REACT_MAX_ITERS})")
        reply = self._react_fallback_synthesis(log)
        self._clear_status()
        return reply, log, iters, 'max_iters'

    def _write_react_trace(self, source: str, user_text: str,
                           log: List[Tuple[str, str]],
                           iters: List[Dict[str, Any]],
                           final_response: str,
                           exit_reason: str,
                           recall: Optional[List[Tuple[str, str]]] = None) -> None:
        """Append one ReAct session to logs/chat_trace_{character}.txt
        as the literal byte-stream that was sent to the LLM, with no
        editorial annotation. Each iteration is one (USER, ASSISTANT)
        pair; the SYSTEM message is shown once (it's byte-stable across
        iterations). A single dim header line per turn carries minimal
        metadata (timestamp / source / iters / exit) so multiple turns
        in the same file are separable, and that's it."""
        try:
            from pathlib import Path
            log_dir = Path(__file__).resolve().parent.parent / 'logs'
            log_dir.mkdir(exist_ok=True)
            trace_path = log_dir / f'chat_trace_{self.character_name}.txt'
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
                                  autonomous: bool = False) -> None:
        """Persist one ReAct turn's trace into the reasoning_history
        collection. Note content is the rendered full trace (used as the
        prompt-side full form). Properties carry a one-line compressed
        digest (used when this trace ages out of the full window) plus
        metadata. Ring-prunes oldest entries when the collection grows
        past _REASONING_HISTORY_RING_SIZE."""
        if not self._reasoning_history_collection_id:
            return
        try:
            actions: List[str] = []
            iter_lines: List[str] = []
            for idx, it in enumerate(iters):
                raw = it.get('raw', '') or ''
                thought = ''
                action_summary = (raw or '').strip()
                tool = '?'
                try:
                    parsed = json.loads(raw)
                    if isinstance(parsed, dict):
                        thought = str(parsed.get('thought', '') or '').strip()
                        tool = str(parsed.get('tool', '?') or '?')
                        action_summary = json.dumps(
                            {k: v for k, v in parsed.items() if k != 'thought'},
                            ensure_ascii=False)
                except Exception:
                    pass
                actions.append(tool)

                obs_text = ''
                for label, content in (it.get('appended') or []):
                    if isinstance(label, str) and label.startswith('$step'):
                        s = str(content or '')
                        if len(s) > _REASONING_HISTORY_OBS_CAP:
                            s = s[:_REASONING_HISTORY_OBS_CAP] + '… [truncated]'
                        obs_text = s
                        break

                iter_lines.append(f"  ITER {idx + 1}:")
                if thought:
                    iter_lines.append(f"    thought: {thought}")
                iter_lines.append(f"    action:  {action_summary}")
                if obs_text:
                    iter_lines.append(f"    obs:     {obs_text}")

            header_label = '(autonomous)' if autonomous else f'({source})'
            header = f"INPUT {header_label}: {user_text}"
            full_lines = [header] + iter_lines
            if final_response:
                resp = final_response
                if len(resp) > _REASONING_HISTORY_OBS_CAP:
                    resp = resp[:_REASONING_HISTORY_OBS_CAP] + '… [truncated]'
                full_lines.append(f"  RESPONSE: {resp}")
            full_trace = "\n".join(full_lines)

            user_short = user_text if len(user_text) <= 60 else user_text[:57] + '…'
            # Compressed form: input prompt + action chain + a truncated
            # snippet of what Jill actually said. The action chain alone
            # ("→ search → respond") tells future-Jill what the loop did
            # but not what was concluded — useless for self-reflection
            # probes that ask "what have I claimed?". Including a response
            # snippet costs ~200 chars per aged trace and lets the older
            # awareness-feed entries carry information content, not just
            # mechanics.
            resp_short = ''
            if final_response:
                _r = final_response.replace('\n', ' ').strip()
                if len(_r) > 200:
                    _r = _r[:197].rstrip() + '…'
                resp_short = _r
            compressed = (
                f"{header_label} '{user_short}' → "
                + (" → ".join(actions) if actions else "(no actions)")
                + (f" → said: \"{resp_short}\"" if resp_short else "")
            )

            success, note_id, err, _ = self.resource_manager.create_note(
                self.character_name, full_trace, "text", "chat-loop",
                source or "", "",
                {
                    "kind": "reasoning_trace",
                    "user_text": user_text,
                    "autonomous": bool(autonomous),
                    "exit_reason": exit_reason,
                    "n_iters": len(iters),
                    "compressed": compressed,
                },
            )
            if not success or not note_id:
                logger.warning(
                    f"[{self.character_name}] reasoning_history note create failed: {err}")
                return
            self.resource_manager.add_to_collection(
                self._reasoning_history_collection_id, note_id,
                self.character_name, operation='add')
            self.resource_manager.mark_persistent(note_id, self.character_name)

            # Ring-prune oldest entries if we've exceeded the cap.
            coll = self.resource_manager.get_resource(self._reasoning_history_collection_id)
            note_ids = ((coll or {}).get('properties') or {}).get('content', []) or []
            if len(note_ids) > _REASONING_HISTORY_RING_SIZE:
                excess = len(note_ids) - _REASONING_HISTORY_RING_SIZE
                for old_id in note_ids[:excess]:
                    try:
                        self.resource_manager.delete_resource(old_id)
                    except Exception:
                        pass
        except Exception as e:
            logger.warning(f"[{self.character_name}] _write_reasoning_history failed: {e}")

    def _get_reasoning_history_block(self) -> str:
        """Build the ## Recent reasoning block for the user-message prefix.
        Returns '' when there's nothing to surface. Last
        _REASONING_HISTORY_RECENT entries are read; the most recent
        _REASONING_HISTORY_FULL render in full, older render compressed."""
        if not self._reasoning_history_collection_id:
            return ''
        coll = self.resource_manager.get_resource(self._reasoning_history_collection_id)
        if not coll:
            return ''
        note_ids = (coll.get('properties') or {}).get('content', []) or []
        if not note_ids:
            return ''
        recent = note_ids[-_REASONING_HISTORY_RECENT:]
        sections: List[str] = []
        n = len(recent)
        full_threshold = max(0, n - _REASONING_HISTORY_FULL)
        for idx, nid in enumerate(recent):
            note = self.resource_manager.get_resource(nid)
            if not note:
                continue
            props = note.get('properties') or {}
            offset_label = f"trace -{n - idx}"
            if idx >= full_threshold:
                content = str(props.get('content', '') or '').strip()
                if content:
                    sections.append(f"### {offset_label} (full)\n{content}")
            else:
                compressed = str(props.get('compressed', '') or '').strip()
                if compressed:
                    sections.append(f"### {offset_label}: {compressed}")
        if not sections:
            return ''
        return ("## Recent reasoning (my own ReAct traces from prior turns)\n"
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

        if not text:
            if close:
                self.store.close_dialog(source)
            return

        orientation = self._orientation_summary(source, text)
        # Auto-RAG: pull top-k durable memories that match this turn's input.
        # Injected next to the Companion block in the system prompt; no ReAct
        # tool call required. Cheap miss (one embedding query).
        recall = self._recall(text, k=3)
        # Active concerns: combined surface of semantic-recall hits (which
        # reinforce engagement on user turns) + top-N by current weight
        # (always-on budget). On autonomous turns, recall is read-only —
        # autonomous use must NOT register as user engagement.
        concerns = self._get_concerns_for_prompt(text, refresh=not autonomous)
        # Concern firing (Phase B-display): user-turn impulses. Skipped on
        # autonomous turns — _handle_tick has already identified what fired
        # and is the path that's running this turn.
        if not autonomous:
            self._check_and_fire_concerns(source)

        log: List[Tuple[str, str]] = []
        iters: List[Dict[str, Any]] = []
        exit_reason = 'crashed'
        try:
            reply, log, iters, exit_reason = self._run_react_loop(
                source, text, orientation, recall=recall, concerns=concerns)
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
        self._publish_say(reply)
        logger.info(f"[{self.character_name}] -> {source} ({act_type}): {reply!r}")

        # Bump last_acted_at on the fired concern AFTER successful reply
        # publication — the autonomous run has now actually executed.
        if autonomous and autonomous_concern_id and exit_reason != 'crashed':
            note = self.resource_manager.get_resource(autonomous_concern_id)
            if note:
                note.setdefault('properties', {})['last_acted_at'] = (
                    datetime.now(timezone.utc).isoformat())

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
    # Triggered by the `tick` sensor's heartbeat. Runs check_and_fire
    # without emitting Phase B impulses (we'll surface fires via the
    # autonomous-act path), then dispatches up to _AUTONOMOUS_FIRE_CAP
    # concerns through the standard ReAct pipeline. Surplus concerns
    # remain due (last_acted_at unchanged) and will be picked up on
    # the next tick.
    # ------------------------------------------------------------------

    _AUTONOMOUS_FIRE_CAP = 2

    def _handle_tick(self) -> None:
        """Per-tick autonomy pass. Identifies due concerns, runs up to
        _AUTONOMOUS_FIRE_CAP ReAct loops on their instructions, prints
        a CLI preamble per fire and a deferral note if any are dropped.
        """
        fired = self._check_and_fire_concerns('User', emit_impulse=False)
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
            except Exception:
                pass

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
            except Exception:
                pass
            try:
                self._process_user_turn(
                    source='User', text=instruction, close=False,
                    autonomous=True, autonomous_concern_id=nid)
            except Exception as e:
                logger.error(f"[{self.character_name}] autonomous fire failed for {nid}: {e}")
                import traceback
                traceback.print_exc()

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
                    logger.info(f"[{self.character_name}] <- tick")
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
