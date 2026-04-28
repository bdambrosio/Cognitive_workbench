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

# Decay tau in days, per concern category. Effective weight at read time
# is exp(-(now - last_engaged_at) / tau), starting from 1.0 at engagement.
# Recall hits refresh last_engaged_at. When effective weight drops below
# the satisfied threshold, the concern transitions from active to satisfied
# (lazy — computed on read, no background sweep).
_CONCERN_DECAY_TAU_DAYS = {
    'one_shot': 1.0,    # ~3 days to drop below 0.05
    'derived':  7.0,    # ~3 weeks to drop below 0.05
    'durable':  30.0,   # ~3 months to drop below 0.05
}
_CONCERN_SATISFIED_THRESHOLD = 0.1
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

# Per-iteration auto-binding: $step1, $step2, ... names the result of each
# action so subsequent actions can reference it. Scoped to the current turn
# only — does not leak into conversation history or the next turn's loop.
_REACT_BINDING_RE = re.compile(r'^\$step\d+$')


# ─── LLM backend ────────────────────────────────────────────────────────────

class _ChatBackend:
    """Thin OpenAI-compatible chat client with structured-CoT support.

    Routes (checked in order):
      1. `api_key` set → unified OpenAI-compat path. POST to
         {base_url}/v1/chat/completions with `Authorization: Bearer
         <env[api_key]>`. Grammar / chat_template_kwargs are NOT attached
         (cloud endpoints reject them). The api_key field is the NAME of
         an environment variable, not the key itself.
      2. server in ('openrouter', 'openai') → utils.llm_api.LLM (legacy
         cloud shortcut, kept for back-compat).
      3. anything else (local, vllm, llama.cpp, sglang_api_server, lmstudio,
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

        # ---- Feature flags (on by default per project decision) ----
        self.discourse_enabled = bool((character_config.get('discourse') or {}).get('enabled', True))
        self.orientation_enabled = bool((character_config.get('orientation') or {}).get('enabled', True))
        self.history_limit = int((character_config.get('chat') or {}).get('history_limit', 20))

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

    @staticmethod
    def _seed_concern_name(idx: int) -> str:
        return f"chat:concern:seed:{idx}"

    def _seed_concerns_from_config(self, character_config: dict) -> None:
        """Instantiate concerns listed under YAML key `concerns:` if they
        don't already exist. Each seed gets a stable named-note slot so
        re-init is idempotent. Seed concerns carry seed=True and are
        immune to decay-induced satisfaction (they're architectural
        baseline, not ephemeral user-derived content)."""
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
            self._add_concern(text, category=category, entity=entity,
                              provenance='asserted', seed=True, name=name)

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
          - last_engaged_at always refreshed (the recurrence IS engagement)
          - status: satisfied → active (revival)
          - category: one_shot → durable (intensity promotion)
          - durable / derived: category unchanged (no de-promotion)
        Returns the same note_id so callers have a uniform shape."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return note_id
        props = note.setdefault('properties', {})
        props['last_engaged_at'] = datetime.now(timezone.utc).isoformat()
        if props.get('status') == 'satisfied':
            props['status'] = 'active'
        if props.get('category') == 'one_shot':
            props['category'] = 'durable'
        return note_id

    def _add_concern(self, text: str, category: str = 'durable',
                     entity: str = 'User', provenance: str = 'asserted',
                     seed: bool = False, name: str = '') -> Optional[str]:
        """Create a new concern note and add it to the concerns Collection,
        OR — if a near-twin already exists — refresh and promote/revive
        the existing note instead. Returns the resulting note_id (newly
        created or pre-existing), or None on failure. Held under
        _faiss_lock for the FAISS-touching span; pre-add similarity check
        also serializes through the same lock."""
        if not self._concerns_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        if category not in _CONCERN_CATEGORIES:
            category = 'durable'
        if provenance not in ('asserted', 'inferred'):
            provenance = 'inferred' if category == 'derived' else 'asserted'

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

    @staticmethod
    def _concern_decayed_weight(properties: Dict[str, Any]) -> float:
        """Weight = exp(-Δdays / tau) where Δ is now − last_engaged_at and
        tau depends on category. Returns 0.0 for malformed timestamps."""
        import math
        try:
            last_iso = properties.get('last_engaged_at') or properties.get('created_at')
            if not last_iso:
                return 0.0
            last = datetime.fromisoformat(str(last_iso))
            if last.tzinfo is None:
                last = last.replace(tzinfo=timezone.utc)
            delta_days = (datetime.now(timezone.utc) - last).total_seconds() / 86400.0
            cat = str(properties.get('category', 'durable') or 'durable').lower()
            tau = _CONCERN_DECAY_TAU_DAYS.get(cat, _CONCERN_DECAY_TAU_DAYS['durable'])
            if tau <= 0:
                return 1.0
            return math.exp(-max(0.0, delta_days) / tau)
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
        """Reset last_engaged_at to now for one concern note. Called when
        a concern surfaces via semantic recall."""
        note = self.resource_manager.get_resource(note_id)
        if not note:
            return
        props = note.setdefault('properties', {})
        props['last_engaged_at'] = datetime.now(timezone.utc).isoformat()

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
                         threshold: float = 0.5) -> List[Tuple[str, str, str, float]]:
        """Semantic recall over the concerns collection. Hits refresh
        last_engaged_at on the source notes. Returns (note_id, text,
        category, weight) tuples for active matches."""
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
                # Reinforce: hits refresh last_engaged_at, then we
                # recompute weight so the returned tuple reflects the
                # post-refresh value (close to 1.0).
                self._refresh_concern_engagement(note_id)
                w = self._concern_decayed_weight(note.get('properties') or {})
                cat = str(props.get('category', 'durable') or 'durable')
                out.append((note_id, doc.strip(), cat, w))
            return out
        except Exception as e:
            logger.warning(f"[{self.character_name}] _recall_concerns failed: {e}")
            return []

    def _get_concerns_for_prompt(self, query: str
                                 ) -> List[Tuple[str, str]]:
        """Combined concern surface for the system prompt. Union of:
          - semantic recall on the user's input (reinforces matches)
          - top-N active concerns by current weight (always-on budget)
        Deduped by note_id; ordering: recall hits first (most relevant
        to this turn), then top-N filler. Returns (text, category)."""
        seen: set = set()
        out: List[Tuple[str, str]] = []
        for nid, text, cat, _w in self._recall_concerns(query, k=3):
            if nid in seen:
                continue
            seen.add(nid)
            out.append((text, cat))
        for nid, text, cat, _w in self._top_active_concerns():
            if nid in seen:
                continue
            seen.add(nid)
            out.append((text, cat))
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
        "fast.\n"
        "- `durable`: an ongoing directive {entity} expects {character} to "
        "uphold over time (\"keep me posted on X\", \"help me think through "
        "my dissertation\"). Decays slowly.\n"
        "- `derived`: something {character} noticed worth tracking that the "
        "user did NOT explicitly request (\"user keeps coming back to topic "
        "X\"). Use sparingly — only when there's substantive evidence in the "
        "exchange.\n"
        "SKIP from concerns:\n"
        "- Modifiers like \"be brief\" / \"don't use emoji\" — those go to "
        "memories as preferences, not concerns.\n"
        "- Items already covered by an existing concern in the system prompt.\n"
        "- Speculative inferences without textual support.\n\n"
        "Output ONLY this JSON shape — nothing else:\n"
        "  {{\"frame\": \"<hypothetical|roleplay|counterfactual|instructional|none>\", "
        "\"memories\": [{{\"text\": \"<≤200 chars, third-person about {entity}>\", "
        "\"category\": \"<fact|preference|commitment>\"}}, …], "
        "\"concerns\": [{{\"text\": \"<≤200 chars, action-oriented "
        "third-person directive>\", \"category\": \"<one_shot|durable|derived>\"}}, …]}}\n"
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
            for text, category in raw_concerns:
                if len(text) > 240:
                    text = text[:240].rstrip()
                # Concerns derived by reflection: provenance follows category.
                # one_shot/durable typically come from explicit user requests
                # (asserted); derived is by definition inferred.
                provenance = 'inferred' if category == 'derived' else 'asserted'
                if self._add_concern(text, category=category, entity=source,
                                     provenance=provenance, seed=False):
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
                                  ) -> Tuple[str, List[Tuple[str, str]], List[Tuple[str, str]]]:
        """Normalize reflection output to (frame, memories, concerns) where
        each list contains (text, category) tuples.

        Accepts:
          - New envelope: {"frame": "...", "memories": [...], "concerns": [...]}
            with each list of {text, category} dicts (or bare strings).
          - Legacy envelope without `concerns`: still parses, concerns=[].
          - Bare list of strings: assumed frame=none, treated as memories.
        Anything else returns ('unknown', [], []) — caller treats non-`none`
        frame as a suppression signal so this fails safe.
        """

        def _normalize_items(raw: Any, allowed: Tuple[str, ...],
                             default_cat: str) -> List[Tuple[str, str]]:
            out: List[Tuple[str, str]] = []
            if not isinstance(raw, list):
                return out
            for item in raw:
                if isinstance(item, str) and item.strip():
                    out.append((item.strip(), default_cat))
                elif isinstance(item, dict):
                    t = str(item.get('text', '') or '').strip()
                    c = str(item.get('category', default_cat) or default_cat).strip().lower()
                    if c not in allowed:
                        c = default_cat
                    if t:
                        out.append((t, c))
            return out

        # Old shape — bare list. Assume frame=none.
        if isinstance(payload, list):
            return (_REFLECT_FRAME_OK,
                    _normalize_items(payload, _MEMORY_CATEGORIES, 'fact'),
                    [])

        # New envelope.
        if isinstance(payload, dict):
            frame = str(payload.get('frame', '') or '').strip().lower() or 'unknown'
            mems = _normalize_items(payload.get('memories', []),
                                    _MEMORY_CATEGORIES, 'fact')
            cons = _normalize_items(payload.get('concerns', []),
                                    _CONCERN_CATEGORIES, 'durable')
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

        if not text and not close:
            return

        self._inbox.put({'source': source, 'text': text, 'close': close})

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
        """Browser → chat: status transitions and hard delete.
        Payload: {"concern_id": "Note_X", "action": "close|reopen|satisfy|
                  abandon|activate|delete", "type": "user|derived"}.

        The action vocabulary mirrors what resource_browser.py emits.
        `delete` is a hard delete (the browser confirms with a dialog
        first); status actions go through _set_concern_status."""
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
            assessment = evaluate(
                event=event,
                character_concerns=[],
                user_concerns=[],
                goals_compact=[],
                recent_context=recent_str,
                activity_state='chat-only (no autonomous activity)',
                llm_generate=self._make_llm_callable('triage'),
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
                             concerns: Optional[List[Tuple[str, str]]] = None) -> str:
        """Build the persona/state portion of the system prompt — shared base
        for ReAct mode. The prose-only directive that used to live here was
        moved out: ReAct supplies its own JSON-emit directive in
        _build_react_system_prompt. There is no non-ReAct chat path now."""
        parts: List[str] = []
        parts.append(f"You are {self.character_name}, speaking in first person.")
        if self.persona:
            parts.append("## Persona\n" + self.persona)
        if self.capabilities:
            parts.append("## Capabilities (chat-only mode)\n" + self.capabilities)
        if self.setting:
            parts.append("## Setting\n" + self.setting)
        companion = self._companion_state.get(source, '').strip()
        if companion:
            parts.append(
                f"## Companion model of {source} (fair-witness texture, not a brief to flatter)\n"
                f"{companion}"
            )
        if concerns:
            # Active concerns sit ABOVE memories: concerns are directives
            # to advance, memories are background to draw on. Rendering
            # is compact (badges, not headers) since the typical set is
            # 1-5 items.
            con_lines = [f"- [{cat}] {text}" for text, cat in concerns]
            parts.append(
                f"## Active concerns (instructions to keep ready to advance)\n"
                f"one_shot = a specific request to fulfill once; durable = "
                f"ongoing directive; derived = something Jill noticed worth "
                f"tracking. Recall reinforces these on engagement; without it "
                f"they decay.\n\n"
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
            parts.append("## Outstanding discourse objects\n" + disc)
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
                                   recall: Optional[List[Tuple[str, str]]] = None,
                                   concerns: Optional[List[Tuple[str, str]]] = None) -> str:
        base = self._build_system_prompt(source, orientation, recall=recall, concerns=concerns)
        react = (
            "\n\n## ReAct Tool Loop — READ FIRST\n"
            "Each emission is ONE JSON action object — nothing else. Prose "
            "around the JSON is discarded and the loop will retry. Output "
            "begins with `{` and ends with `}`.\n"
            "\n"
            "You do NOT know: today's date, current weather, recent news, "
            "current prices, or anything requiring fresh information. For "
            "time-sensitive or fact-specific questions, your first action "
            "is `search`.\n"
            "\n"
            "Tools (each emission picks ONE):\n"
            "1. `{\"tool\": \"process_text\", \"source\": <string|$stepN>, \"instruction\": <string>}` — "
            "LLM pass over text in context. Use to formulate queries, render results in your voice, extract info.\n"
            "2. `{\"tool\": \"search\", \"query\": <string|$stepN>}` — web search (digested synthesis + sources).\n"
            "3. `{\"tool\": \"fetch_text\", \"url\": <string|$stepN>}` — full text from a single URL "
            "(or local file path). Use when a search hit looks promising and the snippet isn't enough; "
            "always pass the result through process_text before responding.\n"
            "4. `{\"tool\": \"respond\", \"text\": <string|$stepN>}` — final reply, exits loop. "
            "Must be in your voice; pass search/fetch results through process_text first or write the reply yourself.\n"
            "\n"
            "Each action's result auto-binds to `$step1, $step2, …` (per-turn scope).\n"
            "\n"
            "Worked example. User: 'what's the weather in Berkeley tomorrow?'\n"
            "  Iter 1: `{\"tool\": \"search\", \"query\": \"Berkeley CA weather forecast tomorrow\"}` → $step1\n"
            "  Iter 2: `{\"tool\": \"process_text\", \"source\": \"$step1\", \"instruction\": \"answer the user in your voice in 1-2 sentences, citing the source domain\"}` → $step2\n"
            "  Iter 3: `{\"tool\": \"respond\", \"text\": \"$step2\"}` → loop exits.\n"
            "\n"
            "Output ONLY one JSON object. No prose, no apology, no explanation."
        )
        return base + react

    def _build_react_prompt(self, source: str, user_text: str, orientation: str,
                            log: List[Tuple[str, str]],
                            recall: Optional[List[Tuple[str, str]]] = None,
                            concerns: Optional[List[Tuple[str, str]]] = None) -> List[Dict[str, str]]:
        """Single user-role text containing history + current input + working log."""
        parts: List[str] = []
        history = self.store.get_recent_turns(source, limit=self.history_limit, scope='all')
        if history and history[-1].get('direction') == 'in' and str(history[-1].get('text', '')) == user_text:
            history = history[:-1]
        if history:
            parts.append("## Conversation history")
            for t in history:
                who = source if t.get('direction') == 'in' else self.character_name
                parts.append(f"{who}: {t.get('text', '')}")
            parts.append("")
        parts.append("## Current user input")
        parts.append(user_text)
        parts.append("")
        parts.append("## Working log")
        if not log:
            parts.append("(empty — emit your first action)")
        else:
            for label, content in log:
                parts.append(f"{label}:\n{content}")
        parts.append("")
        parts.append("Emit next action:")
        return [
            {'role': 'system', 'content': self._build_react_system_prompt(source, orientation, recall=recall, concerns=concerns)},
            {'role': 'user', 'content': "\n".join(parts)},
        ]

    def _run_react_loop(self, source: str, user_text: str, orientation: str,
                        recall: Optional[List[Tuple[str, str]]] = None,
                        concerns: Optional[List[Tuple[str, str]]] = None
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
        for i in range(REACT_MAX_ITERS):
            pre_log_len = len(log)
            prompt = self._build_react_prompt(source, user_text, orientation, log, recall=recall, concerns=concerns)
            sys_msg = prompt[0]['content']
            usr_msg = prompt[1]['content']
            try:
                raw = self.backend.chat(prompt, max_tokens=2048, temperature=0.7,
                                        cot_profile='none')
            except Exception as e:
                logger.error(f"[{self.character_name}] ReAct iter {i+1} LLM failed: {e}")
                iters.append({'system': sys_msg, 'user': usr_msg,
                              'raw': f'(LLM call failed: {e})', 'appended': []})
                reply = self._react_fallback_synthesis(log, str(e))
                return reply, log, iters, 'llm_error'

            iters.append({'system': sys_msg, 'user': usr_msg, 'raw': raw or '',
                          'appended': []})

            action = self._parse_react_action(raw)
            if action is None:
                logger.warning(f"[{self.character_name}] ReAct iter {i+1}: unparseable: {raw[:160]!r}")
                log.append(('NOTE', "Previous output was prose, not JSON. The user's task above is "
                            "unanswered. Do NOT apologize — emit ONE JSON action now to address it."))
                iters[-1]['appended'] = log[pre_log_len:]
                continue

            tool = action.get('tool')
            if tool == 'respond':
                text = self._resolve_react_value(action.get('text', ''), log)
                logger.info(f"[{self.character_name}] ReAct iter {i+1}: respond ({len(text)} chars)")
                # respond appends nothing to the log (loop exits); appended stays []
                return text, log, iters, 'respond'

            binding = f'$step{i+1}'
            log.append((f'ACTION {i+1}', json.dumps(action)))

            if tool == 'process_text':
                src = self._resolve_react_value(action.get('source', ''), log)
                ins = action.get('instruction', '')
                obs = self._run_process_text(src, ins) if (src and ins) else '(missing source or instruction)'
            elif tool == 'search':
                q = self._resolve_react_value(action.get('query', ''), log)
                result = self._run_web_search(q) if q else None
                obs = self._format_react_search_observation(result) if result else '(search failed or empty query)'
            elif tool == 'fetch_text':
                u = self._resolve_react_value(action.get('url', ''), log)
                obs = self._run_fetch_text(u)
            else:
                obs = f"unknown tool {tool!r}; available: process_text, search, fetch_text, respond"

            log.append((binding, obs))
            iters[-1]['appended'] = log[pre_log_len:]
            logger.info(f"[{self.character_name}] ReAct iter {i+1}: {tool} → {binding} ({len(obs)} chars)")

        logger.warning(f"[{self.character_name}] ReAct hit max iters ({REACT_MAX_ITERS})")
        reply = self._react_fallback_synthesis(log)
        return reply, log, iters, 'max_iters'

    def _write_react_trace(self, source: str, user_text: str,
                           log: List[Tuple[str, str]],
                           iters: List[Dict[str, Any]],
                           final_response: str,
                           exit_reason: str,
                           recall: Optional[List[Tuple[str, str]]] = None) -> None:
        """Append one ReAct session to logs/chat_trace_{character}.txt.

        Captures only the reasoning loop — recall context, system prompt,
        initial context, per-iteration emissions/appends, final response.
        Post-turn side effects (discourse update, reflection / memories
        written) are NOT in the trace; they're side outputs of the turn,
        not part of the response-generation context.

        Called immediately after the ReAct loop completes (before the slow
        post-turn LLM calls) so format_prompt has access to the trace
        without waiting on reflection.

        format_prompt.py 'Load Chat Trace' reads this file by section.
        """
        try:
            from pathlib import Path
            log_dir = Path(__file__).resolve().parent.parent / 'logs'
            log_dir.mkdir(exist_ok=True)
            trace_path = log_dir / f'chat_trace_{self.character_name}.txt'
            ts = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
            sep = '=' * 80
            sub = '-' * 80
            n_iters = len(iters)
            n_recall = len(recall or [])
            lines = [
                '', sep,
                f'Chat session: {ts}',
                f'Source: {source}',
                f'User input: {user_text}',
                f'Iterations: {n_iters}',
                f'Recall hits: {n_recall}',
                f'Exit: {exit_reason}',
                f'Final response length: {len(final_response)} chars',
                sep, '',
            ]

            if recall:
                lines.append(sub)
                lines.append('>>> RECALL HITS (auto-RAG over memories collection)')
                lines.append(sub)
                for text, cat in recall:
                    lines.append(f'[{cat}] {text}')
                lines.append('')

            if iters:
                first = iters[0]
                lines.append(sub)
                lines.append('>>> SYSTEM PROMPT (sent verbatim every iteration)')
                lines.append(sub)
                lines.append(first.get('system', ''))
                lines.append('')
                lines.append(sub)
                lines.append('>>> INITIAL CONTEXT (iter-1 user message; working log empty)')
                lines.append(sub)
                lines.append(first.get('user', ''))
                lines.append('')

                for idx, it in enumerate(iters):
                    lines.append(sub)
                    lines.append(f'>>> ITERATION {idx + 1} — MODEL RAW EMISSION (thought + tool call)')
                    lines.append(sub)
                    lines.append(it.get('raw', ''))
                    lines.append('')

                    appended = it.get('appended', []) or []
                    lines.append(sub)
                    lines.append(f'>>> ITERATION {idx + 1} — APPENDED TO CONTEXT FOR NEXT ITER')
                    lines.append(sub)
                    if not appended:
                        # respond exit, or LLM-error before any append
                        if exit_reason == 'respond' and idx == n_iters - 1:
                            lines.append('(respond — loop exits, nothing appended)')
                        elif exit_reason == 'llm_error' and idx == n_iters - 1:
                            lines.append('(LLM call failed — nothing appended)')
                        else:
                            lines.append('(nothing appended)')
                    else:
                        for label, content in appended:
                            lines.append(f'{label}:')
                            lines.append(str(content))
                            lines.append('')
                    lines.append('')

            lines.append(sub)
            lines.append('>>> FINAL RESPONSE TO USER')
            lines.append(sub)
            lines.append(final_response)
            lines.append('')
            lines.append(sep)
            with open(trace_path, 'a', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
        except Exception as e:
            logger.warning(f"[{self.character_name}] react trace write failed: {e}")

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

    def _process_user_turn(self, source: str, text: str, close: bool) -> None:
        # Each turn drives a ReAct loop: process_text / search / respond.
        # The prior web:/search: prefix path is gone — the model decides
        # when to search via the search tool. Only the final respond text
        # enters conversation history / discourse / companion. The ReAct
        # scratchpad is per-turn and not persisted.
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
        # also reinforce engagement) + top-N by current weight (always-on
        # budget). Rendered as a separate block above the memories block.
        concerns = self._get_concerns_for_prompt(text)

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

        self.store.record_outgoing(source, reply, act_type='say', close=close)
        self._publish_say(reply)
        logger.info(f"[{self.character_name}] -> {source}: {reply!r}")

        # Trace write goes immediately after publish, before any slow
        # post-turn LLM work, so format_prompt can read the reasoning
        # trace without waiting on discourse/reflection. Skipped if we
        # never entered the ReAct loop (pre-loop crash above).
        if iters:
            self._write_react_trace(
                source, text, log, iters, reply, exit_reason, recall=recall)

        # Post-turn work (discourse + reflection + close + persist) is
        # ~5-15s of LLM calls and conceptually a side effect of the turn.
        # Submitted to a single-worker executor so the main thread can
        # return to the inbox immediately. The single worker keeps post-
        # turn jobs ordered relative to the turns they summarize.
        try:
            self._post_turn_executor.submit(self._post_turn_work, source, close)
        except RuntimeError:
            # Executor already shut down (process tearing down).
            # Fall back to synchronous so nothing is silently dropped.
            self._post_turn_work(source, close)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._open_zenoh()

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
