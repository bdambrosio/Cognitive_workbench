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

        # ---- Long-term memory (cross-conversation, per-character) ----
        # A 'memories' Collection holds episodic specifics that should
        # survive past the rolling Companion summary. Auto-RAG queries it
        # at turn start; a post-turn reflection step decides what to write.
        self._memories_collection_id: Optional[str] = None
        self._init_memories()

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

    def _remember(self, text: str, entity: str = "") -> Optional[str]:
        """Persist one memory string into the memories Collection. Returns
        the new note_id, or None on failure."""
        if not self._memories_collection_id:
            return None
        text = (text or "").strip()
        if not text:
            return None
        try:
            success, note_id, err, _ = self.resource_manager.create_note(
                self.character_name, text, "text", "chat-loop", entity or "",
                "",
                {
                    "kind": "memory",
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

    def _recall(self, query: str, k: int = 3, threshold: float = 0.3) -> List[str]:
        """Semantic search over the memories Collection. Returns ranked chunk
        strings, highest score first. Empty list on miss / not yet indexed /
        any error."""
        if not self._memories_collection_id or not query:
            return []
        try:
            ok, results, err = self.resource_manager.search_collection(
                self.character_name, self._memories_collection_id, query,
                mode='semantic', limit=k, threshold=threshold)
            if not ok or not results:
                return []
            out: List[str] = []
            for r in results:
                doc = r.get('document') if isinstance(r, dict) else None
                if isinstance(doc, str) and doc.strip():
                    out.append(doc.strip())
            return out
        except Exception as e:
            logger.warning(f"[{self.character_name}] _recall failed: {e}")
            return []

    # Reflection prompt: extract durable episodic specifics from the latest
    # exchange. Companion already absorbs personality/style/mood, so the bar
    # for memory is "would NOT be recoverable from the companion model on a
    # fresh re-read" — names, places, commitments, stable preferences.
    _REFLECT_SYS = (
        "You watch chats between {character} and {entity}. Your job: extract "
        "any episodic specifics from the latest exchange that should survive "
        "into FUTURE conversations — facts that would NOT be obvious from a "
        "persona description or rolling companion summary.\n\n"
        "CAPTURE:\n"
        "- Personal facts the user shared (names, places, relationships).\n"
        "- Stable preferences expressed plainly.\n"
        "- Long-running project/work context.\n"
        "- Specific commitments or follow-ups agreed.\n\n"
        "SKIP:\n"
        "- Pleasantries, mood, conversational tone.\n"
        "- Anything already in the companion model verbatim.\n"
        "- One-off questions with no stable signal (\"what's the weather\").\n"
        "- Hypothetical / brainstorm content unless explicitly affirmed.\n\n"
        "Output ONLY a JSON array of strings, each ≤ 200 chars, third-person "
        "about {entity}. Output `[]` if nothing qualifies. No prose."
    )

    def _reflect_and_remember(self, source: str) -> List[str]:
        """Run a single reflection LLM call over the latest exchange; for any
        memory strings returned, persist them. Returns the list written (for
        trace/logging). Failure-tolerant: any error path returns []."""
        if not self._memories_collection_id:
            return []
        try:
            dialog = self._build_dialog(source, limit=4)
            if not dialog:
                return []
            convo = "\n".join(f"{t['source']}: {t['text']}" for t in dialog)
            companion = self._companion_state.get(source, '').strip()
            sys_msg = self._REFLECT_SYS.format(
                character=self.character_name, entity=source)
            user_parts = []
            if companion:
                user_parts.append(
                    "## Existing companion model (do NOT re-extract from this; "
                    "use only to avoid duplicates)\n" + companion)
            user_parts.append("## Latest exchange\n" + convo)
            user_parts.append(
                "Return JSON array now. Each string is one durable memory, "
                "third-person, ≤ 200 chars. `[]` if nothing qualifies.")
            result = self._llm_generate(
                [{'role': 'system', 'content': sys_msg},
                 {'role': 'user', 'content': "\n\n".join(user_parts)}],
                max_tokens=4096, temperature=0.3, is_json=True,
                cot_profile='none')
            if not result.success:
                return []
            payload = result.text
            memories: List[str] = []
            if isinstance(payload, list):
                memories = [str(m).strip() for m in payload if str(m).strip()]
            elif isinstance(payload, str):
                # Cloud LLM may return raw text; try a salvage parse.
                stripped = payload.strip()
                if stripped.startswith('['):
                    try:
                        parsed = json.loads(stripped)
                        if isinstance(parsed, list):
                            memories = [str(m).strip() for m in parsed if str(m).strip()]
                    except Exception:
                        return []
            if not memories:
                return []
            written: List[str] = []
            for m in memories:
                if len(m) > 240:
                    m = m[:240].rstrip()
                if self._remember(m, entity=source):
                    written.append(m)
            if written:
                logger.info(f"[{self.character_name}] remembered {len(written)} item(s) from {source}")
            return written
        except Exception as e:
            logger.warning(f"[{self.character_name}] _reflect_and_remember failed: {e}")
            return []

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

        # Resource queryables for resource_browser. Chat mode exposes only the
        # Notes/Collections surface; concerns and graph remain executive-only.
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

    def _build_system_prompt(self, source: str, orientation: str,
                             recall: Optional[List[str]] = None) -> str:
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
        if recall:
            # Episodic specifics retrieved from prior conversations. Distinct
            # from the rolling Companion summary: these are durable facts
            # (names, places, commitments) that should not decay with style.
            mem_block = "\n".join(f"- {m}" for m in recall)
            parts.append(
                f"## Recalled memories (from prior conversations with {source})\n"
                f"{mem_block}"
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
                                   recall: Optional[List[str]] = None) -> str:
        base = self._build_system_prompt(source, orientation, recall=recall)
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
                            recall: Optional[List[str]] = None) -> List[Dict[str, str]]:
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
            {'role': 'system', 'content': self._build_react_system_prompt(source, orientation, recall=recall)},
            {'role': 'user', 'content': "\n".join(parts)},
        ]

    def _run_react_loop(self, source: str, user_text: str, orientation: str,
                        recall: Optional[List[str]] = None
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
            prompt = self._build_react_prompt(source, user_text, orientation, log, recall=recall)
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
                           recall: Optional[List[str]] = None,
                           memories_written: Optional[List[str]] = None) -> None:
        """Append one ReAct session to logs/chat_trace_{character}.txt.

        Continuing-computation layout:
          - SYSTEM PROMPT (once; constant across iters, sent verbatim each call)
          - INITIAL CONTEXT (once; iter-1 user message: history + current
            input + empty working log + 'Emit next action:')
          - Per iteration k:
              MODEL RAW EMISSION — the full thought + tool call (debug)
              APPENDED TO CONTEXT FOR NEXT ITER — what the runtime carried
                  forward into the working log (parsed action + observation,
                  or NOTE for parse error, or nothing for respond/exit).
                  The diff between EMISSION and APPENDED is exactly the
                  reasoning that gets compressed away before the next iter.
          - FINAL RESPONSE TO USER

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
                for m in recall:
                    lines.append(f'- {m}')
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

            if memories_written:
                lines.append(sub)
                lines.append('>>> MEMORIES WRITTEN (post-turn reflection)')
                lines.append(sub)
                for m in memories_written:
                    lines.append(f'- {m}')
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

        log: List[Tuple[str, str]] = []
        iters: List[Dict[str, Any]] = []
        exit_reason = 'crashed'
        try:
            reply, log, iters, exit_reason = self._run_react_loop(
                source, text, orientation, recall=recall)
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

        self._update_discourse_async(source)
        # Post-turn reflection: decide whether anything from this exchange
        # should land in long-term memory. Runs after companion update so
        # the reflection prompt can see the latest companion state and
        # avoid duplicating it.
        memories_written = self._reflect_and_remember(source)

        # Trace write deferred to here so it can include memories written
        # during reflection. Skipped if we never entered the ReAct loop
        # (no iters means the crash path above caught a pre-loop error).
        if iters:
            self._write_react_trace(
                source, text, log, iters, reply, exit_reason,
                recall=recall, memories_written=memories_written)

        if close:
            self.store.close_dialog(source)

        # Autosave per turn — small write, but means we never lose a
        # completed exchange to a launcher kill or crash.
        self._persist_to_disk()

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
            # Final flush before tearing down zenoh, so SIGINT on the
            # launcher captures any in-flight discourse/ToM updates that
            # might race the autosave above.
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
