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



# Make sibling src/ modules importable when launched from src/ as cwd.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_THIS_DIR)
_TOOLS_DIR = os.path.join(_SRC_DIR, 'tools')
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from affect.publisher import AffectPublisher  # noqa: E402
from canvas.publisher import CanvasPublisher, default_key as canvas_default_key  # noqa: E402
from utils.json_utils import repair_json_string  # noqa: E402
from utils.file_utils import atomic_write_text  # noqa: E402
from utils.voice_pipeline import VOICE_MODALITY  # noqa: E402
from chat.backend import _ChatBackend  # noqa: E402
from chat.memories import (  # noqa: E402,F401 — mixin + back-compat re-exports
    MemoriesMixin, _MEMORIES_COLLECTION_NAME, _MEMORY_CATEGORIES)
from chat.threads import ThreadsMixin, _AGENT_THREADS_COLLECTION_NAME  # noqa: E402,F401
from chat.reflection import (  # noqa: E402,F401
    ReflectionMixin, _REFLECT_FRAME_OK, _CONCERN_INSTRUCTION_NARROWNESS_RULE,
    _REFLECT_STAGE6_RULE)
from chat.react import (  # noqa: E402,F401
    ReactMixin, REACT_MAX_ITERS, REACT_MAX_FORMAT_RETRIES,
    REACT_ACTION_SCHEMA, _REACT_TOOLS, _REACT_BINDING_RE)
from chat.concerns import (  # noqa: E402,F401
    ConcernsMixin, _AGENT_CONCERNS_COLLECTION_NAME,
    _USER_CONCERNS_COLLECTION_NAME, _CONCERN_STATUSES,
    _USER_CONCERN_DECAY_PER_TURN, _USER_CONCERN_BUMP_THRESHOLD,
    _USER_CONCERN_BUMP_AMOUNT, _USER_CONCERN_BUMP_MAX_PER_TURN,
    _USER_CONCERN_PRUNE_THRESHOLD, _USER_CONCERN_PROMPT_BUDGET,
    _USER_CONCERN_STALE_DAYS, _USER_CONCERN_HIGH_STRENGTH,
    _AGENT_CONCERN_FIRE_THRESHOLD,
    _AGENT_CONCERN_SERVICE_FULL, _AGENT_CONCERN_SERVICE_PARTIAL,
    _AGENT_CONCERN_PROMPT_BUDGET, _AGENT_CONCERN_BUMP_THRESHOLD,
    _AGENT_CONCERN_BUMP_AMOUNT, _AGENT_CONCERN_RHYTHM_HOURS_ALLOWED,
    _AGENT_CONCERN_DEFAULT_RHYTHM_HOURS, _CONCERN_SUCCESSOR_MAX_DEPTH,
    _CONCERN_WIP_MAX_CHARS, _CONCERN_RECURRENCE_THRESHOLD,
    _FIRE_OUTCOME_EXPIRY_TURNS, _FIRE_OUTCOME_EXPIRY_DAYS,
    _FIRE_OUTCOME_MAX_PER_REFLECTION, _FIRE_OUTCOME_DIGEST_CHARS,
    _FIRE_OUTCOME_EVIDENCE_CHARS, _FIRE_OUTCOME_JUDGED,
    _agent_concern_growth_for_elapsed, _triage_defer_cooldown_hours,
    _snap_rhythm_hours)
from chat.tools import ToolsMixin, _TOOLS_DIR as _TOOLS_DIR  # noqa: E402,F401
from chat.prompts import (  # noqa: E402,F401
    PromptsMixin, _REASONING_HISTORY_COLLECTION_NAME,
    _REASONING_HISTORY_RING_SIZE, _REASONING_HISTORY_RECENT,
    _REASONING_HISTORY_FULL, _REASONING_HISTORY_OBS_CAP)
from chat.zenoh_io import ZenohMixin  # noqa: E402


logger = logging.getLogger('chat_loop')




























# ─── ChatLoop ───────────────────────────────────────────────────────────────

class ChatLoop(MemoriesMixin, ThreadsMixin, ReflectionMixin, ReactMixin,
               ConcernsMixin, ToolsMixin, PromptsMixin, ZenohMixin):
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
        # Max tokens per ReAct action emission. Default is generous so
        # respond/display content has room. Benchmarks that hit hard questions
        # where the model dumps a runaway into an unbounded tool arg can lower
        # this (e.g. 4096) so the runaway truncates before chat()'s HTTP read
        # timeout fires — letting the parse-retry recover instead of erroring
        # the whole turn out of the loop.
        self.react_max_tokens = int((character_config.get('chat') or {}).get('react_max_tokens', 8192))
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
        # Auto-discovered tools from src/tools/. Each entry exposes
        # `react_invoke`; the catalog and dispatch path read from this
        # registry rather than hardcoded if/elif chains. Adding a tool =
        # dropping a dir under src/tools/ — no chat_loop edit required.
        self._discovered_tools: Dict[str, Dict[str, Any]] = self._discover_tools()
        self._tool_module_cache: Dict[str, Any] = {}
        if self._discovered_tools:
            logger.info(
                f"[{character_name}] discovered tools: "
                f"{sorted(self._discovered_tools.keys())}")

        # ---- Resource manager + conversation store ----
        from infospace_resource_manager import InfospaceResourceManager
        from conversation_store import ConversationStore
        world_config = character_config.get('world_config') or {}
        # Per-agent scoping guarantee: each ChatLoop gets its OWN manager,
        # and agent_name= makes persistence, FAISS indexes, and the
        # named_collections registry per-character on disk
        # (scenarios/<world>/resources/<agent>/). Named collections like
        # "memories" or "agent_concerns" can therefore never collide
        # across characters sharing a world.
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
        # Per-turn cache of pending autonomous fires being surfaced this
        # turn (fire digest). Set at user-turn entry, read by
        # _build_system_prompt; empty on autonomous turns.
        self._pending_fire_digest: List[Dict[str, Any]] = []
        # Per-turn cache of the WIP inventory for a wip_reviewer fire
        # (seed flag). Read by _build_system_prompt; empty on every
        # other turn.
        self._wip_review_inventory: List[Tuple[str, str, float, str]] = []
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

        # Embodied idle micro-gaze for the ChatterBot head (launcher
        # --head-aliveness). Off by default; no-op if the bot is absent.
        from affect.head_aliveness import HeadAliveness
        self._head_aliveness = HeadAliveness(
            enabled=bool(character_config.get('head_aliveness_enabled', False)))

        # Voice sensor: Pi mic → STT → user-like turn + wake-word orient
        # (launcher --voice). Off by default; no-op if the bot is absent.
        from chat.voice_sensor import VoiceSensor
        self._voice_sensor = VoiceSensor(
            self.character_name,
            enabled=bool(character_config.get('voice_enabled', False)),
            wake_word=character_config.get('voice_wake_word'))

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



    # Recency-bias parameters for _recall. The penalty is multiplicative
    # and saturates: a brand-new memory gets factor 1.0; an ancient memory
    # asymptotes to factor (1 - _RECALL_RECENCY_MAX_PENALTY). With the
    # current values (max=0.20, tau=14d), a 100%-relevant 30-day-old
    # memory loses ~17% to age — enough that two memories tied within
    # ~0.05 similarity flip toward the newer, but not enough to demote a
    # strong topical match in favor of an irrelevant fresh one.
    _RECALL_RECENCY_MAX_PENALTY = 0.20
    _RECALL_RECENCY_TAU_DAYS = 14.0






    def _reasoning_trace_path(self) -> 'Path':
        """Path to <memory>/reasoning_trace.jsonl — one JSON record per
        ReAct turn, append-only. Line N of the file is record N
        (= turn_seq N, 1-indexed). The active-recall subagent reads
        this file directly with line-range semantics where each line is
        one parseable JSON object."""
        return self._memory_dir() / 'reasoning_trace.jsonl'





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























































    # ------------------------------------------------------------------
    # Discourse / ToM
    # ------------------------------------------------------------------

    def _get_tracker(self, other_name: str):
        from discourse import DiscourseTracker
        tracker = self._discourse_trackers.get(other_name)
        if tracker is None:
            # triage+CRUD path (docs/design_note_agreements_rag.md):
            # bounded per-segment working set, byte-identical carryover
            # for untouched items, and date-stamp aging at assembly. The
            # legacy combined call had no enforceable pruning and grew
            # the state monotonically.
            tracker = DiscourseTracker(self._llm_generate, self.character_name,
                                       other_name, use_triage_crud=True)
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









    # --- ReAct status line (CLI feedback during the loop) -----------------
    # Subsequent _emit_status calls overwrite the previous one via \r, so the
    # user sees a single line that mutates as the loop progresses
    # ("thinking…" → "using search…" → "thinking…" → "using process_text…").
    # _clear_status wipes the line before the response is published, so the
    # CLI's response print starts on a clean line.

    _STATUS_DIM = '\033[2m'
    _STATUS_RESET = '\033[0m'
    _STATUS_PAD = 80  # enough to overwrite any reasonable previous status






    _DATA_URI_RE = re.compile(r'^data:(image/[a-zA-Z0-9.+-]+);base64,(.+)$', re.DOTALL)





    # Matches any <img ... src="http://127.0.0.1:8789/proxy?url=..."> regardless
    # of attribute order or surrounding tag content. Compiled once; used by the
    # preflight to validate proxy URLs before they reach the browser, where a
    # bad URL would render as a silent broken-image icon.
    _PROXY_IMG_RE = re.compile(
        r'''<img\b[^>]*?\bsrc\s*=\s*["'](http://127\.0\.0\.1:8789/proxy\?[^"']+)["']''',
        re.IGNORECASE,
    )










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
            atomic_write_text(path, content)
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
                                  image_ref: Optional[str] = None,
                                  fire_id: Optional[str] = None) -> None:
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
            # Fire-outcome join key: lets a trajectory builder assemble
            # (trace beats, outcome) for one autonomous episode
            # (docs/fire-outcome-capture.md §3.1). Absent on user turns.
            if fire_id:
                record['fire_id'] = fire_id

            path = self._reasoning_trace_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
        except Exception as e:
            logger.warning(
                f"[{self.character_name}] _write_reasoning_history failed: {e}")






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
                           image_url: Optional[str] = None,
                           modality: Optional[str] = None,
                           fire_id: Optional[str] = None) -> None:
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
                image_url=image_url, modality=modality,
                fire_id=fire_id)
        finally:
            self._current_turn = None
            self._affect.set_mode('idle')

    def _process_user_turn_inner(self, source: str, text: str, close: bool,
                                 autonomous: bool = False,
                                 autonomous_concern_id: Optional[str] = None,
                                 image_url: Optional[str] = None,
                                 modality: Optional[str] = None,
                                 fire_id: Optional[str] = None) -> None:
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
        self._pending_fire_digest = []
        if not autonomous:
            self._decay_user_concerns_per_turn()
            self._bump_user_concerns_on_input(text)
            # Evidence bump for agent_concerns: same input, same gating.
            # Decay is service-based for agent concerns, so bump only.
            self._bump_agent_concerns_on_input(text)
            # Fire-outcome aging: each user turn widens the reaction
            # window on pending autonomous fires; records past the cap
            # resolve to 'unobserved' (docs/fire-outcome-capture.md §4).
            self._age_pending_fire_outcomes()
            # Fire digest: surface each surviving pending fire once in
            # this turn's prompt so the user gets a reaction opportunity
            # inside the judgment window. Rendered by
            # _build_system_prompt; empty on autonomous turns (a fire
            # shouldn't be prompted to talk about other fires).
            self._pending_fire_digest = self._take_unsurfaced_pending_fires()

        # WIP-review inventory: only a wip_reviewer concern's own fire
        # sees the WIP accumulated across the other concerns. Failure
        # here degrades to an inventory-less review, not a lost turn.
        self._wip_review_inventory = []
        if autonomous and autonomous_concern_id:
            try:
                rev_note = self.resource_manager.get_resource(
                    autonomous_concern_id) or {}
                if (rev_note.get('properties') or {}).get('wip_reviewer'):
                    self._wip_review_inventory = self._collect_concern_wip(
                        exclude_id=autonomous_concern_id)
            except Exception as e:
                logger.warning(
                    f"[{self.character_name}] WIP-review inventory failed: {e}")

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
            # Speak the reply only when the turn arrived by voice — keyed on the
            # turn's modality, not the speaker's identity, so an attributed voice
            # turn (a resolved name) still gets spoken (cw-voice-sensor-plan.md §10).
            speak = (not autonomous) and modality == VOICE_MODALITY
            self._publish_say(reply, speak=speak)
        self._last_reply_at = datetime.now(timezone.utc).isoformat()
        logger.info(f"[{self.character_name}] -> {source} ({act_type}): {reply!r}")

        # Service the fired agent_concern AFTER successful reply
        # publication. Decrements activation per exit_reason and stamps
        # last_fired_at — the autonomous run has now actually executed.
        if autonomous and autonomous_concern_id:
            self._service_agent_concern(autonomous_concern_id, exit_reason)

        # Fire-outcome capture (phase 1): register this fire for outcome
        # judgment by a later user-turn's reflection stage 6
        # (docs/fire-outcome-capture.md §4). Silent fires resolve
        # immediately as 'unobservable' and never enter the registry.
        # Only real executions register — a crashed loop has no act to
        # judge. _register_fire_outcome is itself failure-tolerant.
        if (autonomous and autonomous_concern_id and fire_id
                and exit_reason in ('respond', 'max_iters')):
            self._register_fire_outcome(
                fire_id, autonomous_concern_id, exit_reason, reply,
                intentionally_silent)

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
                autonomous=autonomous, image_ref=image_ref, fire_id=fire_id)

        # Post-turn work (discourse + reflection + close + persist) is
        # ~5-15s of LLM calls and conceptually a side effect of the turn.
        # Skipped on autonomous turns — there's no user side to update
        # discourse/companion from, and reflection over a Jill-only
        # monologue would pollute the memory store. We still need a
        # disk persist so last_acted_at survives a restart.
        if autonomous:
            # WIP continuity: rewrite the root concern's running summary
            # from this fire's outcome. One LLM call, on the post-turn
            # executor so the inbox loop isn't blocked. Only for fires
            # that actually ran ReAct to a meaningful exit.
            if autonomous_concern_id and exit_reason in ('respond', 'max_iters'):
                try:
                    self._post_turn_executor.submit(
                        self._update_concern_wip, autonomous_concern_id,
                        text, log, reply, exit_reason)
                except RuntimeError:
                    # Executor already shut down (process tearing down).
                    # Fall back to synchronous so nothing is silently dropped.
                    self._update_concern_wip(
                        autonomous_concern_id, text, log, reply, exit_reason)
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
    # The growth pass and the fire-check are deliberately LLM-free —
    # pure activation arithmetic — so this path stays cheap on ticks
    # where nothing is due. Once a concern crosses threshold, one triage
    # LLM call may run per candidate (_triage_fire_candidate) to judge
    # whether acting now is warranted; its 'defer' verdict is cached on
    # the note and aged (rhythm-scaled cooldown, cleared early by an
    # evidence bump), so an over-threshold-but-deferred concern does NOT
    # re-cost an LLM call every tick. Keep it that way: no LLM calls on
    # the idle-tick path, at most one cached-and-aged call per due
    # concern on the post-threshold path.
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
        # Triage gate: per-candidate judgment (cached + aged) between
        # threshold crossing and ReAct dispatch. 'defer' and 'reset'
        # candidates drop out here; only 'fire' verdicts proceed.
        to_run = [c for c in fired
                  if self._triage_fire_candidate(*c) == 'fire']
        if not to_run:
            return
        # Cap dispatch; the rest stay due (last_acted_at unchanged) and
        # surface on the next tick.
        deferred = to_run[self._AUTONOMOUS_FIRE_CAP:]
        to_run = to_run[:self._AUTONOMOUS_FIRE_CAP]

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
            # fire_id: minted at dispatch, stamped into this fire event,
            # the react-trace record, and the pending-outcome record — the
            # join key that lets a trajectory builder assemble (trace
            # beats, outcome) for one episode (docs/fire-outcome-capture.md).
            fire_id = str(uuid.uuid4())
            outcome: Dict[str, Any] = {
                'event': 'fire',
                'fire_id': fire_id,
                'concern_id': nid,
                'concern_text': text,
                'instruction': instruction,
                'started_at': started_at.isoformat(),
            }
            # Tag self-extension fires so autonomy_review can score the
            # capability-proposal stream distinctly (Phase 2a).
            fired_note = self.resource_manager.get_resource(nid)
            if fired_note and (fired_note.get('properties') or {}).get('self_extension'):
                outcome['kind'] = 'capability_proposal'
            # Imperative wrapper. Without it, instruction bodies written
            # as reference docs ("InfluxDB lives at...") get acknowledged
            # rather than executed. The framing forces "act now" and
            # supplies the firing mode for instructions that branch on it.
            # WIP from earlier fires (kept on the root of a successor
            # chain) rides along so consecutive fires don't start cold.
            root = self.resource_manager.get_resource(self._root_concern_id(nid))
            wip = str(((root or {}).get('properties') or {}).get('wip', '') or '').strip()
            wip_section = (
                f"\n\nPrior work-in-progress from earlier fires of this "
                f"concern (don't redo what's done):\n{wip}" if wip else ""
            )
            wrapped_instruction = (
                f"A concern of mine has fired: {text}\n"
                f"Mode: autonomous\n\n"
                f"Execute the following procedure now and produce the "
                f"appropriate output. If the procedure specifies silence "
                f"under some condition, stay silent.\n\n"
                f"{instruction}{wip_section}"
            )
            try:
                # source=self.character_name marks this turn as agent-
                # originated; `_process_user_turn` skips record_incoming
                # on autonomous=True so no fake user message is logged.
                # The conversation history surfaced into the prompt is
                # still the User dialogue (see _build_react_user_prefix).
                self._process_user_turn(
                    source=self.character_name, text=wrapped_instruction, close=False,
                    autonomous=True, autonomous_concern_id=nid, fire_id=fire_id)
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
                modality = msg.get('modality')
                img_tag = ' [+image]' if image_url else ''
                logger.info(f"[{self.character_name}] <- {source}: {text!r} (close={close}){img_tag}")

                try:
                    self._process_user_turn(source, text, close,
                                            image_url=image_url, modality=modality)
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

