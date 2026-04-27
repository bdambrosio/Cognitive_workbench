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
    6. discourse update  → DiscourseTracker (analyze + update_tom)
"""

from __future__ import annotations

import json
import logging
import os
import queue
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


# ─── LLM backend ────────────────────────────────────────────────────────────

class _ChatBackend:
    """Thin OpenAI-compatible chat client with structured-CoT support.

    Routes by server name:
      - 'openrouter' / 'openai' → utils.llm_api.LLM (cloud clients;
        grammar attachment skipped — cloud APIs don't accept GBNF).
      - anything else (local, vllm, llama.cpp, sglang_api_server, lmstudio,
        unset) → POST {base_url}/v1/chat/completions directly. The
        SGLangAPIServer wrapper accepts both `grammar` (GBNF, forwarded
        as ebnf) and `ebnf` directly; llama-server accepts `grammar`.

    `cot_profile` selects a GBNF skeleton for the <think> block. It is
    only attached when the model gates as reasoning (auto-detected by
    name or forced via `is_reasoning=True`).
    """

    def __init__(self, server: str, model: str, base_url: str,
                 is_reasoning: Optional[bool] = None):
        self.server = (server or 'local').lower()
        self.model = model or ''
        self.base_url = (base_url or 'http://127.0.0.1:5000').rstrip('/')
        if self.base_url.endswith('/v1'):
            self.base_url = self.base_url[:-3]
        self._cloud_llm = None
        if self.server in ('openrouter', 'openai'):
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

        resp = requests.post(url, headers={'Content-Type': 'application/json'}, json=body, timeout=120)
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

        # ---- Discourse / ToM (lazy; one tracker per other-entity) ----
        self._discourse_trackers: Dict[str, Any] = {}
        self._discourse_state: Dict[str, str] = {}
        self._tom_state: Dict[str, str] = {}
        self._restore_chat_state_from_notes()

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
    # ~150-token JSON envelope); reasoning models also burn budget on the
    # constrained <think> block, so the floor must accommodate both.
    _PROFILE_TOKEN_FLOOR = {
        'triage': 768,
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
    # Persistence — discourse_state and tom_state via named Notes,
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
        """Re-populate _discourse_state and _tom_state from named notes that
        were persisted in a previous session."""
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
                elif kind == "tom_state":
                    self._tom_state[entity] = content
            if self._discourse_state or self._tom_state:
                logger.info(
                    f"[{self.character_name}] restored chat state: "
                    f"{len(self._discourse_state)} discourse, "
                    f"{len(self._tom_state)} tom entries"
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
        """Run discourse + ToM updates after a turn. Synchronous for now."""
        if not self.discourse_enabled:
            return
        try:
            dialog = self._build_dialog(entity, limit=self.history_limit)
            if not dialog:
                return
            tracker = self._get_tracker(entity)
            prev_disc = self._discourse_state.get(entity, '')
            prev_tom = self._tom_state.get(entity, '')
            # Discourse and ToM calls have 3000-token budgets and finish
            # naturally; grammar adds no value here and the byte-cap +
            # permissive-answer pattern leaks thinking into the output,
            # which then contaminates self._discourse_state /
            # self._tom_state and re-injects into chat_reply's system
            # prompt on subsequent turns. Run unconstrained.
            tracker.llm_generate = self._make_llm_callable('none')
            new_disc = tracker.analyze_segment(
                dialog, start=0, end=len(dialog) - 1,
                previous_discourse_state=prev_disc, tom=prev_tom,
            )
            if new_disc:
                self._discourse_state[entity] = str(new_disc)
                self._save_state_note('discourse_state', entity, str(new_disc))
            tracker.llm_generate = self._make_llm_callable('none')
            new_tom = tracker.update_tom_from_discourse_segment(
                dialog, character_name=entity, start=0, end=len(dialog) - 1,
                discourse_state=self._discourse_state.get(entity, ''),
                previous_tom_state=prev_tom,
            )
            if new_tom:
                self._tom_state[entity] = str(new_tom)
                self._save_state_note('tom_state', entity, str(new_tom))
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

    def _build_system_prompt(self, source: str, orientation: str) -> str:
        parts: List[str] = []
        parts.append(f"You are {self.character_name}, speaking in first person.")
        if self.persona:
            parts.append("## Persona\n" + self.persona)
        if self.capabilities:
            parts.append("## Capabilities (chat-only mode)\n" + self.capabilities)
        if self.setting:
            parts.append("## Setting\n" + self.setting)
        tom = self._tom_state.get(source, '').strip()
        if tom:
            parts.append(f"## What you currently believe about {source}\n{tom}")
        disc = self._discourse_state.get(source, '').strip()
        if disc:
            parts.append("## Outstanding discourse objects\n" + disc)
        if orientation:
            parts.append(orientation)
        parts.append(
            "Respond as a single conversational turn. Do not narrate actions, "
            "do not roleplay tools you do not have, do not emit JSON or code "
            "fences unless the user asked for them."
        )
        return "\n\n".join(parts)

    def _build_chat_messages(self, source: str, new_text: str, orientation: str) -> List[Dict[str, str]]:
        system = self._build_system_prompt(source, orientation)
        msgs: List[Dict[str, str]] = [{'role': 'system', 'content': system}]
        history = self.store.get_recent_turns(source, limit=self.history_limit, scope='all')
        # The just-recorded incoming turn is in history; drop the trailing entry
        # if it duplicates new_text to avoid sending it twice.
        if history and history[-1].get('direction') == 'in' and str(history[-1].get('text', '')) == new_text:
            history = history[:-1]
        for t in history:
            role = 'user' if t.get('direction') == 'in' else 'assistant'
            msgs.append({'role': role, 'content': str(t.get('text', ''))})
        msgs.append({'role': 'user', 'content': new_text})
        return msgs

    # ------------------------------------------------------------------
    # Per-turn handling
    # ------------------------------------------------------------------

    def _process_user_turn(self, source: str, text: str, close: bool) -> None:
        self.store.record_incoming(source, text, close=close)

        if not text:
            if close:
                self.store.close_dialog(source)
            return

        orientation = self._orientation_summary(source, text)
        messages = self._build_chat_messages(source, text, orientation)

        try:
            # Suppress thinking entirely for the main chat reply: Qwen3.6's
            # over-deliberation on multi-turn prompts caused thinking to
            # leak into the answer channel under any byte-budget grammar.
            # Conversational replies don't need deliberation; let the model
            # answer directly. With thinking off, all of max_tokens is
            # available for the answer (vs. ~50% before) so we bump it
            # modestly to comfortably cover deep multi-paragraph replies.
            reply = self.backend.chat(messages, max_tokens=1024, temperature=0.7,
                                      cot_profile='none',
                                      enable_thinking=False)
        except Exception as e:
            logger.error(f"[{self.character_name}] LLM call failed: {e}")
            reply = f"[{self.character_name}] I had trouble generating a reply: {e}"

        reply = (reply or '').strip()
        if not reply:
            reply = "(no reply)"

        self.store.record_outgoing(source, reply, act_type='say', close=close)
        self._publish_say(reply)
        logger.info(f"[{self.character_name}] -> {source}: {reply!r}")

        self._update_discourse_async(source)

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
