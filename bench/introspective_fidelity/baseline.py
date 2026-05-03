"""Bench baseline — minimal chat agent for matched-backend comparison.

A traditional chat loop with conversation history only: same LLM backend
as Jill (via _ChatBackend), no concerns, no reasoning_history, no
discourse/companion trackers, no reflection, no tools, no autoRAG. The
agent the introspective-fidelity benchmark scores against to isolate
Jill's architectural contribution from the LLM's contribution.

Exposes the subset of ChatLoop's interface the runner uses
(_process_user_turn, store.get_recent_turns, snapshot attribute stubs,
cleanup hooks) so introspective_fidelity/runner.py can swap
implementations via --baseline without restructure.

The empty snapshot stubs are load-bearing: the judge's rubric scores
agent claims against ground-truth state. With concerns/reasoning_history/
discourse all empty, any baseline claim of architectural state ("I'm
tracking X as a commitment") scores as Accuracy=0 — which is the
intended behavior of the comparison.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List

# Reuse ChatLoop's backend abstraction so model wiring stays identical.
# _ChatBackend supports anthropic native, OpenAI-compatible cloud, and
# local OpenAI-compatible endpoints — same surface Jill uses.
from chat.chat_loop import _ChatBackend

logger = logging.getLogger(__name__)


class _LocalConversationStore:
    """Tiny in-memory store mimicking the subset of ConversationStore's
    interface the bench runner uses (just get_recent_turns)."""

    def __init__(self):
        self._turns: Dict[str, List[Dict[str, str]]] = {}

    def record(self, source: str, direction: str, text: str) -> None:
        self._turns.setdefault(source, []).append(
            {"direction": direction, "text": text})

    def get_recent_turns(self, source: str, limit: int = 10,
                         scope: str = "all") -> List[Dict[str, str]]:
        all_turns = self._turns.get(source, [])
        return all_turns[-limit:] if limit else all_turns


class _NullPostTurnExecutor:
    """Stub for `loop._post_turn_executor.shutdown(wait=True)` in the
    runner's cleanup. Baseline has no background work."""

    def shutdown(self, wait: bool = True) -> None:
        pass


class BaselineChatAgent:
    """Minimal chat agent: optional system prompt + history + LLM call.
    No persona-state, no reflection, no concerns, no memory recall, no
    tools.

    Maintains the surface needed by introspective_fidelity/runner.py:
      - _process_user_turn(source, text, close)
      - store.get_recent_turns(source, limit, scope)
      - _concerns_collection_id, _reasoning_history_collection_id
        (None — snapshot helpers short-circuit on these)
      - _discourse_state, _companion_state ({} — snapshot helpers
        return empty for unknown sources)
      - resource_manager (None — only accessed when collection IDs are
        non-None, which they aren't here)
      - _post_turn_executor.shutdown(wait), _persist_to_disk()
        (no-ops)
    """

    def __init__(self, llm_config: Dict[str, Any], system_prompt: str,
                 character_name: str, max_tokens: int = 1500,
                 temperature: float = 0.7):
        self.character_name = character_name
        self.system_prompt = (system_prompt or "").strip()
        self.max_tokens = max_tokens
        self.temperature = temperature

        # Same backend Jill uses, configured the same way from the
        # scenario YAML's llm_config block.
        self.backend = _ChatBackend(
            server=llm_config.get('server', 'local'),
            model=llm_config.get('model', ''),
            base_url=(llm_config.get('vllm_url')
                      or llm_config.get('base_url')
                      or 'http://127.0.0.1:5000'),
            is_reasoning=llm_config.get('is_reasoning_model'),
            api_key=llm_config.get('api_key'),
        )

        # Conversation history shared across turns (single source —
        # the bench runner uses one source consistently).
        self._history: List[Dict[str, str]] = []

        # Snapshot-surface stubs (see class docstring).
        self._concerns_collection_id = None
        self._reasoning_history_collection_id = None
        self._discourse_state: Dict[str, str] = {}
        self._companion_state: Dict[str, str] = {}
        self.resource_manager = None
        self._post_turn_executor = _NullPostTurnExecutor()
        self.store = _LocalConversationStore()

    def _process_user_turn(self, source: str, text: str,
                           close: bool = False) -> None:
        """Drive one user turn: build messages from optional system prompt
        + full prior history + current input, call the backend, append the
        reply to history, and mirror to the local store so the runner's
        _latest_reply works unchanged."""
        text = (text or "").strip()
        if not text:
            return

        msgs: List[Dict[str, str]] = []
        if self.system_prompt:
            msgs.append({"role": "system", "content": self.system_prompt})
        msgs.extend(self._history)
        msgs.append({"role": "user", "content": text})

        try:
            reply = self.backend.chat(
                msgs,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
            )
        except Exception as e:
            logger.error(f"[{self.character_name}] backend.chat failed: {e}")
            reply = f"[{self.character_name}] (backend error: {e})"

        reply = (reply or "").strip() or "(no reply)"

        self._history.append({"role": "user", "content": text})
        self._history.append({"role": "assistant", "content": reply})

        self.store.record(source, "in", text)
        self.store.record(source, "out", reply)

    def _persist_to_disk(self) -> None:
        """No-op. Baseline keeps no on-disk state; runner calls this in
        cleanup."""
        pass
