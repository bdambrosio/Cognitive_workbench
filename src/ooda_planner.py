"""
OODA Planner — strategic-level incremental planner for the cognitive agent.

Phase 1: Communication ownership.
  The OODA planner owns all user-facing communication. The goal executor
  (infospace_executor) calls into this module rather than publishing
  directly to Zenoh. Default policy: passthrough (forward say/ask to user
  as-is). Future phases add strategic filtering and the full OODA planning
  loop.

Phase 2+: Action schemas, context assembly, continuous planning loop,
  sleep/consolidation, concern-to-goal translation.
"""

import json
import logging
import queue
import time
from datetime import datetime
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

_ASK_TIMEOUT_SECONDS = 300  # 5 minutes


class OodaPlanner:
    """Strategic planner that owns user-facing communication.

    In Phase 1, this is a thin mediation layer: the goal executor calls
    ``handle_goal_say`` / ``handle_goal_ask`` instead of publishing directly,
    and the default policy is passthrough.  The main loop routes user
    replies to ``route_ask_reply`` when ``awaiting_ask_response`` is set.
    """

    def __init__(
        self,
        agent_name: str,
        session,                    # Zenoh session
        action_publisher,           # Zenoh publisher for action channel
        conversation_store,         # ConversationStore instance
        executive_node,             # reference for action_counter, dialog state
    ):
        self.agent_name = agent_name
        self.session = session
        self.action_publisher = action_publisher
        self.conversation_store = conversation_store
        self.executive_node = executive_node

        # Ask state — mirrors what was on executive_node, now owned here
        self.awaiting_ask_response = False
        self._ask_response_queue: queue.Queue = queue.Queue()

    # ── User-facing communication ───────────────────────────────────────

    def say_to_user(
        self,
        text: str,
        target: str = 'User',
        close: bool = False,
    ) -> None:
        """Publish a message to the user (or another agent) via Zenoh.

        This is the single site for outbound agent communication.
        """
        if target.lower() == 'user':
            target = 'User'

        content_payload = {'source': self.agent_name, 'text': text}
        if close:
            content_payload['close'] = True
        sense_data = {
            'timestamp': datetime.now().isoformat(),
            'sequence_id': 0,
            'mode': 'text',
            'content': json.dumps(content_payload),
        }
        self.session.put(
            f"cognitive/{target}/sense_data",
            json.dumps(sense_data),
        )

        # Publish to action channel for UI display
        action_data = {
            'type': 'say',
            'action_type': 'say',
            'action_id': f'action_{self.executive_node.action_counter}',
            'timestamp': datetime.now().isoformat(),
            'text': text,
            'source': self.agent_name,
            'target': target,
            'is_text_only': True,
        }
        self.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        if hasattr(self.executive_node, 'last_say_text'):
            self.executive_node.last_say_text = text
        if self.conversation_store:
            self.conversation_store.record_outgoing(
                target=target,
                text=text,
                act_type='say',
                close=close,
            )

        logger.info(f"Say [{target}]: {text}")

        # Dialog close handling for non-User targets
        if close and target.lower() != 'user':
            try:
                if self.conversation_store:
                    self.conversation_store.close_dialog(target)
                self.executive_node._dialog_cooldowns[target] = time.time()
                self.executive_node._dialog_purposes.pop(target, None)
                logger.info(f"Say [{target}]: dialog closed (close flag)")
            except Exception as e:
                logger.warning(f"Say close_dialog failed for {target}: {e}")

    def ask_user(self, question_text: str, target: str = 'User') -> Optional[str]:
        """Publish a question and block until the user responds or timeout.

        Returns the response text, or None on timeout.
        Called from the goal executor thread; the main loop routes replies
        via ``route_ask_reply``.
        """
        # Signal BEFORE publishing to prevent race
        self.awaiting_ask_response = True
        self.executive_node.execution_paused = True
        self.executive_node._publish_execution_state()

        # Drain stale items
        while not self._ask_response_queue.empty():
            try:
                self._ask_response_queue.get_nowait()
            except queue.Empty:
                break

        # Publish question
        action_data = {
            'type': 'ask',
            'action_type': 'ask',
            'action_id': f'action_{self.executive_node.action_counter}',
            'timestamp': datetime.now().isoformat(),
            'text': str(question_text),
            'source': self.agent_name,
            'target': target,
            'is_text_only': True,
        }
        self.action_publisher.put(json.dumps(action_data))
        self.executive_node.action_counter += 1
        if self.conversation_store:
            self.conversation_store.record_outgoing(
                target=target,
                text=str(question_text),
                act_type='ask',
                close=False,
            )

        # For non-User targets, send question to their sense_data
        if target != 'User':
            sense_data = {
                'timestamp': datetime.now().isoformat(),
                'sequence_id': 0,
                'mode': 'text',
                'content': json.dumps({
                    'source': self.agent_name,
                    'text': str(question_text),
                }),
            }
            self.session.put(
                f"cognitive/{target}/sense_data",
                json.dumps(sense_data),
            )

        logger.info(
            f"Ask: '{question_text}' -> blocking for response "
            f"(timeout={_ASK_TIMEOUT_SECONDS}s)"
        )

        try:
            response_text = self._ask_response_queue.get(
                timeout=_ASK_TIMEOUT_SECONDS
            )
        except queue.Empty:
            response_text = None

        # Clear blocking state
        self.awaiting_ask_response = False
        self.executive_node.execution_paused = False
        self.executive_node._publish_execution_state()

        if response_text is None:
            logger.info("Ask: timed out or interrupted waiting for response")
        else:
            logger.info(f"Ask: received response ({len(response_text)} chars)")

        return response_text

    # ── Goal executor signal handlers (default passthrough) ─────────────

    def handle_goal_say(
        self,
        text: str,
        target: str = 'User',
        close: bool = False,
    ) -> None:
        """Handle a say signal from the goal executor.

        Phase 1: passthrough — forward to user as-is.
        Future: OODA planner can suppress, batch, or add context.
        """
        self.say_to_user(text, target=target, close=close)

    def handle_goal_ask(
        self,
        question_text: str,
        target: str = 'User',
    ) -> Optional[str]:
        """Handle an ask signal from the goal executor.

        Phase 1: passthrough — forward to user, return response.
        Future: OODA planner can answer from context or redirect.

        Blocks the calling (goal executor) thread until a response arrives.
        """
        return self.ask_user(question_text, target=target)

    # ── Main loop integration ───────────────────────────────────────────

    def route_ask_reply(self, text: str) -> None:
        """Route a user reply to the pending ask.

        Called by the main loop when it detects a User message while
        ``awaiting_ask_response`` is True.
        """
        self._ask_response_queue.put(text)
        logger.info(f"Ask reply routed: \"{text[:60]}\"")
