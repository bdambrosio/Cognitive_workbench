#!/usr/bin/env python3
"""
Concern triage — bridges active concerns to task proposals.

Two nomination paths feed a single triage queue:
  1. Activation monitor: sustained high activation (slow burn)
  2. Orient-stage nomination: strong bump on a numerically cold concern (novel event)

A periodic LLM triage call processes candidates against existing tasks and decides:
  create_task | attach_to_task | defer | dismiss
"""

import json
import logging
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)

# ── Configuration ────────────────────────────────────────────────────────────

ACTIVATION_THRESHOLD = 0.55          # Concern activation level to auto-nominate
TREND_REQUIREMENT = 'rising'         # Must be rising (not just high) for auto-nomination
ORIENT_BUMP_THRESHOLD = 'strong'     # Orient bump level that bypasses activation filter
ORIENT_COLD_CEILING = 0.35           # Concern must be below this to count as "cold"
TRIAGE_COOLDOWN_SECS = 120.0         # Min seconds between triage LLM calls
DEFER_TICKS = 30                     # How many idle ticks a deferred concern is suppressed
MAX_CANDIDATES = 6                   # Cap candidates per triage call


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class TriageCandidate:
    """A concern nominated for triage consideration."""
    concern_id: str
    concern_label: str
    concern_description: str
    source: str                      # 'activation' | 'orient_bypass' | 'seed_unserviced'
    activation: float
    seeded: bool = False
    nominated_at: float = field(default_factory=time.monotonic)


@dataclass
class TriageDecision:
    """Output of the triage LLM call for one candidate."""
    concern_id: str
    action: str                      # 'create_task' | 'attach_to_task' | 'defer' | 'dismiss'
    task_intention: str = ''         # For create_task: what the task should accomplish
    existing_task_id: str = ''       # For attach_to_task: which task to link to
    reason: str = ''


# ── Triage prompt ────────────────────────────────────────────────────────────

TRIAGE_SYSTEM_PROMPT = """\
You are a triage module for an autonomous cognitive agent. Your job is to decide \
whether active concerns warrant creating new tasks, attaching to existing tasks, \
or being deferred/dismissed.

You will receive:
- CANDIDATES: concerns that have been nominated (high activation or event-triggered)
- EXISTING_TASKS: tasks already proposed, establishing, or active — with their intentions, linked concerns, cycle_state (idle/running), and last_executed timestamp
- CONTEXT: brief summary of current agent state

For each candidate, decide ONE action:
- create_task: This concern needs a new task. Provide a clear task_intention.
- attach_to_task: This concern is already being addressed by an existing task. Provide the task_wip_id.
- defer: Not actionable right now, but may become so. The concern will be re-evaluated later.
- dismiss: False alarm — this concern doesn't actually warrant a task.

Respond with a JSON array. Each element:
{"concern_id": "...", "action": "create_task|attach_to_task|defer|dismiss", \
"task_intention": "...", "existing_task_id": "...", "reason": "short explanation"}

Rules:
- Prefer attaching to existing tasks over creating duplicates.
- Task intentions should be concrete and actionable, not vague aspirations.
- Defer is fine — not everything needs immediate action.
- Keep reasons to one sentence.
"""


# ── Main class ───────────────────────────────────────────────────────────────

class ConcernTriage:
    """Manages concern nomination, deduplication, and LLM-backed triage."""

    def __init__(self):
        self._candidates: List[TriageCandidate] = []
        self._deferred: Dict[str, int] = {}          # concern_id → remaining defer ticks
        self._defer_streak: Dict[str, int] = {}      # concern_id → consecutive defer count
        self._last_triage_time: float = 0.0
        self._triage_tick_count: int = 0

    # ── Nomination ───────────────────────────────────────────────────────

    def nominate_from_activation(
        self,
        concern_id: str,
        concern_label: str,
        concern_description: str,
        activation: float,
        trend: str,
    ):
        """Nominate a concern based on sustained high activation.

        Called during idle ticks. Only nominates if activation exceeds threshold
        and trend is rising — avoids noise from transient spikes.
        """
        if activation < ACTIVATION_THRESHOLD:
            return
        if trend != TREND_REQUIREMENT:
            return
        if self._is_deferred(concern_id):
            return
        if self._is_already_nominated(concern_id):
            return
        self._candidates.append(TriageCandidate(
            concern_id=concern_id,
            concern_label=concern_label,
            concern_description=concern_description,
            source='activation',
            activation=activation,
        ))
        logger.info(f'🎯 Triage: nominated {concern_label} (activation={activation:.2f}, trend={trend})')

    def nominate_from_orient(
        self,
        concern_id: str,
        concern_label: str,
        concern_description: str,
        activation: float,
        bump_level: str,
    ):
        """Nominate a concern via orient-stage bypass.

        Called after orient when a strong bump hits a cold concern —
        contextually important but numerically low.
        """
        if bump_level != ORIENT_BUMP_THRESHOLD:
            return
        if activation > ORIENT_COLD_CEILING:
            return  # Not cold — activation path will handle it
        if self._is_deferred(concern_id):
            return
        if self._is_already_nominated(concern_id):
            return
        self._candidates.append(TriageCandidate(
            concern_id=concern_id,
            concern_label=concern_label,
            concern_description=concern_description,
            source='orient_bypass',
            activation=activation,
        ))
        logger.info(
            f'🎯 Triage: orient-nominated {concern_label} '
            f'(activation={activation:.2f}, bump={bump_level})'
        )

    # ── Triage execution ─────────────────────────────────────────────────

    def should_run_triage(self) -> bool:
        """Check whether a triage pass should run now."""
        if not self._candidates:
            return False
        now = time.monotonic()
        return (now - self._last_triage_time) >= TRIAGE_COOLDOWN_SECS

    def tick_deferred(self):
        """Decrement deferred concern timers. Called each idle tick."""
        self._triage_tick_count += 1
        expired = [cid for cid, remaining in self._deferred.items() if remaining <= 1]
        for cid in expired:
            del self._deferred[cid]
        for cid in self._deferred:
            self._deferred[cid] -= 1

    def run_triage(
        self,
        existing_tasks: List[Dict[str, Any]],
        agent_context: str,
        llm_generate: Callable,
    ) -> List[TriageDecision]:
        """Execute LLM triage on current candidates.

        Args:
            existing_tasks: All tasks in proposed/establishing/active state,
                            each with task_wip_id, intention, status, linked_concern_id.
            agent_context: Brief summary of agent state for framing.
            llm_generate: Callable matching executive_node.llm_generate signature.

        Returns:
            List of TriageDecision objects.
        """
        self._last_triage_time = time.monotonic()

        # Cap and consume candidates
        candidates = self._candidates[:MAX_CANDIDATES]
        self._candidates = self._candidates[MAX_CANDIDATES:]

        if not candidates:
            return []

        # Format prompt
        candidates_text = json.dumps([
            {
                'concern_id': c.concern_id,
                'label': c.concern_label,
                'description': c.concern_description,
                'activation': round(c.activation, 2),
                'source': c.source,
            }
            for c in candidates
        ], indent=2)

        tasks_text = json.dumps([
            {
                'task_wip_id': t.get('task_wip_id', ''),
                'intention': t.get('intention', ''),
                'status': t.get('status', ''),
                'linked_concern_id': t.get('linked_concern_id', ''),
                'cycle_state': t.get('cycle_state', ''),
                'last_executed': t.get('last_executed', ''),
            }
            for t in existing_tasks
        ], indent=2) if existing_tasks else '[]'

        user_prompt = (
            f"CANDIDATES:\n{candidates_text}\n\n"
            f"EXISTING_TASKS:\n{tasks_text}\n\n"
            f"CONTEXT:\n{agent_context}\n\n"
            f"Decide for each candidate."
        )

        try:
            resp = llm_generate(
                [TRIAGE_SYSTEM_PROMPT, user_prompt],
                max_tokens=500,
                temperature=0.2,
                is_json=True,
            )
            resp_text = getattr(resp, 'text', '') if hasattr(resp, 'text') else str(resp)
        except Exception as e:
            logger.warning(f'Triage LLM call failed: {e}')
            # On failure, defer all candidates to avoid retrying immediately
            for c in candidates:
                self._deferred[c.concern_id] = DEFER_TICKS
            return []

        # Parse response — resp_text may already be parsed JSON (list/dict)
        # when the LLM backend uses is_json=True internally
        decisions = self._parse_triage_response(resp_text, candidates)

        # Apply defer decisions
        for d in decisions:
            if d.action == 'defer':
                self._defer_streak[d.concern_id] = self._defer_streak.get(d.concern_id, 0) + 1
                self._deferred[d.concern_id] = DEFER_TICKS
            elif d.action == 'dismiss':
                self._deferred[d.concern_id] = DEFER_TICKS * 3
                self._defer_streak.pop(d.concern_id, None)
            else:
                # create_task or attach_to_task — reset streak
                self._defer_streak.pop(d.concern_id, None)

        logger.info(
            f'🎯 Triage complete: {len(decisions)} decisions — '
            + ', '.join(
                f'{d.concern_id}→{d.action}({d.reason})' for d in decisions
            )
        )
        return decisions

    # ── Internal helpers ─────────────────────────────────────────────────

    def _is_deferred(self, concern_id: str) -> bool:
        return concern_id in self._deferred

    def _is_already_nominated(self, concern_id: str) -> bool:
        return any(c.concern_id == concern_id for c in self._candidates)

    def _parse_triage_response(
        self,
        text,
        candidates: List[TriageCandidate],
    ) -> List[TriageDecision]:
        """Parse LLM JSON array into TriageDecision list.

        text may be a string (raw LLM output) or an already-parsed list/dict
        (when the backend's is_json mode returns parsed JSON directly).
        """
        valid_actions = {'create_task', 'attach_to_task', 'defer', 'dismiss'}
        candidate_ids = {c.concern_id for c in candidates}
        decisions = []

        # Handle already-parsed JSON (list or dict)
        if isinstance(text, (list, dict)):
            items = text if isinstance(text, list) else [text]
        else:
            # Strip markdown fences from string
            t = str(text).strip()
            if t.startswith('```'):
                first_nl = t.find('\n')
                if first_nl != -1:
                    t = t[first_nl + 1:]
                if t.endswith('```'):
                    t = t[:-3]
                t = t.strip()
            try:
                items = json.loads(t)
                if not isinstance(items, list):
                    items = [items]
            except (json.JSONDecodeError, TypeError):
                logger.warning(f'Triage: failed to parse LLM response as JSON')
                for c in candidates:
                    decisions.append(TriageDecision(
                        concern_id=c.concern_id,
                        action='defer',
                        reason='triage parse failure — will retry later',
                    ))
                return decisions

        seen_ids = set()
        for item in items:
            if not isinstance(item, dict):
                continue
            cid = item.get('concern_id', '')
            action = item.get('action', '')
            if cid not in candidate_ids or cid in seen_ids:
                continue
            if action not in valid_actions:
                action = 'defer'
            seen_ids.add(cid)
            decisions.append(TriageDecision(
                concern_id=cid,
                action=action,
                task_intention=item.get('task_intention', ''),
                existing_task_id=item.get('existing_task_id', ''),
                reason=item.get('reason', ''),
            ))

        # Any candidate not in response gets deferred
        for c in candidates:
            if c.concern_id not in seen_ids:
                decisions.append(TriageDecision(
                    concern_id=c.concern_id,
                    action='defer',
                    reason='not addressed in triage response',
                ))

        return decisions

    @property
    def candidate_count(self) -> int:
        return len(self._candidates)

    @property
    def deferred_count(self) -> int:
        return len(self._deferred)

    def get_status(self) -> Dict[str, Any]:
        """Return triage state for diagnostics / UI."""
        return {
            'candidates': [
                {'concern_id': c.concern_id, 'label': c.concern_label,
                 'source': c.source, 'activation': round(c.activation, 2)}
                for c in self._candidates
            ],
            'deferred': dict(self._deferred),
            'last_triage_time': self._last_triage_time,
            'tick_count': self._triage_tick_count,
        }
