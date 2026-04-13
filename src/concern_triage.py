#!/usr/bin/env python3
"""
Concern triage — nomination-only module.

Two nomination paths track which concerns are activated:
  1. Activation monitor: sustained high activation (slow burn)
  2. Orient-stage nomination: strong bump on a numerically cold concern (novel event)

The OODA planner reads the nominated candidates and decides whether to
submit goals, update concerns, or wait. The LLM triage call and
task-creation logic have been removed — the OODA planner handles
concern-to-goal translation directly.
"""

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List

logger = logging.getLogger(__name__)

# ── Configuration ────────────────────────────────────────────────────────────

ACTIVATION_THRESHOLD = 0.55          # Concern activation level to auto-nominate
TREND_REQUIREMENT = 'rising'         # Must be rising (not just high) for auto-nomination
ORIENT_BUMP_THRESHOLD = 'strong'     # Orient bump level that bypasses activation filter
ORIENT_COLD_CEILING = 0.35           # Concern must be below this to count as "cold"
DEFER_TICKS = 30                     # How many idle ticks a deferred concern is suppressed


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


# ── Main class ───────────────────────────────────────────────────────────────

class ConcernTriage:
    """Manages concern nomination and deduplication.

    Tracks which concerns have been activated and provides the candidate
    list to the OODA planner for strategic reasoning.
    """

    def __init__(self):
        self._candidates: List[TriageCandidate] = []
        self._deferred: Dict[str, int] = {}          # concern_id -> remaining defer ticks
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
        logger.info(f'Triage: nominated {concern_label} (activation={activation:.2f}, trend={trend})')

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
            f'Triage: orient-nominated {concern_label} '
            f'(activation={activation:.2f}, bump={bump_level})'
        )

    # ── Candidate access (for OODA planner) ─────────────────────────────

    def get_candidates(self) -> List[TriageCandidate]:
        """Return current candidates without consuming them."""
        return list(self._candidates)

    def consume_candidates(self) -> List[TriageCandidate]:
        """Return and clear current candidates."""
        candidates = self._candidates
        self._candidates = []
        return candidates

    def defer_concern(self, concern_id: str, ticks: int = DEFER_TICKS):
        """Suppress a concern from nomination for a number of ticks."""
        self._deferred[concern_id] = ticks

    # ── Tick maintenance ────────────────────────────────────────────────

    def tick_deferred(self):
        """Decrement deferred concern timers. Called each idle tick."""
        self._triage_tick_count += 1
        expired = [cid for cid, remaining in self._deferred.items() if remaining <= 1]
        for cid in expired:
            del self._deferred[cid]
        for cid in self._deferred:
            self._deferred[cid] -= 1

    # ── Internal helpers ────────────────────────────────────────────────

    def _is_deferred(self, concern_id: str) -> bool:
        return concern_id in self._deferred

    def _is_already_nominated(self, concern_id: str) -> bool:
        return any(c.concern_id == concern_id for c in self._candidates)

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
            'tick_count': self._triage_tick_count,
        }
