#!/usr/bin/env python3
"""
OODA Living State — continuously maintained record of the agent's orientation.

Updated incrementally at each OODA phase (observe/orient/act). No LLM calls;
all updates complete in microseconds. Persisted as a named Note (_ooda_state)
with debouncing.
"""

from __future__ import annotations

import json
import logging
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Sequence

logger = logging.getLogger(__name__)

NOTE_NAME = "_ooda_state"

# Trend thresholds for activation history
_TREND_RISING = 0.05
_TREND_FALLING = -0.05
_ACTIVATION_HISTORY_LEN = 5
_TRANSITIONS_MAXLEN = 12
_EPISTEMIC_MAXLEN = 5
_PERSIST_DEBOUNCE_SECS = 2.0


def _trend_label(history: Sequence[float]) -> str:
    """Compute trend from activation history: rising, falling, or stable."""
    if len(history) < 3:
        return "stable"
    mid = len(history) // 2
    early = sum(list(history)[:mid]) / mid
    late = sum(list(history)[mid:]) / (len(history) - mid)
    delta = late - early
    if delta > _TREND_RISING:
        return "rising"
    elif delta < _TREND_FALLING:
        return "falling"
    return "stable"


def _trend_arrow(trend: str) -> str:
    return {"rising": "\u2191", "falling": "\u2193", "stable": "\u2192"}.get(trend, "")


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _now_hms() -> str:
    return datetime.now().strftime("%H:%M:%S")


class OodaLivingState:
    """Living OODA orientation state, updated at each loop phase."""

    def __init__(self) -> None:
        # Orientation landscape
        self.concern_activations: List[Dict[str, Any]] = []
        # [{id, activation, trend}] sorted descending by activation
        self.user_concern_snapshot: Dict[str, int] = {}
        # {status: count} e.g. {"ongoing": 2, "dormant": 1}
        self.goal_field: List[Dict[str, Any]] = []
        # [{goal_id, label, status}]
        self.foregrounded_goal_id: Optional[str] = None

        # Last event interpretation
        self.last_event: Dict[str, Any] = {}

        # Recent transitions ring buffer
        self.transitions: deque = deque(maxlen=_TRANSITIONS_MAXLEN)

        # Epistemic boundary markers
        self.epistemic_markers: List[str] = []

        # Internal: activation history for trend computation
        self._activation_history: Dict[str, deque] = {}

        # Persistence bookkeeping
        self._dirty: bool = False
        self._last_persist_time: float = 0.0

    # ── Update methods (called from _main_loop_tick) ───────────────────

    def update_after_observe(self, event) -> None:
        """Record what arrived from the Observe phase.

        Args:
            event: EventPacket dataclass instance.
        """
        content_preview = (getattr(event, "content", "") or "")[:80]
        self.last_event = {
            "event_type": getattr(event, "event_type", ""),
            "classification": getattr(event, "classification", ""),
            "source": getattr(event, "source", ""),
            "content_preview": content_preview,
            "goal_id": getattr(event, "goal_id", None),
            "timestamp": _now_iso(),
        }
        self._dirty = True

    def update_after_orient(
        self,
        oriented,
        concern_activations: Dict[str, float],
    ) -> None:
        """Record assessment and update activation trends.

        Args:
            oriented: OrientedEvent dataclass (event + assessment).
            concern_activations: The executive_node's _character_concern_activations dict.
        """
        assessment = getattr(oriented, "assessment", None) or {}

        # Extract assessment summary
        self.last_event["assessment"] = {
            "matters": assessment.get("matters", ""),
            "action": (assessment.get("action_evaluation") or {}).get("action_choice", ""),
            "goal_relevance": (assessment.get("salience_factors") or {}).get("goal_relevance", ""),
            "urgency": (assessment.get("salience_factors") or {}).get("urgency", ""),
            "rationale": assessment.get("overall_rationale", ""),
        }

        # Update activation history and compute trends
        for cid, value in concern_activations.items():
            if cid not in self._activation_history:
                self._activation_history[cid] = deque(maxlen=_ACTIVATION_HISTORY_LEN)
            self._activation_history[cid].append(value)

        # Build sorted activation list with trends
        self.concern_activations = []
        for cid, value in sorted(
            concern_activations.items(), key=lambda x: x[1], reverse=True
        ):
            history = self._activation_history.get(cid, deque())
            trend = _trend_label(history)
            self.concern_activations.append({
                "id": cid,
                "activation": round(value, 3),
                "trend": trend,
            })

        # Extract epistemic markers from assessment notes
        notes = assessment.get("notes", "")
        if "epistemic:" in notes:
            for segment in notes.split(";"):
                segment = segment.strip()
                if segment.startswith("epistemic:"):
                    marker = segment[len("epistemic:"):].strip()
                    if marker and marker != "none":
                        self.epistemic_markers.append(marker)
                        if len(self.epistemic_markers) > _EPISTEMIC_MAXLEN:
                            self.epistemic_markers = self.epistemic_markers[-_EPISTEMIC_MAXLEN:]
                    break

        self._dirty = True

    def update_after_act(self, action) -> None:
        """Record action taken and append transition to ring buffer.

        Args:
            action: Action dataclass (type, payload, assessment).
        """
        action_type = getattr(action, "type", "unknown")
        self.last_event["action_taken"] = action_type

        # Build transition line
        ts = _now_hms()
        goal_id = self.last_event.get("goal_id") or ""
        goal_part = f" {goal_id}" if goal_id else ""

        # Top concern with trend arrow
        concern_part = ""
        if self.concern_activations:
            top = self.concern_activations[0]
            concern_part = f" | {top['id']}{_trend_arrow(top['trend'])}"

        # Resulting state hint
        result_hint = action_type
        if action_type in ("proceed_goal", "dispatch_goal", "reuse_goal"):
            result_hint = "running"
        elif action_type == "chat_response":
            result_hint = "responded"
        elif action_type == "no_action":
            result_hint = "idle"
        elif action_type == "terminate_goal":
            result_hint = "terminated"

        line = f"[{ts}] {action_type}{goal_part}{concern_part} | \u2192 {result_hint}"
        self.transitions.append(line)
        self._dirty = True

    def update_user_concerns_snapshot(self, concerns: Sequence[Dict[str, Any]]) -> None:
        """Refresh user concern status counts."""
        counts: Dict[str, int] = {}
        for c in concerns:
            status = c.get("status", "unknown")
            counts[status] = counts.get(status, 0) + 1
        self.user_concern_snapshot = counts
        self._dirty = True

    def update_goals(
        self,
        goals: Sequence[Dict[str, Any]],
        foregrounded_id: Optional[str] = None,
    ) -> None:
        """Refresh goal field snapshot."""
        self.goal_field = []
        for g in goals[:12]:
            self.goal_field.append({
                "goal_id": g.get("goal_id"),
                "label": g.get("name") or (g.get("goal_text") or "")[:80],
                "status": g.get("status", "?"),
            })
        self.foregrounded_goal_id = foregrounded_id
        self._dirty = True

    # ── Persistence ────────────────────────────────────────────────────

    def maybe_persist(self, write_named_note_fn: Callable[[str, str], Any]) -> None:
        """Save to named Note if dirty and debounce interval has elapsed."""
        if not self._dirty:
            return
        now = time.monotonic()
        if now - self._last_persist_time < _PERSIST_DEBOUNCE_SECS:
            return
        try:
            content = json.dumps(self.to_dict(), default=str)
            write_named_note_fn(NOTE_NAME, content)
            self._dirty = False
            self._last_persist_time = now
        except Exception as e:
            logger.warning(f"OodaLivingState: persist failed: {e}")

    def load(self, infospace_executor) -> bool:
        """Load state from named Note. Returns True if loaded successfully."""
        try:
            result = infospace_executor.execute_action({
                "type": "load", "target": NOTE_NAME, "out": "$_ooda_state_tmp",
            })
            if not result or result.get("status") != "success":
                return False
            rid = result.get("resource_id")
            if not rid:
                return False
            content = infospace_executor._get_content(rid)
            if not content:
                return False
            data = json.loads(content) if isinstance(content, str) else content
            if isinstance(data, dict):
                self.from_dict(data)
                return True
        except Exception as e:
            logger.info(f"OodaLivingState: no prior state to load ({e})")
        return False

    # ── Serialization ──────────────────────────────────────────────────

    def to_dict(self) -> Dict[str, Any]:
        return {
            "concern_activations": self.concern_activations,
            "user_concern_snapshot": self.user_concern_snapshot,
            "goal_field": self.goal_field,
            "foregrounded_goal_id": self.foregrounded_goal_id,
            "last_event": self.last_event,
            "transitions": list(self.transitions),
            "epistemic_markers": self.epistemic_markers,
            "_activation_history": {
                k: list(v) for k, v in self._activation_history.items()
            },
            "updated": _now_iso(),
        }

    def from_dict(self, data: Dict[str, Any]) -> None:
        self.concern_activations = data.get("concern_activations", [])
        self.user_concern_snapshot = data.get("user_concern_snapshot", {})
        self.goal_field = data.get("goal_field", [])
        self.foregrounded_goal_id = data.get("foregrounded_goal_id")
        self.last_event = data.get("last_event", {})
        self.transitions = deque(
            data.get("transitions", []), maxlen=_TRANSITIONS_MAXLEN
        )
        self.epistemic_markers = data.get("epistemic_markers", [])
        # Restore activation history
        hist = data.get("_activation_history", {})
        self._activation_history = {
            k: deque(v, maxlen=_ACTIVATION_HISTORY_LEN)
            for k, v in hist.items()
        }
        self._dirty = False
