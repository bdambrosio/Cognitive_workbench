#!/usr/bin/env python3
"""
Output size guidance for LLM-based tools.

Converts a character evaluator assessment + user concerns into a target output
size directive that the planner passes to the tool step producing the primary
product of a goal.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Base token budget — calibrated so that "standard" (1.0×) yields a moderate-length
# output (~1200 tokens ≈ 900 words), "thorough" scales up, "concise" scales down.
_BASE_TOKENS = 1200

_ENGAGEMENT_HIGH_STANCES = frozenset({"committed", "concerned", "frustrated"})
_ENGAGEMENT_MID_STANCES = frozenset({"enthusiastic", "skeptical"})
_ENGAGEMENT_LOW_STANCES = frozenset({"exploratory", "ambivalent"})

_LEVEL_ORDER = {"none": 0, "low": 1, "weak": 1, "moderate": 2, "high": 3, "strong": 3}

MIN_TARGET = 300
MAX_TARGET = 4000


# ---------------------------------------------------------------------------
# Core computation
# ---------------------------------------------------------------------------

def compute_output_guidance(
    assessment: Optional[Dict[str, Any]],
    active_concerns: Optional[List[Dict[str, Any]]],
    goal_text: str,
) -> Dict[str, Any]:
    """Compute output size guidance from evaluation context.

    Args:
        assessment: CharacterEvaluator output dict (may be None).
        active_concerns: Active user concerns from UserConcernModel (may be None/empty).
        goal_text: The goal string being planned.

    Returns:
        Dict with keys: target_tokens (int), output_style (str), reasoning (str).
    """
    if not assessment:
        return _default_guidance("no assessment available")

    sal = assessment.get("salience_factors") or {}

    # ------------------------------------------------------------------
    # 1. Engagement depth  (from matching user concerns)
    # ------------------------------------------------------------------
    engagement_factor, engagement_detail = _compute_engagement(
        active_concerns or [], sal,
    )

    # ------------------------------------------------------------------
    # 2. Urgency factor
    # ------------------------------------------------------------------
    urgency_raw = sal.get("urgency", "none")
    urgency_level = _LEVEL_ORDER.get(urgency_raw, 0)
    if urgency_level >= 3:
        urgency_factor = 0.6
    elif urgency_level >= 2:
        urgency_factor = 0.8
    else:
        urgency_factor = 1.0

    # ------------------------------------------------------------------
    # 3. Novelty / familiarity factor
    # ------------------------------------------------------------------
    novelty_raw = sal.get("novelty", "none")
    novelty_level = _LEVEL_ORDER.get(novelty_raw, 0)
    # Use max touch_count from active concerns as familiarity proxy
    max_touch = max(
        (c.get("touch_count", 0) for c in (active_concerns or [])),
        default=0,
    )
    if novelty_level >= 2 and max_touch < 3:
        novelty_factor = 1.2
    elif novelty_level <= 1 and max_touch > 5:
        novelty_factor = 0.7
    else:
        novelty_factor = 1.0

    # ------------------------------------------------------------------
    # Combine
    # ------------------------------------------------------------------
    raw = _BASE_TOKENS * engagement_factor * urgency_factor * novelty_factor
    target_tokens = int(max(MIN_TARGET, min(MAX_TARGET, raw)))

    if engagement_factor >= 1.3:
        style = "thorough"
    elif engagement_factor <= 0.6:
        style = "concise"
    else:
        style = "standard"

    reasoning = (
        f"{engagement_detail}, "
        f"urgency={urgency_raw}(x{urgency_factor}), "
        f"novelty={novelty_raw}/touch={max_touch}(x{novelty_factor})"
    )

    logger.info(
        f"output_sizing: target={target_tokens}t style={style} | {reasoning}"
    )

    return {
        "target_tokens": target_tokens,
        "output_style": style,
        "reasoning": reasoning,
    }


# ---------------------------------------------------------------------------
# Engagement sub-computation
# ---------------------------------------------------------------------------

def _compute_engagement(
    concerns: List[Dict[str, Any]],
    salience_factors: Dict[str, Any],
) -> tuple[float, str]:
    """Derive engagement factor from user concerns and salience.

    Returns (factor, detail_string).
    """
    if not concerns:
        # No tracked concerns — use user_concern_relevance from evaluator
        uc_rel = salience_factors.get("user_concern_relevance", "none")
        level = _LEVEL_ORDER.get(uc_rel, 0)
        if level >= 3:
            return 1.5, f"engagement=strong(no concerns tracked, uc_rel={uc_rel})"
        elif level >= 2:
            return 1.0, f"engagement=moderate(no concerns tracked, uc_rel={uc_rel})"
        else:
            return 0.7, f"engagement=low(no concerns tracked, uc_rel={uc_rel})"

    # Find the most activated concern
    max_weight = 0.0
    dominant_stance = "exploratory"
    for c in concerns:
        w = c.get("weight", 0.0)
        if isinstance(w, (int, float)) and w > max_weight:
            max_weight = w
            dominant_stance = c.get("stance", "exploratory")

    if max_weight > 0.7 and dominant_stance in _ENGAGEMENT_HIGH_STANCES:
        factor = 1.5
    elif max_weight > 0.4 or dominant_stance in _ENGAGEMENT_MID_STANCES:
        factor = 1.0
    elif dominant_stance in _ENGAGEMENT_LOW_STANCES:
        factor = 0.5
    else:
        factor = 0.7

    detail = f"engagement: weight={max_weight:.2f} stance={dominant_stance}(x{factor})"
    return factor, detail


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_guidance(reason: str) -> Dict[str, Any]:
    """Return neutral default when signals are unavailable."""
    return {
        "target_tokens": _BASE_TOKENS,
        "output_style": "standard",
        "reasoning": f"default — {reason}",
    }


def format_guidance_for_planner(guidance: Dict[str, Any]) -> str:
    """Format guidance dict as a compact directive string for planner injection."""
    target = guidance.get("target_tokens", _BASE_TOKENS)
    style = guidance.get("output_style", "standard")
    return (
        f"OUTPUT GUIDANCE (apply target_tokens to the primary-product tool call only):\n"
        f"  target_tokens: {target}\n"
        f"  style: {style}\n"
    )
