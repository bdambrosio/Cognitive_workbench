"""Unit tests for character orientation evaluator V2 (LLM-backed with fallback)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from character_evaluator import (
    DEFAULT_CHARACTER_CONCERNS,
    IDLE_ORIENTATION_POLICY,
    build_event_dict,
    build_orientation_summary,
    evaluate,
    _expand_llm_result,
    _content_touches_idle,
    _format_concerns_block,
    _format_user_concerns_block,
    _format_goals_block,
    EVALUATOR_SYSTEM_PROMPT,
    EVALUATOR_USER_PROMPT,
)


# ── Fallback path tests (no LLM provided) ──────────────────────────────
# These exercise the minimal rule-based fallback when llm_generate=None.

def test_fallback_user_text_triggers_inform_user():
    ev = build_event_dict("user_text", "Hello, what time is it?", "user")
    a = evaluate(ev, DEFAULT_CHARACTER_CONCERNS, [], [], "", "")
    assert a["event_type"] == "user_text"
    assert a["matters"] == "yes"
    assert a["action_evaluation"]["action_choice"] == "inform_user"
    assert a["_meta"]["source"] == "fallback"


def test_fallback_goal_prefix_formulate_new_goal():
    ev = build_event_dict(
        "goal_initiation",
        "goal: summarize the quarterly report",
        "user",
        is_goal_command=True,
    )
    a = evaluate(ev, DEFAULT_CHARACTER_CONCERNS, [], [], "", "")
    assert a["action_evaluation"]["action_choice"] == "formulate_new_goal"
    assert a["salience_factors"]["goal_relevance"] == "strong"


def test_fallback_proceed_triggers_existing_goal():
    ev = build_event_dict(
        "goal_initiation",
        "proceed goal_3",
        "user",
        is_goal_command=False,
        is_proceed_like=True,
    )
    a = evaluate(ev, DEFAULT_CHARACTER_CONCERNS, [], [], "", "")
    assert a["action_evaluation"]["action_choice"] == "trigger_existing_goal"


def test_fallback_sensor_alert():
    ev = build_event_dict(
        "sensor_event",
        "Service unreachable: api gateway timeout",
        "sensor:check-health",
        disposition="alert",
    )
    a = evaluate(ev, DEFAULT_CHARACTER_CONCERNS, [], [], "", "")
    assert a["matters"] == "yes"
    assert a["salience_factors"]["urgency"] == "high"
    assert a["action_evaluation"]["action_choice"] == "inform_user"


# ── Expansion tests (simulate LLM output) ──────────────────────────────

def test_expand_greeting_assessment():
    """Simulates what the LLM should return for 'Hi Jill. How are you functioning?'"""
    llm_output = {
        "matters": "yes",
        "concerns": ["homeostasis:strong", "attend_to_user:strong"],
        "goal_relevance": "none",
        "urgency": "none",
        "novelty": "low",
        "persistence": "none",
        "retention": "session",
        "action": "inform_user",
        "rationale": "User asking about agent operating status",
        "epistemic": "status_unverified",
    }
    a = _expand_llm_result(llm_output, "test1", "user_text", "2025-01-01T00:00:00Z",
                            DEFAULT_CHARACTER_CONCERNS, None)
    assert a["matters"] == "yes"
    assert a["salience_factors"]["character_concern_relevance"] == "strong"
    assert a["salience_factors"]["goal_relevance"] == "none"
    assert a["retention_evaluation"]["retention_choice"] == "session_context"
    assert "User asking about agent operating status" in a["overall_rationale"]
    assert "health-check data" in (a.get("notes") or "")
    assert a["_meta"]["source"] == "llm"


def test_expand_goal_command():
    llm_output = {
        "matters": "yes",
        "concerns": ["attend_to_user:moderate"],
        "goal_relevance": "strong",
        "urgency": "none",
        "novelty": "high",
        "persistence": "moderate",
        "retention": "persistent",
        "action": "formulate_new_goal",
        "rationale": "Explicit goal request for report summarization",
        "epistemic": "none",
    }
    a = _expand_llm_result(llm_output, "test2", "goal_initiation", "2025-01-01T00:00:00Z",
                            DEFAULT_CHARACTER_CONCERNS, None)
    assert a["salience_factors"]["goal_relevance"] == "strong"
    assert a["retention_evaluation"]["retention_choice"] == "persistent_context"
    assert a["action_evaluation"]["action_choice"] == "formulate_new_goal"


def test_expand_sensor_alert():
    llm_output = {
        "matters": "yes",
        "concerns": ["homeostasis:strong"],
        "goal_relevance": "none",
        "urgency": "high",
        "novelty": "high",
        "persistence": "moderate",
        "retention": "session",
        "action": "inform_user",
        "rationale": "Health check reports service down",
        "epistemic": "requires_tool",
    }
    a = _expand_llm_result(llm_output, "test3", "sensor_event", "2025-01-01T00:00:00Z",
                            DEFAULT_CHARACTER_CONCERNS, None)
    assert a["salience_factors"]["urgency"] == "high"
    assert a["salience_factors"]["character_concern_relevance"] == "strong"
    assert "tool consultation" in (a.get("notes") or "")


def test_expand_trivial_message():
    llm_output = {
        "matters": "unclear",
        "concerns": ["attend_to_user:weak"],
        "goal_relevance": "none",
        "urgency": "none",
        "novelty": "none",
        "persistence": "none",
        "retention": "ignore",
        "action": "no_action",
        "rationale": "Minimal acknowledgment, no substantive content",
        "epistemic": "none",
    }
    a = _expand_llm_result(llm_output, "test4", "user_text", "2025-01-01T00:00:00Z",
                            DEFAULT_CHARACTER_CONCERNS, None)
    assert a["matters"] == "unclear"
    assert a["retention_evaluation"]["retention_choice"] == "ignore"


def test_expand_invalid_values_clamped():
    """Invalid enum values should be clamped to safe defaults."""
    llm_output = {
        "matters": "banana",
        "concerns": ["homeostasis:extreme"],
        "goal_relevance": "mega",
        "urgency": "super",
        "novelty": "???",
        "persistence": "forever",
        "retention": "session",
        "action": "launch_missiles",
        "rationale": "test",
        "epistemic": "none",
    }
    a = _expand_llm_result(llm_output, "test5", "user_text", "2025-01-01T00:00:00Z",
                            DEFAULT_CHARACTER_CONCERNS, None)
    # Invalid matters → "yes" (safe default)
    assert a["matters"] == "yes"
    # Invalid relevance values → "none"
    assert a["salience_factors"]["goal_relevance"] == "none"
    # Invalid action → "no_action"
    assert a["action_evaluation"]["action_choice"] == "no_action"


# ── LLM integration path test (mock) ───────────────────────────────────

def test_evaluate_with_mock_llm():
    """Full evaluate() with a mock llm_generate that returns valid JSON."""
    mock_response = {
        "matters": "yes",
        "concerns": ["homeostasis:strong", "attend_to_user:moderate"],
        "goal_relevance": "none",
        "urgency": "none",
        "novelty": "low",
        "persistence": "none",
        "retention": "session",
        "action": "inform_user",
        "rationale": "User asking about agent status",
        "epistemic": "status_unverified",
    }

    class MockResult:
        success = True
        text = mock_response  # some providers return dict directly

    def mock_llm_generate(messages, max_tokens=256, temperature=0.2, is_json=True):
        return MockResult()

    ev = build_event_dict("user_text", "Hi Jill. How are you functioning today?", "user")
    a = evaluate(ev, DEFAULT_CHARACTER_CONCERNS, [], [], "", "",
                 llm_generate=mock_llm_generate)
    assert a["_meta"]["source"] == "llm"
    assert a["salience_factors"]["character_concern_relevance"] == "strong"
    assert a["salience_factors"]["goal_relevance"] == "none"


def test_evaluate_llm_failure_falls_back():
    """If the LLM call fails, evaluate() should fall back gracefully."""
    class MockResult:
        success = False
        text = ""
        error = "timeout"

    def mock_llm_generate(messages, max_tokens=256, temperature=0.2, is_json=True):
        return MockResult()

    ev = build_event_dict("user_text", "Hello there", "user")
    a = evaluate(ev, DEFAULT_CHARACTER_CONCERNS, [], [], "", "",
                 llm_generate=mock_llm_generate)
    assert a["_meta"]["source"] == "fallback"
    assert a["matters"] == "yes"


# ── Prompt formatting tests ────────────────────────────────────────────

def test_format_concerns_block():
    block = _format_concerns_block(DEFAULT_CHARACTER_CONCERNS)
    assert "homeostasis" in block
    assert "attend_to_user" in block
    assert "epistemic norm" in block.lower() or "Epistemic" in block


def test_format_user_concerns_block_empty():
    assert "(none tracked)" in _format_user_concerns_block([])


def test_format_user_concerns_block_with_data():
    uc = [{"concern_label": "solar inverter", "concern_description": "Investigating startup sag", "status": "ongoing"}]
    block = _format_user_concerns_block(uc)
    assert "solar inverter" in block
    assert "[ongoing]" in block


def test_format_goals_block_empty():
    assert "(no active goals)" in _format_goals_block([])


def test_prompt_templates_have_placeholders():
    assert "{concerns_block}" in EVALUATOR_SYSTEM_PROMPT
    assert "{event_type}" in EVALUATOR_USER_PROMPT
    assert "{content}" in EVALUATOR_USER_PROMPT


# ── Orientation summary tests ──────────────────────────────────────────

def test_orientation_summary_from_greeting_assessment():
    """Orientation block for 'How are you functioning?' should include posture + epistemic."""
    llm_output = {
        "matters": "yes",
        "concerns": ["homeostasis:strong", "attend_to_user:strong"],
        "goal_relevance": "none",
        "urgency": "none",
        "novelty": "low",
        "persistence": "none",
        "retention": "session",
        "action": "inform_user",
        "rationale": "User asking about agent operating status",
        "epistemic": "status_unverified",
    }
    assessment = _expand_llm_result(llm_output, "t1", "user_text", "2025-01-01T00:00:00Z",
                                     DEFAULT_CHARACTER_CONCERNS, None)
    summary = build_orientation_summary(assessment, "Hi Jill. How are you functioning today?")
    assert "ORIENTATION" in summary
    assert "homeostasis" in summary
    assert "attend_to_user" in summary
    assert "health-check" in summary.lower() or "diagnostics" in summary.lower()
    assert "Posture:" in summary


def test_orientation_summary_empty_when_no_assessment():
    assert build_orientation_summary(None) == ""


def test_orientation_summary_idle_hint():
    """When content asks about idle behavior, idle policy should appear."""
    llm_output = {
        "matters": "yes",
        "concerns": ["attend_to_user:moderate"],
        "goal_relevance": "none",
        "urgency": "none",
        "novelty": "moderate",
        "persistence": "none",
        "retention": "session",
        "action": "inform_user",
        "rationale": "User asking about idle behavior",
        "epistemic": "none",
    }
    assessment = _expand_llm_result(llm_output, "t2", "user_text", "2025-01-01T00:00:00Z",
                                     DEFAULT_CHARACTER_CONCERNS, None)
    summary = build_orientation_summary(assessment, "What do you do when you have nothing to do?")
    assert "Idle orientation" in summary
    assert "reviewing unresolved concerns" in summary


def test_orientation_summary_no_idle_for_normal_chat():
    """Normal conversation should NOT include idle policy."""
    llm_output = {
        "matters": "yes",
        "concerns": ["attend_to_user:moderate"],
        "goal_relevance": "none",
        "urgency": "none",
        "novelty": "low",
        "persistence": "none",
        "retention": "session",
        "action": "inform_user",
        "rationale": "User asking about weather",
        "epistemic": "none",
    }
    assessment = _expand_llm_result(llm_output, "t3", "user_text", "2025-01-01T00:00:00Z",
                                     DEFAULT_CHARACTER_CONCERNS, None)
    summary = build_orientation_summary(assessment, "What's the weather like?")
    assert "Idle orientation" not in summary


def test_content_touches_idle():
    assert _content_touches_idle("What do you do when you're not busy?")
    assert _content_touches_idle("Do you have any free time activities?")
    assert _content_touches_idle("What happens during downtime?")
    assert not _content_touches_idle("Summarize the quarterly report")
    assert not _content_touches_idle("Hi Jill, how are you?")
