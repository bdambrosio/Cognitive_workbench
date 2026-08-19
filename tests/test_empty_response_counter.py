"""The empty-response counter.

An empty response is not presumed broken — an empty respond on a peer turn is
the deliberate don't-acknowledge idiom. This covers only the generic
`_llm_generate` path, where the caller asked for text and got none, and it
separates the two causes because they need different fixes.

Motivating incident, 2026-08-18: a discourse `crud` call returned 5,018 chars
of reasoning and no content, at max_tokens 16,384 with finish=stop. Discourse
state silently did not update; the only trace was "returning prior state
unchanged". Budget was never the problem, so raising it would not have helped.
"""

import logging
import sys
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from chat.chat_loop import ChatLoop


class _Loop:
    """Just the counter, without standing a whole ChatLoop up."""
    character_name = "TestChar"
    _record_empty_response = ChatLoop._record_empty_response

    def __init__(self, finish_reason, reasoning_chars):
        self.backend = SimpleNamespace(last_finish_reason=finish_reason,
                                       last_reasoning_chars=reasoning_chars)


def test_finish_length_is_attributed_to_budget():
    lp = _Loop("length", 1300)
    lp._record_empty_response("triage", 300)
    assert lp._empty_response_counts == {"triage/budget": 1}


def test_finish_stop_is_attributed_to_the_model_stopping():
    """The real 2026-08-18 shape: plenty of budget, reasoning emitted, no
    content, clean stop. More tokens would not have helped."""
    lp = _Loop("stop", 5018)
    lp._record_empty_response(None, 16384)
    assert lp._empty_response_counts == {"(none)/stopped": 1}


def test_counts_accumulate_per_site_and_cause():
    lp = _Loop("stop", 900)
    lp._record_empty_response("triage", 16384)
    lp._record_empty_response("triage", 16384)
    lp.backend.last_finish_reason = "length"
    lp._record_empty_response("triage", 16384)
    assert lp._empty_response_counts == {"triage/stopped": 2, "triage/budget": 1}


def test_works_when_backend_exposes_nothing():
    """Cloud routes do not report finish_reason; the counter must not care."""
    lp = _Loop(None, None)
    lp._record_empty_response("none", 4096)
    assert lp._empty_response_counts == {"none/stopped": 1}


def test_it_logs_loudly(caplog):
    """The whole point is that this stops being silent."""
    lp = _Loop("stop", 5018)
    with caplog.at_level(logging.WARNING, logger="chat_loop"):
        lp._record_empty_response(None, 16384)
    assert "EMPTY RESPONSE" in caplog.text
    assert "cause=stopped" in caplog.text
    assert "reasoning_chars=5018" in caplog.text
