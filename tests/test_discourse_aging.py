"""Unit tests for discourse-state aging: date-stamp provenance,
legacy-tag grandfathering, and the stale sweep at assembly in the
triage+CRUD path."""

import os
import sys
from datetime import datetime, timedelta

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from discourse import (
    DiscourseTracker,
    _DISCOURSE_STALE_DAYS,
    _age_and_sweep_items,
    _item_stamp_date,
    _parse_current_agreements,
)

TODAY = datetime.now().date()
FRESH = (TODAY - timedelta(days=5)).isoformat()
STALE = (TODAY - timedelta(days=_DISCOURSE_STALE_DAYS + 1)).isoformat()


class StubLLM:
    """Returns queued responses (last one sticks); records prompts."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.prompts = []

    def __call__(self, messages, bindings=None, max_tokens=400,
                 temperature=0.4, stops=None, is_json=False):
        self.prompts.append(messages[0])
        text = self.responses.pop(0) if len(self.responses) > 1 else self.responses[0]
        return {"success": True, "text": text}


def make_tracker(responses):
    return DiscourseTracker(StubLLM(responses), "Jill", "User",
                            use_triage_crud=True)


PRIOR_STATE = f"""DISCOURSE STATE (segment 0-15):

CURRENT AGREEMENTS:
- fresh item kept as-is ({FRESH})
- stale item to sweep ({STALE})
- legacy item to grandfather (established earlier)

KEY DECISIONS MADE:
- Adopted: stale decision ({STALE}), resolves X
- Adopted: legacy decision (established earlier), resolves Y
"""

DIALOG = [{"source": "User", "text": "hello"},
          {"source": "Jill", "text": "hi"}]


# ── helpers ────────────────────────────────────────────────────────────

def test_item_stamp_date():
    assert _item_stamp_date(f"thing ({FRESH})") == TODAY - timedelta(days=5)
    # multiple stamps: most recent wins
    assert _item_stamp_date(f"thing ({STALE}) and ({FRESH})") == (
        TODAY - timedelta(days=5))
    assert _item_stamp_date("no stamp here (established earlier)") is None
    assert _item_stamp_date("bogus (2026-99-99)") is None


def test_age_and_sweep():
    items = [
        f"fresh ({FRESH})",
        f"stale ({STALE})",
        "legacy (established earlier)",
        "legacy new (this segment)",
        "unstamped line",
    ]
    kept = _age_and_sweep_items(items, TODAY, "test")
    assert kept == [
        f"fresh ({FRESH})",
        f"legacy ({TODAY.isoformat()})",
        f"legacy new ({TODAY.isoformat()})",
        "unstamped line",   # no stamp → never swept
    ]


def test_parse_current_agreements_accepts_date_stamps():
    items = _parse_current_agreements(PRIOR_STATE)
    assert items == [
        f"fresh item kept as-is ({FRESH})",
        f"stale item to sweep ({STALE})",
        "legacy item to grandfather (established earlier)",
    ]


# ── triage+CRUD assembly ───────────────────────────────────────────────

def test_sweep_and_grandfather_at_assembly():
    # triage flags nothing; CRUD adds nothing.
    tracker = make_tracker(["NONE\nEND_TRIAGE", "NONE\nEND_CRUD"])
    new_state = tracker.analyze_segment(
        DIALOG, 0, 1, previous_discourse_state=PRIOR_STATE)

    assert f"- fresh item kept as-is ({FRESH})" in new_state
    assert "stale item to sweep" not in new_state
    assert f"- legacy item to grandfather ({TODAY.isoformat()})" in new_state
    # decisions: stale swept, legacy grandfathered — no longer append-only
    assert "stale decision" not in new_state
    assert f"Adopted: legacy decision ({TODAY.isoformat()}), resolves Y" in new_state


def test_update_restamps_and_add_is_kept():
    crud = (f"UPDATE 1: stale item, re-affirmed with new scope ({TODAY.isoformat()})\n"
            f"ADD: brand new agreement ({TODAY.isoformat()})\n"
            f"ADD_DECISION: Adopted: new decision ({TODAY.isoformat()}), resolves Z\n"
            "END_CRUD")
    tracker = make_tracker(["2: touched by segment\nEND_TRIAGE", crud])
    new_state = tracker.analyze_segment(
        DIALOG, 0, 1, previous_discourse_state=PRIOR_STATE)

    # the stale item was flagged and UPDATEd → restamped, survives sweep
    assert f"- stale item, re-affirmed with new scope ({TODAY.isoformat()})" in new_state
    assert f"- brand new agreement ({TODAY.isoformat()})" in new_state
    assert f"Adopted: new decision ({TODAY.isoformat()}), resolves Z" in new_state


def test_crud_prompt_carries_today_stamp():
    tracker = make_tracker(["NONE\nEND_TRIAGE", "NONE\nEND_CRUD"])
    tracker.analyze_segment(DIALOG, 0, 1, previous_discourse_state=PRIOR_STATE)
    crud_prompt = tracker.llm_generate.prompts[-1]
    assert f"({TODAY.isoformat()})" in crud_prompt
    assert "{{$today}}" not in crud_prompt
