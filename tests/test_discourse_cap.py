"""cap_discourse_state — the ceiling that keeps the discourse state from
becoming most of every prompt (see chat_loop._DISCOURSE_STATE_MAX_CHARS)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from chat.chat_loop import cap_discourse_state  # noqa: E402


def _state(n, body="agreement text that is reasonably long " * 3):
    head = "DISCOURSE STATE (segment 0-19):\n\nCURRENT AGREEMENTS:"
    return head + "\n" + "\n".join(f"- {i:03d} {body}" for i in range(n))


def test_under_limit_is_untouched():
    s = _state(3)
    assert cap_discourse_state(s, limit=10000) == s


def test_over_limit_drops_oldest_first_and_fits():
    s = _state(400)
    out = cap_discourse_state(s, limit=10000)
    assert len(out) <= 10000
    kept = [l for l in out.split("\n") if l.startswith("- ")]
    assert kept, "everything was dropped"
    # Oldest-first: the survivors are the tail of the original.
    assert kept[-1].startswith("- 399")
    assert not any(l.startswith("- 000") for l in kept)


def test_pruning_is_announced_not_silent():
    out = cap_discourse_state(_state(400), limit=10000)
    assert "older entries pruned" in out
    # The count is real, not a placeholder.
    kept = sum(1 for l in out.split("\n") if l.startswith("- "))
    assert f"[{400 - kept} older entries pruned" in out


def test_headers_survive_even_when_most_bullets_go():
    out = cap_discourse_state(_state(400), limit=1500)
    assert out.startswith("DISCOURSE STATE (segment 0-19):")
    assert "CURRENT AGREEMENTS:" in out
    assert len(out) <= 1500


def test_nothing_droppable_keeps_headers_rather_than_crashing():
    """No bullet lines means nothing the cap is allowed to remove. It must
    return the state intact rather than truncate mid-header or raise."""
    s = "DISCOURSE STATE (segment 0-1):\n\nCURRENT AGREEMENTS:\n[none]"
    out = cap_discourse_state(s, limit=10)   # limit is unreachable here
    assert out == s
