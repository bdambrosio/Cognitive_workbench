"""Per-turn ceiling on rendered conversation history.

history_limit bounds the prompt by turn count, which says nothing about
size. On 2026-08-15 a turn was rejected at 57,145 prompt + 8,192 reserved
output against a 65,336 window — over by one token — because a session
with one large paste reached the ceiling well before its twentieth turn.

The rejected alternative was a global token budget with oldest-first
eviction: one large recent turn would evict everything behind it, leaving
a single turn of context. Capping each contributor keeps the worst case at
history_limit * cap and lets no turn crowd out its neighbours.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.prompts import _cap_turn, _HISTORY_TURN_MAX_CHARS  # noqa: E402


def test_ordinary_turns_pass_through_untouched():
    """Median real turn is 95 chars; 99% are under 2,718. The cap must be
    invisible for essentially all traffic."""
    for text in ('', 'ok', 'Hi Jill - I need to remember that I need yogurt',
                 'x' * 95, 'x' * 2718):
        assert _cap_turn(text) == text


def test_turn_at_exactly_the_cap_is_untouched():
    text = 'x' * _HISTORY_TURN_MAX_CHARS
    assert _cap_turn(text) == text


def test_oversized_turn_is_truncated_and_says_so():
    text = 'y' * 17630          # the largest real turn observed
    out = _cap_turn(text)
    assert len(out) < len(text)
    assert out.startswith('y' * 100)
    assert 'truncated' in out
    assert '17630' in out, 'must report the original size'


def test_truncation_points_at_the_pull_path():
    """Elided text is not lost — conversation.txt is on disk and recall
    greps it. The marker has to say so, or the agent concludes the
    material does not exist."""
    out = _cap_turn('z' * 9000)
    assert 'recall' in out


def test_head_is_kept_not_the_tail():
    """A turn's opening carries its intent; the tail is what recall is
    for. Keeping the tail instead would strip the request and leave the
    trailing detail."""
    text = 'INTENT-MARKER ' + ('q' * 9000) + ' TAIL-MARKER'
    out = _cap_turn(text)
    assert 'INTENT-MARKER' in out
    assert 'TAIL-MARKER' not in out


def test_worst_case_prompt_is_bounded():
    """The property that makes this a fix: 20 turns can no longer exceed
    20 * cap, whatever the underlying text."""
    turns = ['w' * 200_000 for _ in range(20)]
    total = sum(len(_cap_turn(t)) for t in turns)
    assert total < 20 * (_HISTORY_TURN_MAX_CHARS + 200)
