"""Esc in the CLI asks the agent to stop its turn (chat/zenoh_io.py). The
request is accepted only while a turn that a person's message started is in
process (Bruce and Jill, 2026-10-04): a sensor turn, an agent's message, a
concern fire or a scheduled item is not stopped by it."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.zenoh_io import ZenohMixin  # noqa: E402


class _Host(ZenohMixin):
    character_name = 'Tester'
    _interrupt_requested = False

    def __init__(self, current_turn):
        self._current_turn = current_turn
        self.replies = []

    def _reply(self, query, payload):
        self.replies.append(payload)


@pytest.mark.parametrize('current_turn, accepted', [
    ({'kind': 'user', 'source': 'User'}, True),
    ({'kind': 'user', 'source': 'Voice'}, True),
    ({'kind': 'user', 'source': 'sensor:rss-watcher'}, False),
    ({'kind': 'user', 'source': 'Claude'}, False),
    ({'kind': 'autonomous', 'source': 'Tester'}, False),
    (None, False),
])
def test_only_a_turn_a_person_started_accepts_an_interrupt(current_turn, accepted):
    host = _Host(current_turn)
    host._handle_interrupt_query(query=None)
    assert host.replies == [{'success': True, 'accepted': accepted}]
    assert host._interrupt_requested is accepted
