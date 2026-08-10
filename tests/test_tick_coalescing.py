"""Ticks collapse while a turn is in flight.

A character's inbox is drained by one thread, between turns only. A
20-minute turn (live 2026-08-10: a LAN sweep) therefore used to queue ~20
heartbeats and run that many back-to-back autonomy passes on drain. Safe
to collapse because activation growth is elapsed-time based, so one tick
after a gap grows exactly as much as the ticks it replaced.
"""

import json
import queue
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.zenoh_io import ZenohMixin


class _Sample:
    def __init__(self, source):
        self.payload = json.dumps({
            'mode': 'text',
            'content': json.dumps({'source': source, 'text': ''}),
        }).encode('utf-8')


class _Host(ZenohMixin):
    """Minimal stand-in: the tick path touches only these attributes."""
    def __init__(self):
        self.character_name = 'Tester'
        self._inbox = queue.Queue()
        self._tick_pending = False


def _drain(host):
    """Mimic the consumer at chat_loop.run: clear the flag on dequeue."""
    got = []
    while not host._inbox.empty():
        msg = host._inbox.get()
        if msg.get('kind') == 'tick':
            host._tick_pending = False
        got.append(msg)
    return got


def test_ticks_collapse_while_unhandled():
    host = _Host()
    for _ in range(20):
        host._on_sense_data(_Sample('sensor:tick'))
    assert host._inbox.qsize() == 1, "20 heartbeats must queue as one"
    assert _drain(host) == [{'kind': 'tick'}]


def test_a_tick_after_the_handled_one_still_arrives():
    """Collapsing must not swallow the NEXT heartbeat — the flag clears on
    dequeue, before handling, so autonomy keeps its pulse."""
    host = _Host()
    host._on_sense_data(_Sample('sensor:tick'))
    _drain(host)
    host._on_sense_data(_Sample('sensor:tick'))
    assert host._inbox.qsize() == 1


def test_user_messages_are_never_collapsed():
    """Only the heartbeat is a heartbeat. Real input queues one per
    message however busy the character is."""
    host = _Host()
    host._on_sense_data(_Sample('sensor:tick'))
    for _ in range(3):
        s = _Sample('User')
        s.payload = json.dumps({
            'mode': 'text',
            'content': json.dumps({'source': 'User', 'text': 'hello?'}),
        }).encode('utf-8')
        host._on_sense_data(s)
    kinds = [m.get('kind') for m in _drain(host)]
    assert kinds == ['tick', 'user', 'user', 'user']
