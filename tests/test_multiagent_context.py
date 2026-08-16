"""Per-interlocutor context assembly.

The state layer (discourse, companion, conversation history) has been
keyed by interlocutor all along. The prompt-assembly layer was not: the
reasoning-trace block is one stream across every counterpart, and until
2026-08-16 it rendered each record as `USER INPUT:` regardless of who the
turn was actually from. With User + Jack + Sentinel co-resident that is
not cosmetic — it is the prompt asserting one relationship where there
are three.

These tests pin the two rules that fix it:
  1. every trace names its origin (peer name, or an autonomous fire);
  2. records belonging to another interlocutor render as a digest, and
     the full-render budget is spent on THIS thread's records.

Rule 2's budget is the subtle half. Taking the last N of the mixed list
lets a burst on one thread push every record of the other into digest —
observed on a real two-agent trace where answering Jack yielded zero full
traces while three of the six were his.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.prompts import PromptsMixin, _REASONING_HISTORY_FULL  # noqa: E402
from chat.concerns import (ConcernsMixin,  # noqa: E402
                           _AGENT_CONCERN_FIRE_THRESHOLD,
                           _CONCERN_SUCCESSOR_MAX_DEPTH)


def _rec(seq, source, autonomous=False, text=None):
    return {'turn_seq': seq, 'source': source, 'autonomous': autonomous,
            'user_input': text or f'input {seq}',
            'raw_response': f'response {seq}'}


class _Prompts(PromptsMixin):
    character_name = 'Jill'
    config = {}
    _peers = ['Jack']
    _current_turn = {}
    resource_manager = None

    def __init__(self, records):
        self._records = records

    def _load_reasoning_records(self):
        return self._records


def _headings(block):
    return [l for l in block.splitlines() if l.startswith('### ')]


# -- rule 1: origin is always named ----------------------------------

def test_every_trace_names_its_origin():
    p = _Prompts([_rec(1, 'User'), _rec(2, 'Jack'),
                  _rec(3, 'Jill', autonomous=True)])
    block = p._get_reasoning_history_block('User')
    assert 'turn from User' in block
    assert 'turn from Jack' in block
    assert 'autonomous concern fire' in block


def test_peer_input_is_not_labelled_user_input():
    """The regression that motivated this: a Jack turn rendered as
    `USER INPUT:`, so Jack's words read as the user's."""
    p = _Prompts([_rec(1, 'Jack', text='I am heading east')])
    block = p._get_reasoning_history_block('Jack')
    assert 'INPUT FROM Jack: I am heading east' in block
    assert 'USER INPUT:' not in block


def test_autonomous_fire_is_labelled_as_a_concern_not_a_speaker():
    p = _Prompts([_rec(1, 'Jill', autonomous=True, text='check the feed')])
    block = p._get_reasoning_history_block('User')
    assert 'CONCERN INSTRUCTION: check the feed' in block
    assert 'INPUT FROM' not in block


# -- rule 2: other threads digest, budget spent on this thread --------

def test_other_thread_records_render_as_digest():
    p = _Prompts([_rec(1, 'Jack'), _rec(2, 'User')])
    block = p._get_reasoning_history_block('User')
    heads = _headings(block)
    assert heads[0].startswith('### trace #1 (turn from Jack):')  # digest
    assert heads[1] == '### trace #2 — turn from User'            # full


def test_full_budget_is_spent_on_this_thread_not_the_last_n():
    """A run of other-thread turns must not starve this thread."""
    records = ([_rec(i, 'Jack') for i in range(1, 4)]
               + [_rec(i, 'User') for i in range(4, 7)])
    jack = _headings(_Prompts(records)._get_reasoning_history_block('Jack'))
    full = [h for h in jack if ' — ' in h]
    assert len(full) == _REASONING_HISTORY_FULL
    assert all('from Jack' in h for h in full), \
        'the full renderings must be the Jack turns, not the recent User ones'


def test_own_autonomous_work_counts_as_this_thread():
    """A concern fire is my own continuation, not a third relationship —
    it must not be demoted to digest while answering anyone."""
    p = _Prompts([_rec(1, 'Jill', autonomous=True)])
    assert ' — autonomous concern fire' in _headings(
        p._get_reasoning_history_block('Jack'))[0]


def test_thread_isolation_does_not_drop_records():
    """Digest, never omit: the other exchange still happened and she may
    need to know it did."""
    records = [_rec(1, 'Jack'), _rec(2, 'User'), _rec(3, 'Sentinel')]
    for entity in ('User', 'Jack', 'Sentinel'):
        block = _Prompts(records)._get_reasoning_history_block(entity)
        assert len(_headings(block)) == 3, entity


# -- concern counterpart stamping ------------------------------------

class _Concerns(ConcernsMixin):
    _peers = ['Jack', 'Sentinel']

    def __init__(self, source):
        self._current_turn = {'source': source}


def test_counterpart_is_the_speaker_for_user_and_peers():
    assert _Concerns('User')._turn_counterpart() == 'User'
    assert _Concerns('Jack')._turn_counterpart() == 'Jack'
    assert _Concerns('Sentinel')._turn_counterpart() == 'Sentinel'


def test_counterpart_falls_back_for_sources_with_no_dialogue():
    """A sensor turn names no conversation; stamping it would leave the
    concern pointing at an entity get_recent_turns can never resume."""
    for src in ('sensor:world-presence', 'sensor:tick', 'Nobody', '', None):
        assert _Concerns(src)._turn_counterpart() == 'User'


# -- hop-budget carrier ----------------------------------------------
#
# Every other budget in the loop turns exhaustion into a continuation:
# max_iters synthesizes a remainder, `yield` carries one verbatim, both
# spawn a concern. The exchange budget alone dropped the message, so a
# two-agent activity ended at hop 7 with the sender believing it had
# spoken and the peer still waiting — and nothing anywhere recording that
# an activity had been underway.

class _FakeRM:
    def __init__(self, notes=None):
        self.notes = notes or {}

    def get_resource(self, nid):
        return self.notes.get(nid)


class _Carrier(ConcernsMixin):
    character_name = 'Jack'
    _peers = ['Jill']

    def __init__(self, current_turn=None, notes=None):
        self._current_turn = current_turn or {}
        self.resource_manager = _FakeRM(notes)
        self.added = []

    def _add_agent_concern(self, **kw):
        self.added.append(kw)
        return f'Note_{len(self.added)}'


def test_carrier_is_addressed_to_the_peer_not_the_user():
    """entity=<peer> is what makes the resumed turn load the peer thread
    rather than the User one — without it the continuation wakes up in the
    wrong conversation."""
    c = _Carrier()
    assert c._spawn_concern_from_hop_exhaustion('Jill', 'heading your way')
    assert c.added[0]['entity'] == 'Jill'


def test_carrier_fires_on_the_next_tick():
    c = _Carrier()
    c._spawn_concern_from_hop_exhaustion('Jill', 'heading your way')
    props = c.added[0]['extra_properties']
    assert props['activation'] == _AGENT_CONCERN_FIRE_THRESHOLD
    assert props['system_spawned'] is True
    assert c.added[0]['category'] == 'one_shot'


def test_carrier_names_the_tool_and_carries_the_text():
    """The action is known in advance, so it is stated rather than left to
    judgement — a stated intention has outlived an explicit persona norm
    three times."""
    c = _Carrier()
    c._spawn_concern_from_hop_exhaustion('Jill', 'the slope levels out here')
    instr = c.added[0]['instruction']
    assert 'agent-say' in instr
    assert 'the slope levels out here' in instr


def test_carrier_chain_is_bounded_by_the_successor_depth_cap():
    """Resuming refills the hop budget, so without a bound two agents
    could hand off to each other forever at tick rate."""
    at_cap = {'N': {'properties':
                    {'successor_depth': _CONCERN_SUCCESSOR_MAX_DEPTH}}}
    c = _Carrier({'autonomous_concern_id': 'N'}, at_cap)
    assert c._spawn_concern_from_hop_exhaustion('Jill', 'more') is None
    assert c.added == []


def test_carrier_deepens_the_chain_while_under_the_cap():
    under = {'N': {'properties': {'successor_depth': 1}}}
    c = _Carrier({'autonomous_concern_id': 'N'}, under)
    assert c._spawn_concern_from_hop_exhaustion('Jill', 'more')
    assert c.added[0]['extra_properties']['successor_depth'] == 2


def test_carrier_needs_both_a_peer_and_something_to_say():
    c = _Carrier()
    assert c._spawn_concern_from_hop_exhaustion('', 'text') is None
    assert c._spawn_concern_from_hop_exhaustion('Jill', '   ') is None
    assert c.added == []
