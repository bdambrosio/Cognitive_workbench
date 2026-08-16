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
from chat.concerns import ConcernsMixin  # noqa: E402


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
