"""Who asks for a thinking channel, and who declines one.

`effective = per_call if per_call is not None else baseline` meant None
could not express "off" — it meant "defer" — so no call site could
decline a scenario baseline. The two one-word probes budget 8 tokens for
a verdict and both fail open, so under coord_search_luna's `low` or the
gpt-oss bench's `medium` they were being handed a thinking channel and
would have gone quiet rather than loud.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import pytest

import shutil
import threading
import uuid
from pathlib import Path

from chat.backend import _ChatBackend
from chat.chat_loop import ChatLoop
from infospace_resource_manager import InfospaceResourceManager


@pytest.fixture
def loop():
    world = f"pytest_scratch_{uuid.uuid4().hex[:8]}"
    mgr = InfospaceResourceManager(world, world_config={"world_name": world})
    inst = object.__new__(ChatLoop)
    inst.character_name = "Jill"
    inst.resource_manager = mgr
    inst._faiss_lock = threading.Lock()
    inst._current_turn = {}
    inst._autonomy_log_path = lambda: Path(f"/tmp/{world}.jsonl")
    yield inst
    shutil.rmtree(Path(__file__).parent.parent / "scenarios" / world,
                  ignore_errors=True)


def make_concern(loop):
    ok, nid, err, _ = loop.resource_manager.create_note(
        "Jill", "a concern", "text", "pytest", "User", "",
        {"kind": "agent_concern", "status": "active", "activation": 0.9,
         "rhythm_hours": 24, "instruction": "check", "exclude_from_index": True})
    assert ok, err
    return nid


class Recorder(_ChatBackend):
    """Captures the request body instead of posting it."""

    def __init__(self, baseline=None):
        object.__setattr__(self, 'base_url', 'http://127.0.0.1:5000')
        self.reasoning_effort = baseline
        self.bodies = []

    def _effective(self, per_call):
        """Mirror of the resolution in chat(), which is what we assert on."""
        eff = (per_call if per_call is not None else self.reasoning_effort)
        return None if eff == 'none' else eff


def test_a_call_inherits_the_scenario_baseline():
    assert Recorder('medium')._effective(None) == 'medium'


def test_a_call_can_override_the_baseline():
    assert Recorder('high')._effective('low') == 'low'


def test_none_declines_the_baseline_entirely():
    """The whole point: an 8-token probe under a `medium` scenario."""
    assert Recorder('medium')._effective('none') is None


def test_none_is_harmless_with_no_baseline():
    assert Recorder(None)._effective('none') is None


def test_nothing_is_sent_when_neither_side_asks():
    assert Recorder(None)._effective(None) is None


def test_the_validator_accepts_none_and_still_rejects_junk():
    b = Recorder(None)
    for good in ('low', 'medium', 'high', 'none'):
        assert str(good).strip().lower() in ('low', 'medium', 'high', 'none')
    with pytest.raises(RuntimeError):
        _ChatBackend.chat(b, [{'role': 'user', 'content': 'x'}],
                          reasoning_effort='enthusiastic')


# ── the probes decline, triage opts in ─────────────────────────────────

class KwargRecorder:
    """Stands in for the backend; remembers how each call site asked."""

    def __init__(self, reply):
        self.reply = reply
        self.kwargs = None

    def chat(self, messages, **kwargs):
        self.kwargs = kwargs
        return self.reply


def test_the_process_text_probe_declines_a_baseline():
    from chat.react import ReactMixin
    obj = object.__new__(ReactMixin)
    obj.character_name = 'Jill'
    obj.backend = KwargRecorder('material')

    ReactMixin._source_is_reference(obj, 'some text', 'translate it')

    assert obj.backend.kwargs['reasoning_effort'] == 'none'
    assert obj.backend.kwargs['max_tokens'] == 8


def test_the_memory_probe_declines_a_baseline():
    from chat.memories import MemoriesMixin
    obj = object.__new__(MemoriesMixin)
    obj.character_name = 'Jill'
    obj.backend = KwargRecorder('distinct')

    MemoriesMixin._memory_relation(obj, 'User is 80', 'User is 79')

    assert obj.backend.kwargs['reasoning_effort'] == 'none'
    assert obj.backend.kwargs['max_tokens'] == 8


def test_fire_triage_asks_for_low_when_the_session_has_reasoning(loop):
    nid = make_concern(loop)
    loop._reasoning_effort = 'low'
    loop.backend = KwargRecorder('{"verdict": "fire"}')

    loop._triage_fire_candidate(nid, 'c', 'do the thing')

    assert loop.backend.kwargs['reasoning_effort'] == 'low'


def test_fire_triage_never_escalates_past_low(loop):
    """A scenario declaring high means it for the work, not for a small
    JSON verdict — 11c8bc5b is what that costs."""
    nid = make_concern(loop)
    loop._reasoning_effort = 'high'
    loop.backend = KwargRecorder('{"verdict": "fire"}')

    loop._triage_fire_candidate(nid, 'c', 'do the thing')

    assert loop.backend.kwargs['reasoning_effort'] == 'low'


def test_fire_triage_survives_a_loop_without_the_attribute(loop):
    """Mixin attribute owned by ChatLoop.__init__; a partially built loop
    (every test fixture, and any future caller) must not blow up here —
    the except would swallow it into a fire-open."""
    nid = make_concern(loop)
    loop.backend = KwargRecorder('{"verdict": "defer", "reason": "x"}')

    assert loop._triage_fire_candidate(nid, 'c', 'do it') == 'defer'
    assert loop.backend.kwargs['reasoning_effort'] is None


def test_fire_triage_sends_nothing_when_reasoning_is_off(loop):
    """Gemma's template has no reasoning_effort variable, so any value at
    all switches thinking ON from a default of off."""
    nid = make_concern(loop)
    loop._reasoning_effort = None
    loop.backend = KwargRecorder('{"verdict": "fire"}')

    loop._triage_fire_candidate(nid, 'c', 'do the thing')

    assert loop.backend.kwargs['reasoning_effort'] is None
