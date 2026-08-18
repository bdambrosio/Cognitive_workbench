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


def test_fire_triage_does_not_request_reasoning(loop):
    """Tried on 2026-08-17 and reverted the same evening.

    Fire triage is the busiest LLM call in the system (87 in a four-hour
    window) and reasoning cost ~5x on it: median 0.9s -> 4.6s against
    Qwen3.8-27B, with a 20.7s outlier. What it bought could not be shown.
    The case it was added for — x-feed deferring on a 401 that had already
    been fixed — returned `defer` identically with thinking on and off,
    because there is no conflicting evidence there to weigh, only a stale
    self-authored note and nothing measured to contradict it. More
    deliberation over one bad input buys a better-argued wrong answer.

    The real repair is a measured-now signal for tool health, the same
    shape as _world_position_line. Until that exists, triage pays nothing.
    """
    nid = make_concern(loop)
    loop._reasoning_effort = 'low'
    loop.backend = KwargRecorder('{"verdict": "fire"}')

    loop._triage_fire_candidate(nid, 'c', 'do the thing')

    assert 'reasoning_effort' not in loop.backend.kwargs


def test_declining_asserts_thinking_off_rather_than_omitting():
    """A pin that works only because of how someone launched vllm is not a
    pin. Verified live 2026-08-17 against Qwen3.8-27B served with
    --default-chat-template-kwargs '{"enable_thinking": false}': nothing
    sent gave 0 reasoning chars, reasoning_effort=low alone gave 256 — the
    request kwarg beats the server default in both directions, so the
    decline has to be positive."""
    import inspect
    src = inspect.getsource(_ChatBackend.chat)
    assert "enable_thinking is False or reasoning_effort == 'none'" in src
