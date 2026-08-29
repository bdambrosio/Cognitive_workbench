"""Characterization tests for the MAIN chat ReAct loop (`_run_react_loop`).

The subagent loops were consolidated onto `chat.subagents.subagent.Subagent`;
this loop deliberately stayed separate — it carries $stepN resolution
across tools, citation tracking, affect signalling, canvas episodes, image
blocks, yield semantics, and an exit contract the concern layer branches
on. Separate means untested, and this is the most load-bearing function in
the system, so these pin its observable behavior.

Approach: `_run_react_loop` lives on ReactMixin and touches 30 attributes
of its host ChatLoop — but every one is either a plain attribute or a
method, and the heavy dependencies (resource manager, zenoh, memory dirs)
live inside the `_run_*` tool methods, not the loop. So the loop runs
against a stub host that supplies those 30 and nothing else. No ChatLoop
is constructed and no real world state is touched, per the project rule
against exercising live resource-manager state in tests.

What is pinned here above all is the RETURN CONTRACT:
`(reply, log, iters, exit_reason)` with exit_reason in
{respond, yield, llm_error, max_iters} — because `concerns.py` and
`chat_loop.py` branch on those exact strings to decide concern servicing,
successor spawning, and activation decrement.
"""

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.react import ReactMixin, REACT_MAX_ITERS, REACT_MAX_FORMAT_RETRIES  # noqa: E402


class _Affect:
    """No-op affect surface. Records calls so the tests can assert the
    loop always leaves the affect state closed out."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def _rec(*a, **k):
            self.calls.append(name)
        return _rec


class _Canvas:
    def __init__(self):
        self.turns = 0

    def new_turn(self):
        self.turns += 1

    def set_content(self, *a, **k):
        pass


class _Backend:
    """Scripted backend. `replies` are returned in order; the last one
    repeats so the loop can be driven to its iteration cap."""

    def __init__(self, replies, finish_reason='stop'):
        self.replies = list(replies)
        self.calls = []
        self.last_finish_reason = finish_reason
        self.finish_length_events = 0

    def _note_finish(self):
        if self.last_finish_reason in ('length', 'max_tokens'):
            self.finish_length_events += 1

    def chat(self, messages, **kwargs):
        self.calls.append(kwargs)
        self._note_finish()
        idx = min(len(self.calls) - 1, len(self.replies) - 1)
        return self.replies[idx]


class Host(ReactMixin):
    """Minimal ChatLoop stand-in exposing exactly what the loop touches."""

    def __init__(self, replies, finish_reason='stop', tool_result='OK: tool ran'):
        self.backend = _Backend(replies, finish_reason)
        self.character_name = 'Tester'
        self.react_max_tokens = 8192
        self._reasoning_effort = None
        self._affect = _Affect()
        self._canvas = _Canvas()
        self._step_sources = {}
        self._pending_tool_meta = None
        self._pending_tool_image = None
        self._discovered_tools = {}
        self._tool_result = tool_result
        self.status = []

    # -- prompt construction (content is irrelevant to control flow) --
    def _build_react_system_prompt(self, *a, **k):
        return 'SYSTEM'

    def _build_react_user_prefix(self, *a, **k):
        return 'USER PREFIX\n'

    # -- status / affect plumbing --
    def _emit_status(self, msg):
        self.status.append(msg)

    def _clear_status(self):
        self.status.append('(clear)')

    # -- discovered-tool registry --
    def _canonical_tool_name(self, tool):
        return tool if tool in self._discovered_tools else None

    def _dispatch_discovered_tool(self, name, action, *a, **k):
        return self._tool_result

    # -- builtin tool runners, stubbed --
    def _run_process_text(self, *a, **k):
        return self._tool_result

    def _run_remember(self, q):
        return self._tool_result

    def _run_inspect(self, q):
        return self._tool_result

    def _run_inspect_external(self, q):
        return self._tool_result

    def _run_security(self, q):
        return self._tool_result

    def _run_justify(self, *a, **k):
        return self._tool_result

    def _run_agent_say(self, *a, **k):
        return self._tool_result

    def _run_display(self, *a, **k):
        return self._tool_result

    def _get_external_repo(self):
        return None

    def _diagnose_process_text_args(self, *a, **k):
        return None


def run(host, source='user', user_text='hello', orientation=''):
    return host._run_react_loop(source, user_text, orientation)


def act(**kw):
    return json.dumps(kw)


# --------------------------------------------------------------------
# The exit contract. concerns.py and chat_loop.py branch on these exact
# strings; changing one silently changes concern lifecycle behavior.
# --------------------------------------------------------------------

def test_respond_exits_with_reply_and_respond_reason():
    host = Host([act(thought='t', tool='respond', text='the reply')])
    reply, log, iters, exit_reason = run(host)
    assert reply == 'the reply'
    assert exit_reason == 'respond'
    assert len(iters) == 1


def test_yield_exits_with_yield_reason_and_records_next_slice():
    host = Host([act(thought='t', tool='yield', next='finish the survey',
                     text='picking this up in the background')])
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'yield'
    assert host._react_yield_next == 'finish the survey'
    assert 'background' in reply


def test_yield_to_a_waiting_user_never_returns_an_empty_reply():
    """A user turn that yields with no text must still say something —
    the handoff is stated plainly rather than shipping '(no reply)'."""
    host = Host([act(thought='t', tool='yield', next='keep digging', text='')])
    reply, log, iters, exit_reason = run(host, source='user')
    assert exit_reason == 'yield'
    assert reply.strip()
    assert 'keep digging' in reply


def test_backend_failure_exits_llm_error_without_raising():
    class Boom(_Backend):
        def chat(self, messages, **kwargs):
            raise RuntimeError('backend down')

    host = Host([''])
    host.backend = Boom([''])
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'llm_error'
    assert isinstance(reply, str) and reply


def test_iteration_cap_exits_max_iters():
    """Never emitting respond runs the cap out and lands on max_iters,
    which is what spawns a successor concern on autonomous fires."""
    host = Host([act(thought='t', tool='recall', query='q')])
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'max_iters'
    assert isinstance(reply, str) and reply


# --------------------------------------------------------------------
# Format retries — the capability the subagent loops do not have.
# --------------------------------------------------------------------

def test_persistent_format_failure_bails_after_one_iterations_retries():
    """A format stumble costs a format retry, not an action iteration —
    the capability the subagent loops lack, where an unparseable emission
    consumes one of 10-12 iterations.

    But the budget is not spent per-iteration for the whole cap: once one
    iteration exhausts its retries, the loop bails straight to fallback
    synthesis rather than trying 11 more times. So a model stuck emitting
    prose costs 1 + REACT_MAX_FORMAT_RETRIES action calls plus one
    synthesis call — not 12 iterations' worth."""
    host = Host(['not json at all'])
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'max_iters'
    attempts = 1 + REACT_MAX_FORMAT_RETRIES
    assert len(host.backend.calls) == attempts + 1, host.backend.calls
    # The extra call is the fallback synthesis, on its own smaller budget.
    assert host.backend.calls[-1]['max_tokens'] == 4096
    assert host.backend.calls[0]['max_tokens'] == host.react_max_tokens


def test_recovers_when_a_retry_produces_valid_json():
    host = Host(['garbage', act(thought='t', tool='respond', text='recovered')])
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'respond'
    assert reply == 'recovered'


def test_truncation_is_distinguished_from_malformed_output():
    """finish_reason=length gets a different corrective note than plain
    bad JSON — the loop tells the model it was cut off, not that it
    emitted nonsense."""
    host = Host(['{"thought": "unterminated'], finish_reason='length')
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'max_iters'
    notes = [c for lbl, c in log if lbl == 'NOTE']
    assert notes, 'expected a corrective NOTE in the working log'
    assert any('token limit' in n for n in notes), notes

    # A TRUNCATED ATTEMPT IS RETRIED AT DOUBLE, NOT AT THE SAME WALL.
    # The corrective note tells a model that overran its `thought` to be
    # briefer. It cannot tell a model to reason less, and reasoning consuming
    # the whole budget is what produced empty content with finish=length on
    # 2026-08-28. Re-sending the same ceiling into the same wall is not a
    # retry. Doubling is capped at one step, so the budget never runs away.
    action_budgets = [c.get('max_tokens') for c in host.backend.calls
                      if c.get('max_tokens') in (8192, 16384)]
    assert action_budgets[0] == 8192, action_budgets
    assert set(action_budgets[1:]) == {16384}, action_budgets

    # And the run says so in its own record rather than only in a log. The
    # count lives on the BACKEND, which every caller shares — the subagents,
    # which hold no reference to the loop, and the fallback synthesis below.
    # A counter on the loop recorded 0 for a run whose `inspect_external`
    # subagent truncated twice, and missed the fallback call here.
    #
    # Four, not three: the three action emissions plus the fallback synthesis,
    # which this backend also truncates. Every truncated completion counts.
    assert host.backend.finish_length_events == len(host.backend.calls) == 4


# --------------------------------------------------------------------
# $stepN binding — cross-tool state the subagent loops do not carry.
# --------------------------------------------------------------------

def test_step_bindings_resolve_across_iterations():
    """A later action referencing $step1 receives the earlier tool's
    output with its OK:/EMPTY:/ERROR: prefix stripped."""
    host = Host([act(thought='t', tool='recall', query='q'),
                 act(thought='t', tool='respond', text='$step1')],
                tool_result='OK: remembered content')
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'respond'
    assert reply == 'remembered content', 'prefix should be stripped'


def test_unknown_tool_reports_and_continues():
    host = Host([act(thought='t', tool='no_such_tool')])
    reply, log, iters, exit_reason = run(host)
    assert exit_reason == 'max_iters'
    assert any('ERROR' in c for _, c in log)


# --------------------------------------------------------------------
# Invariants that must hold on every exit path.
# --------------------------------------------------------------------

@pytest.mark.parametrize('replies,expected', [
    ([act(thought='t', tool='respond', text='x')], 'respond'),
    ([act(thought='t', tool='yield', next='more', text='ok')], 'yield'),
    ([act(thought='t', tool='recall', query='q')], 'max_iters'),
])
def test_shape_of_return_is_stable_on_every_exit(replies, expected):
    host = Host(replies)
    result = run(host)
    assert isinstance(result, tuple) and len(result) == 4
    reply, log, iters, exit_reason = result
    assert isinstance(reply, str)
    assert isinstance(log, list)
    assert isinstance(iters, list)
    assert exit_reason == expected


@pytest.mark.parametrize('replies', [
    [act(thought='t', tool='respond', text='x')],
    [act(thought='t', tool='yield', next='more', text='ok')],
    [act(thought='t', tool='recall', query='q')],
])
def test_affect_loop_is_always_closed_out(replies):
    """enter_loop must be matched by exit_loop on every path, or the
    affect display is left showing a turn that already ended."""
    host = Host(replies)
    run(host)
    assert 'enter_loop' in host._affect.calls
    assert 'exit_loop' in host._affect.calls


def test_canvas_episode_opens_once_per_turn():
    host = Host([act(thought='t', tool='respond', text='x')])
    run(host)
    assert host._canvas.turns == 1
