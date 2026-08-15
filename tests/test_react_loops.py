"""Characterization tests for the four ReAct-shaped loops.

These lock in *current* behavior ahead of any consolidation. They are
deliberately behavioral, not aspirational: where the loops differ today
(iteration caps, fallback wording), the test records the difference
rather than asserting what a unified loop ought to do.

Scope note: `_run_react_loop` (react.py) is not exercised end-to-end.
It is a ChatLoop method with live dependencies — resource manager, zenoh,
memory dirs — and standing project policy is to never instantiate those
against real state in a test. Its pure helpers and its published
constants are covered here; the loop body itself remains uncovered and
is the main gap a consolidation would need to close.

Every filesystem target is a tmp_path. No test touches a real world,
memory dir, or trace dir.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

import chat.subagents.security as sec           # noqa: E402
import chat.subagents.recall as rem           # noqa: E402
import chat.subagents.code_subagent as cs       # noqa: E402
from chat.react import ReactMixin, REACT_MAX_ITERS  # noqa: E402


class FakeBackend:
    """Scripted _ChatBackend stand-in.

    Returns `replies` in order; repeats the last one once exhausted so a
    loop can be driven to its iteration cap. Records every kwarg it was
    called with, which is how the reasoning_effort wiring is asserted.
    """

    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = []

    def chat(self, messages, **kwargs):
        self.calls.append(kwargs)
        idx = min(len(self.calls) - 1, len(self.replies) - 1)
        return self.replies[idx]

    @property
    def n_calls(self):
        return len(self.calls)


def _respond(text):
    return '{"tool": "respond", "text": "%s"}' % text


# --------------------------------------------------------------------
# Shared control-flow contract. All three subagents implement the same
# skeleton; parametrizing documents that fact and keeps them honest.
# --------------------------------------------------------------------

def _run_security(backend, tmp_path, **kw):
    return sec.security(query='what is listening?', llm_backend=backend,
                        trace_dir=tmp_path / 'traces',
                        baseline_dir=tmp_path / 'baselines', **kw)


def _run_recall(backend, tmp_path, **kw):
    mem = tmp_path / 'memory'
    mem.mkdir(parents=True, exist_ok=True)
    return rem.recall(query='what did we decide?', memory_dir=mem,
                        llm_backend=backend, trace_dir=tmp_path / 'traces',
                        **kw)


def _run_inspect(backend, tmp_path, **kw):
    repo = tmp_path / 'repo'
    repo.mkdir(parents=True, exist_ok=True)
    (repo / 'a.py').write_text('x = 1\n')
    return cs.inspect(query='where is x set?', repo_root=repo,
                      llm_backend=backend, trace_dir=tmp_path / 'traces', **kw)


def _run_inspect_external(backend, tmp_path, **kw):
    repo = tmp_path / 'ext'
    repo.mkdir(parents=True, exist_ok=True)
    (repo / 'a.py').write_text('y = 2\n')
    return cs.inspect_external(query='where is y set?', repo_root=repo,
                               llm_backend=backend,
                               trace_dir=tmp_path / 'traces', **kw)


RUNNERS = [
    pytest.param(_run_security, sec, 'security', id='security'),
    pytest.param(_run_recall, rem, 'recall', id='recall'),
    pytest.param(_run_inspect, cs, 'inspect', id='inspect'),
    pytest.param(_run_inspect_external, cs, 'inspect_external',
                 id='inspect_external'),
]


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_respond_action_terminates_with_its_text(run, mod, label, tmp_path):
    """`respond` is the terminal action in all three loops, and its
    `text` becomes the returned answer verbatim."""
    backend = FakeBackend([_respond('the answer')])
    out = run(backend, tmp_path)
    assert 'the answer' in out
    assert backend.n_calls == 1


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_unparseable_output_burns_an_iteration(run, mod, label, tmp_path):
    """Unparseable emissions do not abort and do not retry for free —
    each one costs an iteration. This is the behavior that diverges from
    the main chat loop, which has dedicated format retries on top of a
    schema; recorded here so a consolidation has to decide deliberately.
    """
    backend = FakeBackend(['not json at all'])
    out = run(backend, tmp_path)
    assert backend.n_calls == mod._MAX_ITERS
    assert 'max iterations' in out


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_unknown_tool_is_reported_but_loop_continues(run, mod, label,
                                                     tmp_path):
    """An unrecognized tool yields an ERROR observation rather than an
    exception, and the loop keeps going until the cap."""
    backend = FakeBackend(['{"tool": "no_such_tool"}'])
    out = run(backend, tmp_path)
    assert backend.n_calls == mod._MAX_ITERS
    assert 'max iterations' in out


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_llm_exception_exits_without_raising(run, mod, label, tmp_path):
    """A backend that raises ends the loop with a reported error rather
    than propagating — the parent ReAct loop binds the result to a
    $stepN observation and must always get a string."""

    class Boom:
        def chat(self, messages, **kwargs):
            raise RuntimeError('backend exploded')

    out = run(Boom(), tmp_path)
    assert isinstance(out, str)
    assert 'error' in out.lower()


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_trace_file_is_written(run, mod, label, tmp_path):
    """Every loop writes exactly one trace file per call, named for its
    subagent. The prefix and the header tag are the observable contract
    consolidation had to preserve — trace files are read by hand, and
    renaming them silently would orphan every existing one."""
    backend = FakeBackend([_respond('done')])
    run(backend, tmp_path)
    traces = list((tmp_path / 'traces').glob('*.txt'))
    assert len(traces) == 1
    assert traces[0].name.startswith(label + '_'), traces[0].name
    body = traces[0].read_text()
    assert body.startswith('=' * 80)
    assert f'[{label}]' in body
    assert 'exit=respond' in body
    assert 'FINAL ANSWER:' in body
    assert 'done' in body


# --------------------------------------------------------------------
# reasoning_effort plumbing (Stage 0). The point of these two is that
# the flag reaches every loop, and that it stays absent by default so
# non-reasoning backends are untouched.
# --------------------------------------------------------------------

@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_reasoning_effort_is_forwarded_when_set(run, mod, label, tmp_path):
    backend = FakeBackend([_respond('ok')])
    run(backend, tmp_path, reasoning_effort='low')
    assert backend.calls[0].get('reasoning_effort') == 'low'


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_reasoning_effort_defaults_to_none(run, mod, label, tmp_path):
    """Default must be None, not 'low'. A non-reasoning backend (Gemma4,
    whose template defaults enable_thinking false) would otherwise have
    thinking switched on silently."""
    backend = FakeBackend([_respond('ok')])
    run(backend, tmp_path)
    assert backend.calls[0].get('reasoning_effort') is None


# --------------------------------------------------------------------
# Divergence the consolidation has to reconcile. Asserting the current
# numbers means a unified loop cannot quietly change them.
# --------------------------------------------------------------------

def test_iteration_caps_currently_differ():
    assert REACT_MAX_ITERS == 12
    assert sec._MAX_ITERS == 12
    assert cs._MAX_ITERS == 12
    assert rem._MAX_ITERS == 10, 'recall runs a tighter budget than the rest'


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_every_subagent_sends_the_response_schema(run, mod, label, tmp_path):
    """Structured output reaches all four entry points.

    Until 2026-08-15 only the main loop forced schema-valid JSON, on the
    evidence that subagents had produced zero unparseable emissions in
    ~1400 traced iterations. That held for one model; the first run against
    Gemma-4-31B produced 9 unparseable emissions in a single call. The
    schema is what makes it structural rather than model-dependent.
    """
    backend = FakeBackend([_respond('ok')])
    run(backend, tmp_path)
    assert backend.calls[0].get('response_schema') is not None


def test_subagents_reuse_the_main_loop_schema_rather_than_a_copy():
    """One schema definition, so the two cannot drift apart."""
    from chat.react import REACT_ACTION_SCHEMA
    from chat.subagents.subagent import Subagent
    assert Subagent.response_schema is REACT_ACTION_SCHEMA
    assert REACT_ACTION_SCHEMA['required'] == ['thought', 'tool']


# --------------------------------------------------------------------
# Main-loop pure helpers. The loop body is out of reach (see module
# docstring); these are its testable surface.
# --------------------------------------------------------------------

def test_parse_react_action_recovers_dicts_and_rejects_the_rest():
    parse = ReactMixin._parse_react_action
    assert parse('{"tool": "respond"}') == {'tool': 'respond'}
    # Fenced output is the common Qwen/Gemma emission shape.
    assert parse('```json\n{"tool": "respond"}\n```') == {'tool': 'respond'}
    assert parse('') is None
    assert parse('no json here') is None
    assert parse(None) is None
    # A bare JSON list is not an action.
    assert parse('[1, 2, 3]') is None


def test_resolve_react_value_strips_observation_prefixes():
    """$stepN substitution must hand downstream tools clean content —
    the OK:/EMPTY:/ERROR: prefix is signal for the model reading the
    log, not for the user reading a reply."""
    resolve = ReactMixin._resolve_react_value
    log = [('$step1', 'OK: clean content'),
           ('$step2', 'ERROR: it broke'),
           ('$step3', 'EMPTY: nothing found')]
    assert resolve('$step1', log) == 'clean content'
    assert resolve('$step2', log) == 'it broke'
    assert resolve('$step3', log) == 'nothing found'
    # Literals pass through untouched.
    assert resolve('just text', log) == 'just text'
    # An unmatched binding resolves to empty rather than raising.
    assert resolve('$step9', log) == ''


def test_react_max_iters_is_published_for_prompt_text():
    """reflection.py and tools.py import this constant into prompt
    strings shown to the model, so it is part of the module contract,
    not an internal detail."""
    import chat.reflection as reflection
    import chat.tools as tools
    assert 'REACT_MAX_ITERS' in Path(reflection.__file__).read_text()
    assert 'REACT_MAX_ITERS' in Path(tools.__file__).read_text()
