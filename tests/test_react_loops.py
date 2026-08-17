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
from chat.subagents.subagent import Subagent  # noqa: E402


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
def test_unparseable_output_gives_up_early(run, mod, label, tmp_path):
    """Consecutive unparseable emissions stop the loop instead of grinding
    to the iteration cap.

    Was: each one cost an iteration and the loop ran all 12, recorded here
    "so a consolidation has to decide deliberately". Decided 2026-08-16 —
    a code_subagent call spent 7 iterations and ~6 minutes re-emitting the
    same oversized `respond`, so the loop now bails after
    `max_consecutive_unparseable`.
    """
    backend = FakeBackend(['not json at all'])
    out = run(backend, tmp_path)
    assert backend.n_calls == Subagent.max_consecutive_unparseable
    assert 'unparseable' in out
    assert 'valid JSON' in out


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_truncated_emission_is_told_it_was_too_long(run, mod, label,
                                                    tmp_path):
    """A parse failure caused by the token limit gets different advice
    than malformed output — otherwise the model re-emits the same
    oversized payload verbatim, which is exactly what happened live."""

    class TruncatingBackend(FakeBackend):
        last_finish_reason = 'length'

        def chat(self, messages, **kwargs):
            self.prompts.append(messages[-1]['content'])
            return super().chat(messages, **kwargs)

        prompts: list = []

    backend = TruncatingBackend(['{"tool": "respond", "text": "301|  '])
    backend.prompts = []
    out = run(backend, tmp_path)

    assert backend.n_calls == Subagent.max_consecutive_unparseable
    retries = backend.prompts[1:]
    assert retries and all('cut off by the token limit' in p for p in retries)
    assert not any('not valid JSON' in p for p in retries)
    assert 'too large to emit' in out


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_unparseable_counter_resets_on_success(run, mod, label, tmp_path):
    """The cap counts *consecutive* failures — one stumble mid-loop must
    not poison a run that recovers."""
    backend = FakeBackend(['not json at all', _respond('recovered')])
    out = run(backend, tmp_path)
    assert backend.n_calls == 2
    assert 'recovered' in out


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


# --------------------------------------------------------------------
# Subagent reasoning-effort ceiling.
#
# A character's reasoning_effort baseline reaches every emission, and
# almost none of a subagent's are the thinking — it picks a primitive,
# picks a path, emits JSON. Measured 2026-08-16 on a cloud reasoning
# model: one turn made 19 inspect calls of ~5 iterations each, so ~100 of
# ~130 emissions were a subagent choosing which file to read, all at
# effort=high, billed and not returned.
# --------------------------------------------------------------------

def test_subagent_clamps_an_expensive_baseline():
    assert Subagent._clamp_effort('high') == 'low'
    assert Subagent._clamp_effort('medium') == 'low'


def test_clamp_is_a_ceiling_not_an_override():
    """A character already at or below the ceiling keeps its own value,
    and 'unset' stays unset so nothing is sent."""
    assert Subagent._clamp_effort('low') == 'low'
    assert Subagent._clamp_effort(None) is None


def test_a_subclass_may_raise_its_own_ceiling():
    class Thinky(Subagent):
        max_reasoning_effort = 'high'
    assert Thinky._clamp_effort('high') == 'high'

    class Unbounded(Subagent):
        max_reasoning_effort = None
    assert Unbounded._clamp_effort('high') == 'high'


@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_every_subagent_applies_the_ceiling(run, mod, label, tmp_path):
    """All three subagents construct through Subagent.__init__, so the
    clamp is one choke point rather than three hand-offs."""
    backend = FakeBackend([_respond('done')])
    run(backend, tmp_path, reasoning_effort='high')
    assert backend.calls, 'expected at least one call'
    assert all(c.get('reasoning_effort') == 'low' for c in backend.calls), \
        [c.get('reasoning_effort') for c in backend.calls]


# --------------------------------------------------------------------
# Parsed, but not an action.
#
# A cloud model with no schema constraint emitted its ANSWER as JSON —
# {"text": ...}, {"creation_occurrences": ...} — and, once it decided the
# tools were broken, {"error": ...}. The old reply was "unknown tool
# None", which names nothing actionable and reads as a tool failure. 160
# of 217 observations in one run were that error, and the model concluded
# its file reads were returning nothing. They never were, not once: there
# were zero EMPTY observations in the entire run.
# --------------------------------------------------------------------

TOOLLESS_SHAPES = [
    pytest.param('{"text": "the whole answer"}', id='bare-text'),
    pytest.param('{"creation_occurrences": [1, 2]}', id='invented-schema'),
    pytest.param('{"error": "tool returned no output"}', id='refusal'),
]


@pytest.mark.parametrize('raw', TOOLLESS_SHAPES)
@pytest.mark.parametrize('run,mod,label', RUNNERS)
def test_toolless_emission_is_named_as_a_format_problem(run, mod, label,
                                                        raw, tmp_path):
    backend = FakeBackend([raw, _respond('recovered')])
    out = run(backend, tmp_path)
    assert 'recovered' in out, 'the loop must continue, not abort'
    feedback = backend.calls  # kwargs only; check the prompt instead
    assert backend.n_calls == 2


@pytest.mark.parametrize('raw', TOOLLESS_SHAPES)
def test_the_feedback_says_what_to_do_instead(raw, tmp_path):
    """Naming the missing field is the point — a model that cannot tell a
    format slip from a broken tool starts refusing."""

    class Recording(FakeBackend):
        prompts: list = []

        def chat(self, messages, **kwargs):
            self.prompts.append(messages[-1]['content'])
            return super().chat(messages, **kwargs)

    backend = Recording([raw, _respond('ok')])
    backend.prompts = []
    _run_inspect(backend, tmp_path)

    note = backend.prompts[1]
    assert 'no `tool` field' in note
    assert 'not a tool failure' in note        # the belief to head off
    assert '"tool": "respond"' in note         # the shape to emit
    assert 'unknown tool None' not in note     # the unhelpful old wording


def test_a_toolless_emission_does_not_count_as_unparseable(tmp_path):
    """It parsed. Counting it toward the unparseable cap would bail on a
    model that is merely mis-shaping its actions and can be corrected."""
    backend = FakeBackend(['{"text": "a"}', '{"text": "b"}',
                           '{"text": "c"}', '{"text": "d"}',
                           _respond('recovered')])
    out = _run_inspect(backend, tmp_path)
    assert 'recovered' in out
    assert backend.n_calls == 5


# --------------------------------------------------------------------
# process_text `source` validation. `source` must BE material, not name
# where material lives — a pointer transforms into a synthesis of text the
# model never saw (observed 2026-08-17). Shape can't separate a short
# pointer from short material, so the guard asks; these script the probe.
# --------------------------------------------------------------------

class _Diagnosing(ReactMixin):
    """ReactMixin's arg validation with a scripted probe backend."""

    character_name = 'Jill'
    persona = ''

    def __init__(self, probe_verdict='material'):
        self.backend = FakeBackend([probe_verdict])

    def diagnose(self, raw_src, resolved_src=None, instruction='Rewrite it.'):
        return self._diagnose_process_text_args(
            raw_src, resolved_src if resolved_src is not None else raw_src,
            instruction, [('$step1', 'earlier output')])


def test_source_naming_material_is_rejected():
    h = _Diagnosing('reference')
    diag = h.diagnose('Recent reasoning traces #2728 through #2733',
                      instruction='Draft the proposal itself.')
    assert diag is not None
    assert 'names where material lives' in diag
    assert '$stepN' in diag                     # the recovery path


def test_short_literal_material_is_allowed():
    h = _Diagnosing('material')
    assert h.diagnose('eggs, milk, flour',
                      instruction='Reformat as a checklist.') is None


def test_long_source_skips_the_probe_entirely():
    """Nobody writes 200 characters of pointer. Staying off the common
    path is the reason the probe is affordable at all."""
    h = _Diagnosing('reference')
    assert h.diagnose('word ' * 100) is None
    assert h.backend.n_calls == 0


def test_a_resolved_binding_skips_the_probe():
    """$stepN content came from a real tool — material by construction,
    however short it reads."""
    h = _Diagnosing('reference')
    assert h.diagnose('$step1', resolved_src='ok short output') is None
    assert h.backend.n_calls == 0


def test_the_probe_fails_open():
    """A guard against one silent failure mode must not make process_text
    unavailable whenever the backend hiccups."""
    class Broken(FakeBackend):
        def chat(self, messages, **kwargs):
            raise RuntimeError('backend down')

    h = _Diagnosing()
    h.backend = Broken([])
    assert h.diagnose('the dialogue with Jack') is None


def test_an_off_format_verdict_fails_open():
    h = _Diagnosing('I think this is probably a reference, but...')
    assert h.diagnose('the dialogue with Jack') is None


def test_cheaper_checks_run_before_the_probe():
    """An unresolved binding is diagnosable from shape alone; spending an
    LLM call to say so would be waste."""
    h = _Diagnosing('reference')
    diag = h.diagnose('$step9', resolved_src='')
    assert diag is not None and 'unresolved binding' in diag
    assert h.backend.n_calls == 0
