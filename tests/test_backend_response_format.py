"""Structured-output constraint per route.

`grammar` and `chat_template_kwargs` are genuinely local-only — cloud
rejects them. `response_format` is not, and bundling all three behind one
`is_cloud` guard left the cloud route as the only path with no
structured-output constraint at all. Measured 2026-08-16 on gpt-5.6-luna
at reasoning_effort=low: 77 unparseable emissions in a single run, 76 of
them finish=stop — complete outputs that simply were not JSON.

The shape differs by route, and the difference was verified against the
live API rather than reasoned about:

  strict=true            → 400, "'additionalProperties' is required to be
    supplied and to be false" — and setting it false would forbid every
    tool's own arguments (text, query, to, ...).
  json_schema non-strict → 200, but the model reads required:[thought,
    tool] as the list of fields to emit and DROPS the args, 3/3 trials.
    A `respond` with no `text` is not an answer.
  json_object            → 200, all fields intact, 3/3.

So on cloud the schema is worse than useless; the only thing needed is
the guarantee that the emission parses.
"""

import json
import sys
from pathlib import Path

import pytest
import requests

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.backend import _ChatBackend        # noqa: E402
from chat.react import REACT_ACTION_SCHEMA   # noqa: E402


@pytest.fixture
def captured(monkeypatch):
    """Intercept the request body without making a call."""
    box = {}

    def fake_post(url, **kw):
        box['body'] = kw.get('json')
        raise RuntimeError('intercepted')

    monkeypatch.setattr(requests, 'post', fake_post)
    return box


def _send(captured, backend, **kw):
    try:
        backend.chat([{'role': 'user', 'content': 'emit one JSON action'}], **kw)
    except Exception:
        pass
    return (captured.get('body') or {}).get('response_format')


def _local():
    # A CONFIGURED MODEL, NOT A PLACEHOLDER. This read `model='m'` until
    # 2026-08-29. src/chat/model_params.py raises UnknownModel for a model with
    # no recommended temperature — deliberately, so a run cannot inherit a
    # default — and it fires before the request body exists, so every
    # assertion here was passing on `None` rather than on a response_format.
    # `_cloud()` already used a real name, which is why only the local tests
    # went red.
    return _ChatBackend(server='local', model='Qwen3.8-27B',
                        base_url='http://127.0.0.1:5000')


def _cloud():
    return _ChatBackend(server='local', model='gpt-5.6-luna',
                        base_url='https://api.openai.com/v1',
                        api_key='OPENAI_API_KEY')


def test_local_gets_the_full_schema():
    """vLLM / SGLang honour json_schema, and it also bounds the thought's
    maxLength so a runaway thought cannot eat the token budget."""
    assert _local().is_cloud is False


def test_local_route_sends_json_schema(captured):
    rf = _send(captured, _local(), response_schema=REACT_ACTION_SCHEMA)
    assert rf['type'] == 'json_schema'
    assert rf['json_schema']['schema'] == REACT_ACTION_SCHEMA


def test_cloud_route_sends_the_schema_too(captured):
    """required:[thought,tool] is the constraint that stops a model
    emitting its ANSWER as JSON instead of an action. json_object was
    tried first and is not enough: it guarantees the emission parses and
    nothing about its fields, and dropped `tool` 1/3 when probed against
    the real subagent prompt."""
    rf = _send(captured, _cloud(), response_schema=REACT_ACTION_SCHEMA)
    assert rf['type'] == 'json_schema'
    assert rf['json_schema']['schema'] == REACT_ACTION_SCHEMA


def test_cloud_does_not_ask_for_strict_mode(captured):
    """strict=true 400s on this schema — it demands
    additionalProperties=false, and the extras are deliberate so each
    tool's own args pass through."""
    rf = _send(captured, _cloud(), response_schema=REACT_ACTION_SCHEMA)
    assert 'strict' not in rf['json_schema']


def test_cloud_route_is_constrained_at_all():
    """Regression guard for the original bug: the constraint used to be
    inside `if not is_cloud`, so cloud emissions were unconstrained."""
    assert _cloud().is_cloud is True


@pytest.mark.parametrize('make', [_local, _cloud], ids=['local', 'cloud'])
def test_no_constraint_when_the_caller_asks_for_none(captured, make):
    """Only callers that pass a schema get a response_format — a plain
    prose call must not be forced into JSON on either route."""
    assert _send(captured, make()) is None


def test_the_schema_would_be_rejected_by_strict_mode():
    """Documents WHY cloud cannot use json_schema strict: the schema
    deliberately allows extra properties so each tool's own args pass
    through, and strict mode forbids exactly that."""
    assert REACT_ACTION_SCHEMA['additionalProperties'] is True
    assert REACT_ACTION_SCHEMA['required'] == ['thought', 'tool']
    # `text`, `query`, `to`, … are none of them required, which is why the
    # non-strict schema route saw them dropped.
    assert 'text' not in REACT_ACTION_SCHEMA['properties']
