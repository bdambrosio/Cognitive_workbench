"""Output reservation is clamped to the served model's context window.

vLLM rejects at admission when prompt_tokens + max_tokens exceeds
max_model_len — nothing is generated and the whole turn is lost to
fallback synthesis. Observed 2026-08-15 at 57,145 + 8,192 against 65,336:
over by one token.

The window belongs to whichever model the server currently holds, not to
any scenario, so it is discovered from /v1/models rather than declared in
YAML. A scenario cannot know it, and a hand-copied value goes stale the
moment the backend is swapped — which on this rig is several times a day.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))

from chat.backend import _ChatBackend  # noqa: E402


def _backend(window, cloud=False):
    # NAMED, NOT EMPTY. `model=''` asks the backend to resolve what the server
    # is serving, which now means an HTTP call to base_url — unreachable here
    # by design — and the temperature gate then has nothing to look up. Both
    # postdate this test. The window is still injected below; only the identity
    # is real.
    b = _ChatBackend(server='local', model='Qwen3.8-27B',
                     base_url='http://x:5000')
    b._server_max_model_len = window   # skip the /v1/models fetch
    b.is_cloud = cloud
    return b


def _msgs(chars):
    return [{'role': 'user', 'content': 'x' * chars}]


def test_no_clamp_when_there_is_room():
    b = _backend(65336)
    assert b._clamp_max_tokens(_msgs(200), 8192) == 8192


def test_the_one_token_overflow_is_clamped():
    """The real failure: a prompt just under the ceiling plus a full
    output reservation tipping one token over."""
    b = _backend(65336)
    out = b._clamp_max_tokens(_msgs(171_000), 8192)   # ~57k tokens
    assert out < 8192
    assert out > 256


def test_clamped_reservation_actually_fits():
    b = _backend(65336)
    chars = 171_000
    out = b._clamp_max_tokens(_msgs(chars), 8192)
    est_prompt = int(chars / b._CHARS_PER_TOKEN) + b._CLAMP_MARGIN_TOKENS
    assert est_prompt + out <= 65336


def test_impossible_prompt_returns_a_floor_not_a_silent_tiny_reply():
    """When the prompt alone overruns the window there is nothing to
    clamp to. Returning ~0 would produce an empty answer that reads as
    the model having nothing to say; the floor lets the server reject
    honestly and the caller's fallback handle it."""
    b = _backend(65336)
    assert b._clamp_max_tokens(_msgs(300_000), 8192) == 256


def test_cloud_backends_are_left_alone():
    """Cloud providers meter reasoning separately and max_tokens caps
    visible output only, so the arithmetic does not transfer."""
    b = _backend(65336, cloud=True)
    assert b._clamp_max_tokens(_msgs(300_000), 8192) == 8192


def test_unknown_window_disables_the_clamp():
    """A server that does not publish max_model_len must not have a
    guessed window imposed on it."""
    b = _backend(0)
    assert b._clamp_max_tokens(_msgs(300_000), 8192) == 8192


def test_window_is_probed_once_then_cached(monkeypatch):
    calls = {'n': 0}

    class _Resp:
        ok = True

        @staticmethod
        def json():
            return {'data': [{'max_model_len': 4096}]}

    def _get(url, timeout=None):
        calls['n'] += 1
        return _Resp()

    import chat.backend as backend_mod
    monkeypatch.setattr(backend_mod.requests, 'get', _get)
    b = _ChatBackend(server='local', model='', base_url='http://x:5000')
    assert b._server_window() == 4096
    assert b._server_window() == 4096
    assert calls['n'] == 1, 'window must be fetched once, not per call'


def test_chat_actually_applies_the_clamp_to_the_request(monkeypatch):
    """Wiring, not just arithmetic.

    The first version of these tests exercised _clamp_max_tokens directly
    and stayed green when the call was deleted from chat() — the clamp was
    correct and unreachable. This asserts the value that reaches the wire.
    """
    sent = {}

    class _Resp:
        ok = True
        status_code = 200

        @staticmethod
        def json():
            return {'choices': [{'message': {'content': 'ok'},
                                 'finish_reason': 'stop'}]}

    def _post(url, headers=None, json=None, timeout=None):
        sent.update(json or {})
        return _Resp()

    import chat.backend as backend_mod
    monkeypatch.setattr(backend_mod.requests, 'post', _post)
    b = _backend(65336)
    b.chat(_msgs(171_000), max_tokens=8192)
    assert sent, 'no request was sent'
    assert sent['max_tokens'] < 8192, (
        f"clamp not applied on the wire: max_tokens={sent['max_tokens']}")


def test_unreachable_server_does_not_raise(monkeypatch):
    """A failed probe disables the clamp rather than taking down the call
    it was meant to protect."""
    def _boom(url, timeout=None):
        raise OSError('connection refused')

    import chat.backend as backend_mod
    monkeypatch.setattr(backend_mod.requests, 'get', _boom)
    b = _ChatBackend(server='local', model='', base_url='http://x:5000')
    assert b._server_window() == 0
    assert b._clamp_max_tokens(_msgs(300_000), 8192) == 8192
