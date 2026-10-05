"""The check-email tool as the chat agent calls it (agreed with Jill,
2026-10-05): the observation is the mail itself. The IMAP server is replaced
by raw messages held in a list."""
import contextlib
import importlib.util
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))


def _raw(sender, subject, body):
    return (f"From: {sender}\r\nTo: me@example.com\r\nSubject: {subject}\r\n"
            f"Date: Mon, 05 Oct 2026 08:00:00 -0700\r\n"
            f"Content-Type: text/plain\r\n\r\n{body}\r\n").encode()


@pytest.fixture
def check_email(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "check_email_tool", SRC / "tools" / "check-email" / "tool.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setenv("GMAIL_ADDRESS", "me@example.com")
    monkeypatch.setenv("GMAIL_APP_PASSWORD", "x")

    @contextlib.contextmanager
    def no_server(address, password):
        yield None

    monkeypatch.setattr(mod, "_IMAPConnection", no_server)

    def run(messages):
        monkeypatch.setattr(mod, "_fetch_emails", lambda *a: messages)
        return mod.react_invoke({})

    return run


def test_observation_shows_sender_subject_and_body_of_each_email(check_email):
    out = check_email([
        _raw("Alice <alice@example.com>", "Invoice 14", "Payment is due Friday."),
        _raw("Bob <bob@example.com>", "Lunch", "Noon at the usual place?"),
    ])
    text = out["text"]
    for expected in ("alice@example.com", "Invoice 14", "Payment is due Friday.",
                     "bob@example.com", "Lunch", "Noon at the usual place?"):
        assert expected in text
    assert text.index("Invoice 14") < text.index("Lunch")


def test_long_body_is_cut_and_the_full_length_is_stated(check_email):
    out = check_email([_raw("Alice <alice@example.com>", "Long", "a" * 6000 + "END")])
    assert "END" not in out["text"]
    assert "6003" in out["text"]
