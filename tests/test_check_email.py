"""The check-email tool as the chat agent calls it (agreed with Jill,
2026-10-05): the observation is the mail itself. The IMAP server is replaced
by raw messages held in a list."""
import contextlib
import imaplib
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


class _Server:
    """Folders of raw messages behind the IMAP calls the tool makes. Parses
    a folder name as Gmail does: a name with a space must be quoted."""

    def __init__(self, folders):
        self.folders = folders
        self.open = []

    def select(self, name, readonly=False):
        assert readonly
        if name.startswith('"') and name.endswith('"'):
            name = name[1:-1]
        elif ' ' in name:
            raise imaplib.IMAP4.error("EXAMINE command error: BAD [b'Could not parse command']")
        if name not in self.folders:
            return 'NO', [b'[NONEXISTENT] Unknown Mailbox: ' + name.encode()]
        self.open = self.folders[name]
        return 'OK', [str(len(self.open)).encode()]

    def search(self, charset, criteria):
        return 'OK', [b' '.join(str(i + 1).encode() for i in range(len(self.open)))]

    def fetch(self, mid, what):
        return 'OK', [(b'', self.open[int(mid) - 1])]


@pytest.fixture
def check_folder(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "check_email_tool", SRC / "tools" / "check-email" / "tool.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setenv("GMAIL_ADDRESS", "me@example.com")
    monkeypatch.setenv("GMAIL_APP_PASSWORD", "x")
    server = _Server({"[Gmail]/All Mail": [
        _raw("Alice <alice@example.com>", "Invoice 14", "Payment is due Friday.")]})

    @contextlib.contextmanager
    def logged_in(address, password):
        yield server

    monkeypatch.setattr(mod, "_IMAPConnection", logged_in)
    return lambda folder: mod.react_invoke({"folder": folder})


def test_folder_name_with_a_space_is_read(check_folder):
    out = check_folder("[Gmail]/All Mail")
    assert out["status"] == "ok"
    assert "Invoice 14" in out["text"]


def test_refused_folder_is_reported_as_the_folder_not_as_a_login_failure(check_folder):
    out = check_folder("[Gmail]/Nowhere")
    assert out["status"] == "error"
    assert "[Gmail]/Nowhere" in out["text"]
    assert "Unknown Mailbox" in out["text"]
