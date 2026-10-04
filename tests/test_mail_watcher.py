"""The mail-watcher sensor (agreed with Jill, 2026-10-03): what she is told
about mail, and when. The IMAP server is replaced by an inbox held in a list."""
import importlib.util
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))


class _Inbox:
    """An inbox as (uid, sender, subject) rows behind the IMAP calls the
    sensor makes."""

    def __init__(self, rows):
        self.rows = rows

    def __call__(self, address, password):
        return self

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def select(self, folder, readonly=False):
        assert readonly
        return 'OK', [b'']

    def uid(self, command, *args):
        if command == 'search':
            uids = [u for u, _, _ in self.rows]
            if args[1] != 'ALL':
                low = int(args[1].split()[1].split(':')[0])
                # As IMAP does: `n:*` includes the newest message even when
                # its UID is below n.
                uids = sorted({u for u in uids if u >= low} | ({max(uids)} if uids else set()))
            return 'OK', [' '.join(str(u) for u in uids).encode()]
        uid = int(args[0])
        _, sender, subject = next(r for r in self.rows if r[0] == uid)
        return 'OK', [(b'', f"From: {sender}\r\nSubject: {subject}\r\n\r\n".encode())]


@pytest.fixture
def sensor(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "sensor_mail_watcher", SRC / "sensors/mail-watcher/sensor.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("TEST_MAIL_ADDRESS", "bruce@example.com")
    monkeypatch.setenv("TEST_MAIL_PASSWORD", "app-password")
    return module


CTX = {'character_name': 'Tester', 'parameters': {'accounts': [
    {'address_env': 'TEST_MAIL_ADDRESS', 'password_env': 'TEST_MAIL_PASSWORD'},
    {'address_env': 'UNSET_MAIL_ADDRESS', 'password_env': 'UNSET_MAIL_PASSWORD'}]}}


def test_what_the_inbox_already_holds_is_not_reported_and_later_mail_is(sensor, monkeypatch):
    inbox = _Inbox([(7, "Old Sender <old@example.com>", "Already here")])
    monkeypatch.setattr(sensor, "IMAPConnection", inbox)
    assert sensor.run(CTX)['status'] == 'nothing'        # first poll: baseline only
    assert sensor.run(CTX)['status'] == 'nothing'        # nothing arrived; the newest is not repeated
    inbox.rows.append((9, "Ann Buyer <ann@example.com>", "Your review"))
    report = sensor.run(CTX)
    assert report['status'] == 'ok' and report['metadata']['item_count'] == 1
    assert 'From Ann Buyer <ann@example.com>: "Your review"' in report['content']
    assert "Already here" not in report['content']
    assert sensor.run(CTX)['status'] == 'nothing'        # reported once


def test_an_account_with_no_password_set_is_skipped_and_the_rest_still_run(sensor, monkeypatch):
    inbox = _Inbox([(1, "a@example.com", "one")])
    monkeypatch.setattr(sensor, "IMAPConnection", inbox)
    sensor.run(CTX)
    inbox.rows.append((2, "b@example.com", "two"))
    assert 'From b@example.com: "two"' in sensor.run(CTX)['content']


def test_an_account_that_cannot_be_read_does_not_stop_the_sensor(sensor, monkeypatch):
    def refuse(address, password):
        raise OSError("login refused")
    monkeypatch.setattr(sensor, "IMAPConnection", refuse)
    assert sensor.run(CTX)['status'] == 'nothing'
