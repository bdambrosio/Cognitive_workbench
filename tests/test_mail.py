"""Notices: dry run without credentials, the record kept, no recipient
handled, the practice list read from the environment."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from client_ui import mail                                      # noqa: E402


def test_dry_run_without_credentials_records_the_notice(monkeypatch):
    monkeypatch.delenv("SMTP_PASS", raising=False)
    monkeypatch.delenv("MAIL_DRY_RUN", raising=False)
    mail.sent.clear()
    rec = mail.send(["a@example.test"], "hello", "body", "http://x/y")
    assert rec["dry_run"] is True and rec["error"] is None
    assert rec["body"].endswith("body\n\nhttp://x/y\n")
    assert mail.sent == [rec]


def test_no_recipient_is_an_error_not_an_exception(monkeypatch):
    monkeypatch.setenv("MAIL_DRY_RUN", "1")
    rec = mail.send([], "hello", "body")
    assert rec["error"] == "no recipient"


def test_practice_emails_and_site_url(monkeypatch):
    monkeypatch.setenv("PRACTICE_EMAILS", "A@Example.test, b@example.test,")
    assert mail.practice_emails() == ["a@example.test", "b@example.test"]
    monkeypatch.setenv("SITE_URL", "https://client.tuuyi.com/")
    assert mail.site_url() == "https://client.tuuyi.com"


def test_cli_dry_run(monkeypatch, capsys):
    monkeypatch.setenv("MAIL_DRY_RUN", "1")
    assert mail.main(["--test", "a@example.test"]) == 0
    assert "dry run" in capsys.readouterr().out
