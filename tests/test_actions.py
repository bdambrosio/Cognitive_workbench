"""The action list and its reminders (chat/actions.py), agreed with Jill
2026-10-22: the tool makes every change and ids never change; done is final;
a reminder fires once, emails Bruce whether or not autonomy is on, and gives
her a turn only with autonomy on; a new reminder time fires again."""
import importlib.util
import sys
from datetime import datetime
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import chat.actions as actions  # noqa: E402
from chat.actions import ActionsMixin  # noqa: E402
from client_ui import mail  # noqa: E402


class _Host(ActionsMixin):
    character_name = "TestJill"

    def __init__(self, tmp: Path, autonomy=True):
        self._mem = tmp
        self._autonomy_enabled = autonomy
        self.turns = []

    def _memory_dir(self):
        return self._mem

    def _process_user_turn(self, **kw):
        self.turns.append(kw)


def _at(monkeypatch, when: str):
    monkeypatch.setattr(actions, "_now", lambda: datetime.fromisoformat(when))


def _mail(monkeypatch):
    monkeypatch.setenv("MAIL_DRY_RUN", "1")
    monkeypatch.setenv("GMAIL_ADDRESS", "bruce@example.com")
    monkeypatch.setenv("GMAIL_APP_PASSWORD", "not-a-real-password")
    mail.sent.clear()


def test_items_keep_their_ids_and_done_is_final(tmp_path, monkeypatch):
    _at(monkeypatch, "2026-10-22 20:00")
    host = _Host(tmp_path)
    assert host._run_actions("add", {"text": "Renew domain"}).startswith("ERROR")  # no context
    assert host._run_actions("add", {"text": "Renew domain", "context": "Bruce, expiry notice"}
                             ).startswith("OK: added a1")
    assert host._run_actions("add", {"text": "Call Ed", "context": "Bruce asked"}
                             ).startswith("OK: added a2")
    assert host._run_actions("done", {"id": "a1"}).startswith("OK: a1 is done")
    assert host._run_actions("done", {"id": "a1"}).startswith("ERROR")
    assert host._run_actions("edit", {"id": "a1", "text": "x"}).startswith("ERROR")
    open_list = host._run_actions("list", {})
    assert "a2: Call Ed" in open_list and "a1" not in open_list
    assert "a1: Renew domain" in host._run_actions("list", {"which": "all"})
    # Reopening is a new item; the next id is never a reused one.
    assert host._run_actions("add", {"text": "Renew domain", "context": "payment failed",
                                     "follows": "a1"}).startswith("OK: added a3")


def test_an_unreadable_file_is_left_as_it_is(tmp_path):
    f = tmp_path / actions.ACTIONS_FILE
    f.write_text("- id: a1\n  text: [unclosed\n", encoding="utf-8")
    before = f.read_text(encoding="utf-8")
    host = _Host(tmp_path)
    assert host._run_actions("add", {"text": "t", "context": "c"}).startswith("ERROR")
    assert f.read_text(encoding="utf-8") == before


def test_a_reminder_fires_once_with_an_email_and_a_turn(tmp_path, monkeypatch):
    _mail(monkeypatch)
    _at(monkeypatch, "2026-10-22 20:00")
    host = _Host(tmp_path)
    host._run_actions("add", {"text": "Check the sync fix", "context": "Bruce asked",
                              "remind_at": "2026-10-24 15:00"})
    assert "Check the sync fix (a1)" in host._render_reminders_block()
    assert host._fire_due_reminder() is False                  # not due yet
    _at(monkeypatch, "2026-10-24 15:01")
    assert host._fire_due_reminder() is True
    assert [m["subject"] for m in mail.sent] == ["[TestJill reminder] Check the sync fix"]
    assert mail.sent[0]["to"] == ["bruce@example.com"]
    assert len(host.turns) == 1 and "Check the sync fix" in host.turns[0]["text"]
    assert host._fire_due_reminder() is False                  # once only
    assert len(mail.sent) == 1
    assert host._render_reminders_block() == ""


def test_the_email_goes_without_autonomy_and_no_turn_runs(tmp_path, monkeypatch):
    _mail(monkeypatch)
    _at(monkeypatch, "2026-10-24 16:00")
    host = _Host(tmp_path, autonomy=False)
    host._run_actions("add", {"text": "Pay invoice", "context": "c",
                              "remind_at": "2026-10-24 15:00"})
    assert host._fire_due_reminder() is True
    assert len(mail.sent) == 1 and host.turns == []


def test_a_new_reminder_time_fires_again_and_done_items_never_fire(tmp_path, monkeypatch):
    _mail(monkeypatch)
    _at(monkeypatch, "2026-10-24 16:00")
    host = _Host(tmp_path, autonomy=False)
    host._run_actions("add", {"text": "A", "context": "c", "remind_at": "2026-10-24 15:00"})
    host._run_actions("add", {"text": "B", "context": "c", "remind_at": "2026-10-24 15:00"})
    host._run_actions("done", {"id": "a2"})
    assert host._fire_due_reminder() is True
    assert host._fire_due_reminder() is False                  # B is done
    host._run_actions("edit", {"id": "a1", "remind_at": "2026-10-24 15:30"})
    assert host._fire_due_reminder() is True
    assert [m["subject"] for m in mail.sent] == ["[TestJill reminder] A"] * 2
    saved = yaml.safe_load((tmp_path / actions.ACTIONS_FILE).read_text(encoding="utf-8"))
    assert saved[0]["reminded"] == "2026-10-24 16:00"


def _draft_tool():
    path = Path(__file__).resolve().parents[1] / "src/tools/draft-email/tool.py"
    spec = importlib.util.spec_from_file_location("draft_email_tool", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_drafts_folder_is_found_by_its_attribute():
    class _Conn:
        def __init__(self, rows):
            self.rows = rows

        def list(self):
            return "OK", self.rows
    find = _draft_tool()._drafts_folder
    assert find(_Conn([b'(\\HasNoChildren) "/" "INBOX"',
                       b'(\\Drafts \\HasNoChildren) "/" "[Gmail]/Entw&APw-rfe"'])
                ) == '"[Gmail]/Entw&APw-rfe"'
    assert find(_Conn([b'(\\HasNoChildren) "/" "INBOX"'])) is None
