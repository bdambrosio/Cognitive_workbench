"""The action list and its reminders (Bruce's request, agreed with Jill,
2026-10-22).

THE LIST is `<memory>/actions.yaml`, a list of items:

    - id: a3
      text: Check the calendar sync fix
      context: Bruce asked on 2026-10-22, after the sync dropped two events
      added: 2026-10-22 20:30
      due: 2026-10-24                 # optional, any wording
      remind_at: 2026-10-24 15:00     # optional, local time
      reminded: 2026-10-24 15:00      # written when the reminder fires
      follows: a1                     # optional: the done item this reopens
      done: 2026-10-25 09:10          # absent while the item is open

The `actions` action makes every change itself (add, list, done, edit), so
the agent never rewrites the file. Ids are never reused or changed. Done is
final: a done item is not edited or reopened; reopening something is a new
item that `follows` the old one. Done items stay in the file. Bruce may edit
the file by hand; it is read again on every call and every tick. Nothing
here repeats: a recurring job belongs in Bruce's schedule.yaml.

A REMINDER is an open item whose `remind_at` has passed and that has no
`reminded`. The tick fires one at a time, whether or not autonomy is on:
`reminded` is written first, so a crash cannot fire it twice; then the
harness emails Bruce the item, so delivery does not depend on her turn;
then, with autonomy on, she gets a turn to put it on the canvas. Editing
`remind_at` clears `reminded`, so the new time fires. Her prompt lists the
upcoming reminders under their own heading, apart from Bruce's schedule and
from her concerns.

The email goes from and to the Gmail account the mail tools read
(GMAIL_ADDRESS, GMAIL_APP_PASSWORD), with the subject "[Jill reminder] ...".
"""
from __future__ import annotations

import logging
import os
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional

from chat.background import _now, _parse_at as _parse_time

logger = logging.getLogger(__name__)

ACTIONS_FILE = 'actions.yaml'
#: How many upcoming reminders the prompt lists.
REMINDERS_PROMPT_ITEMS = 5
#: The fields `edit` may change. `id`, `added`, `done` and `reminded` are the
#: tool's own.
EDITABLE = ('text', 'context', 'due', 'remind_at')


def _stamp(at: datetime) -> str:
    return f"{at:%Y-%m-%d %H:%M}"


class ActionsMixin:

    # ---- the file ---------------------------------------------------------

    def _actions_path(self):
        return self._memory_dir() / ACTIONS_FILE

    def _load_actions(self) -> List[Dict[str, Any]]:
        """The items, as written. Raises ValueError when the file cannot be
        read as a list of items: an edit must never overwrite a file it could
        not read."""
        f = self._actions_path()
        if not f.is_file():
            return []
        import yaml
        try:
            raw = yaml.safe_load(f.read_text(encoding='utf-8')) or []
        except Exception as e:
            raise ValueError(f"{ACTIONS_FILE} is unreadable: {e}")
        if not isinstance(raw, list) or not all(isinstance(it, dict) for it in raw):
            raise ValueError(f"{ACTIONS_FILE} is not a list of items")
        return raw

    def _save_actions(self, items: List[Dict[str, Any]]) -> None:
        import yaml
        from utils.file_utils import atomic_write_text
        atomic_write_text(self._actions_path(),
                          yaml.safe_dump(items, sort_keys=False, allow_unicode=True))

    # ---- the `actions` action ---------------------------------------------

    def _run_actions(self, op: Any, args: Dict[str, Any]) -> str:
        """The `actions` action. Returns the observation."""
        op = str(op or '').strip()
        try:
            items = self._load_actions()
        except ValueError as e:
            logger.warning(f"[{self.character_name}] actions: {e}")
            return f"ERROR: {e}; nothing was changed"
        if op == 'add':
            return self._actions_add(items, args)
        if op == 'list':
            return self._actions_list(items, str(args.get('which') or 'open').strip())
        if op in ('done', 'edit'):
            ident = str(args.get('id') or '').strip()
            item = next((it for it in items if str(it.get('id')) == ident), None)
            if item is None:
                return f"ERROR: no action item {ident!r}; `list` shows the ids"
            if item.get('done'):
                return (f"ERROR: {ident} was done on {item['done']}, and done is final; "
                        f"to reopen it, `add` a new item with `follows: {ident}`")
            if op == 'done':
                item['done'] = _stamp(_now())
                self._save_actions(items)
                return f"OK: {ident} is done: {item.get('text')}"
            return self._actions_edit(items, item, args)
        return "ERROR: `op` must be add, list, done or edit"

    def _actions_add(self, items: List[Dict[str, Any]], args: Dict[str, Any]) -> str:
        text = str(args.get('text') or '').strip()
        context = str(args.get('context') or '').strip()
        if not text or not context:
            return ("ERROR: an item needs `text` (what to do) and `context` (why it "
                    "exists: who or what prompted it)")
        item: Dict[str, Any] = {'id': self._next_action_id(items), 'text': text,
                                'context': context, 'added': _stamp(_now())}
        due = str(args.get('due') or '').strip()
        if due:
            item['due'] = due
        err = self._set_remind_at(item, args.get('remind_at'))
        if err:
            return err
        follows = str(args.get('follows') or '').strip()
        if follows:
            if not any(str(it.get('id')) == follows for it in items):
                return f"ERROR: `follows` names {follows!r}, which is not an item"
            item['follows'] = follows
        items.append(item)
        self._save_actions(items)
        return f"OK: added {item['id']}: {self._action_line(item)}"

    def _actions_edit(self, items: List[Dict[str, Any]], item: Dict[str, Any],
                      args: Dict[str, Any]) -> str:
        given = [k for k in EDITABLE if k in args]
        if not given:
            return f"ERROR: `edit` changes one or more of {', '.join(EDITABLE)}"
        for k in given:
            if k == 'remind_at':
                err = self._set_remind_at(item, args[k])
                if err:
                    return err
                continue
            value = str(args[k] or '').strip()
            if k in ('text', 'context') and not value:
                return f"ERROR: `{k}` cannot be empty"
            if value:
                item[k] = value
            else:
                item.pop(k, None)
        self._save_actions(items)
        return f"OK: edited {item['id']}: {self._action_line(item)}"

    @staticmethod
    def _set_remind_at(item: Dict[str, Any], value: Any) -> Optional[str]:
        """Set or clear `remind_at`; a new time clears `reminded`. Returns an
        error observation, or None."""
        if value in (None, ''):
            item.pop('remind_at', None)
            item.pop('reminded', None)
            return None
        at = _parse_time(value)
        if at is None:
            return "ERROR: `remind_at` must be a local time, YYYY-MM-DD HH:MM"
        item['remind_at'] = _stamp(at)
        item.pop('reminded', None)
        return None

    @staticmethod
    def _next_action_id(items: List[Dict[str, Any]]) -> str:
        nums = [int(str(it.get('id'))[1:]) for it in items
                if str(it.get('id', '')).startswith('a') and str(it.get('id'))[1:].isdigit()]
        return f"a{max(nums, default=0) + 1}"

    @staticmethod
    def _action_line(item: Dict[str, Any]) -> str:
        parts = [str(item.get('text'))]
        if item.get('due'):
            parts.append(f"due {item['due']}")
        if item.get('remind_at'):
            parts.append(f"remind {item['remind_at']}"
                         + (" (reminded)" if item.get('reminded') else ""))
        if item.get('follows'):
            parts.append(f"follows {item['follows']}")
        if item.get('done'):
            parts.append(f"done {item['done']}")
        return "; ".join(parts) + f"\n  why: {item.get('context', '')}"

    def _actions_list(self, items: List[Dict[str, Any]], which: str) -> str:
        if which not in ('open', 'all'):
            return "ERROR: `which` must be open or all"
        shown = [it for it in items if which == 'all' or not it.get('done')]
        if not shown:
            return f"EMPTY: no {'open ' if which == 'open' else ''}action items"
        return "OK: " + "\n".join(f"{it.get('id')}: {self._action_line(it)}" for it in shown)

    # ---- reminders --------------------------------------------------------

    def _due_reminder(self, items: List[Dict[str, Any]], now: datetime
                      ) -> Optional[Dict[str, Any]]:
        """The open item whose reminder came due earliest and has not fired."""
        due = [(at, it) for it in items
               if not it.get('done') and not it.get('reminded')
               for at in [_parse_time(it.get('remind_at') or '')]
               if at is not None and at <= now]
        return min(due, key=lambda x: x[0])[1] if due else None

    def _fire_due_reminder(self) -> bool:
        """Fire one due reminder: record it, email Bruce, then, with autonomy
        on, give her a turn on it. Returns True when one fired."""
        try:
            items = self._load_actions()
        except ValueError as e:
            logger.warning(f"[{self.character_name}] reminders: {e}")
            return False
        now = _now()
        item = self._due_reminder(items, now)
        if item is None:
            return False
        item['reminded'] = _stamp(now)
        self._save_actions(items)
        at = _parse_time(item['remind_at'])
        late = (f"; it is now {_stamp(now)}, so this is late"
                if now - at > timedelta(minutes=10) else "")
        mailed = self._email_reminder(item)
        logger.info(f"[{self.character_name}] reminder fired: {item['id']} {item['text']!r} "
                    f"({mailed})")
        if getattr(self, '_autonomy_enabled', False):
            text = (f"A reminder Bruce asked for has come due: {item['text']}\n"
                    f"Item {item['id']}, reminder set for {item['remind_at']}{late}.\n"
                    f"Why it exists: {item.get('context', '')}\n"
                    + (f"Due: {item['due']}\n" if item.get('due') else "")
                    + f"Email to Bruce: {mailed}.\nMode: autonomous\n\n"
                    f"Put this reminder on the canvas for Bruce with `display`, adding "
                    f"anything you know that he would want with it.")
            self._process_user_turn(source=self.character_name, text=text, close=False,
                                    autonomous=True, trigger=f"reminder: {item['id']}")
        return True

    def _email_reminder(self, item: Dict[str, Any]) -> str:
        """Email Bruce the reminder. Returns what happened, for her turn and
        the log."""
        from client_ui import mail
        address = os.environ.get('GMAIL_ADDRESS', '').strip()
        password = os.environ.get('GMAIL_APP_PASSWORD', '').strip()
        if not address or not password:
            logger.warning(f"[{self.character_name}] reminder email not sent: "
                           f"GMAIL_ADDRESS or GMAIL_APP_PASSWORD is not set")
            return "not sent (GMAIL_ADDRESS or GMAIL_APP_PASSWORD is not set)"
        body = "\n".join(
            [item['text'], "", f"Why: {item.get('context', '')}"]
            + ([f"Due: {item['due']}"] if item.get('due') else [])
            + [f"Reminder set for {item['remind_at']} (item {item['id']} on "
               f"{self.character_name}'s action list)."])
        rec = mail.send([address], f"[{self.character_name} reminder] {item['text']}", body,
                        account=(address, password))
        if rec['error']:
            return f"failed ({rec['error']})"
        return "dry run, not sent" if rec['dry_run'] else "sent"

    def _render_reminders_block(self, now: Optional[datetime] = None) -> str:
        """Upcoming reminders for the prompt; empty when there are none."""
        now = now or _now()
        try:
            items = self._load_actions()
        except ValueError:
            return ''
        rows = sorted(((at, it) for it in items
                       if not it.get('done') and not it.get('reminded')
                       for at in [_parse_time(it.get('remind_at') or '')]
                       if at is not None and at > now), key=lambda x: x[0])
        if not rows:
            return ''
        lines = ["## Reminders Bruce asked for (on my action list; each fires once, "
                 "by email and on the canvas)"]
        for at, it in rows[:REMINDERS_PROMPT_ITEMS]:
            lines.append(f"- {at:%a %Y-%m-%d %H:%M}: {it['text']} ({it['id']})")
        return "\n".join(lines)
