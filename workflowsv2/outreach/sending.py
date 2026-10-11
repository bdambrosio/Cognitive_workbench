"""Sending one outreach email and recording one reply.

The one copy of each, used by the page (app.py), where a person presses a
button, and by after_daily.py, which runs after the daily run with nobody
present. Both therefore send the same email and leave the same record.

Email needs SMTP_USER, SMTP_PASS, MAIL_FROM and POSTAL_ADDRESS in the
environment (~/.config/tuuyi-outreach.env). Without SMTP_PASS a send is a dry
run: nothing goes out and nothing is recorded.
"""
from __future__ import annotations

import datetime
import os
import sys
from pathlib import Path
from typing import Any, Dict, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.outreach import contacts, runner                        # noqa: E402
from client_ui import mail                                              # noqa: E402

FOLLOWUP_AFTER_DAYS = 6
CLOSE_AFTER_DAYS = 7
OPTOUT_URL = "https://tuuyi.com/optout"
OPTOUT_API = "https://tuuyi.com/api/optout"


class Refused(RuntimeError):
    """The email was not sent. `status` is the HTTP status the page answers
    with: 409 the person may not be emailed, 422 the request lacks something,
    502 the mail server refused it."""

    def __init__(self, status: int, why: str):
        super().__init__(why)
        self.status = status


def refusal(cid: str, q: Optional[Dict[str, Any]] = None) -> Optional[str]:
    """Why this person may not be emailed, or None when they may. `q` is
    their qualification; without it the runner's record is read."""
    if contacts.opted_out(cid):
        return "they asked not to be contacted again"
    if not contacts.get(cid).get("email"):
        return "no email address"
    if q is None:
        q = runner._read_json(runner.DATA / cid / "qualification.json") or {}
    if not runner.email_allowed(q):
        return f"their region ({q.get('region') or 'not recorded'}) is contacted on LinkedIn"
    if not os.environ.get("POSTAL_ADDRESS", "").strip():
        return "POSTAL_ADDRESS is not set, and an email must carry it"
    return None


def footer(address: str) -> str:
    """The text every email ends with: the practice's name and postal address, and how to opt out."""
    return (f"--\nTuuyi · {os.environ.get('POSTAL_ADDRESS', '').strip()}\n"
            f"If you'd rather not hear from me again: {OPTOUT_URL}?e={address}")


def compose(cid: str, which: str, subject: str, message: str) -> Dict[str, Any]:
    """The email exactly as it would go: `from`, `to`, `subject`, `text`,
    `headers`, and `dry_run`. `which` is "draft" (the first message) or
    "followup". Raises Refused when it may not or cannot be sent."""
    why_not = refusal(cid)
    if why_not:
        raise Refused(409, f"not sent: {why_not}")
    if which not in ("draft", "followup"):
        raise Refused(422, "which must be draft or followup")
    if not message.strip() or (which == "draft" and not subject.strip()):
        raise Refused(422, "the subject and the message are needed")
    address = contacts.get(cid)["email"]
    headers = {"List-Unsubscribe": f"<mailto:info@tuuyi.com?subject=unsubscribe>, <{OPTOUT_API}?e={address}>",
               "List-Unsubscribe-Post": "List-Unsubscribe=One-Click"}
    subject = subject.strip()
    if which == "followup":
        first = [n for n in contacts.notes(cid)
                 if str(n.get("title") or "").startswith(contacts.SENT_NOTE) and n.get("message_id")]
        if first:                      # a reply in the thread of the first email
            headers.update({"In-Reply-To": first[-1]["message_id"], "References": first[-1]["message_id"]})
            subject = subject or f"Re: {first[-1].get('subject', '')}".strip()
        if not subject:
            raise Refused(422, "the first message did not go by email: give the follow-up a subject")
    return {"from": os.environ.get("MAIL_FROM") or os.environ.get("SMTP_USER") or "(not set)",
            "to": address, "subject": subject, "text": message.strip() + "\n\n" + footer(address),
            "headers": headers, "dry_run": mail.dry_run()}


def send(cid: str, which: str, subject: str, message: str, on: str) -> Dict[str, Any]:
    """Send the email `compose` gives and record it: a note "Message sent
    <on>" or "Follow-up sent <on>" holding the message without the footer,
    with the channel, address, subject and Message-ID beside it, and the
    entry moved to the sent stage. Returns the email with `sent` true.

    Nothing is recorded unless the mail server accepted the email: a refusal
    raises Refused, and a dry run returns `sent` false."""
    email = compose(cid, which, subject, message)
    shown = {k: v for k, v in email.items() if k != "headers"}
    rec = mail.send([email["to"]], email["subject"], email["text"], headers=email["headers"])
    if rec["error"]:
        raise Refused(502, f"not sent: {rec['error']}")
    if rec["dry_run"]:
        return {**shown, "sent": False, "note": "dry run: SMTP_PASS is not set, so nothing was sent or recorded"}
    title, stage, action, days = ((contacts.SENT_NOTE, "Initial sent", "Follow up if no response", FOLLOWUP_AFTER_DAYS)
                                  if which == "draft" else
                                  (contacts.FOLLOWUP_NOTE, "Follow-up sent", "Close if no response", CLOSE_AFTER_DAYS))
    contacts.create_note(cid, f"{title} {on}", message.strip(), channel="email",
                         to=email["to"], subject=email["subject"], message_id=rec["message_id"])
    contacts.upsert_entry(cid, {"stage": stage, "last_contact": on, "next_action": action,
                                "next_action_date": (datetime.date.today() + datetime.timedelta(
                                    days=days)).isoformat()})
    return {**shown, "sent": True}


def record_reply(cid: str, text: str, on: str) -> bool:
    """The person has answered. Record their exact words as a note "Reply
    received <on>" and move the entry to Replied (an entry already in
    conversation stays there). Returns False, and records nothing, when this
    reply is already recorded for them."""
    # compared without regard to spacing: notes carried over from Attio were re-spaced
    if any(n["text"].split() == text.split() for n in contacts.note_texts(cid, contacts.REPLY_NOTE)):
        return False
    entry = contacts.entry_of(cid)
    contacts.create_note(cid, f"{contacts.REPLY_NOTE} {on}", text.strip())
    values: Dict[str, Any] = {"last_contact": on, "next_action": "Read the reply", "next_action_date": on}
    if entry is None or contacts.stage_of(entry) != "Conversation":
        values["stage"] = "Replied"
    contacts.upsert_entry(cid, values)
    return True
