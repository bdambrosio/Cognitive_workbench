"""What daily.sh does after the daily run, with nobody present.

    python3 workflowsv2/outreach/after_daily.py [--dry-run] [--only <contact id>] [--run-failed]

In this order:

  1. Read the mail that has arrived at bruce@tuuyi.com since the last time
     (info@ is delivered there too), and record what it finds:
       an opt-out      the contact is marked opted out (contacts.opt_out);
       a reply         the person's words are recorded as the page's "Record
                       the reply" button records them, and the reply is read
                       and an answer drafted (`runner.py replies`);
       a message from the website contact form, or an automatic message
                       (an out-of-office answer, a delivery failure): listed
                       in the summary, and nothing else is done with it.
     Any other mail is left alone and not counted.
  2. Send the first message to everyone at Ready to contact who can be
     emailed (runner.emailable) and whose draft has a subject and no flags
     from the writer. Each goes through sending.send, so it is the email the
     page would send and leaves the same record.
  3. Send the follow-up drafted by the daily run (`runner.py followups`) to
     everyone at Initial sent whose follow-up date has come, whose first
     message went by email, and who can still be emailed. It goes as a reply
     in the thread of the first email. A follow-up with flags from the writer
     is held; someone whose first message went another way is left to the
     page.
  4. Email the summary to SUMMARY_TO and print it; daily.sh puts what is
     printed into the report for Jill.

A message is sorted by its headers, not by what it says. A reply that asks
not to be contacted again is therefore recorded as a reply: the reply reader
marks it `declined` (PROSPECT.md §22) and the person presses "Opted out".

The mailbox is opened read-only with SMTP_USER and SMTP_PASS, the same
account the emails are sent from. The folder read is All Mail and not the
inbox, so that mail a person archived before the run is still seen; that
folder also holds what the practice sent, which is passed over. The last
message seen is kept in prospects/mailbox.json; the first run records where
the mailbox stands and reads nothing older.

`--dry-run` prints what would be recorded and sent; it writes nothing and
sends nothing. `--only` sends to that one contact and nobody else.
`--run-failed` says in the summary that the daily run before it failed.
"""
from __future__ import annotations

import argparse
import datetime
import email
import email.policy
import email.utils
import json
import logging
import os
import subprocess
import sys
from email.message import EmailMessage
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2.outreach import contacts, runner, sending               # noqa: E402
from client_ui import mail                                              # noqa: E402
from utils.file_utils import atomic_write_text                          # noqa: E402
from utils.imap_utils import IMAPConnection                             # noqa: E402

logger = logging.getLogger("outreach.after_daily")

MODEL = REPO / "measure/models/local_qwen38flashnext.yaml"
SUMMARY_TO = "bruce.dambrosio@gmail.com"
#: What the website's Worker (contact-worker/worker.js) puts on its mail.
FORM_FROM = "contact@tuuyi.com"
OPTOUT_SUBJECT = "[tuuyi.com] opt-out:"
FORM_SUBJECT = "[tuuyi.com]"
#: The address and subject of the List-Unsubscribe mailto in every email sent (sending.compose).
UNSUBSCRIBE_TO, UNSUBSCRIBE_SUBJECT = "info@tuuyi.com", "unsubscribe"
#: Gmail's folder of everything received and sent, archived or not.
FOLDER = '"[Gmail]/All Mail"'


def today() -> str:
    return datetime.date.today().isoformat()


# ---- the mailbox ----------------------------------------------------------------

def new_mail(state_file: Path) -> Tuple[List[EmailMessage], Dict[str, int]]:
    """The messages that reached FOLDER after the one recorded in
    `state_file`, oldest first, and the state to record once they are dealt
    with. With no state, or a mailbox whose numbering was reset, there are no
    messages and the state is where the mailbox stands now."""
    state = runner._read_json(state_file) or {}
    with IMAPConnection(os.environ["SMTP_USER"], os.environ["SMTP_PASS"]) as conn:
        conn.select(FOLDER, readonly=True)
        validity = int(conn.response("UIDVALIDITY")[1][0])
        _, found = conn.uid("search", None, "ALL")
        uids = sorted(int(u) for u in found[0].split())
        top = uids[-1] if uids else 0
        if state.get("uidvalidity") != validity:
            return [], {"uidvalidity": validity, "last_uid": top}
        messages = []
        for uid in (u for u in uids if u > state["last_uid"]):
            _, got = conn.uid("fetch", str(uid), "(BODY.PEEK[])")
            raw = next((part[1] for part in got if isinstance(part, tuple)), None)
            if raw:
                messages.append(email.message_from_bytes(raw, policy=email.policy.default))
        return messages, {"uidvalidity": validity, "last_uid": top}


def _address(header: Any) -> str:
    return email.utils.parseaddr(str(header or ""))[1].strip().lower()


def _failed_recipient(msg: EmailMessage) -> str:
    """The address a delivery report says could not be reached, or ''."""
    for part in msg.walk():
        if part.get_content_type() == "message/delivery-status":
            for block in part.get_payload():
                recipient = str(block.get("Final-Recipient") or "")
                if recipient:
                    return recipient.split(";")[-1].strip().lower()
    return _address(msg.get("X-Failed-Recipients"))


def _reply_text(msg: EmailMessage) -> str:
    """The person's words: the plain-text body without the quoted message
    below it. '' when the message has no plain-text part."""
    part = msg.get_body(preferencelist=("plain",))
    if part is None:
        return ""
    lines = part.get_content().replace("\r\n", "\n").rstrip().split("\n")
    while lines and (not lines[-1].strip() or lines[-1].lstrip().startswith(">")):
        lines.pop()
    return "\n".join(lines).strip()


def own_addresses() -> List[str]:
    """The addresses the practice sends from."""
    return [a for a in (_address(os.environ.get(k)) for k in ("SMTP_USER", "MAIL_FROM")) if a]


def sent_ids() -> Dict[str, str]:
    """The Message-ID of every email recorded as sent, with the contact it went to."""
    return {n["message_id"]: c["id"] for c in contacts.everyone() for n in c.get("notes") or []
            if n.get("message_id")}


def sort_message(msg: EmailMessage, sent: Dict[str, str]) -> Optional[Dict[str, str]]:
    """What one message is, from its headers: `{kind, ...}` with kind
    `optout` (and `address`), `form` (and `subject`, `address`), `automatic`
    (and `subject`, `address`: the sender, or for a delivery failure the
    address that failed), or `reply` (and `contact`: the id, `text`). None
    for mail that is none of these, which includes what the practice sent."""
    sender, subject = _address(msg.get("From")), str(msg.get("Subject") or "").strip()
    if sender in own_addresses():
        return None
    if sender == FORM_FROM and subject.startswith(OPTOUT_SUBJECT):
        return {"kind": "optout", "address": subject[len(OPTOUT_SUBJECT):].strip().lower()}
    if sender == FORM_FROM and subject.startswith(FORM_SUBJECT) and msg.get("Reply-To"):
        return {"kind": "form", "subject": subject[len(FORM_SUBJECT):].strip(), "address": _address(msg.get("Reply-To"))}
    to = [a.lower() for _, a in email.utils.getaddresses([str(msg.get("To") or "")])]
    if UNSUBSCRIBE_TO in to and subject.lower() == UNSUBSCRIBE_SUBJECT:
        return {"kind": "optout", "address": sender}
    if msg.get_content_type() == "multipart/report":
        return {"kind": "automatic", "subject": subject, "address": _failed_recipient(msg) or sender}
    if str(msg.get("Auto-Submitted") or "no").strip().lower() != "no":
        return {"kind": "automatic", "subject": subject, "address": sender}
    thread = " ".join(str(msg.get(h) or "") for h in ("In-Reply-To", "References")).split()
    cid = next((sent[i] for i in thread if i in sent), None)
    if cid is None:
        person = contacts.find_by_email(sender)
        cid = person["id"] if person else None
    if cid is None:
        return None
    return {"kind": "reply", "contact": cid, "text": _reply_text(msg)}


def read_mailbox(messages: List[EmailMessage], dry: bool) -> Dict[str, List[str]]:
    """Sort each message and record the opt-outs and replies (nothing is
    recorded when `dry`). Returns, per heading of the summary, one line for
    each message."""
    out: Dict[str, List[str]] = {"optouts": [], "replies": [], "forms": [], "automatic": [], "attention": []}
    sent = sent_ids()
    for msg in messages:
        what = sort_message(msg, sent)
        if what is None:
            continue
        if what["kind"] == "optout":
            person = contacts.find_by_email(what["address"])
            if person is None:
                out["optouts"].append(f"{what['address']} (not the address of any contact; nothing recorded)")
                continue
            if not dry and not contacts.opted_out(person["id"]):
                contacts.opt_out(person["id"], today())
            out["optouts"].append(f"{person['name']} <{what['address']}>")
        elif what["kind"] == "form":
            out["forms"].append(f"{what['subject']} <{what['address']}>")
        elif what["kind"] == "automatic":
            out["automatic"].append(f"{what['address']}: {what['subject']}")
        else:
            name = contacts.get(what["contact"])["name"]
            if not what["text"]:
                out["attention"].append(f"A reply from {name} has no plain-text part; read it in the mailbox "
                                        f"and record it on the page.")
            elif contacts.opted_out(what["contact"]):
                out["attention"].append(f"{name}, who opted out, wrote again; read it in the mailbox.")
            elif dry or sending.record_reply(what["contact"], what["text"], today()):
                out["replies"].append(name)
    return out


# ---- sending --------------------------------------------------------------------

def _send_one(c: Dict[str, Any], which: str, drafted: Dict[str, Any], dry: bool,
              out: Dict[str, List[str]]) -> None:
    """Send `drafted` (draft.json or followup.json) to the contact `c` as
    `which`, unless it has flags or lacks text, and add one line to `out`
    under `sent`, `held` or `failed`. A follow-up needs no subject: it goes
    as a reply to the first email."""
    cid, who = c["id"], f"{c['name']} <{c['email']}>"
    subject = drafted.get("edited_subject") or drafted.get("subject") or ""
    message = drafted.get("edited_message") or drafted.get("message") or ""
    if drafted.get("flags"):
        out["held"].append(f"{who}: {'; '.join(str(f) for f in drafted['flags'])}")
        return
    if not message.strip() or (which == "draft" and not subject.strip()):
        out["held"].append(f"{who}: the draft has no {'subject' if message.strip() else 'message'}")
        return
    try:
        if dry:
            sending.compose(cid, which, subject, message)
            rec = {"sent": True}
        else:
            rec = sending.send(cid, which, subject, message, today())
    except (sending.Refused, contacts.ContactError) as e:
        out["failed"].append(f"{who}: {e}")
        return
    (out["sent"] if rec["sent"] else out["failed"]).append(who if rec["sent"] else f"{who}: {rec['note']}")


def send_ready(dry: bool, only: Optional[str] = None) -> Dict[str, List[str]]:
    """Send the first message to each person at Ready to contact who can be
    emailed and whose draft has a subject and no flags from the writer.
    Returns, per heading of the summary, one line for each person: `sent`,
    `held` (a draft a person has to look at first), `failed`."""
    out: Dict[str, List[str]] = {"sent": [], "held": [], "failed": []}
    for c in contacts.entries():
        cid = c["id"]
        if contacts.stage_of(c) != "Ready to contact" or (only and cid != only):
            continue
        d = runner.DATA / cid
        if not runner.emailable(cid, runner._read_json(d / "qualification.json") or {}):
            continue
        _send_one(c, "draft", runner._read_json(d / "draft.json") or {}, dry, out)
    return out


def send_followups(dry: bool, only: Optional[str] = None) -> Dict[str, List[str]]:
    """Send the drafted follow-up to each person at Initial sent whose
    follow-up date has come, whose first message went by email, and who can
    still be emailed. Returns lines under `sent`, `held`, `failed`, as
    send_ready does."""
    out: Dict[str, List[str]] = {"sent": [], "held": [], "failed": []}
    for c in contacts.entries():
        cid = c["id"]
        if not contacts.due(c, "Initial sent", today()) or (only and cid != only):
            continue
        if not any(str(n.get("title") or "").startswith(contacts.SENT_NOTE) and n.get("message_id")
                   for n in contacts.notes(cid)):
            continue                     # the first message did not go by email; the page lists them
        d = runner.DATA / cid
        if not runner.emailable(cid, runner._read_json(d / "qualification.json") or {}):
            continue
        _send_one(c, "followup", runner._read_json(d / "followup.json") or {}, dry, out)
    return out


def emailed_today() -> int:
    """How many emails are recorded as sent today, by this run or from the page."""
    return sum(1 for c in contacts.everyone() for n in c.get("notes") or []
               if n.get("channel") == "email" and n.get("date") == today())


# ---- the summary ----------------------------------------------------------------

def summary(found: Dict[str, List[str]], sent: Dict[str, List[str]], problems: List[str], dry: bool) -> Tuple[str, str]:
    """The subject and text of the summary."""
    due = [c["name"] for c in contacts.entries() if contacts.due(c, "Initial sent", today())]
    n_sent = len(sent["sent"]) if dry else emailed_today()
    subject = (f"Tuuyi outreach {today()}: {n_sent} {'to send' if dry else 'sent'}, {len(found['replies'])} replies, "
               f"{len(found['optouts'])} opt-outs, {len(found['forms'])} from the contact form")
    rows: List[str] = [f"NOT DONE: {p}" for p in problems + sent["failed"]]
    for title, lines in (
            (f"Emails that would be sent: {n_sent}" if dry else
             f"Emails sent today: {n_sent} ({len(sent['sent'])} by this run)", sent["sent"]),
            (f"Drafts not sent, for you to look at on the page: {len(sent['held'])}", sent["held"]),
            (f"Opt-outs: {len(found['optouts'])}", found["optouts"]),
            (f"Replies: {len(found['replies'])}", found["replies"]),
            (f"Messages from the contact form: {len(found['forms'])}", found["forms"]),
            (f"Automatic messages (out of office, delivery failures): {len(found['automatic'])}", found["automatic"]),
            (f"Follow-ups due, not sent automatically: {len(due)}", due)):
        rows += ["", title + ("." if not lines else ":")] + [f"  {line}" for line in lines]
    rows += [""] + found["attention"] + ["The page: http://127.0.0.1:8810"]
    return subject, "\n".join(rows).strip() + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="print what would be recorded and sent; change nothing")
    ap.add_argument("--only", default=None, help="send to this one contact, by id, and to nobody else")
    ap.add_argument("--run-failed", action="store_true", help="the daily run before this step failed; say so")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    dry = args.dry_run
    problems: List[str] = ["the daily run failed; see prospects/daily.log"] if args.run_failed else []
    if mail.dry_run() and not dry:
        problems.append("SMTP_PASS is not set, so no email could be sent")
    state_file = runner.DATA / "mailbox.json"
    found = {"optouts": [], "replies": [], "forms": [], "automatic": [], "attention": []}
    try:
        messages, state = new_mail(state_file)
        found = read_mailbox(messages, dry)
        if not dry:
            atomic_write_text(state_file, json.dumps(state) + "\n")
    except Exception as e:                                             # noqa: BLE001
        logger.exception("the mailbox could not be read")
        problems.append(f"the mailbox could not be read ({type(e).__name__}: {e})")
    if found["replies"] and not dry:
        read = subprocess.run([sys.executable, str(HERE / "runner.py"), "replies", "--model", str(MODEL)],
                              cwd=str(REPO), capture_output=True, text=True)
        if read.returncode:
            logger.error("runner.py replies failed: %s", read.stderr[-2000:])
            problems.append("the new replies were recorded but could not be read; the next daily run reads them")
    sent = send_ready(dry, args.only)
    for k, lines in send_followups(dry, args.only).items():
        sent[k] += [f"follow-up to {line}" for line in lines]
    subject, text = summary(found, sent, problems, dry)
    if dry:
        print(f"DRY RUN: nothing was recorded or sent.\n\n{subject}\n\n{text}")
        return 0
    rec = mail.send([SUMMARY_TO], subject, text)
    if rec["error"] or rec["dry_run"]:
        problems.append("the summary email could not be sent")
        print("NOT DONE: the summary email could not be sent.")
    print(text)
    return 1 if problems or sent["failed"] else 0


if __name__ == "__main__":
    sys.exit(main())
