"""The practice's page for the outreach workflow. Local only.

    python3 workflowsv2/outreach/app.py [--port 8810] [--model <yaml>]

The page keeps nothing of its own. What it shows comes from the contacts
(contacts.py: who is at which stage, the notes of each exchange) and from the
runner's records under prospects/ (the brief, the evidence, the draft); what
it writes goes to the same two places, and only when a person presses a
button. A LinkedIn message is copied by the person into LinkedIn, who then
presses Sent. An email is sent only by /api/send_email with `confirm` set,
which the page calls when the person presses Confirm send after reading the
whole email as it will go; nothing else here sends mail.

Email needs SMTP_USER, SMTP_PASS and MAIL_FROM (bruce@tuuyi.com) and
POSTAL_ADDRESS in the environment; daily.sh reads them from
~/.config/tuuyi-outreach.env. Without SMTP_PASS the send is a dry run and
nothing is recorded.

It binds to 127.0.0.1 and has no login. It is not part of the client site.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from fastapi import FastAPI, HTTPException                              # noqa: E402
from fastapi.responses import FileResponse                              # noqa: E402
from pydantic import BaseModel                                          # noqa: E402

from workflowsv2.outreach import contacts, runner, schemas                # noqa: E402
from utils.file_utils import atomic_write_text                          # noqa: E402
from client_ui import mail                                              # noqa: E402

logger = logging.getLogger("outreach.app")

MODEL = REPO / "measure/models/local_qwen38flashnext.yaml"
RUN_LOG = runner.DATA / "run.log"
#: The sections of the page, in the order they are shown, and the stage each
#: takes its cards from. A person at a sent stage is in the section of that
#: stage when the next action date has come, and in "waiting" until then.
GROUPS = ("replies", "followup", "ready", "noresponse", "waiting", "qualified", "research")
GROUP_OF = {"Ready to contact": "ready", "Qualified": "qualified", "Research": "research",
            "Initial sent": "followup", "Follow-up sent": "noresponse",
            "Replied": "replies", "Conversation": "replies"}
FOLLOWUP_AFTER_DAYS = 6
CLOSE_AFTER_DAYS = 7
OPTOUT_URL = "https://tuuyi.com/optout"
OPTOUT_API = "https://tuuyi.com/api/optout"

app = FastAPI()
_run: Dict[str, Any] = {"proc": None, "what": None, "started": None}


def today() -> str:
    return datetime.date.today().isoformat()


def _local(cid: str) -> Dict[str, Any]:
    """The runner's record of this person, where there is one."""
    d = runner.DATA / cid
    out: Dict[str, Any] = {}
    for key, f in (("qualification", "qualification.json"), ("draft", "draft.json"),
                   ("followup", "followup.json"), ("replies", "replies.json")):
        if (d / f).is_file():
            out[key] = json.loads((d / f).read_text(encoding="utf-8"))
    if (d / "brief.md").is_file():
        out["brief"] = (d / "brief.md").read_text(encoding="utf-8")
    return out


def _text(c: Dict[str, Any], field: str) -> Optional[str]:
    return (c.get("entry") or {}).get(field) or None


@app.get("/")
def page() -> FileResponse:
    return FileResponse(HERE / "static" / "outreach.html")


@app.get("/api/queue")
def queue() -> Dict[str, Any]:
    entries = contacts.entries()
    shown: List[tuple] = []
    counts: Dict[str, int] = {}
    for e in entries:
        stage = contacts.stage_of(e)
        counts[stage] = counts.get(stage, 0) + 1
        if stage not in GROUP_OF:
            continue
        later = _text(e, "next_action_date")
        if stage == "Ready to contact" and later and later > today():
            counts["(put off)"] = counts.get("(put off)", 0) + 1
            continue
        group = GROUP_OF[stage]
        if group in ("followup", "noresponse") and not contacts.due(e, stage, today()):
            group = "waiting"
        shown.append((e, stage, group))
    cards = [_card(*t) for t in shown]
    cards.sort(key=lambda c: GROUPS.index(c["group"]))
    return {"cards": cards, "counts": counts,
            "kinds": [k for k in schemas.TYPES if k != "none"]}


def _card(e: Dict[str, Any], stage: str, group: str) -> Dict[str, Any]:
    """One contact in the outreach list as the page shows it."""
    later = _text(e, "next_action_date")
    name = e["name"]
    local = _local(e["id"])
    draft = local.get("draft") or {}
    follow = local.get("followup") or {}
    rid = e["id"]
    sent = [{"what": what, **n} for what, title in (("First message", contacts.SENT_NOTE),
                                                    ("Follow-up", contacts.FOLLOWUP_NOTE),
                                                    ("Reply", contacts.REPLY_NOTE),
                                                    ("Answer", contacts.ANSWER_NOTE))
            for n in contacts.note_texts(rid, title, e["notes"])]
    return {"record_id": rid, "name": name, "stage": stage, "group": group,
            "last_contact": _text(e, "last_contact"), "due": later,
            "next_action": _text(e, "next_action"),
            "followup": follow.get("edited_message") or follow.get("message"),
            "followup_idea": follow.get("idea"), "sent": sent,
            "replies": local.get("replies") or [],
            "contact": {k: e.get(k) or "" for k in contacts.PERSON_FIELDS},
            "how_known": _text(e, "notes") or "",
            "title": e.get("title"), "linkedin": e.get("linkedin"),
            "category": _text(e, "category"),
            "region": (local.get("qualification") or {}).get("region") or "unknown",
            "fit_rationale": _text(e, "fit_rationale"),
            "message": draft.get("edited_message") or draft.get("message"),
            "subject": draft.get("edited_subject") or draft.get("subject") or "",
            "opted_out": contacts.opted_out(rid),
            "email_ok": _email_refusal(rid, local) is None,
            "email_why_not": _email_refusal(rid, local),
            "flags": ((local.get("qualification") or {}).get("flags", []) + draft.get("flags", [])
                      if group in ("ready", "qualified", "research") else follow.get("flags", [])
                      if group == "followup" else []),
            "brief": local.get("brief")}


class Sent(BaseModel):
    record_id: str
    name: str
    message: str


@app.post("/api/sent")
def sent(body: Sent) -> Dict[str, Any]:
    """The person has sent the message by hand. Record the exact text and move
    the entry to Initial sent."""
    try:
        contacts.create_note(body.record_id, f"{contacts.SENT_NOTE} {today()}", body.message)
        contacts.upsert_entry(body.record_id, {"stage": "Initial sent", "last_contact": today(),
                                            "next_action": "Follow up if no response",
                                            "next_action_date": (datetime.date.today() + datetime.timedelta(
                                                days=FOLLOWUP_AFTER_DAYS)).isoformat()})
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True}


@app.post("/api/followup_sent")
def followup_sent(body: Sent) -> Dict[str, Any]:
    """The person has sent the follow-up by hand. It is the only one: the next
    action is to close the approach if nothing comes back."""
    try:
        contacts.create_note(body.record_id, f"{contacts.FOLLOWUP_NOTE} {today()}", body.message)
        contacts.upsert_entry(body.record_id, {"stage": "Follow-up sent", "last_contact": today(),
                                            "next_action": "Close if no response",
                                            "next_action_date": (datetime.date.today() + datetime.timedelta(
                                                days=CLOSE_AFTER_DAYS)).isoformat()})
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True}


class Replied(BaseModel):
    record_id: str
    reply: str


@app.post("/api/replied")
def replied(body: Replied) -> Dict[str, Any]:
    """The person has answered. Record their exact words and move the entry to
    Replied (an entry already in conversation stays there); then have the
    reply read, unless a run is going, in which case the next run reads it."""
    if not body.reply.strip():
        raise HTTPException(422, "paste the reply")
    try:
        # compared without regard to spacing: notes carried over from Attio were re-spaced
        if any(n["text"].split() == body.reply.split() for n in contacts.note_texts(body.record_id, contacts.REPLY_NOTE)):
            raise HTTPException(409, "this reply is already recorded for this person")
        entry = contacts.entry_of(body.record_id)
        contacts.create_note(body.record_id, f"{contacts.REPLY_NOTE} {today()}", body.reply.strip())
        values: Dict[str, Any] = {"last_contact": today(), "next_action": "Read the reply",
                                  "next_action_date": today()}
        if entry is None or contacts.stage_of(entry) != "Conversation":
            values["stage"] = "Replied"
        contacts.upsert_entry(body.record_id, values)
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    reading = not _running()
    if reading:
        _start([[sys.executable, str(HERE / "runner.py"), "replies", "--model", str(MODEL)]], "reading the reply")
    return {"ok": True, "reading": reading}


class Answered(BaseModel):
    record_id: str
    name: str
    note_id: str                       # the reply that is answered
    message: str


@app.post("/api/answer_sent")
def answer_sent(body: Answered) -> Dict[str, Any]:
    """The person has sent the answer by hand. Record the exact text; the
    exchange is now a conversation."""
    try:
        contacts.create_note(body.record_id, f"{contacts.ANSWER_NOTE} {today()}", body.message)
        contacts.upsert_entry(body.record_id, {"stage": "Conversation", "last_contact": today(),
                                            "next_action": "Wait for their reply",
                                            "next_action_date": (datetime.date.today() + datetime.timedelta(
                                                days=CLOSE_AFTER_DAYS)).isoformat()})
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    _in_reply(body.name, body.note_id, {"answer_sent": today()})
    return {"ok": True}


def _in_reply(name: str, note_id: str, values: Dict[str, Any]) -> None:
    """Set values in the runner's record of one reply."""
    f = runner.DATA / runner.slug(name) / "replies.json"
    have = json.loads(f.read_text(encoding="utf-8")) if f.is_file() else []
    rec = next((r for r in have if r.get("note_id") == note_id), None)
    if rec is None:
        raise HTTPException(404, "no such reply on record for this person")
    rec.update(values)
    atomic_write_text(f, json.dumps(have, indent=1, ensure_ascii=False) + "\n")


class Faq(BaseModel):
    name: str
    note_id: str                       # the reply whose answer filled [answer needed]
    add: bool                          # False: the practice cancelled; nothing is written
    question: str = ""
    answer: str = ""


@app.post("/api/faq")
def faq(body: Faq) -> Dict[str, Any]:
    """Add the entry the person edited to ANSWERS.md, or record that they
    cancelled it. Either way the offer is not made again for this reply."""
    if body.add:
        if not body.question.strip() or not body.answer.strip():
            raise HTTPException(422, "a question and an answer are needed")
        f = runner.ANSWERS_PATH
        have = f.read_text(encoding="utf-8") if f.is_file() else ""
        entry = (f"## {' '.join(body.question.split())}\n\n{body.answer.strip()}\n\n"
                 f"Source: Bruce's answer to {body.name}, {today()}. Checked {today()}.\n")
        atomic_write_text(f, have.rstrip("\n") + "\n\n" + entry)
    _in_reply(body.name, body.note_id, {"faq": "added" if body.add else "cancelled", "faq_at": today()})
    return {"ok": True}


class Record(BaseModel):
    record_id: str


@app.post("/api/conversation")
def conversation(body: Record) -> Dict[str, Any]:
    try:
        contacts.upsert_entry(body.record_id, {"stage": "Conversation"})
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True}


def _email_refusal(cid: str, local: Optional[Dict[str, Any]] = None) -> Optional[str]:
    """Why this person may not be emailed, or None when they may."""
    if contacts.opted_out(cid):
        return "they asked not to be contacted again"
    if not contacts.get(cid).get("email"):
        return "no email address"
    q = (local if local is not None else _local(cid)).get("qualification") or {}
    if not runner.email_allowed(q):
        return f"their region ({q.get('region') or 'not recorded'}) is contacted on LinkedIn"
    if not os.environ.get("POSTAL_ADDRESS", "").strip():
        return "POSTAL_ADDRESS is not set, and an email must carry it"
    return None


def footer(address: str) -> str:
    """The text every email ends with: the practice's name and postal address, and how to opt out."""
    return (f"--\nTuuyi · {os.environ.get('POSTAL_ADDRESS', '').strip()}\n"
            f"If you'd rather not hear from me again: {OPTOUT_URL}?e={address}")


class Email(BaseModel):
    record_id: str
    which: str = "draft"               # or "followup"
    subject: str
    message: str
    confirm: bool = False


@app.post("/api/send_email")
def send_email(body: Email) -> Dict[str, Any]:
    """Without `confirm`: the email exactly as it would go, for the person to
    read. With `confirm`: send that email and record it as /api/sent or
    /api/followup_sent would, with the channel and Message-ID beside the text."""
    try:
        why_not = _email_refusal(body.record_id)
        if why_not:
            raise HTTPException(409, f"not sent: {why_not}")
        if body.which not in ("draft", "followup"):
            raise HTTPException(422, "which must be draft or followup")
        if not body.message.strip() or (body.which == "draft" and not body.subject.strip()):
            raise HTTPException(422, "the subject and the message are needed")
        address = contacts.get(body.record_id)["email"]
        headers = {"List-Unsubscribe": f"<mailto:info@tuuyi.com?subject=unsubscribe>, <{OPTOUT_API}?e={address}>",
                   "List-Unsubscribe-Post": "List-Unsubscribe=One-Click"}
        subject = body.subject.strip()
        if body.which == "followup":
            first = [n for n in contacts.notes(body.record_id)
                     if str(n.get("title") or "").startswith(contacts.SENT_NOTE) and n.get("message_id")]
            if first:                  # a reply in the thread of the first email
                headers.update({"In-Reply-To": first[-1]["message_id"], "References": first[-1]["message_id"]})
                subject = subject or f"Re: {first[-1].get('subject', '')}".strip()
            if not subject:
                raise HTTPException(422, "the first message did not go by email: give the follow-up a subject")
        text = body.message.strip() + "\n\n" + footer(address)
        shown = {"from": os.environ.get("MAIL_FROM") or os.environ.get("SMTP_USER") or "(not set)",
                 "to": address, "subject": subject, "text": text, "dry_run": mail.dry_run()}
        if not body.confirm:
            return shown
        rec = mail.send([address], subject, text, headers=headers)
        if rec["error"]:
            raise HTTPException(502, f"not sent: {rec['error']}")
        if rec["dry_run"]:
            return {**shown, "sent": False, "note": "dry run: SMTP_PASS is not set, so nothing was sent or recorded"}
        title, stage, action, days = ((contacts.SENT_NOTE, "Initial sent", "Follow up if no response", FOLLOWUP_AFTER_DAYS)
                                      if body.which == "draft" else
                                      (contacts.FOLLOWUP_NOTE, "Follow-up sent", "Close if no response", CLOSE_AFTER_DAYS))
        contacts.create_note(body.record_id, f"{title} {today()}", body.message.strip(), channel="email",
                             to=address, subject=subject, message_id=rec["message_id"])
        contacts.upsert_entry(body.record_id, {"stage": stage, "last_contact": today(), "next_action": action,
                                               "next_action_date": (datetime.date.today() + datetime.timedelta(
                                                   days=days)).isoformat()})
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {**shown, "sent": True}


@app.post("/api/opted_out")
def opted_out(body: Record) -> Dict[str, Any]:
    """The person asked not to be contacted again (the opt-out page mails info@)."""
    try:
        contacts.opt_out(body.record_id, today())
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True}


@app.post("/api/find_email")
def find_email(body: Record) -> Dict[str, Any]:
    """Look up the contact's work address with the address-finding services;
    an address is recorded only when the evidence shows they work there now."""
    if contacts.opted_out(body.record_id):
        raise HTTPException(409, "they asked not to be contacted again")
    try:
        return runner.look_up_email(body.record_id)
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))


class Skip(BaseModel):
    record_id: str
    reason: str


@app.post("/api/skip")
def skip(body: Skip) -> Dict[str, Any]:
    try:
        contacts.not_pursuing(body.record_id, f"Skipped by the practice. {body.reason.strip()}", today())
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True}


class Later(BaseModel):
    record_id: str
    until: str


@app.post("/api/later")
def later(body: Later) -> Dict[str, Any]:
    try:
        contacts.upsert_entry(body.record_id, {"next_action_date": body.until})
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True}


class Edit(BaseModel):
    name: str
    message: str
    subject: Optional[str] = None      # for a draft that may go by email
    which: str = "draft"               # or "followup", or "answer" with the reply's note_id
    note_id: str = ""


@app.post("/api/edit")
def edit(body: Edit) -> Dict[str, Any]:
    """Keep an edited message in the runner's record of the draft or of the
    follow-up, beside the text the model wrote."""
    if body.which == "answer":
        _in_reply(body.name, body.note_id, {"edited_answer": body.message, "edited_at": today()})
        return {"ok": True}
    if body.which not in ("draft", "followup"):
        raise HTTPException(422, "which must be draft, followup or answer")
    f = runner.DATA / runner.slug(body.name) / f"{body.which}.json"
    if not f.is_file():
        raise HTTPException(404, f"no {body.which} on record for this person")
    d = json.loads(f.read_text(encoding="utf-8"))
    d["edited_message"], d["edited_at"] = body.message, today()
    if body.subject is not None:
        d["edited_subject"] = body.subject
    atomic_write_text(f, json.dumps(d, indent=1, ensure_ascii=False) + "\n")
    return {"ok": True}


class Add(BaseModel):
    name: str
    title: str = ""
    firm: str = ""
    domain: str = ""
    email: str = ""
    linkedin: str = ""
    how_known: str = ""
    pasted: str = ""
    pasted_source: str = ""


@app.post("/api/add")
def add(body: Add) -> Dict[str, Any]:
    """Hand a name to the workflow: the person goes into the outreach list at
    Research, and the next run works on them."""
    name = body.name.strip()
    if not name:
        raise HTTPException(422, "a name is needed")
    try:
        person = contacts.find_person(name) or contacts.create_person(
            name, body.title, body.linkedin, firm=body.firm, domain=body.domain, email=body.email)
        rid = person["id"]
        if contacts.opted_out(rid):
            raise HTTPException(409, f"{name} asked not to be contacted again")
        entry = contacts.entry_of(rid)
        if entry is not None and contacts.stage_of(entry) != "Research":
            raise HTTPException(409, f"{name} is already in the list at '{contacts.stage_of(entry)}'")
        values: Dict[str, Any] = {"stage": "Research"}
        if body.how_known.strip():
            values["notes"] = body.how_known.strip()
        contacts.upsert_entry(rid, values)
        if body.pasted.strip():
            contacts.create_note(rid, f"{contacts.EVIDENCE_NOTE}: {body.pasted_source.strip() or 'pasted text'}",
                              body.pasted.strip())
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True, "record_id": rid}


class ContactEdit(BaseModel):
    record_id: str
    values: Dict[str, str]
    how_known: Optional[str] = None


@app.post("/api/contact")
def contact(body: ContactEdit) -> Dict[str, Any]:
    """A person corrected the contact's own fields, and how they know them."""
    try:
        contacts.update_person(body.record_id, body.values)
        if body.how_known is not None and contacts.entry_of(body.record_id) is not None:
            contacts.upsert_entry(body.record_id, {"notes": body.how_known.strip()})
    except contacts.ContactError as e:
        raise HTTPException(422, str(e))
    return {"ok": True}


def _running() -> bool:
    return _run["proc"] is not None and _run["proc"].poll() is None


def _start(steps: List[List[str]], what: str) -> None:
    """Start the runner commands, in order, as the one background job."""
    runner.DATA.mkdir(parents=True, exist_ok=True)
    cmd = " && ".join(" ".join(f"'{a}'" for a in argv) for argv in steps)
    out = open(RUN_LOG, "w", encoding="utf-8")
    _run.update(proc=subprocess.Popen(["bash", "-c", cmd], cwd=str(REPO), stdout=out, stderr=subprocess.STDOUT),
                what=what, started=datetime.datetime.now().isoformat(timespec="seconds"))


class Run(BaseModel):
    what: str                          # "daily", "scout" or "replies"
    kind: str = ""
    want: int = 5
    firms: bool = False


@app.post("/api/run")
def run(body: Run) -> Dict[str, Any]:
    """Start the runner, then the push, as one background job. One at a time."""
    if _running():
        raise HTTPException(409, f"a run is going: {_run['what']}")
    py, script = sys.executable, str(HERE / "runner.py")
    if body.what == "scout":
        if body.kind not in schemas.TYPES or body.kind == "none":
            raise HTTPException(422, "choose a kind to scout for")
        first = [py, script, "scout", "--kind", body.kind, "--want", str(body.want), "--contacts", "--model", str(MODEL)]
        first += ["--firms"] if body.firms else []
        what = f"scout for {body.kind}" + (", by firm" if body.firms else "")
    elif body.what == "replies":
        first = [py, script, "replies", "--model", str(MODEL)]
        what = "reading replies and drafting answers"
    elif body.what == "daily":
        first = [py, script, "daily", "--model", str(MODEL)]
        what = "today's work"
    else:
        raise HTTPException(422, "what must be daily, scout or replies")
    _start([first] + ([[py, script, "push", "--scouted"]] if body.what == "scout" else []), what)
    return {"ok": True, "what": what}


@app.get("/api/run")
def run_status() -> Dict[str, Any]:
    proc = _run["proc"]
    lines: List[str] = []
    if RUN_LOG.is_file():
        lines = [x for x in RUN_LOG.read_text(encoding="utf-8", errors="replace").splitlines()
                 if " outreach " in x or x.startswith(("#", "- **", "replies:", "followups:"))][-12:]
    return {"running": _running(), "what": _run["what"],
            "started": _run["started"], "exit": None if proc is None else proc.poll(), "log": lines}


def main() -> int:
    global MODEL
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8810)
    ap.add_argument("--model", type=Path, default=MODEL)
    args = ap.parse_args()
    MODEL = args.model
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
