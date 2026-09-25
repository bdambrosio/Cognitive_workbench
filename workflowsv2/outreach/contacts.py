"""The practice's contacts, as the outreach workflow keeps them: one JSON file
per person under prospects/contacts/.

A contact file holds the person (name, title, firm, links, email), their
entry in the outreach list when they have one (stage, kind, dates, next
action), and the notes that record each approach. It replaced the Attio
workspace on 2026-09-25; `attio_record_id` on a contact carried over from
Attio names the record it came from.

EVERY WRITE is one read-modify-write of one person's file, done under a lock
shared by the page and the runner, and written to a temporary file and
renamed over the old one (utils.file_utils). A failed write leaves the
previous file whole. daily.sh copies the folder to a dated backup before
each run.

    {"id": "jane_smith", "name": "Jane Smith", "title": "", "firm": "",
     "domain": "", "linkedin": "", "email": "", "phone": "", "location": "",
     "description": "",
     "created": "2026-09-25",
     "entry": null | {"stage": "Research", "category": "", "relationship": "",
                      "problem_recognition": "", "fit_rationale": "",
                      "notes": "", "last_contact": "", "next_action": "",
                      "next_action_date": ""},
     "notes": [{"note_id": "...", "title": "...", "date": "2026-09-25", "text": "..."}],
     "tasks": [{"text": "...", "due": "2026-09-23", "done": false}]}
"""
from __future__ import annotations

import datetime
import fcntl
import json
import re
import sys
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
if str(REPO / "src") not in sys.path:
    sys.path.insert(0, str(REPO / "src"))

from utils.file_utils import atomic_write_text                         # noqa: E402

DIR = HERE / "prospects" / "contacts"
PERSON_FIELDS = ("name", "title", "firm", "domain", "linkedin", "email", "phone", "location", "description")
#: The fields a person may edit on the page. Not the name: the runner keeps
#: its records for a person under a directory named from it.
EDITABLE = PERSON_FIELDS[1:]
#: The fields of an entry in the outreach list.
ENTRY_FIELDS = ("stage", "category", "relationship", "problem_recognition", "fit_rationale", "notes",
                "last_contact", "next_action", "next_action_date")
#: The list's stages, in order. "Not pursuing" is not a stage: the entry is
#: removed and a note records why (not_pursuing).
STAGES = ("Research", "Qualified", "Ready to contact", "Initial sent", "Follow-up sent",
          "Replied", "Conversation")

#: A note whose title starts with this is text the practice pasted as
#: evidence (a LinkedIn post, an email); the rest of the title says where it
#: came from.
EVIDENCE_NOTE = "Evidence"
#: The notes that record an exchange, by the start of their title; the date
#: follows. The page writes them with the exact text; the runner reads them.
SENT_NOTE, FOLLOWUP_NOTE, REPLY_NOTE = "Message sent", "Follow-up sent", "Reply received"
ANSWER_NOTE = "Answer sent"


class ContactError(RuntimeError):
    """The contact asked for does not exist, or a write would clash with one that does."""


def slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def _today() -> str:
    return datetime.date.today().isoformat()


# ---- files ------------------------------------------------------------------

def _path(cid: str) -> Path:
    if not cid or not re.fullmatch(r"[a-z0-9_]+", cid):
        raise ContactError(f"not a contact id: {cid!r}")
    return DIR / f"{cid}.json"


@contextmanager
def _locked() -> Iterator[None]:
    """Held for each read-modify-write, so the page and the runner, which are
    separate processes, never write over each other's change."""
    DIR.mkdir(parents=True, exist_ok=True)
    with open(DIR / ".lock", "w") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(f, fcntl.LOCK_UN)


def _write(c: Dict[str, Any]) -> None:
    atomic_write_text(_path(c["id"]), json.dumps(c, indent=1, ensure_ascii=False) + "\n")


def get(cid: str) -> Dict[str, Any]:
    p = _path(cid)
    if not p.is_file():
        raise ContactError(f"no contact {cid}")
    return json.loads(p.read_text(encoding="utf-8"))


def everyone() -> List[Dict[str, Any]]:
    DIR.mkdir(parents=True, exist_ok=True)
    return [json.loads(p.read_text(encoding="utf-8")) for p in sorted(DIR.glob("*.json"))]


def _change(cid: str, fn) -> Dict[str, Any]:
    with _locked():
        c = get(cid)
        out = fn(c)
        _write(c)
    return c if out is None else out


# ---- reading ----------------------------------------------------------------

def find_person(name: str) -> Optional[Dict[str, Any]]:
    """The contact of this name, or None. Names are compared as their ids are
    made: case, spaces and punctuation do not count."""
    p = DIR / f"{slug(name)}.json"
    return json.loads(p.read_text(encoding="utf-8")) if slug(name) and p.is_file() else None


def known(name: str) -> bool:
    """Whether the practice already has a person of this name."""
    return find_person(name) is not None


def _same_firm(c: Dict[str, Any], firm: str, domain: str) -> bool:
    return bool((domain and (c.get("domain") or "").lower() == domain.lower())
                or (firm and (c.get("firm") or "").strip().lower() == firm.strip().lower()))


def firm_known(firm: str = "", domain: str = "") -> bool:
    """Whether any contact works at this firm, by its web domain or its exact name."""
    return any(_same_firm(c, firm, domain) for c in everyone())


def colleagues(firm: str, domain: str, but: str = "") -> str:
    """The other contacts at this firm, each with their stage in the outreach
    list: what "one approach per firm" is judged from. Empty when there is
    nobody else."""
    rows = []
    for c in everyone():
        if c["id"] == but or not _same_firm(c, firm, domain):
            continue
        where = f"in the outreach list at stage '{stage_of(c)}'" if c.get("entry") else "not in the outreach list"
        rows.append(f"A colleague at the same firm, {c['name']}, is {where}.")
    return "\n".join(rows)


def contact_record(c: Dict[str, Any]) -> str:
    """What is recorded about approaches to this person, as the text the
    qualification reads: the list stage and last contact, each task, and the
    title and date of each note. Empty when there is nothing."""
    rows: List[str] = []
    e = c.get("entry")
    if e:
        rows.append(f"In the outreach list at stage '{e.get('stage')}', last contact "
                    f"{e.get('last_contact') or 'not recorded'}.")
    for t in c.get("tasks") or []:
        rows.append(f"Task ({'done' if t.get('done') else 'open'}, due {t.get('due') or 'no date'}): "
                    f"{' '.join((t.get('text') or '').split())}")
    for n in c.get("notes") or []:
        rows.append(f"Note of {n.get('date')}: {n.get('title') or '(untitled)'}")
    return "\n".join(rows)


def contact_for(name: str, firm: str = "", domain: str = "") -> str:
    """Everything the qualification is told about earlier approaches: the
    person's own record when there is one, and their colleagues at the firm."""
    c = find_person(name)
    rows: List[str] = []
    if c is not None:
        rows.append(contact_record(c))
        firm, domain = c.get("firm") or firm, c.get("domain") or domain
    rows.append(colleagues(firm, domain, but=c["id"] if c else ""))
    return "\n".join(r for r in rows if r)


def notes(cid: str) -> List[Dict[str, Any]]:
    return get(cid).get("notes") or []


def note_texts(cid: str, title: str, among: Optional[List[Dict[str, Any]]] = None) -> List[Dict[str, str]]:
    """The notes on a person whose title starts with `title`, oldest first,
    each as `{note_id, date, text}`. `among` is the person's notes when the
    caller has already read them."""
    out = [{"note_id": n["note_id"], "date": n["date"], "text": (n.get("text") or "").strip()}
           for n in (notes(cid) if among is None else among) if (n.get("title") or "").startswith(title)]
    return sorted(out, key=lambda n: n["date"])


def stage_of(c: Optional[Dict[str, Any]]) -> Optional[str]:
    return ((c or {}).get("entry") or {}).get("stage")


def due(c: Dict[str, Any], stage: str, on: str) -> bool:
    """Whether the person's entry is at `stage` and its next action date has
    come. An entry with no such date counts as due: nobody has said when."""
    when = (c.get("entry") or {}).get("next_action_date")
    return stage_of(c) == stage and (not when or str(when) <= on)


def entries() -> List[Dict[str, Any]]:
    """Every contact with an entry in the outreach list."""
    return [c for c in everyone() if c.get("entry")]


def entry_of(cid: str) -> Optional[Dict[str, Any]]:
    """The contact when they have an entry in the outreach list, else None."""
    c = get(cid)
    return c if c.get("entry") else None


def candidate_from(c: Dict[str, Any]) -> Dict[str, Any]:
    """One contact as a candidate record for the runner: the name, title, firm
    and links; the entry's relationship and notes as the relationship note;
    each evidence note as pasted text."""
    out: Dict[str, Any] = {"name": c["name"], "contact_id": c["id"]}
    for k in ("title", "firm"):
        if c.get(k):
            out[k] = c[k]
    urls = [u for u in (f"https://{c['domain']}" if c.get("domain") else "", c.get("linkedin") or "") if u]
    if urls:
        out["urls"] = urls
    e = c.get("entry") or {}
    said = [f"The practice marked the relationship: {e['relationship']}."] if e.get("relationship") else []
    if e.get("notes"):
        said.append(str(e["notes"]).strip())
    if said:
        out["relationship"] = "\n".join(said)
    pasted = [{"source": (n.get("title") or "")[len(EVIDENCE_NOTE):].strip(" :") or "text pasted by the practice",
               "date": n.get("date") or "", "text": n.get("text") or ""}
              for n in c.get("notes") or [] if (n.get("title") or "").startswith(EVIDENCE_NOTE)]
    if pasted:
        out["pasted"] = pasted
    return out


# ---- writing ----------------------------------------------------------------

def create_person(name: str, title: str = "", linkedin: str = "", **fields: str) -> Dict[str, Any]:
    """A new contact. Raises when one of that name exists."""
    cid = slug(name)
    if not cid:
        raise ContactError(f"not a usable name: {name!r}")
    c = {"id": cid, **{k: "" for k in PERSON_FIELDS}, "created": _today(), "entry": None, "notes": [], "tasks": []}
    c.update({k: str(v).strip() for k, v in fields.items() if k in PERSON_FIELDS})
    c.update(name=name.strip(), title=title.strip(), linkedin=linkedin.strip())
    with _locked():
        if _path(cid).is_file():
            raise ContactError(f"there is already a contact named {name!r}")
        _write(c)
    return c


def update_person(cid: str, values: Dict[str, Any]) -> Dict[str, Any]:
    """Set the person's own fields, other than the name."""
    bad = set(values) - set(EDITABLE)
    if bad:
        raise ContactError(f"not an editable contact field: {', '.join(sorted(bad))}")
    return _change(cid, lambda c: c.update({k: str(v).strip() for k, v in values.items()}))


def upsert_entry(cid: str, values: Dict[str, Any]) -> Dict[str, Any]:
    """The person's entry in the outreach list, created or updated. Only the
    fields named in `values` are written."""
    bad = set(values) - set(ENTRY_FIELDS)
    if bad:
        raise ContactError(f"not an entry field: {', '.join(sorted(bad))}")

    def fn(c):
        c["entry"] = {**(c.get("entry") or {k: "" for k in ENTRY_FIELDS}), **values}
    return _change(cid, fn)


def create_note(cid: str, title: str, text: str) -> Dict[str, Any]:
    note = {"note_id": uuid.uuid4().hex, "title": title, "date": _today(), "text": text}
    _change(cid, lambda c: c.setdefault("notes", []).append(note))
    return note


def not_pursuing(cid: str, reason: str, on: str) -> None:
    """Record that the practice will not approach this person: a note "Not
    pursuing <date>" holding the reason and what the entry said, and the
    entry removed. The contact stays, so scouting does not bring them back,
    and the next qualification reads the note in the contact record."""
    def fn(c):
        e = c.get("entry")
        was = (f"\n\nThe entry was at stage '{e.get('stage')}'. Kind: {e.get('category') or '(none)'}. "
               f"Fit rationale then: {e.get('fit_rationale') or '(none)'}") if e else ""
        c.setdefault("notes", []).append({"note_id": uuid.uuid4().hex, "title": f"Not pursuing {on}",
                                          "date": _today(), "text": reason.strip() + was})
        c["entry"] = None
    _change(cid, fn)
