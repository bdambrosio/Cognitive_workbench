"""Research, qualify and draft a first message for people the practice names.

    workflowsv2/outreach/daily.sh                                  (the whole day's work, and the page)
    python3 workflowsv2/outreach/runner.py daily [--pool 5] [--want 3]
    python3 workflowsv2/outreach/runner.py run --candidates <file.yaml> [--only <slug>]
    python3 workflowsv2/outreach/runner.py research|qualify|draft|brief ...   (one stage)
    python3 workflowsv2/outreach/runner.py followups|replies               (after a message is sent)

`daily` works on the names waiting at Research in the contacts; reads the replies
the practice has recorded and drafts the follow-ups that are due; then, when fewer than
`--pool` people are Ready to contact, scouts for the day's kind (the kinds take turns by date; people
an earlier scout found of that kind are taken before any new search)
(`--want` of the people found go on to research); and pushes everything it
qualified. The model is the local Qwen unless `--model` names another; the three calls
that write a message to send (the first message, the follow-up, the answer to a reply)
use the `--writer` model, Claude Opus 5.5 unless it names another.

THE CANDIDATES FILE is a YAML list. Each entry has `name`, and any of `firm`,
`urls` (pages to fetch), `pasted` (a list of `{source, date, text}`: text
copied by hand from a page this program cannot fetch, such as a LinkedIn
post), `relationship` (how the practice knows the person; the method treats
it as fact) and `notes`.

FOUR STAGES, each a schema-constrained emission under method/PROSPECT.md with
no tools; everything that touches the network is code.

    research   saves each pasted text and each given URL as an evidence file;
               asks the model for up to four searches (§12), runs them on
               Tavily, saves each page found, and asks once per found page
               whether it is about this person (§13). One round of search.
    qualify    one emission per candidate over the evidence kept (§14). Every
               citation is checked against its evidence file by program; one
               whose quote is not in the file is dropped and recorded.
    draft      for a `strong` candidate only, the message (§15), checked the
               same way.
    brief      brief.md per candidate and summary.md for the file, written
               from the records. No model.
    push       not part of `run`. For a `strong` candidate with a draft: the
               person's contact (contacts.py) is found by name or created, the person
               is put in the outreach list at "Ready to contact" with the
               rationale and the three selects, and the brief is attached as a
               note. A plausible candidate goes to "Qualified" with the brief;
               a weak or rejected one leaves the list with a "Not pursuing"
               note that gives the reason. An
               existing person record is never changed, and an entry at any
               stage but Research is left alone: a person set it. Recorded in
               pushed.json; a candidate with that file is not pushed again.

    scout      `scout --kind "<a kind from PROSPECT.md §4>" --model <yaml>`. The
               model proposes up to three descriptions of such people (§16),
               differing from the searches already logged; Exa's people search
               runs them; a name already in the contacts or already in the record is
               skipped; each new person gets a first look at their profile
               (§17), which costs no search. Those who fit become candidates,
               with the profile as their first evidence file, and go through
               the four stages at once. Nothing found by scouting is written to
               the contacts unless `push` later finds it strong. With `--firms` the
               search is for firms (§18, §19); for each firm that fits, one
               people search at that firm and one emission (§20) choose whom
               to approach, with a checked citation that they work there now.
               One candidate per firm; a firm a contact works at is skipped.

    followups  for each person at "Initial sent" whose next action date has
               come: one emission (§21) drafts the one follow-up, from the first
               message as the page recorded it, the contact record, and
               the qualification and evidence when there are any. No search.
    replies    for each reply the page recorded (a note "Reply
               received <date>") that has not been read: one emission (§22)
               says what the reply gives and what to do next. The quotes are
               checked against the reply. The next step goes into the entry's
               next action, and a note "Reply read <date>" records what it gave.
               A second emission (§23) drafts the practice's answer, kept in
               the reply's record for the page to show. The page starts this
               stage when a reply is recorded; `daily` runs it for whatever is
               still unread or unanswered.

WITHOUT --candidates the candidates are the contacts in the outreach list
at stage Research: a person adds a name there, with how they know them in the
entry's Notes and any pasted text in a note titled "Evidence: <source>".

NOTHING IS SENT. A person reads the brief, edits the message and sends it.
The category is the model's and is never changed by code; what the checks
find is written into the brief for the person to weigh.

THE RECORD is `prospects/<slug>/` (not in git: it holds personal data):
candidate.yaml, evidence/*.md, research.json, qualification.json, draft.json,
brief.md, followup.json, replies.json, issues.jsonl. A stage whose record exists is not run again unless
`--redo` is given, because searches cost money and evidence is dated.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import sys
import types
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for p in (str(REPO), str(REPO / "src")):
    if p not in sys.path:
        sys.path.insert(0, p)

from workflowsv2 import issues                                          # noqa: E402
from workflowsv2.emit import emit                                       # noqa: E402
from workflowsv2.claims_audit.decompose import backend_from_model       # noqa: E402
from workflowsv2.outreach import contacts, exa, schemas                   # noqa: E402
from chat.workflow import load_workflow                                 # noqa: E402
from utils import tavily_client                                         # noqa: E402
from utils.doc_extract import html_to_markdown, pdf_to_markdown         # noqa: E402
from utils.file_utils import append_jsonl, atomic_write_text, read_jsonl  # noqa: E402

logger = logging.getLogger("outreach")

METHOD_PATH = HERE / "method" / "PROSPECT.md"
DATA = HERE / "prospects"
STAGES = ("research", "qualify", "draft", "brief")
#: Words of one evidence file shown to the model, as sorting shows 1500.
PAGE_WORDS = 1500
#: A fetched page with fewer words than this has no readable text.
MIN_WORDS = 20
RESULTS_PER_QUERY = 4
#: Hosts that answer a program with a login page. Given as a URL they are
#: recorded as not fetched; the practice pastes the text instead.
NOT_FETCHABLE = ("linkedin.com",)
_UA = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0 Safari/537.36"


def today() -> str:
    return datetime.date.today().isoformat()


#: A person's directory name here is their contact id.
slug = contacts.slug


def _write_json(path: Path, obj: Any) -> None:
    atomic_write_text(path, json.dumps(obj, indent=1, ensure_ascii=False) + "\n")


def _read_json(path: Path) -> Optional[Dict[str, Any]]:
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None


def _ask(backend, user: str, schema: Dict[str, Any], max_tokens: int) -> Dict[str, Any]:
    return emit(types.SimpleNamespace(backend=backend), load_workflow(METHOD_PATH),
                user, schema, max_tokens)


def candidate_text(c: Dict[str, Any]) -> str:
    """The candidate record as every prompt states it. Pasted text is not
    repeated here; it reaches the model as evidence files."""
    rows = [f"Name: {c['name']}"]
    if c.get("firm"):
        rows.append(f"Firm: {c['firm']}")
    if c.get("urls"):
        rows.append("Links: " + ", ".join(c["urls"]))
    if c.get("notes"):
        rows.append(f"Notes: {str(c['notes']).strip()}")
    rows.append("Relationship note: " + (str(c.get("relationship") or "").strip() or "(none)"))
    return "The candidate record:\n\n" + "\n".join(rows)


# ---- evidence files ---------------------------------------------------------

def page_text(url: str) -> Optional[str]:
    """The readable text of a page, or None when it could not be fetched."""
    try:
        req = urllib.request.Request(url, headers={"User-Agent": _UA})
        with urllib.request.urlopen(req, timeout=20) as resp:
            data, kind = resp.read(), resp.headers.get_content_type()
        if kind == "application/pdf" or urlparse(url).path.lower().endswith(".pdf"):
            return pdf_to_markdown(data)
        html = data.decode(resp.headers.get_content_charset() or "utf-8", "replace")
        from bs4 import BeautifulSoup
        title = BeautifulSoup(html, "html.parser").title
        head = f"Page title: {title.get_text(strip=True)}\n\n" if title else ""
        return head + html_to_markdown(html)
    except Exception as e:                                     # noqa: BLE001
        logger.warning("fetch %s: %s", url, e)
        return None


def save_evidence(ev_dir: Path, n: int, label: str, header: Dict[str, str], text: str) -> str:
    """Write one evidence file and return its name. The header lines are part
    of the file, so the line numbers the model cites count from the top."""
    name = f"{n:02d}_{slug(label)[:40] or 'page'}.md"
    head = "\n".join(f"{k}: {v}" for k, v in header.items() if v)
    atomic_write_text(ev_dir / name, f"{head}\n---\n{text.strip()}\n")
    return name


def load_evidence(cand_dir: Path, names: List[str]) -> Dict[str, List[str]]:
    return {n: (cand_dir / "evidence" / n).read_text(encoding="utf-8").splitlines()
            for n in names}


# ---- research ---------------------------------------------------------------

def propose_queries(backend, c: Dict[str, Any], files: List[Dict[str, Any]],
                    cand_dir: Path) -> Dict[str, Any]:
    have = []
    for f in files:
        body = (cand_dir / "evidence" / f["file"]).read_text(encoding="utf-8").partition("\n---\n")[2]
        have.append(f"- {f['file']} ({f['source']}): " + " ".join(body.split()[:60]))
    user = (f"{candidate_text(c)}\n\nThe evidence files gathered so far:\n\n"
            + ("\n".join(have) or "(none)")
            + "\n\nThis step proposes web searches. Emit the answer per PROSPECT.md §12.")
    out = _ask(backend, user, schemas.queries_schema(), 4096)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    queries = [q.strip() for q in obj.get("queries") or [] if isinstance(q, str) and q.strip()][:4]
    return {"queries": queries, "reason": str(obj.get("reason") or "").strip(),
            "parse": out.get("parse"), "parse_error": out.get("parse_error")}


def about_candidate(backend, c: Dict[str, Any], name: str, lines: List[str]) -> Dict[str, Any]:
    shown, _ = schemas.numbered(lines, PAGE_WORDS)
    user = (f"{candidate_text(c)}\n\nOne page a search found, `{name}`, with line numbers:\n\n"
            f"{shown}\n\nThis step says whether the page is about the candidate. "
            f"Emit the answer per PROSPECT.md §13.")
    out = _ask(backend, user, schemas.page_schema(), 2048)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    about = obj.get("about_candidate") if obj.get("about_candidate") in schemas.ABOUT else None
    return {"about": about, "reason": str(obj.get("reason") or "").strip()}


def research(backend, c: Dict[str, Any], cand_dir: Path) -> Dict[str, Any]:
    ev_dir = cand_dir / "evidence"
    ev_dir.mkdir(parents=True, exist_ok=True)
    files: List[Dict[str, Any]] = []
    seen: set = set()

    def add(label: str, header: Dict[str, str], text: str, found_by: str) -> Dict[str, Any]:
        name = save_evidence(ev_dir, len(files) + 1, label, header, text)
        row = {"file": name, "source": header.get("Source", ""), "found_by": found_by,
               "words": len(text.split())}
        files.append(row)
        return row

    for p in c.get("pasted") or []:
        text = str(p.get("text") or "").strip()
        if text:
            add("pasted", {"Source": str(p.get("source") or "text pasted by the practice"),
                           "Date of the text": str(p.get("date") or ""),
                           "Copied by hand on": today()}, text, "pasted")
    for url in c.get("urls") or []:
        seen.add(url.rstrip("/"))
        host = urlparse(url).netloc.lower()
        if any(host == h or host.endswith("." + h) for h in NOT_FETCHABLE):
            logger.info("%s is not fetched; paste its text instead", url)
            continue
        text = page_text(url)
        if text is None or len(text.split()) < MIN_WORDS:
            issues.note(cand_dir, "research", "unreadable_page",
                        f"{url} gave no readable text", severity="note")
            continue
        add(host, {"Source": url, "Retrieved": today()}, text, "given")

    asked = propose_queries(backend, c, files, cand_dir)
    if asked["parse"] not in ("parsed", "repaired"):
        issues.note(cand_dir, "research", "no_queries",
                    f"the search proposal returned nothing usable ({asked['parse_error']})")
    for q in asked["queries"]:
        try:
            results = tavily_client.search(q, RESULTS_PER_QUERY)
        except tavily_client.TavilyError as e:
            issues.note(cand_dir, "research", "search_failed", f"{q!r}: {e}")
            logger.warning("search %r: %s", q, e)
            continue
        logger.info("%s: %r gave %d results", c["name"], q, len(results))
        for r in results:
            url = str(r.get("url") or "")
            text = str(r.get("raw_content") or r.get("content") or "")
            if not url or url.rstrip("/") in seen or len(text.split()) < MIN_WORDS:
                continue
            seen.add(url.rstrip("/"))
            row = add(urlparse(url).netloc, {"Source": url, "Title": str(r.get("title") or ""),
                                             "Retrieved": today(), "Found by the search": q},
                      text, f"search: {q}")
            row.update(about_candidate(backend, c, row["file"],
                                       (ev_dir / row["file"]).read_text(encoding="utf-8").splitlines()))
    rec = {"at": today(), "model": backend.resolved_model(), "queries": asked["queries"],
           "queries_reason": asked["reason"], "files": files}
    _write_json(cand_dir / "research.json", rec)
    return rec


def firm_domain(c: Dict[str, Any]) -> str:
    """The web domain of the candidate's firm: the host of the first link that
    is not a profile on a site the program does not fetch."""
    for u in c.get("urls") or []:
        host = urlparse(u).netloc.lower().removeprefix("www.")
        if host and not any(host == h or host.endswith("." + h) for h in NOT_FETCHABLE):
            return host
    return ""


def better_contact(c: Dict[str, Any], q: Dict[str, Any], data: Path, use_contacts: bool) -> Optional[Path]:
    """When the qualification names a better person to approach at the firm,
    that person becomes a candidate waiting for research, unless the practice
    already has them. Returns their directory when one was made."""
    name = str(q.get("better_contact_name") or "").strip()
    if not name or not slug(name) or slug(name) == slug(c["name"]):
        return None
    cand_dir = data / slug(name)
    if cand_dir.exists() or (use_contacts and contacts.known(name)):
        return None
    new = {"name": name, "firm": c.get("firm") or "", "title": q.get("better_contact_role") or "",
           "urls": [u for u in c.get("urls") or [] if firm_domain({"urls": [u]})],
           "notes": f"Named as the better person to approach at the firm when {c['name']} was qualified."}
    cand_dir.mkdir(parents=True)
    atomic_write_text(cand_dir / "candidate.yaml", yaml.safe_dump(new, allow_unicode=True, sort_keys=False))
    _write_json(cand_dir / "first_look.json", {"at": today(), "fits": "yes", "prospect_type": q.get("prospect_type"),
                                               "reason": new["notes"]})
    logger.info("%s: %s is named as the better contact and waits for research", c["name"], name)
    return cand_dir


def kept_files(rec: Dict[str, Any]) -> List[Dict[str, Any]]:
    """The evidence qualification reads: everything given or pasted, and each
    found page not judged to be about someone else."""
    return [f for f in rec["files"] if f["found_by"] in ("given", "pasted") or f.get("about") != "no"]


# ---- qualify and draft ------------------------------------------------------

def evidence_block(cand_dir: Path, files: List[Dict[str, Any]]) -> str:
    parts = []
    for f in files:
        lines = (cand_dir / "evidence" / f["file"]).read_text(encoding="utf-8").splitlines()
        shown, _ = schemas.numbered(lines, PAGE_WORDS)
        doubt = (" The practice's check could not tell whether this page is about the candidate."
                 if f.get("about") in ("unsure", None) and f["found_by"].startswith("search") else "")
        parts.append(f"Evidence file `{f['file']}`.{doubt}\n\n{shown}")
    return "\n\n".join(parts) or "(no evidence files)"


def qualify(backend, c: Dict[str, Any], cand_dir: Path, contact: str = "") -> Dict[str, Any]:
    files = kept_files(_read_json(cand_dir / "research.json") or {"files": []})
    evidence = load_evidence(cand_dir, [f["file"] for f in files])
    base = (f"{candidate_text(c)}\n\nThe contact record: {contact or '(the person is not in the contact system)'}\n\n"
            f"Today is {today()}.\n\n{evidence_block(cand_dir, files)}\n\n"
            f"This step qualifies the candidate. Emit the answer per PROSPECT.md §14.")
    user, attempts = base, 0
    while True:
        attempts += 1
        out = _ask(backend, user, schemas.qualification_schema(), 16384)
        q, dropped = schemas.clean_qualification(out.get("obj"), evidence)
        unusable = out.get("parse") not in ("parsed", "repaired") or q["category"] is None
        unsupported = q["category"] == "strong" and not q["why_now_citations"]
        if attempts == 2 or not (unusable or unsupported):
            break
        # One repeat, with the failure stated. A strong category needs a
        # checked fact behind `why_now` (PROSPECT.md §14).
        user = base + ("\n\nYour previous answer could not be used: "
                       + ("it did not parse or gave no category." if unusable else
                          "the category was `strong` and no citation for `why_now` matched its "
                          "evidence file. Copy each quote exactly from the numbered text, or "
                          "choose the category the evidence supports."))
    for d in dropped:
        issues.note(cand_dir, "qualify", "citation_dropped",
                    f"{d.get('in')}: {d.get('file')} {d.get('lines')}: {d.get('why')}", severity="note")
    flags = []
    if unusable:
        flags.append("the qualification returned nothing usable")
    if unsupported:
        flags.append("category is strong and `why_now` rests on no checked citation")
    if q["unanswered"]:
        flags.append("questions not answered: " + ", ".join(map(str, q["unanswered"])))
    if dropped:
        flags.append(f"{len(dropped)} citation(s) did not match their evidence file and were removed")
    rec = {**q, "at": today(), "model": backend.resolved_model(), "attempts": attempts,
           "evidence_files": [f["file"] for f in files], "citations_dropped": dropped,
           "flags": flags, "parse": out.get("parse")}
    _write_json(cand_dir / "qualification.json", rec)
    return rec


def draft(backend, c: Dict[str, Any], cand_dir: Path) -> Optional[Dict[str, Any]]:
    q = _read_json(cand_dir / "qualification.json")
    if not q or q.get("category") != "strong":
        return None
    files = kept_files(_read_json(cand_dir / "research.json") or {"files": []})
    evidence = load_evidence(cand_dir, [f["file"] for f in files])
    shown = {k: q[k] for k in ("prospect_type", "relationship", "problem_recognition", "category", "why_person", "why_now",
                               "why_now_citations", "use_case", "concerns")}
    user = (f"{candidate_text(c)}\n\nThe qualification:\n\n"
            f"{json.dumps(shown, indent=1, ensure_ascii=False)}\n\n"
            f"{evidence_block(cand_dir, files)}\n\n"
            f"This step drafts the first message. Emit the answer per PROSPECT.md §15.")
    for _ in range(2):
        # Asked again once when nothing usable comes back: a strong candidate
        # with no message is not pushed, and would wait for a person to notice.
        out = _ask(backend, user, schemas.draft_schema(), 16000)
        d, dropped, flags = schemas.clean_draft(out.get("obj"), evidence)
        if out.get("parse") in ("parsed", "repaired") and d["message"]:
            break
    else:
        flags.append("the draft returned nothing usable")
    rec = {**d, "at": today(), "model": backend.resolved_model(),
           "citations_dropped": dropped, "flags": flags}
    _write_json(cand_dir / "draft.json", rec)
    return rec


# ---- the brief --------------------------------------------------------------

def _cite_lines(cand_dir: Path, cites: List[Dict[str, Any]], sources: Dict[str, str]) -> List[str]:
    out = []
    for ci in cites:
        where = f"{ci['file']} lines {ci['lines'][0]}-{ci['lines'][1]}"
        if not ci.get("at_lines", True):
            where = f"{ci['file']} (the quote is in the file, not at the lines given)"
        out.append(f"- \"{' '.join(ci['quote'].split())}\" ({sources.get(ci['file'], '')}; {where})")
    return out


def brief(c: Dict[str, Any], cand_dir: Path) -> str:
    r = _read_json(cand_dir / "research.json") or {"files": []}
    q = _read_json(cand_dir / "qualification.json") or {}
    # A draft left by an earlier qualification that was `strong` is not shown.
    d = _read_json(cand_dir / "draft.json") if q.get("category") == "strong" else None
    sources = {f["file"]: f["source"] for f in r["files"]}
    head = c["name"] + (f", {c['firm']}" if c.get("firm") else "")
    rows = [f"# {head}", "",
            f"**Category:** {q.get('category')}  **Kind:** {q.get('prospect_type')}  "
            f"**Relationship:** {q.get('relationship')}  "
            f"**Problem recognition:** {q.get('problem_recognition')}  "
            f"**Qualified:** {q.get('at')} by {q.get('model')}", ""]
    if c.get("relationship"):
        rows += [f"**Relationship:** {str(c['relationship']).strip()}", ""]
    rows += [f"**Why this person:** {q.get('why_person')}", "", f"**Why now:** {q.get('why_now')}", ""]
    rows += _cite_lines(cand_dir, q.get("why_now_citations") or [], sources) + [""]
    if q.get("use_case"):
        rows += [f"**Use proposed:** {q['use_case']}", ""]
    if q.get("reject_reason"):
        rows += [f"**Reason to reject:** {q['reject_reason']}", ""]
    flags = list(q.get("flags") or []) + list((d or {}).get("flags") or [])
    if q.get("concerns") or flags:
        rows += ["**Concerns:** " + (q.get("concerns") or "")] + [f"- CHECK: {f}" for f in flags] + [""]
    if d:
        rows += ["## Suggested message", "", d["message"], "", f"**Angle:** {d['angle']}", ""]
        if d.get("assumes"):
            rows += [f"**The message assumes:** {d['assumes']}", ""]
        rows += ["**The opening rests on:**"] + _cite_lines(cand_dir, d["rests_on"], sources) + [""]
    rows += ["## The research questions", ""]
    for a in q.get("answers") or []:
        rows += [f"{a['question']}. {a['answer']}"] + _cite_lines(cand_dir, a["citations"], sources)
    rows += ["", "## Evidence files", ""]
    for f in r["files"]:
        about = f" (about the candidate: {f['about']}. {f.get('reason', '')})" if "about" in f else ""
        rows.append(f"- {f['file']}: {f['source']}, {f['found_by']}{about}")
    text = "\n".join(rows) + "\n"
    atomic_write_text(cand_dir / "brief.md", text)
    return text


#: The list stage a category is written as. A weak or rejected person gets no
#: stage: they leave the list with a "Not pursuing" note (contacts.not_pursuing).
STAGE_OF = {"strong": "Ready to contact", "plausible": "Qualified"}
CATEGORIES_PUSHED = ("strong", "plausible", "weak", "reject")
#: The stages this program may move an entry out of. Any other stage was set
#: by a person or records a message sent, and is left alone.
OURS = (None, "Research")


def push(c: Dict[str, Any], cand_dir: Path) -> Optional[Dict[str, Any]]:
    """Write one qualified candidate into the contacts: the list stage for its
    category, the rationale and the three selects, and for a strong or
    plausible candidate the brief as a note. A strong candidate with no
    usable draft is not written. Returns the record of what was written."""
    q = _read_json(cand_dir / "qualification.json") or {}
    d = _read_json(cand_dir / "draft.json")
    category = q.get("category")
    if category not in CATEGORIES_PUSHED or (category == "strong" and not (d and d.get("message"))):
        logger.info("%s: no category, or strong with no draft; nothing is pushed", c["name"])
        return None
    if (cand_dir / "pushed.json").is_file():
        logger.info("%s: already pushed", c["name"])
        return None
    person = contacts.find_person(c["name"])
    created = person is None
    if created and category != "strong":
        return None                     # a name nobody put in the contacts, and not worth putting there
    if created:
        linkedin = next((u for u in c.get("urls") or [] if "linkedin.com" in urlparse(u).netloc.lower()), "")
        person = contacts.create_person(c["name"], str(c.get("title") or ""), linkedin,
                                        firm=str(c.get("firm") or ""), domain=firm_domain(c))
    rid = person["id"]
    entry = None if created else contacts.entry_of(rid)
    if entry is not None and contacts.stage_of(entry) not in OURS:
        logger.info("%s: the entry is at '%s', set by a person; left alone", c["name"], contacts.stage_of(entry))
        return None
    why = q.get("reject_reason") or f"{q.get('why_person', '')} {q.get('why_now', '')}".strip()
    if category not in STAGE_OF:
        contacts.not_pursuing(rid, f"Qualified as {category}. {why}", today())
        rec = {"at": today(), "contact_id": rid, "person_created": False, "not_pursuing": True, "reason": why}
        _write_json(cand_dir / "pushed.json", rec)
        return rec
    stage = STAGE_OF[category]
    values: Dict[str, Any] = {"stage": stage, "fit_rationale": why}
    if category == "strong":
        values.update(next_action="Review the draft and send on LinkedIn", next_action_date=today())
    for field, key in (("category", "prospect_type"), ("relationship", "relationship"),
                       ("problem_recognition", "problem_recognition")):
        if q.get(key) and q[key] != "none":
            values[field] = q[key]
    contacts.upsert_entry(rid, values)
    note_id = None
    if category in ("strong", "plausible"):
        note_id = contacts.create_note(rid, f"Outreach brief {today()}", brief(c, cand_dir))["note_id"]
    rec = {"at": today(), "contact_id": rid, "person_created": created, "note_id": note_id,
           "entry_values": values}
    _write_json(cand_dir / "pushed.json", rec)
    return rec


def pickup(data: Path) -> List[Dict[str, Any]]:
    """The candidates waiting in the contacts: every entry of the outreach list at
    stage Research, as candidate records. This is how the practice hands a
    name to the workflow."""
    return [contacts.candidate_from(e) for e in contacts.entries() if contacts.stage_of(e) == "Research"]


# ---- after a message is sent --------------------------------------------------

def _known_dir(c: Dict[str, Any], data: Path) -> Path:
    """The person's record directory, made when the workflow has none: someone
    the practice wrote to before the workflow existed."""
    cand_dir = data / slug(c["name"])
    if not (cand_dir / "candidate.yaml").is_file():
        cand_dir.mkdir(parents=True, exist_ok=True)
        atomic_write_text(cand_dir / "candidate.yaml", yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
    return cand_dir


def sent_block(cid: str) -> str:
    """The messages the practice recorded as sent to this person, with their
    dates, as a prompt states them. Empty when none was recorded."""
    got = contacts.notes(cid)
    rows = [f"{label}, sent {n['date']}:\n\n{n['text']}"
            for label, title in (("The first message", contacts.SENT_NOTE), ("The follow-up", contacts.FOLLOWUP_NOTE),
                                 ("An answer to an earlier reply", contacts.ANSWER_NOTE))
            for n in contacts.note_texts(cid, title, got)]
    return "\n\n".join(rows)


def followup(backend, c: Dict[str, Any], cand_dir: Path, contact: str, sent: str) -> Dict[str, Any]:
    q = _read_json(cand_dir / "qualification.json")
    files = kept_files(_read_json(cand_dir / "research.json") or {"files": []})
    shown = {k: q.get(k) for k in ("prospect_type", "relationship", "problem_recognition", "why_person",
                                   "why_now", "use_case", "concerns")} if q else None
    user = (f"{candidate_text(c)}\n\nThe contact record: {contact or '(nothing recorded)'}\n\n"
            f"Today is {today()}.\n\n"
            + (sent or "The first message: its text was not recorded.") + "\n\n"
            + (f"The qualification:\n\n{json.dumps(shown, indent=1, ensure_ascii=False)}\n\n" if shown else "")
            + f"{evidence_block(cand_dir, files)}\n\n"
            f"This step drafts the follow-up. Emit the answer per PROSPECT.md §21.")
    for _ in range(2):                                     # asked again once, as `draft` is
        out = _ask(backend, user, schemas.followup_schema(), 16000)
        f, flags = schemas.clean_followup(out.get("obj"))
        if out.get("parse") in ("parsed", "repaired") and f["message"]:
            break
    else:
        flags.append("the follow-up returned nothing usable")
    rec = {**f, "at": today(), "model": backend.resolved_model(), "first_message_recorded": bool(sent),
           "flags": flags}
    _write_json(cand_dir / "followup.json", rec)
    return rec


def followups(writer, data: Path) -> List[str]:
    """Draft the follow-up for everyone at Initial sent whose date has come
    and who has none drafted. Returns their names."""
    names = []
    for e in contacts.entries():
        if not contacts.due(e, "Initial sent", today()):
            continue
        c = contacts.candidate_from(e)
        cand_dir = _known_dir(c, data)
        if (cand_dir / "followup.json").is_file():
            continue
        logger.info("%s: follow-up", c["name"])
        followup(writer, c, cand_dir, contacts.contact_record(e), sent_block(e["id"]))
        names.append(c["name"])
    return names


REPLY_STAGES = ("Replied", "Conversation")


def read_reply(backend, c: Dict[str, Any], sent: str, reply: Dict[str, str]) -> Dict[str, Any]:
    """What one reply gives (§22), with each quote looked for in the reply."""
    lines = reply["text"].splitlines()
    shown, _ = schemas.numbered(lines, PAGE_WORDS)
    user = (f"{candidate_text(c)}\n\n" + (sent or "The messages the practice sent were not recorded.")
            + f"\n\nThe person's reply, received {reply['date']}, as the file `{schemas.REPLY_FILE}`, "
            f"with line numbers:\n\n{shown}\n\n"
            f"This step records what the reply gives. Emit the answer per PROSPECT.md §22.")
    out = _ask(backend, user, schemas.reply_schema(), 4096)
    r, flags = schemas.clean_reply(out.get("obj"), lines)
    if out.get("parse") not in ("parsed", "repaired") or not r["gives"]:
        flags.append("the reading of the reply returned nothing usable")
    return {**r, "note_id": reply["note_id"], "received": reply["date"], "reply": reply["text"],
            "at": today(), "model": backend.resolved_model(), "flags": flags}


def answer(backend, c: Dict[str, Any], cand_dir: Path, sent: str, rec: Dict[str, Any]) -> Dict[str, Any]:
    """The practice's answer to one reply that has been read (§23)."""
    shown, _ = schemas.numbered(rec["reply"].splitlines(), PAGE_WORDS)
    q = _read_json(cand_dir / "qualification.json")
    gave = {k: rec[k] for k in ("gives", "introduced_name", "next_step")}
    user = (f"{candidate_text(c)}\n\n" + (sent or "The messages the practice sent were not recorded.")
            + f"\n\nThe person's reply, received {rec['received']}, with line numbers:\n\n{shown}\n\n"
            f"What the reply gives, as recorded:\n\n{json.dumps(gave, indent=1, ensure_ascii=False)}\n\n"
            + (f"The qualification:\n\n" + json.dumps({k: q.get(k) for k in ("prospect_type", "why_person", "use_case")},
                                                       indent=1, ensure_ascii=False) + "\n\n" if q else "")
            + "This step drafts the practice's answer to the reply. Emit your output per PROSPECT.md §23.")
    for _ in range(2):                                     # asked again once, as `draft` is
        out = _ask(backend, user, schemas.answer_schema(), 16000)
        a, flags = schemas.clean_answer(out.get("obj"))
        if out.get("parse") in ("parsed", "repaired") and a["message"]:
            break
    else:
        flags.append("the answer returned nothing usable")
    return {**a, "at": today(), "model": backend.resolved_model(), "flags": flags}


def replies(backend, writer, data: Path) -> List[str]:
    """Read every recorded reply that has not been read, and draft the answer
    to every reply that has none. The record is replies.json, one entry per
    reply; the contact gets the next step as the entry's next action and a note of
    what it gave. Returns the names of the people something was done for."""
    names = []
    for e in contacts.entries():
        if contacts.stage_of(e) not in REPLY_STAGES:
            continue
        rid = e["id"]
        got = contacts.note_texts(rid, contacts.REPLY_NOTE, e["notes"])
        c = contacts.candidate_from(e)
        cand_dir = _known_dir(c, data)
        log = cand_dir / "replies.json"
        have = json.loads(log.read_text(encoding="utf-8")) if log.is_file() else []
        for reply in got:
            if not reply["text"] or reply["note_id"] in [h["note_id"] for h in have]:
                continue
            logger.info("%s: reply of %s", c["name"], reply["date"])
            rec = read_reply(backend, c, sent_block(rid), reply)
            have.append(rec)
            _write_json(log, have)
            gave = ", ".join(g["what"] for g in rec["gives"]) or "nothing usable"
            body = "\n".join([f"- **{g['what']}**: {g['note']} (\"{g['quote']}\")" for g in rec["gives"]]
                             + ([f"\nIntroduces: {rec['introduced_name']}"] if rec["introduced_name"] else [])
                             + [f"\nNext step: {rec['next_step']}"] + [f"- CHECK: {f}" for f in rec["flags"]])
            contacts.create_note(rid, f"Reply read {today()}: {gave}", body)
            if rec["next_step"]:
                contacts.upsert_entry(rid, {"next_action": rec["next_step"], "next_action_date": today()})
            names.append(c["name"])
        for rec in have:
            if "answer" in rec or not rec["gives"]:
                continue
            logger.info("%s: answer to the reply of %s", c["name"], rec["received"])
            rec["answer"] = answer(writer, c, cand_dir, sent_block(rid), rec)
            _write_json(log, have)
            if c["name"] not in names:
                names.append(c["name"])
    return names


SCOUT_LOG = "scout_log.jsonl"


def waiting(data: Path, kind: str) -> List[Dict[str, Any]]:
    """People of this kind an earlier scout found to fit and nobody has
    researched yet."""
    out = []
    for look in sorted(data.glob("*/first_look.json")):
        rec = _read_json(look) or {}
        if rec.get("fits") == "yes" and rec.get("prospect_type") == kind \
                and not (look.parent / "research.json").is_file():
            out.append(yaml.safe_load((look.parent / "candidate.yaml").read_text(encoding="utf-8")))
    return out


def scout(backend, kind: str, data: Path, want: int) -> List[Dict[str, Any]]:
    """Find new people of one kind. Returns at most `want` candidates that
    passed the first look, taking first those an earlier scout left waiting;
    it searches only when they are fewer than `want`. Every person looked at
    leaves a directory, so nobody is looked at twice."""
    found = waiting(data, kind)
    if len(found) >= want:
        return found[:want]
    log = data / SCOUT_LOG
    earlier = [r["query"] for r in read_jsonl(log) if r.get("kind") == kind]
    user = (f"The kind of prospect: `{kind}`.\n\nSearches already made for this kind:\n\n"
            + ("\n".join(f"- {q}" for q in earlier) or "(none)")
            + "\n\nThis step proposes searches for new people. Emit the answer per PROSPECT.md §16.")
    out = _ask(backend, user, schemas.scout_schema(), 4096)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    queries = [q.strip() for q in obj.get("queries") or [] if isinstance(q, str) and q.strip()][:3]
    for q in queries:
        results = exa.search(q, "people")
        new = 0
        for r in results:
            name, url, text = str(r.get("title") or "").strip(), str(r.get("url") or ""), str(r.get("text") or "")
            cand_dir = data / slug(name)
            if not name or not slug(name) or cand_dir.exists() or len(text.split()) < MIN_WORDS:
                continue
            if contacts.known(name):
                continue
            new += 1
            c = {"name": name, "urls": [url] if url else [],
                 "notes": f"Found by the scout search: {q}",
                 "pasted": [{"source": f"Professional profile as the search service holds it, {url}",
                             "date": str(r.get("publishedDate") or "")[:10], "text": text}]}
            cand_dir.mkdir(parents=True)
            atomic_write_text(cand_dir / "candidate.yaml", yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
            shown, _ = schemas.numbered(text.splitlines(), PAGE_WORDS)
            look = _ask(backend, f"The person found: {name}.\n\nTheir professional profile, with line numbers:\n\n"
                                 f"{shown}\n\nThis step takes a first look. Emit the answer per PROSPECT.md §17.",
                        schemas.first_look_schema(), 2048)
            lo = look.get("obj") if isinstance(look.get("obj"), dict) else {}
            rec = {"at": today(), "kind_sought": kind, "query": q, "fits": lo.get("fits"),
                   "prospect_type": lo.get("prospect_type"), "reason": str(lo.get("reason") or "").strip()}
            _write_json(cand_dir / "first_look.json", rec)
            logger.info("scout: %s: fits=%s (%s)", name, rec["fits"], rec["reason"][:90])
            if rec["fits"] == "yes" and rec["prospect_type"] == kind:
                found.append(c)                 # someone who fits another kind waits for a scout of that kind
        append_jsonl(log, {"at": today(), "kind": kind, "query": q, "results": len(results), "new": new})
    return found[:want]


FIRMS = "_firms"
#: The people search made for every firm that fits. One query, the same for
#: every kind: PROSPECT.md §4 says who at a firm is the person, and §20 picks.
PEOPLE_AT = ("people who work at {firm} ({domain}) on acquisitions, M&A, corporate development, "
             "investments or technical due diligence")
PROFILE_WORDS = 500


def scout_firms(backend, kind: str, data: Path, want: int) -> List[Dict[str, Any]]:
    """Find firms of one kind, then the person to approach at each. One
    candidate per firm. A firm a contact works at is skipped: the practice
    knows it. Every firm looked at leaves a record under _firms/."""
    found = waiting(data, kind)
    if len(found) >= want:
        return found[:want]
    log = data / SCOUT_LOG
    earlier = [r["query"] for r in read_jsonl(log) if r.get("kind") == kind and r.get("firms")]
    out = _ask(backend, f"The kind of prospect: `{kind}`.\n\nSearches already made for firms of this kind:\n\n"
               + ("\n".join(f"- {q}" for q in earlier) or "(none)")
               + "\n\nThis step proposes searches for firms. Emit the answer per PROSPECT.md §18.",
               schemas.scout_schema(), 4096)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    for q in [x.strip() for x in obj.get("queries") or [] if isinstance(x, str) and x.strip()][:3]:
        results = exa.search(q, "company")
        new = 0
        for r in results:
            if len(found) >= want:
                break
            firm, url, text = str(r.get("title") or "").strip(), str(r.get("url") or ""), str(r.get("text") or "")
            domain = urlparse(url).netloc.lower().removeprefix("www.")
            firm_dir = data / FIRMS / slug(firm)
            if not slug(firm) or firm_dir.exists() or len(text.split()) < MIN_WORDS \
                    or contacts.firm_known(firm, domain):
                continue
            new += 1
            firm_dir.mkdir(parents=True)
            shown, _ = schemas.numbered(text.splitlines(), PAGE_WORDS)
            look = _ask(backend, f"The firm found: {firm} ({url}).\n\nIts profile, with line numbers:\n\n{shown}\n\n"
                                 f"This step takes a first look at a firm. Emit the answer per PROSPECT.md §19.",
                        schemas.first_look_schema(), 2048)
            lo = look.get("obj") if isinstance(look.get("obj"), dict) else {}
            rec: Dict[str, Any] = {"at": today(), "firm": firm, "url": url, "kind_sought": kind, "query": q,
                                   "fits": lo.get("fits"), "prospect_type": lo.get("prospect_type"),
                                   "reason": str(lo.get("reason") or "").strip()}
            logger.info("scout: firm %s: fits=%s (%s)", firm, rec["fits"], rec["reason"][:90])
            if rec["fits"] == "yes":
                c = person_at(backend, firm, url, domain, rec, text, firm_dir, data)
                if c is not None:
                    found.append(c)
            _write_json(firm_dir / "firm.json", rec)
        append_jsonl(log, {"at": today(), "kind": kind, "firms": True, "query": q,
                           "results": len(results), "new": new})
    return found[:want]


def person_at(backend, firm: str, url: str, domain: str, rec: Dict[str, Any], firm_text: str,
              firm_dir: Path, data: Path) -> Optional[Dict[str, Any]]:
    """The person to approach at one firm, as a candidate; None when the
    search shows nobody suitable or the choice cannot be checked. What was
    chosen and why is written into `rec`."""
    people = exa.search(PEOPLE_AT.format(firm=firm, domain=domain), "people")
    files: Dict[str, Dict[str, Any]] = {}
    for n, r in enumerate(people, 1):
        text = str(r.get("text") or "")
        if str(r.get("title") or "").strip() and len(text.split()) >= MIN_WORDS:
            name = save_evidence(firm_dir, n, str(r["title"]), {
                "Source": f"Professional profile as the search service holds it, {r.get('url') or ''}",
                "Date of the text": str(r.get("publishedDate") or "")[:10]}, text)
            files[name] = r
    if not files:
        rec["chosen"] = None
        rec["chosen_reason"] = "the people search found nobody"
        return None
    shown = "\n\n".join(f"Profile file `{f}`:\n\n"
                         + schemas.numbered((firm_dir / f).read_text(encoding='utf-8').splitlines(), PROFILE_WORDS)[0]
                         for f in files)
    firm_shown, _ = schemas.numbered(firm_text.splitlines(), PROFILE_WORDS)
    out = _ask(backend, f"The firm: {firm} ({url}), a `{rec['prospect_type']}`.\n\nIts profile:\n\n{firm_shown}\n\n"
                        f"People a search found for this firm:\n\n{shown}\n\n"
                        f"This step chooses whom to approach at the firm. Emit the answer per PROSPECT.md §20.",
               schemas.whom_schema(), 4096)
    obj = out.get("obj") if isinstance(out.get("obj"), dict) else {}
    first, alt = str(obj.get("first") or "").strip(), str(obj.get("alternate") or "").strip()
    evidence = {f: (firm_dir / f).read_text(encoding="utf-8").splitlines() for f in files}
    kept, _ = schemas.check_citations([obj.get("first_citation")], evidence)
    rec.update(chosen=None, chosen_reason=str(obj.get("reason") or "").strip(),
               alternate=str(files[alt].get("title")) if alt in files else None)
    if first not in files:
        rec["chosen_reason"] = (rec["chosen_reason"] + " (the model chose nobody)").strip()
        return None
    if not kept or kept[0]["file"] != first:
        rec["chosen_reason"] = (rec["chosen_reason"] + f" ({files[first].get('title')} was chosen, but the citation "
                                "showing they work at the firm did not match their profile)").strip()
        return None
    r = files[first]
    name = str(r["title"]).strip()
    cand_dir = data / slug(name)
    if cand_dir.exists() or contacts.known(name):
        rec["chosen_reason"] += f" ({name} is already known)"
        return None
    rec["chosen"] = name
    c = {"name": name, "firm": firm, "title": str(obj.get("first_role") or "").strip(),
         "urls": [u for u in (url, str(r.get("url") or "")) if u],
         "notes": (f"Found by the firm scout search: {rec['query']}. Chosen at {firm}: {rec['chosen_reason']}"
                   + (f" Alternate at the firm, not to be approached meanwhile: {rec['alternate']}."
                      if rec["alternate"] else "")),
         "pasted": [{"source": f"Professional profile as the search service holds it, {r.get('url') or ''}",
                     "date": str(r.get("publishedDate") or "")[:10], "text": str(r.get("text") or "")}]}
    cand_dir.mkdir(parents=True)
    atomic_write_text(cand_dir / "candidate.yaml", yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
    _write_json(cand_dir / "first_look.json", {"at": today(), "fits": "yes", "prospect_type": rec["prospect_type"],
                                               "reason": f"Chosen at {firm}: {rec['chosen_reason']}"})
    return c


def summary(cands: List[Dict[str, Any]], data: Path) -> str:
    rows = [f"# Outreach run of {today()}", ""]
    for c in cands:
        q = _read_json(data / slug(c["name"]) / "qualification.json") or {}
        why = q.get("reject_reason") or q.get("why_now") or q.get("why_person") or ""
        rows.append(f"- **{c['name']}**: {q.get('category')} ({q.get('prospect_type')}). {why}"
                    + (f" CHECK: {'; '.join(q['flags'])}" if q.get("flags") else ""))
    text = "\n".join(rows) + "\n"
    atomic_write_text(data / "summary.md", text)
    return text


# ---- the command line -------------------------------------------------------

def load_candidates(path: Path) -> List[Dict[str, Any]]:
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    cands = doc.get("candidates") if isinstance(doc, dict) else doc
    if not isinstance(cands, list) or not all(isinstance(c, dict) and c.get("name") for c in cands):
        raise SystemExit(f"{path}: expected a list of candidates, each with a `name`")
    return cands


#: The kinds the daily run scouts for, in turn, and those scouted by firm
#: because the person to approach has to be chosen (PROSPECT.md §4).
DAILY_KINDS = ("M&A adviser", "Repeat acquirer", "Searcher", "Small PE-family office",
               "Technical feedback", "VC / Investor")
FIRM_KINDS = ("Repeat acquirer", "Small PE-family office")
MODEL = REPO / "measure/models/local_qwen38flashnext.yaml"
WRITER = REPO / "measure/models/anthropic_opus55_medium.yaml"
RECORDS = {"research": "research.json", "qualify": "qualification.json", "draft": "draft.json"}


def work(backend, writer, cands: List[Dict[str, Any]], stages, data: Path, use_contacts: bool,
         redo: bool = False) -> None:
    """Run the named stages for each candidate. A stage whose record exists
    is not run again unless `redo`."""
    for c in cands:
        cand_dir = data / slug(c["name"])
        cand_dir.mkdir(parents=True, exist_ok=True)
        atomic_write_text(cand_dir / "candidate.yaml", yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
        for st in stages:
            if st == "push":
                push(c, cand_dir)
                continue
            if st == "brief":
                brief(c, cand_dir)
                continue
            if (cand_dir / RECORDS[st]).is_file() and not redo:
                logger.info("%s: %s is already recorded", c["name"], st)
                continue
            logger.info("%s: %s", c["name"], st)
            if st == "qualify":
                q = qualify(backend, c, cand_dir,
                            contacts.contact_for(c["name"], str(c.get("firm") or ""), firm_domain(c)) if use_contacts else "")
                better_contact(c, q, data, use_contacts)
                continue
            if st == "draft":
                draft(writer, c, cand_dir)
                continue
            research(backend, c, cand_dir)


def unpushed(data: Path) -> List[Dict[str, Any]]:
    """People scouting found who are qualified and not yet written to the contacts."""
    return [yaml.safe_load((f.parent / "candidate.yaml").read_text(encoding="utf-8"))
            for f in sorted(data.glob("*/first_look.json"))
            if (f.parent / "qualification.json").is_file() and not (f.parent / "pushed.json").is_file()]


def next_kind(day: Optional[datetime.date] = None) -> str:
    """The kind for the day: DAILY_KINDS in turn, by the date, so that the
    people approached in a week are of different kinds."""
    return DAILY_KINDS[(day or datetime.date.today()).toordinal() % len(DAILY_KINDS)]


#: How many backups of the records `backup` keeps.
BACKUPS_KEPT = 14


def backup(data: Path) -> Path:
    """Copy the records (the contacts and every person's files; not the logs)
    to _backups/<date and time>.tar.gz under `data`, and keep the newest
    BACKUPS_KEPT. Raises when the copy cannot be made: the day's work does not
    start without one."""
    import tarfile
    out = data / "_backups"
    out.mkdir(parents=True, exist_ok=True)
    dest = out / f"{datetime.datetime.now().strftime('%Y-%m-%dT%H-%M-%S')}.tar.gz"
    tmp = dest.with_name(dest.name + ".tmp")
    with tarfile.open(tmp, "w:gz") as tar:
        for p in sorted(data.iterdir()):
            if p.name != "_backups" and p.suffix != ".log":
                tar.add(p, arcname=p.name)
    tmp.replace(dest)
    for old in sorted(out.glob("*.tar.gz"))[:-BACKUPS_KEPT]:
        old.unlink()
    return dest


def daily(backend, writer, data: Path, pool: int, want: int) -> str:
    """The whole day's work, in order: a backup of the records; the names waiting at Research in the contacts;
    the replies recorded and not yet read, and the follow-ups that are due;
    then, when fewer than `pool` people are Ready to contact, a scout for the
    day's kind. Everything qualified is pushed. Returns what a
    person needs to read."""
    data.mkdir(parents=True, exist_ok=True)
    logger.info("backup: %s", backup(data))
    named = pickup(data)
    work(backend, writer, named, STAGES + ("push",), data, use_contacts=True)
    work(backend, writer, unpushed(data), ("push",), data, use_contacts=True)      # left by an earlier scout
    read, drafted = replies(backend, writer, data), followups(writer, data)
    ready = sum(1 for e in contacts.entries() if contacts.stage_of(e) == "Ready to contact")
    lines = [f"Names you added, researched today: {len(named)}.",
             f"Replies read: {len(read)}{' (' + ', '.join(read) + ')' if read else ''}.",
             f"Follow-ups drafted: {len(drafted)}{' (' + ', '.join(drafted) + ')' if drafted else ''}.",
             f"Ready to contact before scouting: {ready} (scouting starts below {pool})."]
    scouted: List[Dict[str, Any]] = []
    if ready < pool:
        kind = next_kind()
        by_firm = kind in FIRM_KINDS
        lines.append(f"Scouted for: {kind}{', by firm' if by_firm else ''}.")
        scouted = (scout_firms if by_firm else scout)(backend, kind, data, want)
        work(backend, writer, scouted, STAGES + ("push",), data, use_contacts=True)
    ready = sum(1 for e in contacts.entries() if contacts.stage_of(e) == "Ready to contact")
    lines.append(f"Ready to contact now: {ready}.")
    return summary(named + scouted, data) + "\n" + "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=STAGES + ("run", "push", "scout", "daily", "followups", "replies"))
    ap.add_argument("--kind", choices=[k for k in schemas.TYPES if k != "none"], default=None,
                    help="for `scout`: the kind of prospect to look for")
    ap.add_argument("--candidates", type=Path, default=None,
                    help="a YAML file of candidates; without it, the contacts at stage Research")
    ap.add_argument("--model", type=Path, default=MODEL, help="default: the local Qwen model file")
    ap.add_argument("--writer", type=Path, default=WRITER,
                    help="the model for the calls that write a message to send; default: Claude Opus 5.5")
    ap.add_argument("--firms", action="store_true",
                    help="for `scout`: find firms of the kind, then the person to approach at each")
    ap.add_argument("--want", type=int, default=3,
                    help="for `scout` and `daily`: how many of the people found go on to research now; "
                         "the rest wait for the next scout (research costs searches)")
    ap.add_argument("--pool", type=int, default=5,
                    help="for `daily`: scout when fewer than this many people are Ready to contact")
    ap.add_argument("--scouted", action="store_true",
                    help="the candidates are the people scouting found to fit who are qualified and not yet pushed")
    ap.add_argument("--only", default=None, help="one candidate, by slug (the name in lower case, _ for spaces)")
    ap.add_argument("--redo", action="store_true", help="run a stage again although its record exists")
    ap.add_argument("--contacts", action="store_true",
                    help="read the person's history from the contacts for the qualification (reads only)")
    ap.add_argument("--data", type=Path, default=DATA)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    stages = STAGES if args.stage in ("run", "scout") else (args.stage,)
    backend = backend_from_model(args.model) if stages not in (("brief",), ("push",)) else None
    writer = (backend_from_model(args.writer)
              if args.stage in ("daily", "followups", "replies") or "draft" in stages else None)
    if args.stage == "daily":
        print(daily(backend, writer, args.data, args.pool, args.want))
        return 0
    if args.stage in ("followups", "replies"):
        args.data.mkdir(parents=True, exist_ok=True)
        names = (followups(writer, args.data) if args.stage == "followups"
                 else replies(backend, writer, args.data))
        print(f"{args.stage}: {len(names)}" + (f" ({', '.join(names)})" if names else ""))
        return 0
    if args.stage == "scout":
        if not args.kind:
            raise SystemExit("scout needs --kind")
        args.data.mkdir(parents=True, exist_ok=True)
        cands = (scout_firms if args.firms else scout)(backend, args.kind, args.data, args.want)
    elif args.scouted:
        cands = unpushed(args.data)
    else:
        cands = load_candidates(args.candidates) if args.candidates else pickup(args.data)
    todo = [c for c in cands if args.only in (None, slug(c["name"]))]
    if not todo and args.only:
        raise SystemExit(f"no candidate with the slug {args.only}")
    work(backend, writer, todo, stages, args.data, args.contacts, args.redo)
    print(summary(cands, args.data))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
