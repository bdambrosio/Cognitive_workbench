"""Research, qualify and draft a first message for people the practice names.

    python3 workflowsv2/outreach/runner.py run --candidates <file.yaml> --model <yaml> [--only <slug>]
    python3 workflowsv2/outreach/runner.py research|qualify|draft|brief ...   (one stage)

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
               person's record in Attio is found by name or created, the person
               is put in the outreach list at "Ready to contact" with the
               rationale and the three selects, and the brief is attached as a
               note. A plausible candidate goes to "Qualified" with the brief;
               a weak or rejected one to "Not pursuing" with the reason. An
               existing person record is never changed, and an entry at any
               stage but Research is left alone: a person set it. Recorded in
               attio.json; a candidate with that file is not pushed again.

    scout      `scout --kind "<a kind from PROSPECT.md §4>" --model <yaml>`. The
               model proposes up to three descriptions of such people (§16),
               differing from the searches already logged; Exa's people search
               runs them; a name already in Attio or already in the record is
               skipped; each new person gets a first look at their profile
               (§17), which costs no search. Those who fit become candidates,
               with the profile as their first evidence file, and go through
               the four stages at once. Nothing found by scouting is written to
               Attio unless `push` later finds it strong. With `--firms` the
               search is for firms (§18, §19); for each firm that fits, one
               people search at that firm and one emission (§20) choose whom
               to approach, with a checked citation that they work there now.
               One candidate per firm; a firm Attio already has is skipped.

WITHOUT --candidates the candidates are the entries of the Attio outreach list
at stage Research: a person adds a name there, with how they know them in the
entry's Notes and any pasted text in a note titled "Evidence: <source>".

NOTHING IS SENT. A person reads the brief, edits the message and sends it.
The category is the model's and is never changed by code; what the checks
find is written into the brief for the person to weigh.

THE RECORD is `prospects/<slug>/` (not in git: it holds personal data):
candidate.yaml, evidence/*.md, research.json, qualification.json, draft.json,
brief.md, issues.jsonl. A stage whose record exists is not run again unless
`--redo` is given, because searches cost money and evidence is dated.
"""
from __future__ import annotations

import argparse
import datetime
import json
import logging
import re
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
from workflowsv2.outreach import attio, exa, schemas                    # noqa: E402
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


def slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


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


def better_contact(c: Dict[str, Any], q: Dict[str, Any], data: Path, use_attio: bool) -> Optional[Path]:
    """When the qualification names a better person to approach at the firm,
    that person becomes a candidate waiting for research, unless the practice
    already has them. Returns their directory when one was made."""
    name = str(q.get("better_contact_name") or "").strip()
    if not name or not slug(name) or slug(name) == slug(c["name"]):
        return None
    cand_dir = data / slug(name)
    if cand_dir.exists() or (use_attio and attio.known(name)):
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
    out = _ask(backend, user, schemas.draft_schema(), 8192)
    d, dropped, flags = schemas.clean_draft(out.get("obj"), evidence)
    if out.get("parse") not in ("parsed", "repaired") or not d["message"]:
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


#: The list stage each category is written as. `Not pursuing` is written only
#: when the list has that stage; until then such an entry stays where it is.
STAGE_OF = {"strong": "Ready to contact", "plausible": "Qualified",
            "weak": "Not pursuing", "reject": "Not pursuing"}
#: The stages this program may move an entry out of. Any other stage was set
#: by a person or records a message sent, and is left alone.
OURS = (None, "Research")


def push(c: Dict[str, Any], cand_dir: Path) -> Optional[Dict[str, Any]]:
    """Write one qualified candidate into Attio: the list stage for its
    category, the rationale and the three selects, and for a strong or
    plausible candidate the brief as a note. A strong candidate with no
    usable draft is not written. Returns the record of what was written."""
    q = _read_json(cand_dir / "qualification.json") or {}
    d = _read_json(cand_dir / "draft.json")
    category = q.get("category")
    if category not in STAGE_OF or (category == "strong" and not (d and d.get("message"))):
        logger.info("%s: no category, or strong with no draft; nothing is pushed", c["name"])
        return None
    if (cand_dir / "attio.json").is_file():
        logger.info("%s: already pushed", c["name"])
        return None
    person = attio.find_person(c["name"])
    created = person is None
    if created and category != "strong":
        return None                     # a name nobody put in Attio, and not worth putting there
    if created:
        linkedin = next((u for u in c.get("urls") or [] if "linkedin.com" in urlparse(u).netloc.lower()), "")
        person = attio.create_person(c["name"], str(c.get("title") or ""), linkedin)
    rid = person["id"]["record_id"]
    entry = None if created else attio.entry_of(rid)
    if entry is not None and attio.stage_of(entry) not in OURS:
        logger.info("%s: the entry is at '%s', set by a person; left alone", c["name"], attio.stage_of(entry))
        return None
    stage = STAGE_OF[category]
    if stage not in attio.stages():
        issues.note(cand_dir, "push", "no_stage", f"the outreach list has no stage '{stage}'; "
                    f"{c['name']} ({category}) was left where it is")
        return None
    why = q.get("reject_reason") or f"{q.get('why_person', '')} {q.get('why_now', '')}".strip()
    values: Dict[str, Any] = {"stage": stage, "fit_rationale": why}
    if category == "strong":
        values.update(next_action="Review the draft and send on LinkedIn", next_action_date=today())
    for slug_, key in (("category", "prospect_type"), ("relationship", "relationship"),
                       ("probelm_recognition", "problem_recognition")):   # the list's own spelling
        if q.get(key) and q[key] != "none":
            values[slug_] = q[key]
    written = attio.upsert_entry(rid, values)
    note_id = None
    if category in ("strong", "plausible"):
        note = attio.create_note(rid, f"Outreach brief {today()}", brief(c, cand_dir))
        note_id = (note.get("id") or {}).get("note_id")
    rec = {"at": today(), "record_id": rid, "person_created": created,
           "entry_id": written["id"]["entry_id"], "note_id": note_id, "entry_values": values}
    _write_json(cand_dir / "attio.json", rec)
    return rec


def pickup(data: Path) -> List[Dict[str, Any]]:
    """The candidates waiting in Attio: every entry of the outreach list at
    stage Research, as candidate records. This is how the practice hands a
    name to the workflow."""
    return [attio.candidate_from(e) for e in attio.entries() if attio.stage_of(e) == "Research"]


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
            if attio.known(name):
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
            if rec["fits"] == "yes":
                found.append(c)
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
    candidate per firm. A firm Attio already has is skipped: the practice
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
                    or attio.firm_record(firm, domain) is not None:
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
    if cand_dir.exists() or attio.known(name):
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=STAGES + ("run", "push", "scout"))
    ap.add_argument("--kind", choices=[k for k in schemas.TYPES if k != "none"], default=None,
                    help="for `scout`: the kind of prospect to look for")
    ap.add_argument("--candidates", type=Path, default=None,
                    help="a YAML file of candidates; without it, the entries at stage Research in Attio")
    ap.add_argument("--model", type=Path, default=None, help="required by every stage but `brief`")
    ap.add_argument("--firms", action="store_true",
                    help="for `scout`: find firms of the kind, then the person to approach at each")
    ap.add_argument("--want", type=int, default=5,
                    help="for `scout`: how many of the people found go on to research now; "
                         "the rest wait for the next scout (research costs searches)")
    ap.add_argument("--scouted", action="store_true",
                    help="the candidates are the people scouting found to fit who are qualified and not yet pushed")
    ap.add_argument("--only", default=None, help="one candidate, by slug (the name in lower case, _ for spaces)")
    ap.add_argument("--redo", action="store_true", help="run a stage again although its record exists")
    ap.add_argument("--attio", action="store_true",
                    help="read the person's history from Attio for the qualification (reads only)")
    ap.add_argument("--data", type=Path, default=DATA)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    stages = STAGES if args.stage in ("run", "scout") else (args.stage,)
    if stages not in (("brief",), ("push",)) and args.model is None:
        raise SystemExit("--model is required")
    backend = backend_from_model(args.model) if args.model else None
    if args.stage == "scout":
        if not args.kind:
            raise SystemExit("scout needs --kind")
        args.data.mkdir(parents=True, exist_ok=True)
        cands = (scout_firms if args.firms else scout)(backend, args.kind, args.data, args.want)
    elif args.scouted:
        cands = [yaml.safe_load((f.parent / "candidate.yaml").read_text(encoding="utf-8"))
                 for f in sorted(args.data.glob("*/first_look.json"))
                 if (f.parent / "qualification.json").is_file() and not (f.parent / "attio.json").is_file()]
    else:
        cands = load_candidates(args.candidates) if args.candidates else pickup(args.data)
    todo = [c for c in cands if args.only in (None, slug(c["name"]))]
    if not todo and args.only:
        raise SystemExit(f"no candidate with the slug {args.only}")
    records = {"research": "research.json", "qualify": "qualification.json", "draft": "draft.json"}
    for c in todo:
        cand_dir = args.data / slug(c["name"])
        cand_dir.mkdir(parents=True, exist_ok=True)
        atomic_write_text(cand_dir / "candidate.yaml", yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
        for st in stages:
            if st == "push":
                push(c, cand_dir)
                continue
            if st == "brief":
                brief(c, cand_dir)
                continue
            if (cand_dir / records[st]).is_file() and not args.redo:
                logger.info("%s: %s is already recorded", c["name"], st)
                continue
            logger.info("%s: %s", c["name"], st)
            if st == "qualify":
                q = qualify(backend, c, cand_dir,
                            attio.contact_for(c["name"], str(c.get("firm") or ""), firm_domain(c)) if args.attio else "")
                better_contact(c, q, args.data, args.attio)
                continue
            {"research": research, "qualify": qualify, "draft": draft}[st](backend, c, cand_dir)
    print(summary(cands, args.data))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
