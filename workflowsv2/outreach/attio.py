"""The practice's Attio workspace, as the outreach workflow uses it.

Attio holds the state of every approach: the person, their entry in the
outreach list with its stage, the notes, the follow-up tasks. This module
reads that state so a person already approached is qualified with the history
in view, and writes a reviewed candidate into it. It sends nothing to anyone.

Env vars:
  ATTIO_API_KEY — an access token with record, list entry, note and task
                  read-write, and object and list configuration read
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import requests

API = "https://api.attio.com/v2"
LIST = "tuuyi_outreach"
_TIMEOUT = 30.0


class AttioError(RuntimeError):
    """A request failed, or the workspace did not hold what the call needs."""


class SeveralOfOneName(AttioError):
    """More than one people record carries the name asked for."""


def _call(method: str, path: str, body: Optional[Dict[str, Any]] = None,
          params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    key = os.getenv("ATTIO_API_KEY", "").strip()
    if not key:
        raise AttioError("ATTIO_API_KEY is not set")
    try:
        resp = requests.request(method, API + path, json=body, params=params, timeout=_TIMEOUT,
                                headers={"Authorization": f"Bearer {key}"})
    except requests.exceptions.RequestException as e:
        raise AttioError(f"{method} {path}: {e}") from e
    if resp.status_code >= 300:
        raise AttioError(f"{method} {path}: HTTP {resp.status_code}: {resp.text[:300]}")
    try:
        out = resp.json()
    except ValueError as e:
        raise AttioError(f"{method} {path}: unparseable response: {e}") from e
    if method == "DELETE":
        return out if isinstance(out, dict) else {}
    if not isinstance(out, dict) or "data" not in out:
        raise AttioError(f"{method} {path}: no `data` in the response: {str(out)[:200]}")
    return out


def _first(record: Dict[str, Any], slug: str, field: str) -> Any:
    vals = (record.get("values") or record.get("entry_values") or {}).get(slug) or []
    return vals[0].get(field) if vals else None


# ---- reading ----------------------------------------------------------------

def find_person(name: str) -> Optional[Dict[str, Any]]:
    """The people record whose full name is `name`, or None. Raises when
    several records carry the name: a person decides which one is meant."""
    out = _call("POST", "/objects/people/records/query",
                {"filter": {"name": {"full_name": {"$eq": name}}}, "limit": 10})
    hits = out["data"]
    if len(hits) > 1:
        raise SeveralOfOneName(f"{len(hits)} people records are named {name!r}")
    return hits[0] if hits else None


def known(name: str) -> bool:
    """Whether the practice already has a person of this name."""
    try:
        return find_person(name) is not None
    except SeveralOfOneName:
        return True


def firm_record(firm: str = "", domain: str = "") -> Optional[Dict[str, Any]]:
    """The company record for a firm, by its web domain, else by its exact
    name. None when Attio has neither."""
    for f in ([{"domains": domain}] if domain else []) + ([{"name": firm}] if firm else []):
        hits = _call("POST", "/objects/companies/records/query", {"filter": f, "limit": 2})["data"]
        if hits:
            return hits[0]
    return None


def colleagues(company: Dict[str, Any], but: str = "") -> str:
    """The people Attio holds at this firm, other than the record `but`, each
    with their stage in the outreach list: what "one approach per firm" is
    judged from. Empty when there is nobody else."""
    rows: List[str] = []
    for ref in (company.get("values") or {}).get("team") or []:
        rid = ref.get("target_record_id")
        if not rid or rid == but:
            continue
        person = _call("GET", f"/objects/people/records/{rid}")["data"]
        entry = entry_of(rid)
        where = f"in the outreach list at stage '{stage_of(entry)}'" if entry else "not in the outreach list"
        rows.append(f"A colleague at the same firm, {_first(person, 'name', 'full_name')}, is {where}.")
    return "\n".join(rows)


def contact_record(person: Dict[str, Any]) -> str:
    """What Attio holds about approaches to this person, as the text the
    qualification reads: the list stage and last contact, each task linked
    to the person, and the title and date of each note. Empty when there is
    nothing."""
    rid = person["id"]["record_id"]
    rows: List[str] = []
    for ref in _call("GET", f"/objects/people/records/{rid}/entries")["data"]:
        if ref.get("list_api_slug") != LIST:
            continue
        e = _call("GET", f"/lists/{LIST}/entries/{ref['entry_id']}")["data"]
        stage = _first(e, "stage", "status")
        stage = stage.get("title") if isinstance(stage, dict) else stage
        rows.append(f"In the outreach list at stage '{stage}', last contact "
                    f"{_first(e, 'last_contact', 'value') or 'not recorded'}.")
    tasks = _call("GET", "/tasks", params={"linked_object": "people", "linked_record_id": rid,
                                           "limit": 50})["data"]
    for t in tasks:
        rows.append(f"Task ({'done' if t.get('is_completed') else 'open'}, "
                    f"due {(t.get('deadline_at') or 'no date')[:10]}): "
                    f"{' '.join((t.get('content_plaintext') or '').split())}")
    notes = _call("GET", "/notes", params={"parent_object": "people", "parent_record_id": rid,
                                           "limit": 50})["data"]
    for n in notes:
        rows.append(f"Note of {(n.get('created_at') or '')[:10]}: {n.get('title') or '(untitled)'}")
    return "\n".join(rows)


def contact_for(name: str, firm: str = "", domain: str = "") -> str:
    """Everything the qualification is told about earlier approaches: the
    person's own record when Attio has them, and their colleagues at the firm,
    found through the person's company or, for someone Attio does not have,
    through the firm's domain or name."""
    person = find_person(name)
    rows: List[str] = []
    company = None
    if person is not None:
        rows.append(contact_record(person))
        cid = _first(person, "company", "target_record_id")
        if cid:
            company = _call("GET", f"/objects/companies/records/{cid}")["data"]
    if company is None:
        company = firm_record(firm, domain)
    if company is not None:
        rows.append(colleagues(company, but=person["id"]["record_id"] if person else ""))
    return "\n".join(r for r in rows if r)


def stage_of(entry: Dict[str, Any]) -> Optional[str]:
    status = _first(entry, "stage", "status")
    return status.get("title") if isinstance(status, dict) else status


def entries() -> List[Dict[str, Any]]:
    """Every entry of the outreach list. The list is small; the stage is
    compared by the caller."""
    return _call("POST", f"/lists/{LIST}/entries/query", {"limit": 500})["data"]


def entry_of(record_id: str) -> Optional[Dict[str, Any]]:
    """The person's entry in the outreach list, or None."""
    for ref in _call("GET", f"/objects/people/records/{record_id}/entries")["data"]:
        if ref.get("list_api_slug") == LIST:
            return _call("GET", f"/lists/{LIST}/entries/{ref['entry_id']}")["data"]
    return None


def stages() -> List[str]:
    return [s["title"] for s in _call("GET", f"/lists/{LIST}/attributes/stage/statuses")["data"]
            if not s.get("is_archived")]


#: A note on the person whose title starts with this is text the practice
#: pasted as evidence (a LinkedIn post, an email); the rest of the title says
#: where it came from.
EVIDENCE_NOTE = "Evidence"


def candidate_from(entry: Dict[str, Any]) -> Dict[str, Any]:
    """One outreach-list entry as a candidate record for the runner: the
    person's name, title, firm and links; the entry's `notes` text and
    `relationship` as the relationship note; each evidence note as pasted text."""
    rid = entry["parent_record_id"]
    person = _call("GET", f"/objects/people/records/{rid}")["data"]
    c: Dict[str, Any] = {"name": _first(person, "name", "full_name"), "attio_record_id": rid}
    if _first(person, "job_title", "value"):
        c["title"] = _first(person, "job_title", "value")
    urls: List[str] = []
    company = _first(person, "company", "target_record_id")
    if company:
        co = _call("GET", f"/objects/companies/records/{company}")["data"]
        c["firm"] = _first(co, "name", "value")
        if _first(co, "domains", "domain"):
            urls.append(f"https://{_first(co, 'domains', 'domain')}")
    if _first(person, "linkedin", "value"):
        urls.append(_first(person, "linkedin", "value"))
    if urls:
        c["urls"] = urls
    option = _first(entry, "relationship", "option")
    said = [f"The practice marked the relationship: {option.get('title')}."] if isinstance(option, dict) else []
    if _first(entry, "notes", "value"):
        said.append(str(_first(entry, "notes", "value")).strip())
    if said:
        c["relationship"] = "\n".join(said)
    notes = _call("GET", "/notes", params={"parent_object": "people", "parent_record_id": rid,
                                           "limit": 50})["data"]
    pasted = [{"source": (n.get("title") or "")[len(EVIDENCE_NOTE):].strip(" :") or "text pasted by the practice",
               "date": (n.get("created_at") or "")[:10], "text": n.get("content_plaintext") or ""}
              for n in notes if (n.get("title") or "").startswith(EVIDENCE_NOTE)]
    if pasted:
        c["pasted"] = pasted
    return c


# ---- writing ----------------------------------------------------------------

def create_person(name: str, job_title: str = "", linkedin: str = "") -> Dict[str, Any]:
    first, _, last = name.strip().partition(" ")
    values: Dict[str, Any] = {"name": [{"first_name": first, "last_name": last, "full_name": name.strip()}]}
    if job_title:
        values["job_title"] = job_title
    if linkedin:
        values["linkedin"] = linkedin
    return _call("POST", "/objects/people/records", {"data": {"values": values}})["data"]


def upsert_entry(record_id: str, values: Dict[str, Any]) -> Dict[str, Any]:
    """The person's entry in the outreach list, created or updated. Only the
    attributes named in `values` are written."""
    return _call("PUT", f"/lists/{LIST}/entries",
                 {"data": {"parent_object": "people", "parent_record_id": record_id,
                           "entry_values": values}})["data"]


def create_note(record_id: str, title: str, markdown: str) -> Dict[str, Any]:
    return _call("POST", "/notes", {"data": {"parent_object": "people", "parent_record_id": record_id,
                                             "title": title, "format": "markdown",
                                             "content": markdown}})["data"]


def not_pursuing(record_id: str, reason: str, on: str) -> None:
    """Record that the practice will not approach this person: a note on the
    person, "Not pursuing <date>", holding the reason and what the list entry
    said, and the entry removed from the outreach list. The list has no such
    stage and the practice's token cannot add one; the note is the record, the
    person stays in Attio so scouting does not bring them back, and the next
    qualification reads the note's title and date in the contact record."""
    entry = entry_of(record_id)
    was = ""
    if entry is not None:
        was = (f"\n\nThe entry was at stage '{stage_of(entry)}'. "
               f"Fit rationale then: {_first(entry, 'fit_rationale', 'value') or '(none)'}")
    create_note(record_id, f"Not pursuing {on}", reason.strip() + was)
    if entry is not None:
        _call("DELETE", f"/lists/{LIST}/entries/{entry['id']['entry_id']}")
