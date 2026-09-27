"""Exa /search, used to find people by description, and Exa's Agent API,
used to find one person's work email address.

`category: people` returns individuals with the text of their professional
profile as Exa's index holds it. This program never requests a page from
LinkedIn; what it reads is Exa's copy.

`find_email` starts one agent run (POST /agent/runs) whose output schema asks
for an address, then polls it (GET /agent/runs/{id}). Exa bills the run and,
separately, each address it finds.

Env vars:
  EXA_API_KEY — required
"""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List

import requests

API_URL = "https://api.exa.ai/search"
AGENT_URL = "https://api.exa.ai/agent/runs"
_TIMEOUT = 90.0
#: How long find_email waits for an agent run, and how often it asks.
AGENT_WAIT, AGENT_POLL = 300.0, 5.0


class ExaError(RuntimeError):
    """The search could not be made or its answer could not be used."""


def _key() -> str:
    key = os.getenv("EXA_API_KEY", "").strip()
    if not key:
        raise ExaError("EXA_API_KEY is not set")
    return key


def _json(resp: requests.Response) -> Dict[str, Any]:
    if resp.status_code not in (200, 201):
        raise ExaError(f"HTTP {resp.status_code}: {resp.text[:200]}")
    try:
        body = resp.json()
    except ValueError as e:
        raise ExaError(f"unparseable response: {e}") from e
    if not isinstance(body, dict):
        raise ExaError(f"not an object: {str(body)[:200]}")
    return body


def find_email(name: str, firm: str = "", title: str = "", linkedin: str = "") -> Dict[str, Any]:
    """{"email": the work address found or "", "sources": the URLs Exa cites
    for it, "confidence": Exa's word for it, "cost": the run's total in
    dollars, "run": the run id}. Raises when the run fails or does not finish
    within AGENT_WAIT. A cited source need not show the address itself: on the
    first live run (2026-09-27) both sources only confirmed the person's role."""
    who = ", ".join(x for x in (name, title, f"at {firm}" if firm else "") if x)
    body = {"query": f"Find the work email address of {who}." + (f" Their LinkedIn profile: {linkedin}" if linkedin else ""),
            "effort": "low",
            "input": {"data": [{"name": name, "company": firm, "linkedin_url": linkedin}]},
            "outputSchema": {"type": "object", "properties": {"email": {"type": "string", "format": "email"}}}}
    headers = {"x-api-key": _key()}
    try:
        run = _json(requests.post(AGENT_URL, headers=headers, timeout=_TIMEOUT, json=body))
        rid = str(run.get("id") or "")
        if not rid:
            raise ExaError(f"no run id in the response: {str(run)[:200]}")
        waited = 0.0
        while run.get("status") not in ("completed", "failed", "cancelled"):
            if waited >= AGENT_WAIT:
                raise ExaError(f"run {rid} did not finish in {AGENT_WAIT:.0f} s")
            time.sleep(AGENT_POLL)
            waited += AGENT_POLL
            run = _json(requests.get(f"{AGENT_URL}/{rid}", headers=headers, timeout=_TIMEOUT))
    except requests.exceptions.RequestException as e:
        raise ExaError(f"request failed: {e}") from e
    if run.get("status") != "completed":
        raise ExaError(f"run {rid} ended {run.get('status')}: {str(run.get('error') or '')[:200]}")
    out = run.get("output") or {}
    structured = out.get("structured") if isinstance(out.get("structured"), dict) else {}
    ground = [g for g in out.get("grounding") or [] if isinstance(g, dict) and g.get("field") == "structured.email"]
    return {"email": str(structured.get("email") or "").strip(),
            "sources": [str(c.get("url")) for g in ground for c in g.get("citations") or [] if isinstance(c, dict) and c.get("url")],
            "confidence": str(ground[0].get("confidence") or "") if ground else "",
            "cost": (run.get("costDollars") or {}).get("total"), "run": rid}


def search(query: str, category: str = "people", num_results: int = 10,
           max_chars: int = 6000) -> List[Dict[str, Any]]:
    """The results for one query: each a dict with `title` (for a person,
    their name), `url`, `publishedDate` (when Exa read the page) and `text`.
    An empty list means nothing was found; anything unreadable raises."""
    try:
        resp = requests.post(API_URL, headers={"x-api-key": _key()}, timeout=_TIMEOUT,
                             json={"query": query, "category": category, "type": "auto",
                                   "numResults": num_results,
                                   "contents": {"text": {"maxCharacters": max_chars}}})
    except requests.exceptions.RequestException as e:
        raise ExaError(f"request failed: {e}") from e
    body = _json(resp)
    if "results" not in body:
        raise ExaError(f"no `results` in the response: {str(body)[:200]}")
    return [r for r in body["results"] or [] if isinstance(r, dict)]
