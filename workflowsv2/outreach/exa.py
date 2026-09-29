"""Exa /search, used to find people by description.

`category: people` returns individuals with the text of their professional
profile as Exa's index holds it. This program never requests a page from
LinkedIn; what it reads is Exa's copy.

Work email addresses come from email_finder.py, not from Exa: its agent
lookup returned addresses its cited sources did not show (2026-09-27).

Env vars:
  EXA_API_KEY — required
"""
from __future__ import annotations

import os
from typing import Any, Dict, List

import requests

API_URL = "https://api.exa.ai/search"
_TIMEOUT = 90.0


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
