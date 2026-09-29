"""Address-finding services: one verified work address for one person.

Findymail is asked first (it found 15 of 19 US prospects on 2026-09-29, and
returns only addresses it has verified); Prospeo is the second source (10 of
19, and no charge when it finds nothing). Each returns the organisation it
ties the address to, which the practice's own check reads against the
evidence: each service returned one address at an employer the person had
left.

Env vars:
  FINDYMAIL_API_KEY — Bearer token
  PROSPEO_API_KEY   — sent as X-KEY
"""
from __future__ import annotations

import os
from typing import Any, Dict

import requests

FINDYMAIL_URL = "https://app.findymail.com/api/search"
PROSPEO_URL = "https://api.prospeo.io/enrich-person"
_TIMEOUT = 120.0


class FinderError(RuntimeError):
    """The service could not be asked, or its answer could not be read."""


def _key(name: str) -> str:
    key = os.environ.get(name, "").strip()
    if not key:
        raise FinderError(f"{name} is not set")
    return key


def _post(url: str, headers: Dict[str, str], body: Dict[str, Any]) -> Dict[str, Any]:
    try:
        resp = requests.post(url, headers=headers, json=body, timeout=_TIMEOUT)
        return resp.json()
    except (requests.exceptions.RequestException, ValueError) as e:
        raise FinderError(f"{url}: {e}") from e


def findymail(name: str, linkedin: str = "", domain: str = "") -> Dict[str, str]:
    """{"email", "organisation"}, both empty when nothing was found. Asked by
    the LinkedIn link when there is one, then by name and firm domain."""
    headers = {"Authorization": f"Bearer {_key('FINDYMAIL_API_KEY')}", "Accept": "application/json"}
    tries = ([("business-profile", {"linkedin_url": linkedin})] if linkedin else []) \
        + ([("name", {"name": name, "domain": domain})] if domain else [])
    for path, body in tries:
        contact = _post(f"{FINDYMAIL_URL}/{path}", headers, body).get("contact") or {}
        if contact.get("email"):
            return {"email": str(contact["email"]).strip(),
                    "organisation": str(contact.get("company") or contact.get("domain") or "")}
    return {"email": "", "organisation": ""}


def prospeo(name: str, linkedin: str = "", domain: str = "") -> Dict[str, str]:
    """{"email", "organisation"}, both empty when nothing was found. Only an
    address Prospeo has verified is returned."""
    headers = {"X-KEY": _key("PROSPEO_API_KEY"), "Content-Type": "application/json"}
    first, _, last = name.partition(" ")
    tries = ([{"linkedin_url": linkedin}] if linkedin else []) \
        + ([{"first_name": first, "last_name": last, "company_website": domain}] if domain else [])
    for data in tries:
        body = _post(PROSPEO_URL, headers, {"data": data, "only_verified_email": True})
        email = ((body.get("person") or {}).get("email") or {})
        if email.get("email") and email.get("status") == "VERIFIED":
            return {"email": str(email["email"]).strip(),
                    "organisation": str((body.get("company") or {}).get("name") or "")}
    return {"email": "", "organisation": ""}
