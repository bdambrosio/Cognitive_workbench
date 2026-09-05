"""Keep the Cloudflare Access policy in step with the engagements' client
emails, so adding a client on the practice page is one step, not two.

    CF_API_TOKEN_FILE    a file holding an API token with Access edit rights
    CF_ACCOUNT_ID        the Cloudflare account
    CF_ACCESS_APP_ID     the Access application for client.tuuyi.com

`ensure_emails(emails)` adds any address not already allowed to the
application's first Allow policy. It never removes an address: a client
whose engagement closed keeps a login that shows them nothing, and the
practice removes people by hand in the dashboard when it wants to.

Unconfigured (no token file, no ids) it does nothing and says so once in
the log, so the site runs without Cloudflare in local use and in tests.
"""
from __future__ import annotations

import json
import logging
import os
import urllib.request
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger("client_ui.cf_access")

API = "https://api.cloudflare.com/client/v4"


def _config() -> Optional[Dict[str, str]]:
    token_file = os.environ.get("CF_API_TOKEN_FILE")
    acct, app = os.environ.get("CF_ACCOUNT_ID"), os.environ.get("CF_ACCESS_APP_ID")
    if not (token_file and acct and app):
        return None
    p = Path(token_file).expanduser()
    if not p.is_file():
        logger.warning("CF_API_TOKEN_FILE %s is not a file; Access policy not synced", p)
        return None
    return {"token": p.read_text(encoding="utf-8").strip(), "account": acct, "app": app}


def _http(method: str, url: str, token: str, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers={
        "Authorization": f"Bearer {token}", "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode())


#: Test seam: replaced by a fake with the same signature.
transport: Callable[..., Dict[str, Any]] = _http


def _emails_in(policy: Dict[str, Any]) -> List[str]:
    out = []
    for rule in policy.get("include") or []:
        e = (rule.get("email") or {}).get("email")
        if e:
            out.append(e.strip().lower())
    return out


def ensure_emails(emails: List[str]) -> Dict[str, Any]:
    """Add `emails` to the application's first Allow policy. Returns
    {"synced": bool, "added": [...], "reason": str}."""
    want = sorted({e.strip().lower() for e in emails if e and e.strip()})
    cfg = _config()
    if cfg is None:
        return {"synced": False, "added": [], "reason": "Cloudflare Access not configured"}
    if not want:
        return {"synced": True, "added": [], "reason": "no emails"}
    base = f"{API}/accounts/{cfg['account']}/access/apps/{cfg['app']}/policies"
    try:
        listing = transport("GET", base, cfg["token"])
        policies = [p for p in (listing.get("result") or []) if p.get("decision") == "allow"]
        if not policies:
            return {"synced": False, "added": [], "reason": "the application has no Allow policy"}
        policy = sorted(policies, key=lambda p: p.get("precedence") or 0)[0]
        have = set(_emails_in(policy))
        new = [e for e in want if e not in have]
        if not new:
            return {"synced": True, "added": [], "reason": "already allowed"}
        include = list(policy.get("include") or []) + [{"email": {"email": e}} for e in new]
        body = {k: policy[k] for k in ("name", "decision", "precedence", "exclude", "require",
                                       "session_duration") if k in policy}
        body["include"] = include
        res = transport("PUT", f"{base}/{policy['id']}", cfg["token"], body)
        if not res.get("success"):
            return {"synced": False, "added": [], "reason": f"Cloudflare said {res.get('errors')}"}
        logger.info("Access policy %r now allows %s", policy.get("name"), ", ".join(new))
        return {"synced": True, "added": new, "reason": "added"}
    except Exception as e:                                     # noqa: BLE001
        logger.error("Access policy sync failed: %s", e)
        return {"synced": False, "added": [], "reason": f"{type(e).__name__}: {e}"}
