"""
check-x-feed — new posts from the members of the user's curated X (Twitter) List.

Polls each List member individually via GET /2/users/{id}/tweets with
since_id, so under X's pay-per-use billing ($0.005 per post read) only
genuinely new posts are ever billed — a quiet feed costs ~nothing. The
List itself is used only for curation (membership refreshed ~daily);
its own /tweets endpoint has no since_id and would re-bill the same
posts on every call, which is why it is not used here.

No relevance filtering happens in this tool — it returns the raw new
posts and the ReAct loop judges what (if anything) is worth surfacing.

Required environment:
  X_BEARER_TOKEN  app-only bearer token from the X developer portal
  X_LIST_ID       numeric id of the curated X List to follow
"""

import json
import logging
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import requests

_logger = logging.getLogger(__name__)

_API_BASE = "https://api.twitter.com/2"
_STATE_PATH = Path("~/.cache/cognitive/check-x-feed/state.json").expanduser()
_MEMBERS_TTL_S = 24 * 3600      # refresh List membership daily
_BOOTSTRAP_RESULTS = 5          # first-ever fetch per member (endpoint minimum)
_MAX_RESULTS = 25               # per-member cap once since_id is known
_HTTP_TIMEOUT_S = 20


def _load_state() -> dict:
    if not _STATE_PATH.exists():
        return {}
    try:
        return json.loads(_STATE_PATH.read_text())
    except (json.JSONDecodeError, OSError) as e:
        _logger.warning(f"check-x-feed: could not read state, starting fresh: {e}")
        return {}


def _save_state(state: dict) -> None:
    try:
        _STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        _STATE_PATH.write_text(json.dumps(state, indent=2))
    except OSError as e:
        _logger.warning(f"check-x-feed: could not write state: {e}")


def _normalize_list_id(raw: str) -> Optional[str]:
    """Accept the numeric list id or a full list URL (x.com/i/lists/<id>)."""
    if raw.isdigit():
        return raw
    m = re.search(r"/lists/(\d+)", raw)
    return m.group(1) if m else None


class _AuthError(Exception):
    """Token rejected — abort the whole sweep, the user must fix credentials."""


class _RateLimited(Exception):
    """HTTP 429 — stop the sweep; since_id makes the remainder lossless."""


def _get(path: str, token: str, params: dict, log) -> dict:
    resp = requests.get(
        f"{_API_BASE}{path}",
        headers={"Authorization": f"Bearer {token}"},
        params=params,
        timeout=_HTTP_TIMEOUT_S,
    )
    if resp.status_code in (401, 403):
        raise _AuthError(f"X API rejected the token (HTTP {resp.status_code}): {resp.text[:200]}")
    if resp.status_code == 429:
        raise _RateLimited(f"X API rate limit (HTTP 429): {resp.text[:200]}")
    if resp.status_code != 200:
        raise requests.HTTPError(f"X API HTTP {resp.status_code} on {path}: {resp.text[:200]}")
    return resp.json()


def _refresh_members(state: dict, list_id: str, token: str, log) -> None:
    data = _get(
        f"/lists/{list_id}/members",
        token,
        {"max_results": 100, "user.fields": "username,name"},
        log,
    )
    members = [
        {"id": u["id"], "username": u.get("username", "?"), "name": u.get("name", "")}
        for u in data.get("data", [])
    ]
    if data.get("meta", {}).get("next_token"):
        log.warning("check-x-feed: list has >100 members; only the first 100 are tracked")
    state["list_id"] = list_id
    state["members"] = members
    state["members_fetched_at"] = datetime.now(timezone.utc).isoformat()
    log.info(f"check-x-feed: refreshed list {list_id} membership: {len(members)} members")


def _members_stale(state: dict, list_id: str) -> bool:
    if state.get("list_id") != list_id or not state.get("members"):
        return True
    try:
        fetched = datetime.fromisoformat(state["members_fetched_at"])
    except (KeyError, ValueError):
        return True
    return (datetime.now(timezone.utc) - fetched).total_seconds() > _MEMBERS_TTL_S


def _fetch_member_posts(member: dict, since_id: Optional[str], token: str, log) -> dict:
    params = {
        "exclude": "replies,retweets",
        "tweet.fields": "created_at,note_tweet",
    }
    if since_id:
        params["since_id"] = since_id
        params["max_results"] = _MAX_RESULTS
    else:
        params["max_results"] = _BOOTSTRAP_RESULTS
    return _get(f"/users/{member['id']}/tweets", token, params, log)


def _post_line(member: dict, tweet: dict) -> str:
    text = (tweet.get("note_tweet") or {}).get("text") or tweet.get("text", "")
    text = " ".join(text.split())
    when = tweet.get("created_at", "")[:16].replace("T", " ")
    url = f"https://x.com/{member['username']}/status/{tweet['id']}"
    return f"- **@{member['username']}** ({member['name']}) {when}Z:\n  {text}\n  {url}"


def react_invoke(args: Dict[str, Any], *, character_name=None, backend=None, logger=None) -> Dict[str, Any]:
    log = logger or _logger

    token = os.environ.get("X_BEARER_TOKEN", "").strip()
    list_raw = os.environ.get("X_LIST_ID", "").strip()
    if not token or not list_raw:
        missing = [n for n, v in (("X_BEARER_TOKEN", token), ("X_LIST_ID", list_raw)) if not v]
        return {"status": "error", "text": f"missing environment: {', '.join(missing)} — X feed checking is not configured"}
    list_id = _normalize_list_id(list_raw)
    if not list_id:
        return {"status": "error", "text": f"X_LIST_ID is not a list id or list URL: {list_raw!r} — expected the numeric id or an x.com/i/lists/<id> URL"}

    state = _load_state()
    try:
        if _members_stale(state, list_id):
            _refresh_members(state, list_id, token, log)
    except (_AuthError, _RateLimited, requests.RequestException) as e:
        log.error(f"check-x-feed: could not fetch list members: {e}")
        return {"status": "error", "text": f"could not fetch X list members: {e}"}

    since_ids = state.setdefault("since_ids", {})
    lines: list[str] = []
    bootstrapped: list[str] = []
    reads = 0
    partial = None

    for member in state.get("members", []):
        uid = member["id"]
        try:
            data = _fetch_member_posts(member, since_ids.get(uid), token, log)
        except _AuthError as e:
            log.error(f"check-x-feed: {e}")
            _save_state(state)
            return {"status": "error", "text": str(e)}
        except _RateLimited as e:
            log.warning(f"check-x-feed: rate limited at @{member['username']}, stopping sweep: {e}")
            partial = "Sweep stopped early by an X rate limit; remaining accounts will be picked up next check."
            break
        except requests.RequestException as e:
            log.warning(f"check-x-feed: skipping @{member['username']}: {e}")
            continue

        newest = data.get("meta", {}).get("newest_id")
        is_bootstrap = uid not in since_ids
        if newest and int(newest) > int(since_ids.get(uid, 0)):
            since_ids[uid] = newest
        tweets = data.get("data", [])
        reads += len(tweets)
        if is_bootstrap and tweets:
            bootstrapped.append(member["username"])
        for tweet in sorted(tweets, key=lambda t: t.get("created_at", ""), reverse=True):
            lines.append(_post_line(member, tweet))
        time.sleep(0.2)

    _save_state(state)
    log.info(f"check-x-feed: {reads} posts read this sweep across {len(state.get('members', []))} members")

    if not lines:
        return {"status": "ok", "text": "No new posts from the X list since the last check."}

    parts = [f"New posts from the X list ({len(lines)}):", ""]
    parts.extend(lines)
    if bootstrapped:
        parts.append("")
        parts.append(f"(First check for @{', @'.join(bootstrapped)} — their items above are recent history, not necessarily new.)")
    if partial:
        parts.append("")
        parts.append(f"({partial})")
    return {"status": "ok", "text": "\n".join(parts)}
