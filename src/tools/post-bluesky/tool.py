"""
Bluesky posting tool (AT Protocol).

Authenticates via com.atproto.server.createSession and creates a post
via com.atproto.repo.createRecord.
Body content comes from input_value (resolved from target/value by executor).

Env vars:
  BLUESKY_ACCOUNT_HANDLE — Bluesky handle (e.g. alice.bsky.social)
  BLUESKY_APP_PASSWORD   — App Password from Bluesky Settings

Zero pip dependencies — stdlib only (urllib, json, ssl).
"""

import hashlib
import json
import logging
import os
import ssl
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from typing import Any, Dict, Optional, Tuple

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
BSKY_API = "https://bsky.social/xrpc"
MAX_POST_GRAPHEMES = 300
DEDUP_WINDOW_SECS = 600  # 10 minutes

# ---------------------------------------------------------------------------
# Dedup cache: content_hash → (timestamp, cached_result)
# ---------------------------------------------------------------------------
_dedup_cache: Dict[str, Tuple[float, Dict]] = {}


def _dedup_key(body: str) -> str:
    return hashlib.sha256(body.encode("utf-8")).hexdigest()[:16]


def _check_dedup(body: str) -> Optional[Dict]:
    """Return cached result if an identical post happened within the dedup window."""
    key = _dedup_key(body)
    entry = _dedup_cache.get(key)
    if entry:
        ts, cached_result = entry
        if time.monotonic() - ts < DEDUP_WINDOW_SECS:
            return cached_result
        del _dedup_cache[key]
    return None


def _record_dedup(body: str, result: Dict):
    key = _dedup_key(body)
    _dedup_cache[key] = (time.monotonic(), result)

# ---------------------------------------------------------------------------
# Uniform return helpers
# ---------------------------------------------------------------------------

def _fail(executor: InfospaceExecutor, reason: str,
          value: Optional[str] = None, extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "failed", value=value or reason, reason=reason, extra=extra,
    )


def _success(executor: InfospaceExecutor, value: str,
             resource_id: Optional[str], extra: Optional[Dict[str, Any]] = None):
    return executor._create_uniform_return(
        "success", value=value, resource_id=resource_id, extra=extra,
    )


# ---------------------------------------------------------------------------
# Note helper
# ---------------------------------------------------------------------------

def _create_note(text_content: str, agent_name: str, resource_manager,
                 source_skill: str = 'post-bluesky',
                 tool_metadata: Optional[Dict] = None) -> Optional[str]:
    """Create a Note for the post confirmation."""
    if not resource_manager:
        logger.error("resource_manager required for creating Notes")
        return None
    success, note_id, error_msg, _ = resource_manager.create_note(
        character_name=agent_name,
        content=text_content,
        format_type='text',
        source_skill=source_skill,
        source_value=(text_content or '')[:100],
        note_name='',
        extra_props={'tool_metadata': tool_metadata or {}},
    )
    if success:
        return note_id
    logger.error(f"Failed to create Note: {error_msg}")
    return None


# ---------------------------------------------------------------------------
# HTTP helper (stdlib only — no requests dependency)
# ---------------------------------------------------------------------------

def _api_request(endpoint: str, payload: Dict, access_token: Optional[str] = None) -> Dict:
    """POST JSON to a Bluesky XRPC endpoint and return the parsed response."""
    url = f"{BSKY_API}/{endpoint}"
    data = json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"}
    if access_token:
        headers["Authorization"] = f"Bearer {access_token}"

    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    ctx = ssl.create_default_context()

    with urllib.request.urlopen(req, context=ctx) as resp:
        return json.loads(resp.read().decode("utf-8"))


# ---------------------------------------------------------------------------
# Main tool entry point
# ---------------------------------------------------------------------------

def tool(input_value, runtime=None, **kwargs):
    """
    Bluesky posting tool.

    Args:
        input_value: Post body text (from target/value, resolved by executor)
        **kwargs: agent_name, executor, resource_manager

    Returns:
        uniform_return with Note ID containing post confirmation
    """
    executor: InfospaceExecutor = kwargs.get("executor")
    if not executor:
        return {"status": "failed", "reason": "executor not available",
                "value": None, "resource_id": None}

    agent_name = kwargs.get('agent_name')
    if not agent_name:
        return _fail(executor, 'agent_name required in kwargs')

    resource_manager = kwargs.get('resource_manager')
    if not resource_manager:
        return _fail(executor, 'resource_manager required in kwargs')

    # Credentials from environment
    handle = os.getenv('BLUESKY_ACCOUNT_HANDLE', '').strip()
    app_password = os.getenv('BLUESKY_APP_PASSWORD', '').strip()
    if not handle or not app_password:
        return _fail(executor,
                     'BLUESKY_ACCOUNT_HANDLE and BLUESKY_APP_PASSWORD environment variables required')

    # Body from input_value
    body = ''
    if input_value:
        body = str(input_value)
    if not body:
        return _fail(executor, 'post body required (use target or value)')

    # Append tags as hashtags
    tags = kwargs.get("tags") or []
    if isinstance(tags, str):
        tags = [t.strip() for t in tags.split(",") if t.strip()]
    tag_suffix = ""
    if tags:
        tag_suffix = "\n\n" + " ".join(f"#{t.lstrip('#')}" for t in tags)

    # Truncate body to fit within 300 graphemes including tags
    max_body = 294 - len(tag_suffix)
    if len(body) > max_body:
        body = body[:max_body] + '...'

    # Build facets for hashtag rich-text linking
    facets = []
    full_text = body + tag_suffix
    text_bytes = full_text.encode("utf-8")
    for tag in tags:
        tag_clean = tag.lstrip("#")
        hashtag = f"#{tag_clean}"
        # Find byte position of this hashtag in the full text
        hashtag_bytes = hashtag.encode("utf-8")
        byte_start = text_bytes.find(hashtag_bytes)
        if byte_start >= 0:
            facets.append({
                "index": {
                    "byteStart": byte_start,
                    "byteEnd": byte_start + len(hashtag_bytes),
                },
                "features": [{
                    "$type": "app.bsky.richtext.facet#tag",
                    "tag": tag_clean,
                }],
            })
    body = full_text

    # Dedup check — reject identical posts within 10-minute window
    cached = _check_dedup(body)
    if cached:
        logger.warning(f"post-bluesky: dedup hit — identical post sent within last {DEDUP_WINDOW_SECS}s, "
                       f"returning cached result")
        return cached

    # Authenticate
    try:
        session = _api_request("com.atproto.server.createSession", {
            "identifier": handle,
            "password": app_password,
        })
    except urllib.error.HTTPError as e:
        error_body = e.read().decode("utf-8", errors="replace") if e.fp else ""
        logger.error(f"Bluesky auth failed ({e.code}): {error_body}")
        if e.code == 401:
            return _fail(executor, 'authentication_failed',
                         value='Bluesky authentication failed. Check BLUESKY_ACCOUNT_HANDLE and BLUESKY_APP_PASSWORD.')
        return _fail(executor, 'authentication_failed',
                     value=f'Bluesky authentication error (HTTP {e.code}).')
    except Exception as e:
        logger.error(f"Bluesky auth error: {e}")
        return _fail(executor, 'authentication_failed',
                     value='Failed to authenticate with Bluesky.')

    access_token = session.get("accessJwt")
    did = session.get("did")
    if not access_token or not did:
        return _fail(executor, 'authentication_failed',
                     value='Bluesky session missing accessJwt or did.')

    # Create post
    now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    post_record = {
        "$type": "app.bsky.feed.post",
        "text": body,
        "createdAt": now,
    }
    if facets:
        post_record["facets"] = facets

    logger.info(f"post-bluesky: posting as {handle} ({len(body)} chars)")

    try:
        result = _api_request("com.atproto.repo.createRecord", {
            "repo": did,
            "collection": "app.bsky.feed.post",
            "record": post_record,
        }, access_token=access_token)
    except urllib.error.HTTPError as e:
        error_body = e.read().decode("utf-8", errors="replace") if e.fp else ""
        logger.error(f"Bluesky create post failed ({e.code}): {error_body}")
        return _fail(executor, 'post_failed',
                     value=f'Failed to create Bluesky post (HTTP {e.code}).')
    except Exception as e:
        logger.error(f"Bluesky post error: {e}")
        return _fail(executor, 'post_failed',
                     value='Failed to create Bluesky post.')

    post_uri = result.get("uri", "")
    post_cid = result.get("cid", "")

    # Build confirmation
    timestamp = datetime.now(timezone.utc).isoformat()
    preview = body[:80] + ('...' if len(body) > 80 else '')
    confirmation_text = f"Posted to Bluesky: {preview}"

    metadata = {
        'uri': post_uri,
        'cid': post_cid,
        'handle': handle,
        'timestamp': timestamp,
    }

    note_id = _create_note(confirmation_text, agent_name, resource_manager,
                           tool_metadata=metadata)
    if not note_id:
        result = _success(executor, confirmation_text, None, extra=metadata)
        _record_dedup(body, result)
        return result

    logger.info(f"post-bluesky: posted uri={post_uri} cid={post_cid}")
    result = _success(executor, confirmation_text, note_id, extra=metadata)
    _record_dedup(body, result)
    return result


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    handle = os.getenv('BLUESKY_ACCOUNT_HANDLE', '').strip()
    app_password = os.getenv('BLUESKY_APP_PASSWORD', '').strip()

    if not handle or not app_password:
        print("Set BLUESKY_ACCOUNT_HANDLE and BLUESKY_APP_PASSWORD environment variables")
        sys.exit(1)

    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <post_text>")
        sys.exit(1)

    text = sys.argv[1]
    if len(text) > 294:
        text = text[:294] + '...'

    print(f"Posting as {handle}")
    print(f"Text: {text}")
    print()

    try:
        session = _api_request("com.atproto.server.createSession", {
            "identifier": handle,
            "password": app_password,
        })
        now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        result = _api_request("com.atproto.repo.createRecord", {
            "repo": session["did"],
            "collection": "app.bsky.feed.post",
            "record": {
                "$type": "app.bsky.feed.post",
                "text": text,
                "createdAt": now,
            },
        }, access_token=session["accessJwt"])
        print(f"Posted! URI: {result.get('uri')}")
        print(f"CID: {result.get('cid')}")
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace") if e.fp else ""
        print(f"HTTP error {e.code}: {body}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
