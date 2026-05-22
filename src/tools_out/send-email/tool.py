"""
Gmail SMTP email sending tool.

Connects to Gmail via SMTP (App Password auth) and sends a plain-text email.
Body content comes from input_value (resolved from target/value by executor).

Env vars:
  GMAIL_ADDRESS      — Gmail address (also the SMTP username and From address)
  GMAIL_APP_PASSWORD — 16-char App Password from Google Account settings

Zero pip dependencies — stdlib only (smtplib, email, ssl).
"""

import email.utils
import hashlib
import logging
import os
import smtplib
import ssl
import time
from datetime import datetime, timezone
from email.mime.text import MIMEText
from typing import Any, Dict, List, Optional, Tuple

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SMTP_HOST = "smtp.gmail.com"
SMTP_PORT = 587
MAX_BODY_CHARS = 100_000
DEDUP_WINDOW_SECS = 600  # 10 minutes

# ---------------------------------------------------------------------------
# Dedup cache: (content_hash, key_args_hash) → (timestamp, cached_result)
# ---------------------------------------------------------------------------
_dedup_cache: Dict[Tuple[str, str], Tuple[float, Dict]] = {}


def _dedup_key(body: str, to: str, subject: str) -> Tuple[str, str]:
    content_hash = hashlib.sha256(body.encode("utf-8")).hexdigest()[:16]
    args_hash = hashlib.sha256(f"{to}|{subject}".encode("utf-8")).hexdigest()[:16]
    return (content_hash, args_hash)


def _get_executor_dedup_cache(executor: Optional[InfospaceExecutor]) -> Dict[Tuple[str, str], Tuple[float, Dict]]:
    if not executor:
        return {}
    cache = getattr(executor, "_tool_dedup_cache", None)
    if not isinstance(cache, dict):
        cache = {}
        executor._tool_dedup_cache = cache
    tool_cache = cache.get("send-email")
    if not isinstance(tool_cache, dict):
        tool_cache = {}
        cache["send-email"] = tool_cache
    return tool_cache


def _check_dedup(body: str, to: str, subject: str, executor: Optional[InfospaceExecutor] = None) -> Optional[Dict]:
    """Return cached result if an identical send happened within the dedup window."""
    key = _dedup_key(body, to, subject)
    for cache in (_get_executor_dedup_cache(executor), _dedup_cache):
        entry = cache.get(key)
        if entry:
            ts, cached_result = entry
            if time.monotonic() - ts < DEDUP_WINDOW_SECS:
                return cached_result
            del cache[key]
    return None


def _record_dedup(body: str, to: str, subject: str, result: Dict, executor: Optional[InfospaceExecutor] = None):
    key = _dedup_key(body, to, subject)
    _get_executor_dedup_cache(executor)[key] = (time.monotonic(), result)
    _dedup_cache[key] = (time.monotonic(), result)

# ---------------------------------------------------------------------------
# Uniform return helpers (same interface as check-email)
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
# Note helper (single confirmation Note, not a Collection)
# ---------------------------------------------------------------------------

def _create_note(text_content: str, agent_name: str, resource_manager,
                 source_skill: str = 'send-email',
                 tool_metadata: Optional[Dict] = None) -> Optional[str]:
    """Create a Note for the send confirmation."""
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
# Address parsing
# ---------------------------------------------------------------------------

def _parse_addresses(raw: str) -> List[str]:
    """Split a comma-separated address string into a clean list."""
    if not raw:
        return []
    return [addr.strip() for addr in raw.split(',') if addr.strip()]


# ---------------------------------------------------------------------------
# Main tool entry point
# ---------------------------------------------------------------------------

def tool(input_value, runtime=None, **kwargs):
    """
    Gmail SMTP email sending tool.

    Args:
        input_value: Email body text (from target/value, resolved by executor)
        **kwargs: to, subject, cc, bcc, reply_to,
                  agent_name, executor, resource_manager

    Returns:
        uniform_return with Note ID containing send confirmation
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
    gmail_address = os.getenv('GMAIL_ADDRESS', '').strip()
    gmail_password = os.getenv('GMAIL_APP_PASSWORD', '')
    if not gmail_address or not gmail_password:
        return _fail(executor,
                     'GMAIL_ADDRESS and GMAIL_APP_PASSWORD environment variables required')

    # Recipient — defaults to mailbox owner if omitted
    to_raw = kwargs.get('to', '')
    to_addrs = _parse_addresses(to_raw)
    if not to_addrs:
        to_addrs = [gmail_address]

    subject = kwargs.get('subject', '')
    if not subject:
        return _fail(executor, 'subject parameter required')

    # Optional parameters
    cc_addrs = _parse_addresses(kwargs.get('cc', ''))
    bcc_addrs = _parse_addresses(kwargs.get('bcc', ''))
    reply_to_id = kwargs.get('reply_to', '').strip()

    # Body from input_value
    body = ''
    if input_value:
        body = str(input_value)
    if not body:
        return _fail(executor, 'email body required (use target or value)')

    if len(body) > MAX_BODY_CHARS:
        body = body[:MAX_BODY_CHARS] + '\n\n[... truncated]'

    # Build MIME message
    msg = MIMEText(body, 'plain', 'utf-8')
    msg['From'] = gmail_address
    msg['To'] = ', '.join(to_addrs)
    msg['Subject'] = subject
    msg['Date'] = email.utils.formatdate(localtime=True)
    msg['Message-ID'] = email.utils.make_msgid(domain=gmail_address.split('@')[-1])

    if cc_addrs:
        msg['Cc'] = ', '.join(cc_addrs)

    if reply_to_id:
        msg['In-Reply-To'] = reply_to_id
        msg['References'] = reply_to_id

    # All recipients for SMTP envelope
    all_recipients = to_addrs + cc_addrs + bcc_addrs

    logger.info(f"send-email: to={to_addrs} subject={subject!r}")

    # Per-goal guard: only one send-email per planner session
    plan_id = getattr(getattr(executor, 'executive_node', None), 'current_plan_id', None)
    if plan_id:
        used = getattr(executor, '_side_effect_used_goals', None)
        if used is None:
            used = {}
            executor._side_effect_used_goals = used
        goal_key = f"send-email:{plan_id}"
        if goal_key in used:
            logger.warning(f"send-email: blocked — already sent in plan {plan_id}, returning cached result")
            return used[goal_key]

    # Dedup check — reject identical sends within 10-minute window
    to_display_dedup = ', '.join(to_addrs)
    cached = _check_dedup(body, to_display_dedup, subject, executor=executor)
    if cached:
        logger.warning(f"send-email: dedup hit — identical email to {to_display_dedup} subject={subject!r} "
                       f"sent within last {DEDUP_WINDOW_SECS}s, returning cached result")
        return cached

    # Send
    try:
        ctx = ssl.create_default_context()
        with smtplib.SMTP(SMTP_HOST, SMTP_PORT) as server:
            server.starttls(context=ctx)
            server.login(gmail_address, gmail_password)
            server.sendmail(gmail_address, all_recipients, msg.as_string())
            logger.info(f"send-email: sent to {to_addrs}, message-id={msg['Message-ID']}")
    except smtplib.SMTPAuthenticationError:
        logger.error(f"SMTP authentication failed for {gmail_address}")
        return _fail(executor, 'authentication_failed',
                     value='Gmail SMTP authentication failed. Check GMAIL_ADDRESS and GMAIL_APP_PASSWORD.')
    except Exception as e:
        logger.error(f"SMTP send error: {e}")
        return _fail(executor, 'send_failed',
                     value='Failed to send email via Gmail SMTP.')

    # Build confirmation
    timestamp = datetime.now(timezone.utc).isoformat()
    to_display = ', '.join(to_addrs)
    confirmation_text = f"Sent to {to_display}: {subject}"

    metadata = {
        'message_id': msg['Message-ID'],
        'to': to_display,
        'subject': subject,
        'timestamp': timestamp,
    }
    if cc_addrs:
        metadata['cc'] = ', '.join(cc_addrs)
    if bcc_addrs:
        metadata['bcc'] = ', '.join(bcc_addrs)
    if reply_to_id:
        metadata['in_reply_to'] = reply_to_id

    note_id = _create_note(confirmation_text, agent_name, resource_manager,
                           tool_metadata=metadata)
    if not note_id:
        # Email was sent but we couldn't create the confirmation Note
        result = _success(executor, confirmation_text, None, extra=metadata)
        _record_dedup(body, to_display, subject, result, executor=executor)
        if plan_id and hasattr(executor, '_side_effect_used_goals'):
            executor._side_effect_used_goals[f"send-email:{plan_id}"] = result
        return result

    logger.info(f"send-email: confirmation Note {note_id}")
    result = _success(executor, confirmation_text, note_id, extra=metadata)
    _record_dedup(body, to_display, subject, result, executor=executor)
    # Record for per-goal guard
    if plan_id and hasattr(executor, '_side_effect_used_goals'):
        executor._side_effect_used_goals[f"send-email:{plan_id}"] = result
    return result


# ---------------------------------------------------------------------------
# Standalone smoke test (no executor needed)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    gmail_address = os.getenv('GMAIL_ADDRESS', '').strip()
    gmail_password = os.getenv('GMAIL_APP_PASSWORD', '').strip()

    if not gmail_address or not gmail_password:
        print("Set GMAIL_ADDRESS and GMAIL_APP_PASSWORD environment variables")
        sys.exit(1)

    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <to> <subject> <body>")
        sys.exit(1)

    to_addr = sys.argv[1]
    subject = sys.argv[2]
    body = sys.argv[3]

    print(f"Sending from {gmail_address} to {to_addr}")
    print(f"Subject: {subject}")
    print(f"Body: {body[:200]}")
    print()

    msg = MIMEText(body, 'plain', 'utf-8')
    msg['From'] = gmail_address
    msg['To'] = to_addr
    msg['Subject'] = subject
    msg['Date'] = email.utils.formatdate(localtime=True)
    msg['Message-ID'] = email.utils.make_msgid(domain=gmail_address.split('@')[-1])

    try:
        ctx = ssl.create_default_context()
        with smtplib.SMTP(SMTP_HOST, SMTP_PORT) as server:
            server.starttls(context=ctx)
            server.login(gmail_address, gmail_password)
            server.sendmail(gmail_address, [to_addr], msg.as_string())
        print(f"Sent! Message-ID: {msg['Message-ID']}")
    except smtplib.SMTPAuthenticationError as e:
        print(f"Auth failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Send failed: {e}")
        sys.exit(1)
