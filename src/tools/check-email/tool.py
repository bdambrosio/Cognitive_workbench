"""
Gmail IMAP read-only email tool.

Connects to Gmail via IMAP (App Password auth), retrieves emails matching
search criteria, and returns them as a Collection of Notes.

Env vars:
  GMAIL_ADDRESS      — Gmail address (also the IMAP username)
  GMAIL_APP_PASSWORD — 16-char App Password from Google Account settings

Zero pip dependencies — stdlib only (imaplib, email, ssl).
"""

import email
import email.header
import email.utils
import imaplib
import logging
import os
import re
import ssl
from datetime import datetime
from typing import Any, Dict, List, Optional

from infospace_executor import InfospaceExecutor

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
IMAP_HOST = "imap.gmail.com"
IMAP_PORT = 993
MAX_BODY_CHARS = 50_000
ABSOLUTE_MAX_LIMIT = 50

# ---------------------------------------------------------------------------
# Uniform return helpers (same interface as semantic-scholar, search-obsidian)
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
# Note / Collection helpers (same pattern as semantic-scholar/tool.py)
# ---------------------------------------------------------------------------

def _create_note(text_content: str, agent_name: str, resource_manager,
                 source_skill: str = 'check-email',
                 tool_metadata: Optional[Dict] = None) -> Optional[str]:
    """Create a Note. Content is text-only; metadata in tool_metadata."""
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


def _create_collection(note_ids: List[str], agent_name: str,
                       resource_manager,
                       source_skill: str = 'check-email') -> Optional[str]:
    """Create a Collection and return its ID."""
    if not resource_manager:
        logger.error("resource_manager required for creating Collections")
        return None
    success, collection_id, error_msg, _ = resource_manager.create_collection(
        agent_name, note_ids, 'list',
        f'{len(note_ids)} emails', '', '', {},
    )
    if success:
        logger.info(f"Created Collection {collection_id} with {len(note_ids)} items")
        return collection_id
    logger.error(f"Failed to create Collection: {error_msg}")
    return None


# ---------------------------------------------------------------------------
# IMAP connection context manager
# ---------------------------------------------------------------------------

class _IMAPConnection:
    """Context manager: connects to Gmail IMAP, logs in, auto-disconnects."""

    def __init__(self, address: str, app_password: str):
        self._address = address
        self._app_password = app_password
        self._conn: Optional[imaplib.IMAP4_SSL] = None

    def __enter__(self) -> imaplib.IMAP4_SSL:
        ctx = ssl.create_default_context()
        self._conn = imaplib.IMAP4_SSL(IMAP_HOST, IMAP_PORT, ssl_context=ctx)
        self._conn.login(self._address, self._app_password)
        logger.info(f"IMAP login successful for {self._address}")
        return self._conn

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass
            try:
                self._conn.logout()
            except Exception:
                pass
        return False


# ---------------------------------------------------------------------------
# Search criteria builder
# ---------------------------------------------------------------------------

def _build_search_criteria(*, from_addr: str = '', subject: str = '',
                           since: str = '', before: str = '',
                           query: str = '', unseen_only: bool = False) -> str:
    """Translate tool params into an IMAP SEARCH string."""
    parts: List[str] = []

    if from_addr:
        parts.append(f'FROM "{from_addr}"')
    if subject:
        parts.append(f'SUBJECT "{subject}"')
    if since:
        # IMAP wants DD-Mon-YYYY
        try:
            dt = datetime.strptime(since, "%Y-%m-%d")
            parts.append(f'SINCE {dt.strftime("%d-%b-%Y")}')
        except ValueError:
            logger.warning(f"Invalid 'since' date format: {since}, expected YYYY-MM-DD")
    if before:
        try:
            dt = datetime.strptime(before, "%Y-%m-%d")
            parts.append(f'BEFORE {dt.strftime("%d-%b-%Y")}')
        except ValueError:
            logger.warning(f"Invalid 'before' date format: {before}, expected YYYY-MM-DD")
    if query:
        parts.append(f'TEXT "{query}"')
    if unseen_only:
        parts.append('UNSEEN')

    if not parts:
        return 'ALL'
    return ' '.join(parts)


# ---------------------------------------------------------------------------
# Fetch emails
# ---------------------------------------------------------------------------

def _fetch_emails(conn: imaplib.IMAP4_SSL, folder: str,
                  search_criteria: str, limit: int) -> List[bytes]:
    """Select folder (readonly), search, return raw MIME bytes for newest N."""
    status, _ = conn.select(folder, readonly=True)
    if status != 'OK':
        logger.error(f"Failed to select folder '{folder}'")
        return []

    status, data = conn.search(None, search_criteria)
    if status != 'OK' or not data or not data[0]:
        logger.info(f"No messages matched criteria in '{folder}'")
        return []

    msg_ids = data[0].split()
    # Newest first — take the last `limit` ids, reversed
    msg_ids = list(reversed(msg_ids[-limit:]))

    raw_messages: List[bytes] = []
    for mid in msg_ids:
        status, msg_data = conn.fetch(mid, '(RFC822)')
        if status == 'OK' and msg_data and msg_data[0] and isinstance(msg_data[0], tuple):
            raw_messages.append(msg_data[0][1])

    logger.info(f"Fetched {len(raw_messages)} emails from '{folder}'")
    return raw_messages


# ---------------------------------------------------------------------------
# Parse a single email
# ---------------------------------------------------------------------------

def _decode_header(raw: str) -> str:
    """Decode RFC 2047 encoded header value."""
    if not raw:
        return ''
    parts = email.header.decode_header(raw)
    decoded: List[str] = []
    for fragment, charset in parts:
        if isinstance(fragment, bytes):
            decoded.append(fragment.decode(charset or 'utf-8', errors='replace'))
        else:
            decoded.append(fragment)
    return ' '.join(decoded)


def _strip_html(html: str) -> str:
    """Rough tag strip for HTML fallback — not a full parser."""
    text = re.sub(r'<style[^>]*>.*?</style>', '', html, flags=re.DOTALL | re.IGNORECASE)
    text = re.sub(r'<script[^>]*>.*?</script>', '', text, flags=re.DOTALL | re.IGNORECASE)
    text = re.sub(r'<br\s*/?>', '\n', text, flags=re.IGNORECASE)
    text = re.sub(r'</?p[^>]*>', '\n', text, flags=re.IGNORECASE)
    text = re.sub(r'<[^>]+>', '', text)
    text = re.sub(r'&nbsp;', ' ', text)
    text = re.sub(r'&amp;', '&', text)
    text = re.sub(r'&lt;', '<', text)
    text = re.sub(r'&gt;', '>', text)
    text = re.sub(r'&#\d+;', '', text)
    # Collapse blank lines
    text = re.sub(r'\n{3,}', '\n\n', text)
    return text.strip()


def _parse_email(raw_bytes: bytes) -> Dict[str, Any]:
    """Extract subject/from/to/date/message_id/body_text from raw MIME."""
    msg = email.message_from_bytes(raw_bytes)

    subject = _decode_header(msg.get('Subject', ''))
    from_addr = _decode_header(msg.get('From', ''))
    to_addr = _decode_header(msg.get('To', ''))
    date_str = msg.get('Date', '')
    message_id = msg.get('Message-ID', '')

    # Parse date into ISO format if possible
    date_iso = ''
    if date_str:
        parsed = email.utils.parsedate_to_datetime(date_str)
        if parsed:
            date_iso = parsed.isoformat()

    # Extract body: prefer text/plain, fall back to tag-stripped HTML
    body_text = ''
    body_html = ''

    if msg.is_multipart():
        for part in msg.walk():
            content_type = part.get_content_type()
            disposition = str(part.get('Content-Disposition', ''))
            if 'attachment' in disposition:
                continue
            if content_type == 'text/plain' and not body_text:
                payload = part.get_payload(decode=True)
                if payload:
                    charset = part.get_content_charset() or 'utf-8'
                    body_text = payload.decode(charset, errors='replace')
            elif content_type == 'text/html' and not body_html:
                payload = part.get_payload(decode=True)
                if payload:
                    charset = part.get_content_charset() or 'utf-8'
                    body_html = payload.decode(charset, errors='replace')
    else:
        content_type = msg.get_content_type()
        payload = msg.get_payload(decode=True)
        if payload:
            charset = msg.get_content_charset() or 'utf-8'
            decoded = payload.decode(charset, errors='replace')
            if content_type == 'text/plain':
                body_text = decoded
            elif content_type == 'text/html':
                body_html = decoded

    # Use plain text if available, otherwise strip HTML
    if not body_text and body_html:
        body_text = _strip_html(body_html)

    # Cap body length
    if len(body_text) > MAX_BODY_CHARS:
        body_text = body_text[:MAX_BODY_CHARS] + '\n\n[... truncated]'

    return {
        'subject': subject,
        'from': from_addr,
        'to': to_addr,
        'date': date_iso or date_str,
        'message_id': message_id,
        'body': body_text,
    }


# ---------------------------------------------------------------------------
# Main tool entry point
# ---------------------------------------------------------------------------

def tool(input_value, runtime=None, **kwargs):
    """
    Gmail IMAP read-only email tool.

    Args:
        input_value: Unused (kept for interface compatibility)
        **kwargs: folder, limit, since, before, from_addr, subject, query,
                  unseen_only, agent_name, executor, resource_manager

    Returns:
        uniform_return with Collection ID containing one Note per email
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

    # Parameters
    folder = kwargs.get('folder', 'INBOX')
    limit = min(int(kwargs.get('limit', 10)), ABSOLUTE_MAX_LIMIT)
    since = kwargs.get('since', '')
    before = kwargs.get('before', '')
    from_addr = kwargs.get('from_addr', '')
    subject = kwargs.get('subject', '')
    query = kwargs.get('query', '')
    unseen_only = str(kwargs.get('unseen_only', 'false')).lower() in ('true', '1', 'yes')

    # Build search criteria
    search_criteria = _build_search_criteria(
        from_addr=from_addr, subject=subject,
        since=since, before=before,
        query=query, unseen_only=unseen_only,
    )
    logger.info(f"check-email: folder={folder} limit={limit} criteria={search_criteria}")

    # Connect, fetch, parse
    try:
        with _IMAPConnection(gmail_address, gmail_password) as conn:
            raw_messages = _fetch_emails(conn, folder, search_criteria, limit)
    except imaplib.IMAP4.error:
        logger.error(f"IMAP authentication failed for {gmail_address}")
        return _fail(executor, 'authentication_failed',
                     value='Gmail IMAP authentication failed. Check GMAIL_ADDRESS and GMAIL_APP_PASSWORD.')
    except Exception as e:
        logger.error(f"IMAP connection error: {e}")
        return _fail(executor, 'connection_failed',
                     value='Failed to connect to Gmail IMAP server.')

    if not raw_messages:
        empty_coll_id = _create_collection([], agent_name, resource_manager)
        if not empty_coll_id:
            return _fail(executor, 'Failed to create empty Collection')
        return _success(
            executor, '0 items []', empty_coll_id,
            {"item_count": 0, "folder": folder, "search_criteria": search_criteria},
        )

    # Parse each email and create Notes
    note_ids: List[str] = []
    for raw in raw_messages:
        parsed = _parse_email(raw)
        body = parsed.pop('body', '')
        # Headers go into tool_metadata; body is the Note content
        note_id = _create_note(body, agent_name, resource_manager,
                               tool_metadata=parsed)
        if note_id:
            note_ids.append(note_id)
        else:
            logger.warning(f"Failed to create Note for email: {parsed.get('subject', '?')}")

    if not note_ids:
        return _fail(executor, 'Failed to create any Notes from fetched emails')

    # Create Collection
    collection_id = _create_collection(note_ids, agent_name, resource_manager)
    if not collection_id:
        return _fail(executor, 'Failed to create Collection')

    # Display value
    item_count = len(note_ids)
    display_ids = note_ids[:5]
    note_list_str = ', '.join(display_ids)
    if item_count > 5:
        note_list_str += ', ...'
    collection_value = f"{item_count} items [{note_list_str}]"

    logger.info(f"check-email created Collection {collection_id} with {item_count} emails "
                f"from folder={folder}")
    return _success(
        executor, collection_value, collection_id,
        {"item_count": item_count, "folder": folder,
         "search_criteria": search_criteria, "note_ids": note_ids},
    )


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

    folder = sys.argv[1] if len(sys.argv) > 1 else 'INBOX'
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    print(f"Connecting to {gmail_address}, folder={folder}, limit={limit}\n")

    try:
        with _IMAPConnection(gmail_address, gmail_password) as conn:
            raw_messages = _fetch_emails(conn, folder, 'ALL', limit)
    except imaplib.IMAP4.error as e:
        print(f"Auth failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

    if not raw_messages:
        print("No messages found.")
        sys.exit(0)

    for i, raw in enumerate(raw_messages, 1):
        parsed = _parse_email(raw)
        print(f"{'='*72}")
        print(f"[{i}] Subject: {parsed['subject']}")
        print(f"    From:    {parsed['from']}")
        print(f"    Date:    {parsed['date']}")
        body_preview = parsed['body'][:300].replace('\n', '\n    ')
        print(f"    Body:    {body_preview}")
        if len(parsed['body']) > 300:
            print(f"    ... ({len(parsed['body'])} chars total)")
        print()

    print(f"Fetched {len(raw_messages)} emails.")
