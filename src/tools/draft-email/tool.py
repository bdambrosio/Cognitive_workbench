"""
draft-email — save a message in the user's Gmail Drafts folder (2026-10-22,
Bruce's request, agreed with Jill). It never sends: the user reviews and
sends from Gmail.

  GMAIL_ADDRESS      — Gmail address (the From line and the IMAP username)
  GMAIL_APP_PASSWORD — its app password

The Drafts folder is the one Gmail marks with the \\Drafts attribute, so a
Gmail account in another language works too.
"""

import imaplib
import logging
import os
import time
from email.message import EmailMessage
from email.utils import formatdate, make_msgid
from typing import Optional

from utils.imap_utils import IMAPConnection

_logger = logging.getLogger(__name__)


def _drafts_folder(conn: imaplib.IMAP4_SSL) -> Optional[str]:
    """The name of the folder with the \\Drafts attribute, quoted for IMAP."""
    status, rows = conn.list()
    if status != "OK":
        return None
    for row in rows or []:
        line = row.decode("utf-8", errors="replace") if isinstance(row, bytes) else str(row)
        flags, _, rest = line.partition(")")
        if "\\Drafts" in flags.lstrip("(").split():
            # rest is ` "/" "[Gmail]/Drafts"`: delimiter, then the name.
            name = rest.strip().split(" ", 1)[1].strip()
            return name if name.startswith('"') else f'"{name}"'
    return None


def react_invoke(args, *, character_name=None, backend=None, logger=None):
    """ReAct entry-point — see Skill.md for the args contract."""
    log = logger or _logger
    to = str(args.get("to") or "").strip()
    subject = str(args.get("subject") or "").strip()
    body = str(args.get("body") or "").strip()
    cc = str(args.get("cc") or "").strip()
    if not to or not subject or not body:
        return {"status": "error",
                "text": "draft-email: `to`, `subject` and `body` are all required"}
    address = os.getenv("GMAIL_ADDRESS", "").strip()
    password = os.getenv("GMAIL_APP_PASSWORD", "").strip()
    if not address or not password:
        return {"status": "error",
                "text": "draft-email: GMAIL_ADDRESS and GMAIL_APP_PASSWORD are not set"}

    msg = EmailMessage()
    msg["From"] = address
    msg["To"] = to
    if cc:
        msg["Cc"] = cc
    msg["Subject"] = subject
    msg["Date"] = formatdate(localtime=True)
    msg["Message-ID"] = make_msgid()
    msg.set_content(body + "\n")

    try:
        with IMAPConnection(address, password) as conn:
            folder = _drafts_folder(conn)
            if folder is None:
                return {"status": "error",
                        "text": "draft-email: no Drafts folder found in the Gmail account"}
            status, detail = conn.append(folder, "(\\Draft)",
                                         imaplib.Time2Internaldate(time.time()),
                                         msg.as_bytes())
    except Exception as e:                                     # noqa: BLE001
        log.warning(f"draft-email failed: {type(e).__name__}: {e}")
        return {"status": "error", "text": f"draft-email: {type(e).__name__}: {e}"}
    if status != "OK":
        log.warning(f"draft-email: IMAP APPEND returned {status} {detail}")
        return {"status": "error", "text": f"draft-email: Gmail refused the draft ({status})"}
    log.info(f"draft-email: saved a draft to {to}: {subject!r}")
    return {"status": "ok",
            "text": (f"saved in Gmail Drafts for the user to review and send — To: {to}"
                     + (f"; Cc: {cc}" if cc else "") + f"; Subject: {subject}")}
