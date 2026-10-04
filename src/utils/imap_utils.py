"""Gmail over IMAP: the connection and header decoding shared by the
check-email tool and the mail-watcher sensor. Read-only use; callers select
a folder with `readonly=True`."""

import email.header
import imaplib
import logging
import ssl
from typing import List, Optional

logger = logging.getLogger(__name__)

IMAP_HOST = "imap.gmail.com"
IMAP_PORT = 993


class IMAPConnection:
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


def decode_header(raw: str) -> str:
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
