#!/usr/bin/env python3
"""One way to send a notice: SMTP with the credentials in the environment.

    SMTP_USER, SMTP_PASS   the Gmail account and its app password
    MAIL_FROM              the address the notice comes from (info@tuuyi.com,
                           a "Send mail as" alias of that account)
    SMTP_HOST, SMTP_PORT   default smtp.gmail.com, 465 (SSL)
    PRACTICE_EMAILS        comma-separated; who the practice notices go to
    SITE_URL               the client site's public address, for links

    python3 src/client_ui/mail.py --test <address>     send one test notice

DRY RUN WHEN THERE ARE NO CREDENTIALS. Without SMTP_PASS, or with
MAIL_DRY_RUN=1, `send` logs the notice and appends it to `sent` instead of
connecting. Tests and local runs rely on that; nothing else in the site
checks whether mail is configured.
"""
from __future__ import annotations

import argparse
import logging
import os
import smtplib
import sys
from email.message import EmailMessage
from typing import Any, Dict, List, Optional

logger = logging.getLogger("client_ui.mail")

#: Every notice sent or dry-run in this process, oldest first.
sent: List[Dict[str, Any]] = []


def dry_run() -> bool:
    return bool(os.environ.get("MAIL_DRY_RUN")) or not os.environ.get("SMTP_PASS")


def practice_emails() -> List[str]:
    return [e.strip().lower() for e in os.environ.get("PRACTICE_EMAILS", "").split(",")
            if e.strip()]


def site_url() -> str:
    return os.environ.get("SITE_URL", "http://127.0.0.1:8803").rstrip("/")


def send(to: List[str], subject: str, body: str, link: Optional[str] = None) -> Dict[str, Any]:
    """Send one plain-text notice. `link` is appended on its own line. Returns
    the record kept in `sent`. Never raises on delivery failure: the site's
    stages do not depend on mail, so a failed notice is logged and the
    record says so."""
    to = [t for t in to if t]
    text = body.rstrip() + (f"\n\n{link}\n" if link else "\n")
    rec: Dict[str, Any] = {"to": to, "subject": subject, "body": text,
                           "dry_run": dry_run(), "error": None}
    if not to:
        rec["error"] = "no recipient"
        logger.warning("mail: no recipient for %r", subject)
    elif rec["dry_run"]:
        logger.info("mail (dry run) to %s: %s", ", ".join(to), subject)
    else:
        msg = EmailMessage()
        msg["From"] = os.environ.get("MAIL_FROM") or os.environ["SMTP_USER"]
        msg["To"] = ", ".join(to)
        msg["Subject"] = subject
        msg.set_content(text)
        host = os.environ.get("SMTP_HOST", "smtp.gmail.com")
        port = int(os.environ.get("SMTP_PORT", "465"))
        try:
            with smtplib.SMTP_SSL(host, port, timeout=30) as s:
                s.login(os.environ["SMTP_USER"], os.environ["SMTP_PASS"])
                s.send_message(msg)
            logger.info("mail to %s: %s", ", ".join(to), subject)
        except Exception as e:                                 # noqa: BLE001
            rec["error"] = f"{type(e).__name__}: {e}"
            logger.error("mail to %s failed: %s", ", ".join(to), rec["error"])
    sent.append(rec)
    return rec


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", metavar="ADDRESS", required=True,
                    help="send one test notice to this address")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    rec = send([args.test], "Tuuyi: test notice",
               "This is a test notice from the client site. If you are reading "
               "it in your inbox, notices work.", site_url())
    print("dry run — set SMTP_USER, SMTP_PASS and MAIL_FROM to send" if rec["dry_run"]
          else (f"failed: {rec['error']}" if rec["error"] else f"sent to {args.test}"))
    return 1 if rec["error"] else 0


if __name__ == "__main__":
    sys.exit(main())
