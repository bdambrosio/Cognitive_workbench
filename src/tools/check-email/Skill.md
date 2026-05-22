---
name: check-email
description: Read recent emails from a Gmail inbox via IMAP. Returns text with subject, sender, date, and body per email. Read-only — never marks messages as read.
args:
  folder: optional string (default "INBOX") — IMAP folder, e.g. "[Gmail]/Sent Mail", "[Gmail]/All Mail"
  limit: optional int (default 10, max 50) — max emails to return
  since: optional string — emails on or after this date, YYYY-MM-DD
  before: optional string — emails before this date, YYYY-MM-DD
  from_addr: optional string — filter by sender address or name
  subject: optional string — filter by subject line
  query: optional string — full-text search (IMAP TEXT command)
  unseen_only: optional string (default "false") — "true" to return only unread messages
---

# check-email

Read emails from Gmail using IMAP. Newest first. Connects via SSL to `imap.gmail.com:993`, opens folders in read-only mode (no side effects on the mailbox).

## Required environment

- `GMAIL_ADDRESS` — Gmail address (IMAP username)
- `GMAIL_APP_PASSWORD` — 16-character App Password from Google Account → Security → App Passwords

Without both, the tool returns an error and no data.

## Gmail folder names

| Folder | IMAP name |
|---|---|
| Inbox | `INBOX` |
| Sent | `[Gmail]/Sent Mail` |
| Drafts | `[Gmail]/Drafts` |
| All Mail | `[Gmail]/All Mail` |
| Starred | `[Gmail]/Starred` |
| Important | `[Gmail]/Important` |

## Examples

```json
{"thought": "check the last 5 inbox emails", "tool": "check-email", "limit": 5}
```

```json
{"thought": "find unread emails about the quarterly report", "tool": "check-email", "query": "quarterly report", "unseen_only": "true"}
```

```json
{"thought": "emails from boss this month", "tool": "check-email", "from_addr": "boss@company.com", "since": "2026-05-01"}
```

## Notes

- Email body is capped at 50,000 characters per message.
- Body prefers plain text; HTML is tag-stripped as fallback.
