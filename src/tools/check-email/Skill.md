---
name: check-email
type: python
description: "Read Gmail via IMAP. Returns Collection of text Notes, one per email (body as content, headers in tool_metadata)."
---

# check-email

Read emails from Gmail using IMAP with App Password authentication. Returns a Collection of text Notes — one Note per email, newest first. Read-only: never marks messages as read or modifies the mailbox.

## Input

All parameters are optional:

| Param | Default | Purpose |
|-------|---------|---------|
| `folder` | `"INBOX"` | IMAP folder (`[Gmail]/Sent Mail`, `[Gmail]/Drafts`, `[Gmail]/All Mail`, etc.) |
| `limit` | `10` (max 50) | Maximum number of emails to return |
| `since` | — | Emails on or after this date (`YYYY-MM-DD`) |
| `before` | — | Emails before this date (`YYYY-MM-DD`) |
| `from_addr` | — | Filter by sender address or name |
| `subject` | — | Filter by subject line |
| `query` | — | Full-text search (IMAP TEXT command) |
| `unseen_only` | `"false"` | Only return unread emails (`"true"` to enable) |

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID containing text Notes.
- Each Note's content is the email body text (plain text preferred, HTML tag-stripped as fallback).
- Email headers (subject, from, to, date, message_id) are stored as metadata accessible via `get-metadata`.

Failure (`status: "failed"`):
- `reason`: Error description (e.g., `GMAIL_ADDRESS and GMAIL_APP_PASSWORD environment variables required`, `authentication_failed`)

## Requirements

- `GMAIL_ADDRESS` — Gmail address (also the IMAP username)
- `GMAIL_APP_PASSWORD` — 16-character App Password from Google Account > Security > App Passwords

## Gmail Folder Names

| Folder | IMAP name |
|--------|-----------|
| Inbox | `INBOX` |
| Sent | `[Gmail]/Sent Mail` |
| Drafts | `[Gmail]/Drafts` |
| All Mail | `[Gmail]/All Mail` |
| Spam | `[Gmail]/Spam` |
| Trash | `[Gmail]/Trash` |
| Starred | `[Gmail]/Starred` |
| Important | `[Gmail]/Important` |

## Behavior

- Connects via SSL to `imap.gmail.com:993` with verified certificates
- Opens folders in read-only mode — no side effects on the mailbox
- Returns newest emails first
- Email body capped at 50,000 characters per message
- Credentials are never logged; only the email address appears in log messages
- Authentication errors return a generic message without leaking IMAP error details

## Common Workflows

**Check recent inbox:**
```json
{"type":"check-email","limit":5,"out":"$inbox"}
{"type":"synthesize","target":"$inbox","focus":"summarize what needs attention","out":"$summary"}
```

**Search for emails from a specific sender:**
```json
{"type":"check-email","from_addr":"boss@company.com","since":"2026-02-01","out":"$boss_emails"}
{"type":"extract","target":"$boss_emails","instruction":"Extract action items and deadlines","out":"$tasks"}
```

**Find unread emails about a topic:**
```json
{"type":"check-email","query":"quarterly report","unseen_only":"true","out":"$unread"}
{"type":"summarize","target":"$unread","focus":"key points from unread quarterly report emails","out":"$brief"}
```

**Combine with web search:**
```json
{"type":"check-email","subject":"meeting notes","since":"2026-02-15","out":"$meetings"}
{"type":"search-web","query":"latest industry trends in AI governance","out":"$trends"}
{"type":"synthesize","target":["$meetings","$trends"],"focus":"prepare briefing combining internal notes and external context","out":"$briefing"}
```
