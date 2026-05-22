---
name: send-email
type: python
description: "Send an email via Gmail SMTP. Body from target/value, headers as parameters. Returns confirmation Note with message-id in metadata."
schema_hint:
  to: "string (recipient address, default: GMAIL_ADDRESS)"
  subject: "string (subject line, required)"
  target: "$variable (email body from prior step, optional)"
  value: "string (literal email body, optional)"
  cc: "string (CC addresses, comma-separated, optional)"
  bcc: "string (BCC addresses, comma-separated, optional)"
  reply_to: "string (Message-ID to thread under, optional)"
  out: "$variable (optional)"
---

# send-email

Send an email via Gmail SMTP using App Password authentication. The email body comes from `target` (a Note or Collection from a prior step) or `value` (literal text). Returns a confirmation Note with the sent message-id in metadata.

## Input

Body content (one of):

- `target`: Variable or resource ID — content becomes the email body (plain text)
- `value`: Literal string — used as the email body directly

Header parameters:

| Param | Required | Purpose |
|-------|----------|---------|
| `to` | no | Recipient address(es), comma-separated. Default: `GMAIL_ADDRESS` (the mailbox owner) |
| `subject` | yes | Subject line |
| `cc` | no | CC address(es), comma-separated |
| `bcc` | no | BCC address(es), comma-separated |
| `reply_to` | no | Message-ID to thread as a reply (sets In-Reply-To and References headers) |

## Output

Success (`status: "success"`):
- `resource_id`: Note ID containing confirmation text (e.g., `"Sent to alice@example.com: Meeting notes"`).
- Metadata (accessible via `get-metadata`): `message_id`, `to`, `cc`, `bcc`, `subject`, `timestamp`.

Failure (`status: "failed"`):
- `reason`: Error description (e.g., `GMAIL_ADDRESS and GMAIL_APP_PASSWORD environment variables required`, `authentication_failed`)

## Requirements

- `GMAIL_ADDRESS` — Gmail address (also the SMTP username and From address)
- `GMAIL_APP_PASSWORD` — 16-character App Password from Google Account > Security > App Passwords

## Behavior

- Connects via STARTTLS to `smtp.gmail.com:587`
- Sends as plain text (UTF-8)
- If `to` is omitted, sends to `GMAIL_ADDRESS` (the mailbox owner)
- From address is always `GMAIL_ADDRESS` (cannot be spoofed)
- Authentication errors return a generic failure message

## Common Workflows

**Notify the operator (omit `to` — sends to mailbox owner):**
```json
{"type":"generate-note","prompt":"Summarize today's completed tasks and any blockers","out":"$report"}
{"type":"send-email","target":"$report","subject":"Daily Agent Report","out":"$sent"}
```

**Send a generated report to someone else:**
```json
{"type":"generate-note","prompt":"Write a weekly status update covering: shipped auth module, started caching layer, blocked on DB migration","out":"$report"}
{"type":"send-email","target":"$report","to":"team@company.com","subject":"Weekly Status Update","out":"$sent"}
```

**Reply to an email found via check-email:**
```json
{"type":"check-email","from_addr":"alice@company.com","subject":"Q1 planning","limit":1,"out":"$thread"}
{"type":"extract","target":"$thread","instruction":"Extract the Message-ID header","out":"$msg_id"}
{"type":"send-email","value":"Thanks Alice, I'll review the Q1 plan and get back to you by Friday.","to":"alice@company.com","subject":"Re: Q1 planning","reply_to":"$msg_id","out":"$sent"}
```

**Send synthesized research:**
```json
{"type":"search-web","query":"latest developments in AI safety 2026","out":"$research"}
{"type":"synthesize","target":"$research","focus":"key takeaways","format":"executive","out":"$summary"}
{"type":"send-email","target":"$summary","to":"boss@company.com","subject":"AI Safety Brief - Feb 2026","out":"$sent"}
```
