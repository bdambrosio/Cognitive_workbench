---
name: draft-email
description: Put an email in the Drafts folder of the user's Gmail account, for the user to review and send. It never sends anything; nothing leaves until the user presses send. Use it when the user asks you to write or send an email.
args:
  to: required string — the recipient's address; several are separated by commas
  subject: required string — the subject line
  body: required string — the plain-text message (may be a `$stepN` binding)
  cc: optional string — addresses to copy, separated by commas
---

# draft-email

Saves one plain-text message in the Drafts folder of the user's Gmail
account (the account `check-email` reads by default). The user opens it in
Gmail, can change it, and sends it or deletes it. The tool cannot send, and
cannot change or delete a draft once saved; a second call makes a second
draft.

When the user asks you to send an email, draft it with this tool and tell
them it is waiting in Drafts.

The observation repeats the recipients and subject as saved.
