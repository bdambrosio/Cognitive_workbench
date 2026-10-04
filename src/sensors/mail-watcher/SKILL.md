---
name: mail-watcher
description: Reports the sender and subject of mail that arrived since the last poll
type: code
schedule: "15m"
parameters:
  accounts:
    - address_env: GMAIL_ADDRESS
      password_env: GMAIL_APP_PASSWORD
---

Polls each account's inbox over IMAP, read-only, and reports new messages by
sender and subject. Each account is named by two environment variables: the
address and a Gmail app password. An account whose variables are not set is
skipped.
