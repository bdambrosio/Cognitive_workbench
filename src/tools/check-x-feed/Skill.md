---
name: check-x-feed
description: Return posts that appeared since the last check from the members of the user's curated X (Twitter) List. Use for the recurring X-feed sweep. Returns raw posts (author, time, text, link) with no relevance filtering — judge what is worth surfacing yourself and frame it in your own voice.
args: {}
---

# check-x-feed

Returns new posts from the accounts on the user's curated X List. Takes no
arguments — the List to follow and the credentials come from the
environment. Tracks a per-account `since_id` so each call returns only
posts that appeared since the previous call.

## Behavior

- Fetches the List membership (cached, refreshed daily), then polls each
  member's timeline for posts newer than the last seen id. Replies and
  retweets are excluded.
- Returns `{status: "ok", text: <markdown list of posts>}` — one entry per
  post with author, timestamp, full text, and link. Posts are grouped by
  author, newest first within each.
- The first check for an account returns its few most recent posts as
  history (marked as such in the output).
- Returns `{status: "ok", text: "No new posts ..."}` when nothing appeared.
- Returns `{status: "error", text: ...}` on missing configuration or a
  rejected token. A rate-limit mid-sweep returns the partial result with a
  note; the remaining accounts are picked up on the next call.

## Typical use

Called on a rhythm by the X-feed concern. Read the posts, judge relevance
the way the user would, surface at most one or two genuinely interesting
items with their links, stay silent otherwise.

```json
{"thought": "check what's new on the X list", "tool": "check-x-feed"}
```

## Required environment

- `X_BEARER_TOKEN` — app-only bearer token from the X developer portal
  (pay-per-use billing must be enabled on the account).
- `X_LIST_ID` — the curated X List to follow: either its numeric id or
  the full list URL (`https://x.com/i/lists/<id>`); the tool accepts both.

## Cost model

X bills $0.005 per post read (pay-per-use, Feb 2026 pricing). Because each
account is polled with `since_id`, only genuinely new posts are billed —
check frequency does not affect cost, feed volume does. A quiet list costs
approximately nothing; the one-time bootstrap reads up to 5 posts per list
member. The List's own `/tweets` endpoint is deliberately not used: it has
no `since_id` and would re-bill the same posts every call.

## State

`~/.cache/cognitive/check-x-feed/state.json` — cached List membership plus
the per-account `since_id` map. Delete it to force a full re-bootstrap.
