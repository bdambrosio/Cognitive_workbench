---
name: post-bluesky
type: python
description: "Post to Bluesky (AT Protocol). Body from target/value. Returns confirmation Note with post URI in metadata."
schema_hint:
  target: "$variable (post body from prior step, optional)"
  value: "string (literal post body, optional)"
  tags: "string or list (optional): comma-separated tags or list, e.g. 'AI,research' or ['AI','research']"
  out: "$variable (optional)"
---

# post-bluesky

Post a text message to Bluesky using the AT Protocol. The post body comes from `target` (a Note or Collection from a prior step) or `value` (literal text). Returns a confirmation Note with the post URI and CID in metadata.

## Input

Body content (one of):

- `target`: Variable or resource ID — content becomes the post body (plain text)
- `value`: Literal string — used as the post body directly

Optional:

- `tags`: Hashtags to append to the post. Either a comma-separated string (`"AI,research"`) or a list (`["AI", "research"]`). Tags are appended as `#AI #research` and linked as clickable Bluesky tags.

## Output

Success (`status: "success"`):
- `resource_id`: Note ID containing confirmation text (e.g., `"Posted to Bluesky: Hello world..."`).
- Metadata (accessible via `get-metadata`): `uri`, `cid`, `timestamp`.

Failure (`status: "failed"`):
- `reason`: Error description (e.g., `BLUESKY_ACCOUNT_HANDLE and BLUESKY_APP_PASSWORD environment variables required`, `authentication_failed`)

## Requirements

- `BLUESKY_ACCOUNT_HANDLE` — Bluesky handle (e.g., `alice.bsky.social`)
- `BLUESKY_APP_PASSWORD` — App Password from Bluesky Settings > App Passwords

## Behavior

- Authenticates via `com.atproto.server.createSession` to obtain an access token
- Creates a post via `com.atproto.repo.createRecord`
- Posts as plain text (UTF-8) with optional hashtag facets
- Post text is truncated at 294 characters (+ "..." if truncated) to stay within Bluesky's 300 grapheme limit
- Tags are appended after the body and linked as clickable hashtags via AT Protocol facets

## Common Workflows

**Post a simple status update with tags:**
```json
{"type":"post-bluesky","value":"Just deployed a new feature! Everything is running smoothly.","tags":"devops,shipping","out":"$posted"}
```

**Post a generated summary:**
```json
{"type":"generate-note","prompt":"Write a short tweet-length summary of today's progress","out":"$update"}
{"type":"post-bluesky","target":"$update","out":"$posted"}
```

**Post synthesized research:**
```json
{"type":"search-web","query":"latest developments in AI safety 2026","out":"$research"}
{"type":"synthesize","target":"$research","focus":"one key takeaway","format":"brief","out":"$summary"}
{"type":"post-bluesky","target":"$summary","out":"$posted"}
```
