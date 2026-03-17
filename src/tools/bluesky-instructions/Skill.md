---
name: bluesky-instructions
type: instruction
description: "Returns API instructions (not data) for retrieving Bluesky metrics via AT Protocol. Read the returned instructions, then use fetch-text + extract to get actual data."
schema_hint:
  target: "$variable (Bluesky handle, e.g. 'name.bsky.social')"
  out: "$variable (optional)"
---

# bluesky-instructions

Returns instructions for retrieving engagement metrics for a Bluesky account using the public AT Protocol API. This tool does NOT return metrics data — it returns API endpoints and usage patterns. Use `fetch-text` to call the endpoints described below, then `extract` or `extract-struct` to parse the JSON responses.

## API Base URL

`https://public.api.bsky.app/xrpc/`

## Endpoints

### Profile Metrics

**URL:** `https://public.api.bsky.app/xrpc/app.bsky.actor.getProfile?actor={HANDLE}`

Returns: `followersCount`, `followsCount`, `postsCount`, `displayName`, `description`

### Post Engagement (Author Feed)

**URL:** `https://public.api.bsky.app/xrpc/app.bsky.feed.getAuthorFeed?actor={HANDLE}&limit={N}`

Returns per-post: `likeCount`, `repostCount`, `replyCount`, `quoteCount`, post text, timestamp.

`limit` controls how many posts to return (max 100).

## Usage

Use `fetch-text` to call these endpoints, then `extract` or `extract-struct` to parse the JSON response.

**Example: Get profile metrics**
```json
{"type":"fetch-text","value":"https://public.api.bsky.app/xrpc/app.bsky.actor.getProfile?actor={HANDLE}","out":"$profile_raw"}
{"type":"extract-struct","target":"$profile_raw","instruction":"Extract followersCount, followsCount, postsCount, displayName","out":"$profile_metrics"}
```

**Example: Get recent post engagement**
```json
{"type":"fetch-text","value":"https://public.api.bsky.app/xrpc/app.bsky.feed.getAuthorFeed?actor={HANDLE}&limit=10","out":"$feed_raw"}
{"type":"extract","target":"$feed_raw","instruction":"For each post, extract: text (first 80 chars), likeCount, repostCount, replyCount, quoteCount, and createdAt timestamp. Format as a table.","out":"$engagement"}
```

## Response Format

Profile response (key fields):
```json
{
  "did": "did:plc:...",
  "handle": HANDLE",
  "displayName": "name",
  "followersCount": 42,
  "followsCount": 100,
  "postsCount": 15
}
```

Feed response contains `feed[]` array, each entry has `post.record.text`, `post.likeCount`, `post.repostCount`, `post.replyCount`, `post.quoteCount`, `post.record.createdAt`.
