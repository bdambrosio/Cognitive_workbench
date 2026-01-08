---
name: search-web
type: python
description: "Search web using Google CSE. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count"
---

# search-web

Search web using Google Custom Search Engine. Returns Collection of structured Notes with filtered excerpts.

## Input

- `value`: Query string (e.g., "weather forecast Berkeley CA October 2025")

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID containing structured Notes, each with:
  - `text`: Filtered excerpt (query-relevant snippet)
  - `format`: "html"
  - `metadata.uri`: Full URL
  - `metadata.domain`: Domain name
  - `char_count`: Character count

## Behavior

- Returns filtered excerpts from multiple URLs (not full content)
- Requires `GOOGLE_API_KEY` and `GOOGLE_CX` environment variables

## Planning Notes

- Use `metadata.uri` in `project` operations for consistent access
- For complete content from a specific URL, use `fetch-text` instead
- Results are filtered excerpts, not full page content

## Examples

```json
{"type":"search-web","value":"what are transformers in AI","out":"$results"}
{"type":"summarize","target":"$results","focus":"what are transformers","out":"$summary"}
```
