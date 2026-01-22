---
name: search-web
type: python
description: "Search web using Google CSE. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count"
---

# search-web

Search web using Google Custom Search Engine. Returns Collection of structured Notes with filtered excerpts.

## Input

- `query`: Query string (e.g., "weather forecast Berkeley CA October 2025")

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

## Content Structure

Each Note in the returned Collection has the following JSON structure:
```json
{
  "text": "Filtered excerpt from webpage...",
  "format": "html",
  "metadata": {
    "uri": "https://example.com/page",
    "domain": "example.com",
    "source_url": "https://example.com/page",
    "elapsed_ms": 250
  },
  "char_count": 1500
}
```

**Important:** All result data is in the Note's `content` field (a dict). Engine metadata (creation date, source tool, etc.) is separate and accessed via `get_resource_metadata()`, not via `content['metadata']`.

## Field Access Examples

**Extract URLs for fetching full content:**
```json
{"type":"search-web","query":"transformers AI","out":"$results"}
{"type":"project","target":"$results","fields":["metadata.uri"],"out":"$urls"}
{"type":"pluck","target":"$urls","field":"metadata.uri","out":"$url_list"}
```

**Extract search result metadata:**
```json
{"type":"project","target":"$results","fields":["metadata.uri","metadata.domain","text"],"out":"$result_info"}
```

**Filter by domain:**
```json
{"type":"filter-structured","target":"$results","where":"metadata.domain == 'arxiv.org'","out":"$arxiv_results"}
```

## Planning Notes

- Use `metadata.uri` in `project` operations for consistent access
- For complete content from a specific URL, use `fetch-text` instead
- Results are filtered excerpts, not full page content

## Examples

```json
{"type":"search-web","query":"what are transformers in AI","out":"$results"}
{"type":"summarize","target":"$results","focus":"what are transformers","out":"$summary"}
```
