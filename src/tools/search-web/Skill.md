---
name: search-web
type: python
description: "Search web using Google CSE. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count"
---

# search-web

Search web using Google Custom Search Engine. Returns Collection of structured Notes with substantial page content.

## Input

- `query`: Query string (e.g., "weather forecast Berkeley CA October 2025")

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID containing structured Notes, each with:
  - `text`: Keyword-filtered page content (up to ~8K chars per result — substantial extracted text, not brief snippets)
  - `format`: "html" (or "pdf" if GROBID-parsed)
  - `metadata.uri`: Full URL
  - `metadata.domain`: Domain name
  - `char_count`: Character count

## Behavior

- Fetches each URL, extracts text, scores paragraphs by keyword relevance, keeps high-signal content
- For PDFs (when GROBID configured): returns full parsed text, not filtered
- LLM relevance filter discards off-topic pages entirely
- Results contain substantial text content — use extract/synthesize directly on the Collection
- Requires `GOOGLE_API_KEY` and `GOOGLE_CX` environment variables

## Content Structure

Each Note in the returned Collection has the following JSON structure:
```json
{
  "text": "Substantial keyword-filtered page content...",
  "format": "html",
  "metadata": {
    "uri": "https://example.com/page",
    "domain": "example.com",
    "source_url": "https://example.com/page",
    "elapsed_ms": 250
  },
  "char_count": 5000
}
```

**Important:** All result data is in the Note's `content` field (a dict). Engine metadata (creation date, source tool, etc.) is separate and accessed via `get_resource_metadata()`, not via `content['metadata']`.

## Key Principle

**Results already contain substantial page content in the `text` field.** Use extract/synthesize directly on the Collection. Only use fetch-text if you specifically need the complete unfiltered page content from a single URL.

## Common Workflows

**Direct synthesis (preferred):**
```json
{"type":"search-web","query":"transformers in AI","out":"$results"}
{"type":"synthesize","target":"$results","focus":"what are transformers","out":"$summary"}
```

**Per-result extraction then synthesis:**
```json
{"type":"search-web","query":"climate change effects","out":"$results"}
{"type":"map","target":"$results","operation":"extract","instruction":"Extract key statistics and findings","out":"$findings"}
{"type":"synthesize","target":"$findings","focus":"summary of effects","out":"$report"}
```

**Filter by domain then analyze:**
```json
{"type":"filter-structured","target":"$results","where":"metadata.domain == 'arxiv.org'","out":"$arxiv_results"}
{"type":"synthesize","target":"$arxiv_results","focus":"recent research","out":"$summary"}
```

**Extract metadata:**
```json
{"type":"project","target":"$results","fields":["metadata.uri","metadata.domain","text"],"out":"$result_info"}
```

## Planning Notes

- Use `metadata.uri` in `project` operations for consistent access
- Results contain substantial content — typically enough for extract/synthesize without re-fetching
- For complete unfiltered page content from a specific URL, use `fetch-text`
