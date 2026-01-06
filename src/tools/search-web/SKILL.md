---
name: search-web
type: python
description: "Search web using Google CSE. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count"
---

# Web Search Tool

Search web using Google Custom Search Engine. Returns Collection of structured Notes with filtered excerpts from multiple URLs.

## Purpose

Find information on the web. Returns query-relevant snippets from multiple URLs (filtered excerpts, not full content). Use `fetch-text` when you have a specific URL and want complete content.

## Input

- `value`: Query string (e.g., "weather forecast Berkeley CA October 2025")

## Output

Returns Collection ID containing one structured Note per search result. Each Note contains JSON with uniform structure:
- `text`: Filtered excerpt (query-relevant snippet)
- `format`: "html"
- `metadata.source_url`: Full URL
- `metadata.uri`: Alias for source_url (standardized URI field)
- `metadata.domain`: Domain name
- `char_count`: Character count
- `metadata.elapsed_ms`: Search time

## Behavior & Performance

- Returns filtered excerpts from multiple URLs (not full content)
- Format matches semantic-scholar and search-obsidian for consistency
- Requires `GOOGLE_API_KEY` and `GOOGLE_CX` environment variables

## Guidelines

- Use `metadata.uri` in `project` operations for consistent access across all tools
- For complete content from a specific URL, use `fetch-text` instead
- Results are filtered excerpts, not full page content

## Usage Examples

Search and summarize:
```json
{"type":"search-web","value":"what are transformers in AI","out":"$results"}
{"type":"summarize","target":"$results","focus":"what are transformers","out":"$summary"}
```

Filter results:
```json
{"type":"search-web","value":"transformer papers","out":"$results"}
{"type":"filter-collection","target":"$results","predicate":"contains arxiv.org","out":"$arxiv_only"}
```
