---
name: search-obsidian
type: python
description: "Search Obsidian notes using Obsidian Local REST API. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count"
---

# Obsidian Search Tool

Search Obsidian notes using Obsidian Local REST API. Returns Collection of structured Notes.

## Purpose

Find information in Obsidian vault. Searches by filename first, then fetches note content and filters by content match. Only searches markdown files (`.md`).

## Input

- `value`: Query string (literal search text, not a variable)

## Output

Returns Collection ID containing one structured Note per search result. Each Note contains JSON with uniform structure:
- `text`: Note content
- `format`: "markdown"
- `metadata.source_url`: Obsidian URI (e.g., "obsidian://vault/notes/machine-learning.md")
- `metadata.uri`: Alias for source_url (standardized URI field)
- `metadata.domain`: "obsidian"
- `char_count`: Character count
- `metadata.elapsed_ms`: Search time

## Behavior & Performance

- Searches by filename first, then content match
- Only searches markdown files
- Returns structured Notes matching search-web/semantic-scholar format for consistency

## Guidelines

- Requires Obsidian Local REST API plugin installed and enabled
- Requires `OBSIDIAN_MCP_URL` environment variable (defaults to `http://127.0.0.1:27123` for HTTP, or `https://127.0.0.1:27124` for HTTPS)
- Requires `OBSIDIAN_MCP_API_KEY` environment variable (get from Obsidian Local REST API plugin settings)
- The `value` parameter must be a literal string (e.g., `"grobid"`), not a variable reference (e.g., `"$grobid"`)
- Use `metadata.uri` in `project` operations for consistent access across all tools

## Usage Examples

Search and summarize:
```json
{"type":"search-obsidian","value":"neural networks","out":"$notes"}
{"type":"summarize","target":"$notes","focus":"what are neural networks","out":"$summary"}
```

Filter results:
```json
{"type":"search-obsidian","value":"python scripts","out":"$notes"}
{"type":"filter-collection","target":"$notes","predicate":"contains code","out":"$code_notes"}
```
