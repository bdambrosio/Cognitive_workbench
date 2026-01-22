---
name: search-obsidian
type: python
description: "Search Obsidian notes using Obsidian Local REST API. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count"
---

# search-obsidian

Search Obsidian vault using Local REST API. Returns Collection of structured Notes.

## Input

- `query`: Query string (literal search text, NOT a variable)

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID containing structured Notes, each with:
  - `text`: Note content
  - `format`: "markdown"
  - `metadata.uri`: Obsidian URI
  - `metadata.domain`: "obsidian"
  - `char_count`: Character count

## Behavior

- Searches by filename first, then content match
- Only searches markdown files (`.md`)
- Returns structured Notes matching search-web/semantic-scholar format

## Requirements

- Obsidian Local REST API plugin installed and enabled
- `OBSIDIAN_MCP_URL` environment variable (default: `http://127.0.0.1:27123`)
- `OBSIDIAN_MCP_API_KEY` environment variable

## Examples

```json
{"type":"search-obsidian","query":"neural networks","out":"$notes"}
{"type":"summarize","target":"$notes","focus":"what are neural networks","out":"$summary"}
```
