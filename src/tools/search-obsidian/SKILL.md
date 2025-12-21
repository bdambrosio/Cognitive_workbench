---
name: search-obsidian
type: python
description: "Search Obsidian notes using Obsidian Local REST API. Returns Collection of JSON Notes with fields text, metadata.uri (alias: source_url), metadata.domain, format, char_count). Use to find information in Obsidian vault."
schema_hint:
  value: "string (query - literal search text, not a variable)"
  out: "$variable"
examples:
  - '{"type":"search-obsidian","value":"transformer architecture","out":"$notes"}'
  - '{"type":"project","target":"$notes","fields":["metadata.uri","metadata.domain"],"out":"$urls"}'
  - '{"type":"filter-structured","target":"$notes","where":"char_count > 1000","out":"$long_notes"}'
---

# Obsidian Search Tool (Level 4)
## Input
- Query string (e.g., "machine learning notes")
## Output
- Collection ID containing one structured Note per search result
- Each Note contains JSON with uniform structure:

```json
{
  "text": "# Machine Learning\n\nThis note discusses...",
  "format": "markdown",
  "metadata": {
    "source_url": "obsidian://vault/notes/machine-learning.md",
    "uri": "obsidian://vault/notes/machine-learning.md",
    "domain": "obsidian",
    "elapsed_ms": 45
  },
  "char_count": 1123
}
```

## Configuration
Requires Obsidian Local REST API plugin to be installed and enabled in Obsidian.

- `OBSIDIAN_MCP_URL` environment variable (defaults to `http://127.0.0.1:27123` for HTTP, or `https://127.0.0.1:27124` for HTTPS)
- `OBSIDIAN_MCP_API_KEY` environment variable (required) - Get your API key from Obsidian Local REST API plugin settings

## Search Behavior
Searches by filename first, then fetches note content and filters by content match. Only searches markdown files (`.md`).

## Example Note Structure

Each Note in the returned Collection contains:
```json
{
  "text": "# Note Title\n\nNote content here...",
  "format": "markdown",
  "metadata": {
    "source_url": "obsidian://vault/path/to/note.md",
    "uri": "obsidian://vault/path/to/note.md",
    "domain": "obsidian",
    "elapsed_ms": 50
  },
  "char_count": 523
}
```

**Note**: The `uri` field is a standardized URI field (alias for `source_url`) for consistency with `search-web` and search primitives. Use `metadata.uri` in `project` operations for consistent access across all tools.

**Important**: The `value` parameter must be a literal string (e.g., `"grobid"`), not a variable reference (e.g., `"$grobid"`).

## Common Workflows

**Search and summarize:**
```json
{"type":"search-obsidian","value":"neural networks","out":"$notes"}
{"type":"summarize","target":"$notes","focus":"what are neural networks","out":"$summary"}
```

**Filter results:**
```json
{"type":"search-obsidian","value":"python scripts","out":"$notes"}
{"type":"filter-collection","target":"$notes","predicate":"contains code","out":"$code_notes"}
```

