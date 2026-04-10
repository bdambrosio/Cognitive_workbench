---
name: semantic-scholar
type: python
description: "Search academic papers. Returns Collection of text Notes with paper text (full text via GROBID when available, otherwise abstract)."
---

# semantic-scholar

Search Semantic Scholar API. Returns Collection of text Notes.

## Input

- `query`: Search string (required)
- `limit`: Max results (int, default: 10)

## What Lives Where

- **Note content** (`get_text(note_id)`): Paper text — full text via GROBID, or abstract only.
  This is where the **abstract** lives. NOT in metadata.
- **Metadata** (`tool("get-metadata", target=note_id)`): title, authors, year, citations,
  venue, uri, doi, paper_id, pdf_url. Returns a JSON Note — parse with `get_json()`.

Do NOT re-fetch the paper via metadata.uri — the text is already loaded.

## Common Workflows

**Content analysis** (extract/synthesize directly on the Collection):
```json
{"type":"semantic-scholar","query":"BERT model","out":"$papers"}
{"type":"synthesize","target":"$papers","focus":"key contributions","out":"$summary"}
```

**Bibliography extraction** (use extract-references, not extract):
```python
items = get_items("$papers")
r = tool("extract-references", path=items[0], out="$refs")
```

**Metadata access:**
```python
items = get_items("$papers")
r = tool("get-metadata", target=items[0], out="$meta")
meta = get_json("$meta")  # {"title": "...", "authors": [...], "citations": 172280, ...}
```
