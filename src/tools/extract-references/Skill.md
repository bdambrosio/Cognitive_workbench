---
name: extract-references
type: python
description: "Extract bibliography/references from PDF files using GROBID and return a Collection of Notes (one per reference)."
schema_hint: {"path": "string (PDF file path or Note ID)", "grobid_url": "string"}
---

# extract-references

Extract bibliography/references from PDF files using GROBID. Returns a Collection of Notes, where each Note contains structured metadata for one reference (compatible with format-citation).

## Input

- `path`: PDF file path (absolute) or Note ID containing PDF URL/metadata (required)
- `grobid_url`: Optional GROBID server URL (from world_config)

## Output

Success returns:
- `resource_id`: Collection ID containing Notes (one Note per reference)
- Each Note contains:
  - `data`: Structured reference metadata (title, authors, year, venue, doi, url)
  - `metadata`: Source PDF, reference index, raw citation text

## Behavior

- Uses GROBID to parse PDF and extract references from `<bibl>` elements
- Creates one Note per reference with structured metadata
- Returns empty Collection if no references found
- Reference Notes are compatible with `format-citation` tool

## Examples

```json
{"type":"extract-references","path":"/path/to/paper.pdf","out":"$refs"}
{"type":"extract-references","path":"$paper_note","out":"$refs"}
{"type":"extract-references","path":"$refs","format-citation":"bibtex","out":"$bibtex"}
```
