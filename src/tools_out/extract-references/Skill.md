---
name: extract-references
type: python
description: "Extract bibliography/references from PDF files using GROBID and return a Collection of Notes (one per reference)."
schema_hint: {"path": "string (PDF file path or Note ID)", "grobid_url": "string"}
resolve:
  path: resource_id
---

# extract-references

Extract bibliography/references from PDF files using GROBID. Returns a Collection of Notes, where each Note contains structured metadata for one reference (compatible with format-citation).

## Input

- `path`: PDF file path (absolute) or Note ID containing PDF URL/metadata (required)
- `grobid_url`: Optional GROBID server URL (from world_config)

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID containing Notes (one Note per reference)
- Each Note contains:
  - `data`: Structured reference metadata (title, authors, year, venue, doi, url)
  - `metadata`: Source PDF, reference index, raw citation text

## Behavior

- Uses GROBID to parse PDF and extract references from `<bibl>` elements
- Creates one Note per reference with structured metadata
- Returns empty Collection if no references found
- Reference Notes are compatible with `format-citation` tool
- **Note ID input:** When given a Note ID (e.g., from `semantic-scholar`), looks up `pdf_url` from the Note's `tool_metadata` automatically — no manual metadata extraction needed

## Planning Notes

**This is the right tool for bibliography/reference extraction from papers.** It uses GROBID structural parsing (deterministic, fast) rather than LLM extraction (slow, lossy). Prefer this over `extract` or `map(extract)` with citation-related instructions.

**Common workflow with `semantic-scholar`:**
1. `semantic-scholar` → `$paper` Collection
2. `get_items("$paper")[0]` → Note ID (contains `pdf_url` in `tool_metadata`)
3. `extract-references(path=note_id)` → `$refs` Collection of structured citation Notes
4. Each citation Note is JSON: `{"title": "...", "authors": [...], "year": 2020, "venue": "..."}`
5. Read with `get_text(note_id)` + `json.loads()` in Python

**Do NOT:**
- Pass a Collection ID or `$binding` string as `path` — pass the actual Note ID from `get_items()`
- Use `pluck(field="text")` on result Notes — content is JSON, not plain text
- Use `extract` or `map(extract)` for reference lists — this tool is faster, structured, and deterministic

## Examples

```json
{"type":"extract-references","path":"/path/to/paper.pdf","out":"$refs"}
{"type":"extract-references","path":"Note_1234","out":"$refs"}
{"type":"format-citation","target":"$refs","format":"bibtex","out":"$bibtex"}
```

**Full semantic-scholar pipeline:**
```json
{"type":"semantic-scholar","query":"attention is all you need","limit":1,"out":"$paper"}
```
```python
items = get_items("$paper")
r = tool("extract-references", path=items[0], out="$refs")
```
