---
name: semantic-scholar
type: python
description: "Search academic papers. Returns Collection of text Notes with paper text (full text via GROBID when available, otherwise abstract)."
---

# semantic-scholar

Search academic papers using Semantic Scholar API. Returns Collection of text Notes with full paper text when PDF available.

## Input

- `query`: Query string (e.g., "attention mechanisms in neural networks")
- `limit`: Optional result limit (int, default: 10)

## Output

Success (`status: "success"`):
- `resource_id`: Collection ID containing text Notes.
- Each Note content is body text: full paper text (via GROBID) or abstract.
- Paper metadata is stored in a separate metadata Note linked by a `meta` Relation.

## Behavior

- When GROBID configured and PDF available, `text` contains full paper content
- Otherwise `text` contains the abstract
- Requires `SEMANTIC_SCHOLAR_API_KEY` environment variable
- Requires `grobid_url` in YAML config for full text extraction

## Metadata Access

Paper metadata (title/authors/year/citations/venue/uri/doi/paper_id/pdf_url) is linked as metadata Notes via `meta` Relations.

## Key Principle

**Results already contain full paper text in the `text` field.** Use extract/synthesize directly on the Collection — do NOT project metadata.uri for fetching. The URI is a PDF link for reference only; the text content is already loaded.

## Common Workflows

**Direct synthesis (preferred):**
```json
{"type":"semantic-scholar","query":"BERT model","out":"$papers"}
{"type":"synthesize","target":"$papers","focus":"key contributions of BERT","out":"$summary"}
```

**Per-paper extraction then synthesis:**
```json
{"type":"semantic-scholar","query":"attention mechanisms","out":"$papers"}
{"type":"map","target":"$papers","operation":"extract","instruction":"Extract the main architectural innovation","out":"$innovations"}
{"type":"synthesize","target":"$innovations","focus":"comparison of approaches","out":"$report"}
```

**Metadata workflow:**
- Use relation primitives (`find-relations`, `related`) to fetch `meta` Notes for paper Notes.
- Then run structured operations (`project`, `filter-structured`, `sort`) on those metadata Notes.
